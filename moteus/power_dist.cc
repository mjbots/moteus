// Copyright 2019 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mbed.h"

#include "moteus/fdcan.h"
#include "moteus/millisecond_timer.h"
#include "moteus/power_dist_hw.h"

namespace {

const uint32_t kPrechargeMs = 100;

enum State {
  kPowerOff,
  kPrecharging,
  kPowerOn,
};

void SetClock() {
  // RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  // Select HSI as system clock source and configure the HCLK, PCLK1
  // and PCLK2 clocks dividers.
  RCC_ClkInitStruct.ClockType = (
      RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
      RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 16 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  // 16 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  // 16 MHz
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    return;
  }

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      mbed_die();
    }
  }

  SystemCoreClockUpdate();
}

}

int main(void) {
  // Drop our speed down to nothing, because we don't really need to
  // go fast for this and we might as well save the battery.
  SetClock();

  moteus::FDCan::Filter filters[] = {
    { 0x10005, 0xffffff, moteus::FDCan::FilterMode::kMask,
      moteus::FDCan::FilterAction::kAccept,
      moteus::FDCan::FilterType::kExtended,
    },
  };

  moteus::FDCan can(
      [&]() {
        moteus::FDCan::Options options;
        options.td = PA_12;
        options.rd = PA_11;
        options.slow_bitrate = 125000;
        options.fast_bitrate = 125000;

        options.filter_begin = &filters[0];
        options.filter_end = &filters[0] + (sizeof(filters) / sizeof(filters[0]));

        return options;
      }());

  DigitalOut led1(DEBUG_LED1, 1);

  // On my prototype r2 board, this pin is shorted to ground.  Don't
  // drive it high!
  DigitalOut led2(DEBUG_LED2, 0);

  DigitalOut switch_led(PWR_LED);
  DigitalIn power_switch(PWR_SW, PullUp);

  DigitalOut fet_precharge(FET_PRECHARGE);
  DigitalOut fet_main(FET_MAIN);

  moteus::MillisecondTimer timer;

  State state = kPowerOff;
  uint32_t precharge_start = 0;

  // Set up an interrupt to trigger every 50ms just to kick us out of sleep mode.

  uint32_t last_can = 0;

  char can_status_data[8] = {
    0, // switch status
    0, // lock time in 0.1s
  };

  char can_command_data[8] = {};

  char& power_switch_status = can_status_data[0];
  uint8_t& lock_time = reinterpret_cast<uint8_t&>(can_status_data[1]);

  FDCAN_RxHeaderTypeDef can_rx_header;

  for (;;) {
    {
      const auto now = timer.read_ms() / 100;
      if (now != last_can) {
        last_can = now;

        can.Send(0x10004, std::string_view(can_status_data,
                                           sizeof(can_status_data)));

        if (lock_time > 0) { lock_time--; }
      }

      if (can.Poll(&can_rx_header, can_command_data)) {
        if (can_command_data[1] > 0) {
          lock_time = static_cast<uint8_t>(can_command_data[1]);
        }
      }
    }

    switch (state) {
      case kPowerOff: {
        switch_led.write(1);  // inverted
        fet_main.write(0);
        fet_precharge.write(0);
        break;
      }
      case kPrecharging: {
        switch_led.write((timer.read_ms() / 20) % 2);
        fet_main.write(0);
        fet_precharge.write(1);
        break;
      }
      case kPowerOn: {
        switch_led.write(0);
        fet_main.write(1);
        fet_precharge.write(1);
        break;
      }
    }

    power_switch_status = (power_switch.read() == 0) ? 1 : 0;

    // Handle state transitions.
    if (power_switch_status == 1) {
      // The switch is on.  Try to get into the power on state.
      switch (state) {
        case kPowerOff: {
          precharge_start = timer.read_ms();
          state = kPrecharging;
          break;
        }
        case kPrecharging: {
          const uint32_t elapsed_ms = timer.read_ms() - precharge_start;
          if (elapsed_ms > kPrechargeMs) {
            state = kPowerOn;
          }
          break;
        }
        case kPowerOn: {
          // Nothing to do here.
          break;
        }
      }
    } else {
      if (lock_time == 0) {
        // Only turn off once our lock time has expired.
        state = kPowerOff;
      }
    }
  }
}
