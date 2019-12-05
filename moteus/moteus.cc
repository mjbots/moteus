// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

#include <inttypes.h>

#include <functional>

#include "mbed.h"

#include "mjlib/micro/async_exclusive.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"
#include "mjlib/multiplex/micro_server.h"

#include "moteus/board_debug.h"
#include "moteus/firmware_info.h"
#include "moteus/millisecond_timer.h"
#include "moteus/moteus_controller.h"
#include "moteus/moteus_hw.h"
#include "moteus/stm32_flash.h"
#include "moteus/system_info.h"

#if defined(TARGET_STM32F4)
#include "moteus/stm32f446_async_uart.h"
#elif defined(TARGET_STM32G4)
#include "moteus/stm32g4_async_uart.h"
#else
#error "Unknown target"
#endif

auto* const MyDWT = DWT;
auto* const MyFLASH = FLASH;

using namespace moteus;
namespace micro = mjlib::micro;
namespace multiplex = mjlib::multiplex;

#if defined(TARGET_STM32F4)
using HardwareUart = Stm32F446AsyncUart;
#elif defined(TARGET_STM32G4)
using HardwareUart = Stm32G4AsyncUart;
#else
#error "Unknown target"
#endif

extern "C" {
void SetupClock() {
#if defined(TARGET_STM32G4)
  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_FDCAN |
        RCC_PERIPHCLK_USART2 |
        RCC_PERIPHCLK_USART3 |
        RCC_PERIPHCLK_ADC12 |
        RCC_PERIPHCLK_ADC345
        ;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      mbed_die();
    }
  }
#endif
}
}


int main(void) {
  SetupClock();

  DigitalIn hwrev0(HWREV_PIN0, PullUp);
  DigitalIn hwrev1(HWREV_PIN1, PullUp);
  DigitalIn hwrev2(HWREV_PIN2, PullUp);

  // To enable cycle counting.
  if (0) {
    ITM->LAR = 0xC5ACCE55;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  const uint8_t this_hw_rev =
      0x07 & (~(hwrev0.read() |
                (hwrev1.read() << 1) |
                (hwrev2.read() << 2)));
  MJ_ASSERT(this_hw_rev == (MOTEUS_HW_REV - MOTEUS_HW_REV_OFFSET));

  // I initially used a Ticker here to enqueue events at 1ms
  // intervals.  However, it introduced jitter into the current
  // sampling interrupt, and I couldn't figure out how to get the
  // interrupt priorities right.  Thus for now we just poll to look
  // for millisecond turnover.
  MillisecondTimer timer;

  micro::SizedPool<12288> pool;

  HardwareUart rs485(&pool, &timer, []() {
      HardwareUart::Options options;
      options.tx = MOTEUS_UART_TX;
      options.rx = MOTEUS_UART_RX;
      options.dir = MOTEUS_UART_DIR;
      options.baud_rate = 115200;
      return options;
    }());

  multiplex::MicroServer multiplex_protocol(&pool, &rs485, {});

  micro::AsyncStream* serial = multiplex_protocol.MakeTunnel(1);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(serial);
  micro::CommandManager command_manager(&pool, serial, &write_stream);
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream);
  Stm32Flash flash_interface;
  micro::PersistentConfig persistent_config(pool, command_manager, flash_interface);

  SystemInfo system_info(pool, telemetry_manager);
  FirmwareInfo firmware_info(pool, telemetry_manager, MOTEUS_FIRMWARE_VERSION);

  MoteusController moteus_controller(
      &pool, &persistent_config, &telemetry_manager, &timer, &firmware_info);

  BoardDebug board_debug(
      &pool, &command_manager, &telemetry_manager, &multiplex_protocol,
      moteus_controller.bldc_servo());

  persistent_config.Register("id", multiplex_protocol.config(), [](){});

  persistent_config.Load();

  moteus_controller.Start();
  command_manager.AsyncStart();
  multiplex_protocol.Start(moteus_controller.multiplex_server());

  auto old_time = timer.read_ms();

  for (;;) {
    rs485.Poll();
    moteus_controller.Poll();

    const auto new_time = timer.read_ms();

    if (new_time != old_time) {
      telemetry_manager.PollMillisecond();
      system_info.PollMillisecond();
      moteus_controller.PollMillisecond();
      board_debug.PollMillisecond();

      old_time = new_time;
    }

    SystemInfo::idle_count++;
  }

  return 0;
}

extern "C" {
  void abort() {
    mbed_die();
  }
}
