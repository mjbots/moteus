// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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
#include "mjlib/multiplex/micro_stream_datagram.h"

#include "fw/board_debug.h"
#include "fw/clock_manager.h"
#include "fw/firmware_info.h"
#include "fw/git_info.h"
#include "fw/millisecond_timer.h"
#include "fw/moteus_controller.h"
#include "fw/moteus_hw.h"
#include "fw/system_info.h"
#include "fw/uuid.h"

#if defined(TARGET_STM32G4)
#include "fw/fdcan.h"
#include "fw/fdcan_micro_server.h"
#include "fw/stm32g4_async_uart.h"
#include "fw/stm32g4_flash.h"
#else
#error "Unknown target"
#endif

extern "C" {
  uint32_t kMoteusFirmwareVersion = MOTEUS_FIRMWARE_VERSION;
}

auto* const MyDWT = DWT;
auto* const MyFLASH = FLASH;

using namespace moteus;
namespace micro = mjlib::micro;
namespace multiplex = mjlib::multiplex;

#if defined(TARGET_STM32G4)
using HardwareUart = Stm32G4AsyncUart;
using Stm32Flash = Stm32G4Flash;
#else
#error "Unknown target"
#endif

extern "C" {
void SetupClock() {
#if defined(TARGET_STM32G4)
  {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 170 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  //  85 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  //  85 MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
      mbed_die();
    }

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_FDCAN |
        RCC_PERIPHCLK_USART2 |
        RCC_PERIPHCLK_USART3 |
        RCC_PERIPHCLK_ADC12 |
        RCC_PERIPHCLK_ADC345 |
        RCC_PERIPHCLK_I2C1
        ;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      mbed_die();
    }

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
#endif
}
}

#if defined(TARGET_STM32G4)
extern "C" {
extern char _sccmram;
extern char _siccmram;
extern char _eccmram;
extern char _sccmram;
}
#endif

namespace moteus {
volatile uint8_t g_measured_hw_family;
volatile uint8_t g_measured_hw_rev;
volatile uint8_t g_measured_hw_pins;
MoteusHwPins g_hw_pins;
}

namespace {
struct CanConfig {
  uint32_t prefix = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(prefix));
  }

  bool operator==(const CanConfig& rhs) const {
    return prefix == rhs.prefix;
  }
};
}

int main(void) {
#if defined(TARGET_STM32G4)
  std::memcpy(&_sccmram, &_siccmram, &_eccmram - &_sccmram);
#endif

  FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

#if 0
  // We map SRAM1 onto address 0 just to make finding accesses to
  // address 0 easier.
  SYSCFG->MEMRMP = (SYSCFG->MEMRMP & ~SYSCFG_MEMRMP_MEM_MODE_Msk) | 0x03;  // SRAM1
#endif

  SetupClock();

  // I initially used a Ticker here to enqueue events at 1ms
  // intervals.  However, it introduced jitter into the current
  // sampling interrupt, and I couldn't figure out how to get the
  // interrupt priorities right.  Thus for now we just poll to look
  // for millisecond turnover.
  MillisecondTimer timer;

  const auto family_and_version = DetectMoteusFamily(&timer);
  g_measured_hw_family = family_and_version.family;
  g_measured_hw_rev = family_and_version.hw_version;
  g_measured_hw_pins = family_and_version.hw_pins;

  g_hw_pins = FindHardwarePins(family_and_version);

  // We do our compatibility check *after* calling FindHardwarePins so
  // the the family specific pins are known.  That way mbed_die
  // doesn't choke being unable to find the power LED for instance.

  if (family_and_version.hw_version < 0) {
    // This firmware is not compatible with this board.
    mbed_die();
  }

  // We require cycle counting be enabled for some things.
  {
    ITM->LAR = 0xC5ACCE55;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  // Turn on our power light.
  DigitalOut power_led(g_hw_pins.power_led, 0);

  micro::SizedPool<20000> pool;

  std::optional<HardwareUart> rs485;
  if (g_hw_pins.uart_tx != NC) {
    rs485.emplace(&pool, &timer, []() {
      HardwareUart::Options options;
      options.tx = g_hw_pins.uart_tx;
      options.rx = g_hw_pins.uart_rx;
      options.dir = g_hw_pins.uart_dir;
      options.baud_rate = 3000000;
      return options;
                                 }());
  }

  FDCan fdcan([]() {
      FDCan::Options options;

      options.td = g_hw_pins.can_td;
      options.rd = g_hw_pins.can_rd;

      options.slow_bitrate = 1000000;
      options.fast_bitrate = 5000000;

      options.fdcan_frame = true;
      options.bitrate_switch = true;
      options.automatic_retransmission = true;

      // Family 0 uses a TCAN334GDCNT, which has a very low loop
      // delay.  Other families use chips with a longer loop delay.
      options.delay_compensation = g_measured_hw_family != 0;

      options.tdc_offset = 13;  // 13 / 85MHz ~= 152ns
      options.tdc_filter = 2; // 2 / 85MHz ~= 23ns

      return options;
    }());
  FDCanMicroServer fdcan_micro_server(&fdcan);
  multiplex::MicroServer multiplex_protocol(
      &pool, &fdcan_micro_server,
      []() {
        multiplex::MicroServer::Options options;
        options.max_tunnel_streams = 3;
        return options;
      }());

  micro::AsyncStream* serial = multiplex_protocol.MakeTunnel(1);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(serial);
  micro::CommandManager command_manager(&pool, serial, &write_stream);
  char micro_output_buffer[2048] = {};
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream, micro_output_buffer);
  Stm32Flash flash_interface;
  micro::PersistentConfig persistent_config(
      pool, command_manager, flash_interface, micro_output_buffer);

  SystemInfo system_info(pool, telemetry_manager);
  FirmwareInfo firmware_info(pool, telemetry_manager,
                             kMoteusFirmwareVersion, MOTEUS_MODEL_NUMBER);
  Uuid uuid(persistent_config);
  ClockManager clock(&timer, persistent_config, command_manager);

  MoteusController moteus_controller(
      &pool, &persistent_config,
      &command_manager,
      &telemetry_manager,
      &multiplex_protocol,
      &clock,
      &system_info,
      &timer,
      &firmware_info,
      &uuid);

  BoardDebug board_debug(
      &pool, &command_manager, &telemetry_manager, &multiplex_protocol,
      moteus_controller.bldc_servo());

  persistent_config.Register("id", multiplex_protocol.config(), [](){});

  GitInfo git_info;
  telemetry_manager.Register("git", &git_info);

  CanConfig can_config, old_can_config;

  persistent_config.Register(
      "can", &can_config,
      [&can_config, &fdcan, &fdcan_micro_server, &old_can_config]() {
        // We only update our config if it has actually changed.
        // Re-initializing the CAN-FD controller can cause packets to
        // be lost, so don't do it unless actually necessary.
        if (can_config == old_can_config) {
          return;
        }
        old_can_config = can_config;

        FDCan::Filter filters[1] = {};
        filters[0].id1 = can_config.prefix << 16;
        filters[0].id2 = 0x1fff0000u;
        filters[0].mode = FDCan::FilterMode::kMask;
        filters[0].action = FDCan::FilterAction::kAccept;
        filters[0].type = FDCan::FilterType::kExtended;
        FDCan::FilterConfig filter_config;
        filter_config.begin = std::begin(filters);
        filter_config.end = std::end(filters);
        filter_config.global_std_action = FDCan::FilterAction::kAccept;
        filter_config.global_ext_action = FDCan::FilterAction::kReject;
        fdcan.ConfigureFilters(filter_config);

        fdcan_micro_server.SetPrefix(can_config.prefix);
      });

  persistent_config.Load();

  moteus_controller.Start();
  command_manager.AsyncStart();
  multiplex_protocol.Start(moteus_controller.multiplex_server());

  auto old_time = timer.read_us();

  for (;;) {
    if (rs485) {
      rs485->Poll();
    }
#if defined(TARGET_STM32G4)
    fdcan_micro_server.Poll();
#endif
    moteus_controller.Poll();
    multiplex_protocol.Poll();

    const auto new_time = timer.read_us();

    const auto delta_us = MillisecondTimer::subtract_us(new_time, old_time);
    if (moteus_controller.bldc_servo()->config().timing_fault &&
        delta_us >= 4000) {
      // We missed several entire polling cycles.  Fault if we can.
      moteus_controller.bldc_servo()->Fault(moteus::errc::kTimingViolation);
    }

    if (delta_us >= 1000) {
      telemetry_manager.PollMillisecond();
      system_info.PollMillisecond();
      moteus_controller.PollMillisecond();
      board_debug.PollMillisecond();
      system_info.SetCanResetCount(fdcan_micro_server.can_reset_count());

      old_time += 1000;
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
