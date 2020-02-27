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
#include "mjlib/multiplex/micro_stream_datagram.h"

#include "moteus/board_debug.h"
#include "moteus/firmware_info.h"
#include "moteus/git_info.h"
#include "moteus/millisecond_timer.h"
#include "moteus/moteus_controller.h"
#include "moteus/moteus_hw.h"
#include "moteus/system_info.h"

#if defined(TARGET_STM32F4)
#include "moteus/stm32f446_async_uart.h"
#include "moteus/stm32f4_flash.h"
#elif defined(TARGET_STM32G4)
#include "moteus/fdcan.h"
#include "moteus/fdcan_micro_server.h"
#include "moteus/stm32g4_async_uart.h"
#include "moteus/stm32g4_flash.h"
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
using Stm32Flash = Stm32F4Flash;
#elif defined(TARGET_STM32G4)
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

#if defined(TARGET_STM32G4)
extern "C" {
extern char _sccmram;
extern char _siccmram;
extern char _eccmram;
extern char _sccmram;
}
#endif

volatile uint8_t g_measured_hw_rev;

int main(void) {
#if defined(TARGET_STM32G4)
  std::memcpy(&_sccmram, &_siccmram, &_eccmram - &_sccmram);
#endif

  FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

  SetupClock();

  // Turn on our power light.
  DigitalOut power_led(POWER_LED, 0);

  DigitalIn hwrev0(HWREV_PIN0, PullUp);
  DigitalIn hwrev1(HWREV_PIN1, PullUp);
  DigitalIn hwrev2(HWREV_PIN2, PullUp);

  // To enable cycle counting.
#ifdef MOTEUS_PERFORMANCE_MEASURE
  {
    ITM->LAR = 0xC5ACCE55;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
#endif

  const uint8_t this_hw_rev =
      0x07 & (~(hwrev0.read() |
                (hwrev1.read() << 1) |
                (hwrev2.read() << 2)));
  g_measured_hw_rev = this_hw_rev;
  MJ_ASSERT(this_hw_rev == moteus::kHardwareInterlock[MOTEUS_HW_REV]);

  // I initially used a Ticker here to enqueue events at 1ms
  // intervals.  However, it introduced jitter into the current
  // sampling interrupt, and I couldn't figure out how to get the
  // interrupt priorities right.  Thus for now we just poll to look
  // for millisecond turnover.
  MillisecondTimer timer;

  micro::SizedPool<14000> pool;

  HardwareUart rs485(&pool, &timer, []() {
      HardwareUart::Options options;
      options.tx = MOTEUS_UART_TX;
      options.rx = MOTEUS_UART_RX;
      options.dir = MOTEUS_UART_DIR;
      options.baud_rate = 3000000;
      return options;
    }());

#if defined(TARGET_STM32G4)
  FDCan fdcan([]() {
      FDCan::Options options;

      options.td = MOTEUS_CAN_TD;
      options.rd = MOTEUS_CAN_RD;

      options.slow_bitrate = 1000000;
      options.fast_bitrate = 5000000;

      options.fdcan_frame = true;
      options.bitrate_switch = true;
      options.automatic_retransmission = true;

      return options;
    }());
  FDCanMicroServer fdcan_micro_server(&fdcan);
  multiplex::MicroServer multiplex_protocol(&pool, &fdcan_micro_server, {});
#elif defined(TARGET_STM32F4)
  multiplex::MicroStreamDatagram stream_datagram(&pool, &rs485, {});
  multiplex::MicroServer multiplex_protocol(&pool, &stream_datagram, {});
#else
#error "Unknown target"
#endif

  micro::AsyncStream* serial = multiplex_protocol.MakeTunnel(1);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(serial);
  micro::CommandManager command_manager(&pool, serial, &write_stream);
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream);
  Stm32Flash flash_interface;
  micro::PersistentConfig persistent_config(pool, command_manager, flash_interface);

  SystemInfo system_info(pool, telemetry_manager);
  FirmwareInfo firmware_info(pool, telemetry_manager,
                             MOTEUS_FIRMWARE_VERSION, MOTEUS_MODEL_NUMBER);

  MoteusController moteus_controller(
      &pool, &persistent_config, &telemetry_manager, &timer, &firmware_info);

  BoardDebug board_debug(
      &pool, &command_manager, &telemetry_manager, &multiplex_protocol,
      moteus_controller.bldc_servo());

  persistent_config.Register("id", multiplex_protocol.config(), [](){});

  GitInfo git_info;
  telemetry_manager.Register("git", &git_info);

  persistent_config.Load();

  moteus_controller.Start();
  command_manager.AsyncStart();
  multiplex_protocol.Start(moteus_controller.multiplex_server());

  auto old_time = timer.read_ms();

  for (;;) {
    rs485.Poll();
#if defined(TARGET_STM32G4)
    fdcan_micro_server.Poll();
#endif
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
