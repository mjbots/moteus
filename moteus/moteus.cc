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
#include "mjlib/micro/multiplex_protocol.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"

#include "moteus/board_debug.h"
#include "moteus/millisecond_timer.h"
#include "moteus/moteus_controller.h"
#include "moteus/moteus_hw.h"
#include "moteus/stm32f446_async_uart.h"
#include "moteus/stm32_flash.h"
#include "moteus/system_info.h"

using namespace moteus;
namespace micro = mjlib::micro;

int main(void) {
  DigitalIn hwrev0(HWREV_PIN0, PullUp);
  DigitalIn hwrev1(HWREV_PIN1, PullUp);
  DigitalIn hwrev2(HWREV_PIN2, PullUp);

  const uint8_t this_hw_rev =
      0x07 & (~(hwrev0.read() |
                (hwrev1.read() << 1) |
                (hwrev2.read() << 2)));
  MJ_ASSERT(this_hw_rev == MOTEUS_HW_REV);

  // I initially used a Ticker here to enqueue events at 1ms
  // intervals.  However, it introduced jitter into the current
  // sampling interrupt, and I couldn't figure out how to get the
  // interrupt priorities right.  Thus for now we just poll to look
  // for millisecond turnover.
  MillisecondTimer timer;

  micro::SizedPool<12288> pool;

  Stm32F446AsyncUart rs485(&pool, &timer, []() {
      Stm32F446AsyncUart::Options options;
      options.tx = PA_9;
      options.rx = PA_10;
      options.dir = PA_8;
      options.baud_rate = 3000000;
      return options;
    }());

  micro::MultiplexProtocolServer multiplex_protocol(&pool, &rs485, nullptr, {});

  micro::AsyncStream* serial = multiplex_protocol.MakeTunnel(1);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(serial);
  micro::CommandManager command_manager(&pool, serial, &write_stream);
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream);
  Stm32Flash flash_interface;
  micro::PersistentConfig persistent_config(pool, command_manager, flash_interface);

  SystemInfo system_info(pool, telemetry_manager);

  MoteusController moteus_controller(
      &pool, &persistent_config, &telemetry_manager, &timer);

  BoardDebug board_debug(
      &pool, &command_manager, &telemetry_manager,
      moteus_controller.bldc_servo());

  persistent_config.Register("id", multiplex_protocol.config(), [](){});

  persistent_config.Load();

  moteus_controller.Start();
  command_manager.AsyncStart();
  multiplex_protocol.Start();

  auto old_time = timer.read_ms();

  for (;;) {
    rs485.Poll();

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
