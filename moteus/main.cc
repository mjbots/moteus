// Copyright 2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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
#include "mbed_events.h"
#include "rtos_idle.h"

#include "mjlib/micro/async_exclusive.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/multiplex_protocol.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"
#include "moteus/board_debug.h"
#include "moteus/stm32f446_async_uart.h"
#include "moteus/stm32_flash.h"
#include "moteus/system_info.h"

using namespace moteus;
namespace micro = mjlib::micro;

namespace {
void new_idle_loop() {
  for (;;) {
    SystemInfo::idle_count++;
  }
}
}

int main(void) {
  // We want no sleep modes at all for highest timing resolution
  // w.r.t. interrupts.
  rtos_attach_idle_hook(&new_idle_loop);

  EventQueue queue(2048);
  micro::SizedPool<8192> pool;

  Stm32F446AsyncUart pc(&pool, &queue, []() {
      Stm32F446AsyncUart::Options pc_options;
      pc_options.tx = PC_10;
      pc_options.rx = PC_11;
      pc_options.baud_rate = 115200;
      return pc_options;
    }());
  Stm32F446AsyncUart rs485(&pool, &queue, []() {
      Stm32F446AsyncUart::Options options;
      options.tx = PA_9;
      options.rx = PA_10;
      options.dir = PA_8;
      options.enable_delay_us = 1;
      options.disable_delay_us = 2;
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

  BoardDebug board_debug(&pool, &persistent_config, &command_manager, &telemetry_manager);

  command_manager.AsyncStart();
  persistent_config.Load();
  multiplex_protocol.Start();

  // I initially used a Ticker here to enqueue events at 1ms
  // intervals.  However, it introduced jitter into the current
  // sampling interrupt, and I couldn't figure out how to get the
  // interrupt priorities right.  Thus for now we just poll to look
  // for millisecond turnover.
  Timer timer;
  timer.start();
  int old_time = timer.read_ms();

  for (;;) {
    queue.dispatch(0);

    const int new_time = timer.read_ms();

    if (new_time != old_time) {
      telemetry_manager.PollMillisecond();
      system_info.PollMillisecond();
      board_debug.PollMillisecond();

      old_time = new_time;
    }
    SystemInfo::idle_count++;
  }

  return 0;
}
