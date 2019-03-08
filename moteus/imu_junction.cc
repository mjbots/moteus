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

#include "moteus/imu_junction_hw.h"
#include "moteus/millisecond_timer.h"
#include "moteus/multiplex_protocol_bridge.h"
#include "moteus/stm32f446_async_uart.h"
#include "moteus/stm32_flash.h"
#include "moteus/system_info.h"

using namespace moteus;
namespace micro = mjlib::micro;

int main(void) {
  MillisecondTimer timer;

  micro::SizedPool<12288> pool;

  Stm32F446AsyncUart rs485(&pool, &timer, []() {
      Stm32F446AsyncUart::Options options;
      options.tx = PC_10;
      options.rx = PC_11;
      options.dir = PC_12;
      options.enable_delay_us = 1;
      options.disable_delay_us = 2;
      options.baud_rate = 3000000;
      options.rx_buffer_size = 256;
      return options;
    }());

  Stm32F446AsyncUart slave1(&pool, &timer, []() {
      Stm32F446AsyncUart::Options options;
      options.tx = PA_2;
      options.rx = PA_3;
      options.dir = PA_1;
      options.enable_delay_us = 1;
      options.disable_delay_us = 2;
      options.baud_rate = 3000000;
      options.rx_buffer_size = 256;
      return options;
    }());

  Stm32F446AsyncUart slave2(&pool, &timer, []() {
      Stm32F446AsyncUart::Options options;
      options.tx = PC_6;
      options.rx = PC_7;
      options.dir = PC_8;
      options.enable_delay_us = 1;
      options.disable_delay_us = 2;
      options.baud_rate = 3000000;
      options.rx_buffer_size = 256;
      return options;
    }());

  micro::MultiplexProtocolServer multiplex_protocol(&pool, &rs485, nullptr, []() {
      micro::MultiplexProtocolServer::Options options;
      options.default_id = 64;
      return options;
    }());

  micro::AsyncStream* serial = multiplex_protocol.MakeTunnel(1);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(serial);
  micro::CommandManager command_manager(&pool, serial, &write_stream);
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream);
  Stm32Flash flash_interface;
  micro::PersistentConfig persistent_config(pool, command_manager, flash_interface);

  SystemInfo system_info(pool, telemetry_manager);

  Bridge<2> bridge(&telemetry_manager, &multiplex_protocol, &slave1, &slave2);

  persistent_config.Register("id", multiplex_protocol.config(), [](){});

  persistent_config.Load();

  command_manager.AsyncStart();
  multiplex_protocol.Start();
  bridge.Start();

  auto old_time = timer.read_ms();

  for (;;) {
    rs485.Poll();
    slave1.Poll();
    slave2.Poll();

    const auto new_time = timer.read_ms();

    if (new_time != old_time) {
      telemetry_manager.PollMillisecond();
      system_info.PollMillisecond();

      old_time = new_time;
    }
    SystemInfo::idle_count++;
  }

  return 0;
}
