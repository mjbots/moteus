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
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"
#include "moteus/board_debug.h"
#include "moteus/stm32f446_async_uart.h"
#include "moteus/stm32f446_bldc_foc.h"
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

  Stm32F446AsyncUart::Options pc_options;
  pc_options.tx = PC_10;
  pc_options.rx = PC_11;
  pc_options.baud_rate = 115200;
  Stm32F446AsyncUart pc(&pool, &queue, pc_options);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(&pc);
  micro::CommandManager command_manager(&pool, &pc, &write_stream);
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream);
  Stm32Flash flash_interface;
  micro::PersistentConfig persistent_config(pool, command_manager, flash_interface);

  Stm32F446BldcFoc::Options bldc_options;
  bldc_options.pwm1 = PA_0;
  bldc_options.pwm2 = PA_1;
  bldc_options.pwm3 = PA_2;

  bldc_options.current1 = PC_5;
  bldc_options.current2 = PB_0_ALT0;
  bldc_options.vsense = PC_1_ALT1;

  bldc_options.debug_out = PB_3;

  Stm32F446BldcFoc bldc{&pool, bldc_options};
  Stm32F446BldcFoc::CommandData bldc_command;
  bldc_command.mode = Stm32F446BldcFoc::kPhasePwm;
  bldc_command.phase_a_millipercent = 2000;
  bldc_command.phase_b_millipercent = 3000;
  bldc_command.phase_c_millipercent = 4000;

  bldc.Command(bldc_command);

  SystemInfo system_info(pool, telemetry_manager);

  BoardDebug board_debug(&pool, &command_manager, &telemetry_manager);


  command_manager.AsyncStart();
  persistent_config.Load();

  Ticker ticker;
  micro::StaticFunction<void()> ms_poll  =[&]() {
    telemetry_manager.PollMillisecond();
    system_info.PollMillisecond();
    board_debug.PollMillisecond();
  };
  ticker.attach_us(queue.event(
      Callback<void()>(
          &ms_poll, &micro::StaticFunction<void()>::operator())), 1000);

  queue.dispatch_forever();

  return 0;
}
