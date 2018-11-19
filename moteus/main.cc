// Copyright 2018 Josh Pieper, jjp@pobox.com.  All rights reserved.

#include <inttypes.h>

#include <functional>

#include "mbed.h"
#include "mbed_events.h"
#include "rtos_idle.h"

#include "mjlib/micro/async_exclusive.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "moteus/stm32f446_async_uart.h"
// #include "stm32f446_bldc_foc.h"

using namespace moteus;
namespace micro = mjlib::micro;

namespace {

static constexpr char kMessage[] = "hello\r\n";

class Emitter {
 public:
  Emitter(DigitalOut* led) : led_(led) {}

  void HandleCommand(const std::string_view& command,
                     const micro::CommandManager::Response& response) {
    if (command == "on") {
      *led_ = 1;
    } else if (command == "off") {
      *led_ = 0;
    } else {
      AsyncWrite(*response.stream, "UNKNOWN\r\n", response.callback);
      return;
    }
    AsyncWrite(*response.stream, "OK\r\n", response.callback);
  }

 private:
  DigitalOut* const led_;
};

void new_idle_loop() {
}
}

int main(void) {
  // We want no sleep modes at all for highest timing resolution
  // w.r.t. interrupts.
  rtos_attach_idle_hook(&new_idle_loop);

  EventQueue queue(2048);
  micro::SizedPool<2048> pool;

  DigitalOut led(LED2);

  Stm32F446AsyncUart::Options pc_options;
  pc_options.tx = PC_10;
  pc_options.rx = PC_11;
  pc_options.baud_rate = 9600;
  Stm32F446AsyncUart pc(&pool, &queue, pc_options);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(&pc);
  micro::CommandManager command_manager(&pool, &pc, &write_stream);

  // Stm32F446BldcFoc::Options bldc_options;
  // bldc_options.pwm1 = PA_0;
  // bldc_options.pwm2 = PA_1;
  // bldc_options.pwm3 = PA_2;

  // bldc_options.current1 = PC_5;
  // bldc_options.current2 = PB_0_ALT0;
  // bldc_options.vsense = PC_1_ALT1;

  // bldc_options.debug_out = PB_3;

  // Stm32F446BldcFoc bldc{bldc_options};
  // Stm32F446BldcFoc::CommandData bldc_command;
  // bldc_command.mode = Stm32F446BldcFoc::kPhasePwm;
  // bldc_command.phase_a_millipercent = 2000;
  // bldc_command.phase_b_millipercent = 3000;
  // bldc_command.phase_c_millipercent = 4000;

  // bldc.Command(bldc_command);

  Emitter emitter(&led);

  command_manager.Register("led",
                           std::bind(&Emitter::HandleCommand, &emitter,
                                     std::placeholders::_1, std::placeholders::_2));

  command_manager.AsyncStart();
  queue.dispatch_forever();

  return 0;
}
