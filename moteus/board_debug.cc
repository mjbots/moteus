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

#include "moteus/board_debug.h"

#include <cstdlib>
#include <functional>

#include "mbed.h"

#include "mjlib/base/tokenizer.h"
#include "mjlib/base/visitor.h"

#include "moteus/as5047.h"
#include "moteus/bldc_foc.h"
#include "moteus/drv8323.h"

namespace base = mjlib::base;
namespace micro = mjlib::micro;

namespace moteus {
constexpr float kPi = 3.14159265359f;

class BoardDebug::Impl {
 public:
  Impl(micro::Pool* pool,
       micro::PersistentConfig* persistent_config,
       micro::CommandManager* command_manager,
       micro::TelemetryManager* telemetry_manager)
      : drv8323_(pool, persistent_config, telemetry_manager,
                 []() {
                   Drv8323::Options options;
                   options.mosi = PA_7;
                   options.miso = PA_6;
                   options.sck = PA_5;
                   options.cs = PA_4;
                   options.enable = PA_3;
                   options.fault = PC_4;
                   options.hiz = PC_3;
                   return options;
                 }()),
        bldc_(pool, persistent_config, telemetry_manager, &as5047_,
              []() {
                 BldcFoc::Options options;
                 options.pwm1 = PA_0;
                 options.pwm2 = PA_1;
                 options.pwm3 = PA_2;

                 options.current1 = PC_5;
                 options.current2 = PB_0_ALT0;
                 options.vsense = PC_1_ALT1;

                 options.debug_out = PB_3;

                 return options;
              }()) {

    command_manager->Register(
        "d", std::bind(&Impl::HandleCommand, this,
                       std::placeholders::_1, std::placeholders::_2));

    data_update_ = telemetry_manager->Register("board_debug", &data_);
  }

  void PollMillisecond() {
    drv8323_.PollMillisecond();
  }

  void HandleCommand(const std::string_view& message,
                     const micro::CommandManager::Response& response) {
    base::Tokenizer tokenizer(message, " ");
    const auto command = tokenizer.next();
    if (command == "led") {
      const auto which_led = tokenizer.next();
      const auto state = tokenizer.next();

      if (which_led.empty() || state.empty()) {
        WriteMessage(response, "invalid led command\r\n");
        return;
      }

      DigitalOut* const led = (which_led == "1") ? &led1_ : &led2_;
      bool* const led_state = (which_led == "1") ? &data_.led1 : &data_.led2;
      const bool value = (state != "0");

      *led = !value;  // Our LEDs are active low.
      *led_state = value;
      WriteOk(response);

      data_update_();
      return;
    }
    if (command == "mote") {
      const auto value = tokenizer.next();

      drv8323_.Enable(!(value.empty() || value == "0"));
      WriteOk(response);
      return;
    }
    if (command == "motp") {
      const auto value = tokenizer.next();

      drv8323_.Power(!(value.empty() || value == "0"));
      WriteOk(response);
      return;
    }

    if (command == "off") {
      bldc_.ZeroOffset();
      WriteOk(response);
      return;
    }

    if (command == "pwm") {
      const auto phase_str = tokenizer.next();
      const auto magnitude_str = tokenizer.next();

      if (phase_str.empty() || magnitude_str.empty()) {
        WriteMessage(response, "missing phase or mag\r\n");
        return;
      }

      const float phase = std::strtof(phase_str.data(), nullptr);
      const float magnitude = std::strtof(magnitude_str.data(), nullptr);

      BldcFoc::CommandData command;
      command.mode = BldcFoc::kPhasePwm;

      auto amount = [&](float offset) {
        return (
            (std::sin(phase + offset) + 1.0f) *
            5000.0f * magnitude);
      };

      command.phase_a_centipercent = amount(0.0);
      command.phase_b_centipercent = amount(2 * kPi * 1.0f / 3.0f);
      command.phase_c_centipercent = amount(2 * kPi * 2.0f / 3.0f);

      bldc_.Command(command);
      WriteOk(response);
      return;
    }

    WriteMessage(response, "unknown command\r\n");
  }

  void WriteOk(const micro::CommandManager::Response& response) {
    WriteMessage(response, "OK\r\n");
  }

  void WriteMessage(const micro::CommandManager::Response& response,
                    const std::string_view& message) {
    AsyncWrite(*response.stream, message, response.callback);
  }

  struct Data {
    bool led1 = false;
    bool led2 = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(led1));
      a->Visit(MJ_NVP(led2));
    }
  };

  Data data_;
  micro::StaticFunction<void()> data_update_;

  DigitalOut led1_{PA_11, 1};
  DigitalOut led2_{PA_12, 1};

  AS5047 as5047_{[]() {
      AS5047::Options options;
      options.mosi = PB_15;
      options.miso = PB_14;
      options.sck = PB_13;
      options.cs = PB_12;
      return options;
    }()};

  Drv8323 drv8323_;
  BldcFoc bldc_;
};

BoardDebug::BoardDebug(micro::Pool* pool,
                       micro::PersistentConfig* persistent_config,
                       micro::CommandManager* command_manager,
                       micro::TelemetryManager* telemetry_manager)
    : impl_(pool, pool, persistent_config, command_manager, telemetry_manager) {}

BoardDebug::~BoardDebug() {}

void BoardDebug::PollMillisecond() { impl_->PollMillisecond(); }

}
