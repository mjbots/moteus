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

#include "moteus/drv8323.h"

namespace base = mjlib::base;
namespace micro = mjlib::micro;

namespace moteus {
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
                   return options;
                 }()) {

    command_manager->Register(
        "d", std::bind(&Impl::HandleCommand, this,
                       std::placeholders::_1, std::placeholders::_2));

    data_update_ = telemetry_manager->Register("board_debug", &data_);

    as5047_spi_.format(16, 1);
    as5047_spi_.frequency(10000000);
  }

  void PollMillisecond() {
    // NOTE: This seems to take around 7us (this is 28% of our full
    // cycle period).  I think that if I didn't go through the HAL, I
    // could get this down to something under 2us, which would be more
    // reasonable.
    as5047_cs_ = 0;
    data_.as5047 = as5047_spi_.write(0xffff) & 0x3fff;
    as5047_cs_ = 1;

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
    uint16_t as5047 = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(led1));
      a->Visit(MJ_NVP(led2));
      a->Visit(MJ_NVP(as5047));
    }
  };

  Data data_;
  micro::StaticFunction<void()> data_update_;

  DigitalOut led1_{PA_11, 1};
  DigitalOut led2_{PA_12, 1};

  SPI as5047_spi_{PB_15, PB_14, PB_13};
  DigitalOut as5047_cs_{PB_12, 1};

  Drv8323 drv8323_;
};

BoardDebug::BoardDebug(micro::Pool* pool,
                       micro::PersistentConfig* persistent_config,
                       micro::CommandManager* command_manager,
                       micro::TelemetryManager* telemetry_manager)
    : impl_(pool, pool, persistent_config, command_manager, telemetry_manager) {}

BoardDebug::~BoardDebug() {}

void BoardDebug::PollMillisecond() { impl_->PollMillisecond(); }

}
