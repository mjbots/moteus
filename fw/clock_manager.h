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

#pragma once

#include "mbed.h"

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"

#include "fw/millisecond_timer.h"

namespace moteus {

class ClockManager {
 public:
  static constexpr int kMaxExtraTrim = 8;

  ClockManager(MillisecondTimer* timer,
               mjlib::micro::PersistentConfig& persistent_config,
               mjlib::micro::CommandManager& command_manager)
      : timer_(timer) {
    persistent_config.Register("clock", &clock_, [this]() {
        this->UpdateConfig();
      });
    command_manager.Register("clock", [this](auto&& command, auto&& response) {
        this->Command(command, response);
      });
  }

  void UpdateConfig() {
    const int32_t trim =
        std::max<int32_t>(
            0, std::min<int32_t>(127, clock_.hsitrim + extra_trim_));
    RCC->ICSCR = (RCC->ICSCR & ~0xff000000) | (trim << 24);
  }

  void SetTrim(int extra_trim) {
    extra_trim_ = std::max(-kMaxExtraTrim, std::min(kMaxExtraTrim, extra_trim));
    UpdateConfig();
  }

  int trim() const {
    return extra_trim_;
  }

  void Command(const std::string_view& command,
               const mjlib::micro::CommandManager::Response& response) {
    mjlib::base::Tokenizer tokenizer(command, " ");
    const auto cmd_text = tokenizer.next();

    if (cmd_text == "us") {
      snprintf(output_, sizeof(output_), "%" PRIu32 "\r\n",
               static_cast<uint32_t>(timer_->read_us()));
      WriteMessage(output_, response);
    } else if (cmd_text == "trim") {
      const auto value_str = tokenizer.next();

      if (value_str == "") {
        snprintf(output_, sizeof(output_), "%d\r\n", extra_trim_);
        WriteMessage(output_, response);
      } else {
        SetTrim(std::strtol(value_str.data(), nullptr, 0));
        WriteMessage("OK\r\n", response);
      }
    } else {
      WriteMessage("ERR unknown clock\r\n", response);
    }
  }

  void WriteMessage(const std::string_view& message,
                    const mjlib::micro::CommandManager::Response& response) {
    mjlib::micro::AsyncWrite(*response.stream, message, response.callback);
  }

 private:
  struct Config {
    int32_t hsitrim = 64;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(hsitrim));
    }
  };

  MillisecondTimer* const timer_;
  Config clock_;
  char output_[16] = {};
  int extra_trim_ = 0;
};

}
