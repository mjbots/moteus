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

namespace base = mjlib::base;
namespace micro = mjlib::micro;

namespace moteus {
class BoardDebug::Impl {
 public:
  Impl(micro::CommandManager* command_manager,
       micro::TelemetryManager* telemetry_manager) {
    command_manager->Register(
        "d", std::bind(&Impl::HandleCommand, this,
                       std::placeholders::_1, std::placeholders::_2));

    data_update_ = telemetry_manager->Register("board_debug", &data_);

    as5047_spi_.format(16, 1);
    as5047_spi_.frequency(10000000);

    // Configure the 8323 MISO to have a pullup.
    const auto tmp = GPIOA->PUPDR;
    GPIOA->PUPDR = (tmp & ~GPIO_PUPDR_PUPD6_Msk) | (1 << GPIO_PUPDR_PUPD6_Pos);

    drv8323_spi_.format(16, 1);
    drv8323_spi_.frequency(1000000);
  }

  void PollMillisecond() {
    // NOTE: This seems to take around 7us (this is 28% of our full
    // cycle period).  I think that if I didn't go through the HAL, I
    // could get this down to something under 2us, which would be more
    // reasonable.
    as5047_cs_ = 0;
    data_.as5047 = as5047_spi_.write(0xffff) & 0x3fff;
    as5047_cs_ = 1;

    auto read_8323 = [&](int reg) {
      drv8323_cs_ = 0;
      auto result = drv8323_spi_.write(0x8000 | (reg << 11)) & 0x7ff;
      drv8323_cs_ = 1;
      wait_us(1);
      return result;
    };

    auto& d = data_.drv8323;
    d.fault_status_1 = read_8323(0);
    d.vgs_status_2 = read_8323(1);
    d.driver_control = read_8323(2);
    d.gate_drive_hs = read_8323(3);
    d.gate_drive_ls = read_8323(4);
    d.ocp_control = read_8323(5);
    d.cs_control = read_8323(6);

    d.fault = drv8323_fault_.read() == 0;
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
    if (command == "motw") {
      const auto reg = tokenizer.next();
      const auto value = tokenizer.next();

      if (reg.empty() || value.empty()) {
        WriteMessage(response, "invalid motor write register\r\n");
        return;
      }

      const int int_reg = std::strtol(reg.data(), nullptr, 0);
      const int int_value = std::strtol(value.data(), nullptr, 0);

      drv8323_cs_ = 0;
      drv8323_spi_.write((int_reg << 11) | (int_value & 0x7ff));
      drv8323_cs_ = 1;

      WriteOk(response);
      return;
    }
    if (command == "mote") {
      const auto value = tokenizer.next();

      if (value.empty() || value == "0") {
        drv8323_enable_ = 0;
      } else {
        drv8323_enable_ = 1;
      }
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

  struct Drv8323 {
    uint16_t fault_status_1 = 0;
    uint16_t vgs_status_2 = 0;
    uint16_t driver_control = 0;
    uint16_t gate_drive_hs = 0;
    uint16_t gate_drive_ls = 0;
    uint16_t ocp_control = 0;
    uint16_t cs_control = 0;
    bool fault = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(fault_status_1));
      a->Visit(MJ_NVP(vgs_status_2));
      a->Visit(MJ_NVP(driver_control));
      a->Visit(MJ_NVP(gate_drive_hs));
      a->Visit(MJ_NVP(gate_drive_ls));
      a->Visit(MJ_NVP(ocp_control));
      a->Visit(MJ_NVP(cs_control));
      a->Visit(MJ_NVP(fault));
    }
  };


  struct Data {
    bool led1 = false;
    bool led2 = false;
    uint16_t as5047 = 0;
    Drv8323 drv8323;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(led1));
      a->Visit(MJ_NVP(led2));
      a->Visit(MJ_NVP(as5047));
      a->Visit(MJ_NVP(drv8323));
    }
  };

  Data data_;
  micro::StaticFunction<void()> data_update_;

  DigitalOut led1_{PA_11, 1};
  DigitalOut led2_{PA_12, 1};

  SPI as5047_spi_{PB_15, PB_14, PB_13};
  DigitalOut as5047_cs_{PB_12, 1};

  SPI drv8323_spi_{PA_7, PA_6, PA_5};
  DigitalOut drv8323_cs_{PA_4, 1};

  DigitalOut drv8323_enable_{PA_3, 0};
  DigitalIn drv8323_fault_{PC_4, PullUp};
};

BoardDebug::BoardDebug(micro::Pool* pool,
                       micro::CommandManager* command_manager,
                       micro::TelemetryManager* telemetry_manager)
    : impl_(pool, command_manager, telemetry_manager) {}

BoardDebug::~BoardDebug() {}

void BoardDebug::PollMillisecond() { impl_->PollMillisecond(); }

}
