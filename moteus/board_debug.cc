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

#include "moteus/board_debug.h"

#include <cstdlib>
#include <functional>

#include "mbed.h"

#include "mjlib/base/inplace_function.h"
#include "mjlib/base/tokenizer.h"
#include "mjlib/base/visitor.h"

#include "moteus/as5047.h"
#include "moteus/bldc_servo.h"
#include "moteus/bootloader.h"
#include "moteus/drv8323.h"
#include "moteus/moteus_hw.h"

namespace base = mjlib::base;
namespace micro = mjlib::micro;
namespace multiplex = mjlib::multiplex;

namespace moteus {
constexpr float kPi = 3.14159265359f;
constexpr float kCalibrationStep = 0.002;
constexpr int kMaxCalMs = 200000;  // 200s

namespace {
void recurse(int count, base::inplace_function<void(int)> callback) {
  callback(count - 1);
}
}

class BoardDebug::Impl {
 public:
  Impl(micro::Pool* pool,
       micro::CommandManager* command_manager,
       micro::TelemetryManager* telemetry_manager,
       multiplex::MicroServer* multiplex_protocol,
       BldcServo* bldc_servo)
      : multiplex_protocol_(multiplex_protocol),
        bldc_(bldc_servo) {
    command_manager->Register(
        "d", std::bind(&Impl::HandleCommand, this,
                       std::placeholders::_1, std::placeholders::_2));

    data_update_ = telemetry_manager->Register("board_debug", &data_);
  }

  void PollMillisecond() {
    if (motor_cal_mode_ != kNoMotorCal) {
      DoCalibration();
    }
  }

  void DoCalibration() {
    // speed of 1 is 1 electrical phase per second
    const int kStep = static_cast<int>(cal_speed_ * 65536.0f / 1000.0f);
    const uint16_t position_raw = bldc_->status().position_raw;
    const int32_t delta =
        static_cast<int16_t>(position_raw - cal_old_position_raw_);
    cal_old_position_raw_ = position_raw;
    cal_position_delta_ += delta;
    cal_phase_ += ((motor_cal_mode_ == kPhaseUp) ? 1 : -1) * kStep;
    const bool phase_complete = std::abs(cal_position_delta_) > 65536;
    cal_count_++;

    if (cal_count_ > kMaxCalMs) {
      // Whoops, something is wrong.  Either this motor has a *lot* of
      // poles, or the magnet isn't functioning properly.  Just end
      // with an error.
      if (write_outstanding_) { return; }

      WriteMessage(cal_response_, "CAL timeout\r\n");
      cal_response_ = {};
      motor_cal_mode_ = kNoMotorCal;

      BldcServo::CommandData command;
      command.mode = BldcServo::kStopped;

      bldc_->Command(command);

      return;
    }

    switch (motor_cal_mode_) {
      case kPhaseUp: {
        if (phase_complete) {
          motor_cal_mode_ = kPhaseDown;
          cal_position_delta_ = 0;
        }
        break;
      }
      case kPhaseDown: {
        if (phase_complete) {
          // Try to write out our final message.
          if (write_outstanding_) { return; }


          WriteMessage(cal_response_, "CAL done\r\n");
          cal_response_ = {};
          motor_cal_mode_ = kNoMotorCal;

          BldcServo::CommandData command;
          command.mode = BldcServo::kStopped;

          bldc_->Command(command);

          return;
        }
        break;
      }
      case kNoMotorCal: {
        MJ_ASSERT(false);
        break;
      }
    }

    if (cal_count_ % 20 == 0 && !write_outstanding_) {
      ::snprintf(out_message_, sizeof(out_message_),
                 "%d %d %d\r\n",
                 motor_cal_mode_,
                 cal_phase_,
                 bldc_->status().position_raw);
      write_outstanding_ = true;
      AsyncWrite(*cal_response_.stream, out_message_, [this](auto) {
          write_outstanding_ = false;
        });
    }

    BldcServo::CommandData command;
    command.mode = BldcServo::kVoltageFoc;

    command.theta = (cal_phase_ / 65536.0f) * 2.0f * kPi;
    command.voltage = cal_magnitude_;
    bldc_->Command(command);
  }

  void HandleCommand(const std::string_view& message,
                     const micro::CommandManager::Response& response) {
    base::Tokenizer tokenizer(message, " ");
    const auto cmd_text = tokenizer.next();
    if (cmd_text == "led") {
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

    if (cmd_text == "stop") {
      BldcServo::CommandData command;
      command.mode = BldcServo::kStopped;

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "pwm") {
      const auto phase_str = tokenizer.next();
      const auto magnitude_str = tokenizer.next();

      if (phase_str.empty() || magnitude_str.empty()) {
        WriteMessage(response, "missing phase or mag\r\n");
        return;
      }

      const float phase = std::strtof(phase_str.data(), nullptr);
      const float magnitude = std::strtof(magnitude_str.data(), nullptr);

      BldcServo::CommandData command;
      command.mode = BldcServo::kVoltageFoc;

      command.theta = phase;
      command.voltage = magnitude;

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "cal") {
      const auto magnitude_str = tokenizer.next();

      if (magnitude_str.empty()) {
        WriteMessage(response, "missing mag\r\n");
        return;
      }

      cal_speed_ = 1.0f;

      while (tokenizer.remaining().size()) {
        const auto token = tokenizer.next();

        // We accept optional arguments, each prefixed by a single character.

        if (token.size() < 1) { continue; }
        const char option = token[0];
        const float value = std::strtof(&token[1], nullptr);

        switch (option) {
          case 's': {
            cal_speed_ = value;
            break;
          }
          default: {
            WriteMessage(response, "unknown cal option\r\n");
            return;
          }
        }
      }

      cal_response_ = response;
      motor_cal_mode_ = kPhaseUp;
      cal_phase_ = 0.;
      cal_count_ = 0;
      cal_old_position_raw_ = bldc_->status().position_raw;
      cal_position_delta_ = 0;

      cal_magnitude_ = std::strtof(magnitude_str.data(), nullptr);

      write_outstanding_ = true;
      AsyncWrite(*cal_response_.stream, "CAL start\r\n", [this](auto) {
          write_outstanding_ = false;
        });

      return;
    }

    if (cmd_text == "dq") {
      const auto d_str = tokenizer.next();
      const auto q_str = tokenizer.next();

      if (d_str.empty() || q_str.empty()) {
        WriteMessage(response, "missing d/q current\r\n");
        return;
      }

      const float d = std::strtof(d_str.data(), nullptr);
      const float q = std::strtof(q_str.data(), nullptr);

      BldcServo::CommandData command;
      command.mode = BldcServo::kCurrent;

      command.i_d_A = d;
      command.i_q_A = q;

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "pos" || cmd_text == "tmt" || cmd_text == "zero") {
      const auto pos_str = tokenizer.next();
      const auto vel_str = tokenizer.next();
      const auto max_t_str = tokenizer.next();

      if (pos_str.empty() ||
          vel_str.empty() ||
          max_t_str.empty()) {
        WriteMessage(response, "missing p/v/i\r\n");
        return;
      }

      const float pos = std::strtof(pos_str.data(), nullptr);
      const float vel = std::strtof(vel_str.data(), nullptr);
      const float max_t = std::strtof(max_t_str.data(), nullptr);

      BldcServo::CommandData command;
      // We default to no timeout for debug commands.
      command.timeout_s = std::numeric_limits<float>::quiet_NaN();

      while (tokenizer.remaining().size()) {
        const auto token = tokenizer.next();
        // We accept optional arguments, each prefixed by a single
        // character.
        if (token.size() < 1) { continue; }
        const char option = token[0];
        const float value = std::strtof(&token[1], nullptr);

        switch (option) {
          case 'p': {
            command.kp_scale = value;
            break;
          }
          case 'd': {
            command.kd_scale = value;
            break;
          }
          case 's': {
            command.stop_position = value;
            break;
          }
          case 'f': {
            command.feedforward_Nm = value;
            break;
          }
          case 't': {
            command.timeout_s = value;
            break;
          }
          default: {
            WriteMessage(response, "unknown option\r\n");
            return;
          }
        }
      }

      command.mode =
          (cmd_text == "pos") ? BldcServo::kPosition :
          (cmd_text == "tmt") ? BldcServo::kPositionTimeout :
          (cmd_text == "zero") ? BldcServo::kZeroVelocity :
          BldcServo::kStopped;

      command.position = pos;
      command.velocity = vel;
      command.max_torque_Nm = max_t;

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "index") {
      const auto pos_value = tokenizer.next();
      if (pos_value.empty()) {
        WriteMessage(response, "missing index value\r\n");
        return;
      }

      const float index_value = std::strtof(pos_value.data(), nullptr);

      BldcServo::CommandData command;
      command.mode = BldcServo::kStopped;

      command.set_position = index_value;

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "rezero") {
      BldcServo::CommandData command;
      command.mode = BldcServo::kStopped;

      const auto pos_value = tokenizer.next();
      command.rezero_position =
          (pos_value.empty()) ? 0.0 :
          std::strtof(pos_value.data(), nullptr);

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "clk") {
      const uint32_t clock = bldc_->clock();
      ::snprintf(out_message_, sizeof(out_message_),
                 "%lu\r\n", clock);
      WriteMessage(response, out_message_);
      return;
    }

    if (cmd_text == "die") {
      mbed_die();
    }

    if (cmd_text == "assert") {
      MJ_ASSERT(false);
    }

    if (cmd_text == "stack") {
      volatile int* ptr = {};
      *ptr = 45;
      Recurse(10000);
      WriteOk(response);
      return;
    }

    if (cmd_text == "loop") {
      for (;;) {}
    }

    if (cmd_text == "reset") {
      NVIC_SystemReset();
    }

    if (cmd_text == "flash") {
      // TODO: Get the USART and direction pin from our config.
      MultiplexBootloader(multiplex_protocol_->config()->id, USART1, GPIOA, 8);
      // We should never get here.
      MJ_ASSERT(false);
    }

    WriteMessage(response, "unknown command\r\n");
  }

  void Recurse(int count) {
    recurse(count, [this](int value) { this->Recurse(value - 1); });
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
  base::inplace_function<void()> data_update_;

  DigitalOut led1_{DEBUG_LED1, 1};
  DigitalOut led2_{POWER_LED};

  multiplex::MicroServer* multiplex_protocol_;
  BldcServo* const bldc_;

  char out_message_[20] = {};

  micro::CommandManager::Response cal_response_;
  enum MotorCalMode {
    kNoMotorCal,
    kPhaseUp,
    kPhaseDown,
  };
  MotorCalMode motor_cal_mode_ = kNoMotorCal;
  uint16_t cal_phase_ = 0;
  uint32_t cal_count_ = 0;
  uint16_t cal_old_position_raw_ = 0;
  int32_t cal_position_delta_ = 0;
  float cal_magnitude_ = 0.0f;
  float cal_speed_ = 1.0f;
  bool write_outstanding_ = false;
};

BoardDebug::BoardDebug(micro::Pool* pool,
                       micro::CommandManager* command_manager,
                       micro::TelemetryManager* telemetry_manager,
                       multiplex::MicroServer* micro_server,
                       BldcServo* bldc_servo)
    : impl_(pool, pool, command_manager, telemetry_manager,
            micro_server, bldc_servo) {}

BoardDebug::~BoardDebug() {}

void BoardDebug::PollMillisecond() { impl_->PollMillisecond(); }

}
