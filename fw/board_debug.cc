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

#include "fw/board_debug.h"

#include <cstdlib>
#include <functional>
#include <numeric>

#include "mbed.h"

#include "mjlib/base/inplace_function.h"
#include "mjlib/base/tokenizer.h"
#include "mjlib/base/visitor.h"

#include "fw/as5047.h"
#include "fw/bldc_servo.h"
#include "fw/bootloader.h"
#include "fw/drv8323.h"
#include "fw/moteus_hw.h"

namespace base = mjlib::base;
namespace micro = mjlib::micro;
namespace multiplex = mjlib::multiplex;

namespace moteus {
constexpr float kCalibrationStep = 0.002;
constexpr float kMaxCalPoleCount = 50.0f;

namespace {
void recurse(int count, base::inplace_function<void(int)> callback) {
  callback(count - 1);
}

bool ParseOptions(BldcServo::CommandData* command, base::Tokenizer* tokenizer,
                  const char* valid_options) {
  while (tokenizer->remaining().size()) {
    const auto token = tokenizer->next();
    // We accept optional arguments, each prefixed by a single
    // character.
    if (token.size() < 1) { continue; }
    const char option = token[0];
    if (::strchr(valid_options, option) == nullptr) {
      return false;
    }

    const float value = std::strtof(&token[1], nullptr);
    switch (option) {
      case 'p': {
        command->kp_scale = value;
        break;
      }
      case 'd': {
        command->kd_scale = value;
        break;
      }
      case 's': {
        command->stop_position = value;
        break;
      }
      case 'f': {
        command->feedforward_Nm = value;
        break;
      }
      case 't': {
        command->timeout_s = value;
        break;
      }
      case 'a': {
        command->accel_limit = value;
        break;
      }
      case 'v': {
        command->velocity_limit = value;
        break;
      }
      case 'o': {
        command->fixed_voltage_override = value;
        break;
      }
      default: {
        return false;
      }
    }
  }
  return true;
}
}

class BoardDebug::Impl {
 public:
  struct HistogramSource;

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
    if (histogram_active_) {
      DoHistogram();
    }
  }

  bool ParseHistogramChannel(HistogramSource* source, const std::string_view& spec) {
    // The format of this specification is a single character source:
    //
    //  t - electrical theta
    //  c - compensated
    //  f - filtered
    //  o - offset
    //  r - raw
    //  v - pll output velocity
    //  ... etc ...
    //
    // followed by an optional single digit to specify the encoder
    // channel, and then an optional list of non-digit flags:
    //
    //  d - derivative

    source->derivative = false;

    if (spec.size() < 2) { return true; }
    switch (spec[1]) {
      case 't': {
        source->type = HistogramSource::kElectricalTheta;
        break;
      }
      case 'c': {
        source->type = HistogramSource::kEncoderCompensated;
        break;
      }
      case 'f': {
        source->type = HistogramSource::kEncoderFiltered;
        break;
      }
      case 'o': {
        source->type = HistogramSource::kEncoderOffset;
        break;
      }
      case 'r': {
        source->type = HistogramSource::kEncoderRaw;
        break;
      }
      case 'v': {
        source->type = HistogramSource::kEncoderVelocity;
        break;
      }
      case 'd': {
        source->type = HistogramSource::kCurrentD;
        break;
      }
      case 'q': {
        source->type = HistogramSource::kCurrentQ;
        break;
      }
      case 'e': {
        source->type = HistogramSource::kPositionError;
        break;
      }
      case 'E': {
        source->type = HistogramSource::kPositionErrorRate;
        break;
      }
      default: {
        return true;
      }
    }

    size_t offset = 2;

    if (spec.size() <= offset) { return false; }

    if (spec[2] >= '0' && spec[2] <= '9') {
      source->encoder_channel = spec[2] - '0';
      offset += 1;
    }

    for (; offset < spec.size(); offset++) {
      switch (spec[offset]) {
        case 'd': {
          source->derivative = true;
          break;
        }
        default: {
          return true;
        }
      }
    }

    return false;
  }

  void DoCalibration() {
    const auto old_phase = cal_phase_;

    // speed of 1 is 1 electrical phase per second
    const int kStep = static_cast<int>(cal_speed_ * 65536.0f / 1000.0f);

    const auto& motor_position = bldc_->motor_position();
    const auto& motor_config = *bldc_->motor_position_config();
    const int commutation_source = motor_config.commutation_source;

    const uint64_t raw =
        static_cast<uint64_t>(
            static_cast<uint32_t>(
                motor_position.sources[commutation_source].compensated_value));
    const uint64_t cpr = motor_config.sources[commutation_source].cpr;
    const uint16_t position_raw =
        static_cast<uint16_t>(std::min<uint64_t>(65535, (raw * 65536ll) / cpr));

    if (cal_old_position_raw_) {
      const int32_t delta =
          static_cast<int16_t>(position_raw - *cal_old_position_raw_);
      cal_position_delta_ += delta;
    }
    const bool phase_complete = std::abs(cal_position_delta_) > 65536;
    cal_old_position_raw_ = position_raw;

    cal_phase_ += ((motor_cal_mode_ == kPhaseUp) ? 1 : -1) * kStep;
    cal_count_++;

    const float kMaxTimeMs =
        2.f * // margin
        2.f * // up and down
        1000.f * // ms in s
        kMaxCalPoleCount /
        cal_speed_;

    if (cal_count_ > kMaxTimeMs) {
      // Whoops, something is wrong.  Either this motor has a *lot* of
      // poles, or the magnet isn't functioning properly.  Just end
      // with an error.
      if (write_outstanding_) { return; }

      WriteMessage(cal_response_, "CAL timeout\r\n");
      cal_response_ = {};
      motor_cal_mode_ = kNoMotorCal;

      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kStopped;

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
          command.mode = BldcServo::Mode::kStopped;

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

    if ((cal_count_ % 10) == 0 && !write_outstanding_) {
      const auto& status = bldc_->status();

      ::snprintf(out_message_, sizeof(out_message_),
                 "%d %d %u i1=%d i2=%d i3=%d d=%ld\r\n",
                 motor_cal_mode_,
                 old_phase,
                 position_raw,
                 static_cast<int>(status.cur1_A * 1000),
                 static_cast<int>(status.cur2_A * 1000),
                 static_cast<int>(status.cur3_A * 1000),
                 cal_position_delta_);
      write_outstanding_ = true;
      AsyncWrite(*cal_response_.stream, out_message_, [this](auto) {
          write_outstanding_ = false;
        });
    }

    BldcServo::CommandData command;
    command.mode = BldcServo::Mode::kVoltageFoc;

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
        WriteMessage(response, "ERR invalid led command\r\n");
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
      command.mode = BldcServo::Mode::kStopped;

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "raw") {
      const auto pwm1_str = tokenizer.next();
      const auto pwm2_str = tokenizer.next();
      const auto pwm3_str = tokenizer.next();

      if (pwm1_str.empty() || pwm2_str.empty() || pwm3_str.empty()) {
        WriteMessage(response, "ERR missing pwm value\r\n");
        return;
      }

      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kPwm;
      command.pwm.a = std::strtof(pwm1_str.data(), nullptr);
      command.pwm.b = std::strtof(pwm2_str.data(), nullptr);
      command.pwm.c = std::strtof(pwm3_str.data(), nullptr);

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "pwm") {
      const auto phase_str = tokenizer.next();
      const auto magnitude_str = tokenizer.next();
      const auto phase_rate_str = tokenizer.next();

      if (phase_str.empty() || magnitude_str.empty()) {
        WriteMessage(response, "ERR missing phase or mag\r\n");
        return;
      }

      const float phase = std::strtof(phase_str.data(), nullptr);
      const float magnitude = std::strtof(magnitude_str.data(), nullptr);

      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kVoltageFoc;

      command.theta = phase;
      command.voltage = magnitude;

      if (!phase_rate_str.empty()) {
        command.theta_rate = std::strtof(phase_rate_str.data(), nullptr);
      }

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "cal") {
      const auto magnitude_str = tokenizer.next();

      if (magnitude_str.empty()) {
        WriteMessage(response, "ERR missing mag\r\n");
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
            WriteMessage(response, "ERR unknown cal option\r\n");
            return;
          }
        }
      }

      // Do we at least have a rotor level home on our commutation
      // source?
      const auto& motor_position = bldc_->motor_position();
      if (motor_position.error != MotorPosition::Status::kNone) {
        WriteMessage(response, "ERR encoder configuration error\r\n");
        return;
      }
      if (!motor_position.theta_valid) {
        WriteMessage(response, "ERR no theta available\r\n");
        return;
      }


      cal_response_ = response;
      motor_cal_mode_ = kPhaseUp;
      cal_phase_ = 0.;
      cal_count_ = 0;
      cal_old_position_raw_.reset();
      cal_position_delta_ = 0;

      cal_magnitude_ = std::strtof(magnitude_str.data(), nullptr);

      write_outstanding_ = true;
      AsyncWrite(*cal_response_.stream, "CAL start 2\r\n", [this](auto) {
          write_outstanding_ = false;
        });

      return;
    }

    if (cmd_text == "v") {
      const auto a_str = tokenizer.next();
      const auto b_str = tokenizer.next();
      const auto c_str = tokenizer.next();

      if (a_str.empty() || b_str.empty() || c_str.empty()) {
        WriteMessage(response, "ERR missing a/b/c voltage\r\n");
        return;
      }

      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kVoltage;
      command.phase_v.a = std::strtof(a_str.data(), nullptr);
      command.phase_v.b = std::strtof(b_str.data(), nullptr);
      command.phase_v.c = std::strtof(c_str.data(), nullptr);

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "vdq") {
      const auto d_str = tokenizer.next();
      const auto q_str = tokenizer.next();

      if (d_str.empty() || q_str.empty()) {
        WriteMessage(response, "ERR missing d/q voltage\r\n");
        return;
      }

      const float d_V = std::strtof(d_str.data(), nullptr);
      const float q_V = std::strtof(q_str.data(), nullptr);

      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kVoltageDq;

      command.d_V = d_V;
      command.q_V = q_V;

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "dq") {
      const auto d_str = tokenizer.next();
      const auto q_str = tokenizer.next();

      if (d_str.empty() || q_str.empty()) {
        WriteMessage(response, "ERR missing d/q current\r\n");
        return;
      }

      const float d = std::strtof(d_str.data(), nullptr);
      const float q = std::strtof(q_str.data(), nullptr);

      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kCurrent;

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
        WriteMessage(response, "ERR missing p/v/i\r\n");
        return;
      }

      const float pos = std::strtof(pos_str.data(), nullptr);
      const float vel = std::strtof(vel_str.data(), nullptr);
      const float max_t = std::strtof(max_t_str.data(), nullptr);

      BldcServo::CommandData command;
      // We default to no timeout for debug commands.
      command.timeout_s = std::numeric_limits<float>::quiet_NaN();

      if (!ParseOptions(&command, &tokenizer, "pdsftavo")) {
        WriteMessage(response, "ERR unknown option\r\n");
        return;
      }

      command.mode =
          (cmd_text == "pos") ? BldcServo::Mode::kPosition :
          (cmd_text == "tmt") ? BldcServo::Mode::kPositionTimeout :
          (cmd_text == "zero") ? BldcServo::Mode::kZeroVelocity :
          BldcServo::Mode::kStopped;

      command.position = pos;
      command.velocity = vel;
      command.max_torque_Nm = max_t;

      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "within") {
      const auto min_str = tokenizer.next();
      const auto max_str = tokenizer.next();
      const auto max_t_str = tokenizer.next();

      if (min_str.empty() ||
          max_str.empty() ||
          max_t_str.empty()) {
        WriteMessage(response, "ERR missing min/max/t\r\n");
        return;
      }

      const float min_pos = std::strtof(min_str.data(), nullptr);
      const float max_pos = std::strtof(max_str.data(), nullptr);
      const float max_t = std::strtof(max_t_str.data(), nullptr);

      BldcServo::CommandData command;
      command.timeout_s = std::numeric_limits<float>::quiet_NaN();

      if (!ParseOptions(&command, &tokenizer, "pdtf")) {
        WriteMessage(response, "ERR unknown option\r\n");
        return;
      }

      command.mode = BldcServo::Mode::kStayWithinBounds;

      command.bounds_min = min_pos;
      command.bounds_max = max_pos;
      command.max_torque_Nm = max_t;

      bldc_->Command(command);

      WriteOk(response);
      return;
    }

    if (cmd_text == "ind") {
      const auto volt_str = tokenizer.next();
      const auto period_str = tokenizer.next();

      if (volt_str.empty() ||
          period_str.empty()) {
        WriteMessage(response, "ERR missing volt/period\r\n");
        return;
      }

      const float volt = std::strtof(volt_str.data(), nullptr);
      const int8_t period = std::strtol(period_str.data(), nullptr, 10);
      if (period <= 0) {
        WriteMessage(response, "ERR period must > 0\r\n");
        return;
      }

      BldcServo::CommandData command;

      command.mode = BldcServo::Mode::kMeasureInductance;

      command.d_V = volt;
      command.meas_ind_period = period;

      bldc_->Command(command);

      WriteOk(response);
      return;
    }

    if (cmd_text == "brake") {
      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kBrake;
      bldc_->Command(command);
      WriteOk(response);
      return;
    }

    if (cmd_text == "exact" || cmd_text == "index" /* deprecated */) {
      const auto pos_value = tokenizer.next();
      if (pos_value.empty()) {
        WriteMessage(response, "ERR missing index value\r\n");
        return;
      }

      const float index_value = std::strtof(pos_value.data(), nullptr);

      bldc_->SetOutputPosition(index_value);

      WriteOk(response);
      return;
    }

    if (cmd_text == "nearest" || cmd_text == "rezero" /* deprecated */) {
      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kStopped;

      const auto pos_value = tokenizer.next();
      const float rezero_value =
          (pos_value.empty()) ? 0.0f :
          std::strtof(pos_value.data(), nullptr);

      bldc_->SetOutputPositionNearest(rezero_value);

      WriteOk(response);
      return;
    }

    if (cmd_text == "cfg-set-output") {
      const auto pos_value = tokenizer.next();
      const float set_value =
          (pos_value.empty()) ? 0.0f :
          std::strtof(pos_value.data(), nullptr);

      // Get us within 1 revolution.
      auto* const config = bldc_->motor_position_config();
      config->output.offset = 0.0f;
      bldc_->SetOutputPositionNearest(set_value);

      const float cur_output = bldc_->motor_position().position;
      const float error = set_value - cur_output;

      config->output.offset += error * config->output.sign;

      bldc_->SetOutputPositionNearest(set_value);

      WriteOk(response);
      return;
    }

    if (cmd_text == "req-reindex") {
      bldc_->RequireReindex();

      WriteOk(response);
      return;
    }

    if (cmd_text == "hstart") {
      histogram_count_ms_ = 0;

      // Our defaults.
      hist_x_source_.type = HistogramSource::kEncoderCompensated;
      hist_x_source_.encoder_channel = 0;
      hist_x_source_.derivative = false;

      hist_y_source_.type = HistogramSource::kEncoderCompensated;
      hist_y_source_.encoder_channel = 0;
      hist_y_source_.derivative = true;

      hist_xmin_ = 0.0f;
      hist_xmax_ = 1.0f;

      bool err = false;

      while (true) {
        const auto maybe_option = tokenizer.next();
        if (maybe_option.empty()) { break; }

        // Try to parse this.
        if (maybe_option[0] == 'x') {
          err |= ParseHistogramChannel(&hist_x_source_, maybe_option);
        } else if (maybe_option[0] == 'y') {
          err |= ParseHistogramChannel(&hist_y_source_, maybe_option);
        } else if (maybe_option[0] == 'm') {
          hist_xmin_ = std::strtof(&maybe_option[1], nullptr);
        } else if (maybe_option[0] == 'M') {
          hist_xmax_ = std::strtof(&maybe_option[1], nullptr);
        } else {
          err = true;
        }
      }

      if (err) {
        WriteMessage(response, "ERR could not parse hist options\r\n");
        return;
      }


      histogram_active_ = true;
      WriteOk(response);
      return;
    }

    if (cmd_text == "hend") {
      if (!histogram_active_) {
        WriteOk(response);
        return;
      }

      // Stop adding more things.
      histogram_active_ = false;

      for (size_t i = 0; i < kHistogramBinCount; i++) {
        histogram_values_[i] /= histogram_counts_[i];
      }

      histogram_response_ = response;
      EmitHistogramResponse(0);
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
      // Ensure everything is stopped!
      MoteusEnsureOff();

      MultiplexBootloader(multiplex_protocol_->config()->id, USART1, GPIOA, 8);
      // We should never get here.
      MJ_ASSERT(false);
    }

    WriteMessage(response, "ERR unknown command\r\n");
  }

  float SampleHistogram(HistogramSource* source, bool x_axis) {
    auto maybe_limit_x =
        [&](float value) {
          if (!x_axis) { return value; }
          if (value < 0.0f) { return 0.0f; }
          if (value > 1.0f) { return 1.0f; }
          return value;
        };

    const auto& motor_position = bldc_->motor_position();
    const auto& motor_config = *bldc_->motor_position_config();
    const float cpr = static_cast<float>(
        motor_config.sources[source->encoder_channel].cpr);
    const auto& encoder = motor_position.sources[source->encoder_channel];
    const auto& bldc_status = bldc_->status();

    const float value =
        [&]() {
          switch (source->type) {
            case HistogramSource::kNone: {
              return 0.0f;
            }
            case HistogramSource::kElectricalTheta: {
              return motor_position.electrical_theta / k2Pi;
            }
            case HistogramSource::kEncoderVelocity: {
              return encoder.velocity;
            }
            case HistogramSource::kEncoderCompensated: {
              return encoder.compensated_value / cpr;
            }
            case HistogramSource::kEncoderFiltered: {
              return encoder.filtered_value / cpr;
            }
            case HistogramSource::kEncoderOffset: {
              return encoder.offset_value / cpr;
            }
            case HistogramSource::kEncoderRaw: {
              return encoder.raw / cpr;
            }
            case HistogramSource::kCurrentD: {
              return bldc_status.d_A;
            }
            case HistogramSource::kCurrentQ: {
              return bldc_status.q_A;
            }
            case HistogramSource::kPositionError: {
              return bldc_status.pid_position.error;
            }
            case HistogramSource::kPositionErrorRate: {
              return bldc_status.pid_position.error_rate;
            }
          }
          return 0.0f;
        }();

    if (!source->derivative) {
      return maybe_limit_x(value);
    }

    const float old_value = source->old_value;
    source->old_value = value;

    if (histogram_count_ms_ == 0) {
      return 0.0f;
    } else {
      const float velocity = value - old_value;
      const float wrapped = HistogramSource::IsCyclic(source->type) ?
          MotorPosition::WrapBalancedCpr(velocity, 1.0) : velocity;
      return maybe_limit_x(wrapped * 1e3f);
    }
  }

  void DoHistogram() {
    const float x_value = SampleHistogram(&hist_x_source_, true);
    const float y_value = SampleHistogram(&hist_y_source_, false);

    if (histogram_count_ms_ == 0) {
      // Start with everything at 0.  We don't count a sample this
      // first time, so that any numerically differentiated values
      // will be valid.
      for (auto& value : histogram_values_) { value = 0; }
      for (auto& value : histogram_counts_) { value = 0; }
    } else {
      if (x_value >= hist_xmin_ && x_value <= hist_xmax_) {
        const float scaled_x = (x_value - hist_xmin_) / (hist_xmax_ - hist_xmin_);
        // And after that, we update the current state with a
        // complementary filter.
        const int index =
            std::max<int>(
                0, std::min<int>(
                    histogram_values_.size() - 1,
                    static_cast<int32_t>(scaled_x *
                                         histogram_values_.size())));

        if (histogram_counts_[index] !=
            std::numeric_limits<uint16_t>::max()) {
          histogram_values_[index] += y_value;
          histogram_counts_[index]++;
        }
      }
    }

    histogram_count_ms_++;
  }

  void EmitHistogramResponse(size_t offset) {
    if (offset >= histogram_values_.size()) {
      WriteOk(histogram_response_);
      histogram_response_ = {};
      return;
    }

    ::snprintf(out_message_, sizeof(out_message_),
               "%d %f\r\n",
               offset, static_cast<double>(histogram_values_[offset]));
    write_outstanding_ = true;
    AsyncWrite(*histogram_response_.stream, out_message_,
               [this, offset](auto) {
                 write_outstanding_ = false;
                 EmitHistogramResponse(offset + 1);
               });
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

  DigitalOut led1_{g_hw_pins.debug_led1, 1};
  DigitalOut led2_{g_hw_pins.power_led};

  multiplex::MicroServer* multiplex_protocol_;
  BldcServo* const bldc_;

  char out_message_[64] = {};

  micro::CommandManager::Response cal_response_;

  enum MotorCalMode {
    kNoMotorCal,
    kPhaseUp,
    kPhaseDown,
  };
  MotorCalMode motor_cal_mode_ = kNoMotorCal;
  uint16_t cal_phase_ = 0;
  uint32_t cal_count_ = 0;
  std::optional<uint16_t> cal_old_position_raw_;
  int32_t cal_position_delta_ = 0;
  float cal_magnitude_ = 0.0f;
  float cal_speed_ = 1.0f;
  bool write_outstanding_ = false;

  micro::CommandManager::Response histogram_response_;

  bool histogram_active_ = false;
  uint32_t histogram_count_ms_ = 0;

  struct HistogramSource {
    enum Type {
      kNone = 0,

      kElectricalTheta,

      // The actual raw encoder values.
      kEncoderCompensated,
      kEncoderFiltered,
      kEncoderOffset,
      kEncoderRaw,

      // The pll filtered velocity.
      kEncoderVelocity,

      kCurrentD,
      kCurrentQ,

      kPositionError,
      kPositionErrorRate,
    };

    static bool IsCyclic(Type t) {
      switch (t) {
        case kNone:
        case kEncoderVelocity:
        case kCurrentD:
        case kCurrentQ:
        case kPositionError:
        case kPositionErrorRate: {
          return false;
        }
        case kElectricalTheta:
        case kEncoderCompensated:
        case kEncoderFiltered:
        case kEncoderOffset:
        case kEncoderRaw: {
          return true;
        }
      }
      return false;
    }

    Type type = kNone;
    bool derivative = false;
    int encoder_channel = 0;

    // State used for the numerically differentiated encoder options.
    float old_value = 0.0f;
  };

  HistogramSource hist_x_source_;
  HistogramSource hist_y_source_;

  // Record x positions only between these two values, scaled
  // accordingly.
  float hist_xmin_ = 0.0f;
  float hist_xmax_ = 1.0f;

  static constexpr size_t kHistogramBinCount = 128;
  std::array<float, kHistogramBinCount> histogram_values_ = {};
  std::array<uint16_t, kHistogramBinCount> histogram_counts_ = {};
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
