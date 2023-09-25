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

#include <cmath>
#include <limits>

#include "mjlib/base/limit.h"
#include "mjlib/base/visitor.h"

#include "fw/moteus_hw.h"

namespace moteus {

class PID {
 public:
  struct Config {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float iratelimit = -1.0f;
    float ilimit = 0.0f;
    float max_desired_rate = 0.0f;  // 0 is unlimited
    int8_t sign = 1;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(kp));
      a->Visit(MJ_NVP(ki));
      a->Visit(MJ_NVP(kd));
      a->Visit(MJ_NVP(iratelimit));
      a->Visit(MJ_NVP(ilimit));
      a->Visit(MJ_NVP(max_desired_rate));
      a->Visit(MJ_NVP(sign));
    }
  };

  struct State {
    float integral = 0.0f;
    // When starting with desired rate limits in place, we by default
    // always accept the first desired command with no limiting (users
    // can of course override this value if they want to start from
    // some predetermined value).
    float desired = std::numeric_limits<float>::quiet_NaN();

    // The following are not actually part of the "state", but are
    // present for purposes of being logged with it.
    float error = 0.0f;
    float error_rate = 0.0f;

    float p = 0.0f;
    float d = 0.0f;
    float pd = 0.0f;
    float command = 0.0f;

    void Clear() MOTEUS_CCM_ATTRIBUTE {
      // We implement this solely for speed, because on at least
      // Cortex-M4, just calling the constructor delegates to memset,
      // which is much slower than memberwise assignment.
      integral = 0.0f;

      desired = std::numeric_limits<float>::quiet_NaN();

      error = 0.0f;
      error_rate = 0.0f;
      p = 0.0f;
      d = 0.0f;
      pd = 0.0f;
      command = 0.0f;
    }

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(integral));
      a->Visit(MJ_NVP(desired));
      a->Visit(MJ_NVP(error));
      a->Visit(MJ_NVP(error_rate));
      a->Visit(MJ_NVP(p));
      a->Visit(MJ_NVP(d));
      a->Visit(MJ_NVP(pd));
      a->Visit(MJ_NVP(command));
    }
  };

  PID(const Config* config, State* state)
      : config_(config), state_(state) {}

  struct ApplyOptions {
    float kp_scale = 1.0f;
    float kd_scale = 1.0f;

    ApplyOptions() {}
  };

  float Apply(float measured, float input_desired,
              float measured_rate, float input_desired_rate,
              int rate_hz,
              ApplyOptions apply_options = {}) MOTEUS_CCM_ATTRIBUTE {
    float desired = {};
    float desired_rate = {};

    // First apply max_desired_rate
    if (config_->max_desired_rate != 0.0f &&
        std::isfinite(state_->desired)) {
      const float max_step = config_->max_desired_rate / rate_hz;
      const float proposed_step = input_desired - state_->desired;
      const float actual_step =
          mjlib::base::Limit(proposed_step, -max_step, max_step);
      desired = state_->desired + actual_step;
      desired_rate =
          mjlib::base::Limit(input_desired_rate, -config_->max_desired_rate,
                             config_->max_desired_rate);
    } else {
      desired = input_desired;
      desired_rate = input_desired_rate;
    }

    state_->desired = desired;
    state_->error = measured - desired;
    state_->error_rate = measured_rate - desired_rate;

    const float max_i_update = config_->iratelimit / rate_hz;
    float to_update_i = state_->error * config_->ki / rate_hz;
    if (max_i_update > 0.0f) {
      if (to_update_i > max_i_update) {
        to_update_i = max_i_update;
      } else if (to_update_i < -max_i_update) {
        to_update_i = -max_i_update;
      }
    }

    state_->integral += to_update_i;

    if (state_->integral > config_->ilimit) {
      state_->integral = config_->ilimit;
    } else if (state_->integral < -config_->ilimit) {
      state_->integral = -config_->ilimit;
    }

    state_->p = apply_options.kp_scale * config_->kp * state_->error;
    state_->d = apply_options.kd_scale * config_->kd * state_->error_rate;
    state_->pd = state_->p + state_->d;

    state_->command = config_->sign * (state_->pd + state_->integral);

    return state_->command;
  }

 private:
  const Config* const config_;
  State* const state_;
};

}
