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

#include "fw/ccm.h"

namespace moteus {

class SimplePI {
 public:
  struct Config {
    float kp = 0.0f;
    float ki = 0.0f;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(kp));
      a->Visit(MJ_NVP(ki));
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

    float p = 0.0f;
    float command = 0.0f;

    void Clear() MOTEUS_CCM_ATTRIBUTE {
      // We implement this solely for speed, because on at least
      // Cortex-M4, just calling the constructor delegates to memset,
      // which is much slower than memberwise assignment.
      integral = 0.0f;

      desired = std::numeric_limits<float>::quiet_NaN();

      error = 0.0f;
      p = 0.0f;
      command = 0.0f;
    }

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(integral));
      a->Visit(MJ_NVP(desired));
      a->Visit(MJ_NVP(error));
      a->Visit(MJ_NVP(p));
      a->Visit(MJ_NVP(command));
    }
  };

  SimplePI(const Config* config, State* state)
      : config_(config), state_(state) {}

  float Apply(float measured, float input_desired,
              int rate_hz) MOTEUS_CCM_ATTRIBUTE {
    float desired = {};

    desired = input_desired;

    state_->desired = desired;
    state_->error = measured - desired;

    float to_update_i = state_->error * config_->ki / rate_hz;
    state_->integral += to_update_i;

    state_->p = config_->kp * state_->error;
    state_->command = -1.0f * (state_->p + state_->integral);

    return state_->command;
  }

 private:
  const Config* const config_;
  State* const state_;
};

}
