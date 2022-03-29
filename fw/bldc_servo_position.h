// Copyright 2018-2022 Josh Pieper, jjp@pobox.com.
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

#include "fw/bldc_servo_structs.h"
#include "fw/ccm.h"
#include "fw/measured_hw_rev.h"

namespace moteus {

// We put these functions into a class merely so that we can leave
// them in the header and also have a section definition.
class BldcServoPosition {
 public:

  static float UpdateCommand(
      BldcServoStatus* status,
      const BldcServoConfig* config,
      const BldcServoPositionConfig* position_config,
      float motor_scale16,
      float rate_hz,
      BldcServoCommandData* data,
      float velocity) MOTEUS_CCM_ATTRIBUTE {

    // We go to some lengths in our conversions to and from
    // control_position so as to avoid converting a float directly to
    // an int64, which calls out to a system library that is pretty
    // slow.

    if (!std::isnan(data->position)) {
      status->control_position =
          static_cast<int64_t>(65536ll * 65536ll) *
          static_cast<int64_t>(
              static_cast<int32_t>(motor_scale16 * data->position));
      data->position = std::numeric_limits<float>::quiet_NaN();
    } else if (!status->control_position) {
      status->control_position = status->unwrapped_position_raw;
    }

    auto velocity_command = velocity;

    // This limits our usable velocity to 20kHz modulo the position
    // scale at a 40kHz switching frequency.  1.2 million RPM should
    // be enough for anybody?
    status->control_position =
        (*status->control_position +
         65536ll * static_cast<int32_t>(
             (65536.0f * motor_scale16 * velocity_command) /
             rate_hz));

    if (std::isfinite(config->max_position_slip)) {
      const int64_t current_position = status->unwrapped_position_raw;
      const int64_t slip =
          static_cast<int64_t>(65536ll * 65536ll) *
          static_cast<int32_t>(motor_scale16 * config->max_position_slip);

      const int64_t error =
          current_position - *status->control_position;
      if (error < -slip) {
        *status->control_position = current_position + slip;
      }
      if (error > slip) {
        *status->control_position = current_position - slip;
      }
    }

    bool hit_limit = false;

    const auto saturate = [&](auto value, auto compare) MOTEUS_CCM_ATTRIBUTE {
      if (std::isnan(value)) { return; }
      const auto limit_value = (
          static_cast<int64_t>(65536ll * 65536ll) *
          static_cast<int64_t>(
              static_cast<int32_t>(motor_scale16 * value)));
      if (compare(*status->control_position, limit_value)) {
        status->control_position = limit_value;
        hit_limit = true;
      }
    };
    saturate(position_config->position_min,
             [](auto l, auto r) { return l < r; });
    saturate(position_config->position_max,
             [](auto l, auto r) { return l > r; });

    if (!std::isnan(data->stop_position)) {
      const int64_t stop_position_raw =
          static_cast<int64_t>(65536ll * 65536ll) *
          static_cast<int64_t>(
              static_cast<int32_t>(motor_scale16 * data->stop_position));

      auto sign = [](auto value) MOTEUS_CCM_ATTRIBUTE -> float {
        if (value < 0) { return -1.0f; }
        if (value > 0) { return 1.0f; }
        return 0.0f;
      };
      if (sign(*status->control_position -
               stop_position_raw) * velocity_command > 0.0f) {
        // We are moving away from the stop position.  Force it to be
        // there and zero out our velocity command.
        status->control_position = stop_position_raw;
        data->velocity = 0.0f;
        hit_limit = true;
      }
    }

    if (hit_limit) {
      // We have hit a limit.  Assume a velocity of 0.
      velocity_command = 0.0f;
    }

    return velocity_command;
  }
};

}
