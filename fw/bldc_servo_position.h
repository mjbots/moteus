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

#include "mjlib/base/assert.h"

#include "fw/bldc_servo_structs.h"
#include "fw/ccm.h"
#include "fw/measured_hw_rev.h"
#include "fw/motor_position.h"

namespace moteus {

// We put these functions into a class merely so that we can leave
// them in the header and also have a section definition.
class BldcServoPosition {
 public:
  static void DoVelocityModeLimits(
      BldcServoStatus* status,
      const BldcServoConfig* config,
      float rate_hz,
      BldcServoCommandData* data,
      float velocity) MOTEUS_CCM_ATTRIBUTE {
    const float period_s = 1.0f / rate_hz;

    if (!std::isnan(data->velocity_limit)) {
      if (velocity > data->velocity_limit) { velocity = data->velocity_limit; }
      if (velocity < -data->velocity_limit) { velocity = -data->velocity_limit; }
    }

    // We may have accel or velocity limits here or both, but we don't
    // care about position, only about velocity.
    if (!std::isnan(data->accel_limit)) {
      const float dv = velocity - *status->control_velocity;
      const float initial_sign = (dv > 0.0f) ? 1.0f : -1.0f;
      const float acceleration = data->accel_limit * initial_sign;

      status->control_acceleration = acceleration;
      *status->control_velocity += acceleration * period_s;
      const float final_sign =
          (velocity > *status->control_velocity) ? 1.0f : -1.0f;
      if (final_sign != initial_sign) {
        status->control_acceleration = 0.0f;
        status->control_velocity = velocity;
        status->trajectory_done = true;
      }
    } else {
      // We must have only a velocity limit.  This is easy.
      status->control_acceleration = 0.0f;
      status->control_velocity = velocity;
      status->trajectory_done = true;
    }
  }

  static void DoVelocityOnlyLimit(
      BldcServoStatus* status,
      float dx,
      BldcServoCommandData* data,
      float velocity,
      float period_s) MOTEUS_CCM_ATTRIBUTE {
    const float initial_sign = dx < 0.0f ? 1.0f : -1.0f;
    status->control_acceleration = 0.0f;
    status->control_velocity = -initial_sign * data->velocity_limit;

    const float next_dx = dx - *status->control_velocity * period_s;
    const float final_sign = (next_dx < 0.0f) ? 1.0f : -1.0f;

    // Will we complete this cycle?
    if (final_sign != initial_sign) {
      data->position = std::numeric_limits<float>::quiet_NaN();
      data->position_relative_raw.reset();
      status->control_velocity = velocity;
      status->trajectory_done = true;
    }
  }

  // Compute the time until we should switch from acceleration to deceleration.
  // Returns a value in [0, dt]. Returns 0 if already decelerating, dt if no switch.
  static float ComputeSwitchTime(float v_abs, float dx_abs, float a, float dt) {
    // The switch point is where remaining_distance = stopping_distance.
    // stopping_distance = v²/(2a)
    // After accelerating for time t: v_new = v + a*t, dx_new = dx - v*t - 0.5*a*t²
    // Setting dx_new = v_new²/(2a) and solving:
    //   dx - v*t - 0.5*a*t² = (v + a*t)²/(2a)
    //   2a*dx - 2a*v*t - a²*t² = v² + 2*v*a*t + a²*t²
    //   2a*dx = v² + 4*v*a*t + 2*a²*t²
    //   a²*t² + 2*v*a*t + (0.5*v² - a*dx) = 0
    // Using quadratic formula:
    //   t = (-v + sqrt(0.5*v² + a*dx)) / a

    const float stop_distance = v_abs * v_abs / (2.0f * a);
    if (dx_abs <= stop_distance) {
      return 0.0f;  // Already past switch point
    }

    const float discriminant = 0.5f * v_abs * v_abs + a * dx_abs;
    if (discriminant < 0.0f) return dt;

    const float t_switch = (-v_abs + std::sqrt(discriminant)) / a;
    if (t_switch <= 0.0f) return 0.0f;
    if (t_switch >= dt) return dt;
    return t_switch;
  }

  static void DoVelocityAndAccelLimits(
      BldcServoStatus* status,
      const BldcServoConfig* config,
      float rate_hz,
      BldcServoCommandData* data,
      float velocity) MOTEUS_CCM_ATTRIBUTE {
    const float period_s = 1.0f / rate_hz;

    const float a = data->accel_limit;
    const float v0 = *status->control_velocity;
    const float vf = velocity;

    // Delta between current control state and command.
    const float dx = MotorPosition::IntToFloat(
        (*data->position_relative_raw - *status->control_position_raw));

    if (std::isnan(data->accel_limit)) {
      DoVelocityOnlyLimit(status, dx, data, velocity, period_s);
      return;
    }

    // Work in target velocity frame.
    // v_frame represents our velocity relative to the target velocity.
    const float v_frame = v0 - vf;
    const float v_frame_abs = std::abs(v_frame);
    const float dx_abs = std::abs(dx);

    // Target is stationary in this frame. We're moving toward it if v_frame
    // has same sign as dx (closing in on target position).
    const bool moving_towards = (v_frame * dx >= 0.0f) && (dx != 0.0f);

    // Handle overspeed case.
    if (std::isfinite(data->velocity_limit) && std::abs(v0) > data->velocity_limit) {
      status->control_acceleration = std::copysign(a, -v0);
      *status->control_velocity += status->control_acceleration * period_s;
      return;
    }

    float v1;
    float position_step;

    // For trajectories without velocity limit, use exact switch time calculation.
    if (!std::isfinite(data->velocity_limit) && moving_towards) {
      const float t_switch = ComputeSwitchTime(v_frame_abs, dx_abs, a, period_s);

      if (t_switch > 0.0f && t_switch < period_s) {
        // Switch happens within this step! Handle exactly.
        // Phase 1: accelerate for t_switch
        const float a_accel = std::copysign(a, dx);
        const float v_peak = v0 + a_accel * t_switch;
        const float dx_accel = (v0 + v_peak) * 0.5f * t_switch;

        // Phase 2: decelerate for remaining time
        const float t_decel = period_s - t_switch;
        const float a_decel = -a_accel;
        v1 = v_peak + a_decel * t_decel;
        const float dx_decel = (v_peak + v1) * 0.5f * t_decel;

        status->control_acceleration = a_decel;
        status->control_velocity = v1;
        position_step = dx_accel + dx_decel;
        status->exact_position_step = position_step;
        status->use_exact_position_step = true;
      } else if (t_switch == 0.0f) {
        // Decelerating (already past switch point)
        status->control_acceleration = std::copysign(a, -v_frame);
        *status->control_velocity += status->control_acceleration * period_s;
        v1 = *status->control_velocity;
        position_step = (v0 + v1) * 0.5f * period_s;
      } else {
        // Accelerating (switch is in a future timestep)
        status->control_acceleration = std::copysign(a, dx);
        *status->control_velocity += status->control_acceleration * period_s;
        v1 = *status->control_velocity;
        position_step = (v0 + v1) * 0.5f * period_s;
      }
    } else if (moving_towards) {
      // With velocity limit: use stop_distance to decide accel vs decel.
      // Use v_frame since we're targeting velocity vf, not 0.
      const float stop_distance = v_frame_abs * v_frame_abs / (2.0f * a);
      // Add discrete-time margin: during one step we travel ~v*dt and can't
      // change direction mid-step, so need to start decelerating slightly early.
      const float margin = v_frame_abs * period_s;
      const bool should_decel = (dx_abs <= stop_distance + margin);

      if (should_decel) {
        // Decelerating toward target
        status->control_acceleration = std::copysign(a, -v_frame);
        *status->control_velocity += status->control_acceleration * period_s;
        v1 = *status->control_velocity;
      } else if (std::abs(v0) >= data->velocity_limit) {
        // At velocity limit - cruise
        status->control_acceleration = 0.0f;
        v1 = v0;
      } else {
        // Accelerating
        status->control_acceleration = std::copysign(a, dx);
        *status->control_velocity += status->control_acceleration * period_s;
        v1 = *status->control_velocity;
        // Clamp to velocity limit
        if (std::abs(v1) > data->velocity_limit) {
          status->control_velocity = std::copysign(data->velocity_limit, v1);
          status->control_acceleration = 0.0f;
          v1 = *status->control_velocity;
        }
      }
      position_step = (v0 + v1) * 0.5f * period_s;
    } else {
      // Moving away from target (or stationary) - decelerate toward it
      status->control_acceleration = std::copysign(a, -v_frame);
      *status->control_velocity += status->control_acceleration * period_s;
      v1 = *status->control_velocity;
      position_step = (v0 + v1) * 0.5f * period_s;
    }

    // Compute predicted position after step.
    const float dx_after_step = dx - position_step;

    // Check for completion.
    // Velocity check: did we cross through the target velocity this cycle?
    const float signed_vel_lower = std::min(v0, v1);
    const float signed_vel_upper = std::max(v0, v1);
    const bool velocity_crossed_target = signed_vel_lower <= vf && signed_vel_upper >= vf;

    // Position check: either time-based (for moving cases) or threshold-based.
    bool position_near;
    if (std::abs(vf) > 1e-6f && std::abs(v1) > 1e-6f) {
      // Time-based: within 4 timesteps of target
      position_near = std::abs(dx_after_step / v1) <= (4.0f * period_s);
    } else {
      // Position threshold for stop trajectories
      const float pos_threshold = 4.0f * a * period_s * period_s;
      position_near = std::abs(dx_after_step) < pos_threshold;
    }

    if (velocity_crossed_target && position_near) {
      data->position = std::numeric_limits<float>::quiet_NaN();
      data->position_relative_raw.reset();
      status->control_acceleration = 0.0f;
      status->control_velocity = vf;
      status->trajectory_done = true;
    }
  }

  static void UpdateTrajectory(
      BldcServoStatus* status,
      const BldcServoConfig* config,
      float rate_hz,
      BldcServoCommandData* data,
      float velocity) MOTEUS_CCM_ATTRIBUTE {
    // Clamp the desired velocity to our limit if we have one.
    if (!std::isnan(data->velocity_limit)) {
      if (velocity > data->velocity_limit) { velocity = data->velocity_limit; }
      if (velocity < -data->velocity_limit) { velocity = -data->velocity_limit; }
    }

    if (!data->position_relative_raw) {
      DoVelocityModeLimits(
          status, config, rate_hz, data, velocity);
    } else {
      DoVelocityAndAccelLimits(
          status, config, rate_hz, data, velocity);
    }
  }

  static float UpdateCommand(
      BldcServoStatus* status,
      const BldcServoConfig* config,
      const BldcServoPositionConfig* position_config,
      const MotorPosition::Status* position,
      int64_t absolute_relative_delta,
      float rate_hz,
      BldcServoCommandData* data,
      float velocity) MOTEUS_CCM_ATTRIBUTE {

    if (std::isnan(velocity)) {
      velocity = 0.0f;
    }

    // We go to some lengths in our conversions to and from
    // control_position_raw so as to avoid converting a float directly to
    // an int64, which calls out to a system library that is pretty
    // slow.

    if (std::isnan(data->velocity_limit) &&
        std::isnan(data->accel_limit)) {
      status->trajectory_done = true;
      status->control_acceleration = 0.0f;
      status->control_velocity = velocity;
    } else if (!!data->position_relative_raw ||
               !std::isnan(velocity)) {
      status->trajectory_done = false;
    }

    if (!!data->position_relative_raw &&
        std::isnan(data->velocity_limit) &&
        std::isnan(data->accel_limit)) {
      // With no limits, we immediately set the control position and
      // velocity.
      status->control_position_raw = *data->position_relative_raw;
      data->position = std::numeric_limits<float>::quiet_NaN();
      data->position_relative_raw.reset();
      status->control_acceleration = 0.0f;
      status->control_velocity = velocity;
    } else if (!status->control_position_raw) {
      status->control_position_raw = position->position_relative_raw;

      status->control_acceleration = 0.0f;
      if (std::abs(status->velocity_filt) <
          config->velocity_zero_capture_threshold) {
        status->control_velocity = 0.0f;
      } else {
        status->control_velocity = status->velocity_filt;
      }
    }

    // Save velocity before trajectory update for exact integration.
    const float v_before = status->control_velocity.value_or(0.0f);

    if (!status->trajectory_done) {
      UpdateTrajectory(status, config, rate_hz, data, velocity);
    }

    if (*status->control_velocity > status->motor_max_velocity) {
      status->control_acceleration = 0.0f;
      status->control_velocity = status->motor_max_velocity;
    } else if (*status->control_velocity < -status->motor_max_velocity) {
      status->control_acceleration = 0.0f;
      status->control_velocity = -status->motor_max_velocity;
    }

    auto velocity_command = *status->control_velocity;

    // Compute position step.
    // If exact trajectory step was computed (handling mid-step transitions),
    // use it directly. Otherwise use standard kinematic integration.
    float step;
    if (status->use_exact_position_step) {
      step = status->exact_position_step;
      status->use_exact_position_step = false;  // Reset for next iteration
    } else {
      // Use exact kinematic integration when accelerating:
      // (v_before + v_after) / 2 * dt = v0*dt + 0.5*a*dt² for constant accel.
      // When not accelerating (velocity set directly), use final velocity.
      const float v_after = velocity_command;
      const bool is_accelerating = (status->control_acceleration != 0.0f);
      step = is_accelerating ?
          (v_before + v_after) * 0.5f / rate_hz :
          v_after / rate_hz;
    }
    const int64_t int64_step =
        (static_cast<int64_t>(
            static_cast<int32_t>((static_cast<float>(1ll << 32) * step))) <<
         16);
    status->control_position_raw = *status->control_position_raw + int64_step;

    if (data->position_relative_raw && !std::isnan(velocity)) {
      const float tstep = velocity / rate_hz;
      const int64_t tint64_step =
        (static_cast<int64_t>(
            static_cast<int32_t>((static_cast<float>(1ll << 32) * tstep))) <<
         16);
      data->position_relative_raw = *data->position_relative_raw + tint64_step;
    }

    if (std::isfinite(config->max_position_slip) && !data->synthetic_theta) {
      const int64_t current_position = position->position_relative_raw;
      const int64_t slip =
          MotorPosition::FloatToInt(config->max_position_slip);

      const int64_t error =
          current_position - *status->control_position_raw;
      if (error < -slip) {
        *status->control_position_raw = current_position + slip;
      }
      if (error > slip) {
        *status->control_position_raw = current_position - slip;
      }
    }

    if (std::isfinite(config->max_velocity_slip) && !data->synthetic_theta) {
      const float slip = config->max_velocity_slip;
      const float error = status->velocity - *status->control_velocity;
      if (error < -slip) {
        status->control_acceleration = 0.0f;
        status->control_velocity = status->velocity + slip;
      }
      if (error > slip) {
        status->control_acceleration = 0.0f;
        status->control_velocity = status->velocity - slip;
      }
    }

    bool hit_limit = false;
    const auto delta = absolute_relative_delta;

    const auto saturate = [&](auto value, auto compare) MOTEUS_CCM_ATTRIBUTE {
      if (std::isnan(value)) { return; }
      const auto limit_value = MotorPosition::FloatToInt(value) - delta;
      if (compare(*status->control_position_raw - limit_value, 0)) {
        status->control_position_raw = limit_value;
        hit_limit = true;
      }
    };
    if (!data->ignore_position_bounds) {
      saturate(position_config->position_min,
               [](auto l, auto r) { return l < r; });
      saturate(position_config->position_max,
               [](auto l, auto r) { return l > r; });
    }

    if (!!data->stop_position_relative_raw) {
      const int64_t stop_position_raw = *data->stop_position_relative_raw;

      auto sign = [](auto value) MOTEUS_CCM_ATTRIBUTE -> float {
        if (value < 0) { return -1.0f; }
        if (value > 0) { return 1.0f; }
        return 0.0f;
      };
      if (sign(*status->control_position_raw -
               stop_position_raw) * velocity_command > 0.0f) {
        // We are moving away from the stop position.  Force it to be
        // there and zero out our velocity command.
        status->control_position_raw = stop_position_raw;
        status->control_acceleration = 0.0f;
        status->control_velocity = 0.0f;
        status->trajectory_done = true;
        data->position = std::numeric_limits<float>::quiet_NaN();
        data->position_relative_raw.reset();
        data->velocity = 0.0f;
        hit_limit = true;
      }
    }

    if (hit_limit) {
      // We have hit a limit.  Assume a velocity of 0.
      velocity_command = 0.0f;
      status->control_acceleration = 0.0f;
      status->control_velocity = 0.0f;
    }

    status->control_position =
        !status->control_position_raw ?
        std::numeric_limits<float>::quiet_NaN() :
        MotorPosition::IntToFloat(
            *status->control_position_raw + absolute_relative_delta);

    return velocity_command;
  }
};

}
