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

  static float CalculateAcceleration(
      float prev_accel,
      BldcServoCommandData* data,
      float a,
      float v0,
      float vf,
      float dx,
      float dt) MOTEUS_CCM_ATTRIBUTE {
    // This logic is broken out primarily so that early-return can be
    // used as a control flow mechanism to aid factorization.

    // If we are overspeed, we always slow down to the velocity
    // limit first.
    if (std::isfinite(data->velocity_limit) &&
        std::abs(v0) > data->velocity_limit) {
      return std::copysign(a, -v0);
    }

    // Perform all operations in the target reference frame,
    // i.e. we'll transform our frame so that the target velocity is
    // 0.

    const auto v_frame = v0 - vf;

    if ((v_frame * dx) >= 0.0f && dx != 0.0f) {
      // Moving towards the target.
      // With exact kinematic integration, the stopping distance v²/(2a) is exact.
      // However, we make the switch decision at the START of a step, then apply
      // acceleration for the ENTIRE step. So we need to check: "if I accelerate
      // this step, can I still stop in time?"
      // After accelerating: v' = v + a*dt, dx' = dx - v*dt - 0.5*a*dt²
      // Can stop if: dx' >= v'²/(2a)
      // Solving: dx >= v²/(2a) + 2*v*dt + a*dt²
      const float base_decel_distance = (v_frame * v_frame) / (2.0f * a);
      const float step_adjustment = 2.0f * std::abs(v_frame) * dt + a * dt * dt;
      const float decel_distance = base_decel_distance + step_adjustment;
      const bool should_accelerate = std::abs(dx) >= decel_distance;

      const bool was_decelerating_towards_target =
          (prev_accel != 0.0f) && (prev_accel * v_frame < 0.0f);

      // Also consider deceleration when velocity is very near zero but we were
      // just decelerating. This handles the case where v_frame crosses zero.
      const float velocity_zero_threshold = a * dt * 2.0f;
      const bool velocity_near_zero = std::abs(v_frame) < velocity_zero_threshold;
      const bool was_decelerating = (prev_accel != 0.0f) &&
          (std::copysign(1.0f, prev_accel) != std::copysign(1.0f, v_frame + prev_accel * dt));

      // Hysteresis: once decelerating towards target, continue until
      // overshoot or arrival to prevent floating-point oscillation.
      if (was_decelerating_towards_target ||
          (velocity_near_zero && was_decelerating)) {
        return std::copysign(a, -v_frame);
      }

      if (should_accelerate) {
        if (std::isnan(data->velocity_limit) ||
            std::abs(v0) < data->velocity_limit) {
          return std::copysign(a, dx);
        } else {
          return 0.0f;
        }
      } else {
        return std::copysign(a, -v_frame);
      }
    }

    // Moving away from target.
    return std::copysign(a, -v_frame);
  }

  static void DoVelocityAndAccelLimits(
      BldcServoStatus* status,
      const BldcServoConfig* config,
      float rate_hz,
      BldcServoCommandData* data,
      float velocity) MOTEUS_CCM_ATTRIBUTE {
    const float period_s = 1.0f / rate_hz;

    // This is the most general case.  We decide whether to
    // accelerate, remain constant, or decelerate, then advance the
    // control velocity in an appropriate manner, and finally check
    // for trajectory completion.

    const float a = data->accel_limit;

    const float v0 = *status->control_velocity;
    const float vf = velocity;

    // What is the delta between our current control state and the
    // command.
    const float dx = MotorPosition::IntToFloat(
        (*data->position_relative_raw - *status->control_position_raw));

    if (std::isnan(data->accel_limit)) {
      // We only have a velocity limit, not an acceleration limit.
      DoVelocityOnlyLimit(
          status, dx, data, velocity, period_s);
      return;
    }

    const float acceleration = CalculateAcceleration(
        status->control_acceleration, data, a, v0, vf, dx, period_s);

    status->control_acceleration = acceleration;
    *status->control_velocity += acceleration * period_s;
    const float v1 = *status->control_velocity;

    const float vel_lower = std::min(std::abs(v0), std::abs(v1));
    const float vel_upper = std::max(std::abs(v0), std::abs(v1));

    // If this velocity would exceed the velocity limit, or pass
    // through it while decelerating, make sure we have at least one
    // cycle exactly at the velocity limit so we will properly enter
    // the "cruise" phase.
    if (std::isfinite(data->velocity_limit) &&
        vel_lower < data->velocity_limit &&
        vel_upper > data->velocity_limit) {
      status->control_acceleration = 0.0f;
      status->control_velocity = std::copysign(data->velocity_limit, v0);
    }

    // Compute position step with exact integration for completion check.
    // This predicts where we'll actually be after UpdateCommand integrates.
    const float step_exact = (v0 + v1) * 0.5f * period_s;
    const float dx_after_step = dx - step_exact;

    // Check for position overshoot: position crosses target this step.
    // With exact integration, we can detect this precisely and complete
    // immediately to avoid oscillation around the target.
    const bool would_overshoot = (dx > 0.0f && dx_after_step < 0.0f) ||
                                  (dx < 0.0f && dx_after_step > 0.0f);

    // Check if we're decelerating towards target.
    const bool decelerating_towards_target =
        (acceleration != 0.0f) && (acceleration * (v0 - vf) < 0.0f);

    // Check if velocity is close to target (within a few timesteps).
    const bool velocity_close_to_target =
        std::abs(v1 - vf) < (a * period_s * 5.0f);

    // Complete immediately on overshoot while decelerating.
    if (would_overshoot && decelerating_towards_target && velocity_close_to_target) {
      data->position = std::numeric_limits<float>::quiet_NaN();
      data->position_relative_raw.reset();
      status->control_acceleration = 0.0f;
      status->control_velocity = vf;
      status->trajectory_done = true;
      return;
    }

    const float signed_vel_lower = std::min(v0, v1);
    const float signed_vel_upper = std::max(v0, v1);

    // Did we span the target velocity this cycle?
    const bool target_cross = signed_vel_lower <= vf && signed_vel_upper >= vf;
    const bool target_near = std::abs(v1 - vf) < (a * 0.5f * period_s);

    // Check position using predicted position after the step.
    // The threshold accounts for discrete-time trajectory planning accuracy.
    // Use velocity-based threshold at high velocity, accel-based at low velocity.
    const float velocity_based_threshold =
        std::abs(v0) * period_s * 3.0f + std::abs(v1) * period_s * 3.0f;
    // At low velocity, position threshold is ~11 timesteps of acceleration
    // (distance = 0.5*a*t² means t = sqrt(2*60)*dt ≈ 11*dt for threshold 60*a*dt²).
    const float accel_based_threshold = a * period_s * period_s * 60.0f;
    const float pos_threshold = std::max(velocity_based_threshold,
                                          accel_based_threshold);

    // Use prediction-based early completion to avoid oscillation at trajectory
    // end. The prediction estimates that if we oscillate once more from current
    // position error, we'll get closer to target. If the predicted final error
    // is small, complete now instead of oscillating.
    if (target_cross && decelerating_towards_target) {
      const float v_peak = std::sqrt(2.0f * std::abs(dx_after_step) * a);
      const float oscillation_velocity_threshold = a * period_s * 15.0f;
      if (v_peak < oscillation_velocity_threshold) {
        const float predicted_final_dx =
            2.0f * v_peak * period_s + a * period_s * period_s;
        // Complete if predicted error after oscillation is within threshold.
        if (predicted_final_dx <= pos_threshold) {
          data->position = std::numeric_limits<float>::quiet_NaN();
          data->position_relative_raw.reset();
          status->control_acceleration = 0.0f;
          status->control_velocity = vf;
          status->trajectory_done = true;
          return;
        }
      }
    }

    const bool position_near = std::abs(dx_after_step) <= pos_threshold;

    if ((target_cross || target_near) && position_near) {
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

    // This limits our usable velocity to 20kHz modulo the position
    // scale at a 40kHz switching frequency.  1.2 million RPM should
    // be enough for anybody?
    //
    // Use exact kinematic integration when accelerating:
    // (v_before + v_after) / 2 * dt = v0*dt + 0.5*a*dt² for constant accel.
    // When not accelerating (velocity set directly), use final velocity.
    const float v_after = velocity_command;
    const bool is_accelerating = (status->control_acceleration != 0.0f);
    const float step = is_accelerating ?
        (v_before + v_after) * 0.5f / rate_hz :
        v_after / rate_hz;
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
