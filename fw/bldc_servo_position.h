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
  // Adaptive decel threshold: use computed required_decel when it's
  // within this fraction of max accel (e.g., 0.9 = 90-100% of max).
  static constexpr float kAdaptiveDecelThreshold = 0.9f;

  // Gap region multiplier: derived from kAdaptiveDecelThreshold (T) via
  // u_max = (T + sqrt(T)) / (1 - T), where u = v/(a*dt) at switch point.
  // The gap region covers u in [~4, u_max] where adaptive decel doesn't
  // engage. Multiplier = u_max² with margin.
  // For T=0.9: u_max ≈ 18.5, u_max² ≈ 342, with ~17% margin ≈ 400.
  static constexpr float kGapRegionMultiplier = 400.0f;

  static void DoVelocityModeLimits(
      BldcServoStatus* status,
      const BldcServoConfig* config,
      float period_s,
      BldcServoCommandData* data,
      float velocity) MOTEUS_CCM_ATTRIBUTE {

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

  // Compute deceleration needed to reach target position with target velocity.
  // Returns the required deceleration magnitude (always positive).
  // v_abs: absolute velocity relative to target
  // inv_2dx: precomputed 1.0f / (2.0f * dx_abs) for efficiency
  //
  // Formula derivation:
  // After one step at deceleration a_d:
  //   v1 = v_frame - a_d * dt  (in target frame, so slowing toward 0)
  //   dx_step = (v_frame + v1) / 2 * dt = v_frame * dt - 0.5 * a_d * dt²
  //   dx_remaining = dx - dx_step
  //
  // For exact convergence, we want dx_remaining = v1² / (2 * a_d):
  //   dx - v_frame * dt + 0.5 * a_d * dt² = (v_frame - a_d * dt)² / (2 * a_d)
  //
  // Let u = v_frame, d = dx, t = dt, a = a_d:
  //   d - u*t + 0.5*a*t² = (u - a*t)² / (2*a)
  //   2*a*(d - u*t + 0.5*a*t²) = (u - a*t)²
  //   2*a*d - 2*a*u*t + a²*t² = u² - 2*u*a*t + a²*t²
  //   2*a*d = u²
  //   a = u² / (2*d) = v² / (2*dx)
  //
  // This is the standard kinematic formula!
  static float ComputeRequiredDecel(float v_abs, float inv_2dx) {
    return (v_abs * v_abs) * inv_2dx;
  }

  static float CalculateAcceleration(
      BldcServoCommandData* data,
      float a,
      float v0,
      float vf,
      float dx,
      float dt,
      float gap_threshold) MOTEUS_CCM_ATTRIBUTE {
    // This logic is broken out primarily so that early-return can be
    // used as a control flow mechanism to aid factorization.

    const float v0_abs = std::abs(v0);

    // If we are overspeed, we always slow down to the velocity
    // limit first.
    if (std::isfinite(data->velocity_limit) &&
        v0_abs > data->velocity_limit) {
      return std::copysign(a, -v0);
    }

    // Perform all operations in the target reference frame,
    // i.e. we'll transform our frame so that the target velocity is
    // 0.

    const auto v_frame = v0 - vf;
    const float v_frame_abs = std::abs(v_frame);

    if ((v_frame * dx) >= 0.0f && dx != 0.0f) {
      // We are moving towards the target (in the target frame).
      const float inv_2a = 1.0f / (2.0f * a);
      const float stop_distance = (v_frame * v_frame) * inv_2a;
      const float dx_abs = std::abs(dx);
      // Precompute reciprocal for ComputeRequiredDecel calls.
      const float inv_2dx = 1.0f / (2.0f * dx_abs);
      // Precompute threshold for adaptive decel check.
      const float threshold_a = kAdaptiveDecelThreshold * a;

      if (dx_abs > stop_distance) {
        // Not yet at switch point - accelerate (unless at velocity limit).
        //
        // Check if we should switch early: if next step would overshoot
        // the ideal switch point, switch now.
        const float v_next = v_frame_abs + a * dt;
        const float dx_step = (v_frame_abs + v_next) * 0.5f * dt;
        const float dx_after = dx_abs - dx_step;
        const float stop_distance_next = (v_next * v_next) * inv_2a;

        if (dx_after < stop_distance_next) {
          // Switching early: compute exact decel to reach target.
          const float required_decel = ComputeRequiredDecel(v_frame_abs, inv_2dx);
          // Use exact decel if within threshold range, otherwise use max.
          if (required_decel >= threshold_a && required_decel <= a) {
            return std::copysign(required_decel, -v_frame);
          }
          return std::copysign(a, -v_frame);
        }

        // Gap region check: if close to target, don't accelerate - this
        // would cause oscillation. Instead, coast or gently decel.
        if (dx_abs < gap_threshold && v_frame_abs > 0.0f) {
          // Compute decel needed to stop at target
          const float required_decel = ComputeRequiredDecel(v_frame_abs, inv_2dx);
          if (required_decel <= a) {
            return std::copysign(required_decel, -v_frame);
          }
          return std::copysign(a, -v_frame);
        }

        if (std::isnan(data->velocity_limit) ||
            v0_abs < data->velocity_limit) {
          return std::copysign(a, dx);
        } else {
          return 0.0f;  // At velocity limit - cruise
        }
      } else {
        // Past switch point - decelerate.
        // Compute exact deceleration to reach target.
        const float required_decel = ComputeRequiredDecel(v_frame_abs, inv_2dx);
        // Use exact decel if within threshold range, otherwise use max.
        if (required_decel >= threshold_a && required_decel <= a) {
          return std::copysign(required_decel, -v_frame);
        }
        return std::copysign(a, -v_frame);
      }
    }

    // We are moving away.  Try to fix that.
    return std::copysign(a, -v_frame);
  }

  static void DoVelocityAndAccelLimits(
      BldcServoStatus* status,
      const BldcServoConfig* config,
      float period_s,
      BldcServoCommandData* data,
      float velocity) MOTEUS_CCM_ATTRIBUTE {

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

    // Gap region threshold: when position is within this distance,
    // prevent re-acceleration oscillation by using reduced acceleration
    // or completing.
    const float gap_threshold = kGapRegionMultiplier * a * period_s * period_s;

    const float acceleration = CalculateAcceleration(
        data, a, v0, vf, dx, period_s, gap_threshold);

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

    const float v1_final = *status->control_velocity;

    const float signed_vel_lower = std::min(v0, v1_final);
    const float signed_vel_upper = std::max(v0, v1_final);

    // Precompute values used multiple times below.
    const float dx_abs = std::abs(dx);
    const float v_frame_final = v1_final - vf;
    const float v_frame_final_abs = std::abs(v_frame_final);

    // Check for completion:
    // 1. Velocity crossed or reached target velocity
    // 2. Position is close enough to target
    const bool target_cross = signed_vel_lower <= vf && signed_vel_upper >= vf;
    const bool target_near = v_frame_final_abs < (a * 0.5f * period_s);

    // Position check: use time-to-target based on final velocity.
    // Gap region: when dx is within gap_threshold, complete to prevent oscillation.
    const bool in_gap_region = dx_abs < gap_threshold;

    // Check if we're heading toward the target (closing gap).
    // Use velocity in target frame (v1_final - vf) for non-zero target velocity.
    // Same sign check: both positive or both negative means we're closing.
    const bool closing = (dx * v_frame_final) > 0.0f;

    // Position check: within 10 timesteps at effective velocity.
    // Use closing rate when it's larger than target velocity, otherwise use
    // target velocity as the minimum. This ensures:
    // 1. During active catch-up, closing rate determines threshold
    // 2. When at target velocity (closing rate ~ 0), use target velocity
    //    which gives a reasonable completion threshold for the PID to handle
    const float v_for_threshold = std::max(v_frame_final_abs, std::abs(vf));
    const bool position_near = (dx_abs <= v_for_threshold * 10.0f * period_s);

    // Complete if:
    // 1. Normal completion: velocity reached target AND position is near
    // 2. Gap region completion: close to target, moving toward it, in decel
    //    phase, and velocity is significantly below the bang-bang curve.
    //    The curve is v = sqrt(2*a*dx), the velocity at distance dx from
    //    target on an ideal trajectory. During normal discrete decel,
    //    velocity stays close to this curve. When we've overshot (decel'd
    //    too much relative to position), velocity drops well below the curve.
    //    Using 60% threshold to catch significant overshoot while avoiding
    //    false positives during normal decel.
    //    Check: v < 0.6 * sqrt(2*a*|dx|)
    //    Squared (to avoid sqrt): v² < 0.36 * 2 * a * |dx| = 0.72 * a * |dx|
    const bool below_curve = (v_frame_final_abs * v_frame_final_abs) < (0.72f * a * dx_abs);
    const bool decelerating = (acceleration * v_frame_final) < 0.0f;
    const bool gap_complete = in_gap_region && closing && below_curve && decelerating;

    if (((target_cross || target_near) && position_near) || gap_complete) {
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
      float period_s,
      BldcServoCommandData* data,
      float velocity) MOTEUS_CCM_ATTRIBUTE {
    // Clamp the desired velocity to our limit if we have one.
    if (!std::isnan(data->velocity_limit)) {
      if (velocity > data->velocity_limit) { velocity = data->velocity_limit; }
      if (velocity < -data->velocity_limit) { velocity = -data->velocity_limit; }
    }

    if (!data->position_relative_raw) {
      DoVelocityModeLimits(
          status, config, period_s, data, velocity);
    } else {
      DoVelocityAndAccelLimits(
          status, config, period_s, data, velocity);
    }
  }

  static float UpdateCommand(
      BldcServoStatus* status,
      const BldcServoConfig* config,
      const BldcServoPositionConfig* position_config,
      const MotorPosition::Status* position,
      int64_t absolute_relative_delta,
      float period_s,
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

    // Capture velocity before trajectory update for exact kinematic integration.
    const float v0 = status->control_velocity.value_or(0.0f);

    if (!status->trajectory_done) {
      UpdateTrajectory(status, config, period_s, data, velocity);
    }

    // Velocity after trajectory update, before max velocity clamp.
    const float v1 = status->control_velocity.value_or(0.0f);

    if (*status->control_velocity > status->motor_max_velocity) {
      status->control_acceleration = 0.0f;
      status->control_velocity = status->motor_max_velocity;
    } else if (*status->control_velocity < -status->motor_max_velocity) {
      status->control_acceleration = 0.0f;
      status->control_velocity = -status->motor_max_velocity;
    }

    auto velocity_command = *status->control_velocity;

    // Compute position step.
    // Use exact kinematic integration (average velocity) when ramping.
    // Use final velocity when velocity jumps instantly (velocity-only mode).
    // This limits our usable velocity to 20kHz modulo the position
    // scale at a 40kHz switching frequency.  1.2 million RPM should
    // be enough for anybody?
    const float step = [&]() {
      // When acceleration is non-zero, velocity is ramping - use average
      // for exact kinematic integration.
      if (status->control_acceleration != 0.0f) {
        return (v0 + v1) * 0.5f * period_s;
      }
      // When acceleration is zero (velocity-only mode or at limit),
      // use final velocity for the full timestep.
      return velocity_command * period_s;
    }();
    const int64_t int64_step =
        (static_cast<int64_t>(
            static_cast<int32_t>((static_cast<float>(1ll << 32) * step))) <<
         16);
    status->control_position_raw = *status->control_position_raw + int64_step;

    if (data->position_relative_raw && !std::isnan(velocity)) {
      const float tstep = velocity * period_s;
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
