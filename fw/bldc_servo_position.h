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
  // Jerk limited trajectories.
  //
  // When a jerk limit is active, control_acceleration is only ever
  // changed by slewing it at up to `jerk` toward a target
  // acceleration.  That target is selected anew each cycle from
  // nothing but the current position, velocity, and acceleration by
  // a planner:
  //
  //  * VelocityTargetAcceleration brings (v, a) to (v_des, 0) in
  //    minimum time.
  //  * DoJerkLimited, given a position, has that head toward it at
  //    the velocity limit until stopping any later would overshoot,
  //    and then has it stop.
  //
  // Because no plan is carried from cycle to cycle, a change of
  // target or limits at any point needs no special handling.
  //
  // The target may itself be accelerating, in which case all of the
  // above is relative to it.

  // Commanded positions, and the measured distance to them, are
  // quantized to this resolution.
  static constexpr float kJerkPositionQuantum = 1.0f / 65536.0f;

  // No attempt is made to get closer to the target than the
  // deadband.  A trajectory is complete when it is at rest within
  // the tolerance, at which point it is set to exactly the target.
  //
  // These are sized so that a target which alternates between
  // adjacent quantized values neither starts a new move, nor
  // prevents one from completing.
  static constexpr float kJerkPositionDeadband = kJerkPositionQuantum;
  static constexpr float kJerkPositionTolerance = 3.0f * kJerkPositionQuantum;

  // Begin stopping early by this fraction of the stopping distance.
  // A stop uses all of the available jerk and acceleration, so
  // without margin there would be no means of correcting for
  // accumulated floating point error, and the result would be
  // overshoot.
  static constexpr float kJerkStopMargin = 0.005f;

  struct JerkLimits {
    float accel = 0.0f;  // may be +infinity
    float jerk = 0.0f;
    float inv_jerk = 0.0f;
    float dt = 0.0f;
    float jerk_dt = 0.0f;
    float jerk_dt2 = 0.0f;
  };

  struct JerkState {
    float v = 0.0f;
    float a = 0.0f;
  };

  struct JerkStop {
    // The signed distance covered while coming to rest.
    float dx = 0.0f;

    // The portion of that covered while the acceleration makes its
    // final return to zero.  Nothing can be done during it to
    // lengthen or shorten the stop.
    float final_dx = 0.0f;
  };

  // The limit must not be infinite, which is ensured when the
  // command is prepared.
  static bool JerkLimited(const BldcServoCommandData* data) {
    return data->jerk_limit > 0.0f;
  }

  static JerkLimits MakeJerkLimits(
      const BldcServoCommandData* data, float period_s) MOTEUS_CCM_ATTRIBUTE {
    JerkLimits result;
    // With no acceleration limit, acceleration is constrained only
    // by the rate at which it can be slewed.
    result.accel = std::isnan(data->accel_limit) ?
        std::numeric_limits<float>::infinity() : data->accel_limit;
    result.jerk = data->jerk_limit;
    result.inv_jerk = 1.0f / data->jerk_limit;
    result.dt = period_s;
    result.jerk_dt = data->jerk_limit * period_s;
    result.jerk_dt2 = result.jerk_dt * period_s;
    return result;
  }

  // Move `a` toward `target_a` by no more than `da_limit`.
  static float SlewAcceleration(
      float a, float target_a, float da_limit) MOTEUS_CCM_ATTRIBUTE {
    const float da = target_a - a;
    if (da > da_limit) { return a + da_limit; }
    if (da < -da_limit) { return a - da_limit; }
    return target_a;
  }

  // Advance one cycle while slewing toward `target_a`.  Jerk is
  // constant within a cycle, so the mean acceleration integrates
  // velocity exactly.
  static JerkState AdvanceJerk(
      const JerkState& state, float target_a,
      const JerkLimits& limits) MOTEUS_CCM_ATTRIBUTE {
    JerkState result;
    result.a = SlewAcceleration(state.a, target_a, limits.jerk_dt);
    result.v = state.v + (state.a + result.a) * 0.5f * limits.dt;
    return result;
  }

  // The velocity that would remain to reach `v_des`, which is itself
  // changing at `a_des`, if the acceleration were slewed to `a_des`
  // starting now.
  static float JerkVelocityRemaining(
      const JerkState& state, float v_des, float a_des,
      const JerkLimits& limits) MOTEUS_CCM_ATTRIBUTE {
    const float da = state.a - a_des;
    return v_des - state.v - da * std::abs(da) * (0.5f * limits.inv_jerk);
  }

  // True if a velocity error is no more than `scale` times what can
  // be resolved near `v_des`.  That is the largest of:
  //
  //  * four times the change in velocity from one cycle of jerk
  //  * sixteen times the resolution of a float
  //  * the velocity covered while slewing back to zero the smallest
  //    acceleration which changes a float velocity at all in one
  //    cycle, as anything less has no effect
  //
  // The last is formulated so as to not require a division.
  static bool JerkVelocityNegligible(
      float error, float v_des, float scale,
      const JerkLimits& limits) MOTEUS_CCM_ATTRIBUTE {
    const float scaled = std::abs(error) * (0.25f / scale);
    const float resolution =
        std::numeric_limits<float>::epsilon() * std::abs(v_des);
    return scaled <= std::max(limits.jerk_dt2, 4.0f * resolution) ||
        scaled * limits.jerk_dt2 <= resolution * resolution;
  }

  // True if `state` is indistinguishable from (v_des, a_des), and
  // the acceleration can be brought to exactly `a_des` within one
  // cycle.
  static bool JerkAtVelocity(
      const JerkState& state, float v_des, float a_des,
      const JerkLimits& limits) MOTEUS_CCM_ATTRIBUTE {
    return std::abs(state.a - a_des) <= limits.jerk_dt &&
        JerkVelocityNegligible(v_des - state.v, v_des, 1.0f, limits);
  }

  // Bring (v, a) to (v_des, a_des), where `v_des` is itself changing
  // at `a_des`.
  static float VelocityTargetAcceleration(
      const JerkState& state, float v_des, float a_des,
      const JerkLimits& limits) MOTEUS_CCM_ATTRIBUTE {
    // Accelerate at the limit toward `v_des` until slewing the
    // acceleration back to `a_des` would cover what remains.
    const float remaining =
        JerkVelocityRemaining(state, v_des, a_des, limits);
    if (JerkVelocityNegligible(remaining, v_des, 0.5f, limits)) {
      return a_des;
    }

    const float target_a = std::copysign(limits.accel, remaining);
    const auto next = AdvanceJerk(state, target_a, limits);
    const float next_remaining = JerkVelocityRemaining(
        next, v_des + a_des * limits.dt, a_des, limits);
    if ((next_remaining > 0.0f) == (remaining > 0.0f)) { return target_a; }

    // Another full cycle would overshoot.  Slewing to `a_des`
    // instead leaves `remaining` unchanged, so interpolate between
    // the two to land on the switching curve.
    const float coast_a = SlewAcceleration(state.a, a_des, limits.jerk_dt);
    return next.a + (coast_a - next.a) *
        next_remaining / (next_remaining - remaining);
  }

  // Bring `state` to rest (v = 0, a = 0) in minimum time.  This is
  // valid for any state, including those where the velocity must
  // reverse before coming to rest.
  //
  // `state` is relative to a target which is accelerating at
  // `target_a`, which leaves less acceleration available in one
  // direction than the other.
  static JerkStop CalculateJerkStop(
      const JerkState& state, float target_a,
      const JerkLimits& limits) MOTEUS_CCM_ATTRIBUTE {
    const float inv_2j = 0.5f * limits.inv_jerk;

    // Mirror the problem so that a net deceleration is required.
    // The profile is then: ramp the acceleration down to -peak,
    // optionally hold there, then ramp back up to zero.
    const float sign =
        (state.v + state.a * std::abs(state.a) * inv_2j) < 0.0f ?
        -1.0f : 1.0f;
    const float v = state.v * sign;
    const float a = state.a * sign;

    // The peak that brings velocity to zero with no hold phase
    // satisfies 2 * peak^2 = a^2 + 2 * j * v.
    const float unlimited_peak_sq =
        std::max(0.0f, 0.5f * a * a + limits.jerk * v);
    // A target accelerating at the limit leaves nothing, in which
    // case assume the least that could be distinguished from that.
    const float peak_limit =
        std::max(limits.accel + sign * target_a, limits.jerk_dt);
    const bool hold = unlimited_peak_sq > peak_limit * peak_limit;
    const float peak = hold ? peak_limit : sqrtf(unlimited_peak_sq);

    // Ramp down for t0, starting at (v, a) and ending at (v1, -peak).
    const float t0 = (a + peak) * limits.inv_jerk;
    const float x0 =
        t0 * (v + t0 * (a * (1.0f / 3.0f) - peak * (1.0f / 6.0f)));
    const float v1 = v + (a * a - peak * peak) * inv_2j;

    // Ramp up for t2, starting at (v2, -peak) and ending at rest.
    const float t2 = peak * limits.inv_jerk;
    const float x2 = peak * t2 * t2 * (1.0f / 6.0f);

    // Hold at -peak between them if necessary.
    const float v2 = peak * peak * inv_2j;
    const float x1 = hold ? (v1 * v1 - v2 * v2) / (2.0f * peak) : 0.0f;

    JerkStop result;
    result.dx = sign * (x0 + x1 + x2);
    result.final_dx = sign * x2;
    return result;
  }

  // The acceleration of the commanded velocity, which is zero once
  // it has reached the velocity limit.
  static float TargetAcceleration(
      const BldcServoCommandData* data,
      float velocity) MOTEUS_CCM_ATTRIBUTE {
    const float a = data->acceleration;
    if (a == 0.0f) { return 0.0f; }
    const float outward = (a < 0.0f) ? -velocity : velocity;
    return (outward >= data->velocity_limit) ? 0.0f : a;
  }

  // Having reached a target moving at `velocity`, move with it.
  static void FollowTarget(
      BldcServoStatus* status, float period_s,
      const BldcServoCommandData* data,
      float velocity) MOTEUS_CCM_ATTRIBUTE {
    // This cycle ends where the commanded velocity will then be.
    const float acceleration = TargetAcceleration(data, velocity);
    status->control_acceleration = acceleration;
    status->control_velocity = velocity + acceleration * period_s;
    status->trajectory_done = true;
  }

  // True if, `dx` from a target which is itself moving at `vf` and
  // accelerating at `af`, there is room to keep heading toward it.
  static bool JerkRoomToGo(
      const JerkState& state, float vf, float af, float dx,
      const JerkLimits& limits) MOTEUS_CCM_ATTRIBUTE {
    const float dir = (dx < 0.0f) ? -1.0f : 1.0f;

    // There is if, after one more cycle of accelerating toward the
    // target, we could still stop short of it with some margin.
    const auto next = AdvanceJerk(state, dir * limits.accel, limits);
    const float next_vf = vf + af * limits.dt;
    const float next_dx =
        dx - ((state.v + next.v) - (vf + next_vf)) * 0.5f * limits.dt;
    JerkState next_relative;
    next_relative.v = next.v - next_vf;
    next_relative.a = next.a - af;
    const auto stop = CalculateJerkStop(next_relative, af, limits);
    const float margin =
        kJerkPositionDeadband +
        kJerkStopMargin * std::abs(stop.dx - stop.final_dx);
    return (next_dx - stop.dx) * dir > margin;
  }

  // Advance a jerk limited trajectory by one cycle.  `dx` is the
  // distance to the target position, or nullptr if there is none
  // and only `velocity` is to be reached.
  //
  // This is out of line so that there is only one copy.
  static void DoJerkLimited(
      BldcServoStatus* status,
      float period_s,
      BldcServoCommandData* data,
      float velocity,
      const float* dx) MOTEUS_CCM_NOINLINE_ATTRIBUTE {
    const auto limits = MakeJerkLimits(data, period_s);
    JerkState state;
    state.v = *status->control_velocity;
    state.a = status->control_acceleration;

    // The target is the commanded velocity, which may itself be
    // accelerating.
    const float acceleration = TargetAcceleration(data, velocity);
    float v_des = velocity;
    float a_des = acceleration;
    if (acceleration != 0.0f) {
      // It will stop accelerating upon reaching the velocity limit.
      // Something following it has to begin doing so sooner, so as
      // to arrive at the limit with no acceleration.
      const float sign = (acceleration < 0.0f) ? -1.0f : 1.0f;
      const float until_limit = data->velocity_limit - sign * velocity;
      if (until_limit * 2.0f * limits.jerk <= acceleration * acceleration) {
        v_des = sign * data->velocity_limit;
        a_des = 0.0f;
      }
    }

    if (JerkAtVelocity(state, v_des, a_des, limits) &&
        (!dx || std::abs(*dx) <= kJerkPositionTolerance)) {
      if (dx) {
        status->control_position_raw = *data->position_relative_raw;
        data->position = std::numeric_limits<float>::quiet_NaN();
        data->position_relative_raw.reset();
      }
      // This cycle ends where the target will then be.
      status->control_acceleration = a_des;
      status->control_velocity = v_des + a_des * limits.dt;
      status->trajectory_done = true;
      return;
    }

    // With a position, head toward it at the velocity limit for as
    // long as there is room, and then match the target.  If that
    // overshoots, we head back once it can be recovered.
    bool unlimited = false;
    if (dx && JerkRoomToGo(state, velocity, acceleration, *dx, limits)) {
      const float dir = (*dx < 0.0f) ? -1.0f : 1.0f;
      unlimited = std::isnan(data->velocity_limit);
      v_des = unlimited ? dir : dir * data->velocity_limit;
      a_des = 0.0f;
    }

    const auto next = AdvanceJerk(
        state,
        unlimited ? v_des * limits.accel :
        VelocityTargetAcceleration(state, v_des, a_des, limits),
        limits);
    status->control_acceleration = next.a;
    status->control_velocity = next.v;
  }

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

    if (JerkLimited(data)) {
      DoJerkLimited(status, period_s, data, velocity, nullptr);
      return;
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
        FollowTarget(status, period_s, data, velocity);
      }
    } else {
      // We must have only a velocity limit.  This is easy.
      FollowTarget(status, period_s, data, velocity);
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
      FollowTarget(status, period_s, data, velocity);
    }
  }

  // Compute deceleration needed to reach target position starting
  // from a given velocity.
  //
  // v_abs: absolute velocity relative to target
  // inv_2dx: precomputed 1.0f / (2.0f * dx_abs) for efficiency
  static float ComputeRequiredDecel(float v_abs, float inv_2dx) {
    return (v_abs * v_abs) * inv_2dx;
  }

  static float CalculateAcceleration(
      BldcServoCommandData* data,
      float a,
      float v0,
      float vf,
      float dx,
      float dt) MOTEUS_CCM_ATTRIBUTE {
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

      if (dx_abs > stop_distance) {
        // We have not yet reached the point of needing to decelerate,
        // which would normally mean accelerating.

        // However, check if we should switch early: if next step
        // would overshoot the ideal switch point, switch now.
        const float v_next = v_frame_abs + a * dt;
        const float dx_step = (v_frame_abs + v_next) * 0.5f * dt;
        const float dx_after = dx_abs - dx_step;
        const float stop_distance_next = (v_next * v_next) * inv_2a;

        if (dx_after < stop_distance_next) {
          // We would switch in the middle of the next cycle, so
          // instead start decelerating now.
          const float required_decel = ComputeRequiredDecel(v_frame_abs, inv_2dx);

          if (required_decel > a) {
            // This really shouldn't happen, but if it does, limit our
            // deceleration to the intended limit.
            return std::copysign(a, -v_frame);
          } else {
            return std::copysign(required_decel, -v_frame);
          }
        }

        // With those checks out of the way, we should be good to
        // accelerate now.
        if (std::isnan(data->velocity_limit) ||
            v0_abs < data->velocity_limit) {
          return std::copysign(a, dx);
        } else {
          return 0.0f;  // At velocity limit - cruise
        }
      } else {
        // We are in the region where we should be decelerating.
        // Decelerate as much as necessary but no more than our limit.

        const float required_decel =
            ComputeRequiredDecel(v_frame_abs, inv_2dx);
        if (required_decel > a) {
          return std::copysign(a, -v_frame);
        } else {
          return std::copysign(required_decel, -v_frame);
        }
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
        MotorPosition::WrappingSub(
            *data->position_relative_raw, *status->control_position_raw));

    if (JerkLimited(data)) {
      DoJerkLimited(status, period_s, data, velocity, &dx);
      return;
    }

    if (std::isnan(data->accel_limit)) {
      // We only have a velocity limit, not an acceleration limit.
      DoVelocityOnlyLimit(
          status, dx, data, velocity, period_s);
      return;
    }

    const float acceleration = CalculateAcceleration(
        data, a, v0, vf, dx, period_s);

    status->control_acceleration = acceleration;
    *status->control_velocity += acceleration * period_s;
    const float v1 = *status->control_velocity;

    const float vel_lower = std::min(std::abs(v0), std::abs(v1));
    const float vel_upper = std::max(std::abs(v0), std::abs(v1));

    // If this velocity would exceed the velocity limit, or pass
    // through it while decelerating, make sure we have at least one
    // cycle exactly at the velocity limit so we will properly enter
    // the "cruise" phase.  The cruise direction is the direction the
    // controller is heading (v1); v0 may be zero, or even oppositely
    // signed if a single step crossed zero.
    if (std::isfinite(data->velocity_limit) &&
        vel_lower < data->velocity_limit &&
        vel_upper > data->velocity_limit) {
      status->control_acceleration = 0.0f;
      status->control_velocity = std::copysign(data->velocity_limit, v1);
    }

    const float v1_final = *status->control_velocity;

    const float signed_vel_lower = std::min(v0, v1_final);
    const float signed_vel_upper = std::max(v0, v1_final);

    // Precompute values that will be used multiple times below.
    const float dx_abs = std::abs(dx);
    const float v_frame_final = v1_final - vf;
    const float v_frame_final_abs = std::abs(v_frame_final);

    // We will consider this trajectory completed if both:
    // 1. The velocity crossed or reached target velocity
    // 2. The position is close enough to target
    const bool target_cross = signed_vel_lower <= vf && signed_vel_upper >= vf;
    const bool target_near = v_frame_final_abs < (a * 0.5f * period_s);

    // When evaluating velocity "closeness" as the gate for checking
    // position, we use closing rate when it's larger than target
    // velocity.  This ensures we complete even if the current closing
    // rate is small because of numerical precision.
    const float v_for_threshold = std::max(v_frame_final_abs, std::abs(vf));

    // For the position portion, we're done if our position is close
    // enough to complete.  We consider within 10 timesteps as "close
    // enough".
    const bool position_near = (dx_abs <= v_for_threshold * 10.0f * period_s);

    if ((target_cross || target_near) && position_near) {
      data->position = std::numeric_limits<float>::quiet_NaN();
      data->position_relative_raw.reset();
      FollowTarget(status, period_s, data, vf);
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

    const bool unlimited =
        std::isnan(data->velocity_limit) &&
        std::isnan(data->accel_limit) &&
        !JerkLimited(data);

    // Plan to go no faster than the motor is able to, so that
    // velocity is approached, and followed as it changes, within the
    // other limits.
    const float commanded_velocity_limit = data->velocity_limit;
    const bool planned_max_velocity =
        JerkLimited(data) && !data->synthetic_theta;
    if (data->velocity_limit > status->motor_max_velocity) {
      data->velocity_limit = status->motor_max_velocity;
    }

    const float target_acceleration = TargetAcceleration(data, velocity);

    if (unlimited) {
      // This cycle ends where the commanded velocity will then be.
      status->trajectory_done = true;
      status->control_acceleration = target_acceleration;
      status->control_velocity = velocity + target_acceleration * period_s;
    } else if (!!data->position_relative_raw ||
               !std::isnan(velocity)) {
      status->trajectory_done = false;
    }

    if (!!data->position_relative_raw && unlimited) {
      // With no limits, we immediately set the control position and
      // velocity.
      status->control_position_raw = *data->position_relative_raw;
      data->position = std::numeric_limits<float>::quiet_NaN();
      data->position_relative_raw.reset();
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

    // Capture the velocity before we update the trajectory
    const float v0 = status->control_velocity.value_or(0.0f);

    if (!status->trajectory_done) {
      UpdateTrajectory(status, config, period_s, data, velocity);
    }

    // v1 will be the velocity after the trajectory update, before max
    // velocity clamp.
    const float v1 = status->control_velocity.value_or(0.0f);

    // Otherwise, it is imposed regardless of them.
    if (planned_max_velocity) {
    } else if (*status->control_velocity > status->motor_max_velocity) {
      status->control_acceleration = 0.0f;
      status->control_velocity = status->motor_max_velocity;
    } else if (*status->control_velocity < -status->motor_max_velocity) {
      status->control_acceleration = 0.0f;
      status->control_velocity = -status->motor_max_velocity;
    }

    auto velocity_command = *status->control_velocity;

    // Perform our position integration.
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

    // This fixed point formulation limits our usable velocity to
    // 20kHz modulo the position scale at a 40kHz switching frequency.
    // 1.2 million RPM should be enough for anybody?
    const int64_t int64_step =
        (static_cast<int64_t>(
            static_cast<int32_t>((static_cast<float>(1ll << 32) * step))) <<
         16);
    status->control_position_raw =
        MotorPosition::WrappingAdd(*status->control_position_raw, int64_step);

    // How far the target moves this cycle.
    float tstep = velocity * period_s;
    if (target_acceleration != 0.0f) {
      // The commanded velocity changes at the commanded
      // acceleration, until it reaches the velocity limit.
      float next_velocity = velocity + target_acceleration * period_s;
      if (next_velocity > data->velocity_limit) {
        next_velocity = data->velocity_limit;
      } else if (next_velocity < -data->velocity_limit) {
        next_velocity = -data->velocity_limit;
      }
      data->velocity = next_velocity;
      tstep = (velocity + next_velocity) * 0.5f * period_s;
    }

    if (data->position_relative_raw && !std::isnan(velocity)) {
      const int64_t tint64_step =
        (static_cast<int64_t>(
            static_cast<int32_t>((static_cast<float>(1ll << 32) * tstep))) <<
         16);
      data->position_relative_raw =
          MotorPosition::WrappingAdd(
              *data->position_relative_raw, tint64_step);
    }

    if (std::isfinite(config->max_position_slip) && !data->synthetic_theta) {
      const int64_t current_position = position->position_relative_raw;
      const int64_t slip =
          MotorPosition::FloatToInt(config->max_position_slip);

      const int64_t error =
          MotorPosition::WrappingSub(
              current_position, *status->control_position_raw);
      if (error < -slip) {
        *status->control_position_raw =
            MotorPosition::WrappingAdd(current_position, slip);
      }
      if (error > slip) {
        *status->control_position_raw =
            MotorPosition::WrappingSub(current_position, slip);
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
      const auto limit_value =
          MotorPosition::WrappingSub(MotorPosition::FloatToInt(value), delta);
      if (compare(MotorPosition::WrappingSub(
                      *status->control_position_raw, limit_value),
                  0)) {
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
      if (sign(MotorPosition::WrappingSub(
                   *status->control_position_raw,
                   stop_position_raw)) * velocity_command > 0.0f) {
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
            MotorPosition::WrappingAdd(
                *status->control_position_raw, absolute_relative_delta));

    data->velocity_limit = commanded_velocity_limit;

    return velocity_command;
  }
};

}
