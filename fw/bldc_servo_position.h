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
  // Slew the running acceleration `a_curr` toward `target_a`, but no
  // faster than `da_limit` per call.  `da_limit` is the precomputed
  // per-cycle jerk step `j * dt`; callers must ensure it is finite
  // and non-negative.  (The non-jerk-limited path bypasses this
  // function entirely and assigns target_a directly.)
  static float SlewAcceleration(
      float a_curr, float target_a, float da_limit) MOTEUS_CCM_ATTRIBUTE {
    const float da = target_a - a_curr;
    if (da > da_limit) { return a_curr + da_limit; }
    if (da < -da_limit) { return a_curr - da_limit; }
    return target_a;
  }

  // Compute the position-frame distance required to bring (v_t, a_t)
  // to (0, 0) under a jerk-limited 3-phase profile (ramp `a` to
  // -a_max, hold, ramp back to 0).  When the post-ramp velocity is
  // already below the rest-ramp velocity v3 = a_max^2/(2j), a
  // 2-phase (triangle) profile is used instead.
  //
  // All inputs are in the "target frame" with `v_t >= 0` and we are
  // computing the unsigned distance.
  static float ComputeJerkStopDistance(
      float v_t, float a_t, float a_max, float j,
      float inv_j) MOTEUS_CCM_ATTRIBUTE {
    // Phase 0: ramp a from a_t to -a_max with jerk -j.
    //   t0   = (a_t + a_max) / j   (>= 0 for a_t in [-a_max, a_max])
    //   dv_0 = (a_t^2 - a_max^2) / (2j)
    //   dx_0 = v_t*t0 + 0.5*a_t*t0^2 - j*t0^3/6 = t0 * (v_t + a_t*t0/3 + a_max*t0/6)
    //
    // The closed-form rewrite of dx_0 avoids the redundant j-divide that
    // appears when computing j*t0^3/6 directly.  The same trick applies
    // to the triangle path.
    const float a_max_sq = a_max * a_max;
    const float t0 = (a_t + a_max) * inv_j;
    const float v_after_0 = v_t + (a_t * a_t - a_max_sq) * (0.5f * inv_j);
    const float v3 = a_max_sq * (0.5f * inv_j);
    // dx_0 expansion: t0*(v_t) + 0.5*a_t*t0^2 - j*t0*t0*t0/6
    //               = t0*v_t + t0^2*(0.5*a_t - j*t0/6)
    //               = t0*v_t + t0^2*(0.5*a_t - (a_t+a_max)/6)
    //               = t0*v_t + t0^2*(a_t/3 - a_max/6)
    // The last step substitutes j*t0 = a_t + a_max, which only
    // holds for the specific t0 = (a_t + a_max)/j used here (and
    // tA = (a_t + a_peak)/j in the triangle path below) -- it is
    // not a generic identity for an arbitrary `t`.
    const float t0_sq = t0 * t0;
    const float dx_0 = t0 * v_t + t0_sq *
        (a_t * (1.0f / 3.0f) - a_max * (1.0f / 6.0f));

    if (v_after_0 >= v3) {
      // Trapezoidal-in-a profile.
      const float t1 = (v_after_0 - v3) / a_max;
      const float dx_1 = (v_after_0 + v3) * 0.5f * t1;
      // dx_2 = a_max^3 / (6 * j^2) = a_max * (a_max*inv_j)^2 / 6
      const float ratio = a_max * inv_j;
      const float dx_2 = a_max * ratio * ratio * (1.0f / 6.0f);
      return dx_0 + dx_1 + dx_2;
    }

    // Triangular-in-a profile.  Choose peak |a| so that v ends at 0.
    //   2*a_peak^2 = a_t^2 + 2*j*v_t
    const float a_peak_sq = 0.5f * (a_t * a_t + 2.0f * j * v_t);
    if (a_peak_sq <= 0.0f) { return 0.0f; }
    const float a_peak = sqrtf(a_peak_sq);

    const float tA = (a_t + a_peak) * inv_j;
    const float tA_sq = tA * tA;
    // dxA = tA*v_t + tA^2*(a_t/3 - a_peak/6) using the same identity as dx_0.
    const float dxA = tA * v_t + tA_sq *
        (a_t * (1.0f / 3.0f) - a_peak * (1.0f / 6.0f));
    const float v_mid = v_t + (a_t * a_t - a_peak_sq) * (0.5f * inv_j);
    const float tB = a_peak * inv_j;
    const float tB_sq = tB * tB;
    // dxB = tB*v_mid - 0.5*a_peak*tB^2 + j*tB^3/6
    //     = tB*v_mid + tB^2*(-0.5*a_peak + j*tB/6)
    //     = tB*v_mid + tB^2*(-0.5*a_peak + a_peak/6)
    //     = tB*v_mid - tB^2 * a_peak * (1/3)
    // The last step substitutes j*tB = a_peak, which only holds
    // for tB = a_peak/j (the ramp-back duration in the triangle
    // profile).
    const float dxB = tB * v_mid - tB_sq * a_peak * (1.0f / 3.0f);
    return dxA + dxB;
  }

  // Compute the target acceleration to command for a jerk-limited
  // trajectory bringing (v0, a_curr) toward (vf, 0) at signed remaining
  // distance dx, with acceleration and jerk limits a_max, j.
  //
  // The slew limit on `a` is applied by the caller.  `dt` is used
  // to look ahead one cycle when choosing switch instants, which
  // keeps the discrete-time crossings of the accel/decel/rest-ramp
  // boundaries from accumulating overshoot.  `status` is taken so
  // the per-trajectory "commit to rest" latch (which guarantees the
  // terminal slew of `a` to 0 is not interrupted by bang-bang
  // logic) can be set and observed.
  static float CalculateJerkLimitedAcceleration(
      BldcServoStatus* status,
      BldcServoCommandData* data,
      float a_max,
      float j,
      float inv_j,
      float v0,
      float vf,
      float a_curr,
      float dx,
      float dt,
      float j_dt) MOTEUS_CCM_ATTRIBUTE {
    // The latch fast-path: this runs every cycle once we have
    // committed to the terminal slew, so it is the most performance-
    // critical exit point in the function.
    if (status->trajectory_rest_committed) {
      return 0.0f;
    }

    const float v0_abs = std::abs(v0);

    // Overspeed handling: brake down to the velocity limit.  When
    // we have already braked enough that slewing `a` to 0 from the
    // current state lands |v| at or below v_limit, switch to
    // target_a = 0 -- this gives the same kind of smooth rest-curve
    // entry into cruise that the approach-from-below branch later
    // in this function produces.  The check uses the CURRENT a (not
    // a one-cycle lookahead) because the pre-switch case here is
    // different from the position-target one: there, "small v_t,
    // small a_t" is the natural commit point for the end of the
    // trajectory; here, the analogous "tiny excess, a still ~ 0"
    // state would commit prematurely and leave v stuck above
    // v_limit (no braking force applied).  Requiring |a| itself to
    // be on the rest curve avoids this -- with a_curr small the
    // controller falls through to return -a_max and actually starts
    // braking.
    if (std::isfinite(data->velocity_limit) &&
        v0_abs > data->velocity_limit) {
      const float excess = v0_abs - data->velocity_limit;
      if (a_curr * v0 < 0.0f &&
          2.0f * j * excess <= a_curr * a_curr) {
        return 0.0f;
      }
      return std::copysign(a_max, -v0);
    }

    const float v_frame = v0 - vf;

    // Already at (x_target, v_target): commit to the terminal slew
    // so the trajectory_done gate in DoVelocityAndAccelLimits fires
    // this cycle (when a_curr is also 0) or once `a` slews to 0.
    // Without this short-circuit the "not moving toward target"
    // branch below interprets dx == 0 as a degenerate case and
    // launches a phantom +/-a_max trajectory that takes many cycles
    // to come back.  This is the common path for the cycle
    // immediately after a trajectory terminates -- when (a, v, x)
    // have just been forced to (0, vf, xf) -- and for a host that
    // sends position == current_position to mean "stop here".
    if (dx == 0.0f && v_frame == 0.0f) {
      status->trajectory_rest_committed = true;
      status->trajectory_committed_position = data->position;
      status->trajectory_committed_velocity = data->velocity;
      return 0.0f;
    }

    // If we are not moving toward the target, flip direction.
    if (!((v_frame * dx) >= 0.0f && dx != 0.0f)) {
      return std::copysign(a_max, -v_frame);
    }

    // Work in a "direction-positive" frame.
    const float dir = (dx >= 0.0f) ? 1.0f : -1.0f;
    const float v_t = v_frame * dir;
    const float a_t = a_curr * dir;
    const float dx_abs = dx * dir;

    // One-cycle lookahead: predict (v, a) one step into the future
    // assuming we continue accelerating, and compare the resulting
    // stop-distance against the remaining dx.  Switching early like
    // this keeps the discrete crossing from putting us irretrievably
    // past the optimal switch point.
    const float a_accel_next = SlewAcceleration(a_t, a_max, j_dt);
    const float v_accel_next =
        v_t + (a_t + a_accel_next) * 0.5f * dt;
    const float dx_accel_next =
        dx_abs - (v_t + v_accel_next) * 0.5f * dt;
    const float stop_d_after_accel =
        ComputeJerkStopDistance(
            v_accel_next, a_accel_next, a_max, j, inv_j);

    // Strict `>`: at the exact-crossing point (one cycle of accel
    // would land exactly on the rest curve) we route to the brake
    // branch.  That matches the brake-rest-curve commit's `<=`
    // boundary -- the two checks meet exclusively, so a state on
    // the manifold is handled by exactly one branch.
    if (dx_accel_next > stop_d_after_accel) {
      // We have room to accelerate.  Velocity-limit handling:
      //  - If we are already at or past v_limit, coast (target=0).
      //  - If we are accelerating toward v_limit and applying +a_max
      //    one more cycle would put us past the rest curve
      //    |v| = v_limit - a^2/(2j), switch to target=0 now.
      if (std::isfinite(data->velocity_limit)) {
        if (v0_abs >= data->velocity_limit) {
          return 0.0f;
        }
        if (a_curr * v0 > 0.0f) {
          const float a_after = SlewAcceleration(
              a_curr, std::copysign(a_max, v0), j_dt);
          const float v_after = v0 + (a_curr + a_after) * 0.5f * dt;
          const float v_after_abs = std::abs(v_after);
          // Rest-curve threshold expressed without the divide:
          // |v_after| + a_after^2/(2j) >= v_limit
          // <=>  2*j*(v_limit - |v_after|) <= a_after^2
          const float deficit = data->velocity_limit - v_after_abs;
          if (deficit <= 0.0f ||
              2.0f * j * deficit <= a_after * a_after) {
            return 0.0f;
          }
        }
      }
      return dir * a_max;
    }

    // We must brake.  Pre-switch to the terminal slew the cycle
    // before applying -a_max would carry us past the rest curve
    // (v_t <= a_t^2 / (2j) with a_t < 0): the kinematic condition
    // for "applying target_a=0 from now will arrive at (v=vf,
    // x=x_target)".  Latch the decision so the controller stays in
    // the terminal slew through the rest of the trajectory.
    const float a_brake_next = SlewAcceleration(a_t, -a_max, j_dt);
    const float v_brake_next =
        v_t + (a_t + a_brake_next) * 0.5f * dt;
    if (a_brake_next < 0.0f &&
        v_brake_next * 2.0f * j <= a_brake_next * a_brake_next) {
      status->trajectory_rest_committed = true;
      // Snapshot the host's intent so we can detect a mid-slew
      // retarget.  data->position is the raw float the host wrote
      // (PrepareCommand does not mutate it), so bit-exact float
      // compares work for consumers that send the same target every
      // command frame.
      status->trajectory_committed_position = data->position;
      status->trajectory_committed_velocity = data->velocity;
      return 0.0f;
    }

    return dir * -a_max;
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

    // We may have accel or velocity limits here or both, but we don't
    // care about position, only about velocity.  NaN/negative
    // accel_limit + finite jerk_limit is treated as a_max =
    // +infinity (symmetric with how NaN velocity_limit is treated
    // as no velocity cap by the rest of this path).
    const float j = data->jerk_limit;
    const bool jerk_limited = std::isfinite(j) && j > 0.0f;
    if (!std::isnan(data->accel_limit) || jerk_limited) {
      const float a_max = !std::isnan(data->accel_limit) ?
          data->accel_limit :
          std::numeric_limits<float>::infinity();
      const float v_curr = *status->control_velocity;
      const float a_curr = status->control_acceleration;
      const float dv = velocity - v_curr;
      const float initial_sign = (dv > 0.0f) ? 1.0f : -1.0f;

      // Already-at-target early-out: the cycle immediately after a
      // jerk-limited trajectory terminates re-enters this function
      // with (a, v) forced to (0, velocity).  Without this short-
      // circuit the jerk-limited path would launch a new toy
      // trajectory (target_a = -a_max because dv > 0 is false at
      // dv == 0) and oscillate around vf instead of staying put.
      if (jerk_limited &&
          a_curr == 0.0f && v_curr == velocity) {
        status->trajectory_done = true;
        return;
      }

      const float target_a = [&]() {
        if (!jerk_limited) { return a_max * initial_sign; }
        // Latched fast path: stay in the terminal slew.
        if (status->trajectory_rest_committed) { return 0.0f; }
        // Drive toward `velocity` along the (v, a) phase portrait.
        // When `a` already points toward the target velocity and the
        // remaining |dv| is at or below the rest-ramp curve
        // |dv| = a^2/(2j), command zero acceleration and latch so
        // the slew brings `a` back to 0 just as v reaches target.
        const float dv_abs = std::abs(dv);
        const float a_in_dv = a_curr * initial_sign;
        if (a_in_dv > 0.0f &&
            dv_abs * 2.0f * j <= a_in_dv * a_in_dv) {
          status->trajectory_rest_committed = true;
          status->trajectory_committed_position = data->position;
          status->trajectory_committed_velocity = data->velocity;
          return 0.0f;
        }
        return a_max * initial_sign;
      }();

      const float acceleration =
          jerk_limited ?
          SlewAcceleration(a_curr, target_a, j * period_s) :
          target_a;

      status->control_acceleration = acceleration;
      if (jerk_limited) {
        *status->control_velocity =
            v_curr + (a_curr + acceleration) * 0.5f * period_s;
      } else {
        *status->control_velocity = v_curr + acceleration * period_s;
      }

      if (jerk_limited) {
        // Jerk-limited termination: wait until the slew has brought
        // `a` to exactly 0 (slew clamps inside [-j*dt, j*dt] -> 0,
        // so the final cycle's |Δa| is bounded by j*dt -- the same
        // strict per-cycle jerk bound that applies throughout the
        // trajectory).  The legacy `final_sign != initial_sign`
        // crossing test forced a discontinuous Δa proportional to
        // whatever `a` happened to be at the crossing, violating
        // the documented bound.
        if (status->trajectory_rest_committed &&
            status->control_acceleration == 0.0f) {
          status->control_velocity = velocity;
          status->trajectory_done = true;
        }
      } else {
        const float final_sign =
            (velocity > *status->control_velocity) ? 1.0f : -1.0f;
        if (final_sign != initial_sign) {
          status->control_acceleration = 0.0f;
          status->control_velocity = velocity;
          status->trajectory_done = true;
        }
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

    const float j = data->jerk_limit;
    const bool jerk_limited = std::isfinite(j) && j > 0.0f;
    // NaN/negative `accel_limit` with a finite jerk_limit means
    // "no acceleration cap, but slew acceleration at j*dt".  Treat
    // that as a_max = +infinity so the jerk-limited trajectory math
    // (which produces a 4-segment triangular-in-a profile without
    // any cruise-at-a_max phase) runs unmodified.  Falling all the
    // way back to the velocity-only path -- which forces (a, v) to
    // (0, +/-velocity_limit) discretely -- would silently ignore
    // the jerk limit the user did set.
    const float a =
        !std::isnan(data->accel_limit) ? data->accel_limit :
        (jerk_limited ? std::numeric_limits<float>::infinity() :
                        std::numeric_limits<float>::quiet_NaN());

    const float v0 = *status->control_velocity;
    const float vf = velocity;
    const float a_prev = status->control_acceleration;

    // What is the delta between our current control state and the
    // command.
    const float dx = MotorPosition::IntToFloat(
        MotorPosition::WrappingSub(
            *data->position_relative_raw, *status->control_position_raw));

    if (std::isnan(a)) {
      // No acceleration cap and no jerk cap -- only a velocity
      // limit.  Force the velocity directly.
      DoVelocityOnlyLimit(
          status, dx, data, velocity, period_s);
      return;
    }

    // Precompute reciprocals and j*dt once per cycle so the hot path
    // (CalculateJerkLimitedAcceleration and ComputeJerkStopDistance,
    // both running inside the position ISR at the configured
    // pwm_rate_hz / interrupt_divisor, typically 15-30 kHz) does
    // not eat 14-cycle VDIV.F32s where a multiply suffices.
    const float inv_j = jerk_limited ? (1.0f / j) : 0.0f;
    const float j_dt = jerk_limited ? (j * period_s) : 0.0f;

    const float target_acceleration =
        jerk_limited ?
        CalculateJerkLimitedAcceleration(
            status, data, a, j, inv_j,
            v0, vf, a_prev, dx, period_s, j_dt) :
        CalculateAcceleration(
            data, a, v0, vf, dx, period_s);

    const float acceleration = jerk_limited ?
        SlewAcceleration(a_prev, target_acceleration, j_dt) :
        target_acceleration;

    status->control_acceleration = acceleration;
    // For the jerk-limited path, integrate velocity using the average
    // of the prior and new acceleration (exact for constant jerk
    // within a cycle).  In the legacy path the cycle's acceleration
    // is piecewise constant and replaces the prior value, so the
    // direct v += a*dt rule applies.
    if (jerk_limited) {
      *status->control_velocity =
          v0 + (a_prev + acceleration) * 0.5f * period_s;
    } else {
      *status->control_velocity = v0 + acceleration * period_s;
    }
    const float v1 = *status->control_velocity;

    // For the legacy (non-jerk-limited) path, use the original
    // velocity-crosses-target / position-close-to-target check.
    if (!jerk_limited) {
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
        status->control_velocity =
            std::copysign(data->velocity_limit, v1);
      }

      const float v1_final = *status->control_velocity;
      const float signed_vel_lower = std::min(v0, v1_final);
      const float signed_vel_upper = std::max(v0, v1_final);
      const float v_frame_final = v1_final - vf;
      const float v_frame_final_abs = std::abs(v_frame_final);
      const float dx_abs = std::abs(dx);

      const bool target_cross =
          signed_vel_lower <= vf && signed_vel_upper >= vf;
      const bool target_near = v_frame_final_abs < (a * 0.5f * period_s);

      const float v_for_threshold =
          std::max(v_frame_final_abs, std::abs(vf));
      const bool position_near =
          dx_abs <= v_for_threshold * 10.0f * period_s;

      if ((target_cross || target_near) && position_near) {
        data->position = std::numeric_limits<float>::quiet_NaN();
        data->position_relative_raw.reset();
        status->control_acceleration = 0.0f;
        status->control_velocity = vf;
        status->trajectory_done = true;
      }
      return;
    }

    // Jerk-limited termination.  We have already verified the
    // predicted-endpoint condition once (inside
    // CalculateJerkLimitedAcceleration, which set
    // status->trajectory_rest_committed = true).  After commit, slew
    // brings `a` toward 0 at rate j*dt per cycle.  Terminate only
    // when the slew has clamped `a` to exactly 0: SlewAcceleration
    // returns target_a (= 0) whenever |a_prev| <= j*dt, so the last
    // observed Δa is bounded by j*dt and matches the per-cycle
    // jerk bound that holds throughout the trajectory.
    if (status->trajectory_rest_committed &&
        status->control_acceleration == 0.0f) {
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

    // Invalidate the jerk-limit rest-commit latch if the host has
    // retargeted since we committed.  data->position and
    // data->velocity are the raw fields the host wrote; they do not
    // drift between ISR cycles within a single command frame (only
    // position_relative_raw does, via the integration step at the
    // end of this function).  A new external command frame
    // overwrites these atomically, so a mismatch here means the
    // host wants something different than what we committed to.
    //
    // The comparison uses a kinematic noise floor rather than a
    // bit-exact equality: a host that streams the same logical
    // target with sub-LSB float jitter, or that slowly tracks a
    // moving target with per-command advances below the trajectory
    // generator's own residual, must NOT invalidate the latch on
    // every cycle (the resulting brake/commit oscillation never
    // terminates).
    if (status->trajectory_rest_committed) {
      const float cp = status->trajectory_committed_position;
      const float cv = status->trajectory_committed_velocity;
      const float a_max = data->accel_limit;
      const float j_lim = data->jerk_limit;
      const bool jerk_active =
          std::isfinite(a_max) && std::isfinite(j_lim) && j_lim > 0.0f;
      // Position tolerance: half the rest-curve kinematic residual
      // `a^2 / (j * rate) = a^2 * dt / j`.  Velocity tolerance: half
      // the one-cycle cruise step `a * dt`.  Both fall to zero
      // (strict equality) when no useful jerk limit is configured;
      // the latch is only set on the jerk-limited path anyway.
      const float pos_tol = jerk_active ?
          (0.5f * a_max * a_max * period_s / j_lim) : 0.0f;
      const float vel_tol = jerk_active ?
          (0.5f * a_max * period_s) : 0.0f;
      const bool position_same =
          (std::isnan(data->position) && std::isnan(cp)) ||
          (std::isfinite(data->position) && std::isfinite(cp) &&
           std::abs(data->position - cp) <= pos_tol);
      const bool velocity_same =
          (std::isnan(data->velocity) && std::isnan(cv)) ||
          (std::isfinite(data->velocity) && std::isfinite(cv) &&
           std::abs(data->velocity - cv) <= vel_tol);
      if (!position_same || !velocity_same) {
        status->trajectory_rest_committed = false;
      }
    }

    // We go to some lengths in our conversions to and from
    // control_position_raw so as to avoid converting a float directly to
    // an int64, which calls out to a system library that is pretty
    // slow.

    if (std::isnan(data->velocity_limit) &&
        std::isnan(data->accel_limit)) {
      status->trajectory_done = true;
      status->trajectory_rest_committed = false;
      status->control_acceleration = 0.0f;
      status->control_velocity = velocity;
    } else if (!!data->position_relative_raw ||
               !std::isnan(velocity)) {
      if (!status->trajectory_rest_committed) {
        // No latch held -- either the trajectory is mid-flight or
        // the latch was just invalidated above because the host
        // sent a target that differs from the committed snapshot.
        // Either way, allow UpdateTrajectory to (re-)plan.
        status->trajectory_done = false;
      }
      // If the latch is still committed, the host is re-sending the
      // same target every cycle (move_to() polling pattern).
      // Don't disturb the completed trajectory -- leave
      // trajectory_done = true so UpdateTrajectory is skipped and
      // control_position_raw is not perturbed.
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

    // Capture the velocity before we update the trajectory
    const float v0 = status->control_velocity.value_or(0.0f);

    if (!status->trajectory_done) {
      UpdateTrajectory(status, config, period_s, data, velocity);
    }

    // v1 will be the velocity after the trajectory update, before max
    // velocity clamp.
    const float v1 = status->control_velocity.value_or(0.0f);

    if (*status->control_velocity > status->motor_max_velocity) {
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

    if (data->position_relative_raw && !std::isnan(velocity)) {
      const float tstep = velocity * period_s;
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

    return velocity_command;
  }
};

}
