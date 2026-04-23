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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

#include "mjlib/base/assert.h"

#include "fw/bldc_servo_position.h"
#include "fw/bldc_servo_structs.h"
#include "fw/ccm.h"
#include "fw/error.h"
#include "fw/foc.h"
#include "fw/math.h"
#include "fw/motor_position.h"
#include "fw/pid.h"
#include "fw/simple_pi.h"
#include "fw/torque_model.h"

namespace moteus {

/// A simple first-order exponential low-pass filter.
class ExponentialFilter {
 public:
  ExponentialFilter() {}

  /// cutoff_hz - The rough bandwidth of the filter.  The actually 3dB
  ///   cutoff frequency will be cutoff_hz / (2*pi).
  ExponentialFilter(float rate_hz, float cutoff_hz)
      : alpha_(cutoff_hz / rate_hz),
        one_minus_alpha_(1.0f - alpha_) {}

  void operator()(float input, float* filtered) {
    if (!std::isfinite(*filtered)) {
      *filtered = input;
    } else {
      *filtered = alpha_ * input + one_minus_alpha_ * *filtered;
    }
  }

 private:
  float alpha_ = 1.0f;
  float one_minus_alpha_ = 0.0f;
};

inline float Limit(float a, float min, float max) MOTEUS_CCM_ATTRIBUTE;

inline float Limit(float a, float min, float max) {
  if (a < min) { return min; }
  if (a > max) { return max; }
  return a;
}

inline std::pair<float, errc> LimitCode(
    float a, float min, float max,
    errc code_limit, errc code_nolimit) MOTEUS_CCM_ATTRIBUTE;

inline std::pair<float, errc> LimitCode(
    float a, float min, float max,
    errc code_limit, errc code_nolimit) {
  if (a < min) { return {min, code_limit}; }
  if (a > max) { return {max, code_limit}; }
  return {a, code_nolimit};
}

inline float Threshold(float value, float lower, float upper) MOTEUS_CCM_ATTRIBUTE;

inline float Threshold(float value, float lower, float upper) {
  if (value > lower && value < upper) { return 0.0f; }
  return value;
}

inline float Interpolate(float x, float xl, float xh,
                         float vl, float vh) MOTEUS_CCM_ATTRIBUTE;

inline float Interpolate(float x, float xl, float xh,
                         float vl, float vh) {
  return (x - xl) / (xh - xl) * (vh - vl) + vl;
}

template <typename Array>
int MapConfig(const Array& array, int value) {
  static_assert(sizeof(array) > 0);
  int result = 0;
  for (const auto& item : array) {
    if (value <= item) { return result; }
    result++;
  }
  // Never return past the end.
  return result - 1;
}

// This is used to determine the maximum allowable PWM value so that
// the current sampling is guaranteed to occur while the FETs are
// still low.  It was calibrated using the scope and trial and error.
//
// The primary test is a high torque pulse with absolute position
// limits in place of +-1.0.  Something like "d pos nan 0 1 p0 d0 f1".
// This all but ensures the current controller will saturate.
//
// As of 2026-01-21, 0.45e-6 was the lowest value which always passed.
constexpr float kCurrentSampleTime = 0.60e-6f;

// All of these constants depend upon the pwm rate.
struct RateConfig {
  int int_rate_hz = 0;
  int interrupt_divisor = 0;
  uint32_t interrupt_mask = 0;
  int pwm_rate_hz = 0;
  float min_pwm = 0.0f;
  float max_pwm = 0.0f;
  float max_voltage_ratio = 0.0f;
  float rate_hz = 0.0f;
  float period_s = 0.0f;
  int16_t max_position_delta = 0;

  RateConfig() {}

  RateConfig(int pwm_rate_hz_in, int board_min_pwm_rate_hz) {
    // Limit our PWM rate to even frequencies between 15kHz and 60kHz.
    pwm_rate_hz =
        ((std::max(board_min_pwm_rate_hz,
                   std::min(60000, pwm_rate_hz_in))) / 2) * 2;

    interrupt_divisor = (pwm_rate_hz > 30000) ? 2 : 1;
    interrupt_mask = [&]() {
                       switch (interrupt_divisor) {
                         case 1: return 0u;
                         case 2: return 1u;
                         default: return 0u;
                       }
                     }();

    // The maximum interrupt rate is 30kHz, so if our PWM rate is
    // higher than that, then set up the interrupt at half rate.
    int_rate_hz = pwm_rate_hz / interrupt_divisor;

    min_pwm = kCurrentSampleTime / (0.5f / static_cast<float>(pwm_rate_hz));
    max_pwm = 1.0f - min_pwm;
    max_voltage_ratio = ((max_pwm - 0.5f) * 2.0f);

    rate_hz = int_rate_hz;
    period_s = 1.0f / rate_hz;

    // The maximum amount the absolute encoder can change in one cycle
    // without triggering a fault.  Measured as a fraction of a uint16_t
    // and corresponds to roughly 28krpm, which is the limit of the AS5047
    // encoder.
    //  28000 / 60 = 467 Hz
    //  467 Hz * 65536 / kIntRate ~= 763
    max_position_delta = 28000 / 60 * 65536 / int_rate_hz;
  }
};

constexpr float kDefaultTorqueConstant = 0.1f;
constexpr float kMaxUnconfiguredCurrent = 5.0f;

/// Template class that encapsulates the BldcServo control logic,
/// parameterized on an Impl type that provides both hardware-touching
/// terminal operations and access to all state members.  On the
/// target, the Impl is BldcServo::Impl (zero overhead via inlining).
/// In tests, Impl is a mock context.
///
/// The Impl must provide a long list of things which it isn't
/// important to enumerate aside from that it must compile.
template <typename Impl>
class BldcServoControl {
 public:
  using Control = BldcServoControl_Control;
  using CommandData = BldcServoCommandData;

  // State pulled from BldcServoConfig or the motor struct by the
  // Impl's config-update path.  Lives in the base because only
  // BldcServoControl reads it; derived Impls (production and test
  // fixtures) just write it during initialization / config changes.
  float torque_constant_ = 0.0f;
  float flux_brake_min_voltage_ = 0.0f;
  float derate_temperature_ = 0.0f;
  float motor_derate_temperature_ = 0.0f;

  // ISR epoch counter — set by the Impl (or tests) and compared
  // against position_.epoch inside the control loop to detect new
  // position data.
  uint8_t isr_motor_position_epoch_ = 0;

  BldcServoControl() {}

  /// Process a command, applying defaults and transforming positions.
  /// This mirrors the logic in BldcServo::Impl::Command but without
  /// double-buffering.  Returns an error code if the command is invalid.
  errc PrepareCommand(CommandData* data) {
    if (data->mode == kFault ||
        data->mode == kEnabling ||
        data->mode == kCalibrating ||
        data->mode == kCalibrationComplete) {
      // These are not valid states to command.
      return errc::kSuccess;
    }

    if (data->timeout_s == 0.0f) {
      data->timeout_s = self().config_.default_timeout_s;
    }
    if (std::isnan(data->velocity_limit)) {
      data->velocity_limit = self().config_.default_velocity_limit;
    } else if (data->velocity_limit < 0.0f) {
      data->velocity_limit = std::numeric_limits<float>::quiet_NaN();
    }
    if (std::isnan(data->accel_limit)) {
      data->accel_limit = self().config_.default_accel_limit;
    } else if (data->accel_limit < 0.0f) {
      data->accel_limit = std::numeric_limits<float>::quiet_NaN();
    }

    // If we are going to limit at all, ensure that we have a velocity
    // limit, and that it is no more than the configured maximum velocity.
    if (!std::isnan(data->velocity_limit) || !std::isnan(data->accel_limit)) {
      if (std::isnan(data->velocity_limit)) {
        data->velocity_limit = self().config_.max_velocity;
      } else {
        data->velocity_limit =
            std::min(data->velocity_limit, self().config_.max_velocity);
      }
    }

    // If we have a velocity command and velocity_limit, ensure that
    // the command does not violate the limit.
    if (!std::isnan(data->velocity_limit) &&
        !std::isnan(data->velocity)) {
      data->velocity = Limit(data->velocity,
                             -data->velocity_limit,
                             data->velocity_limit);
    }

    // Transform any position and stop_position command into the
    // relative raw space.
    const auto delta = static_cast<int64_t>(self().absolute_relative_delta()) << 32ll;
    if (!std::isnan(data->position)) {
      data->position_relative_raw =
          MotorPosition::FloatToInt(data->position) - delta;
    } else {
      data->position_relative_raw.reset();
    }

    if (!std::isnan(data->stop_position)) {
      data->stop_position_relative_raw =
          MotorPosition::FloatToInt(data->stop_position) - delta;
    }

    // If we have a case where the position is left unspecified, but
    // we have a velocity and stop condition, then we pick the sign of
    // the velocity so that we actually move.
    if (!data->position_relative_raw &&
        !!data->stop_position_relative_raw &&
        !std::isnan(data->velocity) &&
        data->velocity != 0.0f) {
      data->velocity = std::abs(data->velocity) *
          (((*data->stop_position_relative_raw -
             self().position_.position_relative_raw) > 0) ?
           1.0f : -1.0f);
    }

    if (!!data->stop_position_relative_raw &&
        (std::isfinite(data->accel_limit) ||
         std::isfinite(data->velocity_limit))) {
      // There is no valid use case for using a stop position along
      // with an acceleration or velocity limit.
      return errc::kStopPositionDeprecated;
    }

    if (self().config_.bemf_feedforward != 0.0f &&
        !std::isfinite(data->accel_limit) &&
        !self().config_.bemf_feedforward_override) {
      // We normally don't allow bemf feedforward if an acceleration
      // limit is not applied, as that can easily result in output
      // currents exceeding any configured limits.
      return errc::kBemfFeedforwardNoAccelLimit;
    }

    // Pre-compute this to save time in the ISR.
    data->synthetic_theta =
        self().config_.fixed_voltage_mode ||
        !std::isnan(data->fixed_voltage_override) ||
        !std::isnan(data->fixed_current_override);

    return errc::kSuccess;
  }

  bool is_torque_constant_configured() const MOTEUS_CCM_ATTRIBUTE {
    return self().motor_.Kv != 0.0f;
  }

  void UpdateEffectiveInductance(float i_d_A) MOTEUS_CCM_ATTRIBUTE {
    const float effective_L =
        self().motor_.inductance_d_H +
        self().motor_.inductance_d_scale * std::min(i_d_A, 0.0f);
    self().status_.inductance_d_H = effective_L;
    pid_d_config_.kp = pid_dq_w_ * effective_L;
    // id_char = lambda_m / effective_L
    // See also: mpat Motor.getCharacteristicCurrent()
    self().status_.fw.id_char =
        (effective_L > 0.0f && self().lambda_m_ > 0.0f) ?
        self().lambda_m_ / effective_L :
        0.0f;
  }

  void UpdateFieldWeakeningIdChar() {
    const float L_at_max =
        self().motor_.inductance_d_H +
        self().motor_.inductance_d_scale * (-fw_max_current_A_);
    fw_id_char_at_max_current_ =
        (L_at_max > 0.0f && lambda_m_ > 0.0f) ?
        lambda_m_ / L_at_max : 0.0f;
  }

  void UpdateDerivedMotorConstants() {
    // 1 / sqrt(3) = 0.57735
    v_per_hz_ = self().motor_.Kv == 0.0f ?
        0.0f : 0.57735f * 60.0f / self().motor_.Kv;
    // See also: mpat Motor.getLambdaM()
    lambda_m_ = (v_per_hz_ > 0.0f && self().motor_.poles > 0) ?
        v_per_hz_ / (kPi * self().motor_.poles) : 0.0f;

    // Precomputed constants for the FW base velocity quadratic.  See
    // ISR_CalculateDerivedQuantities.
    v_per_hz_squared_ = v_per_hz_ * v_per_hz_;
    pi_poles_Lq_ =
        kPi * self().motor_.poles * self().motor_.inductance_q_H;
    two_R_v_per_hz_ = 2.0f * self().motor_.resistance_ohm * v_per_hz_;
    R_squared_ =
        self().motor_.resistance_ohm * self().motor_.resistance_ohm;
    fw_V_eff_factor_ =
        self().rate_config_.max_voltage_ratio *
        (1.0f - self().config_.fw.modulation_margin) * 0.5f;
    half_max_voltage_ratio_over_v_per_hz_ =
        (v_per_hz_ > 0.0f) ?
        (self().rate_config_.max_voltage_ratio * 0.5f / v_per_hz_) :
        0.0f;
    max_V_factor_ =
        self().rate_config_.max_voltage_ratio * kSvpwmRatio * 0.5f;
    fw_max_current_A_ =
        self().config_.fw.max_current_ratio * self().config_.max_current_A;
    half_over_R_ = (self().motor_.resistance_ohm > 0.0f) ?
        (0.5f / self().motor_.resistance_ohm) : 0.0f;
    {
      constexpr float kBoardVoltageMargin = 6.0f;
      board_max_velocity_factor_ = (v_per_hz_ > 0.0f) ?
          (0.5f * (self().config_.max_voltage - kBoardVoltageMargin) /
           v_per_hz_) : 0.0f;
    }
    {
      const auto* pos_config = self().motor_position_config();
      const auto cs = pos_config->commutation_source;
      const float cpr = static_cast<float>(pos_config->sources[cs].cpr);
      commutation_inv_cpr_ = (cpr > 0.0f) ? (1.0f / cpr) : 0.0f;
    }
    // Cache a TorqueModel with precomputed reciprocals so the ISR
    // can use multiplications instead of divisions.
    torque_model_ = TorqueModel(
        torque_constant_,
        self().motor_.rotation_current_cutoff_A,
        self().motor_.rotation_current_scale,
        self().motor_.rotation_torque_scale);

    pid_dq_w_ = k2Pi * self().config_.pid_dq_hz;
    pid_d_config_.ki = pid_dq_w_ * self().motor_.resistance_ohm;
    pid_d_config_.max_desired_rate = self().config_.max_current_desired_rate;
    pid_q_config_.kp = pid_dq_w_ * self().motor_.inductance_q_H;
    pid_q_config_.ki = pid_dq_w_ * self().motor_.resistance_ohm;
    pid_q_config_.max_desired_rate = self().config_.max_current_desired_rate;
    UpdateEffectiveInductance(0.0f);
  }

  float current_to_torque(float current) const MOTEUS_CCM_ATTRIBUTE {
    return torque_model_.current_to_torque(current);
  }

  float torque_to_current(float torque) const MOTEUS_CCM_ATTRIBUTE {
    return torque_model_.torque_to_current(torque);
  }

  bool current_control() const MOTEUS_CCM_ATTRIBUTE {
    switch (self().status_.mode) {
      case kNumModes:
        return false;
      case kFault:
      case kCalibrating:
      case kCalibrationComplete:
      case kEnabling:
      case kStopped:
      case kPwm:
      case kVoltage:
      case kVoltageFoc:
      case kVoltageDq:
      case kMeasureInductance:
      case kBrake:
        return false;
      case kCurrent:
      case kPosition:
      case kZeroVelocity:
      case kStayWithinBounds:
        return true;
      case kPositionTimeout:
        return (self().config_.timeout_mode == BldcServoMode::kZeroVelocity ||
                self().config_.timeout_mode == BldcServoMode::kPosition);
    }
    return false;
  }

  bool torque_on() const MOTEUS_CCM_ATTRIBUTE {
    switch (self().status_.mode) {
      case kNumModes:
        return false;
      case kFault:
      case kCalibrating:
      case kCalibrationComplete:
      case kEnabling:
      case kStopped:
        return false;
      case kPwm:
      case kVoltage:
      case kVoltageFoc:
      case kVoltageDq:
      case kCurrent:
      case kPosition:
      case kZeroVelocity:
      case kStayWithinBounds:
      case kMeasureInductance:
      case kBrake:
        return true;
      case kPositionTimeout:
        return self().config_.timeout_mode != 0;
    }
    return false;
  }

  bool ISR_IsOutsideLimits() const MOTEUS_CCM_ATTRIBUTE {
    return ((!std::isnan(self().position_config_.position_min) &&
             self().position_.position < self().position_config_.position_min) ||
            (!std::isnan(self().position_config_.position_max) &&
             self().position_.position > self().position_config_.position_max));
  }

  bool ISR_InvalidLimits() const MOTEUS_CCM_ATTRIBUTE {
    return ((!std::isnan(self().position_config_.position_min) && (
                 std::abs(self().position_config_.position_min) > 32768.0f)) ||
            (!std::isnan(self().position_config_.position_max) && (
                std::abs(self().position_config_.position_max) > 32768.0f)));
  }

  float LimitPwm(float in) const MOTEUS_CCM_ATTRIBUTE {
    return Limit(in, self().rate_config_.min_pwm, self().rate_config_.max_pwm);
  }

  /// Compute electrical theta, perform DQ transform, and calculate derived
  /// quantities.
  ///
  /// If use_synthetic_theta is true, computes electrical theta from
  /// control_position_raw (for fixed voltage mode, etc.).  Otherwise uses
  /// position_.electrical_theta from the encoder.
  ///
  /// Sets: status_.electrical_theta, status_.sin, status_.cos,
  ///       status_.d_A, status_.q_A, status_.torque_Nm, status_.motor_max_velocity
  ///
  /// Returns: SinCos for use by ISR_DoControl
  SinCos ISR_CalculateDerivedQuantities(
      float cur_a, float cur_b, float cur_c,
      bool use_synthetic_theta) MOTEUS_CCM_ATTRIBUTE {
    // Filter bus voltage (matches real firmware ADC filtering).
    slow_bus_v_filter_(self().status_.bus_V, &self().status_.filt_bus_V);
    fast_bus_v_filter_(self().status_.bus_V, &self().status_.filt_1ms_bus_V);

    // Compute electrical theta.
    const float electrical_theta = !use_synthetic_theta ?
        self().position_.electrical_theta :
        WrapZeroToTwoPi(
            self().motor_position_config()->output.sign *
            MotorPosition::IntToFloat(*self().status_.control_position_raw) /
            self().motor_position_config()->rotor_to_output_ratio *
            self().motor_.poles * 0.5f * k2Pi);
    self().status_.electrical_theta = electrical_theta;

    // Compute sin/cos for transforms.
    const SinCos sin_cos = self().cordic(RadiansToQ31(electrical_theta));
    self().status_.sin = sin_cos.s;
    self().status_.cos = sin_cos.c;

    // DQ transform from phase currents.
    DqTransform dq{sin_cos, cur_a, cur_b, cur_c};
    self().status_.d_A = dq.d;
    self().status_.q_A = self().motor_position_config()->output.sign * dq.q;

    // Calculate output torque from q-axis current.
    const float rotor_to_output_ratio =
        self().motor_position_config()->rotor_to_output_ratio;
    // Cache the reciprocal for reuse by later ISR stages.  This runs
    // before ISR_DoControl so any downstream use sees a current value.
    inv_rotor_to_output_ratio_ = (rotor_to_output_ratio != 0.0f) ?
        (1.0f / rotor_to_output_ratio) : 0.0f;
    const bool is_torque_on = torque_on();
    self().status_.torque_Nm = is_torque_on ?
        (current_to_torque(self().status_.q_A) *
         inv_rotor_to_output_ratio_) : 0.0f;
    if (!is_torque_on) {
      self().status_.torque_error_Nm = 0.0f;
    }

    // Calculate maximum achievable motor velocity based on bus voltage.

    const float unfiltered_base_velocity = [&]() {
      if (self().config_.fw.enable && self().v_per_hz_ > 0.0f) {
        // Current-dependent base speed for MTPA on SPMSM (Ld ≈ Lq).
        // With id = 0 below base speed, the voltage magnitude is:
        //   V² = (R·iq + vel·v_per_hz)² + (π·poles·L·vel·iq)²
        // Solving for vel gives a quadratic: a·vel² + b·vel + c = 0.
        //
        // The sign of iq matters: when regenerating (iq < 0), the
        // resistive drop opposes back-EMF, raising the base velocity.
        //
        // Base velocity includes target_modulation (1 - modulation_margin)
        // for field weakening q-axis voltage headroom. This ensures that
        // id = 0 exactly at base_velocity with no discontinuity.
        // Note: no kSvpwmRatio here to match the non-FW path formula.
        const float V_eff =
            self().fw_V_eff_factor_ * self().status_.filt_bus_V;
        const float iq = self().status_.q_A;
        const float omega_L_iq = self().pi_poles_Lq_ * iq;
        const float qa = self().v_per_hz_squared_ + omega_L_iq * omega_L_iq;
        const float qb = self().two_R_v_per_hz_ * iq;
        const float qc =
            self().R_squared_ * iq * iq - V_eff * V_eff;
        const float disc = qb * qb - 4.0f * qa * qc;
        if (disc >= 0.0f && qa > 0.0f) {
          const float vel_rotor_hz =
              (-qb + std::sqrt(disc)) / (2.0f * qa);
          return std::max(0.0f, vel_rotor_hz * rotor_to_output_ratio);
        } else {
          return 0.0f;
        }
      } else {
        return rotor_to_output_ratio *
            self().half_max_voltage_ratio_over_v_per_hz_ *
            self().status_.filt_bus_V;
      }
    }();
    base_velocity_filter_(
        unfiltered_base_velocity, &self().status_.motor_base_velocity);

    // Calculate motor_max_velocity based on field weakening state.
    //
    // Uses the previous cycle's effective_max_current_A (set by
    // ISR_DoCurrentControl) which already accounts for all sources
    // of current derating (temperature, etc.).
    const float max_velocity = [&]() {
      if (!self().config_.fw.enable ||
          fw_id_char_at_max_current_ <= 0.0f) {
        return self().status_.motor_base_velocity;
      }
      // CPSR (Constant Power Speed Ratio):
      // At max speed, id = -Imax = -id_char × (1 - 1 / speed_ratio)
      // Solving: speed_ratio_max = 1 / (1 - Imax/id_char)
      // So: max_velocity = base_velocity / (1 - id_ratio)
      // Note: target_modulation is already incorporated in base_velocity.

      // effective_max_current_A is already <= max_current_A by
      // construction in ISR_DoCurrentControl, so no need to re-min.
      const float fw_max_A =
          self().status_.effective_max_current_A *
          self().config_.fw.max_current_ratio;

      // Compute id_char at the effective max FW current (inductance
      // varies with current due to saturation).
      const float L_at_fw_max =
          self().motor_.inductance_d_H +
          self().motor_.inductance_d_scale * (-fw_max_A);
      const float id_char_at_fw_max =
          (L_at_fw_max > 0.0f && self().lambda_m_ > 0.0f) ?
          self().lambda_m_ / L_at_fw_max :
          0.0f;
      if (id_char_at_fw_max <= 0.0f) {
        return self().status_.motor_base_velocity;
      }

      constexpr float kMaxCpsr = 3.0f;
      const float Imax = std::min(
          self().half_over_R_ * self().status_.filt_bus_V,
          fw_max_A);
      const float id_ratio = Imax / id_char_at_fw_max;
      const float cpsr = (id_ratio >= 1.0f) ? kMaxCpsr :
          std::min(kMaxCpsr, 1.0f / (1.0f - id_ratio));
      const float cpsr_velocity = cpsr * self().status_.motor_base_velocity;

      const float board_max_velocity =
          self().board_max_velocity_factor_ * rotor_to_output_ratio;
      return std::min(cpsr_velocity, board_max_velocity);
    }();
    max_velocity_filter_(max_velocity, &self().status_.motor_max_velocity);

    return sin_cos;
  }

  enum ClearMode {
    kClearIfMode,
    kAlwaysClear,
  };

  void ISR_ClearPid(ClearMode force_clear) MOTEUS_CCM_ATTRIBUTE {
    const bool current_pid_active = [&]() MOTEUS_CCM_ATTRIBUTE {
      switch (self().status_.mode) {
        case kNumModes:
        case kFault:
        case kEnabling:
        case kCalibrating:
        case kCalibrationComplete:
        case kPwm:
        case kVoltage:
        case kVoltageFoc:
        case kVoltageDq:
        case kMeasureInductance:
        case kBrake:
          return false;
        case kCurrent:
        case kPosition:
        case kPositionTimeout:
        case kZeroVelocity:
        case kStayWithinBounds:
          return true;
        case kStopped:
          return self().status_.cooldown_count != 0;
      }
      return false;
    }();

    if (!current_pid_active || force_clear == kAlwaysClear) {
      self().status_.pid_d.Clear();
      self().status_.pid_q.Clear();

      self().status_.pid_d.desired = 0.0f;
      self().status_.pid_q.desired = 0.0f;
    }

    const bool position_pid_active = [&]() MOTEUS_CCM_ATTRIBUTE {
      switch (self().status_.mode) {
        case kNumModes:
        case kStopped:
        case kFault:
        case kEnabling:
        case kCalibrating:
        case kCalibrationComplete:
        case kPwm:
        case kVoltage:
        case kVoltageFoc:
        case kVoltageDq:
        case kCurrent:
        case kMeasureInductance:
        case kBrake:
          return false;
        case kPosition:
        case kPositionTimeout:
        case kZeroVelocity:
        case kStayWithinBounds:
          return true;
      }
      return false;
    }();

    if (!position_pid_active || force_clear == kAlwaysClear) {
      self().status_.pid_position.Clear();
      self().status_.control_position_raw = {};
      self().status_.control_position = std::numeric_limits<float>::quiet_NaN();
      self().status_.control_velocity = {};
      self().status_.control_acceleration = {};
    }
  }

  void ISR_DoPwmControl(const Vec3& pwm) MOTEUS_CCM_ATTRIBUTE {
    self().control_.pwm.a = LimitPwm(pwm.a);
    self().control_.pwm.b = LimitPwm(pwm.b);
    self().control_.pwm.c = LimitPwm(pwm.c);

    self().DoPwmControl(self().control_.pwm);
  }

  /// Assume that the voltages are intended to be balanced around the
  /// midpoint and can be shifted accordingly.
  __attribute__((always_inline))
  void ISR_DoBalancedVoltageControl(const Vec3& voltage) MOTEUS_CCM_ATTRIBUTE {
    self().control_.voltage = voltage;

    const float inv_bus_V = 1.0f / self().status_.filt_bus_V;
    const Vec3 pwm_in = {
        voltage.a * inv_bus_V,
        voltage.b * inv_bus_V,
        voltage.c * inv_bus_V};

    const float pwmmin = std::min(pwm_in.a, std::min(pwm_in.b, pwm_in.c));
    const float pwmmax = std::max(pwm_in.a, std::max(pwm_in.b, pwm_in.c));

    const float offset = 0.5f * (pwmmin + pwmmax) - 0.5f;

    ISR_DoPwmControl(Vec3{
        pwm_in.a - offset,
        pwm_in.b - offset,
        pwm_in.c - offset});
  }

  Vec3 ISR_CalculatePhaseVoltage(const SinCos& sin_cos,
                                 float d_V, float q_V) MOTEUS_CCM_ATTRIBUTE {
    if (self().position_.epoch != isr_motor_position_epoch_) {
      self().status_.mode = kFault;
      self().status_.fault = errc::kConfigChanged;

      return Vec3{0.f, 0.f, 0.f};
    }

    self().control_.d_V = d_V;
    self().control_.q_V = q_V;

    InverseDqTransform idt(
        sin_cos, self().control_.d_V,
        self().motor_position_config()->output.sign * self().control_.q_V);

    return Vec3{idt.a, idt.b, idt.c};
  }

  void ISR_DoVoltageDQ(const SinCos& sin_cos,
                       float d_V, float q_V) MOTEUS_CCM_ATTRIBUTE {
    ISR_DoBalancedVoltageControl(
        ISR_CalculatePhaseVoltage(sin_cos, d_V, q_V));
  }

  void ISR_DoVoltageDQCommand(const SinCos& sin_cos,
                              float d_V, float q_V) MOTEUS_CCM_ATTRIBUTE {
    if (self().motor_.poles == 0) {
      self().status_.mode = kFault;
      self().status_.fault = errc::kMotorNotConfigured;
      return;
    }
    if (!self().position_.theta_valid) {
      self().status_.mode = kFault;
      self().status_.fault = errc::kThetaInvalid;
      return;
    }

    const float max_V = self().max_V_factor_ * self().status_.filt_bus_V;

    ISR_DoBalancedVoltageControl(
        ISR_CalculatePhaseVoltage(
            sin_cos,
            Limit(d_V, -max_V, max_V),
            Limit(q_V, -max_V, max_V)));
  }

  void ISR_DoCurrent(const SinCos& sin_cos, float i_d_A_in, float i_q_A_in,
                     float feedforward_velocity_rotor,
                     bool ignore_position_bounds) MOTEUS_CCM_ATTRIBUTE {
    if (self().motor_.poles == 0) {
      self().status_.mode = kFault;
      self().status_.fault = errc::kMotorNotConfigured;
      return;
    }
    if (!self().position_.theta_valid) {
      self().status_.mode = kFault;
      self().status_.fault = errc::kThetaInvalid;
      return;
    }

    auto limit_q_current = [&](std::pair<float, errc> in_pair) -> std::pair<float, errc> MOTEUS_CCM_ATTRIBUTE {
      const auto in = in_pair.first;
      if (ignore_position_bounds) { return {in, in_pair.second}; }

      if (!std::isnan(self().position_config_.position_max) &&
          self().position_.position > self().position_config_.position_max &&
          in > 0.0f) {
        return std::make_pair(
            in *
            std::max(0.0f,
                     1.0f - (self().position_.position -
                             self().position_config_.position_max) /
                     self().config_.position_derate),
            errc::kLimitPositionBounds);
      }
      if (!std::isnan(self().position_config_.position_min) &&
          self().position_.position < self().position_config_.position_min &&
          in < 0.0f) {
        return std::make_pair(
            in *
            std::max(0.0f,
                     1.0f - (self().position_config_.position_min -
                             self().position_.position) /
                     self().config_.position_derate),
            errc::kLimitPositionBounds);
      }

      return in_pair;
    };

    auto limit_q_velocity = [&](std::pair<float, errc> in_pair) -> std::pair<float, errc> MOTEUS_CCM_ATTRIBUTE {
      const auto in = in_pair.first;
      const float abs_velocity = std::abs(self().position_.velocity);
      if (abs_velocity < self().config_.max_velocity ||
          self().position_.velocity * in < 0.0f) {
        return in_pair;
      }
      const float derate_fraction =
          1.0f - ((abs_velocity - self().config_.max_velocity) /
                  self().config_.max_velocity_derate);
      const float current_limit =
          std::max(0.0f, derate_fraction * self().config_.max_current_A);
      const errc maybe_fault = derate_fraction < 1.0f ? errc::kLimitMaxVelocity : errc::kLimitMaxCurrent;
      return LimitCode(in, -current_limit, current_limit,
                       maybe_fault, in_pair.second);
    };

    float derate_fraction =
        (self().status_.filt_fet_temp_C - derate_temperature_) /
        self().config_.temperature_margin;
    errc derate_fault = errc::kLimitMaxCurrent;
    if (derate_fraction > 0.0f) {
      derate_fault = errc::kLimitFetTemperature;
    }
    if (std::isfinite(self().config_.motor_fault_temperature)) {
      const float motor_derate =
          ((self().status_.filt_motor_temp_C - motor_derate_temperature_) /
           self().config_.motor_temperature_margin);
      if (motor_derate > derate_fraction) {
        derate_fraction = motor_derate;
        derate_fault = errc::kLimitMotorTemperature;
      }
    }

    const float derate_current_A =
        std::max<float>(
            0.0f,
            derate_fraction *
            (self().config_.derate_current_A - self().config_.max_current_A) +
            self().config_.max_current_A);

    const float temp_limit_A = std::min<float>(
        self().config_.max_current_A, derate_current_A);
    self().status_.effective_max_current_A = temp_limit_A;

    auto limit_either_current = [&](std::pair<float, errc> in_pair) MOTEUS_CCM_ATTRIBUTE {
      return LimitCode(in_pair.first, -temp_limit_A, temp_limit_A, derate_fault, in_pair.second);
    };

    // Calculate the field weakening D axis current if enabled.
    //
    // We use a Constant Voltage Constant Power (CVCP) method with a
    // low-pass filter on the output to prevent oscillation at the
    // base velocity boundary.
    const auto [i_d_fw, fw_errc_val] = [&]() -> std::pair<float, errc> {
      // The exponential filter only asymptotically approaches zero, so
      // snap any sub-milliamp residual to exactly 0.  Otherwise a
      // lingering ~1e-30 value would keep kLimitFieldWeakening set
      // long after the motor has stopped.
      constexpr float kIdDeadband = 1e-3f;

      if (!self().config_.fw.enable) {
        // Feed 0 through the filter for smooth decay when FW is disabled.
        fw_id_filter_(0.0f, &self().status_.fw.id_A);
        self().status_.fw.id_A =
            Threshold(self().status_.fw.id_A, -kIdDeadband, kIdDeadband);
        return {self().status_.fw.id_A, errc::kSuccess};
      }

      const float abs_max_current =
          self().config_.fw.max_current_ratio *
          self().status_.effective_max_current_A;

      const float base_speed = self().status_.motor_base_velocity;

      // We use control_velocity rather than the measured speed by
      // default, as it eliminates one possible source of feedback
      // instability.
      const float velocity_for_fw =
          self().status_.control_velocity.value_or(self().position_.velocity);
      const float current_speed = std::abs(velocity_for_fw);

      // CVCP: id = -id_char * (1 - 1/speed_ratio)
      //         = -id_char * (current_speed - base_speed) / current_speed
      //
      // Note: target_modulation is incorporated in base_velocity, so
      // the formula simplifies to this form.  At current_speed =
      // base_speed, id = 0 with no discontinuity.
      const float id_cvcp = (current_speed > base_speed) ?
          -self().status_.fw.id_char *
          (current_speed - base_speed) / current_speed : 0.0f;
      const float id_raw = Limit(id_cvcp, -abs_max_current, 0.0f);

      // Low-pass filter the id command to prevent oscillation at
      // the base velocity boundary.
      fw_id_filter_(id_raw, &self().status_.fw.id_A);
      self().status_.fw.id_A =
          Threshold(self().status_.fw.id_A, -kIdDeadband, kIdDeadband);

      const float result = self().status_.fw.id_A;
      return {
        result,
        result != 0.0f ? errc::kLimitFieldWeakening : errc::kSuccess
      };
    }();

    const auto almost_i_q_A_pair =
        limit_either_current(
            limit_q_velocity(
                limit_q_current(
                    std::make_pair(i_q_A_in, errc::kSuccess))));
    const auto almost_i_d_A_pair = limit_either_current(
        std::make_pair(i_d_A_in + i_d_fw, fw_errc_val));

    const auto almost_i_q_A = almost_i_q_A_pair.first;
    const auto almost_i_d_A = almost_i_d_A_pair.first;
    const auto almost_fault =
        (almost_i_q_A_pair.second == errc::kSuccess) ?
        almost_i_d_A_pair.second : almost_i_q_A_pair.second;

    // Apply our power limits by limiting the maximum current command.
    const float used_d_power_W = 1.5f * old_d_V_ * almost_i_d_A;
    const float used_q_power_W = 1.5f * old_q_V_ * almost_i_q_A;
    const float used_power = used_q_power_W + used_d_power_W;

    const auto [i_d_A, i_q_A, limit_code] = [&]() {
      if (std::abs(used_power) < self().status_.max_power_W) {
        return std::make_tuple(almost_i_d_A, almost_i_q_A, almost_fault);
      }

      const float scale = self().status_.max_power_W / std::abs(used_power);

      // The power formula is P = 1.5 * V * I, so scaling power by 'scale'
      // and back-calculating current gives: I_new = (P * scale) / (1.5 * V)
      // = (1.5 * V * I * scale) / (1.5 * V) = I * scale.
      const float i_d_limited = almost_i_d_A * scale;
      const float i_q_limited = almost_i_q_A * scale;

      return std::make_tuple(i_d_limited, i_q_limited, errc::kLimitMaxPower);
    }();

    self().control_.i_d_A = i_d_A;
    self().control_.i_q_A = i_q_A;

    UpdateEffectiveInductance(i_d_A);

    const float max_V = self().max_V_factor_ * self().status_.filt_bus_V;

    const auto limit_to_max_voltage = [max_V](float denorm_d_V, float denorm_q_V, errc inlimit) {
      const float max_V_sq = max_V * max_V;
      const float denorm_len =
          denorm_d_V * denorm_d_V + denorm_q_V * denorm_q_V;
      if (denorm_len < max_V_sq) {
        return std::make_tuple(denorm_d_V, denorm_q_V, inlimit);
      }

      // D-axis priority: clamp Vd first, then limit Vq to remaining headroom
      const float d_V = Limit(denorm_d_V, -max_V, max_V);
      const float q_max = sqrtf(std::max(0.0f, max_V_sq - d_V * d_V));
      const float q_V = Limit(denorm_q_V, -q_max, q_max);
      return std::make_tuple(d_V, q_V, errc::kLimitMaxVoltage);
    };

    if (!self().config_.voltage_mode_control) {
      // Use the control velocity for improved stability.
      const float control_velocity_rotor =
          self().status_.control_velocity.value_or(
              self().position_.velocity) *
          self().inv_rotor_to_output_ratio_;
      const float omega_electrical =
          control_velocity_rotor * k2Pi * (self().motor_.poles * 0.5f);

      // Cross-coupling feedforward using the commanded currents.  The
      // measured currents include the sensor noise floor which, after
      // multiplication by ω_e×L, injects that noise directly into the
      // voltage command at high speed.  Commanded currents also avoid
      // the unit-delay algebraic loop between the PID output and the
      // current it commands back through the cross-coupling term.
      const float cross_coupling_d_ff =
          -omega_electrical * self().motor_.inductance_q_H * i_q_A *
          self().config_.cross_coupling_feedforward;

      const float denorm_d_V =
          self().pid_d_.Apply(self().status_.d_A, i_d_A, self().rate_config_.period_s) +
          i_d_A * self().config_.current_feedforward * self().motor_.resistance_ohm +
          cross_coupling_d_ff;

      const float cross_coupling_q_ff =
          omega_electrical * self().status_.inductance_d_H * i_d_A *
          self().config_.cross_coupling_feedforward;

      const float denorm_q_V =
          self().pid_q_.Apply(self().status_.q_A, i_q_A, self().rate_config_.period_s) +
          i_q_A * self().config_.current_feedforward * self().motor_.resistance_ohm +
          cross_coupling_q_ff +
          (feedforward_velocity_rotor *
           self().config_.bemf_feedforward *
           self().v_per_hz_);

      auto [d_V, q_V, final_limit_code] =
          limit_to_max_voltage(denorm_d_V, denorm_q_V, limit_code);

      if (final_limit_code != errc::kSuccess) {
        self().status_.fault = final_limit_code;
      }

      // Back-calculation anti-windup.
      const float d_feedforward =
          i_d_A * self().config_.current_feedforward * self().motor_.resistance_ohm +
          cross_coupling_d_ff;
      const float q_feedforward =
          i_q_A * self().config_.current_feedforward * self().motor_.resistance_ohm +
          cross_coupling_q_ff +
          (feedforward_velocity_rotor *
           self().config_.bemf_feedforward *
           self().v_per_hz_);

      if (d_V != denorm_d_V) {
        const float target_pid_d = d_V - d_feedforward;
        self().status_.pid_d.integral =
            -target_pid_d - pid_d_config_.kp * self().status_.pid_d.error;
      }

      if (q_V != denorm_q_V) {
        const float target_pid_q = q_V - q_feedforward;
        self().status_.pid_q.integral =
            -target_pid_q - pid_q_config_.kp * self().status_.pid_q.error;
      }

      self().status_.pid_d.integral = Limit(
          self().status_.pid_d.integral,
          -max_V, max_V);

      self().status_.pid_q.integral = Limit(
          self().status_.pid_q.integral,
          -max_V, max_V);

      self().status_.power_W =
          1.5f * (self().status_.d_A * d_V +
                  self().status_.q_A * q_V);

      ISR_DoVoltageDQ(sin_cos, d_V, q_V);
    } else {
      self().status_.power_W =
          1.5f * (i_d_A * i_d_A * self().motor_.resistance_ohm +
                  i_q_A * i_q_A * self().motor_.resistance_ohm);

      auto [d_V, q_V, final_limit_code] = limit_to_max_voltage(
          i_d_A * self().motor_.resistance_ohm,
          i_q_A * self().motor_.resistance_ohm +
          (feedforward_velocity_rotor *
           self().config_.bemf_feedforward *
           self().v_per_hz_),
          limit_code);

      if (final_limit_code != errc::kSuccess) {
        self().status_.fault = final_limit_code;
      }

      ISR_DoVoltageDQ(sin_cos, d_V, q_V);
    }
  }

  void ISR_DoPosition(const SinCos& sin_cos,
                      CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    PID::ApplyOptions apply_options;
    apply_options.kp_scale = data->kp_scale;
    apply_options.kd_scale = data->kd_scale;
    apply_options.ilimit_scale = data->ilimit_scale;

    ISR_DoPositionCommon(sin_cos, data, apply_options, data->max_torque_Nm,
                         data->feedforward_Nm, data->velocity);
  }

  void ISR_DoZeroVelocity(const SinCos& sin_cos,
                           CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    CommandData zero_velocity;

    zero_velocity.mode = kPosition;
    zero_velocity.position = std::numeric_limits<float>::quiet_NaN();
    zero_velocity.velocity = 0.0f;
    zero_velocity.timeout_s = std::numeric_limits<float>::quiet_NaN();

    PID::ApplyOptions apply_options;
    apply_options.kp_scale = 0.0f;
    apply_options.kd_scale = data->kd_scale;
    apply_options.ilimit_scale = 0.0f;

    ISR_DoPositionCommon(sin_cos, &zero_velocity,
                         apply_options, self().config_.timeout_max_torque_Nm,
                         0.0f, 0.0f);
  }

  void ISR_DoPositionCommon(
      const SinCos& sin_cos, CommandData* data,
      const PID::ApplyOptions& pid_options,
      float max_torque_Nm,
      float feedforward_Nm,
      float velocity) MOTEUS_CCM_ATTRIBUTE {
    const int64_t absolute_relative_delta =
        self().absolute_relative_delta() << 32ll;

    const float velocity_command =
        BldcServoPosition::UpdateCommand(
            &self().status_,
            &self().config_,
            &self().position_config_,
            &self().position_,
            absolute_relative_delta,
            self().rate_config_.period_s,
            data,
            velocity);

    // At this point, our control position and velocity are known.

    if (self().config_.fixed_voltage_mode ||
        !std::isnan(data->fixed_voltage_override) ||
        !std::isnan(data->fixed_current_override)) {
      self().status_.position =
          static_cast<float>(
              static_cast<int32_t>(
                  *self().status_.control_position_raw >> 32)) /
          65536.0f;
      self().status_.velocity = velocity_command;

      if (!std::isnan(data->fixed_current_override)) {
        ISR_DoCurrent(sin_cos,
                      data->fixed_current_override,
                      0.0f, 0.0f, data->ignore_position_bounds);
      } else {
        const float fixed_voltage =
            std::isnan(data->fixed_voltage_override) ?
            self().config_.fixed_voltage_control_V +
            (std::abs(self().status_.velocity) *
             self().v_per_hz_ *
             self().config_.bemf_feedforward) :
            data->fixed_voltage_override;
        ISR_DoVoltageDQ(sin_cos, fixed_voltage, 0.0f);
      }
      return;
    }

    // From this point, we require actual valid position.
    if (!self().position_.position_relative_valid) {
      self().status_.mode = kFault;
      self().status_.fault = errc::kPositionInvalid;
      return;
    }
    if (self().position_.error != MotorPosition::Status::kNone) {
      self().status_.mode = kFault;
      self().status_.fault = errc::kEncoderFault;
      return;
    }

    const float measured_velocity = velocity_command +
        Threshold(
            self().position_.velocity - velocity_command, -self().config_.velocity_threshold,
            self().config_.velocity_threshold);

    const float inertia_feedforward_Nm =
        2.0f * kPi * self().status_.control_acceleration *
        self().config_.inertia_feedforward;

    const float unlimited_torque_Nm =
        (self().pid_position_.Apply(
            (static_cast<int32_t>(
                (self().position_.position_relative_raw -
                 *self().status_.control_position_raw) >> 32) /
             65536.0f),
            0.0,
            measured_velocity, velocity_command,
            self().rate_config_.period_s,
            pid_options) +
         feedforward_Nm +
         inertia_feedforward_Nm);

    if (std::isfinite(self().config_.fault_position_error) &&
        std::abs(self().status_.pid_position.error) >
        self().config_.fault_position_error) {
      self().status_.mode = kFault;
      self().status_.fault = errc::kPositionControlError;
      return;
    }
    if (std::isfinite(self().config_.fault_velocity_error) &&
        std::abs(self().status_.pid_position.error_rate) >
        self().config_.fault_velocity_error) {
      self().status_.mode = kFault;
      self().status_.fault = errc::kVelocityControlError;
      return;
    }

    const auto limited_torque_Nm_pair =
        LimitCode(unlimited_torque_Nm, -max_torque_Nm, max_torque_Nm,
                  errc::kLimitMaxTorque, errc::kSuccess);
    const auto limited_torque_Nm = limited_torque_Nm_pair.first;
    if (limited_torque_Nm_pair.second != errc::kSuccess) {
      self().status_.fault = limited_torque_Nm_pair.second;
    }

    self().control_.torque_Nm = limited_torque_Nm;
    self().status_.torque_error_Nm = self().status_.torque_Nm - self().control_.torque_Nm;

    const float limited_q_A =
        torque_to_current(limited_torque_Nm *
                          self().motor_position_config()->rotor_to_output_ratio);

    {
      const auto* pos_config = self().motor_position_config();
      const auto commutation_source = pos_config->commutation_source;
      const float commutation_position =
          self().position_.sources[commutation_source].filtered_value *
          self().commutation_inv_cpr_;

      auto sample =
          [&](const auto& table, float scale) {
            const int left_index = std::min<int>(
                table.size() - 1,
                static_cast<int>(table.size() * commutation_position));
            const int right_index = (left_index + 1) % table.size();
            const float comp_fraction =
                (commutation_position -
                 static_cast<float>(left_index) / table.size()) *
                static_cast<float>(table.size());
            const float left_comp = table[left_index] * scale;
            const float right_comp = table[right_index] * scale;

            return (right_comp - left_comp) * comp_fraction + left_comp;
          };
      const float q_comp_A = sample(self().motor_.cogging_dq_comp,
                                    self().motor_.cogging_dq_scale);

      self().control_.q_comp_A = q_comp_A;
    }

    const float compensated_q_A = limited_q_A + self().control_.q_comp_A;

    const float q_A =
        is_torque_constant_configured() ?
        compensated_q_A :
        Limit(compensated_q_A, -kMaxUnconfiguredCurrent, kMaxUnconfiguredCurrent);

    // Pre-emptive regen d-axis injection.
    //
    // When regenerating and the estimated regen power exceeds
    // max_regen_power_W, inject d-axis current to dissipate the
    // excess as I^2*R in the windings.  This acts before bus voltage
    // rises, unlike voltage-feedback flux braking.
    const float preemptive_d_A = [&]() MOTEUS_CCM_ATTRIBUTE {
      if (!std::isfinite(self().config_.max_regen_power_W)) {
        return 0.0f;
      }

      const float velocity = self().position_.velocity;
      // Only active during regeneration (braking opposes motion).
      if (velocity * q_A >= 0.0f) { return 0.0f; }

      // P_regen = |q_A| * Kt * |ω_rotor|
      // velocity is in output frame; convert to rotor frame.
      // (Kt already includes the 3/2 factor for 3-phase torque.)
      const float rotor_velocity =
          velocity * self().inv_rotor_to_output_ratio_;
      const float regen_power =
          std::abs(q_A) * torque_constant_ *
          std::abs(rotor_velocity) * k2Pi;

      const float excess =
          regen_power - self().config_.max_regen_power_W;
      if (excess <= 0.0f) { return 0.0f; }

      // P_dissipated = 1.5 * d² * R  →  d = sqrt(P / (1.5 * R))
      return -std::sqrt(
          excess / (1.5f * self().motor_.resistance_ohm));
    }();

    // Voltage-feedback flux braking
    //
    // Alway use negative d-axis current.  The I^2*R dissipation is
    // the same regardless of sign, but negative d_A weakens the field
    // rather than strengthening it, which avoids consuming voltage
    // headroom at any speed and composes naturally with field
    // weakening.  Both are negative, and there is no discontinuity at
    // the base-speed crossing.
    const float fb_d_A = [&]() MOTEUS_CCM_ATTRIBUTE {
      const auto error = (
          self().status_.filt_1ms_bus_V - flux_brake_min_voltage_);

      if (error <= 0.0f) {
        return 0.0f;
      }

      return -(error / self().config_.flux_brake_resistance_ohm);
    }();

    // Use whichever is more negative (stronger).
    const float d_A = std::min(preemptive_d_A, fb_d_A);

    if (d_A != 0.0f) {
      self().status_.fault = errc::kLimitFluxBraking;
    }

    ISR_DoCurrent(
        sin_cos, d_A, q_A,
        velocity_command * self().inv_rotor_to_output_ratio_,
        data->ignore_position_bounds);
  }

  void ISR_DoStayWithinBounds(const SinCos& sin_cos,
                               CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    const auto target_position = [&]() MOTEUS_CCM_ATTRIBUTE -> std::optional<float> {
      if (!std::isnan(data->bounds_min) &&
          self().position_.position < data->bounds_min) {
        return data->bounds_min;
      }
      if (!std::isnan(data->bounds_max) &&
          self().position_.position > data->bounds_max) {
        return data->bounds_max;
      }
      return {};
    }();

    if (!target_position) {
      self().status_.pid_position.Clear();
      self().status_.control_position_raw = {};
      self().status_.control_position = std::numeric_limits<float>::quiet_NaN();
      self().status_.control_velocity = {};
      self().status_.control_acceleration = {};

      PID::ApplyOptions apply_options;
      apply_options.kp_scale = 0.0;
      apply_options.kd_scale = 0.0;
      apply_options.ilimit_scale = 0.0;

      ISR_DoPositionCommon(
          sin_cos, data, apply_options,
          data->max_torque_Nm, data->feedforward_Nm, 0.0f);
      return;
    }

    // Control position to whichever bound we are currently violating.
    PID::ApplyOptions apply_options;
    apply_options.kp_scale = data->kp_scale;
    apply_options.kd_scale = data->kd_scale;
    apply_options.ilimit_scale = data->ilimit_scale;

    const int64_t absolute_relative_delta =
        self().absolute_relative_delta() << 32ll;
    data->position_relative_raw =
        MotorPosition::FloatToInt(*target_position) -
        absolute_relative_delta;
    data->velocity = 0.0;
    self().status_.control_position_raw = data->position_relative_raw;
    self().status_.control_position = *target_position;
    self().status_.control_velocity = 0.0f;
    self().status_.control_acceleration = 0.0f;

    ISR_DoPositionCommon(
        sin_cos, data, apply_options,
        data->max_torque_Nm, data->feedforward_Nm, 0.0f);
  }

  void ISR_DoVoltageFOC(BldcServoCommandData* data) MOTEUS_CCM_ATTRIBUTE {
    data->theta += data->theta_rate * self().rate_config_.period_s;
    SinCos sc = self().cordic(RadiansToQ31(data->theta));
    const float max_voltage = (0.5f - self().rate_config_.min_pwm) *
        self().status_.filt_bus_V * kSvpwmRatio;
    InverseDqTransform idt(sc,
        Limit(data->voltage, -max_voltage, max_voltage), 0);
    ISR_DoBalancedVoltageControl(Vec3{idt.a, idt.b, idt.c});
  }

  void ISR_DoStopped(const SinCos& sin_cos) MOTEUS_CCM_ATTRIBUTE {
    if (self().status_.cooldown_count) {
      self().status_.cooldown_count--;
      ISR_DoCurrent(sin_cos, 0.0f, 0.0f, 0.0f, false);
      return;
    }

    self().DoHardStop();

    self().status_.power_W = 0.0f;
    self().status_.fw.id_A = 0.0f;
  }

  void ISR_DoPositionTimeout(const SinCos& sin_cos,
                              CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    if (self().config_.timeout_mode == kStopped) {
      ISR_DoStopped(sin_cos);
    } else if (self().config_.timeout_mode == kPosition) {
      CommandData timeout_data;
      timeout_data.mode = kPosition;
      timeout_data.position = std::numeric_limits<float>::quiet_NaN();
      timeout_data.velocity_limit = self().config_.default_velocity_limit;
      timeout_data.accel_limit = self().config_.default_accel_limit;
      timeout_data.timeout_s = std::numeric_limits<float>::quiet_NaN();

      PID::ApplyOptions apply_options;
      ISR_DoPositionCommon(
          sin_cos, &timeout_data, apply_options,
          timeout_data.max_torque_Nm,
          0.0f,
          0.0f);
    } else if (self().config_.timeout_mode == kZeroVelocity) {
      ISR_DoZeroVelocity(sin_cos, data);
    } else if (self().config_.timeout_mode == kBrake) {
      self().DoBrake();
    } else {
      ISR_DoStopped(sin_cos);
    }
  }

  void ISR_DoMeasureInductance(const SinCos& sin_cos,
                                CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    const int8_t old_sign = self().status_.meas_ind_phase > 0 ? 1 : -1;
    const float old_sign_float = old_sign > 0 ? 1.0f : -1.0f;

    self().status_.meas_ind_phase += -old_sign;

    if (self().status_.meas_ind_phase == 0) {
      self().status_.meas_ind_phase = -old_sign * data->meas_ind_period;
    }

    const float offset = std::isfinite(data->fixed_voltage_override) ?
        data->fixed_voltage_override :
        0.0f;

    const float max_V = (0.5f - self().rate_config_.min_pwm) *
        self().status_.filt_bus_V * kSvpwmRatio;

    const float excitation =
        data->d_V * (self().status_.meas_ind_phase > 0 ? 1.0f : -1.0f);

    if (data->meas_ind_axis == 0) {
      // D-axis excitation (default behavior).
      const float d_V = Limit(offset + excitation, -max_V, max_V);

      self().status_.meas_ind_integrator +=
          (self().status_.d_A - self().status_.meas_ind_old_d_A) *
          old_sign_float;
      self().status_.meas_ind_old_d_A = self().status_.d_A;

      ISR_DoBalancedVoltageControl(
          ISR_CalculatePhaseVoltage(sin_cos, d_V, 0.0f));
    } else {
      // Q-axis excitation: apply excitation on q-axis, keep d-axis
      // offset to hold rotor position.
      const float d_V = Limit(offset, -max_V, max_V);
      const float q_V = Limit(excitation, -max_V, max_V);

      self().status_.meas_ind_integrator +=
          (self().status_.q_A - self().status_.meas_ind_old_q_A) *
          old_sign_float;
      self().status_.meas_ind_old_q_A = self().status_.q_A;

      ISR_DoBalancedVoltageControl(
          ISR_CalculatePhaseVoltage(sin_cos, d_V, q_V));
    }
  }

  void ISR_StartCalibrating() MOTEUS_CCM_ATTRIBUTE {
    isr_motor_position_epoch_ = self().position_.epoch;
    self().status_.mode = kEnabling;
    self().StartCalibrating();
  }

  void ISR_MaybeChangeMode(CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    switch (data->mode) {
      case kNumModes:
      case kFault:
      case kCalibrating:
      case kCalibrationComplete: {
        MJ_ASSERT(false);
        return;
      }
      case kStopped: {
        self().status_.mode = kStopped;
        return;
      }
      case kEnabling: {
        return;
      }
      case kPwm:
      case kVoltage:
      case kVoltageFoc:
      case kVoltageDq:
      case kCurrent:
      case kPosition:
      case kPositionTimeout:
      case kZeroVelocity:
      case kStayWithinBounds:
      case kMeasureInductance:
      case kBrake: {
        switch (self().status_.mode) {
          case kNumModes: {
            MJ_ASSERT(false);
            return;
          }
          case kFault: {
            return;
          }
          case kStopped: {
            ISR_StartCalibrating();
            return;
          }
          case kEnabling:
          case kCalibrating: {
            return;
          }
          case kCalibrationComplete: {
            // We only care if the epoch changes while we are actively
            // controlling.  By definition, if we are just exiting the
            // pre-startup calibration phase, we are not yet actively
            // controlling.  Thus any epoch changes that happen before
            // then are "not our problem".
            isr_motor_position_epoch_ = self().position_.epoch;
            [[fallthrough]];
          }
          case kPwm:
          case kVoltage:
          case kVoltageFoc:
          case kVoltageDq:
          case kCurrent:
          case kPosition:
          case kZeroVelocity:
          case kStayWithinBounds:
          case kMeasureInductance:
          case kBrake: {
            if ((data->mode == kPosition || data->mode == kStayWithinBounds) &&
                !data->ignore_position_bounds &&
                ISR_IsOutsideLimits()) {
              self().status_.mode = kFault;
              self().status_.fault = errc::kStartOutsideLimit;
            } else if ((data->mode == kPosition ||
                        data->mode == kStayWithinBounds) &&
                       !data->ignore_position_bounds &&
                       ISR_InvalidLimits()) {
              self().status_.mode = kFault;
              self().status_.fault = errc::kInvalidLimits;
            } else {
              self().status_.mode = data->mode;
              ISR_ClearPid(kAlwaysClear);
            }

            if (data->mode == kMeasureInductance) {
              self().status_.meas_ind_phase = 0;
              self().status_.meas_ind_integrator = 0.0f;
              self().status_.meas_ind_old_d_A = self().status_.d_A;
              self().status_.meas_ind_old_q_A = self().status_.q_A;
            }

            return;
          }
          case kPositionTimeout: {
            return;
          }
        }
      }
    }
  }

  void ISR_DoControl(const SinCos& sin_cos,
                     CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    old_d_V_ = self().control_.d_V;
    old_q_V_ = self().control_.q_V;

    self().control_.Clear();

    if (!std::isnan(self().status_.timeout_s) && self().status_.timeout_s > 0.0f) {
      self().status_.timeout_s =
          std::max(0.0f, self().status_.timeout_s - self().rate_config_.period_s);
    }

    // See if we need to update our current mode.
    if (data->mode != self().status_.mode) {
      ISR_MaybeChangeMode(data);
    }

    // Handle our persistent fault conditions.
    if (self().status_.mode != kStopped && self().status_.mode != kFault) {
      if (self().motor_driver_fault()) {
        self().status_.mode = kFault;
        self().status_.fault = errc::kMotorDriverFault;
      }
      if (self().status_.bus_V > self().config_.max_voltage) {
        self().status_.mode = kFault;
        self().status_.fault = errc::kOverVoltage;
      }
      if (self().status_.bus_V < 4.0f) {
        self().status_.mode = kFault;
        self().status_.fault = errc::kUnderVoltage;
      }
      if (self().status_.filt_fet_temp_C > self().config_.fault_temperature) {
        self().status_.mode = kFault;
        self().status_.fault = errc::kOverTemperature;
      }
      if (std::isfinite(self().config_.motor_fault_temperature) &&
          self().status_.filt_motor_temp_C > self().config_.motor_fault_temperature) {
        self().status_.mode = kFault;
        self().status_.fault = errc::kOverTemperature;
      }
    }

    if ((self().status_.mode == kPosition || self().status_.mode == kStayWithinBounds) &&
        !std::isnan(self().status_.timeout_s) &&
        self().status_.timeout_s <= 0.0f) {
      self().status_.mode = kPositionTimeout;
    }

    // Ensure unused PID controllers have zeroed state.
    ISR_ClearPid(kClearIfMode);

    if (self().status_.mode != kFault) {
      self().status_.fault = errc::kSuccess;
    }

    if (current_control()) {
      self().status_.cooldown_count = self().config_.cooldown_cycles;
    }

    switch (self().status_.mode) {
      case kNumModes:
      case kStopped: {
        ISR_DoStopped(sin_cos);
        break;
      }
      case kFault: {
        self().DoFault();
        break;
      }
      case kEnabling: {
        break;
      }
      case kCalibrating: {
        self().DoCalibrating();
        break;
      }
      case kCalibrationComplete: {
        break;
      }
      case kPwm: {
        ISR_DoPwmControl(data->pwm);
        break;
      }
      case kVoltage: {
        ISR_DoBalancedVoltageControl(data->phase_v);
        break;
      }
      case kVoltageFoc: {
        ISR_DoVoltageFOC(data);
        break;
      }
      case kVoltageDq: {
        ISR_DoVoltageDQCommand(sin_cos, data->d_V, data->q_V);
        break;
      }
      case kCurrent: {
        ISR_DoCurrent(sin_cos, data->i_d_A, data->i_q_A, 0.0f,
                      data->ignore_position_bounds);
        break;
      }
      case kPosition: {
        ISR_DoPosition(sin_cos, data);
        break;
      }
      case kPositionTimeout: {
        ISR_DoPositionTimeout(sin_cos, data);
        break;
      }
      case kZeroVelocity: {
        ISR_DoZeroVelocity(sin_cos, data);
        break;
      }
      case kStayWithinBounds: {
        ISR_DoStayWithinBounds(sin_cos, data);
        break;
      }
      case kMeasureInductance: {
        ISR_DoMeasureInductance(sin_cos, data);
        break;
      }
      case kBrake: {
        self().DoBrake();
        break;
      }
    }
  }

 protected:
  Impl& self() { return static_cast<Impl&>(*this); }
  const Impl& self() const { return static_cast<const Impl&>(*this); }

  void InitControlFilters(float pwm_rate_hz) {
    max_velocity_filter_ = ExponentialFilter(pwm_rate_hz, 10.0f);
    base_velocity_filter_ = ExponentialFilter(pwm_rate_hz, 5.0f);
    fw_id_filter_ = ExponentialFilter(
        pwm_rate_hz, self().config_.fw.bandwidth_hz);
    slow_bus_v_filter_ = ExponentialFilter(pwm_rate_hz, 2.0f);
    fast_bus_v_filter_ = ExponentialFilter(pwm_rate_hz, 1000.0f);
  }

  float v_per_hz_ = 0.0f;
  float lambda_m_ = 0.0f;

  // Precomputed constants used by the FW base velocity quadratic in
  // ISR_CalculateDerivedQuantities.
  float v_per_hz_squared_ = 0.0f;           // v_per_hz^2
  float pi_poles_Lq_ = 0.0f;                // kPi * poles * inductance_q_H
  float two_R_v_per_hz_ = 0.0f;             // 2 * R * v_per_hz
  float R_squared_ = 0.0f;                  // R^2
  float fw_V_eff_factor_ = 0.0f;            // max_voltage_ratio * target_modulation * 0.5
  float half_max_voltage_ratio_over_v_per_hz_ = 0.0f;  // max_voltage_ratio * 0.5 / v_per_hz
  float max_V_factor_ = 0.0f;               // max_voltage_ratio * kSvpwmRatio * 0.5 (multiplied by filt_bus_V for max_V)
  float fw_max_current_A_ = 0.0f;           // fw.max_current_ratio * max_current_A
  float fw_id_char_at_max_current_ = 0.0f;  // lambda_m / L_d(-fw_max_current_A_)
  float half_over_R_ = 0.0f;                // 0.5 / resistance_ohm (multiplied by filt_bus_V for Imax voltage limit)
  float board_max_velocity_factor_ = 0.0f;  // 0.5 * (max_voltage - kBoardVoltageMargin) / v_per_hz (multiplied by rotor_to_output_ratio for board_max_velocity)
  float commutation_inv_cpr_ = 0.0f;        // 1 / commutation source cpr (avoids u32->float convert and divide every ISR)
  // Cached torque model — reconstructed in UpdateDerivedMotorConstants
  // so the per-call setup (including reciprocals) stays out of the
  // ISR hot path.
  TorqueModel torque_model_;

  // 1 / rotor_to_output_ratio — computed once per ISR in
  // ISR_CalculateDerivedQuantities and reused by later stages that
  // would otherwise divide by rotor_to_output_ratio repeatedly.
  // Not precomputed at config time because motor_position config
  // changes do not trigger UpdateDerivedMotorConstants.
  float inv_rotor_to_output_ratio_ = 0.0f;

  SimplePI::Config pid_d_config_;
  SimplePI::Config pid_q_config_;
  float pid_dq_w_ = 0.0f;

  ExponentialFilter max_velocity_filter_;
  ExponentialFilter base_velocity_filter_;
  ExponentialFilter fw_id_filter_;
  ExponentialFilter slow_bus_v_filter_;
  ExponentialFilter fast_bus_v_filter_;

  // Previous ISR's applied d/q voltages, used for the power
  // accounting that runs one step behind the current command.
  float old_d_V_ = 0.0f;
  float old_q_V_ = 0.0f;
};

}  // namespace moteus
