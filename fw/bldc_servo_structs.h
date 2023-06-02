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

#include <cstdint>
#include <optional>

#include "mjlib/base/visitor.h"

#include "fw/error.h"
#include "fw/measured_hw_rev.h"
#include "fw/pid.h"
#include "fw/simple_pi.h"

namespace moteus {

struct Vec3 {
  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;

  template <typename Archive>
  void Serialize(Archive* ar) {
    ar->Visit(MJ_NVP(a));
    ar->Visit(MJ_NVP(b));
    ar->Visit(MJ_NVP(c));
  }
};

enum BldcServoMode {
  // In this mode, the entire motor driver will be disabled.
  //
  // When exiting this state, the current offset will be
  // recalibrated.
  kStopped = 0,

  // This stage cannot be commanded directly, but will be entered
  // upon any fault.  Here, the motor driver remains enabled, but
  // the output stage power is removed.  The only valid transition
  // from this state is to kStopped.
  kFault = 1,

  // This mode may not be commanded directly.  It is used when
  // transitioning from kStopped to another mode.
  kEnabling = 2,

  // This mode may not be commanded directly, but is used when
  // transitioning from kStopped to another mode.
  kCalibrating = 3,

  // This mode may not be commanded directly, but is used when
  // transitioning from kStopped to another mode.
  kCalibrationComplete = 4,

  // Directly control the PWM of all 3 phases.
  kPwm = 5,

  // Control the voltage of all three phases
  kVoltage = 6,

  // Control the phase and voltage magnitude
  kVoltageFoc = 7,

  // Control d and q voltage
  kVoltageDq = 8,

  // Control d and q current
  kCurrent = 9,

  // Control absolute position
  kPosition = 10,

  // This state can be commanded directly, and will also be entered
  // automatically upon a watchdog timeout from kPosition.  When in
  // this state, the controller will apply the selected fallback
  // control mode.
  //
  // The only way to exit this state is through a stop command.
  kPositionTimeout = 11,

  // Control to zero velocity through a derivative only version of
  // the position mode.
  kZeroVelocity = 12,

  // This applies the PID controller only to stay within a
  // particular position region, and applies 0 torque when within
  // that region.
  kStayWithinBounds = 13,

  // This mode applies a fixed voltage square waveform in the D axis
  // in order to measure inductance assuming a motor with
  // approximately equal D/Q axis inductances.
  kMeasureInductance = 14,

  // All phases are pulled to ground.
  kBrake = 15,

  kNumModes,
};

struct BldcServoStatus {
  BldcServoMode mode = kStopped;
  errc fault = errc::kSuccess;

  uint16_t adc_cur1_raw = 0;
  uint16_t adc_cur2_raw = 0;
  uint16_t adc_cur3_raw = 0;
  uint16_t adc_voltage_sense_raw = 0;
  uint16_t adc_fet_temp_raw = 0;
  uint16_t adc_motor_temp_raw = 0;

  uint16_t adc_cur1_offset = 2048;
  uint16_t adc_cur2_offset = 2048;
  uint16_t adc_cur3_offset = 2048;

  float cur1_A = 0.0f;
  float cur2_A = 0.0f;
  float cur3_A = 0.0f;

  float bus_V = 0.0f;
  float filt_bus_V = std::numeric_limits<float>::quiet_NaN();
  float filt_1ms_bus_V = std::numeric_limits<float>::quiet_NaN();
  float fet_temp_C = 0.0f;
  float filt_fet_temp_C = std::numeric_limits<float>::quiet_NaN();
  float motor_temp_C = 0.0f;
  float filt_motor_temp_C = std::numeric_limits<float>::quiet_NaN();

  float d_A = 0.0f;
  float q_A = 0.0f;

  float position = 0.0f;
  float velocity = 0.0f;
  float torque_Nm = 0.0f;

  float velocity_filt = 0.0f;

  SimplePI::State pid_d;
  SimplePI::State pid_q;
  PID::State pid_position;

  // This is measured in the same units as MotorPosition's integral
  // units, which is 48 bits to represent 1.0 unit of output
  // revolution.
  std::optional<int64_t> control_position_raw;
  float control_position = std::numeric_limits<float>::quiet_NaN();
  std::optional<float> control_velocity;
  float position_to_set = std::numeric_limits<float>::quiet_NaN();
  float timeout_s = 0.0;
  bool trajectory_done = false;

  float torque_error_Nm = 0.0f;

  float sin = 0.0f;
  float cos = 0.0f;
  uint16_t cooldown_count = 0;
  uint32_t final_timer = 0;
  uint32_t total_timer = 0;

  float meas_ind_old_d_A = 0.0f;
  int8_t meas_ind_phase = 0;
  float meas_ind_integrator = 0.0f;

#ifdef MOTEUS_PERFORMANCE_MEASURE
  struct Dwt {
    uint32_t adc_done = 0;
    uint32_t start_pos_sample = 0;
    uint32_t done_pos_sample = 0;
    uint32_t done_temp_sample = 0;
    uint32_t sense = 0;
    uint32_t curstate = 0;
    uint32_t control_sel_mode = 0;
    uint32_t control_done_pos = 0;
    uint32_t control_done_cur = 0;
    uint32_t control = 0;
    uint32_t done = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(adc_done));
      a->Visit(MJ_NVP(start_pos_sample));
      a->Visit(MJ_NVP(done_pos_sample));
      a->Visit(MJ_NVP(done_temp_sample));
      a->Visit(MJ_NVP(sense));
      a->Visit(MJ_NVP(curstate));
      a->Visit(MJ_NVP(control_sel_mode));
      a->Visit(MJ_NVP(control_done_pos));
      a->Visit(MJ_NVP(control_done_cur));
      a->Visit(MJ_NVP(control));
      a->Visit(MJ_NVP(done));
    }
  };

  Dwt dwt;
#endif

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(mode));
    a->Visit(MJ_NVP(fault));

    a->Visit(MJ_NVP(adc_cur1_raw));
    a->Visit(MJ_NVP(adc_cur2_raw));
    a->Visit(MJ_NVP(adc_cur3_raw));
    a->Visit(MJ_NVP(adc_voltage_sense_raw));
    a->Visit(MJ_NVP(adc_fet_temp_raw));
    a->Visit(MJ_NVP(adc_motor_temp_raw));

    a->Visit(MJ_NVP(adc_cur1_offset));
    a->Visit(MJ_NVP(adc_cur2_offset));
    a->Visit(MJ_NVP(adc_cur3_offset));

    a->Visit(MJ_NVP(cur1_A));
    a->Visit(MJ_NVP(cur2_A));
    a->Visit(MJ_NVP(cur3_A));

    a->Visit(MJ_NVP(bus_V));
    a->Visit(MJ_NVP(filt_bus_V));
    a->Visit(MJ_NVP(filt_1ms_bus_V));
    a->Visit(MJ_NVP(fet_temp_C));
    a->Visit(MJ_NVP(filt_fet_temp_C));
    a->Visit(MJ_NVP(motor_temp_C));
    a->Visit(MJ_NVP(filt_motor_temp_C));

    a->Visit(MJ_NVP(d_A));
    a->Visit(MJ_NVP(q_A));

    a->Visit(MJ_NVP(position));
    a->Visit(MJ_NVP(velocity));
    a->Visit(MJ_NVP(torque_Nm));

    a->Visit(MJ_NVP(velocity_filt));

    a->Visit(MJ_NVP(pid_d));
    a->Visit(MJ_NVP(pid_q));
    a->Visit(MJ_NVP(pid_position));

    a->Visit(MJ_NVP(control_position_raw));
    a->Visit(MJ_NVP(control_position));
    a->Visit(MJ_NVP(control_velocity));
    a->Visit(MJ_NVP(position_to_set));
    a->Visit(MJ_NVP(timeout_s));
    a->Visit(MJ_NVP(trajectory_done));

    a->Visit(MJ_NVP(torque_error_Nm));

    a->Visit(MJ_NVP(sin));
    a->Visit(MJ_NVP(cos));
    a->Visit(MJ_NVP(cooldown_count));
    a->Visit(MJ_NVP(final_timer));
    a->Visit(MJ_NVP(total_timer));

    a->Visit(MJ_NVP(meas_ind_old_d_A));
    a->Visit(MJ_NVP(meas_ind_phase));
    a->Visit(MJ_NVP(meas_ind_integrator));

#ifdef MOTEUS_PERFORMANCE_MEASURE
    a->Visit(MJ_NVP(dwt));
#endif
  }
};

struct BldcServoCommandData {
  BldcServoMode mode = kStopped;

  // For kPwm mode.
  Vec3 pwm;  // 0-1.0

  // For kVoltage mode
  Vec3 phase_v;

  // For kVoltageFoc
  float theta = 0.0f;
  float voltage = 0.0f;
  float theta_rate = 0.0f;

  // For kVoltageDq and kMeasureInductance
  float d_V = 0.0f;
  float q_V = 0.0f;

  // For kFoc mode.
  float i_d_A = 0.0f;
  float i_q_A = 0.0f;

  // For kPosition mode
  float position = 0.0f;  // kNaN means start at the current position.
  float velocity = 0.0f;

  // This should not be set by callers, but is used internally.
  std::optional<int64_t> position_relative_raw;

  float max_torque_Nm = 100.0f;
  float stop_position = std::numeric_limits<float>::quiet_NaN();

  // This should not be set by callers, but is used internally.
  std::optional<int64_t> stop_position_relative_raw;

  float feedforward_Nm = 0.0f;

  float kp_scale = 1.0f;
  float kd_scale = 1.0f;

  float velocity_limit = std::numeric_limits<float>::quiet_NaN();
  float accel_limit = std::numeric_limits<float>::quiet_NaN();

  // If not NaN, temporarily operate in fixed voltage mode.
  float fixed_voltage_override = std::numeric_limits<float>::quiet_NaN();

  float timeout_s = 0.0f;

  // For kStayWithinBounds
  float bounds_min = 0.0f;
  float bounds_max = 0.0f;

  // For kMeasureInductance
  int8_t meas_ind_period = 4;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(mode));

    a->Visit(MJ_NVP(pwm));

    a->Visit(MJ_NVP(phase_v));

    a->Visit(MJ_NVP(theta));
    a->Visit(MJ_NVP(voltage));
    a->Visit(MJ_NVP(theta_rate));

    a->Visit(MJ_NVP(d_V));
    a->Visit(MJ_NVP(q_V));

    a->Visit(MJ_NVP(i_d_A));
    a->Visit(MJ_NVP(i_q_A));

    a->Visit(MJ_NVP(position));
    a->Visit(MJ_NVP(velocity));
    a->Visit(MJ_NVP(position_relative_raw));
    a->Visit(MJ_NVP(max_torque_Nm));
    a->Visit(MJ_NVP(stop_position));
    a->Visit(MJ_NVP(stop_position_relative_raw));
    a->Visit(MJ_NVP(feedforward_Nm));
    a->Visit(MJ_NVP(kp_scale));
    a->Visit(MJ_NVP(kd_scale));
    a->Visit(MJ_NVP(velocity_limit));
    a->Visit(MJ_NVP(accel_limit));
    a->Visit(MJ_NVP(fixed_voltage_override));
    a->Visit(MJ_NVP(timeout_s));
    a->Visit(MJ_NVP(bounds_min));
    a->Visit(MJ_NVP(bounds_max));
    a->Visit(MJ_NVP(meas_ind_period));
  }
};

struct BldcServoMotor {
  uint8_t poles = 0;

  // Invert the order of phase movement.
  uint8_t phase_invert = 0;

  float resistance_ohm = 0.0f;

  // Hz is electrical
  float v_per_hz = 0.0f;

  // Electrical phase offset in radians as a function of encoder
  // position.
  std::array<float, 64> offset = {};

  // These control the higher order motor torque model.
  //
  // When above the cutoff current, the torque is calculated as:
  //
  //   cutoff * torque_constant + tscale * log2(1 + (I - cutoff) * iscale)
  //
  // This models the "rotation" region of magnetic permeability.  By
  // default, the cutoff current is set unreasonably large, so that
  // the controller will not use this region to estimate torque.
  float rotation_current_cutoff_A = 10000.0;
  float rotation_current_scale = 0.05;
  float rotation_torque_scale = 14.7;

  // When in position mode, the cogging DQ table provides an
  // additional Q current to apply based on the commutation encoder.
  // Each of the values in cogging_dq_comp is multiplied by
  // cogging_dq_scale to arrive at the final correction current.
  float cogging_dq_scale = 0.0f;
  std::array<int8_t, 1024> cogging_dq_comp = {{}};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(poles));
    a->Visit(MJ_NVP(phase_invert));
    a->Visit(MJ_NVP(resistance_ohm));
    a->Visit(MJ_NVP(v_per_hz));
    a->Visit(MJ_NVP(offset));
    a->Visit(MJ_NVP(rotation_current_cutoff_A));
    a->Visit(MJ_NVP(rotation_current_scale));
    a->Visit(MJ_NVP(rotation_torque_scale));
    a->Visit(MJ_NVP(cogging_dq_scale));
    a->Visit(MJ_NVP(cogging_dq_comp));
  }
};

struct BldcServoConfig {
  uint16_t pwm_rate_hz =
      (g_measured_hw_family == 0 &&
       g_measured_hw_rev <= 2) ? 60000 :
      30000;

  float i_gain = 20.0f;  // should match csa_gain from drv8323
  float current_sense_ohm = 0.0005f;

  // PWM rise time compensation
  float pwm_comp_off =
      g_measured_hw_family == 0 ?
       ((g_measured_hw_rev <= 6) ? 0.015f :
        (g_measured_hw_rev <= 7) ? 0.055f :
        0.027f) :
      g_measured_hw_family == 1 ?
       0.027f :
      invalid_float()
      ;
  float pwm_comp_mag =
      g_measured_hw_family == 0 ?
       ((g_measured_hw_rev <= 6) ? 0.005f :
        (g_measured_hw_rev <= 7) ? 0.005f :
        0.005f) :
      g_measured_hw_family == 1 ?
       0.005f :
       invalid_float()
      ;
  float pwm_scale = 1.0f;

  // We pick a default maximum voltage based on the board revision.
  float max_voltage =
      g_measured_hw_family == 0 ?
      ((g_measured_hw_rev <= 5) ? 37.0f : 46.0f) :
      g_measured_hw_family == 1 ?
      56.0f :
      invalid_float()
      ;
  float max_power_W = 450.0f;

  float derate_temperature = 50.0f;
  float fault_temperature = 75.0f;

  // If enabled, slightly more instructions are used per cycle, but
  // the motor temperature will be available for throttling in
  // addition to the FET temperature.
  bool enable_motor_temperature = false;
  float motor_derate_temperature = 50.0f;
  float motor_fault_temperature = std::numeric_limits<float>::quiet_NaN();

  float velocity_threshold = 0.0f;
  float position_derate = 0.02f;

  // The current ADCs are driven by the drv8323's op-amp, which is
  // very low impedance.  They thus don't need a lot of sampling
  // time.  (They are on "slow" pins though, which means we can't
  // get away with minimal sampling).
  uint16_t adc_cur_cycles = 2;  // 2, 6, 12, 24, 47, 92, 247, 640

  // However, the aux channels, (voltage and temperature), are just
  // driven by resistor networks (relatively high impedance).  They
  // need a larger sampling time.
  uint16_t adc_aux_cycles = 47;

  // We use the same PID constants for D and Q current control
  // loops.
  SimplePI::Config pid_dq;
  PID::Config pid_position;

  // Use the configured motor resistance to apply a feedforward phase
  // voltage based on the desired current.
  float current_feedforward = 1.0f;

  // Use the configured motor Kv rating to apply a feedforward voltage
  // based on the desired angular velocity.
  float bemf_feedforward = 1.0f;

  // Default values for the position mode velocity and acceleration
  // limits.
  float default_velocity_limit = std::numeric_limits<float>::quiet_NaN();
  float default_accel_limit = std::numeric_limits<float>::quiet_NaN();

  // If true, then the currents in A that are calculated for the D
  // and Q phase are instead directly commanded as voltages on the
  // phase terminals.  This is primarily useful for high resistance
  // motors like gimbal motors when the sense resistors are
  // configured for a low resistance motor.
  bool voltage_mode_control = false;

  // If true, then the controller acts as a cheap gimbal controller
  // and does not use the encoder at all.  Instead, the desired
  // position is used to command an open loop fixed voltage.
  bool fixed_voltage_mode = false;
  float fixed_voltage_control_V = 0.0f;

  float max_position_slip = std::numeric_limits<float>::quiet_NaN();

  float default_timeout_s = 0.1f;
  float timeout_max_torque_Nm = 5.0f;

  // Selects the behavior when in the timeout mode.  The available
  // options map to top level modes, although only the following are
  // valid:
  //  0 - "stopped" - motor driver disengaged
  //  12 - "zero velocity" - derivative only position control
  //  15 - "brake" - all motor phases shorted to ground
  uint8_t timeout_mode = 12;

  // Similar to 'max_voltage', the flux braking default voltage is
  // board rev dependent.
  float flux_brake_min_voltage =
      g_measured_hw_family == 0 ?
      ((g_measured_hw_rev <= 5) ? 34.5f : 43.5f) :
      g_measured_hw_family == 1 ?
      53.0f :
      invalid_float();
  float flux_brake_resistance_ohm = 0.025f;

  float max_current_A = 100.0f;
  float derate_current_A = -20.0f;

  // When the maximum velocity exceeds this value, a current limit
  // will begin to be applied.  When it reaches max_velocity +
  // max_velocity_derate, the maximum allowed current will be 0.
  float max_velocity = 500.0;
  float max_velocity_derate = 2.0;

  uint16_t cooldown_cycles = 256;

  // When starting position control from the "stopped" state, the
  // control velocity will be initialized from 'velocity_filt'.  If
  // the absolute value is less than this, then it will be treated as
  // exactly 0.0.
  float velocity_zero_capture_threshold = 0.05f;

  // A bitfield that selects one of several things to emit from the
  // debug UART at full control rate.
  uint32_t emit_debug = 0;

  BldcServoConfig() {
    pid_dq.kp = 0.005f;
    pid_dq.ki = 30.0f;

    pid_position.kp = 4.0f;
    pid_position.ki = 1.0f;
    pid_position.ilimit = 0.0f;
    pid_position.kd = 0.05f;
    pid_position.sign = -1.0f;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(pwm_rate_hz));
    a->Visit(MJ_NVP(i_gain));
    a->Visit(MJ_NVP(current_sense_ohm));
    a->Visit(MJ_NVP(pwm_comp_off));
    a->Visit(MJ_NVP(pwm_comp_mag));
    a->Visit(MJ_NVP(pwm_scale));
    a->Visit(MJ_NVP(max_voltage));
    a->Visit(MJ_NVP(max_power_W));
    a->Visit(MJ_NVP(derate_temperature));
    a->Visit(MJ_NVP(fault_temperature));
    a->Visit(MJ_NVP(enable_motor_temperature));
    a->Visit(MJ_NVP(motor_derate_temperature));
    a->Visit(MJ_NVP(motor_fault_temperature));
    a->Visit(MJ_NVP(velocity_threshold));
    a->Visit(MJ_NVP(position_derate));
    a->Visit(MJ_NVP(adc_cur_cycles));
    a->Visit(MJ_NVP(adc_aux_cycles));
    a->Visit(MJ_NVP(pid_dq));
    a->Visit(MJ_NVP(pid_position));
    a->Visit(MJ_NVP(current_feedforward));
    a->Visit(MJ_NVP(bemf_feedforward));
    a->Visit(MJ_NVP(default_velocity_limit));
    a->Visit(MJ_NVP(default_accel_limit));
    a->Visit(MJ_NVP(voltage_mode_control));
    a->Visit(MJ_NVP(fixed_voltage_mode));
    a->Visit(MJ_NVP(fixed_voltage_control_V));
    a->Visit(MJ_NVP(max_position_slip));
    a->Visit(MJ_NVP(default_timeout_s));
    a->Visit(MJ_NVP(timeout_max_torque_Nm));
    a->Visit(MJ_NVP(timeout_mode));
    a->Visit(MJ_NVP(flux_brake_min_voltage));
    a->Visit(MJ_NVP(flux_brake_resistance_ohm));
    a->Visit(MJ_NVP(max_current_A));
    a->Visit(MJ_NVP(derate_current_A));
    a->Visit(MJ_NVP(max_velocity));
    a->Visit(MJ_NVP(max_velocity_derate));
    a->Visit(MJ_NVP(cooldown_cycles));
    a->Visit(MJ_NVP(velocity_zero_capture_threshold));
    a->Visit(MJ_NVP(emit_debug));
  }

  static float invalid_float() {
    MJ_ASSERT(false);
    return 0.0f;
  }
};

// This will commonly be different across every device, so it is
// separate to minimize resets due to schemas changing during
// development.
struct BldcServoPositionConfig {
  float position_min = -0.01f;
  float position_max = 0.01f;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(position_min));
    a->Visit(MJ_NVP(position_max));
  }
};

}

namespace mjlib {
namespace base {

template <>
struct IsEnum<moteus::BldcServoMode> {
  static constexpr bool value = true;

  using M = moteus::BldcServoMode;
  static std::array<std::pair<M, const char*>, M::kNumModes> map() {
    return { {
        { M::kStopped, "stopped" },
        { M::kFault, "fault" },
        { M::kEnabling, "enabling" },
        { M::kCalibrating, "calibrating" },
        { M::kCalibrationComplete, "calib_complete" },
        { M::kPwm, "pwm" },
        { M::kVoltage, "voltage" },
        { M::kVoltageFoc, "voltage_foc" },
        { M::kVoltageDq, "voltage_dq" },
        { M::kCurrent, "current" },
        { M::kPosition, "position" },
        { M::kPositionTimeout, "pos_timeout" },
        { M::kZeroVelocity, "zero_vel" },
        { M::kStayWithinBounds, "within" },
        { M::kMeasureInductance, "meas_ind" },
        { M::kBrake, "brake" },
      }};
  }
};

}
}
