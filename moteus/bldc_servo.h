// Copyright 2018 Josh Pieper, jjp@pobox.com.
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

#include "PinNames.h"

#include "mjlib/base/pid.h"
#include "mjlib/base/visitor.h"

#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

#include "moteus/error.h"
#include "moteus/motor_driver.h"
#include "moteus/position_sensor.h"

namespace moteus {

/// Implements a closed loop servo around a brushless DC motor.
class BldcServo {
 public:
  struct Options {
    // These three pins must be on the same timer, and one that
    // supports center aligned PWM.
    PinName pwm1 = NC;
    PinName pwm2 = NC;
    PinName pwm3 = NC;

    PinName current1 = NC;  // Must be sampled from ADC1
    PinName current2 = NC;  // Must be sampled from ADC2

    PinName current3 = NC;
    PinName vsense = NC;  // Must be sampled from ADC3
    PinName tsense = NC;  // Must be sampled from ADC3

    PinName debug_out = NC;

    // If set, a constant telemetry stream will be emitted at 40kHz.
    PinName debug_uart_out = NC;
  };

  BldcServo(mjlib::micro::Pool*,
            mjlib::micro::PersistentConfig*,
            mjlib::micro::TelemetryManager*,
            PositionSensor*,
            MotorDriver*,
            const Options&);
  ~BldcServo();

  void PollMillisecond();

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

  struct Motor {
    uint8_t poles = 0;  // 14
    uint8_t invert = 0;
    float resistance_ohm = 0.0f;  // 0.030

    // Hz is electrical
    float v_per_hz = 0.0f;  // 0.15f / 5.0f;

    float unwrapped_position_scale = 1.0f / 5.0f;

    // Electrical phase offset in radians as a function of encoder
    // position.
    std::array<float, 64> offset = {};

    // After applying inversion, add this value to the position.
    uint16_t position_offset = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(poles));
      a->Visit(MJ_NVP(invert));
      a->Visit(MJ_NVP(resistance_ohm));
      a->Visit(MJ_NVP(v_per_hz));
      a->Visit(MJ_NVP(unwrapped_position_scale));
      a->Visit(MJ_NVP(offset));
      a->Visit(MJ_NVP(position_offset));
    }
  };

  struct Config {
    float i_scale_A = 0.04028f;  // Amps per A/D LSB
    float v_scale_V = 0.00884f;  // V per A/D count

    float max_voltage = 20.0f;
    float max_temperature = 75.0f;

    float feedforward_scale = 1.0f;

    uint16_t adc_cycles = 15;  // 3, 15, 28, 56, 84, 112, 144, 480

    float vel_filter_s = 0.002f;

    // We use the same PID constants for D and Q current control
    // loops.
    mjlib::base::PID::Config pid_dq;
    mjlib::base::PID::Config pid_position;

    Config() {
      pid_dq.kp = 0.1f;
      pid_dq.ki = 30.0f;
      pid_dq.ilimit = 20.0f;
      pid_dq.sign = -1.0f;
      pid_dq.max_desired_rate = 30000.0f;

      pid_position.kp = 450.0f;
      pid_position.ki = 100.0f;
      pid_position.ilimit = 0.0f;
      pid_position.kd = 9.0f;
      pid_position.sign = -1.0f;
    }

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(i_scale_A));
      a->Visit(MJ_NVP(v_scale_V));
      a->Visit(MJ_NVP(max_voltage));
      a->Visit(MJ_NVP(max_temperature));
      a->Visit(MJ_NVP(feedforward_scale));
      a->Visit(MJ_NVP(adc_cycles));
      a->Visit(MJ_NVP(vel_filter_s));
      a->Visit(MJ_NVP(pid_dq));
      a->Visit(MJ_NVP(pid_position));
    }
  };

  // This will commonly be different across every device, so it is
  // separate to minimize resets due to schemas changing during
  // development.
  struct PositionConfig {
    float position_min = -0.01f;
    float position_max = 0.01f;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(position_min));
      a->Visit(MJ_NVP(position_max));
    }
  };

  enum Mode {
    // In this mode, the entire motor driver will be disabled.
    //
    // When exiting this state, the current offset will be
    // recalibrated.
    kStopped,

    // This stage cannot be commanded directly, but will be entered
    // upon any fault.  Here, the motor driver remains enabled, but
    // the output stage power is removed.  The only valid transition
    // from this state is to kStopped.
    kFault,

    // This mode may not be commanded directly.  It is used when
    // transitioning from kStopped to another mode.
    kEnabling,

    // This mode may not be commanded directly, but is used when
    // transitioning from kStopped to another mode.
    kCalibrating,

    // This mode may not be commanded directly, but is used when
    // transitioning from kStopped to another mode.
    kCalibrationComplete,

    // Directly control the PWM of all 3 phases.
    kPwm,

    // Control the voltage of all three phases
    kVoltage,

    // Control the phase and voltage magnitude
    kVoltageFoc,

    // Control d and q current
    kCurrent,

    // Control absolute position
    kPosition,

    kNumModes,
  };

  static std::array<std::pair<Mode, const char*>, kNumModes> ModeMapper() {
    return { {
        { kStopped, "stopped" },
        { kFault, "fault" },
        { kEnabling, "enabling" },
        { kCalibrating, "calibrating" },
        { kCalibrationComplete, "calib_complete" },
        { kPwm, "pwm" },
        { kVoltage, "voltage" },
        { kVoltageFoc, "voltage_foc" },
        { kCurrent, "current" },
        { kPosition, "position" },
      }};
  }

  static std::array<std::pair<errc, const char*>, 0> FaultMapper() {
    return {{}};
  }

  struct Status {
    Mode mode = kStopped;
    errc fault = errc::kSuccess;

    uint16_t adc1_raw = 0;
    uint16_t adc2_raw = 0;
    uint16_t adc3_raw = 0;
    uint16_t position_raw = 0;
    uint16_t fet_temp_raw = 0;

    uint16_t adc1_offset = 2048;
    uint16_t adc2_offset = 2048;

    float cur1_A = 0.0f;
    float cur2_A = 0.0f;
    float bus_V = 0.0f;
    float filt_bus_V = std::numeric_limits<float>::quiet_NaN();
    uint16_t position = 0;
    float fet_temp_C = 0.0f;

    float electrical_theta = 0.0f;

    float d_A = 0.0f;
    float q_A = 0.0f;

    int32_t unwrapped_position_raw = 0;
    float unwrapped_position = 0.0f;
    float velocity = 0.0f;

    mjlib::base::PID::State pid_d;
    mjlib::base::PID::State pid_q;
    mjlib::base::PID::State pid_position;

    float control_position = std::numeric_limits<float>::quiet_NaN();
    bool position_set = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_ENUM(mode, ModeMapper));
      a->Visit(MJ_ENUM(fault, FaultMapper));

      a->Visit(MJ_NVP(adc1_raw));
      a->Visit(MJ_NVP(adc2_raw));
      a->Visit(MJ_NVP(adc3_raw));
      a->Visit(MJ_NVP(position_raw));
      a->Visit(MJ_NVP(fet_temp_raw));

      a->Visit(MJ_NVP(adc1_offset));
      a->Visit(MJ_NVP(adc2_offset));

      a->Visit(MJ_NVP(cur1_A));
      a->Visit(MJ_NVP(cur2_A));
      a->Visit(MJ_NVP(bus_V));
      a->Visit(MJ_NVP(filt_bus_V));
      a->Visit(MJ_NVP(position));
      a->Visit(MJ_NVP(fet_temp_C));
      a->Visit(MJ_NVP(electrical_theta));

      a->Visit(MJ_NVP(d_A));
      a->Visit(MJ_NVP(q_A));

      a->Visit(MJ_NVP(unwrapped_position_raw));
      a->Visit(MJ_NVP(unwrapped_position));
      a->Visit(MJ_NVP(velocity));

      a->Visit(MJ_NVP(pid_d));
      a->Visit(MJ_NVP(pid_q));
      a->Visit(MJ_NVP(pid_position));

      a->Visit(MJ_NVP(control_position));
      a->Visit(MJ_NVP(position_set));
    }
  };

  // Intermediate control outputs.
  struct Control {
    Vec3 pwm;
    Vec3 voltage;

    float d_V = 0.0f;
    float q_V = 0.0f;

    float i_d_A = 0.0f;
    float i_q_A = 0.0f;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pwm));
      a->Visit(MJ_NVP(voltage));
      a->Visit(MJ_NVP(d_V));
      a->Visit(MJ_NVP(q_V));
      a->Visit(MJ_NVP(i_d_A));
      a->Visit(MJ_NVP(i_q_A));
    }
  };

  struct CommandData {
    Mode mode = kStopped;

    // For kPwm mode.
    Vec3 pwm;  // 0-1.0

    // For kVoltage mode
    Vec3 phase_v;

    // For kVoltageFoc
    float theta = 0.0f;
    float voltage = 0.0f;

    // For kFoc mode.
    float i_d_A = 0.0f;
    float i_q_A = 0.0f;

    // For kPosition mode
    float position = 0.0f;  // kNaN means start at the current position.
    float velocity = 0.0f;

    float max_current = 0.0f;

    float kp_scale = 1.0f;
    float kd_scale = 1.0f;

    // If set, then force the position to be the given value.
    std::optional<float> set_position;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_ENUM(mode, ModeMapper));

      a->Visit(MJ_NVP(pwm));

      a->Visit(MJ_NVP(phase_v));

      a->Visit(MJ_NVP(theta));
      a->Visit(MJ_NVP(voltage));

      a->Visit(MJ_NVP(i_d_A));
      a->Visit(MJ_NVP(i_q_A));

      a->Visit(MJ_NVP(position));
      a->Visit(MJ_NVP(velocity));
      a->Visit(MJ_NVP(max_current));
      a->Visit(MJ_NVP(kp_scale));
      a->Visit(MJ_NVP(kd_scale));

      a->Visit(MJ_NVP(set_position));
    }
  };

  void Start();
  void Command(const CommandData&);

  Status status() const;
  const Config& config() const;
  const Motor& motor() const;

  /// Return a clock which increments with every control cycle.
  uint32_t clock() const;

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
