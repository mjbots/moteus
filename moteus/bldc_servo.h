// Copyright 2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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
    PinName vsense = NC;  // Must be sampled from ADC3.
    PinName vtemp = NC;

    PinName debug_out = NC;
  };

  BldcServo(mjlib::micro::Pool*,
            mjlib::micro::PersistentConfig*,
            mjlib::micro::TelemetryManager*,
            PositionSensor*,
            MotorDriver*,
            const Options&);
  ~BldcServo();

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

  struct Config {
    float i_scale_A = 0.02014f;  // Amps per A/D LSB
    float v_scale_V = 0.00884f;  // V per A/D count

    uint8_t motor_poles = 14;
    float motor_offset = -0.61;

    float unwrapped_position_scale = 1.0f / 5.0f;

    // We use the same PID constants for D and Q current control
    // loops.
    mjlib::base::PID::Config pid_dq;
    mjlib::base::PID::Config pid_position;

    Config() {
      pid_dq.kp = 0.01f;
      pid_dq.ki = 30.0f;
      pid_dq.ilimit = 20.0f;
      pid_dq.sign = -1;

      pid_position.kp = 30.0f;
      pid_position.ki = 100.0f;
      pid_position.ilimit = 0.1f;
      pid_position.kd = 1.5f;
    }

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(i_scale_A));
      a->Visit(MJ_NVP(v_scale_V));
      a->Visit(MJ_NVP(motor_poles));
      a->Visit(MJ_NVP(motor_offset));
      a->Visit(MJ_NVP(unwrapped_position_scale));
      a->Visit(MJ_NVP(pid_dq));
      a->Visit(MJ_NVP(pid_position));
    }
  };

  struct Status {
    uint16_t adc1_raw = 0;
    uint16_t adc2_raw = 0;
    uint16_t adc3_raw = 0;
    uint16_t position_raw = 0;

    uint16_t adc1_offset = 2048;
    uint16_t adc2_offset = 2048;

    float cur1_A = 0.0f;
    float cur2_A = 0.0f;
    float bus_V = 0.0f;

    float electrical_theta = 0.0f;

    float d_A = 0.0f;
    float q_A = 0.0f;

    int32_t unwrapped_position_raw = 0;
    float unwrapped_position = 0.0f;
    float velocity = 0.0f;

    bool zero_applied = false;

    mjlib::base::PID::State pid_d;
    mjlib::base::PID::State pid_q;
    mjlib::base::PID::State pid_position;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(adc1_raw));
      a->Visit(MJ_NVP(adc2_raw));
      a->Visit(MJ_NVP(adc3_raw));
      a->Visit(MJ_NVP(position_raw));

      a->Visit(MJ_NVP(adc1_offset));
      a->Visit(MJ_NVP(adc2_offset));

      a->Visit(MJ_NVP(cur1_A));
      a->Visit(MJ_NVP(cur2_A));
      a->Visit(MJ_NVP(bus_V));
      a->Visit(MJ_NVP(electrical_theta));

      a->Visit(MJ_NVP(d_A));
      a->Visit(MJ_NVP(q_A));

      a->Visit(MJ_NVP(unwrapped_position_raw));
      a->Visit(MJ_NVP(unwrapped_position));
      a->Visit(MJ_NVP(velocity));

      a->Visit(MJ_NVP(zero_applied));

      a->Visit(MJ_NVP(pid_d));
      a->Visit(MJ_NVP(pid_q));
      a->Visit(MJ_NVP(pid_position));
    }
  };

  // Intermediate control outputs.
  struct Control {
    Vec3 pwm;
    Vec3 voltage;

    float i_d_A = 0.0f;
    float i_q_A = 0.0f;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pwm));
      a->Visit(MJ_NVP(voltage));
      a->Visit(MJ_NVP(i_d_A));
      a->Visit(MJ_NVP(i_q_A));
    }
  };

  enum Mode {
    kDisabled,
    kPwm,  // Directly control the PWM of all 3 phases.
    kVoltage,  // Command the voltage of all three phases
    kVoltageFoc,  // Command phase and voltage
    kFoc,  // Command d and q current
    kPosition,
    kNumModes,
  };

  static std::array<std::pair<Mode, const char*>, kNumModes> ModeMapper() {
    return { {
        { kDisabled, "disabled" },
        { kPwm, "pwm" },
        { kVoltage, "voltage" },
        { kVoltageFoc, "voltage_foc" },
        { kFoc, "foc" },
        { kPosition, "position" },
      }};
  }

  struct CommandData {
    Mode mode = kDisabled;

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
    }
  };

  void Command(const CommandData&);

  // Stop all switching, assume that all currents are 0, average
  // readings from each of the current sensors to determine the actual
  // zero offset.
  void ZeroOffset();

  Status status() const;

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
