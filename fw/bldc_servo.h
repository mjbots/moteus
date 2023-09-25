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

#include <cstdint>

#include "PinNames.h"

#include "mjlib/base/visitor.h"

#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/aux_adc.h"
#include "fw/aux_port.h"
#include "fw/bldc_servo_structs.h"
#include "fw/error.h"
#include "fw/millisecond_timer.h"
#include "fw/moteus_hw.h"
#include "fw/motor_driver.h"
#include "fw/motor_position.h"
#include "fw/pid.h"
#include "fw/simple_pi.h"

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
    PinName current2 = NC;  // Must be sampled from ADC3
    PinName current3 = NC;  // Must be sampled from ADC2

    PinName vsense = NC;  // Must be sampled from ADC4/5
    PinName tsense = NC;  // Must be sampled from ADC5
    PinName msense = NC;  // Must be sampled from ADC5/4

    PinName debug_dac = NC;
    PinName debug_out = NC;
    PinName debug_out2 = NC;

    // If set, a constant telemetry stream will be emitted at the
    // control rate.
    PinName debug_uart_out = NC;
  };

  BldcServo(mjlib::micro::Pool*,
            mjlib::micro::PersistentConfig*,
            mjlib::micro::TelemetryManager*,
            MillisecondTimer*,
            MotorDriver*,
            AuxADC*,
            AuxPort*,
            AuxPort*,
            MotorPosition*,
            const Options&);
  ~BldcServo();

  void PollMillisecond();

  using Mode = BldcServoMode;
  using Status = BldcServoStatus;
  using CommandData = BldcServoCommandData;
  using Motor = BldcServoMotor;
  using Config = BldcServoConfig;
  using PositionConfig = BldcServoPositionConfig;

  // Intermediate control outputs.
  struct Control {
    Vec3 pwm;
    Vec3 voltage;

    float d_V = 0.0f;
    float q_V = 0.0f;

    float i_d_A = 0.0f;
    float i_q_A = 0.0f;

    float q_comp_A = 0.0f;
    float torque_Nm = 0.0f;

    void Clear() {
      // We implement this manually merely because it is faster than
      // using the constructor which delegates to memset.  It is
      // definitely more brittle.
      pwm.a = 0.0f;
      pwm.b = 0.0f;
      pwm.c = 0.0f;

      voltage.a = 0.0f;
      voltage.b = 0.0f;
      voltage.c = 0.0f;

      d_V = 0.0f;
      q_V = 0.0f;
      i_d_A = 0.0f;
      i_q_A = 0.0f;
      q_comp_A = 0.0f;
      torque_Nm = 0.0f;
    }

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pwm));
      a->Visit(MJ_NVP(voltage));
      a->Visit(MJ_NVP(d_V));
      a->Visit(MJ_NVP(q_V));
      a->Visit(MJ_NVP(i_d_A));
      a->Visit(MJ_NVP(i_q_A));
      a->Visit(MJ_NVP(q_comp_A));
      a->Visit(MJ_NVP(torque_Nm));
    }
  };

  void Start();
  void Command(const CommandData&);

  const Status& status() const;
  const Config& config() const;
  const Control& control() const;
  const AuxPort::Status& aux1() const;
  const AuxPort::Status& aux2() const;
  const MotorPosition::Status& motor_position() const;
  MotorPosition::Config* motor_position_config();
  const MotorPosition::Config* motor_position_config() const;

  void SetOutputPositionNearest(float position);
  void SetOutputPosition(float position);
  void RequireReindex();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
