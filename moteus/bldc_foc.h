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

#include "mjlib/base/visitor.h"

#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

#include "moteus/position_sensor.h"

namespace moteus {

/// Commands a BLDC motor.
class BldcFoc {
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

  BldcFoc(mjlib::micro::Pool*,
          mjlib::micro::PersistentConfig*,
          mjlib::micro::TelemetryManager*,
          PositionSensor*,
          const Options&);
  ~BldcFoc();

  struct Config {
    float i_scale_A = 0.02014f;  // Amps per A/D LSB
    float v_scale_V = 0.00884f;  // V per A/D count

    uint8_t motor_poles = 14;
    float motor_offset = -0.1;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(i_scale_A));
      a->Visit(MJ_NVP(v_scale_V));
      a->Visit(MJ_NVP(motor_poles));
      a->Visit(MJ_NVP(motor_offset));
    }
  };

  struct Status {
    uint16_t adc1_raw = 0;
    uint16_t adc2_raw = 0;
    uint16_t adc3_raw = 0;
    uint16_t position_raw = 0;

    uint16_t adc1_offset = 2048;
    uint16_t adc2_offset = 2048;

    float cur1_A = 0.0;
    float cur2_A = 0.0;
    float bus_V = 0.0;

    float electrical_theta = 0.0;

    float d_A = 0.0;
    float q_A = 0.0;

    // Commands.

    uint16_t phase_a_centipercent = 0;
    uint16_t phase_b_centipercent = 0;
    uint16_t phase_c_centipercent = 0;

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

      a->Visit(MJ_NVP(phase_a_centipercent));
      a->Visit(MJ_NVP(phase_b_centipercent));
      a->Visit(MJ_NVP(phase_c_centipercent));
    }
  };

  enum Mode {
    kDisabled,
    kPhasePwm,
    kFoc,
  };

  struct CommandData {
    Mode mode = kDisabled;

    // For kPhasePwm mode.
    uint16_t phase_a_centipercent = 0;  // 0 - 10000
    uint16_t phase_b_centipercent = 0;  // 0 - 10000
    uint16_t phase_c_centipercent = 0;  // 0 - 10000

    // For kFoc mode.
    int32_t i_d_mA = 0;
    int32_t i_q_mA = 0;
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
