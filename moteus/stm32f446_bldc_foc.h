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

#include "mjlib/micro/pool_ptr.h"

namespace moteus {

/// Commands a BLDC motor.  Pin and peripheral assignments are
/// hardcoded as: TBD
class Stm32F446BldcFoc {
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

  Stm32F446BldcFoc(mjlib::micro::Pool*, const Options&);
  ~Stm32F446BldcFoc();

  struct Config {
    float i_scale_A = 0.02014f;  // Amps per A/D LSB
    float v_scale_V = 0.00884f;  // V per A/D count
  };

  struct Status {
    uint16_t adc1_raw = 0;
    uint16_t adc2_raw = 0;
    uint16_t adc3_raw = 0;

    float cur1_A = 0.0;
    float cur2_A = 0.0;
    float bus_V = 0.0;
  };

  enum Mode {
    kDisabled,
    kPhasePwm,
    kFoc,
  };

  struct CommandData {
    Mode mode = kDisabled;

    // For kPhasePwm mode.
    uint16_t phase_a_millipercent = 0;  // 0 - 10000
    uint16_t phase_b_millipercent = 0;  // 0 - 10000
    uint16_t phase_c_millipercent = 0;  // 0 - 10000

    // For kFoc mode.
    int32_t i_d_mA = 0;
    int32_t i_q_mA = 0;
  };

  void Command(const CommandData&);

  Status status() const;

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
