// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

/// @file
///
/// A C interface to the motor simulator used by the firmware
/// simulation tests, so that the emulator shares the same model.

#include "fw/test/spmsm_motor_simulator.h"

using moteus::test::SpmsmMotorSimulator;

extern "C" {

struct MoteusMotorState {
  float current_a;
  float current_b;
  float current_c;
  float position_rev;
  float velocity_rev_s;
  float torque_Nm;
};

/// The parameters of the default motor, which the firmware needs to
/// be configured with in order to control it.
struct MoteusMotorParams {
  int pole_pairs;
  float resistance_ohm;
  float inductance_H;
  float Kv;
};

void* moteus_motor_new() {
  return new SpmsmMotorSimulator();
}

void moteus_motor_free(void* motor) {
  delete static_cast<SpmsmMotorSimulator*>(motor);
}

void moteus_motor_params(MoteusMotorParams* out) {
  const SpmsmMotorSimulator::Params params;
  out->pole_pairs = params.pole_pairs;
  out->resistance_ohm = params.R;
  out->inductance_H = params.L;
  out->Kv = moteus::test::Mj5208Params::kKv;
}

/// Advance with the given phase to neutral voltages applied.
void moteus_motor_step(void* motor, float dt_s,
                       float v_a, float v_b, float v_c,
                       float load_Nm) {
  static_cast<SpmsmMotorSimulator*>(motor)->Step(
      dt_s, v_a, v_b, v_c, load_Nm);
}

/// Advance with the phases disconnected.
void moteus_motor_step_hiz(void* motor, float dt_s, float load_Nm) {
  static_cast<SpmsmMotorSimulator*>(motor)->StepHiz(dt_s, load_Nm);
}

void moteus_motor_state(void* motor, MoteusMotorState* out) {
  const auto* const sim = static_cast<SpmsmMotorSimulator*>(motor);
  const auto currents = sim->phase_currents();
  out->current_a = currents.a;
  out->current_b = currents.b;
  out->current_c = currents.c;
  out->position_rev = sim->position_rev();
  out->velocity_rev_s = sim->velocity_rev_s();
  out->torque_Nm = sim->torque_em();
}

}
