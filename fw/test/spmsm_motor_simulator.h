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

#pragma once

#include <cmath>
#include <memory>

#include "fw/test/motor_model.h"

namespace moteus {
namespace test {

constexpr double k2Pi_d = 2.0 * M_PI;

// Representative mj5208 motor parameters
struct Mj5208Params {
  static constexpr float kKv = 304.0f;  // RPM/V (line-to-line)
  static constexpr int kPolePairs = 7;  // 14 poles total

  // Motor torque constant from Kv: Kt = 8.3 / Kv (Nm/A for peak current)
  // This is a standard approximation for sinusoidal PMSM.
  static constexpr float kKt = 8.3f / kKv;  // ~0.0273 Nm/A

  // PM flux linkage (Wb) for dq-frame model.
  // In the dq model: T = 1.5 * pole_pairs * lambda_m * I_q
  // So: lambda_m = Kt / (1.5 * pole_pairs)
  static constexpr float kLambdaM = kKt / (1.5f * kPolePairs);  // ~0.0026 Wb

  static constexpr float kR = 0.047f;  // Ohm, phase-center resistance
  static constexpr float kL = 28.6e-6f;  // H, phase-center inductance
  static constexpr float kJ = 5.6e-5f;  // kg*m^2, rotor inertia
  static constexpr float kB = 3e-5f;  // Nm/(rad/s), viscous damping

  // Saturation model parameters (rotation region)
  static constexpr float kRotationCurrentCutoff = 22.5f;  // A
  static constexpr float kRotationCurrentScale = 0.02494f;
  static constexpr float kRotationTorqueScale = 0.6638f;  // Nm
};

// Surface Permanent Magnet Synchronous Motor (SPMSM) simulator.
// Implements a numerical motor model using DQ-frame dynamics.
class SpmsmMotorSimulator {
 public:
  struct Params {
    float lambda_m = Mj5208Params::kLambdaM;  // PM flux linkage (Wb)
    float Kt = Mj5208Params::kKt;  // Torque constant (Nm/A) = 1.5 * pole_pairs * lambda_m
    float R = Mj5208Params::kR;  // Phase resistance (Ohm)
    float L = Mj5208Params::kL;  // Phase inductance (H)
    float J = Mj5208Params::kJ;  // Rotor inertia (kg*m^2)
    int pole_pairs = Mj5208Params::kPolePairs;
    float B = Mj5208Params::kB;  // Viscous damping coefficient (Nm/(rad/s))

    // Saturation model (rotation region): models magnetic saturation at
    // high currents. Above current_cutoff, torque grows logarithmically:
    //   T = cutoff*Kt + torque_scale * log2(1 + (I - cutoff) * current_scale)
    float rotation_current_cutoff = Mj5208Params::kRotationCurrentCutoff;
    float rotation_current_scale = Mj5208Params::kRotationCurrentScale;
    float rotation_torque_scale = Mj5208Params::kRotationTorqueScale;
  };

  struct State {
    double theta_electrical = 0.0;  // Electrical angle (radians), wrapped to [0, 2*pi)
    double theta_mechanical = 0.0;  // Mechanical angle (radians), continuous (not wrapped)
    double omega_mechanical = 0.0;  // Mechanical angular velocity (rad/s)
    double i_d = 0.0;  // D-axis current (A)
    double i_q = 0.0;  // Q-axis current (A)
  };

  SpmsmMotorSimulator() : model_(MakeSimpleModel(Params{})) {}
  explicit SpmsmMotorSimulator(const Params& params)
      : params_(params), model_(MakeSimpleModel(params)) {}
  explicit SpmsmMotorSimulator(std::shared_ptr<MotorModel> model)
      : model_(std::move(model)) {
    SynthesizeParams();
  }

  // Step the motor simulation given phase voltages (a, b, c).
  // dt: time step (seconds)
  // v_a, v_b, v_c: phase voltages
  // T_load: external load torque (Nm, positive torques in positive direction)
  void Step(float dt, float v_a, float v_b, float v_c, float T_load = 0.0f) {
    // Use sub-stepping for numerical stability.
    // Motor model runs at 10x the control rate.
    constexpr int kSubSteps = 10;
    const double sub_dt = static_cast<double>(dt) / kSubSteps;

    for (int i = 0; i < kSubSteps; i++) {
      StepInternal(sub_dt, v_a, v_b, v_c, T_load);
    }
  }

  // Step the motor simulation given DQ-frame voltages.
  // Performs inverse Park and Clarke transforms to convert to phase voltages.
  // dt: time step (seconds)
  // v_d, v_q: DQ-frame voltages
  // T_load: external load torque (Nm, positive torques in positive direction)
  void StepDq(float dt, float v_d, float v_q, float T_load = 0.0f) {
    const float theta = theta_electrical();
    const float sin_t = std::sin(theta);
    const float cos_t = std::cos(theta);

    // Inverse Park: alpha = cos*d - sin*q, beta = sin*d + cos*q
    const float v_alpha = cos_t * v_d - sin_t * v_q;
    const float v_beta = sin_t * v_d + cos_t * v_q;

    // Inverse Clarke: a = alpha, b = -0.5*alpha + sqrt(3)/2*beta,
    //                 c = -0.5*alpha - sqrt(3)/2*beta
    constexpr float kSqrt3_2 = 0.86602540378f;
    const float v_a = v_alpha;
    const float v_b = -0.5f * v_alpha + kSqrt3_2 * v_beta;
    const float v_c = -0.5f * v_alpha - kSqrt3_2 * v_beta;

    Step(dt, v_a, v_b, v_c, T_load);
  }

  // Get the current state.
  const State& state() const { return state_; }
  State& state() { return state_; }

  // Get motor parameters (read-only; to mutate, use model() accessors).
  const Params& params() const { return params_; }

  // Convenience accessors.
  float theta_electrical() const {
    return static_cast<float>(state_.theta_electrical);
  }
  float omega_mechanical() const {
    return static_cast<float>(state_.omega_mechanical);
  }
  float omega_electrical() const {
    return static_cast<float>(state_.omega_mechanical * model_->pole_pairs());
  }
  float i_d() const { return static_cast<float>(state_.i_d); }
  float i_q() const { return static_cast<float>(state_.i_q); }

  // Phase currents (A, B, C) via inverse DQ transform at motor electrical angle.
  // These can be fed into a DqTransform at a different angle (e.g., encoder)
  // to get currents in that reference frame.
  struct PhaseCurrent {
    float a, b, c;
  };

  PhaseCurrent phase_currents() const {
    const float theta = theta_electrical();
    const float sin_t = std::sin(theta);
    const float cos_t = std::cos(theta);
    const float d = i_d();
    const float q = i_q();

    // Inverse Park transform: alpha-beta from d-q
    const float i_alpha = cos_t * d - sin_t * q;
    const float i_beta = sin_t * d + cos_t * q;

    // Inverse Clarke transform: a-b-c from alpha-beta
    constexpr float kSqrt3_2 = 0.86602540378f;
    return PhaseCurrent{
        i_alpha,
        -0.5f * i_alpha + kSqrt3_2 * i_beta,
        -0.5f * i_alpha - kSqrt3_2 * i_beta
    };
  }

  // Position in revolutions (mechanical), continuous (not wrapped).
  float position_rev() const {
    return static_cast<float>(state_.theta_mechanical / k2Pi_d);
  }

  // Velocity in rev/s (mechanical).
  float velocity_rev_s() const {
    return static_cast<float>(state_.omega_mechanical / k2Pi_d);
  }

  // Electromagnetic torque (Nm), including saturation model.
  float torque_em() const {
    return static_cast<float>(current_to_torque(state_.i_q));
  }

  // Convert phase current to torque, applying saturation model.
  // Delegates to the motor model.
  double current_to_torque(double current) const {
    return model_->current_to_torque(current);
  }

  const std::shared_ptr<MotorModel>& model() const { return model_; }

 private:
  void StepInternal(double dt, float v_a, float v_b, float v_c, float T_load) {
    // 1. Convert phase voltages (a,b,c) to DQ frame using Clarke+Park.
    const double sin_t = std::sin(state_.theta_electrical);
    const double cos_t = std::cos(state_.theta_electrical);

    // Clarke transform: abc -> alpha-beta
    // alpha = a, beta = (1/sqrt(3)) * (a + 2*b) = (2/sqrt(3)) * (b + a/2)
    constexpr double kSqrt3 = 1.7320508075688772;
    const double v_alpha = static_cast<double>(v_a);
    const double v_beta = (1.0 / kSqrt3) * (v_a + 2.0 * v_b);

    // Park transform: alpha-beta -> dq
    // d = alpha*cos + beta*sin, q = -alpha*sin + beta*cos
    const double v_d = v_alpha * cos_t + v_beta * sin_t;
    const double v_q = -v_alpha * sin_t + v_beta * cos_t;

    // 2. Solve electrical dynamics.
    // DQ-frame equations with separate L_d, L_q:
    //   L_d * di_d/dt = v_d - R*i_d + omega_e * L_q * i_q
    //   L_q * di_q/dt = v_q - R*i_q - omega_e * L_d * i_d - omega_e * lambda_m
    //
    // Cross-coupling terms use the other axis's inductance.
    const double omega_e = state_.omega_mechanical * model_->pole_pairs();
    const double R = model_->R();
    const double L_d = model_->L_d(state_.i_d, state_.i_q);
    const double L_q = model_->L_q(state_.i_d, state_.i_q);
    const double lambda_m = model_->lambda_m();

    const double di_d_dt = (v_d - R * state_.i_d +
                            omega_e * L_q * state_.i_q) / L_d;
    const double di_q_dt = (v_q - R * state_.i_q -
                            omega_e * L_d * state_.i_d -
                            omega_e * lambda_m) / L_q;

    state_.i_d += di_d_dt * dt;
    state_.i_q += di_q_dt * dt;

    // 3. Compute electromagnetic torque (with saturation model).
    const double T_em = current_to_torque(state_.i_q);

    // 4. Solve mechanical dynamics.
    // J * domega/dt = T_em + T_load - B*omega
    const double domega_dt = (T_em + static_cast<double>(T_load) -
                              model_->B() * state_.omega_mechanical) / model_->J();
    state_.omega_mechanical += domega_dt * dt;

    // 5. Integrate position.
    // theta_mechanical = integral of omega_mechanical
    // theta_electrical = theta_mechanical * pole_pairs (mod 2*pi for transforms)
    state_.theta_mechanical += state_.omega_mechanical * dt;
    state_.theta_electrical += state_.omega_mechanical * model_->pole_pairs() * dt;

    // Wrap theta_electrical to [0, 2*pi) for DQ transforms.
    //
    // theta_mechanical is NOT wrapped to allow continuous position tracking.
    while (state_.theta_electrical >= k2Pi_d) {
      state_.theta_electrical -= k2Pi_d;
    }
    while (state_.theta_electrical < 0.0) {
      state_.theta_electrical += k2Pi_d;
    }
  }

  static std::shared_ptr<SimpleMotorModel> MakeSimpleModel(const Params& p) {
    SimpleMotorModel::Params sp;
    sp.lambda_m = p.lambda_m;
    sp.R = p.R;
    sp.L = p.L;
    sp.J = p.J;
    sp.B = p.B;
    sp.pole_pairs = p.pole_pairs;
    sp.rotation_current_cutoff = p.rotation_current_cutoff;
    sp.rotation_current_scale = p.rotation_current_scale;
    sp.rotation_torque_scale = p.rotation_torque_scale;
    return std::make_shared<SimpleMotorModel>(sp);
  }

  void SynthesizeParams() {
    params_.lambda_m = model_->lambda_m();
    params_.Kt = 1.5f * model_->pole_pairs() * model_->lambda_m();
    params_.R = model_->R();
    params_.L = model_->L_d(0, 0);
    params_.J = model_->J();
    params_.pole_pairs = model_->pole_pairs();
    params_.B = model_->B();
  }

  Params params_;
  std::shared_ptr<MotorModel> model_;
  State state_;
};

}  // namespace test
}  // namespace moteus
