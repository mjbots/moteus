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
#include <iostream>
#include <limits>

#include "mjlib/micro/test/persistent_config_fixture.h"

#include "fw/bldc_servo_control.h"
#include "fw/bldc_servo_position.h"
#include "fw/bldc_servo_structs.h"
#include "fw/motor_position.h"
#include "fw/test/spmsm_motor_simulator.h"

namespace moteus {
namespace test {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// A simulation context that integrates BldcServoControl with a SPMSM
// motor model.  Follows the CRTP pattern from
// bldc_servo_control_test.cc.
class SimulationContext : public BldcServoControl<SimulationContext> {
 public:
  // State members accessed by BldcServoControl via impl_.
  BldcServoStatus status_;
  BldcServoControl_Control control_;
  BldcServoConfig config_;
  BldcServoPositionConfig position_config_;
  BldcServoMotor motor_;
  MotorPosition::Config motor_position_config_;

  SimplePI::Config pid_dq_config;
  PID::Config pid_position_config;

  SimplePI pid_d_{&pid_dq_config, &status_.pid_d};
  SimplePI pid_q_{&pid_dq_config, &status_.pid_q};
  PID pid_position_{&pid_position_config, &status_.pid_position};

  RateConfig rate_config_{30000, 15000};

  float torque_constant_ = 0.0f;  // Set during initialization
  float v_per_hz_ = 0.0f;
  float flux_brake_min_voltage_ = 100.0f;  // Effectively disabled
  float derate_temperature_ = 80.0f;
  float motor_derate_temperature_ = 100.0f;
  uint8_t isr_motor_position_epoch_ = 0;
  uint32_t pwm_counts_ = 4000;
  float old_d_V_ = 0.0f;
  float old_q_V_ = 0.0f;

  // Motor position infrastructure
  mjlib::micro::test::PersistentConfigFixture pcf;
  mjlib::micro::TelemetryManager telemetry_manager{
      &pcf.pool, &pcf.command_manager, &pcf.write_stream, pcf.output_buffer};
  aux::AuxStatus aux1_status_;
  aux::AuxStatus aux2_status_;
  aux::AuxConfig aux1_config_;
  aux::AuxConfig aux2_config_;

  MotorPosition motor_position_{&pcf.persistent_config, &telemetry_manager,
                                &aux1_status_, &aux2_status_,
                                &aux1_config_, &aux2_config_};

  // Reference to motor position status (like bldc_servo.cc)
  const MotorPosition::Status& position_ = motor_position_.status();

  // Motor simulator
  SpmsmMotorSimulator motor_sim_;

  // Simulated encoder counts per revolution (14-bit)
  static constexpr uint32_t kEncoderCpr = 16384;

  // HW mock state for verification.
  int pwm_control_count = 0;
  Vec3 last_pwm{0.5f, 0.5f, 0.5f};  // Neutral PWM (no current)

  int hard_stop_count = 0;
  int fault_count = 0;
  int calibrating_count = 0;
  int brake_count = 0;
  int start_calibrating_count = 0;

  bool fault_state = false;

  // Simulated bus voltage
  float bus_voltage_ = 24.0f;

  // External load torque (Nm)
  float external_torque_ = 0.0f;

  SimulationContext() {
    // Initialize MJ5208-like motor parameters for firmware (motor_) and
    // simulator (motor_sim_) from the same source.
    const auto& mp = Mj5208Params();

    // Firmware motor model
    motor_.poles = mp.kPolePairs * 2;  // 14 poles
    motor_.resistance_ohm = mp.kR;
    motor_.Kv = mp.kKv;
    motor_.rotation_current_cutoff_A = mp.kRotationCurrentCutoff;
    motor_.rotation_current_scale = mp.kRotationCurrentScale;
    motor_.rotation_torque_scale = mp.kRotationTorqueScale;
    torque_constant_ = mp.kKt;
    v_per_hz_ = 0.5f * 60.0f / mp.kKv;  // V/(rev/s), matches firmware

    // Motor simulator (uses Mj5208Params defaults, but explicit for clarity)
    SpmsmMotorSimulator::Params sim_params;
    sim_params.lambda_m = mp.kLambdaM;
    sim_params.Kt = mp.kKt;
    sim_params.R = mp.kR;
    sim_params.L = mp.kL;
    sim_params.J = mp.kJ;
    sim_params.B = mp.kB;
    sim_params.pole_pairs = mp.kPolePairs;
    sim_params.rotation_current_cutoff = mp.kRotationCurrentCutoff;
    sim_params.rotation_current_scale = mp.kRotationCurrentScale;
    sim_params.rotation_torque_scale = mp.kRotationTorqueScale;
    motor_sim_ = SpmsmMotorSimulator(sim_params);

    // Set up position limits (wide range)
    position_config_.position_min = kNaN;
    position_config_.position_max = kNaN;

    // Set up PID parameters (matched to real hardware at 400Hz bandwidth)
    pid_dq_config.kp = 0.065f;
    pid_dq_config.ki = 120.0f;
    pid_dq_config.max_desired_rate = 10000.0f;

    pid_position_config.kp = 4.0f;
    pid_position_config.ki = 1.0f;
    pid_position_config.kd = 0.05f;
    pid_position_config.sign = -1.0f;

    // Set up config parameters
    config_.max_current_A = 40.0f;
    config_.max_velocity = 100.0f;
    config_.max_velocity_derate = 20.0f;
    config_.default_velocity_limit = kNaN;
    config_.default_accel_limit = kNaN;

    // Set up motor position configuration
    motor_position_.config()->rotor_to_output_ratio = 1.0f;
    motor_position_.config()->sources[0].pll_filter_hz = 400.0f;
    motor_position_.motor()->poles = motor_.poles;

    // Initialize bus voltage
    status_.filt_bus_V = bus_voltage_;
    status_.filt_1ms_bus_V = bus_voltage_;
    status_.bus_V = bus_voltage_;
    status_.filt_fet_temp_C = 25.0f;
    status_.max_power_W = 500.0f;

    // Disable timeouts by default
    status_.timeout_s = kNaN;

    // Load configuration
    pcf.persistent_config.Load();

    // Set the motor position rate
    motor_position_.SetRate(rate_config_.period_s);

    // Initialize the encoder state
    aux1_status_.spi.active = true;
    aux1_status_.spi.value = 0;
    aux1_status_.spi.nonce = 1;

    // Run an initial motor position update to establish position
    motor_position_.ISR_Update();

    // Set initial epoch
    isr_motor_position_epoch_ = position_.epoch;

    // Copy motor position config for const access
    motor_position_config_ = *motor_position_.config();
  }

  // HW methods required by BldcServoControl.
  void DoPwmControl(const Vec3& pwm) {
    pwm_control_count++;
    last_pwm = pwm;
  }

  void DoHardStop() {
    hard_stop_count++;
  }

  void DoFault() {
    fault_count++;
  }

  void DoCalibrating() {
    calibrating_count++;
  }

  void DoBrake() {
    brake_count++;
  }

  void StartCalibrating() {
    start_calibrating_count++;
    // For simulation, immediately complete calibration
    status_.mode = kCalibrationComplete;
  }

  bool motor_driver_fault() const {
    return fault_state;
  }

  SinCos cordic(int32_t radians_q31) const {
    // Simple sin/cos implementation for tests.
    const float theta = static_cast<float>(radians_q31) * kPi *
        (1.0f / 2147483648.0f);
    SinCos result;
    result.s = std::sin(theta);
    result.c = std::cos(theta);
    return result;
  }

  const MotorPosition::Config* motor_position_config() const {
    return &motor_position_config_;
  }

  int64_t absolute_relative_delta() const {
    return motor_position_.absolute_relative_delta.load();
  }

  // Step the simulation by one control cycle.
  void StepSimulation(BldcServoCommandData* cmd) {
    const float dt = rate_config_.period_s;

    // 1. Apply PREVIOUS cycle's PWM to motor (realistic one-cycle delay)
    // Convert PWM to phase-to-neutral voltages correctly
    // The PWM values from SVPWM have an offset; we need balanced voltages
    const float v_a_raw = last_pwm.a * bus_voltage_;
    const float v_b_raw = last_pwm.b * bus_voltage_;
    const float v_c_raw = last_pwm.c * bus_voltage_;
    // Compute neutral point and subtract to get phase-to-neutral voltages
    const float v_neutral = (v_a_raw + v_b_raw + v_c_raw) / 3.0f;
    const float v_a = v_a_raw - v_neutral;
    const float v_b = v_b_raw - v_neutral;
    const float v_c = v_c_raw - v_neutral;
    motor_sim_.Step(dt, v_a, v_b, v_c, external_torque_);

    // 2. Update encoder from motor rotor position.
    const float rotor_rev = motor_sim_.position_rev();
    float wrapped_rev = rotor_rev - std::floor(rotor_rev);
    if (wrapped_rev < 0.0f) {
      wrapped_rev += 1.0f;
    }

    aux1_status_.spi.value = static_cast<uint32_t>(wrapped_rev * kEncoderCpr) %
                             kEncoderCpr;
    aux1_status_.spi.nonce++;

    // 3. Update motor position (uses encoder theta via PLL)
    motor_position_.ISR_Update();
    status_.velocity_filt = position_.velocity;

    // 4. Get phase currents from motor, transform to DQ using encoder theta
    const auto phase_cur = motor_sim_.phase_currents();
    const SinCos sin_cos = ISR_CalculateDerivedQuantities(
        phase_cur.a, phase_cur.b, phase_cur.c, cmd->synthetic_theta);

    // 5. Run control to compute NEW PWM (for next cycle)
    ISR_DoControl(sin_cos, cmd);

    // 6. Update reported values
    status_.position = position_.position;
    status_.velocity = position_.velocity;
  }

  // Run simulation for a given duration.
  //
  // Returns the number of steps executed.
  int RunSimulation(BldcServoCommandData* cmd, float duration_s) {
    const float dt = rate_config_.period_s;
    const int num_steps = static_cast<int>(duration_s / dt);

    for (int i = 0; i < num_steps; i++) {
      StepSimulation(cmd);
    }

    return num_steps;
  }

  // Reset simulation state
  void Reset() {
    motor_sim_.state() = SpmsmMotorSimulator::State{};
    aux1_status_.spi.value = 0;
    aux1_status_.spi.nonce++;
    motor_position_.ISR_Update();

    status_.pid_d.Clear();
    status_.pid_q.Clear();
    status_.pid_position.Clear();
    status_.control_position_raw = {};
    status_.control_velocity = {};
    status_.mode = kStopped;
    status_.fault = errc::kSuccess;

    external_torque_ = 0.0f;
    last_pwm = Vec3{0.5f, 0.5f, 0.5f};  // Neutral PWM
  }

  // Set motor position for testing
  void SetMotorPosition(float rev) {
    motor_sim_.state().theta_mechanical = rev * k2Pi;
    motor_sim_.state().theta_electrical =
        rev * k2Pi * motor_sim_.params().pole_pairs;
    motor_sim_.state().omega_mechanical = 0.0f;

    // Update encoder
    float wrapped_rev = rev - std::floor(rev);
    if (wrapped_rev < 0.0f) wrapped_rev += 1.0f;
    aux1_status_.spi.value = static_cast<uint32_t>(wrapped_rev * kEncoderCpr) %
                             kEncoderCpr;
    aux1_status_.spi.nonce++;

    motor_position_.ISR_Update();
  }

  // Process a command, applying defaults and transforming positions.
  // This mirrors the firmware's Command method.
  // Also handles mode transitions if starting from kStopped.
  void Command(BldcServoCommandData* cmd) {
    // Handle mode transitions if needed
    if (status_.mode == kStopped && cmd->mode != kStopped) {
      // Trigger calibration
      ISR_MaybeChangeMode(cmd);

      // Complete calibration immediately (simulation bypass)
      if (status_.mode == kEnabling) {
        status_.mode = kCalibrationComplete;
      }

      // Now switch to the commanded mode
      ISR_MaybeChangeMode(cmd);
    }

    errc err = PrepareCommand(cmd);
    if (err != errc::kSuccess) {
      status_.fault = err;
      status_.mode = kFault;
    }
  }
};

// Simulation fixture for Boost.Test
struct SimulationFixture {
  SimulationContext ctx;

  SimulationFixture() = default;
};

}  // namespace test
}  // namespace moteus
