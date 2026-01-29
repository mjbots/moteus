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
#include <vector>

#include "mjlib/micro/test/persistent_config_fixture.h"

#include "fw/bldc_servo_control.h"
#include "fw/bldc_servo_position.h"
#include "fw/bldc_servo_structs.h"
#include "fw/motor_position.h"
#include "fw/test/spmsm_motor_simulator.h"

namespace moteus {
namespace test {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// Helper to check if a value is close to a target within tolerance
inline bool IsClose(float actual, float expected, float tolerance) {
  return std::abs(actual - expected) < tolerance;
}

// Statistics result for CalcStats
struct Stats {
  float mean = 0.0f;
  float std_dev = 0.0f;
};

// Calculate mean and standard deviation from a vector of samples
inline Stats CalcStats(const std::vector<float>& samples) {
  if (samples.empty()) return {};

  float sum = 0.0f;
  for (float v : samples) {
    sum += v;
  }
  const float mean = sum / samples.size();

  float variance_sum = 0.0f;
  for (float v : samples) {
    const float diff = v - mean;
    variance_sum += diff * diff;
  }
  const float std_dev = std::sqrt(variance_sum / samples.size());
  return {mean, std_dev};
}

// Expected D current range for PWM mode voltage
inline std::pair<float, float> FindDCurrentRange(float voltage, float resistance) {
  const float minimum = std::max(-0.5f, 0.25f * voltage / resistance - 1.7f);
  const float maximum = 1.2f * voltage / resistance + 0.5f;
  return std::make_pair(minimum, maximum);
}

class SimulatedFixture;

// A simulation context that integrates BldcServoControl with a SPMSM
// motor model.  Follows the CRTP pattern from
// bldc_servo_control_test.cc.
class SimulationContext : public BldcServoControl<SimulationContext> {
 public:
  ////////////////////////////////////////////////////////////////
  // State members accessed by BldcServoControl
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

  ////////////////////////////////////////////////////////////////
  // Test state

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
    // Brake mode shorts all phases together and to ground.

    // With zero phase-to-neutral voltage, the motor back-EMF drives
    // current through the winding resistance, creating braking
    // torque.
    last_pwm = Vec3{0.0f, 0.0f, 0.0f};
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
    const auto prior_pwm = last_pwm;
    const float dt = rate_config_.period_s;

    // 1. Update encoder from motor rotor position.
    const float rotor_rev = motor_sim_.position_rev();
    float wrapped_rev = rotor_rev - std::floor(rotor_rev);
    if (wrapped_rev < 0.0f) {
      wrapped_rev += 1.0f;
    }

    aux1_status_.spi.value = static_cast<uint32_t>(wrapped_rev * kEncoderCpr) %
                             kEncoderCpr;
    aux1_status_.spi.nonce++;

    // 2. Update motor position (uses encoder theta via PLL)
    motor_position_.ISR_Update();
    status_.velocity_filt = position_.velocity;

    // 3. Get phase currents from motor, transform to DQ using encoder theta
    const auto phase_cur = motor_sim_.phase_currents();
    const SinCos sin_cos = ISR_CalculateDerivedQuantities(
        phase_cur.a, phase_cur.b, phase_cur.c, cmd->synthetic_theta);

    // 4. Run control to compute NEW PWM (for next cycle)
    ISR_DoControl(sin_cos, cmd);

    // 5. Update reported values
    // In fixed_voltage_mode (synthetic_theta), the ISR sets status_.position
    // and status_.velocity from the commanded values, not encoder. Don't
    // overwrite them.
    if (!cmd->synthetic_theta) {
      status_.position = position_.position;
      status_.velocity = position_.velocity;
    }

    // 6. Apply the previous cycle's PWM to the motor to achieve a
    // realistic one-cycle delay.

    // Convert PWM to phase-to-neutral voltages.
    const float v_a_raw = prior_pwm.a * bus_voltage_;
    const float v_b_raw = prior_pwm.b * bus_voltage_;
    const float v_c_raw = prior_pwm.c * bus_voltage_;

    // Compute neutral point and subtract to get phase-to-neutral voltages
    const float v_neutral = (v_a_raw + v_b_raw + v_c_raw) / 3.0f;
    const float v_a = v_a_raw - v_neutral;
    const float v_b = v_b_raw - v_neutral;
    const float v_c = v_c_raw - v_neutral;
    motor_sim_.Step(dt, v_a, v_b, v_c, external_torque_);
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

  // Step the simulation and fixture at the same time
  void StepWithFixture(BldcServoCommandData* cmd, SimulatedFixture* fixture);

  // Run the simulation and fixture for a given duration
  int RunWithFixture(BldcServoCommandData* cmd, SimulatedFixture* fixture,
                     float duration_s);

  // Reset simulation state
  void Reset() {
    motor_sim_.state() = SpmsmMotorSimulator::State{};
    aux1_status_.spi.value = 0;
    aux1_status_.spi.nonce++;

    // Reset the motor_position source state to match encoder at 0.
    // This prevents the velocity calculation from seeing a huge delta
    // from the previous test's position to 0.
    auto& mp_status = const_cast<MotorPosition::Status&>(motor_position_.status());
    mp_status.sources[0].offset_value = 0;
    mp_status.sources[0].compensated_value = 0.0f;
    mp_status.sources[0].filtered_value = 0.0f;
    mp_status.sources[0].velocity = 0.0f;
    mp_status.sources[0].integral = 0.0f;

    // Set absolute position to 0 (this properly resets position_raw)
    motor_position_.ISR_SetOutputPosition(0.0f);

    // Also reset the relative position tracking (used by trajectory controller)
    mp_status.position_relative_raw = 0;
    mp_status.position_relative_modulo = 0;
    mp_status.velocity = 0.0f;

    status_.pid_d.Clear();
    status_.pid_q.Clear();
    status_.pid_position.Clear();
    status_.control_position_raw = {};
    status_.control_velocity = {};
    // Set status from position (now 0 after ISR_SetOutputPosition)
    status_.position = position_.position;
    status_.velocity = position_.velocity;
    status_.mode = kStopped;
    status_.fault = errc::kSuccess;

    external_torque_ = 0.0f;
    last_pwm = Vec3{0.5f, 0.5f, 0.5f};  // Neutral PWM

    // Reset control state to avoid stale values affecting power limiting.
    // Initialize control_.d_V and control_.q_V to small non-zero values to
    // avoid division by zero in power limiting code (scaled_power / old_V).
    // The old_*_V values get set from control_.*_V at the start of ISR_DoControl.
    control_ = BldcServoControl_Control{};
    control_.d_V = 0.001f;
    control_.q_V = 0.001f;
    old_d_V_ = 0.001f;
    old_q_V_ = 0.001f;
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

    // Set the absolute output position (this sets position_raw correctly)
    motor_position_.ISR_SetOutputPosition(rev);

    // Also reset the relative position tracking (used by trajectory controller)
    // The trajectory controller initializes from position_relative_raw, not
    // position_raw, so we must reset this as well.
    auto& mp_status = const_cast<MotorPosition::Status&>(motor_position_.status());
    const int64_t position_int64 =
        static_cast<int64_t>(static_cast<double>(rev) * (1ll << 48));
    mp_status.position_relative_raw = position_int64;
    mp_status.position_relative_modulo = position_int64;
    mp_status.velocity = 0.0f;

    // Update status to match (for modes like fixed_voltage where status
    // is not updated from encoder during stepping)
    status_.position = position_.position;
    status_.velocity = position_.velocity;
  }

  // Set bus voltage (updates all related status fields)
  void SetBusVoltage(float voltage) {
    bus_voltage_ = voltage;
    status_.filt_bus_V = voltage;
    status_.filt_1ms_bus_V = voltage;
    status_.bus_V = voltage;
  }

  // Sample a value over multiple steps and return statistics.
  // The getter is called after each step; callers should capture what they need.
  template <typename Getter>
  Stats SampleValue(BldcServoCommandData* cmd, int steps, Getter getter) {
    std::vector<float> samples;
    samples.reserve(steps);
    for (int i = 0; i < steps; i++) {
      StepSimulation(cmd);
      samples.push_back(getter());
    }
    return CalcStats(samples);
  }

  // Sample a value over multiple steps with fixture and return statistics.
  template <typename Getter>
  Stats SampleValueWithFixture(BldcServoCommandData* cmd, SimulatedFixture* fixture,
                               int steps, Getter getter) {
    std::vector<float> samples;
    samples.reserve(steps);
    for (int i = 0; i < steps; i++) {
      StepWithFixture(cmd, fixture);
      samples.push_back(getter());
    }
    return CalcStats(samples);
  }

  // Re-apply motor position configuration after changes.
  //
  // Call this after modifying motor_position_.config() to ensure the
  // changes take effect (e.g., output.sign changes).
  //
  // Also resets the motor position state to avoid inconsistencies.
  void ReconfigureMotorPosition() {
    // Apply configuration changes and reset position tracking state.
    motor_position_.ApplyConfig();
    motor_position_.SetRate(rate_config_.period_s);
    motor_position_config_ = *motor_position_.config();

    // Trigger re-initialization from current encoder value
    aux1_status_.spi.nonce++;
    motor_position_.ISR_Update();

    // Update status from fresh motor position
    status_.position = position_.position;
    status_.velocity = position_.velocity;
  }

  // Process a command, applying defaults and transforming positions.
  //
  // This mirrors the firmware's Command method.
  //
  // Also handles mode transitions.
  void Command(BldcServoCommandData* cmd) {
    // Handle mode transitions if needed
    if (status_.mode != cmd->mode) {
      ISR_MaybeChangeMode(cmd);

      // Complete calibration immediately (simulation bypass)
      if (status_.mode == kEnabling) {
        status_.mode = kCalibrationComplete;

        // Now switch to the commanded mode
        ISR_MaybeChangeMode(cmd);
      }
    }

    errc err = PrepareCommand(cmd);
    if (err != errc::kSuccess) {
      status_.fault = err;
      status_.mode = kFault;
    }
  }
};

// Simulated fixture that opposes the DUT motor.
//
// Computes external_torque_ to hold position or follow a trajectory.
// It uses a PID controller with high integral gain for a stiff
// position hold.
class SimulatedFixture {
 public:
  // Configuration for the fixture behavior
  struct Config {
    float kp = 5.0f;      // Nm/rev (proportional)
    float ki = 300.0f;    // Nm/(rev*s) (integral - high for stiff hold)
    float kd = 0.6f;      // Nm/(rev/s) (derivative)
    float ilimit = 0.45f; // Nm (integral limit)

    // Maximum torque the fixture can apply
    float max_torque_Nm = 0.65f;

    Config() = default;
  };

  explicit SimulatedFixture(SimulationContext* ctx)
      : ctx_(ctx), config_() {}

  SimulatedFixture(SimulationContext* ctx, const Config& config)
      : ctx_(ctx), config_(config) {}

  // Hold at a fixed position (like fixture holding rigid)
  void HoldPosition(float position_rev) {
    mode_ = Mode::kHoldPosition;
    target_position_ = position_rev;
    target_velocity_ = 0.0f;
    integral_ = 0.0f;  // Reset integral on mode change
  }

  // Move at constant velocity (like "d pos nan <vel> <torque>")
  void MoveAtVelocity(float velocity_rev_s) {
    mode_ = Mode::kConstantVelocity;
    target_velocity_ = velocity_rev_s;

    // Initialize the position target to avoid discontinuities
    if (!position_initialized_) {
      target_position_ = ctx_->motor_sim_.position_rev();
      position_initialized_ = true;
    }
    integral_ = 0.0f;  // Reset integral on mode change
  }

  // Stop applying fixture torque
  void Stop() {
    mode_ = Mode::kStopped;
    position_initialized_ = false;
    integral_ = 0.0f;
  }

  // Get the current fixture position (tracks motor position in velocity mode)
  float position() const { return target_position_; }

  // Get the current fixture velocity
  float velocity() const { return target_velocity_; }

  // Reset integral term (call before starting a new test)
  void ResetIntegral() { integral_ = 0.0f; }

  // Update fixture and compute external torque. Call this each simulation step.
  void Update(float dt) {
    if (mode_ == Mode::kStopped) {
      ctx_->external_torque_ = 0.0f;
      return;
    }

    // In velocity mode, integrate the target position
    if (mode_ == Mode::kConstantVelocity) {
      target_position_ += target_velocity_ * dt;
    }

    // Compute position and velocity error using motor model's actual values
    const float position_error = target_position_ - ctx_->motor_sim_.position_rev();
    const float velocity_error = target_velocity_ - ctx_->motor_sim_.velocity_rev_s();

    // Update integral (with anti-windup via clamping)
    integral_ += config_.ki * position_error * dt;
    integral_ = std::max(-config_.ilimit, std::min(config_.ilimit, integral_));

    // PID control to compute required torque
    float torque = config_.kp * position_error +
                   integral_ +
                   config_.kd * velocity_error;

    // Clamp to max torque
    torque = std::max(-config_.max_torque_Nm,
                      std::min(config_.max_torque_Nm, torque));

    // Apply as external torque.
    //
    // The fixture "pushes back" against the motor
    ctx_->external_torque_ = torque;
  }

  Config& config() { return config_; }
  const Config& config() const { return config_; }

  // Preset: Rigid hold (high PID gains for stiff position holding)
  //
  // Also resets fixture state (calls Stop() internally).
  void ConfigureRigidHold(float max_torque_Nm = 0.65f) {
    Stop();  // Reset position_initialized_ and integral
    config_.kp = 5.0f;
    config_.ki = 300.0f;
    config_.kd = 0.6f;
    config_.ilimit = 0.45f;
    config_.max_torque_Nm = max_torque_Nm;
  }

  // Preset: Velocity damping only (for power limit tests)
  //
  // No position or integral control, just resistive damping
  void ConfigureVelocityDamping(float kd = 0.15f, float max_torque_Nm = 1.0f) {
    config_.kp = 0.0f;
    config_.ki = 0.0f;
    config_.kd = kd;
    config_.ilimit = 0.0f;
    config_.max_torque_Nm = max_torque_Nm;
    integral_ = 0.0f;
  }

  // Preset: Strong hold for brake mode tests
  void ConfigureStrongHold() {
    config_.kp = 50.0f;
    config_.ki = 300.0f;
    config_.kd = 2.0f;
    config_.ilimit = 0.45f;
    config_.max_torque_Nm = 1.0f;
    integral_ = 0.0f;
  }

 private:
  enum class Mode {
    kStopped,
    kHoldPosition,
    kConstantVelocity,
  };

  SimulationContext* ctx_;
  Config config_;
  Mode mode_ = Mode::kStopped;
  float target_position_ = 0.0f;
  float target_velocity_ = 0.0f;
  bool position_initialized_ = false;
  float integral_ = 0.0f;  // Integral accumulator for PID
};

// Method implementations that require SimulatedFixture to be complete
inline void SimulationContext::StepWithFixture(
    BldcServoCommandData* cmd, SimulatedFixture* fixture) {
  const float dt = rate_config_.period_s;
  if (fixture) {
    fixture->Update(dt);
  }
  StepSimulation(cmd);
}

inline int SimulationContext::RunWithFixture(
    BldcServoCommandData* cmd, SimulatedFixture* fixture, float duration_s) {
  const float dt = rate_config_.period_s;
  const int num_steps = static_cast<int>(duration_s / dt);

  for (int i = 0; i < num_steps; i++) {
    StepWithFixture(cmd, fixture);
  }

  return num_steps;
}

// Simulation fixture for Boost.Test
// Each test case gets a fresh instance with ctx and fixture reset to initial state.
struct SimulationFixture {
  SimulationContext ctx;
  SimulatedFixture fixture{&ctx};

  SimulationFixture() {
    // Ensure clean starting state for each test
    ctx.Reset();
    fixture.Stop();
    fixture.ResetIntegral();
  }
};

// ==========================================================================
// Configuration helpers for common test setups
// ==========================================================================

// Configure voltage mode control settings
// Uses: kp=4.0, ki=0.0, kd=0.03, bemf_feedforward=1.0
inline void ConfigureVoltageModeControl(SimulationContext& ctx) {
  ctx.config_.voltage_mode_control = true;
  ctx.pid_position_config.kp = 4.0f;
  ctx.pid_position_config.ki = 0.0f;
  ctx.pid_position_config.kd = 0.03f;
  ctx.config_.bemf_feedforward = 1.0f;
  // Allow BEMF feedforward without accel_limit (needed for
  // stop_position tests since stop_position can't be used with
  // accel_limit)
  ctx.config_.bemf_feedforward_override = true;
}

// Configure fixed voltage mode (stepper-like operation)
// Uses: voltage_mode_control=true, fixed_voltage_mode=true,
//       fixed_voltage_control_V=0.35, bemf_feedforward=1.0,
//       kp=1.0, ki=0.0, kd=0.01
inline void ConfigureFixedVoltageMode(SimulationContext& ctx) {
  ctx.config_.voltage_mode_control = true;
  ctx.config_.fixed_voltage_mode = true;
  ctx.config_.fixed_voltage_control_V = 0.35f;
  ctx.config_.bemf_feedforward = 1.0f;
  // Allow BEMF feedforward without accel_limit (needed for
  // stop_position tests since stop_position can't be used with
  // accel_limit)
  ctx.config_.bemf_feedforward_override = true;
  ctx.config_.pid_position.kp = 1.0f;
  ctx.config_.pid_position.ki = 0.0f;
  ctx.config_.pid_position.kd = 0.01f;
}

// Set up reversed output sign configuration
//
// Also allows specifying BEMF feedforward (0.0 or 1.0 typically)
inline void SetupReversedOutput(SimulationContext& ctx, float bemf) {
  ctx.motor_position_.config()->output.sign = -1;
  ctx.ReconfigureMotorPosition();
  ctx.config_.bemf_feedforward = bemf;
  // Allow bemf_feedforward without accel_limit (needed for
  // stop_position tests)
  ctx.config_.bemf_feedforward_override = (bemf != 0.0f);
  ctx.config_.pid_position.kp = 1.0f;
  ctx.config_.pid_position.ki = 0.0f;
  ctx.config_.pid_position.kd = 0.05f;
  ctx.position_config_.position_min = kNaN;
  ctx.position_config_.position_max = kNaN;
}

// ==========================================================================
// Command initialization helpers
// ==========================================================================

// Create a default position mode command with common settings
inline BldcServoCommandData MakePositionCommand(
    float position = kNaN,
    float velocity = 0.0f,
    float max_torque_Nm = 0.3f) {
  BldcServoCommandData cmd;
  cmd.mode = kPosition;
  cmd.position = position;
  cmd.velocity = velocity;
  cmd.max_torque_Nm = max_torque_Nm;
  cmd.timeout_s = kNaN;
  return cmd;
}

// Create a current mode command
inline BldcServoCommandData MakeCurrentCommand(
    float i_d_A = 0.0f,
    float i_q_A = 0.0f) {
  BldcServoCommandData cmd;
  cmd.mode = kCurrent;
  cmd.i_d_A = i_d_A;
  cmd.i_q_A = i_q_A;
  cmd.timeout_s = kNaN;
  return cmd;
}

// Create a voltage FOC (PWM) mode command
inline BldcServoCommandData MakeVoltageFocCommand(
    float theta = 0.0f,
    float voltage = 0.0f) {
  BldcServoCommandData cmd;
  cmd.mode = kVoltageFoc;
  cmd.theta = theta;
  cmd.voltage = voltage;
  cmd.timeout_s = kNaN;
  return cmd;
}

// Create a brake mode command
inline BldcServoCommandData MakeBrakeCommand() {
  BldcServoCommandData cmd;
  cmd.mode = kBrake;
  cmd.timeout_s = kNaN;
  return cmd;
}

// Create a stay-within-bounds mode command
inline BldcServoCommandData MakeStayWithinCommand(
    float bounds_min = kNaN,
    float bounds_max = kNaN,
    float feedforward_Nm = 0.0f,
    float max_torque_Nm = 0.3f) {
  BldcServoCommandData cmd;
  cmd.mode = kStayWithinBounds;
  cmd.bounds_min = bounds_min;
  cmd.bounds_max = bounds_max;
  cmd.feedforward_Nm = feedforward_Nm;
  cmd.max_torque_Nm = max_torque_Nm;
  cmd.timeout_s = kNaN;
  cmd.kp_scale = 1.0f;
  cmd.kd_scale = 1.0f;
  return cmd;
}

}  // namespace test
}  // namespace moteus
