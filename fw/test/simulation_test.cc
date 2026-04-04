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

#include <boost/test/auto_unit_test.hpp>

#include <cmath>
#include <string>

#include "fmt/format.h"

#include "fw/test/simulation_fixture.h"

using namespace moteus;
using namespace moteus::test;

BOOST_FIXTURE_TEST_SUITE(SimulationTests, SimulationFixture)

// Position hold at zero with zero velocity command
BOOST_AUTO_TEST_CASE(SimPositionHoldZeroVelocity) {
  auto cmd = MakePositionCommand(0.0f, 0.0f, 1.0f);
  ctx.Command(&cmd);

  // Run for 0.5 seconds to settle
  ctx.RunSimulation(&cmd, 0.5f);

  // Verify position error < 0.01 rev
  BOOST_CHECK_MESSAGE(
      IsClose(ctx.status_.position, 0.0f, 0.01f),
      "Position error: expected ~0, got " << ctx.status_.position);

  // Verify velocity < 0.1 rev/s
  BOOST_CHECK_MESSAGE(
      IsClose(ctx.status_.velocity, 0.0f, 0.1f),
      "Velocity error: expected ~0, got " << ctx.status_.velocity);
}

// Position hold with external torque disturbance
BOOST_AUTO_TEST_CASE(SimPositionHoldWithExternalTorque) {
  auto cmd = MakePositionCommand(0.0f, 0.0f, 1.0f);
  ctx.Command(&cmd);

  // First settle without disturbance
  ctx.RunSimulation(&cmd, 0.2f);

  // Apply external load torque (-0.1 Nm, negative = opposes positive motion)
  ctx.external_torque_ = -0.1f;

  // Run for another 0.5 seconds
  ctx.RunSimulation(&cmd, 0.5f);

  // Verify position is maintained (within 0.03 rev with disturbance)
  BOOST_CHECK_MESSAGE(
      IsClose(ctx.status_.position, 0.0f, 0.03f),
      "Position error with torque: expected ~0, got " << ctx.status_.position);

  // Verify q-axis current is compensating by a non-trivial amount
  BOOST_CHECK_MESSAGE(
      ctx.status_.q_A > 0.1f,
      "Q-axis current should be positive to compensate load, got "
          << ctx.status_.q_A);
}

// Constant velocity tracking using trajectory generation
BOOST_AUTO_TEST_CASE(SimPositionConstantVelocity) {
  auto cmd = MakePositionCommand(kNaN, 1.0f, 1.0f);
  cmd.accel_limit = 10.0f;  // Accelerate at 10 rev/s^2

  ctx.Command(&cmd);

  // Run for 0.5 seconds (should reach steady state at 1 rev/s)
  ctx.RunSimulation(&cmd, 0.5f);

  // Verify velocity tracking within 10%
  BOOST_CHECK_MESSAGE(
      IsClose(ctx.status_.velocity, 1.0f, 0.15f),
      "Velocity tracking: expected ~1.0, got " << ctx.status_.velocity);
}

// Constant velocity with load torque
BOOST_AUTO_TEST_CASE(SimPositionConstantVelocityWithLoad) {
  auto cmd = MakePositionCommand(kNaN, 0.5f, 1.0f);
  cmd.accel_limit = 10.0f;

  ctx.Command(&cmd);

  // First reach steady state
  ctx.RunSimulation(&cmd, 0.3f);

  // Apply load torque (negative = opposes positive velocity)
  ctx.external_torque_ = -0.2f;  // 0.2 Nm load

  // Run more and check velocity is maintained
  ctx.RunSimulation(&cmd, 0.5f);

  // Velocity should still be close to target despite load
  BOOST_CHECK_MESSAGE(
      IsClose(ctx.status_.velocity, 0.5f, 0.15f),
      "Velocity with load: expected ~0.5, got " << ctx.status_.velocity);
}

// Step response to position command with trajectory generation
BOOST_AUTO_TEST_CASE(SimPositionStepResponse) {
  // First hold at 0
  auto cmd = MakePositionCommand(0.0f, 0.0f, 1.0f);
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 0.1f);

  // Step to position 0.25 with accel_limit (trajectory generation)
  cmd.position = 0.25f;
  cmd.accel_limit = 5.0f;
  ctx.Command(&cmd);

  // Run for enough time to complete the move and settle
  // With accel=5 and distance=0.25, trajectory takes ~0.45s
  ctx.RunSimulation(&cmd, 0.8f);

  // Verify position reaches target within tolerance
  BOOST_CHECK_MESSAGE(
      IsClose(ctx.status_.position, 0.25f, 0.01f),
      "Position step: expected ~0.25, got " << ctx.status_.position);

  // Verify settled (velocity near 0)
  BOOST_CHECK_MESSAGE(
      std::abs(ctx.status_.velocity) < 0.1f,
      "Velocity after step: expected ~0, got " << ctx.status_.velocity);
}

// Position move with velocity limit
BOOST_AUTO_TEST_CASE(SimPositionWithVelocityLimit) {
  auto cmd = MakePositionCommand(0.0f, 0.0f, 1.0f);
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 0.1f);

  // Move to position 0.5 with velocity limit of 0.5 rev/s
  cmd.position = 0.5f;
  cmd.velocity_limit = 0.5f;
  cmd.accel_limit = 5.0f;
  ctx.Command(&cmd);

  // Track maximum velocity during move
  float max_velocity = 0.0f;
  const int steps = 45000;  // 1.5 seconds
  for (int i = 0; i < steps; i++) {
    ctx.StepSimulation(&cmd);
    max_velocity = std::max(max_velocity, std::abs(ctx.status_.velocity));
  }

  // Verify velocity never exceeded limit (with margin for simulation dynamics)
  BOOST_CHECK_MESSAGE(
      max_velocity < 0.65f,
      "Velocity exceeded limit: max=" << max_velocity << " limit=0.5");

  // Verify reached target
  BOOST_CHECK_MESSAGE(
      IsClose(ctx.status_.position, 0.5f, 0.02f),
      "Position: expected ~0.5, got " << ctx.status_.position);

  // Verify settled (velocity near 0)
  BOOST_CHECK_MESSAGE(
      std::abs(ctx.status_.velocity) < 0.1f,
      "Velocity after step: expected ~0, got " << ctx.status_.velocity);
}

// Test motor simulator in isolation: steady-state speed with constant voltage
BOOST_AUTO_TEST_CASE(SimMotorSteadyStateSpeed) {
  SpmsmMotorSimulator motor;

  const float V_q = 2.0f;  // Volts
  const float dt = 1.0f / 30000.0f;

  // Run for 1 second to reach steady state
  for (int i = 0; i < 30000; i++) {
    motor.StepDq(dt, 0.0f, V_q, 0.0f);
  }

  // At steady state: omega_e = V_q / lambda_m
  // omega_m = V_q / (pole_pairs * lambda_m) = 2 / (7 * 0.0026) = 110 rad/s
  const float expected_omega = V_q / (Mj5208Params::kPolePairs *
                                       Mj5208Params::kLambdaM);
  BOOST_CHECK_MESSAGE(
      IsClose(motor.omega_mechanical(), expected_omega, 5.0f),
      "2V Q axis expected omega " << expected_omega << ", got "
      << motor.omega_mechanical());
}

// Test motor simulator: torque generation
BOOST_AUTO_TEST_CASE(SimMotorTorqueGeneration) {
  SpmsmMotorSimulator motor;

  const float kIq = 5.0f;

  // Set a known i_q current
  motor.state().i_q = kIq;

  // Expected torque: T = 1.5 * pole_pairs * lambda_m * i_q
  // This equals Kt * i_q where Kt = 1.5 * pole_pairs * lambda_m
  const float expected_torque =
      1.5f * Mj5208Params::kPolePairs * Mj5208Params::kLambdaM * kIq;

  BOOST_CHECK_MESSAGE(
      IsClose(motor.torque_em(), expected_torque, 0.01f),
      "Torque calculation: expected " << expected_torque << ", got "
                                      << motor.torque_em());
}

// Test motor simulator: step torque acceleration
BOOST_AUTO_TEST_CASE(SimMotorStepAcceleration) {
  SpmsmMotorSimulator motor;

  const float dt = 0.0001f;  // 0.1 ms

  // Verify that with constant torque application, velocity increases.
  const float initial_omega = motor.omega_mechanical();

  // Apply q-axis voltage that creates torque
  const float V_q = 5.0f;
  for (int i = 0; i < 100; i++) {
    motor.StepDq(dt, 0.0f, V_q, 0.0f);
  }

  BOOST_CHECK_MESSAGE(
      motor.omega_mechanical() > initial_omega,
      "Velocity should increase with applied voltage, omega = "
          << motor.omega_mechanical());
}

// Test velocity tracking at different bus voltages.
// Commands 500 Hz velocity (higher than achievable) and verifies:
// 1. Final velocity is stable (low oscillation)
// 2. Final velocity matches expected value for bus voltage
BOOST_AUTO_TEST_CASE(SimVelocityAcrossBusVoltages) {
  struct VoltageTest {
    float bus_voltage;
    float expected_velocity;  // Expected velocity (rev/s)
    float tolerance_pct = 2.0f;  // Tolerance as percentage (default 2%)
  };

  // Expected velocities based on motor Kv = 304 RPM/V.
  // At back-EMF limit with SVPWM modulation, achieved velocity is
  // approximately 4.1 rev/s per volt of bus voltage in simulation.
  const VoltageTest tests[] = {
      {18.0f, 73.6f},   // 18V
      {24.0f, 98.2f},   // 24V
      {36.0f, 148.0f},  // 36V
  };

  for (const auto& test : tests) {
    ctx.Reset();

    // Set bus voltage
    ctx.SetBusVoltage(test.bus_voltage);

    // Increase max_velocity so it's not a limiting factor
    ctx.config_.max_velocity = 1000.0f;

    // Command: velocity = 500 Hz (above what any voltage can achieve)
    // with slow acceleration to avoid transients
    auto cmd = MakePositionCommand(kNaN, 500.0f, 5.0f);
    cmd.accel_limit = 100.0f;  // 50 rev/s^2 acceleration

    ctx.Command(&cmd);

    // Run for 5 seconds to reach steady state
    ctx.RunSimulation(&cmd, 5.0f);

    // Record samples over 0.5 seconds to check stability
    const int sample_steps = 15000;  // 0.5 seconds at 30kHz
    std::vector<float> velocity_samples;
    std::vector<float> d_A_samples;
    std::vector<float> q_A_samples;
    velocity_samples.reserve(sample_steps);
    d_A_samples.reserve(sample_steps);
    q_A_samples.reserve(sample_steps);

    for (int i = 0; i < sample_steps; i++) {
      ctx.StepSimulation(&cmd);
      velocity_samples.push_back(ctx.status_.velocity);
      d_A_samples.push_back(ctx.status_.d_A);
      q_A_samples.push_back(ctx.status_.q_A);
    }

    // Use CalcStats helper from fixture
    const Stats vel_stats = CalcStats(velocity_samples);
    const Stats d_A_stats = CalcStats(d_A_samples);
    const Stats q_A_stats = CalcStats(q_A_samples);

    const float mean_velocity = vel_stats.mean;
    const float vel_std_dev = vel_stats.std_dev;
    const float mean_d_A = d_A_stats.mean;
    const float d_A_std_dev = d_A_stats.std_dev;
    const float mean_q_A = q_A_stats.mean;
    const float q_A_std_dev = q_A_stats.std_dev;

    // Diagnostic output
    std::cout << "Bus " << test.bus_voltage << "V: "
              << "vel=" << mean_velocity << "±" << vel_std_dev
              << " d_A=" << mean_d_A << "±" << d_A_std_dev
              << " q_A=" << mean_q_A << "±" << q_A_std_dev
              << "\n";

    // Check 1: Velocity is stable
    const float vel_stability_threshold = 0.5f;
    BOOST_CHECK_MESSAGE(
        vel_std_dev < vel_stability_threshold,
        "Bus " << test.bus_voltage << "V: Velocity unstable, "
               << "std_dev=" << vel_std_dev
               << " threshold=" << vel_stability_threshold
               << " mean=" << mean_velocity);

    // Check 2: Velocity is in expected range for this bus voltage
    const float tolerance = test.expected_velocity * test.tolerance_pct / 100.0f;
    BOOST_CHECK_MESSAGE(
        std::abs(mean_velocity - test.expected_velocity) <= tolerance,
        "Bus " << test.bus_voltage << "V: Velocity out of range, "
               << "got " << mean_velocity << " expected "
               << test.expected_velocity << " +/- " << test.tolerance_pct << "%");

    // Check 3: Velocity scales roughly linearly with bus voltage
    // (back-EMF limited behavior). Expected ~4.5 rev/s per volt.
    const float velocity_per_volt = mean_velocity / test.bus_voltage;
    BOOST_CHECK_MESSAGE(
        velocity_per_volt >= 4.0f && velocity_per_volt <= 5.0f,
        "Bus " << test.bus_voltage << "V: velocity/volt ratio out of range, "
               << "got " << velocity_per_volt << " expected [4.0, 5.0]");

    // Check 4: D-axis current mean should be near zero (no field weakening needed)
    // On real hardware, d_A is essentially 0. Allow small margin for simulation.
    const float d_A_mean_threshold = 1.0f;  // Amps
    BOOST_CHECK_MESSAGE(
        std::abs(mean_d_A) < d_A_mean_threshold,
        "Bus " << test.bus_voltage << "V: D-axis current too high, "
               << "mean=" << mean_d_A << " threshold=" << d_A_mean_threshold);

    // Check 5: Q-axis current mean should be small in steady state at back-EMF limit
    // Some current is needed to overcome friction/damping, but should be small.
    const float q_A_mean_threshold = 2.0f;  // Amps
    BOOST_CHECK_MESSAGE(
        std::abs(mean_q_A) < q_A_mean_threshold,
        "Bus " << test.bus_voltage << "V: Q-axis current too high, "
               << "mean=" << mean_q_A << " threshold=" << q_A_mean_threshold);

    // Check 6: Current tracking stability.
    //
    // On real hardware, d_A and q_A track within approximately ±2.5A. The
    // simulation achieves better tracking (milliamp-level stddev) because it
    // lacks PWM ripple and measurement noise.
    //
    // Use a 1A threshold - well above simulation noise but catches bugs like
    // incorrect PWM-to-voltage conversion.
    const float current_std_threshold = 1.0f;  // Amps
    BOOST_CHECK_MESSAGE(
        d_A_std_dev < current_std_threshold,
        "Bus " << test.bus_voltage << "V: D-axis current stddev too high, "
               << "std_dev=" << d_A_std_dev << " threshold=" << current_std_threshold);
    BOOST_CHECK_MESSAGE(
        q_A_std_dev < current_std_threshold,
        "Bus " << test.bus_voltage << "V: Q-axis current stddev too high, "
               << "std_dev=" << q_A_std_dev << " threshold=" << current_std_threshold);
  }
}

// Test that position control error fault triggers when error exceeds threshold
BOOST_AUTO_TEST_CASE(SimPositionControlErrorFault) {
  ctx.config_.fault_position_error = 0.02f;

  auto cmd = MakePositionCommand(0.0f, 0.0f, 1.0f);
  ctx.Command(&cmd);

  // Let it settle at position 0
  ctx.RunSimulation(&cmd, 0.2f);
  BOOST_TEST(ctx.status_.mode == kPosition);

  // Apply a large external torque to push motor away from target
  ctx.external_torque_ = 0.5f;

  // Run until fault triggers
  ctx.RunSimulation(&cmd, 1.0f);

  BOOST_CHECK(ctx.status_.mode == kFault);
  BOOST_CHECK(ctx.status_.fault == errc::kPositionControlError);
}

// Test that NaN position error threshold (default) does not trigger fault
BOOST_AUTO_TEST_CASE(SimPositionControlErrorFaultDisabled) {
  // fault_position_error defaults to NaN

  auto cmd = MakePositionCommand(0.0f, 0.0f, 1.0f);
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 0.2f);

  // Apply same external torque
  ctx.external_torque_ = 0.5f;
  ctx.RunSimulation(&cmd, 0.5f);

  // Should NOT have faulted
  BOOST_CHECK(ctx.status_.mode != kFault);
}

// Test that velocity control error fault triggers when error rate exceeds threshold
BOOST_AUTO_TEST_CASE(SimVelocityControlErrorFault) {
  ctx.config_.fault_velocity_error = 0.5f;

  // Command constant velocity
  auto cmd = MakePositionCommand(kNaN, 2.0f, 1.0f);
  cmd.accel_limit = 10.0f;
  ctx.Command(&cmd);

  // Let it get up to speed
  ctx.RunSimulation(&cmd, 0.5f);
  BOOST_TEST(ctx.status_.mode == kPosition);

  // Apply a large opposing torque to create velocity error
  ctx.external_torque_ = -1.0f;

  // Run until fault triggers
  ctx.RunSimulation(&cmd, 1.0f);

  BOOST_CHECK(ctx.status_.mode == kFault);
  BOOST_CHECK(ctx.status_.fault == errc::kVelocityControlError);
}

// Test that NaN velocity error threshold (default) does not trigger fault
BOOST_AUTO_TEST_CASE(SimVelocityControlErrorFaultDisabled) {
  // fault_velocity_error defaults to NaN

  auto cmd = MakePositionCommand(kNaN, 2.0f, 1.0f);
  cmd.accel_limit = 10.0f;
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 0.5f);

  // Apply opposing torque
  ctx.external_torque_ = -1.0f;
  ctx.RunSimulation(&cmd, 0.5f);

  // Should NOT have faulted
  BOOST_CHECK(ctx.status_.mode != kFault);
}

// Field weakening velocity tracking test.
//
// Tests that the motor's mechanical velocity tracks the controller's
// commanded velocity during acceleration and deceleration through
// the field weakening region.
BOOST_AUTO_TEST_CASE(SimFieldWeakeningVelocityTracking) {
  // Configure for field weakening with 14V supply
  ctx.SetBusVoltage(14.0f);

  const auto& mp = Mj5208Params();
  ctx.motor_.inductance_d_H = mp.kL;
  ctx.motor_.inductance_q_H = mp.kL;

  ctx.config_.fw.enable = true;
  ctx.config_.fw.modulation_margin = 0.15f;
  ctx.config_.fw.bandwidth_hz = 10.0f;
  ctx.config_.fw.max_current_ratio = 0.5f;

  ctx.UpdateDerivedMotorConstants();
  ctx.UpdateFieldWeakeningIdChar();

  ctx.config_.max_velocity = 200.0f;

  // Test parameters
  const float accel_limit = 150.0f;  // Hz/s (rev/s^2)
  const float velocity_tolerance = 2.0f;  // Hz tolerance

  // Speeds to test (Hz = rev/s)
  const std::vector<float> test_speeds = {40.0f, 50.0f, 60.0f, 80.0f,
                                          100.0f, 120.0f, 130.0f};

  for (const float target_speed : test_speeds) {
    // Reset motor state for each speed test
    ctx.motor_sim_.state() = SpmsmMotorSimulator::State{};
    ctx.Reset();
    fixture.Stop();
    fixture.ResetIntegral();

    ctx.SetBusVoltage(14.0f);

    // Track maximum velocity error during the test
    float max_velocity_error = 0.0f;
    int violation_count = 0;

    // Phase 1: Accelerate to target velocity
    auto cmd = MakePositionCommand(kNaN, target_speed, 1.0f);
    cmd.accel_limit = accel_limit;
    ctx.Command(&cmd);

    // Time to reach target speed + settling margin
    const float accel_time = target_speed / accel_limit + 0.2f;
    const int accel_steps = static_cast<int>(accel_time * ctx.rate_config_.rate_hz);

    for (int i = 0; i < accel_steps; i++) {
      ctx.StepSimulation(&cmd);

      // Check velocity tracking once we have a control velocity
      if (ctx.status_.control_velocity.has_value()) {
        const float control_vel = ctx.status_.control_velocity.value();
        const float actual_vel = ctx.motor_sim_.velocity_rev_s();  // Mechanical velocity from simulator
        const float error = std::abs(control_vel - actual_vel);

        max_velocity_error = std::max(max_velocity_error, error);
        if (error > velocity_tolerance) {
          violation_count++;
        }
      }
    }

    // Phase 2: Decelerate to zero
    cmd.velocity = 0.0f;
    ctx.Command(&cmd);

    const float decel_time = target_speed / accel_limit + 0.2f;
    const int decel_steps = static_cast<int>(decel_time * ctx.rate_config_.rate_hz);

    for (int i = 0; i < decel_steps; i++) {
      ctx.StepSimulation(&cmd);

      if (ctx.status_.control_velocity.has_value()) {
        const float control_vel = ctx.status_.control_velocity.value();
        const float actual_vel = ctx.motor_sim_.velocity_rev_s();  // Mechanical velocity from simulator
        const float error = std::abs(control_vel - actual_vel);

        max_velocity_error = std::max(max_velocity_error, error);
        if (error > velocity_tolerance) {
          violation_count++;
        }
      }
    }

    // Verify velocity tracking was within tolerance
    // Allow a small number of violations during transients
    const int max_violations = static_cast<int>(
        0.0001f * (accel_steps + decel_steps));  // 0.01% of samples
    BOOST_CHECK_MESSAGE(
        violation_count <= max_violations,
        "Speed " << target_speed << " Hz: Too many velocity tracking violations, "
                 << "count=" << violation_count << " max_allowed=" << max_violations
                 << " max_error=" << max_velocity_error << " Hz");

    BOOST_CHECK_MESSAGE(
        max_velocity_error < velocity_tolerance * 2.0f,
        "Speed " << target_speed << " Hz: Max velocity error too large, "
                 << "error=" << max_velocity_error << " Hz");

    // Verify motor came to rest
    BOOST_CHECK_MESSAGE(
        std::abs(ctx.motor_sim_.velocity_rev_s()) < 1.0f,
        "Speed " << target_speed << " Hz: Motor did not stop, "
                 << "final_velocity=" << ctx.motor_sim_.velocity_rev_s() << " Hz");
  }
}

// Helper to configure field weakening for limit tests.
//
// Sets up the bus voltage with field weakening enabled and runs the
// motor to a target velocity in the field weakening region.  Returns
// the command used (caller can continue stepping).
static BldcServoCommandData SetupFieldWeakeningAtSpeed(
    SimulationContext& ctx,
    float target_speed = 100.0f,
    float bus_V = 14.0f,
    float settle_time = 0.0f) {
  ctx.SetBusVoltage(bus_V);

  const auto& mp = Mj5208Params();
  ctx.motor_.inductance_d_H = mp.kL;
  ctx.motor_.inductance_q_H = mp.kL;

  ctx.config_.fw.enable = true;
  ctx.config_.fw.modulation_margin = 0.15f;
  ctx.config_.fw.bandwidth_hz = 10.0f;
  ctx.config_.fw.max_current_ratio = 0.5f;

  ctx.config_.bemf_feedforward = 1.0f;

  ctx.UpdateDerivedMotorConstants();
  ctx.UpdateFieldWeakeningIdChar();

  ctx.config_.max_velocity = 200.0f;

  auto cmd = MakePositionCommand(kNaN, target_speed, 1.0f);
  cmd.accel_limit = 150.0f;
  ctx.Command(&cmd);

  // Accelerate to target speed and optionally settle
  const float accel_time = target_speed / 150.0f + 0.3f;
  ctx.RunSimulation(&cmd, accel_time + settle_time);

  return cmd;
}

// Field weakening with power limit.
//
// Tests two operating points:
// 1. 14V/100Hz: Motor at max FW speed (~61 rev/s), power limit reduces
//    current but speed is already at maximum achievable.
// 2. 24V/100Hz: Motor tracking 100 rev/s in FW region (base ~82),
//    power limit forces speed reduction since the motor cannot sustain
//    the current speed with reduced power.
BOOST_AUTO_TEST_CASE(SimFieldWeakeningPowerLimit) {
  // --- Operating point 1: 14V, max FW speed ---
  {
    auto cmd = SetupFieldWeakeningAtSpeed(ctx, 100.0f, 14.0f, 1.0f);
    ctx.RunSimulation(&cmd, 0.1f);

    const float baseline_power = std::abs(ctx.status_.power_W);
    const float baseline_vel = ctx.motor_sim_.velocity_rev_s();

    const float power_limit = baseline_power * 0.5f;
    ctx.status_.max_power_W = power_limit;
    ctx.RunSimulation(&cmd, 1.0f);

    float sum_power = 0.0f;
    int limit_count = 0;
    const int sample_steps = 3000;
    for (int i = 0; i < sample_steps; i++) {
      ctx.StepSimulation(&cmd);
      sum_power += std::abs(ctx.status_.power_W);
      if (ctx.status_.fault == errc::kLimitMaxPower) limit_count++;
    }
    const float avg_power = sum_power / sample_steps;
    const float final_vel = ctx.motor_sim_.velocity_rev_s();

    std::cout << "FW power limit 14V:"
              << " base_pwr=" << baseline_power
              << " limit=" << power_limit
              << " avg_pwr=" << avg_power
              << " base_vel=" << baseline_vel
              << " final_vel=" << final_vel
              << " active=" << limit_count << "/" << sample_steps
              << std::endl;

    BOOST_CHECK_MESSAGE(limit_count > 0,
        "14V: Power limit was never active");
    BOOST_CHECK_MESSAGE(avg_power < baseline_power * 0.9f,
        "14V: Power not reduced: " << avg_power << " vs " << baseline_power);
    BOOST_CHECK_MESSAGE(ctx.status_.mode != kFault,
        "14V: Fault mode, fault=" << static_cast<int>(ctx.status_.fault));
    BOOST_CHECK_MESSAGE(std::abs(final_vel) > 10.0f,
        "14V: Motor stopped, vel=" << final_vel);
  }

  // --- Operating point 2: 24V/120Hz with load, speed above base ---
  //
  // At 24V commanding 120 rev/s the motor saturates at ~105 (base ~82)
  // with max FW.  A load torque ensures the motor draws significant
  // q-axis power.  A moderate power limit forces the motor to slow
  // but remain above base speed.
  {
    ctx.motor_sim_.state() = SpmsmMotorSimulator::State{};
    ctx.Reset();
    ctx.status_.max_power_W = 500.0f;  // restore default

    auto cmd = SetupFieldWeakeningAtSpeed(ctx, 120.0f, 24.0f, 2.0f);

    // Apply a constant load torque opposing the direction of rotation.
    ctx.external_torque_ = -0.15f;
    ctx.RunSimulation(&cmd, 1.0f);

    const float baseline_power = std::abs(ctx.status_.power_W);
    const float baseline_vel = ctx.motor_sim_.velocity_rev_s();
    const float base_speed = ctx.status_.motor_base_velocity;

    BOOST_CHECK_MESSAGE(baseline_vel > base_speed,
        "24V: Not above base speed: vel=" << baseline_vel
        << " base=" << base_speed);

    // Limit power to 50% of baseline.
    const float power_limit = baseline_power * 0.5f;
    ctx.status_.max_power_W = power_limit;
    ctx.RunSimulation(&cmd, 2.0f);

    float sum_power = 0.0f;
    int limit_count = 0;
    const int sample_steps = 3000;
    for (int i = 0; i < sample_steps; i++) {
      ctx.StepSimulation(&cmd);
      sum_power += std::abs(ctx.status_.power_W);
      if (ctx.status_.fault == errc::kLimitMaxPower) limit_count++;
    }
    const float avg_power = sum_power / sample_steps;
    const float final_vel = ctx.motor_sim_.velocity_rev_s();

    std::cout << "FW power limit 24V+load:"
              << " base_pwr=" << baseline_power
              << " limit=" << power_limit
              << " avg_pwr=" << avg_power
              << " base_vel=" << baseline_vel
              << " final_vel=" << final_vel
              << " base_spd=" << base_speed
              << " active=" << limit_count << "/" << sample_steps
              << std::endl;

    BOOST_CHECK_MESSAGE(limit_count > 0,
        "24V: Power limit was never active");
    BOOST_CHECK_MESSAGE(avg_power < baseline_power * 0.9f,
        "24V: Power not reduced: " << avg_power << " vs " << baseline_power);
    BOOST_CHECK_MESSAGE(final_vel < baseline_vel * 0.95f,
        "24V: Speed was not reduced: " << final_vel
        << " vs baseline " << baseline_vel);
    BOOST_CHECK_MESSAGE(final_vel > base_speed,
        "24V: Speed fell below base speed: " << final_vel
        << " vs base " << base_speed);
    BOOST_CHECK_MESSAGE(ctx.status_.mode != kFault,
        "24V: Fault mode, fault=" << static_cast<int>(ctx.status_.fault));
  }
}

// Field weakening with FET temperature limit.
//
// Tests two operating points:
// 1. 14V/100Hz: Motor at max FW, temperature derating reduces current
//    command.
// 2. 24V/100Hz: Motor at stable FW speed, temperature derating forces
//    speed reduction.
BOOST_AUTO_TEST_CASE(SimFieldWeakeningFetTempLimit) {
  // Set fault_temperature high enough to avoid hard kOverTemperature.
  ctx.config_.fault_temperature = 150.0f;

  // --- Operating point 1: 14V, max FW speed ---
  {
    auto cmd = SetupFieldWeakeningAtSpeed(ctx, 100.0f, 14.0f, 1.0f);
    ctx.RunSimulation(&cmd, 0.1f);
    const float baseline_i_d = ctx.control_.i_d_A;

    const float halfway_temp = ctx.derate_temperature_ +
        ctx.config_.temperature_margin * 0.5f;
    ctx.status_.filt_fet_temp_C = halfway_temp;
    ctx.RunSimulation(&cmd, 2.0f);
    const float derated_i_d = ctx.control_.i_d_A;

    std::cout << "FW FET temp 14V:"
              << " base_id=" << baseline_i_d
              << " derated_id=" << derated_i_d
              << " vel=" << ctx.motor_sim_.velocity_rev_s()
              << std::endl;

    BOOST_CHECK_MESSAGE(
        std::abs(derated_i_d) < std::abs(baseline_i_d) * 0.9f,
        "14V: D current not reduced: " << derated_i_d
        << " vs " << baseline_i_d);
    BOOST_CHECK_MESSAGE(ctx.status_.mode != kFault,
        "14V: Fault mode, fault=" << static_cast<int>(ctx.status_.fault));
  }

  // --- Operating point 2: 24V/120Hz + load, derating above base speed ---
  //
  // At 24V with load commanding 120 rev/s, the motor operates in FW
  // above base speed.  Temperature derating reduces both the max
  // current and the achievable FW speed.  A load torque ensures the
  // motor needs sustained q-axis current to maintain speed.
  {
    ctx.motor_sim_.state() = SpmsmMotorSimulator::State{};
    ctx.Reset();
    ctx.status_.filt_fet_temp_C = 25.0f;  // cool down

    auto cmd = SetupFieldWeakeningAtSpeed(ctx, 120.0f, 24.0f, 2.0f);

    ctx.external_torque_ = -0.15f;
    ctx.RunSimulation(&cmd, 1.0f);

    const float baseline_vel = ctx.motor_sim_.velocity_rev_s();
    const float base_speed = ctx.status_.motor_base_velocity;

    BOOST_CHECK_MESSAGE(baseline_vel > base_speed,
        "24V: Not above base speed: vel=" << baseline_vel
        << " base=" << base_speed);

    // 55% through the derating range:
    // temp_limit_A = max(0, 0.55*(-20-40)+40) = 7 A
    const float warm_temp = ctx.derate_temperature_ +
        ctx.config_.temperature_margin * 0.55f;
    ctx.status_.filt_fet_temp_C = warm_temp;
    ctx.RunSimulation(&cmd, 3.0f);
    const float final_vel = ctx.motor_sim_.velocity_rev_s();

    std::cout << "FW FET temp 24V:"
              << " base_vel=" << baseline_vel
              << " final_vel=" << final_vel
              << " base_spd=" << base_speed
              << " temp=" << warm_temp
              << " ctrl_id=" << ctx.control_.i_d_A
              << std::endl;

    BOOST_CHECK_MESSAGE(final_vel < baseline_vel * 0.95f,
        "24V: Speed was not reduced: " << final_vel
        << " vs baseline " << baseline_vel);
    BOOST_CHECK_MESSAGE(final_vel > base_speed,
        "24V: Speed fell below base speed: " << final_vel
        << " vs base " << base_speed);
    BOOST_CHECK_MESSAGE(ctx.status_.mode != kFault,
        "24V: Fault mode, fault=" << static_cast<int>(ctx.status_.fault));
  }
}

// Field weakening with motor temperature limit.
//
// Tests two operating points:
// 1. 14V/100Hz: Motor at max FW, motor temperature derating reduces
//    current command.
// 2. 24V/100Hz: Motor at stable FW speed, motor temperature derating
//    forces speed reduction.
BOOST_AUTO_TEST_CASE(SimFieldWeakeningMotorTempLimit) {
  // --- Operating point 1: 14V, max FW speed ---
  {
    // Enable motor temperature limiting.
    ctx.config_.motor_fault_temperature = 120.0f;
    ctx.config_.motor_temperature_margin = 20.0f;
    ctx.motor_derate_temperature_ = 100.0f;

    auto cmd = SetupFieldWeakeningAtSpeed(ctx, 100.0f, 14.0f, 1.0f);
    ctx.RunSimulation(&cmd, 0.1f);
    const float baseline_i_d = ctx.control_.i_d_A;

    const float halfway_temp = ctx.motor_derate_temperature_ +
        ctx.config_.motor_temperature_margin * 0.5f;
    ctx.status_.filt_motor_temp_C = halfway_temp;
    ctx.RunSimulation(&cmd, 2.0f);
    const float derated_i_d = ctx.control_.i_d_A;

    std::cout << "FW motor temp 14V:"
              << " base_id=" << baseline_i_d
              << " derated_id=" << derated_i_d
              << " vel=" << ctx.motor_sim_.velocity_rev_s()
              << std::endl;

    BOOST_CHECK_MESSAGE(
        std::abs(derated_i_d) < std::abs(baseline_i_d) * 0.9f,
        "14V: D current not reduced: " << derated_i_d
        << " vs " << baseline_i_d);
    BOOST_CHECK_MESSAGE(ctx.status_.mode != kFault,
        "14V: Fault mode, fault=" << static_cast<int>(ctx.status_.fault));
  }

  // --- Operating point 2: 24V/120Hz + load, derating above base speed ---
  //
  // Same approach as the FET temp test: temperature derating reduces
  // the achievable FW speed, and the load torque ensures sustained
  // q-axis demand.
  {
    ctx.motor_sim_.state() = SpmsmMotorSimulator::State{};
    ctx.Reset();
    ctx.status_.filt_motor_temp_C = 25.0f;  // cool down

    ctx.config_.motor_fault_temperature = 120.0f;
    ctx.config_.motor_temperature_margin = 20.0f;
    ctx.motor_derate_temperature_ = 100.0f;

    auto cmd = SetupFieldWeakeningAtSpeed(ctx, 120.0f, 24.0f, 2.0f);

    ctx.external_torque_ = -0.15f;
    ctx.RunSimulation(&cmd, 1.0f);

    const float baseline_vel = ctx.motor_sim_.velocity_rev_s();
    const float base_speed = ctx.status_.motor_base_velocity;

    BOOST_CHECK_MESSAGE(baseline_vel > base_speed,
        "24V: Not above base speed: vel=" << baseline_vel
        << " base=" << base_speed);

    // 55% through the derating range: temp_limit_A = 7 A
    const float warm_temp = ctx.motor_derate_temperature_ +
        ctx.config_.motor_temperature_margin * 0.55f;
    ctx.status_.filt_motor_temp_C = warm_temp;
    ctx.RunSimulation(&cmd, 3.0f);
    const float final_vel = ctx.motor_sim_.velocity_rev_s();

    std::cout << "FW motor temp 24V:"
              << " base_vel=" << baseline_vel
              << " final_vel=" << final_vel
              << " base_spd=" << base_speed
              << " temp=" << warm_temp
              << " ctrl_id=" << ctx.control_.i_d_A
              << std::endl;

    BOOST_CHECK_MESSAGE(final_vel < baseline_vel * 0.95f,
        "24V: Speed was not reduced: " << final_vel
        << " vs baseline " << baseline_vel);
    BOOST_CHECK_MESSAGE(final_vel > base_speed,
        "24V: Speed fell below base speed: " << final_vel
        << " vs base " << base_speed);
    BOOST_CHECK_MESSAGE(ctx.status_.mode != kFault,
        "24V: Fault mode, fault=" << static_cast<int>(ctx.status_.fault));
  }
}

// Field weakening with temperature limit and no external load.
//
// Verifies that temperature derating reduces the achievable FW speed
// even when there is no load torque opposing the motor.  The firmware
// must reduce motor_max_velocity to account for the derated current,
// preventing the position controller from commanding an unreachable
// speed.
BOOST_AUTO_TEST_CASE(SimFieldWeakeningTempLimitNoLoad) {
  ctx.config_.fault_temperature = 150.0f;

  auto cmd = SetupFieldWeakeningAtSpeed(ctx, 120.0f, 24.0f, 2.0f);
  ctx.RunSimulation(&cmd, 0.1f);

  const float baseline_vel = ctx.motor_sim_.velocity_rev_s();
  const float base_speed = ctx.status_.motor_base_velocity;

  BOOST_CHECK_MESSAGE(baseline_vel > base_speed,
      "Not above base speed: vel=" << baseline_vel
      << " base=" << base_speed);

  // 55% through the derating range (same as the loaded tests).
  const float warm_temp = ctx.derate_temperature_ +
      ctx.config_.temperature_margin * 0.55f;
  ctx.status_.filt_fet_temp_C = warm_temp;
  ctx.RunSimulation(&cmd, 3.0f);
  const float final_vel = ctx.motor_sim_.velocity_rev_s();

  std::cout << "FW temp no-load 24V:"
            << " base_vel=" << baseline_vel
            << " final_vel=" << final_vel
            << " base_spd=" << base_speed
            << " max_vel=" << ctx.status_.motor_max_velocity
            << " temp=" << warm_temp
            << " ctrl_id=" << ctx.control_.i_d_A
            << std::endl;

  BOOST_CHECK_MESSAGE(final_vel <= baseline_vel * 1.01f,
      "Speed increased under limit: " << final_vel
      << " vs baseline " << baseline_vel);
  BOOST_CHECK_MESSAGE(final_vel > base_speed,
      "Speed fell below base speed: " << final_vel
      << " vs base " << base_speed);
  BOOST_CHECK_MESSAGE(ctx.status_.mode != kFault,
      "Fault mode, fault=" << static_cast<int>(ctx.status_.fault));
}

// Helper: sweep flux braking strength and verify energy dissipation.
//
// Assumes the caller has already configured the motor and run it to
// the desired speed.  This function applies an external torque (to
// create regeneration), sweeps flux braking threshold at constant bus
// voltage, and checks that increasing flux braking monotonically
// reduces regenerated power.
static void FluxBrakeSweep(
    SimulationContext& ctx,
    SimulatedFixture& fixture,
    BldcServoCommandData cmd,
    const char* label,
    float bus_V,
    float external_torque,
    float flux_brake_R,
    const std::vector<float>& thresholds) {

  struct Result {
    float threshold;
    float expected_fb_dA;
    float avg_power_W;
    float avg_ctrl_id;
    float avg_q_A;
    float avg_vel;
    bool faulted;
    int fault_code;
  };
  std::vector<Result> results;

  std::cout << label << " sweep: bus=" << bus_V << "V"
            << " ext_torque=" << external_torque
            << " R=" << flux_brake_R << std::endl;

  // Save state so we can restore it for each sweep point.
  const auto saved_motor_state = ctx.motor_sim_.state();

  for (const float threshold : thresholds) {
    // Restore motor to the pre-sweep state.
    ctx.motor_sim_.state() = saved_motor_state;
    ctx.external_torque_ = external_torque;
    ctx.flux_brake_min_voltage_ = threshold;
    ctx.config_.flux_brake_resistance_ohm = flux_brake_R;

    // Settle for 1.5 seconds.
    bool faulted = false;
    for (int i = 0; i < 45000; i++) {
      ctx.StepSimulation(&cmd);
      if (ctx.status_.mode == kFault) { faulted = true; break; }
    }

    const float expected_fb =
        std::max(0.0f, (bus_V - threshold) / flux_brake_R);

    if (faulted) {
      std::cout << "  threshold=" << threshold
                << "V (fb_dA=" << expected_fb
                << "): FAULT=" << static_cast<int>(ctx.status_.fault)
                << std::endl;
      results.push_back({threshold, expected_fb, 0, 0, 0, 0,
                          true, static_cast<int>(ctx.status_.fault)});
      continue;
    }

    // Measure for 0.5 seconds.
    const int measure_steps = 15000;
    float sum_power = 0.0f;
    float sum_ctrl_id = 0.0f;
    float sum_q_A = 0.0f;
    float sum_vel = 0.0f;

    for (int i = 0; i < measure_steps; i++) {
      ctx.StepSimulation(&cmd);
      if (ctx.status_.mode == kFault) { faulted = true; break; }

      sum_power += ctx.status_.power_W;
      sum_ctrl_id += ctx.control_.i_d_A;
      sum_q_A += ctx.status_.q_A;
      sum_vel += ctx.motor_sim_.velocity_rev_s();
    }

    const float n = static_cast<float>(measure_steps);
    const float avg_power = sum_power / n;
    const float avg_ctrl_id = sum_ctrl_id / n;

    std::cout << "  threshold=" << threshold
              << "V (fb_dA=" << expected_fb << "):"
              << " power=" << avg_power << "W"
              << " ctrl_id=" << avg_ctrl_id << "A"
              << " q_A=" << sum_q_A / n << "A"
              << " vel=" << sum_vel / n << " rev/s"
              << " fault=" << static_cast<int>(ctx.status_.fault)
              << std::endl;

    results.push_back({threshold, expected_fb, avg_power, avg_ctrl_id,
                        sum_q_A / n, sum_vel / n,
                        faulted, static_cast<int>(ctx.status_.fault)});
  }

  // --- Assertions ---

  for (const auto& r : results) {
    BOOST_CHECK_MESSAGE(!r.faulted,
        label << ": hard fault at threshold=" << r.threshold
              << "V, fault=" << r.fault_code);
  }

  // Baseline (first point, no flux braking): should be regenerating.
  const auto& baseline = results.front();
  BOOST_CHECK_MESSAGE(baseline.avg_power_W < -0.1f,
      label << ": baseline should regenerate, got "
            << baseline.avg_power_W << "W");

  // Power should be monotonically non-decreasing as flux braking
  // increases (more dissipation → less regeneration).
  for (size_t i = 1; i < results.size(); i++) {
    if (results[i].faulted || results[i - 1].faulted) continue;
    if (results[i].expected_fb_dA <= 0.0f) continue;

    BOOST_CHECK_MESSAGE(
        results[i].avg_power_W >= results[i - 1].avg_power_W - 0.5f,
        label << ": power should be non-decreasing: "
              << "fb_dA=" << results[i - 1].expected_fb_dA
              << " power=" << results[i - 1].avg_power_W << "W"
              << " -> fb_dA=" << results[i].expected_fb_dA
              << " power=" << results[i].avg_power_W << "W");
  }

  // At the highest flux braking level, power should exceed baseline.
  const auto& max_fb = results.back();
  if (!max_fb.faulted) {
    BOOST_CHECK_MESSAGE(max_fb.avg_power_W > baseline.avg_power_W,
        label << ": max FB should reduce regen: "
              << max_fb.avg_power_W << "W vs " << baseline.avg_power_W << "W");
  }
}

// Flux braking below base speed (no field weakening).
//
// At 24V the MJ5208 base speed is ~82 rev/s.  Running at 20 rev/s
// with an external torque that assists rotation creates a
// regenerative operating point with no field weakening active.
// Flux braking should dissipate that energy via I²R.
BOOST_AUTO_TEST_CASE(SimFluxBraking) {
  const float bus_V = 24.0f;
  const float target_speed = 20.0f;
  const float external_torque = 0.15f;
  const float flux_brake_R = 0.1f;

  auto cmd = MakePositionCommand(kNaN, target_speed, 1.0f);
  cmd.accel_limit = 50.0f;
  ctx.SetBusVoltage(bus_V);
  ctx.config_.max_voltage = 50.0f;
  ctx.config_.bemf_feedforward = 1.0f;
  ctx.flux_brake_min_voltage_ = 100.0f;  // disabled during accel
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 1.0f);

  FluxBrakeSweep(ctx, fixture, cmd, "below-base", bus_V,
                 external_torque, flux_brake_R,
                 {25.0f, 24.0f, 23.5f, 23.0f, 22.0f, 20.0f});
}

// Flux braking during field weakening.
//
// At 14V the MJ5208 base speed is ~48 rev/s.  Running at 80 rev/s
// (limited by FW to ~64) with an assisting external torque creates
// a regenerative operating point deep in the field weakening region.
// Flux braking must compose with the existing negative FW d-axis
// current and still reduce regeneration.
BOOST_AUTO_TEST_CASE(SimFieldWeakeningFluxBraking) {
  const float bus_V = 14.0f;
  const float target_speed = 80.0f;
  const float external_torque = 0.15f;
  const float flux_brake_R = 0.1f;

  ctx.flux_brake_min_voltage_ = 100.0f;  // disabled during accel
  auto cmd = SetupFieldWeakeningAtSpeed(ctx, target_speed,
                                        bus_V, 0.5f);
  ctx.config_.max_voltage = 30.0f;

  FluxBrakeSweep(ctx, fixture, cmd, "FW", bus_V,
                 external_torque, flux_brake_R,
                 {15.0f, 14.0f, 13.5f, 13.0f, 12.0f, 11.0f, 10.0f});
}

// After operating in field weakening and then bringing the motor back
// to rest, the reported fw.id_A must settle to exactly 0.
//
// The controller uses an ExponentialFilter on id_raw.  Feeding 0 into
// the filter results in geometric decay — filtered *= (1 - alpha) —
// which asymptotically approaches but never exactly reaches 0.  The
// downstream predicate at `result != 0.0f` then continues to report
// kLimitFieldWeakening indefinitely on a stopped motor.
BOOST_AUTO_TEST_CASE(SimFieldWeakeningIdADecaysToZero) {
  auto cmd = SetupFieldWeakeningAtSpeed(ctx, 100.0f, 14.0f, 1.0f);

  // Confirm that we are actually in the field weakening region with
  // a meaningfully non-zero id_A before decel.
  BOOST_CHECK_MESSAGE(
      ctx.status_.fw.id_A < -0.5f,
      "Did not enter field weakening before decel, id_A="
          << ctx.status_.fw.id_A);

  // Decelerate to zero and hold.
  cmd.velocity = 0.0f;
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 2.0f);

  // Motor should have stopped well below base speed.
  BOOST_CHECK_MESSAGE(
      std::abs(ctx.motor_sim_.velocity_rev_s()) < 2.0f,
      "Motor did not stop, vel="
          << ctx.motor_sim_.velocity_rev_s());

  // Give the exponential filter plenty of time constants to relax.
  // With bandwidth_hz=10 the time constant is ~1/(2*pi*10/(2*pi)) =
  // 100 ms, so 5 s is ~50 tau.
  ctx.RunSimulation(&cmd, 5.0f);

  std::cout << "FW id_A decay: final id_A=" << ctx.status_.fw.id_A
            << " fault=" << static_cast<int>(ctx.status_.fault)
            << " mode=" << static_cast<int>(ctx.status_.mode)
            << " vel=" << ctx.motor_sim_.velocity_rev_s()
            << std::endl;

  // The reported id_A should be exactly zero on a stopped motor below
  // base speed.  If this fails with a tiny non-zero value (e.g.
  // ~1e-30), the exponential filter is asymptotically decaying without
  // ever snapping to zero — which also keeps kLimitFieldWeakening set
  // in the downstream fault reporting.
  BOOST_CHECK_MESSAGE(
      ctx.status_.fw.id_A == 0.0f,
      "fw.id_A did not reach exactly zero, got "
          << ctx.status_.fw.id_A);

  // And the fault code should not still be kLimitFieldWeakening.
  BOOST_CHECK_MESSAGE(
      ctx.status_.fault != errc::kLimitFieldWeakening,
      "Still reporting kLimitFieldWeakening after stop");
}

// Regeneration over-voltage with dynamic bus model.
//
// Spins the motor to 60 rev/s with an assisting external torque,
// then commands an abrupt stop.  The regenerative braking current
// charges the bus capacitor through the source resistance, raising
// V_bus above config_.max_voltage.  Flux braking is enabled with
// default settings, but the 1000 Hz filter delay on filt_1ms_bus_V
// means it cannot react fast enough to prevent the transient
// over-voltage.
BOOST_AUTO_TEST_CASE(SimRegenOverVoltage) {
  // Dynamic bus model: 24V source, 1.5 ohm inline, 100 uF cap.
  ctx.SetBusModel(24.0f, 1.5f, 100e-6f);
  ctx.config_.max_voltage = 37.0f;

  // Enable flux braking at realistic threshold (37 - 3 = 34V).
  ctx.flux_brake_min_voltage_ =
      ctx.config_.max_voltage - ctx.config_.flux_brake_margin_voltage;

  ctx.config_.bemf_feedforward = 1.0f;

  // Phase 1: Spin up to 60 rev/s with assisting external torque.
  // The external torque sustains regenerative current during braking.
  auto cmd = MakePositionCommand(kNaN, 60.0f, 5.0f);
  cmd.accel_limit = 100.0f;
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 1.5f);
  ctx.external_torque_ = 0.2f;  // Assist rotation (regen source)
  ctx.RunSimulation(&cmd, 0.5f);

  BOOST_CHECK_MESSAGE(
      ctx.motor_sim_.velocity_rev_s() > 50.0f,
      "Failed to reach target speed: vel="
      << ctx.motor_sim_.velocity_rev_s());
  BOOST_CHECK_MESSAGE(
      ctx.status_.mode != kFault,
      "Faulted during spin-up: fault="
      << static_cast<int>(ctx.status_.fault));

  // Phase 2: Command abrupt stop with high braking torque.
  // External torque remains, sustaining regen power.
  cmd.velocity = 0.0f;
  cmd.max_torque_Nm = 5.0f;
  cmd.accel_limit = kNaN;

  // Step until over-voltage fault or timeout (1 second).
  bool faulted = false;
  float max_bus_V = 0.0f;
  const int timeout_steps = 30000;
  for (int i = 0; i < timeout_steps; i++) {
    ctx.StepSimulation(&cmd);
    max_bus_V = std::max(max_bus_V, ctx.status_.bus_V);
    if (ctx.status_.mode == kFault) {
      faulted = true;
      break;
    }
  }

  std::cout << "RegenOV: max_bus_V=" << max_bus_V
            << " threshold=" << ctx.config_.max_voltage
            << " final_vel=" << ctx.motor_sim_.velocity_rev_s()
            << std::endl;

  BOOST_CHECK_MESSAGE(faulted,
      "Over-voltage fault was not triggered. "
      "max_bus_V=" << max_bus_V << "V, "
      "threshold=" << ctx.config_.max_voltage << "V");
  BOOST_CHECK_MESSAGE(
      ctx.status_.fault == errc::kOverVoltage,
      "Expected kOverVoltage fault, got fault="
      << static_cast<int>(ctx.status_.fault));
  BOOST_CHECK_MESSAGE(
      max_bus_V > ctx.config_.max_voltage,
      "Bus voltage should have exceeded threshold: "
      "max_bus_V=" << max_bus_V
      << " threshold=" << ctx.config_.max_voltage);
}

BOOST_AUTO_TEST_SUITE_END()
