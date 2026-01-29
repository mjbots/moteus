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
  // approximately 4.1 rev/s per volt of bus voltage. This matches
  // real hardware measurements (71 Hz at 18V).
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
    // (back-EMF limited behavior). Expected ~4.1 rev/s per volt.
    const float velocity_per_volt = mean_velocity / test.bus_voltage;
    BOOST_CHECK_MESSAGE(
        velocity_per_volt >= 3.8f && velocity_per_volt <= 4.5f,
        "Bus " << test.bus_voltage << "V: velocity/volt ratio out of range, "
               << "got " << velocity_per_volt << " expected [3.8, 4.5]");

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

BOOST_AUTO_TEST_SUITE_END()
