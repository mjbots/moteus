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

// Run a sustained regen scenario: spin the motor to a target
// velocity, apply assisting external torque, then let the system
// settle.  The external torque creates continuous regenerative
// power that the controller must handle.
struct RegenResult {
  float max_bus_V = 0.0f;
  bool faulted = false;
  errc fault = errc::kSuccess;
};

static RegenResult RunSustainedRegenScenario(SimulationContext& ctx) {
  ctx.config_.bemf_feedforward = 1.0f;

  // Spin up without external torque.
  auto cmd = MakePositionCommand(kNaN, 60.0f, 5.0f);
  cmd.accel_limit = 100.0f;
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 1.5f);

  if (ctx.status_.mode == kFault) {
    return {ctx.status_.bus_V, true, ctx.status_.fault};
  }

  // Apply external torque and run for 2 seconds while monitoring
  // for over-voltage.
  ctx.external_torque_ = 0.4f;

  RegenResult result;
  const int steps = 60000;  // 2 seconds
  for (int i = 0; i < steps; i++) {
    ctx.StepSimulation(&cmd);
    result.max_bus_V = std::max(result.max_bus_V, ctx.status_.bus_V);
    if (ctx.status_.mode == kFault) {
      result.faulted = true;
      result.fault = ctx.status_.fault;
      break;
    }
  }
  return result;
}

BOOST_FIXTURE_TEST_SUITE(SimulationRegenTests, SimulationFixture)

// Regeneration over-voltage with dynamic bus model.
//
// Without max_regen_power_W, sustained regenerative power through a
// high-impedance bus raises V_bus above config_.max_voltage.  With
// max_regen_power_W set, pre-emptive d-axis injection dissipates
// excess regen energy and keeps the voltage below the fault
// threshold.
//
// Both runs use the identical scenario (same bus model, speed,
// torque); only max_regen_power_W differs.
BOOST_AUTO_TEST_CASE(SimRegenOverVoltage) {
  // --- Without regen power limit: should fault ---
  {
    ctx.SetBusModel(24.0f, 3.0f, 100e-6f);
    ctx.config_.max_voltage = 37.0f;
    ctx.flux_brake_min_voltage_ =
        ctx.config_.max_voltage - ctx.config_.flux_brake_margin_voltage;

    const auto result = RunSustainedRegenScenario(ctx);

    std::cout << "RegenOV(no limit): max_bus_V=" << result.max_bus_V
              << " threshold=" << ctx.config_.max_voltage
              << std::endl;

    BOOST_CHECK_MESSAGE(result.faulted,
        "Over-voltage fault was not triggered. "
        "max_bus_V=" << result.max_bus_V << "V");
    BOOST_CHECK_MESSAGE(
        result.fault == errc::kOverVoltage,
        "Expected kOverVoltage fault, got fault="
        << static_cast<int>(result.fault));
  }

  // --- With regen power limit: should NOT fault ---
  {
    ctx.motor_sim_.state() = SpmsmMotorSimulator::State{};
    ctx.Reset();

    ctx.SetBusModel(24.0f, 3.0f, 100e-6f);
    ctx.config_.max_voltage = 37.0f;
    ctx.flux_brake_min_voltage_ =
        ctx.config_.max_voltage - ctx.config_.flux_brake_margin_voltage;
    ctx.config_.max_regen_power_W = 10.0f;

    const auto result = RunSustainedRegenScenario(ctx);

    std::cout << "RegenOV(limit=10W): max_bus_V=" << result.max_bus_V
              << " threshold=" << ctx.config_.max_voltage
              << std::endl;

    BOOST_CHECK_MESSAGE(!result.faulted,
        "Should not fault with max_regen_power_W=10, "
        "max_bus_V=" << result.max_bus_V << "V, "
        "fault=" << static_cast<int>(result.fault));
    BOOST_CHECK_MESSAGE(
        result.max_bus_V < ctx.config_.max_voltage,
        "Peak voltage " << result.max_bus_V
        << "V should be below " << ctx.config_.max_voltage << "V");
  }
}

// Steady-state regen power limiting.
//
// Runs the motor at constant velocity with an external torque that
// assists rotation, creating a sustained regenerative operating
// point where the regen power significantly exceeds the limit.
// Verifies that the reported electrical power stays near the limit.
//
// The reported power_W is slightly less negative than the limit
// because q-axis copper losses (1.5 * R * i_q²) offset some of
// the mechanical regen power before it reaches the bus.
BOOST_AUTO_TEST_CASE(SimRegenPowerLimitSteadyState) {
  struct TestCase {
    float velocity;
    float external_torque;
    float max_regen_W;
  };

  // Each case must have regen power well above the limit so
  // d-axis injection is active.  Approximate mechanical regen:
  //   P_mech ≈ external_torque * velocity * 2π
  //
  // vel=40 torque=0.15: P≈37.7W, limit=20 → excess 17.7W
  // vel=60 torque=0.15: P≈56.5W, limit=20 → excess 36.5W
  // vel=60 torque=0.20: P≈75.4W, limit=50 → excess 25.4W
  const TestCase cases[] = {
    {40.0f, 0.15f, 20.0f},
    {60.0f, 0.15f, 20.0f},
    {60.0f, 0.20f, 50.0f},
  };

  for (const auto& tc : cases) {
    ctx.motor_sim_.state() = SpmsmMotorSimulator::State{};
    ctx.Reset();
    ctx.config_.bemf_feedforward = 1.0f;
    ctx.config_.max_voltage = 50.0f;
    ctx.flux_brake_min_voltage_ = 100.0f;  // disable voltage-feedback FB
    ctx.config_.max_regen_power_W = tc.max_regen_W;

    // Spin up to target velocity.
    auto cmd = MakePositionCommand(kNaN, tc.velocity, 5.0f);
    cmd.accel_limit = 100.0f;
    ctx.Command(&cmd);
    ctx.RunSimulation(&cmd, 1.5f);

    // Apply assisting external torque and settle.
    ctx.external_torque_ = tc.external_torque;
    ctx.RunSimulation(&cmd, 1.0f);

    BOOST_REQUIRE_MESSAGE(
        ctx.status_.mode != kFault,
        "vel=" << tc.velocity << " torque=" << tc.external_torque
        << " fault=" << static_cast<int>(ctx.status_.fault));

    // Measure steady-state power and d-axis current over 0.5s.
    const auto power_stats = ctx.SampleValue(&cmd, 15000, [&] {
      return ctx.status_.power_W;
    });

    const auto d_stats = ctx.SampleValue(&cmd, 15000, [&] {
      return ctx.control_.i_d_A;
    });

    std::cout << "RegenSS: vel=" << tc.velocity
              << " torque=" << tc.external_torque
              << " limit=" << tc.max_regen_W
              << " power=" << power_stats.mean
              << "±" << power_stats.std_dev
              << " d_A=" << d_stats.mean
              << std::endl;

    // D-axis injection must be active.
    BOOST_CHECK_MESSAGE(d_stats.mean < -1.0f,
        "vel=" << tc.velocity << " limit=" << tc.max_regen_W
        << ": d_A should be negative, got " << d_stats.mean);

    // Bus power should be negative (regenerating).
    BOOST_CHECK_MESSAGE(
        power_stats.mean < 0.0f,
        "vel=" << tc.velocity << " limit=" << tc.max_regen_W
        << ": power should be negative (regen), got "
        << power_stats.mean << "W");

    // Bus regen power magnitude should not exceed the limit.
    BOOST_CHECK_MESSAGE(
        power_stats.mean > -(tc.max_regen_W * 1.2f),
        "vel=" << tc.velocity << " limit=" << tc.max_regen_W
        << ": regen power " << power_stats.mean
        << "W exceeds limit by >20%");

    // Bus regen power should be at least 75% of the limit
    // (q-axis copper losses offset a small fraction).
    BOOST_CHECK_MESSAGE(
        power_stats.mean < -(tc.max_regen_W * 0.75f),
        "vel=" << tc.velocity << " limit=" << tc.max_regen_W
        << ": regen power " << power_stats.mean
        << "W is less than 75% of limit");
  }
}

// Regen power limiting when d-axis current saturates.
//
// When the excess regen power exceeds what max_current_A can
// dissipate (1.5 * I² * R), the d-axis current clips at
// max_current_A and the remaining excess flows to the bus,
// causing regen power to exceed the configured limit.
BOOST_AUTO_TEST_CASE(SimRegenPowerLimitSaturated) {
  const float ext_torque = 0.5f;
  const float velocity = 60.0f;

  ctx.config_.bemf_feedforward = 1.0f;
  ctx.config_.max_voltage = 50.0f;
  ctx.flux_brake_min_voltage_ = 100.0f;  // disable voltage-feedback FB
  ctx.config_.max_regen_power_W = 20.0f;

  // P_mech ≈ 0.5 * 60 * 2π ≈ 188.5W
  // Max d-axis dissipation: 1.5 * 40² * 0.047 = 112.8W
  // Expected net regen: P_mech - P_d_max ≈ 75.7W (before q copper
  // losses, which reduce this further).
  auto cmd = MakePositionCommand(kNaN, velocity, 5.0f);
  cmd.accel_limit = 100.0f;
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 1.5f);

  ctx.external_torque_ = ext_torque;
  ctx.RunSimulation(&cmd, 1.0f);

  BOOST_REQUIRE(ctx.status_.mode != kFault);

  const auto power_stats = ctx.SampleValue(&cmd, 15000, [&] {
    return ctx.status_.power_W;
  });

  const auto d_stats = ctx.SampleValue(&cmd, 15000, [&] {
    return ctx.control_.i_d_A;
  });

  // At this operating point the d-axis is saturated at max_current_A.
  // The net bus power (-31.6W) is determined by the mechanical regen
  // power minus d-axis and q-axis copper losses, including
  // cross-coupling voltage effects between the d and q axes.
  const float expected_power = -31.6f;

  std::cout << "RegenSaturated: power=" << power_stats.mean
            << "±" << power_stats.std_dev
            << " d_A=" << d_stats.mean
            << std::endl;

  // D-axis should be at or near max_current_A.
  BOOST_CHECK_MESSAGE(
      std::abs(d_stats.mean) > ctx.config_.max_current_A * 0.9f,
      "d_A should be near max_current (" << ctx.config_.max_current_A
      << "), got " << d_stats.mean);

  // Regen power should be within 25% of the expected value.
  BOOST_CHECK_MESSAGE(
      std::abs(power_stats.mean - expected_power) <
          std::abs(expected_power) * 0.25f,
      "Saturated regen power " << power_stats.mean
      << "W deviates >25% from expected " << expected_power << "W");
}

// Regen power limiting with a non-unity rotor_to_output_ratio.
//
// With ratio=0.5 (2:1 speed reduction), an output velocity of 30 Hz
// corresponds to a rotor velocity of 60 Hz — the same rotor
// operating point as ratio=1 at vel=60.  The d-axis injection and
// reported power should be comparable between the two cases.
BOOST_AUTO_TEST_CASE(SimRegenPowerLimitWithGearRatio) {
  const float max_regen = 20.0f;
  const float ext_torque = 0.15f;

  // --- Reference case: ratio=1, output vel=60 ---
  ctx.config_.bemf_feedforward = 1.0f;
  ctx.config_.max_voltage = 50.0f;
  ctx.flux_brake_min_voltage_ = 100.0f;
  ctx.config_.max_regen_power_W = max_regen;

  auto cmd = MakePositionCommand(kNaN, 60.0f, 5.0f);
  cmd.accel_limit = 100.0f;
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 1.5f);
  ctx.external_torque_ = ext_torque;
  ctx.RunSimulation(&cmd, 1.0f);
  BOOST_REQUIRE(ctx.status_.mode != kFault);

  const auto ref_power = ctx.SampleValue(&cmd, 15000, [&] {
    return ctx.status_.power_W;
  });
  const auto ref_d = ctx.SampleValue(&cmd, 15000, [&] {
    return ctx.control_.i_d_A;
  });

  std::cout << "RegenRatio: ratio=1 vel=60"
            << " power=" << ref_power.mean
            << " d_A=" << ref_d.mean << std::endl;

  // --- Geared case: ratio=0.5, output vel=30 (rotor vel=60) ---
  ctx.motor_sim_.state() = SpmsmMotorSimulator::State{};
  ctx.Reset();
  ctx.config_.bemf_feedforward = 1.0f;
  ctx.config_.max_voltage = 50.0f;
  ctx.flux_brake_min_voltage_ = 100.0f;
  ctx.config_.max_regen_power_W = max_regen;

  ctx.motor_position_.config()->rotor_to_output_ratio = 0.5f;
  ctx.ReconfigureMotorPosition();

  // Output velocity is 30 Hz (rotor = 60 Hz).
  cmd = MakePositionCommand(kNaN, 30.0f, 5.0f);
  cmd.accel_limit = 50.0f;
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 1.5f);
  ctx.external_torque_ = ext_torque;
  ctx.RunSimulation(&cmd, 1.0f);
  BOOST_REQUIRE_MESSAGE(ctx.status_.mode != kFault,
      "Geared case faulted: " << static_cast<int>(ctx.status_.fault));

  const auto gear_power = ctx.SampleValue(&cmd, 15000, [&] {
    return ctx.status_.power_W;
  });
  const auto gear_d = ctx.SampleValue(&cmd, 15000, [&] {
    return ctx.control_.i_d_A;
  });

  std::cout << "RegenRatio: ratio=0.5 vel=30 (rotor=60)"
            << " power=" << gear_power.mean
            << " d_A=" << gear_d.mean << std::endl;

  // D-axis should be active in both cases.
  BOOST_CHECK(ref_d.mean < -1.0f);
  BOOST_CHECK(gear_d.mean < -1.0f);

  // The d-axis current should be similar since the rotor operating
  // point is the same (same rotor speed and torque).
  BOOST_CHECK_MESSAGE(
      std::abs(ref_d.mean - gear_d.mean) < 5.0f,
      "d_A mismatch: ratio=1 got " << ref_d.mean
      << ", ratio=0.5 got " << gear_d.mean);

  // Reported bus power should be similar.
  BOOST_CHECK_MESSAGE(
      std::abs(ref_power.mean - gear_power.mean) < 5.0f,
      "power mismatch: ratio=1 got " << ref_power.mean
      << "W, ratio=0.5 got " << gear_power.mean << "W");

  // Both should be near the limit.
  BOOST_CHECK(ref_power.mean > -(max_regen * 1.2f));
  BOOST_CHECK(gear_power.mean > -(max_regen * 1.2f));
  BOOST_CHECK(ref_power.mean < -(max_regen * 0.75f));
  BOOST_CHECK(gear_power.mean < -(max_regen * 0.75f));

  // Restore ratio for subsequent tests.
  ctx.motor_position_.config()->rotor_to_output_ratio = 1.0f;
  ctx.ReconfigureMotorPosition();
}

BOOST_AUTO_TEST_SUITE_END()
