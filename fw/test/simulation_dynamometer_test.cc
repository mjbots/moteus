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

// Simulation tests based on dynamometer tests from utils/dynamometer_drive.cc.

#include <boost/test/auto_unit_test.hpp>

#include <cmath>
#include <string>

#include "fmt/format.h"

#include "fw/test/simulation_fixture.h"

using namespace moteus;
using namespace moteus::test;

// ==========================================================================
// Test helper functions (mirrors dynamometer_drive.cc patterns)
// ==========================================================================

// Run position hold test at standard positions.
// Tests: 0.0, -0.2, 0.3 rev
// Tolerance: 0.05 rev
// setup_fn is called before each position subtest to reset/configure state.
template <typename SetupFn>
void RunPositionHoldTest(
    SimulationContext& ctx, const std::string& label,
    SetupFn setup_fn) {
  constexpr float kTolerance = 0.05f;

  for (const float position : {0.0f, -0.2f, 0.3f}) {
    setup_fn();

    auto cmd = MakePositionCommand(position, 0.0f, 0.3f);
    ctx.Command(&cmd);
    ctx.RunSimulation(&cmd, 1.0f);

    BOOST_CHECK_MESSAGE(
        std::abs(ctx.status_.position - position) < kTolerance,
        label << " position hold " << position << ": got " << ctx.status_.position
            << ", error " << std::abs(ctx.status_.position - position)
            << " > " << kTolerance);
  }
}

// Run velocity tracking test at standard velocities.
// Tests: 0, -1.5, 3.0, 10.0, -5.0 rev/s
// Base tolerance: 0.35 rev/s (scaled by tolerance_scale)
// setup_fn is called before each velocity subtest to reset/configure state.
template <typename SetupFn>
void RunVelocityTrackingTest(
    SimulationContext& ctx, float tolerance_scale, const std::string& label,
    SetupFn setup_fn) {
  const float tolerance = 0.35f * tolerance_scale;

  for (const float velocity : {0.0f, -1.5f, 3.0f, 10.0f, -5.0f}) {
    setup_fn();

    auto cmd = MakePositionCommand(kNaN, velocity, 0.3f);
    cmd.accel_limit = 30.0f;

    ctx.Command(&cmd);
    ctx.RunSimulation(&cmd, 1.5f);

    const auto vel_stats = ctx.SampleValue(&cmd, 1000,
        [&] { return ctx.status_.velocity; });

    BOOST_CHECK_MESSAGE(
        std::abs(vel_stats.mean - velocity) < tolerance,
        label << " velocity " << velocity << ": got " << vel_stats.mean
            << ", error " << std::abs(vel_stats.mean - velocity)
            << " > " << tolerance);
  }
}

// Run stop position test at standard positions.
// Tests: 2.0, 0.5 rev
// Base tolerance: 0.07 rev (scaled by tolerance_scale)
// setup_fn is called before each stop position subtest.
template <typename SetupFn>
void RunStopPositionTest(
    SimulationContext& ctx, float tolerance_scale, const std::string& label,
    SetupFn setup_fn) {
  const float tolerance = 0.07f * tolerance_scale;

  for (const float stop_pos : {2.0f, 0.5f}) {
    setup_fn();
    ctx.SetMotorPosition(0.0f);  // Reset to known position like dyno's "d index 0"

    auto cmd = MakePositionCommand(kNaN, 1.0f, 0.3f);
    cmd.stop_position = stop_pos;

    ctx.Command(&cmd);
    ctx.RunSimulation(&cmd, 3.0f);

    const auto pos_stats = ctx.SampleValue(&cmd, 500,
        [&] { return ctx.status_.position; });

    BOOST_CHECK_MESSAGE(
        std::abs(pos_stats.mean - stop_pos) < tolerance,
        label << " stop_position " << stop_pos << ": got " << pos_stats.mean
            << ", error " << std::abs(pos_stats.mean - stop_pos)
            << " > " << tolerance);
  }
}

// Run position limits test at standard limits.
// Tests: 0.1, 1.0, 2.0 rev
// Base tolerance: 0.07 rev (scaled by tolerance_scale)
// setup_fn is called before each limit subtest.
template <typename SetupFn>
void RunPositionLimitsTest(
    SimulationContext& ctx, float tolerance_scale, const std::string& label,
    SetupFn setup_fn) {
  const float tolerance = 0.07f * tolerance_scale;

  for (const float limit : {0.1f, 1.0f, 2.0f}) {
    setup_fn();
    ctx.position_config_.position_min = -limit;
    ctx.position_config_.position_max = 10.0f;

    auto cmd = MakePositionCommand(kNaN, -1.5f, 0.3f);  // Move toward negative limit
    cmd.stop_position = -10.0f;
    cmd.accel_limit = 30.0f;

    ctx.Command(&cmd);
    ctx.RunSimulation(&cmd, 3.0f);

    const auto pos_stats = ctx.SampleValue(&cmd, 500,
        [&] { return ctx.status_.position; });

    BOOST_CHECK_MESSAGE(
        pos_stats.mean > (-limit - tolerance),
        label << " position_min " << -limit << ": got " << pos_stats.mean
            << ", should not go past limit by more than " << tolerance);
  }
}

// Run the standard position/velocity test suite.
// Mirrors dynamometer_drive.cc RunBasicPositionVelocityTest.
// Includes: velocity tracking, stop position, and position limits.
// setup_fn is called before each subtest to reset/configure state.
template <typename SetupFn>
void RunBasicPositionVelocityTest(
    SimulationContext& ctx, float tolerance_scale, const std::string& label,
    SetupFn setup_fn) {
  RunVelocityTrackingTest(ctx, tolerance_scale, label, setup_fn);
  RunStopPositionTest(ctx, tolerance_scale, label, setup_fn);
  RunPositionLimitsTest(ctx, tolerance_scale, label, setup_fn);
}

// Run the full basic position test suite.
// Mirrors dynamometer_drive.cc RunBasicPositionTest.
// Includes: position hold + RunBasicPositionVelocityTest.
// setup_fn is called before each subtest to reset/configure state.
template <typename SetupFn>
void RunBasicPositionTest(
    SimulationContext& ctx, float tolerance_scale, const std::string& label,
    SetupFn setup_fn) {
  RunPositionHoldTest(ctx, label, setup_fn);
  RunBasicPositionVelocityTest(ctx, tolerance_scale, label, setup_fn);
}

BOOST_FIXTURE_TEST_SUITE(SimulationDynamometerTests, SimulationFixture)

// ==========================================================================
// Position control basic tests
// ==========================================================================

// Test position hold at multiple positions
BOOST_AUTO_TEST_CASE(SimPositionHoldMultiplePositions) {
  for (const float target_position : {0.0f, -0.2f, 0.3f}) {
    auto cmd = MakePositionCommand(target_position, 0.0f, 0.2f);
    ctx.Command(&cmd);

    // Run for 1 second to settle
    ctx.RunSimulation(&cmd, 1.0f);

    // Verify position reaches target within 0.05 rev (dyno uses 0.05)
    BOOST_CHECK_MESSAGE(
        IsClose(ctx.status_.position, target_position, 0.05f),
        "Position hold at " << target_position
            << ": expected " << target_position
            << ", got " << ctx.status_.position);
  }
}

// Test velocity tracking at multiple velocities
BOOST_AUTO_TEST_CASE(SimVelocityTrackingMultiple) {
  for (const float target_velocity : {0.0f, -1.5f, 3.0f, 10.0f, -5.0f}) {
    auto cmd = MakePositionCommand(kNaN, target_velocity, 0.2f);
    cmd.accel_limit = 30.0f;  // Dyno uses a30

    ctx.Command(&cmd);

    // Run for 1.5 seconds to reach steady state (dyno waits 1.5s)
    ctx.RunSimulation(&cmd, 1.5f);

    // Verify velocity tracking within 0.35 rev/s (dyno tolerance)
    BOOST_CHECK_MESSAGE(
        IsClose(ctx.status_.velocity, target_velocity, 0.35f),
        "Velocity tracking at " << target_velocity
            << ": expected " << target_velocity
            << ", got " << ctx.status_.velocity);
  }
}

// Test stop position - move at velocity until reaching target
BOOST_AUTO_TEST_CASE(SimStopPosition) {
  auto cmd = MakePositionCommand(0.0f, 0.0f, 0.2f);
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 0.5f);

  for (const float stop_pos : {2.0f, 0.5f}) {
    // Command velocity with stop position
    const float kFixedVelocity = 1.0f;
    cmd.position = kNaN;  // No position target
    cmd.velocity = kFixedVelocity;
    cmd.stop_position = stop_pos;
    ctx.Command(&cmd);

    // After 0.5s, should be moving at commanded velocity
    ctx.RunSimulation(&cmd, 0.5f);
    BOOST_CHECK_MESSAGE(
        std::abs(std::abs(ctx.status_.velocity) - kFixedVelocity) < 0.38f,
        "Stop position " << stop_pos << ": velocity during move "
            << ctx.status_.velocity << " != " << kFixedVelocity);

    // After another 2.5s, should have reached stop position
    ctx.RunSimulation(&cmd, 2.5f);
    BOOST_CHECK_MESSAGE(
        IsClose(ctx.status_.position, stop_pos, 0.07f),
        "Stop position " << stop_pos << ": final position "
            << ctx.status_.position << " != " << stop_pos);

    // Verify stopped (velocity near 0)
    BOOST_CHECK_MESSAGE(
        std::abs(ctx.status_.velocity) < 0.1f,
        "Stop position " << stop_pos << ": velocity after stop "
            << ctx.status_.velocity << " != 0");
  }
}

// Test that output torque is limited to max_torque_Nm
// Verifies that when position error is large, torque is limited to max_torque
BOOST_AUTO_TEST_CASE(SimMaxTorqueLimiting) {
  for (const float max_torque : {0.15f, 0.3f}) {
    // Test positive direction
    {
      ctx.Reset();

      // Configure fixture to be rigid (high torque capacity)
      fixture.ConfigureRigidHold(1.0f);  // Higher than DUT's max_torque
      fixture.HoldPosition(0.0f);

      auto cmd = MakePositionCommand(5.0f, 0.0f, max_torque);  // Large position error
      ctx.Command(&cmd);
      ctx.RunWithFixture(&cmd, &fixture, 1.0f);

      // Calculate torque from q_A current
      const float measured_torque = ctx.status_.q_A * ctx.torque_constant_;

      BOOST_CHECK_MESSAGE(
          IsClose(measured_torque, max_torque, 0.15f),
          "Max torque (positive) " << max_torque << ": measured "
              << measured_torque << " != " << max_torque);
    }

    // Test negative direction
    {
      ctx.Reset();

      // Configure fixture to be rigid (high torque capacity)
      fixture.ConfigureRigidHold(1.0f);  // Higher than DUT's max_torque
      fixture.HoldPosition(0.0f);

      auto cmd = MakePositionCommand(-5.0f, 0.0f, max_torque);  // Large negative position error
      ctx.Command(&cmd);
      ctx.RunWithFixture(&cmd, &fixture, 1.0f);

      // Calculate torque from q_A current
      const float measured_torque = ctx.status_.q_A * ctx.torque_constant_;

      BOOST_CHECK_MESSAGE(
          IsClose(measured_torque, -max_torque, 0.15f),
          "Max torque (negative) " << max_torque << ": measured "
              << measured_torque << " != " << -max_torque);
    }
  }
}

// Test position limits enforcement
BOOST_AUTO_TEST_CASE(SimPositionLimits) {
  for (const float position_limit : {0.1f, 1.0f, 2.0f}) {
    // Test negative limit (position_min)
    {
      ctx.Reset();

      // Configure position limit
      ctx.position_config_.position_min = -position_limit;
      ctx.position_config_.position_max = 10.0f;

      auto cmd = MakePositionCommand(kNaN, 1.5f, 0.65f);  // Move in positive direction first
      cmd.stop_position = 0.0f;
      ctx.Command(&cmd);
      ctx.RunSimulation(&cmd, 3.0f);

      // Now command to move past the negative limit
      cmd.velocity = -1.5f;
      cmd.stop_position = -10.0f;
      ctx.Command(&cmd);
      ctx.RunSimulation(&cmd, 3.0f);

      // Should have stopped at the position limit
      BOOST_CHECK_MESSAGE(
          IsClose(ctx.status_.position, -position_limit, 0.07f),
          "Position min limit " << position_limit << ": position "
              << ctx.status_.position << " != " << -position_limit);
    }

    // Test positive limit (position_max)
    {
      ctx.Reset();

      // Configure position limit
      ctx.position_config_.position_min = -10.0f;
      ctx.position_config_.position_max = position_limit;

      auto cmd = MakePositionCommand(kNaN, 1.5f, 0.65f);
      cmd.stop_position = 10.0f;
      ctx.Command(&cmd);
      ctx.RunSimulation(&cmd, 3.0f);

      // Should have stopped at the position limit
      BOOST_CHECK_MESSAGE(
          IsClose(ctx.status_.position, position_limit, 0.07f),
          "Position max limit " << position_limit << ": position "
              << ctx.status_.position << " != " << position_limit);
    }

    // Reset position limits for next iteration
    ctx.position_config_.position_min = kNaN;
    ctx.position_config_.position_max = kNaN;
  }
}

// ==========================================================================
// Low-speed position control tests
// ==========================================================================

// Test low-speed velocity tracking at various speeds and starting
// positions.
//
// Verifies per-sample velocity consistency (smoothness).
BOOST_AUTO_TEST_CASE(SimPositionLowspeed) {
  // Use high kp and kd for low-speed tracking (like dyno)
  ctx.pid_position_config.kp = 10.0f;
  ctx.pid_position_config.kd = 0.1f;

  for (const float start_pos : {0.0f, 16.0f, 32700.0f}) {
    for (const float speed : {0.5f, -0.5f, 0.1f, -0.1f, 0.01f, -0.01f}) {
      ctx.Reset();
      ctx.SetMotorPosition(start_pos);  // Sets motor, encoder, and trajectory state

      BldcServoCommandData cmd;
      cmd.mode = kPosition;
      cmd.position = kNaN;
      cmd.velocity = speed;
      cmd.max_torque_Nm = 0.65f;
      cmd.timeout_s = kNaN;

      ctx.Command(&cmd);

      // Short settling period to let trajectory initialize
      // (needed for very slow speeds where initialization transients matter)
      ctx.RunSimulation(&cmd, 0.1f);

      // Track position at each sample for per-sample velocity consistency
      constexpr int kIterationCount = 50;
      constexpr float kDelayS = 0.2f;
      constexpr float kTotalTime = kIterationCount * kDelayS;

      std::vector<float> positions;
      positions.reserve(kIterationCount + 1);
      positions.push_back(ctx.status_.position);

      for (int i = 0; i < kIterationCount; i++) {
        ctx.RunSimulation(&cmd, kDelayS);
        positions.push_back(ctx.status_.position);
      }

      const float actual_movement = positions.back() - positions.front();
      const float expected_movement = kTotalTime * speed;

      // Tolerance depends on speed (tighter for faster speeds)
      const float tolerance = (std::abs(speed) > 0.1f) ? 0.07f : 0.021f;

      BOOST_CHECK_MESSAGE(
          IsClose(actual_movement, expected_movement, tolerance),
          "Low-speed " << speed << " from " << start_pos
              << ": movement " << actual_movement
              << " != " << expected_movement
              << " (tolerance " << tolerance << ")");

      // Per-sample velocity consistency check (smoothness)
      // Verify estimated velocity at each sample is within 0.205 of commanded
      for (size_t i = 1; i < positions.size(); i++) {
        const float estimated_velocity =
            (positions[i] - positions[i - 1]) / kDelayS;
        BOOST_CHECK_MESSAGE(
            std::abs(estimated_velocity - speed) < 0.205f,
            "Low-speed " << speed << " from " << start_pos
                << ": per-sample velocity at " << i << " = " << estimated_velocity
                << " != " << speed << " (tolerance 0.205)");
      }
    }
  }
}

// ==========================================================================
// Position wraparound test
// ==========================================================================

// Test that velocity tracking works correctly through position
// wraparound.
//
// Sets position near 32760, then commands positive velocity and
// verifies the velocity remains stable as position wraps to negative.
BOOST_AUTO_TEST_CASE(SimPositionWraparound) {
  ctx.pid_position_config.kd = 0.05f;

  // First establish position mode at zero
  auto cmd = MakePositionCommand(0.0f, 0.0f, 0.1f);
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 0.2f);

  // Set position to near wraparound point (like dyno's "d index 32760")
  ctx.motor_position_.ISR_SetOutputPositionNearest(32760.0f);
  ctx.motor_position_.ISR_Update();
  ctx.status_.position = ctx.position_.position;

  const float start_position = ctx.status_.position;
  BOOST_CHECK_MESSAGE(
      IsClose(start_position, 32760.0f, 1.0f),
      "Start position should be ~32760, got " << start_position);

  // Now command velocity to move toward wraparound
  cmd.position = kNaN;  // Don't command specific position
  cmd.velocity = 4.0f;  // Move at 4 rev/s positive (like dyno)

  ctx.Command(&cmd);

  // Let it settle for 0.5 seconds first
  ctx.RunSimulation(&cmd, 0.5f);

  // Run for 4 seconds (at 4 rev/s = ~16 revolutions, crossing from 32760 to ~-32776)
  // This should wrap around at 32768
  for (int tenth = 0; tenth < 40; tenth++) {
    ctx.RunSimulation(&cmd, 0.1f);

    // Verify velocity stays close to commanded (dyno tolerance: 0.3)
    BOOST_CHECK_MESSAGE(
        IsClose(ctx.status_.velocity, 4.0f, 0.3f),
        "Velocity at t=" << (tenth * 0.1f)
            << "s: " << ctx.status_.velocity << " != 4.0");
  }

  // After ~4.5 seconds at 4 rev/s from 32760, we should have moved
  // ~18 revs crossing 32768 and wrapping to negative. The final
  // position should be < -30000 (as verified in dyno test).
  const float final_position = ctx.status_.position;
  BOOST_CHECK_MESSAGE(
      final_position < -30000.0f,
      "Position should have wrapped to negative (< -30000), got "
          << final_position);
}

// ==========================================================================
// Velocity and acceleration limits test
// ==========================================================================

// Test that velocity and acceleration limits are enforced during
// trajectory following.
BOOST_AUTO_TEST_CASE(SimVelocityAccelLimits) {
  // Configure PID as in dyno test
  ctx.config_.pid_position.kp = 4.0f;
  ctx.config_.pid_position.kd = 0.15f;

  // Use the same trajectory as dyno: pos=3.0, end_vel=-0.5, v0.7, a0.2
  auto cmd = MakePositionCommand(3.0f, -0.5f, 0.5f);
  cmd.velocity_limit = 0.7f;
  cmd.accel_limit = 0.2f;

  ctx.Command(&cmd);

  // Track position and velocity history for windowed calculations
  const float step_s = 0.1f;
  const int steps_per_sample = static_cast<int>(step_s / ctx.rate_config_.period_s);
  const int total_loops = 140;  // 14 seconds at 0.1s per sample

  std::map<float, float> position_history;
  std::map<float, float> velocity_history;

  float done_time = 0.0f;
  bool velocity_exceeded = false;
  bool accel_exceeded = false;
  float max_avg_velocity = 0.0f;
  float max_accel = 0.0f;

  for (int i = 0; i < total_loops; i++) {
    const float time_s = i * step_s;

    // Run simulation for this sample period
    for (int j = 0; j < steps_per_sample; j++) {
      ctx.StepSimulation(&cmd);
    }

    const float position = ctx.status_.position;
    const float velocity = ctx.status_.velocity;

    position_history[time_s] = position;
    velocity_history[time_s] = velocity;

    // Check if trajectory is "done" (velocity near -0.5, position near target)
    const float target_pos = 3.0f - time_s * 0.5f;
    if (std::abs(velocity - (-0.5f)) < 0.2f &&
        std::abs(position - target_pos) < 0.2f &&
        done_time == 0.0f) {
      done_time = time_s;
    }

    // Skip velocity/accel checks for first 0.5 seconds
    const float kVelocityWindow = 0.5f;
    if (time_s < kVelocityWindow) {
      continue;
    }

    // Calculate average velocity over kVelocityWindow
    auto it = position_history.lower_bound(time_s - kVelocityWindow);
    if (it != position_history.end()) {
      const float old_position = it->second;
      const float average_velocity =
          (position - old_position) / kVelocityWindow;

      if (time_s > 12.0f) {
        // After t=12s, should be cruising at -0.5 rev/s
        BOOST_CHECK_MESSAGE(
            std::abs(average_velocity - (-0.5f)) < 0.35f,
            "t=" << time_s << ": Final cruise velocity " << average_velocity
                << " != -0.5 (tolerance 0.35)");
      } else {
        // During motion, velocity should not exceed 0.60 rev/s
        if (std::abs(average_velocity) > 0.60f) {
          velocity_exceeded = true;
        }
        max_avg_velocity = std::max(max_avg_velocity, std::abs(average_velocity));
      }
    }

    // Calculate acceleration over 1.0s window
    const float kAccelWindow = 1.0f;
    if (time_s > kAccelWindow) {
      auto vel_it = velocity_history.lower_bound(time_s - kAccelWindow);
      if (vel_it != velocity_history.end()) {
        const float old_velocity = vel_it->second;
        const float measured_accel =
            (velocity - old_velocity) / kAccelWindow;

        if (std::abs(measured_accel) > 0.60f) {
          accel_exceeded = true;
        }
        max_accel = std::max(max_accel, std::abs(measured_accel));
      }
    }
  }

  // Verify velocity limit was not exceeded
  BOOST_CHECK_MESSAGE(
      !velocity_exceeded,
      "Velocity exceeded 0.60 limit during motion, max_avg=" << max_avg_velocity);

  // Verify acceleration limit was not exceeded
  BOOST_CHECK_MESSAGE(
      !accel_exceeded,
      "Acceleration exceeded 0.60 limit during motion, max=" << max_accel);

  // Verify completion time is approximately 4.8s ±1.0s
  BOOST_CHECK_MESSAGE(
      std::abs(done_time - 4.8f) < 1.0f,
      "Completion time " << done_time << " != 4.8 (tolerance 1.0)");

  std::cout << "Velocity/accel limits: completed in " << done_time
            << "s, max_vel=" << max_avg_velocity
            << ", max_accel=" << max_accel << "\n";
}

// ==========================================================================
// Reversed output sign test
// ==========================================================================

// Test position control with reversed direction (output.sign = -1).
//
// Verifies sign inversion works correctly with proper position state
// reset.  Uses RunBasicPositionTest helper (mirrors dyno's
// ValidatePositionReverse).
BOOST_AUTO_TEST_CASE(SimPositionReverse) {
  // Dyno uses tolerance_scale = 1.0 for reverse test
  constexpr float kToleranceScale = 1.0f;

  for (const float bemf : {0.0f, 1.0f}) {
    auto setup = [&] {
      ctx.Reset();
      SetupReversedOutput(ctx, bemf);
    };

    RunBasicPositionTest(ctx, kToleranceScale,
                         fmt::format("Reverse bemf={}", bemf), setup);
  }
}

// ==========================================================================
// Voltage mode control test
// ==========================================================================

// Test position control with voltage_mode_control=true.
//
// Uses RunBasicPositionTest helper (mirrors dyno's
// ValidateVoltageModeControl).
BOOST_AUTO_TEST_CASE(SimVoltageModeControl) {
  // Dyno uses tolerance_scale = 1.0 for voltage mode control
  constexpr float kToleranceScale = 1.0f;

  auto setup = [&] {
    ctx.Reset();
    ConfigureVoltageModeControl(ctx);
  };

  RunBasicPositionTest(ctx, kToleranceScale, "Voltage mode", setup);
}

// Test BEMF feedforward configuration.
//
// When bemf_feedforward is enabled, it compensates for back-EMF at
// velocity, resulting in smaller PID q_A command needed to maintain
// velocity.
BOOST_AUTO_TEST_CASE(SimBemfFeedforward) {
  for (const float bemf : {0.0f, 1.0f}) {
    // Test velocity tracking at multiple speeds
    for (const float velocity : {-1.5f, 3.0f, 10.0f, -5.0f}) {
      ctx.Reset();
      ctx.config_.bemf_feedforward = bemf;
      ctx.config_.pid_position.kp = 1.0f;
      ctx.config_.pid_position.ki = 0.0f;
      ctx.config_.pid_position.kd = 0.05f;

      auto cmd = MakePositionCommand(kNaN, velocity, 0.2f);
      cmd.accel_limit = 30.0f;

      ctx.Command(&cmd);

      // Let it reach steady state
      ctx.RunSimulation(&cmd, 1.5f);

      const auto q_stats = ctx.SampleValue(&cmd, 3000,
          [&] { return std::abs(ctx.status_.pid_q.command); });

      // With bemf_feedforward != 0.0, q_command should be non-zero but small
      // (within 0.4 A as per dyno test kMaxQError)
      if (bemf != 0.0f) {
        const float kMaxQError = 0.4f;
        BOOST_CHECK_MESSAGE(
            q_stats.mean > 0.0f,
            "BEMF=" << bemf << " vel=" << velocity
                << ": q_command should be non-zero, got " << q_stats.mean);
        BOOST_CHECK_MESSAGE(
            q_stats.mean < kMaxQError,
            "BEMF=" << bemf << " vel=" << velocity
                << ": q_command " << q_stats.mean
                << " exceeds max error " << kMaxQError);
      }

      // Verify velocity tracking is correct regardless of BEMF setting
      BOOST_CHECK_MESSAGE(
          IsClose(ctx.status_.velocity, velocity, 0.35f),
          "BEMF=" << bemf << " vel=" << velocity
              << ": velocity " << ctx.status_.velocity << " != " << velocity);
    }
  }
}

// Test that feedforward_Nm produces the expected motor torque.
//
// The fixture holds position rigidly at 0 while DUT applies
// feedforward torque.
//
// We measure the motor's electromagnetic torque via torque_em() (dyno uses
// torque transducer).
BOOST_AUTO_TEST_CASE(SimFeedforwardTorque) {
  // Configure the fixture to hold rigidly (like dyno's CommandFixtureRigid)
  fixture.ConfigureRigidHold(0.5f);

  for (const float feedforward : {0.15f, 0.30f, -0.15f, -0.30f}) {
    ctx.Reset();

    // Fixture holds position at 0
    fixture.HoldPosition(0.0f);

    // Command DUT: position 0 with feedforward torque (like dyno's "d pos 0 0 max f{}")
    auto cmd = MakePositionCommand(0.0f, 0.0f, 0.5f);
    cmd.feedforward_Nm = feedforward;

    ctx.Command(&cmd);

    // Run with fixture for 1.0s to reach steady state (like dyno's Sleep(1.0))
    ctx.RunWithFixture(&cmd, &fixture, 1.0f);

    // Sample electromagnetic torque (dyno reads torque transducer)
    const auto torque_stats = ctx.SampleValueWithFixture(
        &cmd, &fixture, 3000,
        [&] { return ctx.motor_sim_.torque_em(); });

    // Dyno tolerance is 0.15 Nm
    const float tolerance = 0.15f;

    BOOST_CHECK_MESSAGE(
        std::abs(torque_stats.mean - feedforward) < tolerance,
        "Feedforward " << feedforward << " Nm: measured " << torque_stats.mean
            << " Nm, error " << (torque_stats.mean - feedforward) << " Nm");

    // Also verify the position stayed close to zero
    BOOST_CHECK_MESSAGE(
        std::abs(ctx.status_.position) < 0.05f,
        "Feedforward " << feedforward << " Nm: position drifted to "
            << ctx.status_.position);
  }
}

// ==========================================================================
// Brake mode test
// ==========================================================================

// Test that brake mode generates back-EMF braking torque proportional
// to velocity.
//
// In brake mode, all phases are shorted (neutral PWM), allowing
// back-EMF to drive current through winding resistance, creating
// braking torque.
BOOST_AUTO_TEST_CASE(SimBrakeMode) {
  // Use fixture preset for strong hold
  SimulatedFixture brake_fixture(&ctx);
  brake_fixture.ConfigureStrongHold();

  // Test cases from dynamometer_drive.cc ValidateBrakeMode().
  //
  // The dyno measures fixture torque (positive to maintain positive
  // velocity), but we measure motor torque (negative, opposing
  // motion), so we negate.
  struct BrakeTest {
    float velocity;
    float expected_torque;  // Motor torque (opposes motion)
  };
  const BrakeTest brake_tests[] = {
    { 0.0f, 0.0f },
    { 2.0f, -0.1f },
    { 5.0f, -0.28f },
    { -2.0f, 0.1f },
    { -5.0f, 0.28f },
  };

  // Tolerance: 0.07 Nm (matching dyno)
  constexpr float kTolerance = 0.07f;

  for (const auto& test : brake_tests) {
    ctx.Reset();

    // Enter brake mode on DUT
    auto cmd = MakeBrakeCommand();
    ctx.Command(&cmd);

    // Use fixture to drive DUT at constant velocity
    brake_fixture.MoveAtVelocity(test.velocity);

    // Run for 0.5s to reach steady state (like dyno's Sleep(1.0))
    ctx.RunWithFixture(&cmd, &brake_fixture, 0.5f);

    // Sample electromagnetic torque from motor
    const auto torque_stats = ctx.SampleValueWithFixture(
        &cmd, &brake_fixture, 3000,
        [&] { return ctx.motor_sim_.torque_em(); });
    const float measured_torque = torque_stats.mean;

    BOOST_CHECK_MESSAGE(
        std::abs(measured_torque - test.expected_torque) < kTolerance,
        "Brake at vel=" << test.velocity
            << ": measured " << measured_torque
            << " != expected " << test.expected_torque
            << " (tol=" << kTolerance << ")");
  }
}

// ==========================================================================
// Max velocity config test
// ==========================================================================

// Test that max_velocity config limits velocity in both position mode and
// current mode.
BOOST_AUTO_TEST_CASE(SimMaxVelocity) {
  const float kQCurrent = 1.5f;  // From dyno: kQCurrent = 1.5
  const float kVelocityDerate = 2.0f;  // From dyno: allows up to velocity + 2.0

  for (const float max_vel : {1.0f, 3.0f, 7.0f, 10.0f}) {
    for (const int direction : {1, -1}) {
      for (const bool current_mode : {false, true}) {
        ctx.Reset();

        // Set the max_velocity config (matching firmware defaults)
        ctx.config_.max_velocity = max_vel;
        ctx.config_.max_velocity_derate = kVelocityDerate;

        BldcServoCommandData cmd;
        if (current_mode) {
          // Current mode: "d dq 0.0 q_current" - direct q-axis current
          cmd = MakeCurrentCommand(0.0f, kQCurrent * direction);
        } else {
          // Position mode: "d pos nan velocity 1.0"
          cmd = MakePositionCommand(kNaN, direction * 25.0f, 1.0f);
          cmd.accel_limit = 50.0f;  // Fast acceleration
        }

        ctx.Command(&cmd);

        // Run to reach steady state (dyno: 0.5s warmup + 0.5s measure)
        ctx.RunSimulation(&cmd, 0.5f);

        // Measure average velocity over 0.5 seconds (dyno methodology)
        const float start_pos = ctx.status_.position;
        ctx.RunSimulation(&cmd, 0.5f);
        const float end_pos = ctx.status_.position;
        const float average_speed = std::abs((end_pos - start_pos) / 0.5f);

        // Dyno tolerance: RelativeError(average_speed, velocity + kVelocityDerate) > 0.2
        // This means average_speed should be within 20% of (velocity + kVelocityDerate)
        const float speed_limit = max_vel + kVelocityDerate;
        const float tolerance = 0.2f;  // 20% as in dyno

        BOOST_CHECK_MESSAGE(
            average_speed < speed_limit * (1.0f + tolerance),
            "max_velocity=" << max_vel << " dir=" << direction
                << " current_mode=" << current_mode
                << ": average_speed " << average_speed
                << " exceeds " << speed_limit << " + 20%");

        // Also verify we're not stuck at 0 (motor is actually moving)
        BOOST_CHECK_MESSAGE(
            average_speed > 0.5f,
            "max_velocity=" << max_vel << " dir=" << direction
                << " current_mode=" << current_mode
                << ": average_speed " << average_speed
                << " too low, motor should be moving");
      }
    }
  }
}

// ==========================================================================
// Rezero command test
// ==========================================================================

// Test that ISR_SetOutputPositionNearest properly resets the position counter
BOOST_AUTO_TEST_CASE(SimRezero) {
  // Test values from dyno test, including boundary values
  const std::vector<float> test_values = {
      0.0f, 4.2f, -7.9f, -32764.0f, -32767.0f, 32763.0f, 32767.0f
  };

  for (const float rezero_value : test_values) {
    // First move to a different position
    auto cmd = MakePositionCommand(0.5f, 0.0f, 0.2f);  // Move away from zero
    ctx.Command(&cmd);
    ctx.RunSimulation(&cmd, 0.5f);

    // Issue rezero command
    ctx.motor_position_.ISR_SetOutputPositionNearest(rezero_value);
    ctx.motor_position_.ISR_Update();  // Update to apply the change

    // Update status position
    ctx.status_.position = ctx.position_.position;

    // Verify position is now close to rezero value (within 0.5 rev as per dyno)
    BOOST_CHECK_MESSAGE(
        std::abs(ctx.status_.position - rezero_value) < 0.5f,
        "Rezero to " << rezero_value << ": position "
            << ctx.status_.position << " not within 0.5");
  }
}

// ==========================================================================
// Fixed voltage mode test
// ==========================================================================

// Test that fixed_voltage_mode (stepper-like operation) works for basic
// position and velocity tracking.
//
// In this mode, the motor uses a synthetic theta derived from the
// commanded position rather than encoder feedback.
//
// Uses RunBasicPositionVelocityTest helper (mirrors dyno's
// RunFixedVoltageModeTest).
//
// The tolerance scale 2.3 is from the dyno test.
BOOST_AUTO_TEST_CASE(SimFixedVoltageMode) {
  constexpr float kToleranceScale = 2.3f;

  auto setup = [&] {
    ctx.Reset();
    ConfigureFixedVoltageMode(ctx);
  };

  RunBasicPositionVelocityTest(ctx, kToleranceScale, "Fixed voltage", setup);
}

// ==========================================================================
// Fixed voltage mode reverse test
// ==========================================================================

// Test fixed_voltage_mode with output.sign = -1 (reversed direction).
//
// Runs the same test suite as the forward test.
BOOST_AUTO_TEST_CASE(SimFixedVoltageModeReverse) {
  // Dyno uses tolerance_scale = 2.3 for fixed voltage mode
  constexpr float kToleranceScale = 2.3f;

  auto setup = [&] {
    ctx.Reset();
    ctx.motor_position_config_.output.sign = -1;
    ctx.ReconfigureMotorPosition();
    ConfigureFixedVoltageMode(ctx);
  };

  RunBasicPositionVelocityTest(ctx, kToleranceScale, "Fixed voltage reverse", setup);
}

// ==========================================================================
// Current mode test
// ==========================================================================

// Test direct d/q current mode ("d dq" command).
//
// Verifies:
// - d_A and q_A tracking within tolerance
// - Torque matches Kt * q_A
// - Tests with fixture at speeds 0, -0.2, +0.2 rev/s
BOOST_AUTO_TEST_CASE(SimCurrentMode) {
  const float Kt = ctx.torque_constant_;

  // Test various d/q current combinations
  struct CurrentTest {
    float d_A;
    float q_A;
    float expected_torque;  // Approximate: Kt * q_A
  };
  CurrentTest current_tests[] = {
    {0.0f, 0.0f, 0.0f},
    {2.0f, 0.0f, 0.0f},      // D current produces no torque
    {4.0f, 0.0f, 0.0f},
    {0.0f, 2.0f, 2.0f * Kt}, // Q current produces torque
    {0.0f, 4.0f, 4.0f * Kt},
    {0.0f, -2.0f, -2.0f * Kt},
  };

  // Test with fixture at different speeds (matching dyno: 0, -0.2, +0.2 rev/s)
  for (const float fixture_speed : {0.0f, -0.2f, 0.2f}) {
    SimulatedFixture current_fixture(&ctx);
    current_fixture.ConfigureStrongHold();
    current_fixture.config().max_torque_Nm = 0.5f;

    for (const auto& test : current_tests) {
      ctx.Reset();

      auto cmd = MakeCurrentCommand(test.d_A, test.q_A);
      ctx.Command(&cmd);

      // Fixture moves at specified velocity (or holds if speed=0)
      if (fixture_speed == 0.0f) {
        current_fixture.HoldPosition(0.0f);
      } else {
        current_fixture.MoveAtVelocity(fixture_speed);
      }

      // Run and let currents settle
      ctx.RunWithFixture(&cmd, &current_fixture, 0.3f);

      // Sample d_A, q_A, and torque over time
      float total_d_A = 0.0f;
      float total_q_A = 0.0f;
      float total_torque = 0.0f;
      int samples = 0;
      for (int i = 0; i < 1000; i++) {
        ctx.StepWithFixture(&cmd, &current_fixture);
        total_d_A += ctx.status_.d_A;
        total_q_A += ctx.status_.q_A;
        total_torque += ctx.motor_sim_.torque_em();
        samples++;
      }
      const float avg_d_A = total_d_A / samples;
      const float avg_q_A = total_q_A / samples;
      const float avg_torque = total_torque / samples;

      // Check mode
      BOOST_CHECK_MESSAGE(
          ctx.status_.mode == kCurrent,
          "fs=" << fixture_speed << " d=" << test.d_A << " q=" << test.q_A
              << ": expected kCurrent, got " << ctx.status_.mode);

      // Check d_A tracking (within 1.0A tolerance like dyno)
      BOOST_CHECK_MESSAGE(
          std::abs(avg_d_A - test.d_A) < 1.0f,
          "fs=" << fixture_speed << " d=" << test.d_A << " q=" << test.q_A
              << ": d_A " << avg_d_A << " != " << test.d_A);

      // Check q_A tracking
      BOOST_CHECK_MESSAGE(
          std::abs(avg_q_A - test.q_A) < 1.0f,
          "fs=" << fixture_speed << " d=" << test.d_A << " q=" << test.q_A
              << ": q_A " << avg_q_A << " != " << test.q_A);

      // Check torque (within 0.15 Nm tolerance like dyno)
      BOOST_CHECK_MESSAGE(
          std::abs(avg_torque - test.expected_torque) < 0.15f,
          "fs=" << fixture_speed << " d=" << test.d_A << " q=" << test.q_A
              << ": torque " << avg_torque << " != " << test.expected_torque);
    }
  }
}

// ==========================================================================
// PWM (Voltage FOC) mode test
// ==========================================================================

// Test VoltageFOC mode ("d pwm" command).
// - Voltage ramp: 8 voltages (0 to 0.35 V) at phase 0
// - Phase slew: 30 phases (0 to 29) at voltage 0.45 V
// - D current within V/R range, Q current < 1.5 A
BOOST_AUTO_TEST_CASE(SimPwmMode) {
  using namespace moteus::test;
  const float kMotorResistance = Mj5208Params::kR;  // 0.047 Ω
  const float kMaxQError = 1.5f;

  // Test voltage ramp - 8 voltages from dyno test
  for (const float voltage : {0.0f, 0.05f, 0.1f, 0.15f, 0.2f, 0.25f, 0.3f, 0.35f}) {
    ctx.Reset();

    // Command VoltageFOC mode with phase=0 (d-axis aligned)
    auto cmd = MakeVoltageFocCommand(0.0f, voltage);

    ctx.Command(&cmd);
    ctx.RunSimulation(&cmd, 0.1f);

    // Check mode is kVoltageFoc
    BOOST_CHECK_MESSAGE(
        ctx.status_.mode == kVoltageFoc,
        "PWM mode voltage=" << voltage << ": expected kVoltageFoc, got "
            << ctx.status_.mode);

    // Q current should be small (< 1.5 A)
    BOOST_CHECK_MESSAGE(
        std::abs(ctx.status_.q_A) < kMaxQError,
        "PWM mode voltage=" << voltage << ": |q_A| " << std::abs(ctx.status_.q_A)
            << " > " << kMaxQError);

    // D current should be within expected V/R range
    if (voltage > 0.05f) {
      const auto [min_d, max_d] = FindDCurrentRange(voltage, kMotorResistance);
      BOOST_CHECK_MESSAGE(
          ctx.status_.d_A >= min_d && ctx.status_.d_A <= max_d,
          "PWM mode voltage=" << voltage << ": d_A " << ctx.status_.d_A
              << " not in range [" << min_d << ", " << max_d << "]");
    }
  }

  // Test phase slew - 30 phases from dyno test
  ctx.Reset();

  const float initial_position = ctx.motor_sim_.position_rev();
  const float kSlewVoltage = 0.45f;  // From dyno
  std::vector<float> slew_positions;

  for (float phase = 0.0f; phase < 30.0f; phase += 1.0f) {
    auto cmd = MakeVoltageFocCommand(phase, kSlewVoltage);

    ctx.Command(&cmd);
    ctx.RunSimulation(&cmd, 0.05f);

    // Verify Q current remains small
    BOOST_CHECK_MESSAGE(
        std::abs(ctx.status_.q_A) < kMaxQError,
        "PWM phase slew phase=" << phase << ": |q_A| " << std::abs(ctx.status_.q_A)
            << " > " << kMaxQError);

    // Verify D current within range
    const auto [min_d, max_d] = FindDCurrentRange(kSlewVoltage, kMotorResistance);
    BOOST_CHECK_MESSAGE(
        ctx.status_.d_A >= min_d && ctx.status_.d_A <= max_d,
        "PWM phase slew phase=" << phase << ": d_A " << ctx.status_.d_A
            << " not in range [" << min_d << ", " << max_d << "]");

    slew_positions.push_back(ctx.motor_sim_.position_rev());
  }

  const float final_position = ctx.motor_sim_.position_rev();
  const float position_change = final_position - initial_position;

  // Motor should have moved (approximately multiple electrical revolutions / pole_pairs)
  // With 30 phases at 1 rad each, that's ~4.8 electrical revolutions = ~0.68 mechanical rev
  BOOST_CHECK_MESSAGE(
      std::abs(position_change) > 0.3f,
      "PWM phase slew: position change " << position_change
          << " too small (expected ~0.68 rev)");
}

// ==========================================================================
// Position PID test
// ==========================================================================

// Test position PID gains (kp, kd, ki).
//
// Uses SimulatedFixture with PID control to hold position while DUT
// tries to move. Measures electromagnetic torque via torque_em() to
// verify PID behavior.
//
// kp test: Fixture holds position rigidly while DUT commands
//          different positions.  Verifies torque = kp *
//          position_error.
// kd test: Fixture moves at constant velocity, DUT commands
//          velocity=0.  Verifies torque = kd * velocity.
// ki test: Fixture holds position while DUT has position error.
//          Verifies integrator builds up torque over time.
BOOST_AUTO_TEST_CASE(SimPositionPid) {
  // Fixture uses dyno-matching PID: ki=300, kd=0.6, kp=5.0, ilimit=0.45
  // These are the defaults in SimulatedFixture::Config

  // Test kp: multiple values and positions (dyno tests kp=1.0, 0.5, 0.2)
  //
  // Verify that electromagnetic torque = kp * position_error
  for (const float kp : {1.0f, 0.5f, 0.2f}) {
    for (const float position : {0.0f, -0.1f, 0.1f, -0.2f, 0.2f, -0.3f, 0.3f}) {
      ctx.Reset();
      fixture = SimulatedFixture(&ctx);  // Use dyno-matching defaults

      // NOTE: Must set pid_position_config, not config_.pid_position, as the
      // PID object uses pid_position_config directly.
      ctx.pid_position_config.kp = kp;
      ctx.pid_position_config.ki = 0.0f;
      ctx.pid_position_config.kd = 0.0f;

      // Fixture holds position at 0 (like "d pos 0 0 max_torque")
      fixture.HoldPosition(0.0f);

      auto cmd = MakePositionCommand(position, 0.0f, 0.4f);

      ctx.Command(&cmd);

      // Run with fixture for steady state (1.5s like dyno)
      ctx.RunWithFixture(&cmd, &fixture, 1.5f);

      // Fixture should have held position near 0 (dyno tolerance: 0.02 rev)
      BOOST_CHECK_MESSAGE(
          std::abs(ctx.status_.position) < 0.02f,
          "kp=" << kp << " pos=" << position
              << ": fixture position moved to " << ctx.status_.position
              << " (should be < 0.02)");

      // Electromagnetic torque should match expected = kp * position_error
      // Since position is held near 0, position_error ≈ commanded_position
      const float position_error = position - ctx.status_.position;
      const float expected_torque = kp * position_error;
      const float actual_torque = ctx.motor_sim_.torque_em();

      // Dyno tolerance is 0.15 Nm
      BOOST_CHECK_MESSAGE(
          std::abs(actual_torque - expected_torque) < 0.15f,
          "kp=" << kp << " pos=" << position
              << ": torque_em " << actual_torque
              << " != " << expected_torque << " (tolerance 0.15)");
    }
  }

  // Test kd: multiple values and speeds (dyno tests kd=0.15, 0.1, 0.05)
  //
  // Fixture moves at constant velocity, DUT commands velocity=0
  // DUT produces damping torque = kd * velocity_error
  for (const float kd : {0.15f, 0.1f, 0.05f}) {
    // Test speeds (dyno: -2.0 to 2.0)
    for (const float speed : {-2.0f, -1.0f, -0.5f, 0.0f, 0.5f, 1.0f, 2.0f}) {
      ctx.Reset();
      fixture = SimulatedFixture(&ctx);

      ctx.pid_position_config.kp = 0.0f;
      ctx.pid_position_config.ki = 0.0f;
      ctx.pid_position_config.kd = kd;

      // Fixture moves at constant velocity (like "d pos nan <speed> 0.4")
      fixture.MoveAtVelocity(speed);

      // DUT commands velocity=0 (like "d pos nan 0 0.4")
      auto cmd = MakePositionCommand(kNaN, 0.0f, 0.4f);

      ctx.Command(&cmd);

      // Run for 1s to reach steady state
      ctx.RunWithFixture(&cmd, &fixture, 1.0f);

      // DUT measures velocity and produces damping torque.
      //
      // DUT's PID outputs: torque = kd * (cmd_vel - actual_vel) = kd * (0 - vel) = -kd * vel
      // So torque_em() should be approximately -kd * velocity
      const float expected_torque = -kd * ctx.status_.velocity;
      const float actual_torque = ctx.motor_sim_.torque_em();

      // Dyno tolerance is 0.18 Nm
      BOOST_CHECK_MESSAGE(
          std::abs(actual_torque - expected_torque) < 0.18f,
          "kd=" << kd << " speed=" << speed
              << ": torque_em " << actual_torque
              << " != " << expected_torque << " (tolerance 0.18)"
              << " (actual_vel=" << ctx.status_.velocity << ")");
    }
  }

  // Test ki: multiple values and positions (dyno tests ki=1.0, 0.5, 0.2)
  //
  // Fixture holds position while DUT has position error.
  // Integrator builds up torque = (time - lag) * ki * position_error
  for (const float ki : {1.0f, 0.5f, 0.2f}) {
    // Test positions (dyno: 0.3, 0.15, 0, -0.15, -0.3)
    for (const float position : {0.3f, 0.15f, 0.0f, -0.15f, -0.3f}) {
      ctx.Reset();
      fixture = SimulatedFixture(&ctx);

      ctx.pid_position_config.kp = 0.0f;
      ctx.pid_position_config.ki = ki;
      ctx.pid_position_config.kd = 0.0f;
      ctx.pid_position_config.ilimit = 0.65f;  // Match dyno

      // Fixture holds position at 0
      fixture.HoldPosition(0.0f);

      auto cmd = MakePositionCommand(position, 0.0f, 0.65f);

      ctx.Command(&cmd);

      // Calculate delay time (like dyno test)
      // Wait until integrator builds up to target torque
      const float target_torque = 0.25f;
      const float delay_s = (position == 0.0f) ? 2.0f :
          std::min(target_torque / (std::abs(position) * ki), 5.0f);

      ctx.RunWithFixture(&cmd, &fixture, delay_s);

      // Expected torque = (delay - lag) * ki * position_error
      // position_error ≈ position since actual is held near 0
      const float kTorqueLagS = 0.3f;
      const float position_error = position - ctx.status_.position;
      const float expected_torque = (delay_s - kTorqueLagS) * ki * position_error;
      const float actual_torque = ctx.motor_sim_.torque_em();

      // Dyno tolerance is 0.17 Nm
      BOOST_CHECK_MESSAGE(
          std::abs(actual_torque - expected_torque) < 0.17f,
          "ki=" << ki << " pos=" << position
              << ": torque_em " << actual_torque
              << " != " << expected_torque << " (tolerance 0.17)");

      // Stop DUT to clear I term (like dyno does)
      cmd.mode = kStopped;
      ctx.Command(&cmd);
      ctx.RunWithFixture(&cmd, &fixture, 0.2f);
    }
  }
}

// ==========================================================================
// Stay within bounds test
// ==========================================================================

// Test kStayWithinBounds mode ("d within" command).
//
// This is a full matrix test: 3 feedforward × 2 max_torque × 9 bounds = 54 cases
//
// When inside bounds, motor applies only feedforward.
// When at bounds, motor applies resisting torque.
// NaN bounds or low max_torque allows pushing through.
BOOST_AUTO_TEST_CASE(SimStayWithin) {
  // Match dyno: kp=2.0
  ctx.pid_position_config.kp = 2.0f;
  ctx.pid_position_config.ki = 0.0f;
  ctx.pid_position_config.kd = 0.05f;

  // Fixture speed for pushing DUT
  constexpr float kSpeed = 2.0f;  // rev/s

  // Lambda to evaluate position/torque at bounds (matches dyno logic)
  auto evaluate = [](float bounds, float position, float torque,
                     float max_torque, float feedforward,
                     const char* name) -> bool {
    if (std::isnan(bounds) || max_torque < 0.15f) {
      // Position should go far (no effective bound)
      if (std::abs(position) < 1.8f) {  // Dyno tolerance: 1.8
        BOOST_TEST_MESSAGE(name << " bound unset/low torque, position didn't go far: |"
                           << position << "| < 1.8");
        return false;
      }
      // Torque check
      if (std::isnan(bounds)) {
        // Should only apply feedforward
        if (std::abs(torque - feedforward) > 0.07f) {  // Dyno tolerance: 0.07
          BOOST_TEST_MESSAGE(name << " unexpected torque: " << torque
                             << " != feedforward " << feedforward);
          return false;
        }
      } else {
        // Low max_torque pushthrough - should be at max_torque
        if (std::abs(std::abs(torque) - max_torque) > 0.08f) {  // Dyno tolerance: 0.08
          BOOST_TEST_MESSAGE(name << " pushthrough torque |" << torque
                             << "| != max_torque " << max_torque);
          return false;
        }
      }
    } else {
      // Position should be constrained to bounds
      if (std::abs(position - bounds) > 0.20f) {  // Dyno tolerance: 0.20
        BOOST_TEST_MESSAGE(name << " bounds not constrained: pos="
                           << position << " != bounds=" << bounds);
        return false;
      }
      // Should have some torque resisting
      if (std::abs(torque) < 0.08f) {  // Dyno tolerance: 0.08
        BOOST_TEST_MESSAGE(name << " insufficient torque: |" << torque << "| < 0.08");
        return false;
      }
    }
    return true;
  };

  int test_count = 0;
  int pass_count = 0;

  // Full matrix from dyno test
  for (const float feedforward : {0.0f, -0.1f, 0.1f}) {
    for (const float max_torque : {0.25f, 0.10f}) {
      for (const float bounds_low : {kNaN, -0.5f, -1.5f}) {
        for (const float bounds_high : {kNaN, 0.5f, 1.5f}) {
          test_count++;

          ctx.Reset();
          ctx.SetMotorPosition(0.0f);
          // Reset position tracking to 0 (like "d index 0" command)
          ctx.motor_position_.ISR_SetOutputPositionNearest(0.0f);
          ctx.motor_position_.ISR_Update();
          ctx.status_.position = ctx.position_.position;

          fixture.Stop();
          fixture.ResetIntegral();
          fixture.config().ki = 0.0f;  // No integral for velocity push
          // Match dyno fixture torque limit: 0.22 Nm so motor can hold at bounds
          // when max_torque >= 0.25, but fixture overpowers when max_torque < 0.15
          fixture.config().max_torque_Nm = 0.22f;

          // Set up command
          auto cmd = MakeStayWithinCommand(bounds_low, bounds_high, feedforward, max_torque);

          ctx.Command(&cmd);

          // Push negative direction
          fixture.MoveAtVelocity(-kSpeed);
          ctx.RunWithFixture(&cmd, &fixture, 1.5f);  // 3.0/kSpeed in dyno

          const float low_position = ctx.status_.position;
          const float low_torque = ctx.motor_sim_.torque_em();

          // Push positive direction
          fixture.MoveAtVelocity(kSpeed);
          ctx.RunWithFixture(&cmd, &fixture, 3.0f);  // 6.0/kSpeed in dyno

          const float high_position = ctx.status_.position;
          const float high_torque = ctx.motor_sim_.torque_em();

          // Evaluate
          const bool low_ok = evaluate(bounds_low, low_position, low_torque,
                                       max_torque, feedforward, "low");
          const bool high_ok = evaluate(bounds_high, high_position, high_torque,
                                        max_torque, feedforward, "high");

          if (low_ok && high_ok) {
            pass_count++;
          } else {
            BOOST_TEST_MESSAGE("FAIL: bounds=[" << bounds_low << ", " << bounds_high
                               << "] max_torque=" << max_torque
                               << " ff=" << feedforward
                               << " | low_pos=" << low_position
                               << " low_torque=" << low_torque
                               << " high_pos=" << high_position
                               << " high_torque=" << high_torque);
          }
        }
      }
    }
  }

  BOOST_TEST_MESSAGE("SimStayWithin: " << pass_count << "/" << test_count << " passed");
  BOOST_CHECK_MESSAGE(pass_count == 54,
                      "All 54 cases should pass (dyno requirement), got " << pass_count);
}

// ==========================================================================
// Max position slip test
// ==========================================================================

// Test max_position_slip config.
//
// When enabled, this limits how much the control position can slip
// behind actual.
//
// Methodology:
// 1. DUT moves at desired speed
// 2. Fixture slows DUT down (position error accumulates)
// 3. Fixture releases
// 4. With slip: DUT should NOT catch up (<1.15x desired)
//    Without slip: DUT SHOULD catch up (>1.25x desired)
BOOST_AUTO_TEST_CASE(SimMaxSlip) {
  for (const float max_slip : {kNaN, 0.05f}) {  // Match dyno: 0.05
    ctx.Reset();
    ctx.SetMotorPosition(0.0f);
    ctx.motor_position_.ISR_SetOutputPositionNearest(0.0f);
    ctx.motor_position_.ISR_Update();
    ctx.status_.position = ctx.position_.position;

    // Match dyno PID config
    ctx.pid_position_config.kp = 2.0f;
    ctx.pid_position_config.ki = 0.0f;
    ctx.pid_position_config.kd = 0.05f;
    ctx.config_.max_position_slip = max_slip;

    // Configure fixture to be rigid (high torque)
    fixture.ConfigureRigidHold(0.4f);  // Higher than DUT's 0.2

    constexpr float kDesiredSpeed = 0.5f;

    // Phase 1: DUT moves at desired speed, fixture follows
    auto cmd = MakePositionCommand(kNaN, kDesiredSpeed, 0.2f);

    ctx.Command(&cmd);
    fixture.MoveAtVelocity(kDesiredSpeed);  // Fixture matches DUT speed
    ctx.RunWithFixture(&cmd, &fixture, 0.5f);

    const float initial_position = ctx.status_.position;

    // Continue for 1.0s
    ctx.RunWithFixture(&cmd, &fixture, 1.0f);

    const float spinning_position = ctx.status_.position;
    const float measured_speed = spinning_position - initial_position;

    // Verify base speed achieved (dyno tolerance: 0.052)
    BOOST_CHECK_MESSAGE(
        std::abs(measured_speed - kDesiredSpeed) < 0.052f,
        "max_slip=" << max_slip << ": base speed " << measured_speed
            << " != desired " << kDesiredSpeed);

    // Phase 2: Fixture slows to half speed (DUT falls behind)
    fixture.MoveAtVelocity(0.5f * kDesiredSpeed);  // Fixture slows
    ctx.RunWithFixture(&cmd, &fixture, 2.0f);

    const float slow_position = ctx.status_.position;
    const float slow_speed = (slow_position - spinning_position) / 2.0f;

    // Verify DUT slowed down (dyno tolerance: 0.052)
    BOOST_CHECK_MESSAGE(
        std::abs(slow_speed - 0.5f * kDesiredSpeed) < 0.052f,
        "max_slip=" << max_slip << ": slow speed " << slow_speed
            << " != expected " << 0.5f * kDesiredSpeed);

    // Phase 3: Release fixture
    fixture.Stop();
    ctx.RunWithFixture(&cmd, &fixture, 1.0f);

    const float final_position = ctx.status_.position;
    const float final_speed = final_position - slow_position;

    // Verify catch-up behavior
    if (std::isfinite(max_slip)) {
      // With slip limiting, DUT should NOT catch up
      BOOST_CHECK_MESSAGE(
          final_speed < 1.15f * kDesiredSpeed,
          "max_slip=" << max_slip << ": DUT caught up! final_speed "
              << final_speed << " > " << 1.15f * kDesiredSpeed);
    } else {
      // Without slip limiting, DUT SHOULD catch up
      BOOST_CHECK_MESSAGE(
          final_speed > 1.25f * kDesiredSpeed,
          "max_slip=NaN: DUT didn't catch up! final_speed "
              << final_speed << " < " << 1.25f * kDesiredSpeed);
    }
  }

  ctx.config_.max_position_slip = kNaN;
}

// ==========================================================================
// Slip stop position test
// ==========================================================================

// Test stop_position behavior with slip interaction.
//
// Methodology:
// 1. Fixture holds DUT in place while DUT tries to move with stop_position
// 2. Release fixture, check catch-up behavior
// 3. Wait to reach stop_position
// 4. Drag DUT with fixture, check if it "sticks" (with slip) or returns (without)
BOOST_AUTO_TEST_CASE(SimSlipStopPosition) {
  for (const float slip : {kNaN, 0.04f}) {  // Match dyno: 0.04
    ctx.Reset();
    ctx.SetMotorPosition(0.0f);
    ctx.motor_position_.ISR_SetOutputPositionNearest(0.0f);
    ctx.motor_position_.ISR_Update();
    ctx.status_.position = ctx.position_.position;

    // Match dyno PID config
    ctx.pid_position_config.kp = 5.0f;
    ctx.pid_position_config.ki = 0.0f;
    ctx.pid_position_config.kd = 0.2f;
    ctx.config_.max_position_slip = slip;

    // Configure fixture to hold position rigidly
    fixture.ConfigureRigidHold(0.65f);

    constexpr float kVelocity = 0.1f;

    // Phase 1: Fixture holds DUT at position 0 while DUT tries to move
    auto cmd = MakePositionCommand(kNaN, kVelocity, 0.3f);
    cmd.stop_position = 0.5f;  // "s0.5" in dyno

    ctx.Command(&cmd);
    fixture.HoldPosition(0.0f);  // Fixture holds at 0
    ctx.RunWithFixture(&cmd, &fixture, 3.0f);

    const float before_letgo = ctx.status_.position;

    // Phase 2: Release fixture
    fixture.Stop();
    ctx.RunWithFixture(&cmd, &fixture, 1.0f);

    const float after_letgo = ctx.status_.position;
    const float catch_up_velocity = after_letgo - before_letgo;

    // Verify catch-up behavior
    if (std::isfinite(slip)) {
      // With slip, DUT should NOT catch up much
      BOOST_CHECK_MESSAGE(
          catch_up_velocity < 1.5f * kVelocity,
          "slip=" << slip << ": DUT caught up! velocity "
              << catch_up_velocity << " > " << 1.5f * kVelocity);
    } else {
      // Without slip, DUT SHOULD catch up
      BOOST_CHECK_MESSAGE(
          catch_up_velocity > 2.0f * kVelocity,
          "slip=NaN: DUT didn't catch up! velocity "
              << catch_up_velocity << " < " << 2.0f * kVelocity);
    }

    // Phase 3: Move to test position for sticking test
    // With slip limiting, we can't instantly jump to 0.5, so we either:
    // - For NaN slip: command position 0.5 directly
    // - For finite slip: temporarily disable slip to reach 0.5
    float test_position;
    if (std::isfinite(slip)) {
      // Temporarily disable slip to reach position
      ctx.config_.max_position_slip = kNaN;
      auto cmd2 = MakePositionCommand(0.5f, kNaN, 0.3f);
      ctx.Command(&cmd2);
      ctx.RunWithFixture(&cmd2, &fixture, 2.0f);
      test_position = ctx.status_.position;

      // Re-enable slip for sticking test
      ctx.config_.max_position_slip = slip;
    } else {
      // Without slip, just command position
      auto cmd2 = MakePositionCommand(0.5f, kNaN, 0.3f);
      ctx.Command(&cmd2);
      ctx.RunWithFixture(&cmd2, &fixture, 2.0f);
      test_position = ctx.status_.position;
    }

    // Verify reached test position
    BOOST_CHECK_MESSAGE(
        std::abs(test_position - 0.5f) < 0.1f,
        "slip=" << slip << ": didn't reach test position, got "
            << test_position);

    // Phase 4: Test sticking behavior in both directions
    // Create position hold command at 0.5
    auto hold_cmd = MakePositionCommand(0.5f, kNaN, 0.3f);

    for (const float target : {0.3f, -0.3f}) {
      // Reset fixture position tracking
      fixture.Stop();
      fixture.ResetIntegral();

      // Fixture drags DUT
      fixture.MoveAtVelocity(target > 0 ? 0.5f : -0.5f);
      ctx.RunWithFixture(&hold_cmd, &fixture, 1.0f);

      // Remember motor position after drag
      const float after_drag = ctx.status_.position;

      // Release fixture
      fixture.Stop();
      ctx.RunWithFixture(&hold_cmd, &fixture, 1.0f);

      // Check if DUT returned to target position or "stuck"
      const float final_pos = ctx.status_.position;
      const float moved_back = std::abs(final_pos - after_drag);
      const float dist_from_target = std::abs(final_pos - 0.5f);

      if (std::isfinite(slip)) {
        // With slip, DUT should "stick" (not return much)
        BOOST_CHECK_MESSAGE(
            moved_back < 0.3f,
            "slip=" << slip << " dir=" << target
                << ": DUT should stick, but moved back " << moved_back);
      } else {
        // Without slip, DUT should return close to target position
        BOOST_CHECK_MESSAGE(
            dist_from_target < 0.2f,
            "slip=NaN dir=" << target
                << ": DUT should return, but dist=" << dist_from_target);
      }
    }
  }

  ctx.config_.max_position_slip = kNaN;
}

// ==========================================================================
// Slip bounds test
// ==========================================================================

// Test position bounds enforcement with slip interaction.
//
// Methodology:
// 1. DUT holds at 0.0 with position bounds
// 2. Fixture drags DUT toward bound
// 3. Release fixture
// 4. With slip: DUT stays at bound
//    Without slip: DUT returns to 0
BOOST_AUTO_TEST_CASE(SimSlipBounds) {
  // Increase motor viscous damping to simulate the higher effective damping
  // of the real dynamometer setup (two coupled motors with mechanical losses).
  // This prevents the motor from overshooting the position bound on release.
  ctx.motor_sim_.params().B = 0.01f;  // ~300x higher than default 3e-5

  // Configure fixture
  fixture.ConfigureRigidHold(0.65f);

  for (const float slip : {kNaN, 0.03f}) {  // Match dyno: 0.03
    for (const float direction : {-0.6f, 0.6f}) {
      ctx.Reset();
      ctx.SetMotorPosition(0.0f);
      ctx.motor_position_.ISR_SetOutputPositionNearest(0.0f);
      ctx.motor_position_.ISR_Update();
      ctx.status_.position = ctx.position_.position;

      // Match dyno PID config with position bounds
      ctx.pid_position_config.kp = 5.0f;
      ctx.pid_position_config.ki = 0.0f;
      ctx.pid_position_config.kd = 0.2f;
      ctx.position_config_.position_min = -0.3f;
      ctx.position_config_.position_max = 0.5f;
      ctx.config_.max_position_slip = slip;

      fixture.Stop();
      fixture.ResetIntegral();

      // DUT holds at 0.0
      auto cmd = MakePositionCommand(0.0f, 0.0f, 0.3f);

      ctx.Command(&cmd);

      // Fixture drags toward bound
      const float drag_velocity = (direction > 0) ? 0.5f : -0.5f;
      fixture.MoveAtVelocity(drag_velocity);
      ctx.RunWithFixture(&cmd, &fixture, 2.0f);

      const float pos_after_drag = ctx.status_.position;

      // Release fixture and let motor return
      fixture.Stop();
      ctx.RunWithFixture(&cmd, &fixture, 3.0f);

      // Check final position
      const float bound = (direction > 0) ? 0.5f : -0.3f;
      const float expected = std::isfinite(slip) ? bound : 0.0f;
      const float actual = ctx.status_.position;
      const float error = std::abs(actual - expected);

      // Compact diagnostic output
      fmt::print("SlipBounds: slip={} dir={:.1f} pos_after_drag={:.4f} final={:.4f} expected={:.4f} error={:.4f}\n",
                 std::isfinite(slip) ? fmt::format("{:.6f}", slip) : "NaN",
                 direction, pos_after_drag, actual, expected, error);

      // Matches dyno tolerance of 0.05 rev
      BOOST_CHECK_MESSAGE(
          error < 0.05f,
          "slip=" << slip << " dir=" << direction
              << ": pos=" << actual << " expected=" << expected);
    }
  }
}

// ==========================================================================
// Power limit test
// ==========================================================================

// Test max_power_W config.
// Power limiting should reduce speed when motor is under load.
BOOST_AUTO_TEST_CASE(SimPowerLimit) {
  SimulatedFixture fixture(&ctx);

  // Configure fixture as a resistive load (kd only, no kp or ki)
  // This matches dyno: fixture_->ConfigurePid with kd=0.15
  fixture.config().kp = 0.0f;
  fixture.config().ki = 0.0f;
  fixture.config().kd = 0.15f;
  fixture.config().max_torque_Nm = 1.0f;

  // Match dyno DUT PID config
  ctx.pid_position_config.kp = 5.0f;
  ctx.pid_position_config.ki = 0.0f;
  ctx.pid_position_config.kd = 0.2f;
  ctx.config_.max_position_slip = 0.2f;

  // Test cases matching dyno structure: power_W and expected_speed_Hz
  //
  // Using dyno expected speeds - simulation produces similar results
  // within tolerance
  struct PowerTest {
    float power_W;
    float expected_speed_Hz;
  };
  PowerTest tests[] = {
    {100.0f, 3.90f},   // High power
    {20.0f, 2.5f},     // Medium power
    {10.0f, 1.7f},     // Low power
    {5.0f, 1.2f},      // Very low power
  };

  constexpr float kTolerance = 0.20f;  // 20% relative error, matching dyno

  for (const auto& test : tests) {
    ctx.Reset();
    ctx.pid_position_config.kp = 5.0f;
    ctx.pid_position_config.ki = 0.0f;
    ctx.pid_position_config.kd = 0.2f;
    ctx.config_.max_position_slip = 0.2f;
    ctx.status_.max_power_W = test.power_W;

    // Fixture holds velocity at 0 (resistive load)
    // Dyno: "d pos nan 0 {max_torque}"
    fixture.Stop();
    fixture.config().kp = 0.0f;
    fixture.config().ki = 0.0f;
    fixture.config().kd = 0.15f;
    fixture.HoldPosition(0.0f);  // Hold at 0, creating resistance

    // Command DUT to move at 4.0 rev/s with 1.0 Nm max torque (matching dyno)
    // Dyno: "d pos nan 4.0 1.0"
    auto cmd = MakePositionCommand(kNaN, 4.0f, 1.0f);

    ctx.Command(&cmd);

    // Let it accelerate (dyno: Sleep(0.5))
    ctx.RunWithFixture(&cmd, &fixture, 0.5f);

    // Measure speed over 0.5s (matching dyno measurement window)
    const float start_pos = ctx.status_.position;
    ctx.RunWithFixture(&cmd, &fixture, 0.5f);
    const float end_pos = ctx.status_.position;
    const float avg_speed = (end_pos - start_pos) / 0.5f;

    // Check 20% relative error (matching dyno)
    const float relative_error = std::abs(avg_speed - test.expected_speed_Hz) /
                                 test.expected_speed_Hz;

    BOOST_CHECK_MESSAGE(
        relative_error <= kTolerance,
        "power_W=" << test.power_W << ": speed " << avg_speed
            << " != " << test.expected_speed_Hz
            << " (error " << (relative_error * 100.0f) << "% > "
            << (kTolerance * 100.0f) << "%)");

    // Stop DUT (matching dyno sequence)
    cmd.mode = kStopped;
    ctx.Command(&cmd);
    fixture.Stop();
    ctx.RunWithFixture(&cmd, &fixture, 0.3f);
  }
}

// ==========================================================================
// DQ integrator limit test
// ==========================================================================
// Basic sanity check that the DQ integrator limit works.
// Verifies motor stops cleanly without excessive windup.

BOOST_AUTO_TEST_CASE(SimDqIlimit) {
  // The dyno test notes that proper validation requires spinning at max speed
  // for extended time, which isn't practical. We do a basic sanity check.
  // This test verifies the integrator doesn't wind up excessively
  // by checking that stopping after high-speed operation doesn't
  // result in prolonged torque output.

  // Command high velocity
  auto cmd = MakePositionCommand(kNaN, 400.0f, 0.5f);

  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 2.0f);

  const float steady_state_vel = std::abs(ctx.motor_sim_.velocity_rev_s());

  const float kScale = 0.8;
  cmd.velocity = kScale * steady_state_vel;
  ctx.Command(&cmd);
  ctx.RunSimulation(&cmd, 0.1f);

  // If the Q axis integrator had wound up, then the velocity would
  // not decrease for some time.
  const float vel_after_stop = std::abs(ctx.motor_sim_.velocity_rev_s());

  // This is a very loose check - just ensure motor isn't still accelerating
  BOOST_CHECK_MESSAGE(
      std::abs(vel_after_stop - cmd.velocity) < 1.0,
      "After stop command, velocity " << vel_after_stop
      << " should have decreased to "
      << cmd.velocity);
}

BOOST_AUTO_TEST_SUITE_END()
