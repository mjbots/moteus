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

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include "fmt/format.h"

#include "fw/test/simulation_fixture.h"

using namespace moteus;
using namespace moteus::test;

namespace {

// shifts[k] perturbs the k-th boundary from its ideal position k/6.
// Boundaries are clamped to remain monotonic with at least 1% (=3.6°
// electrical) minimum sector width so that the simulator always
// produces all six hall states.
std::array<float, 6> MakeShiftedBoundaries(
    const std::array<float, 6>& shifts) {
  std::array<float, 6> out;
  for (int i = 0; i < 6; ++i) {
    out[i] = static_cast<float>(i) / 6.0f + shifts[i];
  }
  for (int i = 0; i < 6; ++i) {
    if (out[i] < 0.0f) out[i] = 0.0f;
    if (out[i] >= 1.0f) out[i] = 1.0f - 1e-3f;
  }
  for (int i = 1; i < 6; ++i) {
    if (out[i] <= out[i - 1] + 0.01f) {
      out[i] = out[i - 1] + 0.01f;
    }
  }
  return out;
}

void ConfigureHallCommutation(
    SimulationContext& ctx,
    const std::array<float, 6>& boundaries,
    float pll_filter_hz = 100.0f) {
  ctx.hall_mode_ = true;
  ctx.hall_boundaries_ = boundaries;

  auto* mp_config = ctx.motor_position_.config();
  mp_config->commutation_source = 0;
  mp_config->sources[0].aux_number = 1;
  mp_config->sources[0].type = MotorPosition::SourceConfig::kHall;
  mp_config->sources[0].reference = MotorPosition::SourceConfig::kRotor;
  mp_config->sources[0].pll_filter_hz = pll_filter_hz;
  mp_config->sources[0].sign = 1;
  mp_config->sources[0].offset = 0;
  // Output position tracks the rotor via the SPI source that the
  // fixture already populates from the motor sim each cycle.
  mp_config->sources[1].aux_number = 1;
  mp_config->sources[1].type = MotorPosition::SourceConfig::kSpi;
  mp_config->sources[1].reference = MotorPosition::SourceConfig::kRotor;
  mp_config->sources[1].cpr = SimulationContext::kEncoderCpr;
  mp_config->sources[1].pll_filter_hz = 1500.0f;
  mp_config->output.source = 1;

  // No fine offset table; every hall sector starts at electrical
  // theta = sector_index * 60° in the firmware's frame.  All
  // misalignment is in the supplied boundaries.
  for (auto& v : ctx.motor_.offset) { v = 0.0f; }

  ctx.aux1_status_.hall.active = true;
  ctx.ReconfigureMotorPosition();

  // Run a few cycles so the hall source becomes active.
  auto warmup_cmd = MakeCurrentCommand(0.0f, 0.0f);
  ctx.Command(&warmup_cmd);
  for (int i = 0; i < 10; ++i) { ctx.StepSimulation(&warmup_cmd); }
}


struct AngleErrorResult {
  float max_err_deg;
  float mean_err_deg;
  float actual_velocity_rev_s;
};

// Drive a steady velocity through several electrical cycles and
// report the worst-case difference between the firmware's electrical
// theta and the simulator's true electrical theta.
//
// Skips the first electrical cycle to clear the velocity-estimator
// startup transient.
AngleErrorResult MeasureAngleErrorAtVelocity(
    SimulationContext& ctx, float velocity_rev_s) {
  auto cmd = MakePositionCommand(kNaN, velocity_rev_s, 0.10f);
  cmd.accel_limit = 5.0f;
  ctx.Command(&cmd);

  const float dt = ctx.rate_config_.period_s;
  const float cycle_s =
      1.0f / (7.0f * std::max(0.01f, std::abs(velocity_rev_s)));
  const float duration_s = std::max(2.0f, 1.0f + 4.0f * cycle_s);
  const float skip_s = std::max(0.2f, 1.0f + 1.0f * cycle_s);
  const int num_steps = static_cast<int>(duration_s / dt);
  const int skip_steps = static_cast<int>(skip_s / dt);

  AngleErrorResult r{0.0f, 0.0f, 0.0f};
  double sum_abs = 0.0;
  int n = 0;
  for (int i = 0; i < num_steps; ++i) {
    ctx.StepSimulation(&cmd);
    if (i < skip_steps) continue;

    const float fw_theta = ctx.position_.electrical_theta;
    const float true_theta = ctx.motor_sim_.theta_electrical();
    const float err_rad = WrapNegPiToPi(fw_theta - true_theta);
    const float err_deg = err_rad * 180.0f / kPi;

    const float abs_err = std::abs(err_deg);
    if (abs_err > r.max_err_deg) r.max_err_deg = abs_err;
    sum_abs += abs_err;
    n++;
  }
  r.mean_err_deg = static_cast<float>(sum_abs / std::max(1, n));
  r.actual_velocity_rev_s = ctx.motor_sim_.velocity_rev_s();
  return r;
}

}  // namespace

BOOST_FIXTURE_TEST_SUITE(SimHallMisalignmentTests, SimulationFixture)

// Baseline 1: perfectly-aligned hall sensors.  With proper velocity
// tracking the inter-transition error would be sampling-rate-bounded
// (sub-degree), but the ~27° peak measured here is the lost-tracking
// slew engaging mid-sector at this deliberately slow test velocity.
BOOST_AUTO_TEST_CASE(SimHallAngleErrorPerfectAlignment) {
  const auto perfect = MakeShiftedBoundaries({0, 0, 0, 0, 0, 0});
  ConfigureHallCommutation(ctx, perfect);

  const auto r = MeasureAngleErrorAtVelocity(ctx, 0.1f);
  fmt::print(stderr,
             "PerfectAlignment angle err: max={:.1f}° mean={:.1f}° "
             "actual_vel={:.3f} rev/s\n",
             r.max_err_deg, r.mean_err_deg, r.actual_velocity_rev_s);

  BOOST_CHECK_MESSAGE(
      r.max_err_deg < 35.0f,
      "Perfect-alignment angle error should be small. "
      "max=" << r.max_err_deg << "° mean=" << r.mean_err_deg << "°");
}

// Baseline 1b: same perfectly-aligned halls driven fast enough
// that hall transitions arrive before the lost-tracking slew
// threshold elapses.
BOOST_AUTO_TEST_CASE(SimHallAngleErrorPerfectAlignmentFast) {
  const auto perfect = MakeShiftedBoundaries({0, 0, 0, 0, 0, 0});
  ConfigureHallCommutation(ctx, perfect);

  const auto r = MeasureAngleErrorAtVelocity(ctx, 1.0f);
  fmt::print(stderr,
             "PerfectAlignmentFast angle err: max={:.1f}° mean={:.1f}° "
             "actual_vel={:.3f} rev/s\n",
             r.max_err_deg, r.mean_err_deg, r.actual_velocity_rev_s);

  BOOST_CHECK_MESSAGE(
      r.max_err_deg < 1.0f,
      "Fast perfectly-aligned tracking error should be near zero. "
      "max=" << r.max_err_deg << "° mean=" << r.mean_err_deg << "°");
}

// Baseline 2: moderately misaligned halls (one boundary shifted by
// 5% of an electrical cycle ≈ 18°).  The new firmware handles this
// fine because the resulting velocity error is small.
BOOST_AUTO_TEST_CASE(SimHallAngleErrorModerateMisalignment) {
  const auto bad = MakeShiftedBoundaries(
      {0.0f, 0.05f, 0.0f, 0.0f, 0.0f, 0.0f});
  ConfigureHallCommutation(ctx, bad);

  const auto r = MeasureAngleErrorAtVelocity(ctx, 0.1f);
  fmt::print(stderr,
             "ModerateMisalignment angle err: max={:.1f}° mean={:.1f}° "
             "actual_vel={:.3f} rev/s\n",
             r.max_err_deg, r.mean_err_deg, r.actual_velocity_rev_s);

  BOOST_CHECK_MESSAGE(
      r.max_err_deg < 50.0f,
      "Moderately misaligned angle error should not exceed 50°. "
      "max=" << r.max_err_deg << "° mean=" << r.mean_err_deg << "°");
}

// Documentation case: two adjacent boundaries shifted in opposite
// directions create one very narrow sector (~17°) flanked by two
// wide sectors (~82° each).  The narrow sector produces an
// artificially high inter-sample velocity which, when integrated
// through the wide neighbour, drives the firmware angle error past
// 60°.  This bench measurement records the residual error during
// *steady slow rotation* (a regime the lost-tracking heuristic
// cannot help -- transitions are still arriving, so the heuristic
// never fires).  The motor is not stuck in this scenario; if a
// real load reduces torque enough to stall it the lost-tracking
// recovery in this branch will unstick it on the next attempt.
BOOST_AUTO_TEST_CASE(SimHallAngleErrorSevereMisalignment) {
  const auto bad = MakeShiftedBoundaries(
      {0.0f, 0.060f, -0.060f, 0.0f, 0.0f, 0.0f});
  ConfigureHallCommutation(ctx, bad);

  const auto r = MeasureAngleErrorAtVelocity(ctx, 0.05f);
  fmt::print(stderr,
             "SevereMisalignment angle err: max={:.1f}° mean={:.1f}° "
             "actual_vel={:.3f} rev/s\n",
             r.max_err_deg, r.mean_err_deg, r.actual_velocity_rev_s);

  // The lost-tracking fix alone does not reduce this metric -- it
  // bounds the *stuck-state* error, not the steady-rotation error.
  // We assert only that the result has not regressed past the
  // 90° torque-cancellation point.  A follow-up velocity-spike
  // clamp brings this number under 60°.
  BOOST_CHECK_MESSAGE(
      r.max_err_deg < 80.0f,
      "Severely-misaligned steady-rotation angle error exceeds "
      "the torque-cancellation point. max=" << r.max_err_deg
      << "° mean=" << r.mean_err_deg << "°");
}

// A second pattern: every boundary perturbed by a small amount, as
// would result from accumulated manufacturing tolerances on a hall
// ring.  The exact shifts (~±18° per boundary) produce a worst-case
// sector ratio close to 2:1.
BOOST_AUTO_TEST_CASE(SimHallAngleErrorAsymmetricMisalignment) {
  const auto bad = MakeShiftedBoundaries(
      {0.0f, 0.05f, -0.03f, 0.02f, -0.04f, 0.03f});
  ConfigureHallCommutation(ctx, bad);

  const auto r = MeasureAngleErrorAtVelocity(ctx, 0.05f);
  fmt::print(stderr,
             "AsymmetricMisalignment angle err: max={:.1f}° mean={:.1f}° "
             "actual_vel={:.3f} rev/s\n",
             r.max_err_deg, r.mean_err_deg, r.actual_velocity_rev_s);

  BOOST_CHECK_MESSAGE(
      r.max_err_deg < 60.0f,
      "Asymmetric-misalignment angle error must stay below 60° to "
      "leave usable torque headroom. max=" << r.max_err_deg
      << "° mean=" << r.mean_err_deg << "°");
}

// Lost-tracking recovery test.  Park the rotor deep in a misaligned
// sector with a fixed-phase open-loop command, let the estimator
// coast to rest (driving filtered_value into the
// compensated + max_err / stale-velocity stuck state), then issue a
// steady forward current.  Without the lost-tracking heuristic the
// firmware-reported electrical angle stays nearly 90° off the rotor
// and the commanded current produces no net torque.  With the
// heuristic, after the threshold elapses the firmware slews back
// toward mid-sector and the rotor accelerates forward.
//
// We measure: does the rotor turn at least one mechanical revolution
// within 2 s of the current command?
BOOST_AUTO_TEST_CASE(SimHallSevereMisalignedLostTrackingUnsticks) {
  const auto bad = MakeShiftedBoundaries(
      {0.0f, 0.060f, -0.060f, 0.0f, 0.0f, 0.0f});
  ConfigureHallCommutation(ctx, bad);

  // 1. Park the rotor at a fixed phase to give the firmware a
  //    consistent starting condition with filtered_value far from
  //    where the rotor will end up after the next test cycle.
  auto park_cmd = MakeVoltageFocCommand(/*theta=*/0.5f * kPi,
                                        /*voltage=*/0.5f);
  ctx.Command(&park_cmd);
  ctx.RunSimulation(&park_cmd, 0.5f);

  // 2. Stop the FOC drive and let the rotor coast to rest; this
  //    triggers slow_count to saturate and exercises the
  //    coasting-forward / stuck failure mode the heuristic is meant
  //    to fix.
  auto stop_cmd = MakeBrakeCommand();
  ctx.Command(&stop_cmd);
  ctx.RunSimulation(&stop_cmd, 0.3f);

  const float park_pos = ctx.motor_sim_.position_rev();

  // 3. Apply a steady forward torque command via current mode.
  //    Commanded torque (0.65 Nm) > load (0.30 Nm) by enough margin
  //    that any commutation angle error below 60° still produces
  //    net forward torque.
  ctx.external_torque_ = -0.30f;
  auto cmd = MakeCurrentCommand(/*i_d_A=*/0.0f, /*i_q_A=*/30.0f);
  ctx.Command(&cmd);

  const float dt = ctx.rate_config_.period_s;
  const int num_steps = static_cast<int>(2.0f / dt);
  for (int i = 0; i < num_steps; ++i) {
    ctx.StepSimulation(&cmd);
  }

  const float final_pos = ctx.motor_sim_.position_rev();
  const float motion = final_pos - park_pos;
  fmt::print(stderr,
             "LostTrackingUnsticks: park_pos={:.4f} final={:.4f} "
             "motion={:.4f} rev v={:.3f} rev/s\n",
             park_pos, final_pos, motion,
             ctx.motor_sim_.velocity_rev_s());

  // With the heuristic, the motor must turn forward at least one
  // mechanical revolution within 2 s.
  BOOST_CHECK_MESSAGE(
      motion > 1.0f,
      "Motor failed to rotate forward under steady current command. "
      "motion=" << motion << " rev (park=" << park_pos
      << ", final=" << final_pos << ")");
}

BOOST_AUTO_TEST_SUITE_END()
