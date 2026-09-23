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

#include "fw/fet_thermal_model.h"

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace {
FetThermalParams MakeN1LikeParams() {
  FetThermalParams p;
  p.rds_on_25C_ohm = 1.9e-3f;
  p.rds_on_tempco_per_C = 0.0053f;
  p.gate_switch_charge_coulomb = 28e-9f;
  p.r_junction_pad_C_per_W.fill(0.9f);
  p.tau_junction_pad_s.fill(0.03f);
  // No couplings: each pad conducts only to the thermistor, through
  // 2.5, 3.5, 3.5, 4.5, 4.5, and 5.5 C/W.
  p.g_pad_thermistor_W_per_K = {
    1.0f / 2.5f, 1.0f / 3.5f, 1.0f / 3.5f,
    1.0f / 4.5f, 1.0f / 4.5f, 1.0f / 5.5f };
  // 0.8 J/K gives channel 0's high side a 2 s pad time constant.
  p.c_pad_J_per_K = 0.8f;
  p.r_extra_ohm = 1.0e-3f;
  // The pads reach ambient only through the thermistor's copper, so
  // in steady state each junction's rise over the thermistor is just
  // its loss times its resistance to it.
  p.g_pad_ambient_W_per_K.fill(0.0f);
  p.c_thermistor_J_per_K = 1.0f;
  p.c_board_J_per_K = 5.0f;
  p.g_thermistor_board_W_per_K = 0.05f;
  p.g_thermistor_ambient_W_per_K = 0.2f;
  p.g_board_ambient_W_per_K = 0.05f;
  p.thermistor_lag_s = 0.1f;
  return p;
}

FetThermalModel::Inputs MakeLockedRotorInputs() {
  FetThermalModel::Inputs in;
  in.dt_s = 0.001f;
  in.thermistor_C = 60.0f;
  in.bus_V = 48.0f;
  in.pwm_rate_hz = 20000.0f;
  in.phase_A = { 55.0f, -27.5f, -27.5f };
  in.duty = { 0.5f, 0.5f, 0.5f };
  in.gate.hs_source_A = 0.15f;
  in.gate.hs_sink_A = 0.20f;
  in.gate.ls_source_A = 0.15f;
  in.gate.ls_sink_A = 0.10f;
  in.shunt_ohm = 0.0005f;
  in.junction_fault_C = 150.0f;
  in.junction_margin_C = 25.0f;
  return in;
}

void Run(FetThermalModel* dut, const FetThermalParams& params,
         const FetThermalModel::Inputs& in, FetThermalStatus* out,
         float duration_s) {
  const int steps = static_cast<int>(duration_s / in.dt_s);
  for (int i = 0; i < steps; i++) {
    dut->Update(params, in, out);
  }
}

// Initialize the model at the thermistor temperature, as if the board
// were known to be cold, rather than with the hot start of power up:
// with the junction fault disabled for the first update (which takes
// no time) there is no start of derating to start at.
void StartCold(FetThermalModel* dut, const FetThermalParams& params,
               const FetThermalModel::Inputs& in, FetThermalStatus* out) {
  auto first = in;
  first.junction_fault_C = FetThermalParams::kNaN;
  first.dt_s = 0.0f;
  dut->Update(params, first, out);
}
}

BOOST_AUTO_TEST_CASE(FetThermalModelDisabled) {
  FetThermalModel dut;
  FetThermalStatus status;
  dut.Update(FetThermalParams(), MakeLockedRotorInputs(), &status);
  BOOST_TEST(status.enabled == false);
  BOOST_TEST(std::isnan(status.junction_derate_fraction));
}

BOOST_AUTO_TEST_CASE(FetThermalModelNoLoss,
                     * boost::unit_test::tolerance(1e-3f)) {
  FetThermalModel dut;
  FetThermalStatus status;
  auto in = MakeLockedRotorInputs();
  in.phase_A = { 0.0f, 0.0f, 0.0f };
  in.thermistor_C = 137.5f;

  Run(&dut, MakeN1LikeParams(), in, &status, 1.0f);

  BOOST_TEST(status.enabled == true);
  for (int i = 0; i < 6; i++) {
    BOOST_TEST(status.loss_W[i] == 0.0f);
    BOOST_TEST(status.junction_C[i] == 137.5f);
  }
  // 137.5C is halfway through the 125-150 junction band.
  BOOST_TEST(status.junction_derate_fraction == 0.5f);
}

BOOST_AUTO_TEST_CASE(FetThermalModelLockedRotorSteadyState,
                     * boost::unit_test::tolerance(2e-2f)) {
  FetThermalModel dut;
  FetThermalStatus status;
  const auto params = MakeN1LikeParams();
  const auto in = MakeLockedRotorInputs();

  Run(&dut, params, in, &status, 60.0f);

  // Channel 0 carries +55A, so its high side FET hard switches and
  // both FETs conduct 55A half the time.
  const auto rds = [&](float junction_C) {
    return params.rds_on_25C_ohm *
        (1.0f + params.rds_on_tempco_per_C * (junction_C - 25.0f));
  };
  const float overlap_s = 28e-9f * (1.0f / 0.15f + 1.0f / 0.20f);
  const float expected_sw_W = 0.5f * 48.0f * 55.0f * overlap_s * 20000.0f;
  BOOST_TEST(expected_sw_W == 8.62f);

  const float expected_hs_W =
      55.0f * 55.0f * (rds(status.junction_C[0]) + 1.0e-3f) * 0.5f +
      expected_sw_W;
  BOOST_TEST(status.loss_W[0] == expected_hs_W);

  // The low side also carries the shunt's dissipation and its extra
  // series resistance.
  const float expected_ls_W =
      55.0f * 55.0f * (rds(status.junction_C[1]) + 0.0005f + 1.0e-3f) * 0.5f;
  BOOST_TEST(status.loss_W[1] == expected_ls_W);

  // In steady state the junction rise over the thermistor is just
  // the loss times the total resistance.
  BOOST_TEST(status.junction_C[0] ==
             60.0f + status.loss_W[0] * (0.9f + 2.5f));
  BOOST_TEST(status.pad_C[0] == 60.0f + status.loss_W[0] * 2.5f);

  // Channels 1 and 2 carry negative current, so their low side FETs
  // hard switch.
  BOOST_TEST(status.loss_W[3] > status.loss_W[2]);
  BOOST_TEST(status.loss_W[5] > status.loss_W[4]);
  BOOST_TEST(status.loss_W[3] == status.loss_W[5]);

  // With these constants, the hottest FET is actually the low side
  // of channel 2: it hard switches 27.5A with the slowest gate sink
  // and sits farthest from the thermistor.
  float max_junction_C = -1000.0f;
  for (const auto j : status.junction_C) {
    max_junction_C = std::max(max_junction_C, j);
  }
  BOOST_TEST(status.max_junction_C == max_junction_C);
  BOOST_TEST(status.max_junction_C == status.junction_C[5]);

  // Sanity check the absolute numbers against the hand analysis.
  BOOST_TEST(status.loss_W[0] > 12.0f);
  BOOST_TEST(status.loss_W[0] < 16.0f);
  BOOST_TEST(status.junction_C[0] > 100.0f);
  BOOST_TEST(status.junction_C[0] < 120.0f);
}

BOOST_AUTO_TEST_CASE(FetThermalModelRobustToBadInputs) {
  // A NaN or zero in any input must never propagate into the derate
  // fraction.
  FetThermalModel dut;
  FetThermalStatus status;
  const auto params = MakeN1LikeParams();
  constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

  auto in = MakeLockedRotorInputs();
  in.bus_V = kNaN;
  in.gate = {};
  in.duty = { kNaN, 2.0f, -1.0f };
  Run(&dut, params, in, &status, 0.5f);
  BOOST_TEST(status.enabled == true);
  for (int i = 0; i < 6; i++) {
    BOOST_TEST(std::isfinite(status.loss_W[i]));
    BOOST_TEST(std::isfinite(status.junction_C[i]));
  }
  BOOST_TEST(std::isfinite(status.junction_derate_fraction));

  // Absurd currents, such as the -165 A the current sense reports
  // while the driver is disabled, must not run away through the
  // Rds(on) temperature feedback.
  in = MakeLockedRotorInputs();
  in.phase_A = { -165.0f, -165.0f, -165.0f };
  Run(&dut, params, in, &status, 30.0f);
  for (int i = 0; i < 6; i++) {
    BOOST_TEST(std::isfinite(status.junction_C[i]));
    BOOST_TEST(status.junction_C[i] <= 1000.0f);
  }
  BOOST_TEST(std::isfinite(status.junction_derate_fraction));

  // A NaN thermistor disables the estimate rather than poisoning
  // it, and the model re-initializes cleanly when it returns.
  in = MakeLockedRotorInputs();
  in.thermistor_C = kNaN;
  dut.Update(params, in, &status);
  BOOST_TEST(status.enabled == false);
  BOOST_TEST(std::isnan(status.junction_derate_fraction));

  // Re-initializing starts hot, at the start of derating.
  in.thermistor_C = 40.0f;
  dut.Update(params, in, &status);
  BOOST_TEST(status.enabled == true);
  BOOST_TEST(std::isfinite(status.junction_derate_fraction));
  BOOST_TEST(status.junction_C[0] > 124.0f);
  BOOST_TEST(status.junction_C[0] < 127.0f);

  // A non-finite phase current, current amplitude, or PWM rate
  // disables the estimate for that step rather than poisoning the
  // state, and the model then starts hot again.
  for (const float bad : { kNaN, std::numeric_limits<float>::infinity() }) {
    for (int which = 0; which < 3; which++) {
      FetThermalModel d;
      FetThermalStatus st;
      auto good = MakeLockedRotorInputs();
      good.thermistor_C = 40.0f;
      StartCold(&d, params, good, &st);

      auto glitch = good;
      if (which == 0) { glitch.phase_A[0] = bad; }
      if (which == 1) {
        glitch.averaged = true;
        glitch.current_amplitude_A = bad;
      }
      if (which == 2) { glitch.pwm_rate_hz = bad; }
      d.Update(params, glitch, &st);
      BOOST_TEST(st.enabled == false);
      BOOST_TEST(std::isnan(st.junction_derate_fraction));

      d.Update(params, good, &st);
      BOOST_TEST(st.enabled == true);
      BOOST_TEST(std::isfinite(st.junction_derate_fraction));
      BOOST_TEST(st.max_junction_C > 124.0f);
    }
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelDynamics) {
  FetThermalModel dut;
  FetThermalStatus status;
  const auto params = MakeN1LikeParams();
  const auto in = MakeLockedRotorInputs();

  // Prime the model at the thermistor temperature.
  auto quiet = in;
  quiet.phase_A = { 0.0f, 0.0f, 0.0f };
  StartCold(&dut, params, quiet, &status);

  // After 5ms the junction should have moved, but be well short of
  // its steady state rise over the pad.
  Run(&dut, params, in, &status, 0.005f);
  const float steady_rise_over_pad = status.loss_W[0] * 0.9f;
  const float rise_5ms = status.junction_C[0] - status.pad_C[0];
  BOOST_TEST(rise_5ms > 0.05f * steady_rise_over_pad);
  BOOST_TEST(rise_5ms < 0.30f * steady_rise_over_pad);

  // After 200ms the junction to pad portion should be essentially
  // settled, but the pad should still be far from its steady state.
  Run(&dut, params, in, &status, 0.2f);
  const float rise_200ms = status.junction_C[0] - status.pad_C[0];
  BOOST_TEST(rise_200ms > 0.95f * status.loss_W[0] * 0.9f);
  const float pad_rise_200ms = status.pad_C[0] - in.thermistor_C;
  BOOST_TEST(pad_rise_200ms < 0.2f * status.loss_W[0] * 2.5f);

  // Removing the load lets the junction fall back to the pad
  // quickly.
  Run(&dut, params, quiet, &status, 0.2f);
  BOOST_TEST(std::abs(status.junction_C[0] - status.pad_C[0]) < 0.5f);
}

BOOST_AUTO_TEST_CASE(FetThermalModelDerateFraction) {
  BOOST_TEST(FetThermalModel::DerateFraction(125.0f, 150.0f, 25.0f) == 0.0f);
  BOOST_TEST(FetThermalModel::DerateFraction(150.0f, 150.0f, 25.0f) == 1.0f);
  BOOST_TEST(FetThermalModel::DerateFraction(100.0f, 150.0f, 25.0f) == -1.0f);

  // Without a margin there is only a cutoff at the fault temperature.
  constexpr float kInf = std::numeric_limits<float>::infinity();
  BOOST_TEST(FetThermalModel::DerateFraction(149.0f, 150.0f, 0.0f) == -kInf);
  BOOST_TEST(FetThermalModel::DerateFraction(151.0f, 150.0f, 0.0f) == kInf);
  BOOST_TEST(FetThermalModel::DerateFraction(151.0f, 150.0f, -5.0f) == kInf);

  constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
  BOOST_TEST(std::isnan(FetThermalModel::DerateFraction(100.0f, kNaN, 25.0f)));
  BOOST_TEST(std::isnan(FetThermalModel::DerateFraction(100.0f, kNaN, 0.0f)));
}

BOOST_AUTO_TEST_CASE(FetThermalModelSetPhases) {
  FetThermalModel::Inputs in;

  // Logical phases b and c drive physical channels 3 and 2.
  in.SetPhases(true, false, 1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f,
               0.0f, 0.0f, 0.0f);
  BOOST_TEST(in.phase_A[0] == 1.0f);
  BOOST_TEST(in.phase_A[1] == 2.0f);
  BOOST_TEST(in.phase_A[2] == 3.0f);
  BOOST_TEST(in.duty[0] == 0.1f);
  BOOST_TEST(in.duty[1] == 0.3f);
  BOOST_TEST(in.duty[2] == 0.2f);

  // With phase_invert, b and c drive channels 2 and 3, and
  // servo_stats has already swapped cur2_A and cur3_A.
  in.SetPhases(true, true, 1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f,
               0.0f, 0.0f, 0.0f);
  BOOST_TEST(in.phase_A[0] == 1.0f);
  BOOST_TEST(in.phase_A[1] == 3.0f);
  BOOST_TEST(in.phase_A[2] == 2.0f);
  BOOST_TEST(in.duty[0] == 0.1f);
  BOOST_TEST(in.duty[1] == 0.2f);
  BOOST_TEST(in.duty[2] == 0.3f);

  BOOST_TEST(in.averaged == false);

  // Above kAveragedAboveHz electrical the losses are averaged over a
  // revolution, with the amplitude from the d and q currents.
  in.SetPhases(true, false, 1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f,
               -60.0f, 3.0f, 4.0f);
  BOOST_TEST(in.averaged == true);
  BOOST_TEST(in.current_amplitude_A == 5.0f);

  // With the bridge off the current sense readings are ignored.
  in.SetPhases(false, false, -165.0f, -165.0f, -165.0f, 0.5f, 0.5f, 0.5f,
               100.0f, -165.0f, -165.0f);
  BOOST_TEST(in.averaged == false);
  for (int i = 0; i < 3; i++) {
    BOOST_TEST(in.phase_A[i] == 0.0f);
    BOOST_TEST(in.duty[i] == 0.0f);
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelNoSwitchingWhenHeld,
                     * boost::unit_test::tolerance(1e-4f)) {
  // A leg held fully off, as when braking, only conducts through its
  // low side and does not switch.
  FetThermalModel dut;
  FetThermalStatus status;
  const auto params = MakeN1LikeParams();
  auto in = MakeLockedRotorInputs();
  in.duty = { 0.0f, 0.0f, 0.0f };
  StartCold(&dut, params, in, &status);
  dut.Update(params, in, &status);

  const float rds = params.rds_on_25C_ohm *
      (1.0f + params.rds_on_tempco_per_C * (in.thermistor_C - 25.0f));
  BOOST_TEST(status.loss_W[0] == 0.0f);
  BOOST_TEST(status.loss_W[1] ==
             55.0f * 55.0f * (rds + params.r_extra_ohm + in.shunt_ohm));
}

namespace {
// Run the model with the motor rotating at `electrical_hz`, sampling
// the phase currents every `dt_s`, and return the lowest and highest
// max_junction_C seen over the last few seconds.  With `as_firmware`
// the inputs are built as the firmware does, including the choice of
// averaged losses; otherwise the phase currents are always used
// directly, which is exact given a small enough `dt_s`.
std::pair<float, float> RunRotating(float electrical_hz, float dt_s,
                                    bool as_firmware) {
  FetThermalModel dut;
  FetThermalStatus status;
  const auto params = MakeN1LikeParams();
  auto in = MakeLockedRotorInputs();
  in.dt_s = dt_s;

  constexpr float kAmplitude_A = 30.0f;
  constexpr float kDuration_s = 30.0f;
  constexpr float kWindow_s = 5.0f;
  const int steps = static_cast<int>(kDuration_s / dt_s);
  float lowest = std::numeric_limits<float>::infinity();
  float highest = -std::numeric_limits<float>::infinity();
  for (int step = 0; step < steps; step++) {
    const double t = step * static_cast<double>(dt_s);
    const double theta = 2.0 * M_PI * std::fmod(electrical_hz * t, 1.0);
    const auto phase = [&](int k) {
      return static_cast<float>(
          kAmplitude_A * std::cos(theta - k * 2.0 * M_PI / 3.0));
    };
    in.SetPhases(true, false, phase(0), phase(2), phase(1),
                 0.5f, 0.5f, 0.5f,
                 as_firmware ? electrical_hz : 0.0f, 0.0f, kAmplitude_A);
    dut.Update(params, in, &status);
    if (t > kDuration_s - kWindow_s) {
      lowest = std::min(lowest, status.max_junction_C);
      highest = std::max(highest, status.max_junction_C);
    }
  }
  return {lowest, highest};
}
}

BOOST_AUTO_TEST_CASE(FetThermalModelRotatingAliasing) {
  // The firmware samples the phase currents once per millisecond.
  // Near multiples of 1kHz electrical, samples of the phase currents
  // would see a nearly frozen electrical angle, and the model a
  // locked rotor at an arbitrary angle, or one rotating at the slow
  // beat frequency.
  //
  // Once the electrical period is well below the junction time
  // constant, the junction temperature does not depend on the
  // frequency.  So the reference is the phase currents at a
  // frequency the 1ms samples resolve well.  (Sampling faster
  // instead would make each step's change too small for the float
  // state to resolve.)
  const auto reference = RunRotating(137.0f, 1e-3f, false);
  const float reference_C = 0.5f * (reference.first + reference.second);
  for (const float electrical_hz :
           { 50.0f, 700.0f, 1000.0f, 1000.3f, 2000.1f }) {
    const auto sampled = RunRotating(electrical_hz, 1e-3f, true);
    BOOST_TEST_MESSAGE("electrical_hz=" << electrical_hz
                       << " reference=" << reference.first
                       << ".." << reference.second
                       << " sampled=" << sampled.first
                       << ".." << sampled.second);
    BOOST_TEST(std::abs(sampled.first - reference_C) < 1.0f,
               "electrical_hz=" << electrical_hz);
    BOOST_TEST(std::abs(sampled.second - reference_C) < 1.0f,
               "electrical_hz=" << electrical_hz);
  }
}

namespace {
// A network like the moteus-n1's: FETs in one column, the first two
// next to the thermistor.
FetThermalParams MakeNetworkParams() {
  auto p = MakeN1LikeParams();
  p.c_pad_J_per_K = 0.25f;
  p.g_pad_thermistor_W_per_K = { 0.18f, 0.05f, 0.05f, 0.03f, 0.03f, 0.04f };
  p.network_couplings = 5;
  p.coupling_a = { 0, 1, 2, 2, 4 };
  p.coupling_b = { 1, 3, 3, 4, 5 };
  p.coupling_W_per_K = { 0.03f, 0.02f, 0.05f, 0.04f, 0.05f };
  return p;
}

FetThermalModel::Inputs MakeModerateInputs() {
  // 20A at the angle that puts the full current in channel 0.
  auto in = MakeLockedRotorInputs();
  in.thermistor_C = 40.0f;
  in.phase_A = { 20.0f, -10.0f, -10.0f };
  return in;
}

// The steady junction temperatures of the network for fixed losses,
// by solving G T = P + g_th T_th in double precision.
std::array<double, 6> NetworkSteadyState(const FetThermalParams& p,
                                         const std::array<float, 6>& loss,
                                         double thermistor_C) {
  double G[6][7] = {};
  for (int i = 0; i < 6; i++) {
    G[i][i] = p.g_pad_thermistor_W_per_K[i];
    G[i][6] = loss[i] + p.g_pad_thermistor_W_per_K[i] * thermistor_C;
  }
  for (int k = 0; k < p.network_couplings; k++) {
    const int a = p.coupling_a[k];
    const int b = p.coupling_b[k];
    const double g = p.coupling_W_per_K[k];
    G[a][a] += g;
    G[b][b] += g;
    G[a][b] -= g;
    G[b][a] -= g;
  }
  // Gaussian elimination; G is symmetric positive definite.
  for (int c = 0; c < 6; c++) {
    for (int r = c + 1; r < 6; r++) {
      const double f = G[r][c] / G[c][c];
      for (int k = c; k < 7; k++) { G[r][k] -= f * G[c][k]; }
    }
  }
  std::array<double, 6> pad = {};
  for (int r = 5; r >= 0; r--) {
    double v = G[r][6];
    for (int k = r + 1; k < 6; k++) { v -= G[r][k] * pad[k]; }
    pad[r] = v / G[r][r];
  }
  std::array<double, 6> junction = {};
  for (int i = 0; i < 6; i++) {
    junction[i] = pad[i] + loss[i] * p.r_junction_pad_C_per_W[i];
  }
  return junction;
}
}

BOOST_AUTO_TEST_CASE(FetThermalModelNetworkSteadyState) {
  FetThermalModel dut;
  FetThermalStatus status;
  const auto params = MakeNetworkParams();
  BOOST_TEST(params.valid());
  const auto in = MakeModerateInputs();

  // The pads settle in ~6 s, but the board node behind the
  // thermistor's copper takes minutes, and while it moves the reading
  // lags it.  600 s is well settled.
  StartCold(&dut, params, in, &status);
  Run(&dut, params, in, &status, 600.0f);

  const auto expected = NetworkSteadyState(params, status.loss_W, in.thermistor_C);
  for (int i = 0; i < 6; i++) {
    BOOST_TEST_MESSAGE("fet " << i << " junction " << status.junction_C[i]
                       << " expected " << expected[i]);
    BOOST_TEST(std::abs(status.junction_C[i] - expected[i]) < 0.05);
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelNetworkNoLoss,
                     * boost::unit_test::tolerance(1e-3f)) {
  FetThermalModel dut;
  FetThermalStatus status;
  auto in = MakeModerateInputs();
  in.phase_A = { 0.0f, 0.0f, 0.0f };
  StartCold(&dut, MakeNetworkParams(), in, &status);
  Run(&dut, MakeNetworkParams(), in, &status, 5.0f);
  for (int i = 0; i < 6; i++) {
    BOOST_TEST(status.junction_C[i] == in.thermistor_C);
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelHotStart,
                     * boost::unit_test::tolerance(1e-3f)) {
  // At power up every junction starts at the start of derating, and
  // with no loss relaxes to the thermistor.
  const auto params = MakeNetworkParams();
  auto in = MakeModerateInputs();
  in.phase_A = { 0.0f, 0.0f, 0.0f };
  in.thermistor_C = 40.0f;

  FetThermalModel dut;
  FetThermalStatus status;
  dut.Update(params, in, &status);
  for (int i = 0; i < 6; i++) {
    BOOST_TEST(std::abs(status.junction_C[i] - 125.0f) < 0.1f);
  }
  BOOST_TEST(std::abs(status.junction_derate_fraction) < 0.01f);

  // The heat leaving the pads raises the modeled thermistor reading
  // while the junctions cool, but no estimate reads below the
  // thermistor.
  float last = status.max_junction_C;
  for (int k = 0; k < 600; k++) {
    Run(&dut, params, in, &status, 1.0f);
    BOOST_TEST(status.max_junction_C <= last + 1e-3f);
    last = status.max_junction_C;
    for (int i = 0; i < 6; i++) {
      BOOST_TEST(status.junction_C[i] >= in.thermistor_C);
    }
  }
  BOOST_TEST(status.max_junction_C < in.thermistor_C + 1.0f);

  // Already past the start of derating, it starts at the thermistor.
  {
    FetThermalModel d;
    FetThermalStatus st;
    auto hot = in;
    hot.thermistor_C = 130.0f;
    d.Update(params, hot, &st);
    for (int i = 0; i < 6; i++) { BOOST_TEST(st.junction_C[i] == 130.0f); }
  }
  // With the junction fault disabled there is no start of derating.
  {
    FetThermalModel d;
    FetThermalStatus st;
    auto off = in;
    off.junction_fault_C = FetThermalParams::kNaN;
    d.Update(params, off, &st);
    for (int i = 0; i < 6; i++) { BOOST_TEST(st.junction_C[i] == 40.0f); }
  }
  // Without a hot start, it starts at the thermistor.
  {
    FetThermalModel d;
    FetThermalStatus st;
    auto cold = in;
    cold.hot_start = false;
    d.Update(params, cold, &st);
    for (int i = 0; i < 6; i++) { BOOST_TEST(st.junction_C[i] == 40.0f); }
    BOOST_TEST(st.junction_derate_fraction < -3.0f);
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelNeverBelowThermistor) {
  // A FET whose pad sheds its heat to ambient, idle while its
  // neighbours heat the thermistor, is modeled below the thermistor.
  // Its estimate reads the thermistor instead.
  auto params = MakeNetworkParams();
  params.g_pad_ambient_W_per_K[5] = 0.5f;
  auto in = MakeModerateInputs();
  in.phase_A = { 20.0f, -20.0f, 0.0f };

  FetThermalModel dut;
  FetThermalStatus status;
  StartCold(&dut, params, in, &status);
  Run(&dut, params, in, &status, 60.0f);

  BOOST_TEST(status.pad_C[5] < in.thermistor_C - 1.0f);
  BOOST_TEST(status.junction_C[5] == in.thermistor_C);
  BOOST_TEST(status.junction_C[0] > in.thermistor_C + 1.0f);
}

BOOST_AUTO_TEST_CASE(FetThermalModelFollowsThermistor,
                     * boost::unit_test::tolerance(1e-3f)) {
  // The estimate is the measured thermistor plus the modeled rise
  // over it, so it moves one for one with the reading.
  const auto params = MakeNetworkParams();
  auto in = MakeModerateInputs();
  FetThermalModel a, b;
  FetThermalStatus sa, sb;
  StartCold(&a, params, in, &sa);
  Run(&a, params, in, &sa, 20.0f);
  auto warmer = in;
  warmer.thermistor_C += 10.0f;
  StartCold(&b, params, in, &sb);
  Run(&b, params, in, &sb, 20.0f - in.dt_s);
  b.Update(params, warmer, &sb);
  // The losses differ through Rds(on) only in the last step.
  for (int i = 0; i < 6; i++) {
    BOOST_TEST(sb.junction_C[i] - sa.junction_C[i] == 10.0f);
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelPadToAmbient) {
  // A pad's own path to ambient carries heat that otherwise reaches
  // the thermistor, so its junction runs less far above the reading.
  const auto in = MakeModerateInputs();
  const auto base = MakeNetworkParams();
  auto cooled = base;
  cooled.g_pad_ambient_W_per_K[2] = 0.2f;
  FetThermalModel a, b;
  FetThermalStatus sa, sb;
  StartCold(&a, base, in, &sa);
  StartCold(&b, cooled, in, &sb);
  Run(&a, base, in, &sa, 600.0f);
  Run(&b, cooled, in, &sb, 600.0f);
  BOOST_TEST(sb.junction_C[2] < sa.junction_C[2] - 1.0f);
}

BOOST_AUTO_TEST_CASE(FetThermalModelNetworkCouplingSharesHeat) {
  // Coupling two neighbouring pads carries heat from the hotter to
  // the cooler: the hotter pad runs cooler and the cooler one hotter
  // than with the coupling removed, and the gap between them closes.
  const auto in = MakeModerateInputs();
  auto coupled = MakeNetworkParams();
  coupled.coupling_W_per_K[1] = 0.2f;     // FET 1 to FET 3
  auto uncoupled = coupled;
  uncoupled.coupling_W_per_K[1] = 0.0f;

  FetThermalModel a, b;
  FetThermalStatus sa, sb;
  StartCold(&a, coupled, in, &sa);
  StartCold(&b, uncoupled, in, &sb);
  // Long enough for the board behind the thermistor to settle too.
  Run(&a, coupled, in, &sa, 600.0f);
  Run(&b, uncoupled, in, &sb, 600.0f);

  const int hot = (sb.pad_C[1] > sb.pad_C[3]) ? 1 : 3;
  const int cool = 4 - hot;
  BOOST_TEST(sb.pad_C[hot] > sb.pad_C[cool] + 1.0f);
  BOOST_TEST(sa.pad_C[hot] < sb.pad_C[hot] - 0.5f);
  BOOST_TEST(sa.pad_C[cool] > sb.pad_C[cool]);
  BOOST_TEST((sa.pad_C[hot] - sa.pad_C[cool]) <
             0.5f * (sb.pad_C[hot] - sb.pad_C[cool]));
}

BOOST_AUTO_TEST_CASE(FetThermalModelNetworkIncompleteDisables) {
  // Network constants that are not all finite disable the estimate,
  // rather than quietly running some other model.
  auto broken = MakeNetworkParams();
  broken.coupling_W_per_K[3] = FetThermalParams::kNaN;
  BOOST_TEST(!broken.valid());
  auto pad = MakeNetworkParams();
  pad.g_pad_thermistor_W_per_K[3] = FetThermalParams::kNaN;
  BOOST_TEST(!pad.valid());

  FetThermalModel dut;
  FetThermalStatus status;
  Run(&dut, broken, MakeModerateInputs(), &status, 1.0f);
  BOOST_TEST(status.enabled == false);
}

BOOST_AUTO_TEST_CASE(FetThermalModelRejectsFloatingPad) {
  // A pad with no path to ambient, directly, through the thermistor's
  // copper, or through its neighbours, would heat without bound.
  auto alone = MakeN1LikeParams();
  alone.g_pad_thermistor_W_per_K[4] = 0.0f;
  BOOST_TEST(!alone.valid());
  // Its own path to ambient is enough.
  alone.g_pad_ambient_W_per_K[4] = 0.01f;
  BOOST_TEST(alone.valid());

  // So is the thermistor's copper, through the board...
  auto through_board = MakeN1LikeParams();
  through_board.g_thermistor_ambient_W_per_K = 0.0f;
  BOOST_TEST(through_board.valid());
  // ...but not with neither.
  through_board.g_board_ambient_W_per_K = 0.0f;
  BOOST_TEST(!through_board.valid());

  // Through a neighbour is fine, even at the end of a chain...
  auto chained = MakeNetworkParams();
  chained.g_pad_thermistor_W_per_K[5] = 0.0f;
  chained.g_pad_thermistor_W_per_K[4] = 0.0f;
  BOOST_TEST(chained.valid());

  // ...but not through a coupling of zero.
  chained.coupling_W_per_K[4] = 0.0f;    // FET 4 to FET 5
  BOOST_TEST(!chained.valid());
}

BOOST_AUTO_TEST_CASE(FetThermalModelPerFetJunction) {
  // Each FET has its own junction stage: here the low sides are
  // slower and have more resistance, as on the moteus-n1, where their
  // drain pad is the smaller phase node copper.
  auto params = MakeN1LikeParams();
  for (int i = 0; i < 6; i++) {
    const bool high_side = (i % 2) == 0;
    params.r_junction_pad_C_per_W[i] = high_side ? 5.9f : 8.4f;
    params.tau_junction_pad_s[i] = high_side ? 0.09f : 0.27f;
  }
  // A large pad capacity, so the pads barely move over the junction
  // time constants.
  params.c_pad_J_per_K = 100.0f;
  auto in = MakeModerateInputs();

  FetThermalModel dut;
  FetThermalStatus status;
  auto quiet = in;
  quiet.phase_A = { 0.0f, 0.0f, 0.0f };
  StartCold(&dut, params, quiet, &status);

  // After one time constant each junction has made about 63% of its
  // rise over its pad.
  for (const int i : { 0, 1 }) {
    FetThermalModel d;
    FetThermalStatus st;
    StartCold(&d, params, quiet, &st);
    Run(&d, params, in, &st, params.tau_junction_pad_s[i]);
    const float fraction = (st.junction_C[i] - st.pad_C[i]) /
        (st.loss_W[i] * params.r_junction_pad_C_per_W[i]);
    BOOST_TEST_MESSAGE("fet " << i << " fraction " << fraction);
    BOOST_TEST(std::abs(fraction - 0.632f) < 0.03f);
  }

  // Well past both, each junction sits its own resistance times its
  // loss above its pad.
  Run(&dut, params, in, &status, 3.0f);
  for (int i = 0; i < 6; i++) {
    const float expected = status.loss_W[i] * params.r_junction_pad_C_per_W[i];
    BOOST_TEST(std::abs((status.junction_C[i] - status.pad_C[i]) - expected) <
               0.02f * expected + 0.01f);
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelRejectsBadJunctionConstants) {
  FetThermalModel dut;
  FetThermalStatus status;
  const auto in = MakeModerateInputs();
  for (const float bad : { 0.0f, -1.0f, FetThermalParams::kNaN }) {
    auto r = MakeN1LikeParams();
    r.r_junction_pad_C_per_W[3] = bad;
    BOOST_TEST(!r.valid());
    auto tau = MakeN1LikeParams();
    tau.tau_junction_pad_s[4] = bad;
    BOOST_TEST(!tau.valid());
  }
  auto r = MakeN1LikeParams();
  r.r_junction_pad_C_per_W[3] = 0.0f;
  dut.Update(r, in, &status);
  BOOST_TEST(status.enabled == false);
}

BOOST_AUTO_TEST_CASE(FetThermalModelNetworkRejectsBadConstants) {
  auto negative = MakeNetworkParams();
  negative.coupling_W_per_K[2] = -0.01f;
  BOOST_TEST(!negative.valid());
  auto negative_pad = MakeNetworkParams();
  negative_pad.g_pad_thermistor_W_per_K[1] = -0.01f;
  BOOST_TEST(!negative_pad.valid());
  auto self = MakeNetworkParams();
  self.coupling_b[0] = self.coupling_a[0];
  BOOST_TEST(!self.valid());
  auto out_of_range = MakeNetworkParams();
  out_of_range.coupling_b[4] = 6;
  BOOST_TEST(!out_of_range.valid());
  auto too_many = MakeNetworkParams();
  too_many.network_couplings = FetThermalParams::kMaxCouplings + 1;
  BOOST_TEST(!too_many.valid());
  for (const float bad : { 0.0f, -1.0f, FetThermalParams::kNaN }) {
    auto c_th = MakeNetworkParams();
    c_th.c_thermistor_J_per_K = bad;
    BOOST_TEST(!c_th.valid());
    auto c_b = MakeNetworkParams();
    c_b.c_board_J_per_K = bad;
    BOOST_TEST(!c_b.valid());
    auto lag = MakeNetworkParams();
    lag.thermistor_lag_s = bad;
    BOOST_TEST(!lag.valid());
  }
  for (const float bad : { -0.01f, FetThermalParams::kNaN }) {
    auto amb = MakeNetworkParams();
    amb.g_pad_ambient_W_per_K[2] = bad;
    BOOST_TEST(!amb.valid());
    auto tb = MakeNetworkParams();
    tb.g_thermistor_board_W_per_K = bad;
    BOOST_TEST(!tb.valid());
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelParamsCheckedWhileUninitialized) {
  // The constants are checked when the model initializes.  Until
  // valid constants arrive it stays uninitialized and checks every
  // call, so a model that started with none picks them up.
  FetThermalModel dut;
  FetThermalStatus status;
  const auto in = MakeModerateInputs();
  dut.Update(FetThermalParams(), in, &status);
  BOOST_TEST(status.enabled == false);
  dut.Update(MakeNetworkParams(), in, &status);
  BOOST_TEST(status.enabled == true);
}

namespace {
FetThermalParams WithSpreader(FetThermalParams p, float g_ps, float g_sa) {
  p.g_pad_spreader_W_per_K = g_ps;
  p.g_spreader_ambient_W_per_K = g_sa;
  p.c_spreader_J_per_K = 10.0f;
  return p;
}
}

BOOST_AUTO_TEST_CASE(FetThermalModelSpreaderNeverBelowBare) {
  // With a spreader the board is run with and without it, and each
  // junction reads the hotter, so never below the bare board.
  const auto bare = MakeNetworkParams();
  const auto spread = WithSpreader(bare, 0.5f, 0.5f);
  BOOST_TEST(spread.valid());
  const auto in = MakeModerateInputs();
  FetThermalModel a, b;
  FetThermalStatus sa, sb;
  StartCold(&a, bare, in, &sa);
  StartCold(&b, spread, in, &sb);
  for (int k = 0; k < 60; k++) {
    Run(&a, bare, in, &sa, 5.0f);
    Run(&b, spread, in, &sb, 5.0f);
    for (int i = 0; i < 6; i++) {
      BOOST_TEST(sb.junction_C[i] >= sa.junction_C[i] - 1e-3f);
    }
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelSpreaderCarriesHeat) {
  // A strong spreader with no path of its own to ambient carries heat
  // from FETs loaded at the far end of the board to FET 0 next to the
  // thermistor, which then reads hotter than on the bare board.
  const auto bare = MakeNetworkParams();
  const auto spread = WithSpreader(bare, 1.0f, 0.0f);
  BOOST_TEST(spread.valid());
  auto in = MakeModerateInputs();
  in.phase_A = { -10.0f, -10.0f, 20.0f };
  FetThermalModel a, b;
  FetThermalStatus sa, sb;
  StartCold(&a, bare, in, &sa);
  StartCold(&b, spread, in, &sb);
  Run(&a, bare, in, &sa, 600.0f);
  Run(&b, spread, in, &sb, 600.0f);
  BOOST_TEST(sb.junction_C[0] > sa.junction_C[0] + 5.0f);
}

BOOST_AUTO_TEST_CASE(FetThermalModelSpreaderHotStart,
                     * boost::unit_test::tolerance(1e-3f)) {
  // Both runs start hot.
  const auto params = WithSpreader(MakeNetworkParams(), 1.0f, 0.5f);
  auto in = MakeModerateInputs();
  in.phase_A = { 0.0f, 0.0f, 0.0f };
  FetThermalModel dut;
  FetThermalStatus status;
  dut.Update(params, in, &status);
  for (int i = 0; i < 6; i++) {
    BOOST_TEST(std::abs(status.junction_C[i] - 125.0f) < 0.1f);
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelRejectsBadSpreader) {
  const auto ok = WithSpreader(MakeNetworkParams(), 0.2f, 0.5f);
  BOOST_TEST(ok.valid());
  for (const float bad : { 0.0f, -1.0f, FetThermalParams::kNaN }) {
    auto c = ok;
    c.c_spreader_J_per_K = bad;
    BOOST_TEST(!c.valid());
  }
  for (const float bad : { -0.01f, FetThermalParams::kNaN }) {
    auto g = ok;
    g.g_spreader_ambient_W_per_K = bad;
    BOOST_TEST(!g.valid());
    auto ps = ok;
    ps.g_pad_spreader_W_per_K = bad;
    BOOST_TEST(!ps.valid());
  }
  // Without a spreader its heat capacity does not matter.
  auto none = MakeNetworkParams();
  none.c_spreader_J_per_K = 0.0f;
  BOOST_TEST(none.valid());
}

namespace {
FetThermalParams WithSwitchingEnergies(FetThermalParams p) {
  p.switching_energy_J_per_V_A = 14e-9f;
  p.switching_energy_J_per_A2 = 26e-9f;
  p.deadtime_diode_V_s = 150e-9f;
  p.switching_energy_J = 0.1e-6f;
  p.switching_energy_J_per_V = 0.47e-6f;
  return p;
}

// The losses of the first update from a cold start, when every
// junction is at the thermistor.
std::array<float, 6> FirstLosses(const FetThermalParams& params,
                                 const FetThermalModel::Inputs& in) {
  FetThermalModel dut;
  FetThermalStatus status;
  StartCold(&dut, params, in, &status);
  dut.Update(params, in, &status);
  return status.loss_W;
}
}

BOOST_AUTO_TEST_CASE(FetThermalModelSwitchingEnergies,
                     * boost::unit_test::tolerance(1e-4f)) {
  const auto base = MakeN1LikeParams();
  const auto params = WithSwitchingEnergies(base);
  const auto in = MakeLockedRotorInputs();
  const auto without = FirstLosses(base, in);
  const auto with = FirstLosses(params, in);

  const float f = in.pwm_rate_hz;
  const float V = in.bus_V;
  const float leg_W = 0.5f * f * (0.1e-6f + 0.47e-6f * V);
  const auto hard_W = [&](float i) {
    return f * (14e-9f * V * i + 26e-9f * i * i) + leg_W;
  };
  const auto diode_W = [&](float i) {
    return 2.0f * f * 150e-9f * i + leg_W;
  };
  // Channel 0 carries +55A, so its high side hard switches and its
  // low side's body diode conducts in the dead time; channels 1 and 2
  // carry -27.5A the other way round.
  BOOST_TEST(with[0] - without[0] == hard_W(55.0f));
  BOOST_TEST(with[1] - without[1] == diode_W(55.0f));
  for (const int hs : { 2, 4 }) {
    BOOST_TEST(with[hs] - without[hs] == diode_W(27.5f));
    BOOST_TEST(with[hs + 1] - without[hs + 1] == hard_W(27.5f));
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelHardSwitchingRate,
                     * boost::unit_test::tolerance(1e-4f)) {
  // hard_switching_per_Hz scales the hard switching FET's switching
  // loss, through the gate drive and the energies that do not depend
  // on it, and nothing else.
  const auto params = WithSwitchingEnergies(MakeN1LikeParams());
  auto no_switching = params;
  no_switching.switching_loss_scale = 0.0f;
  no_switching.switching_energy_J_per_V_A = 0.0f;
  no_switching.switching_energy_J_per_A2 = 0.0f;
  auto rated = params;
  rated.hard_switching_per_Hz = 4e-6f;
  const auto in = MakeLockedRotorInputs();
  const auto base = FirstLosses(params, in);
  const auto rest = FirstLosses(no_switching, in);
  const auto with = FirstLosses(rated, in);
  const float k = 4e-6f * in.pwm_rate_hz;
  // Channel 0 carries +55A and channels 1 and 2 -27.5A, so 0, 3 and 5
  // hard switch.
  for (const int hard : { 0, 3, 5 }) {
    BOOST_TEST(base[hard] - rest[hard] > 0.0f);
    BOOST_TEST(with[hard] - base[hard] == k * (base[hard] - rest[hard]));
  }
  for (const int other : { 1, 2, 4 }) {
    BOOST_TEST(with[other] == base[other]);
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelNoSwitchingEnergiesWhenHeld,
                     * boost::unit_test::tolerance(1e-5f)) {
  const auto base = MakeN1LikeParams();
  auto in = MakeLockedRotorInputs();
  in.duty = { 0.0f, 1.0f, 0.0f };
  const auto without = FirstLosses(base, in);
  const auto with = FirstLosses(WithSwitchingEnergies(base), in);
  for (int i = 0; i < 6; i++) {
    BOOST_TEST(with[i] == without[i]);
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelAveragedMatchesLockedMean,
                     * boost::unit_test::tolerance(1e-3f)) {
  // The averaged losses are the mean of the locked rotor losses over
  // a revolution.
  auto params = WithSwitchingEnergies(MakeN1LikeParams());
  params.hard_switching_per_Hz = 4e-6f;
  auto in = MakeLockedRotorInputs();
  constexpr float kAmplitude_A = 40.0f;
  constexpr int kAngles = 3600;
  std::array<double, 6> mean = {};
  for (int k = 0; k < kAngles; k++) {
    const double theta = 2.0 * M_PI * k / kAngles;
    for (int ch = 0; ch < 3; ch++) {
      in.phase_A[ch] = static_cast<float>(
          kAmplitude_A * std::cos(theta - ch * 2.0 * M_PI / 3.0));
    }
    const auto loss = FirstLosses(params, in);
    for (int i = 0; i < 6; i++) { mean[i] += loss[i] / kAngles; }
  }
  in.averaged = true;
  in.current_amplitude_A = kAmplitude_A;
  const auto averaged = FirstLosses(params, in);
  for (int i = 0; i < 6; i++) {
    BOOST_TEST(averaged[i] == static_cast<float>(mean[i]));
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelRejectsNegativeLosses) {
  using Field = float FetThermalParams::*;
  for (const Field field : {
           &FetThermalParams::rds_on_25C_ohm,
           &FetThermalParams::gate_switch_charge_coulomb,
           &FetThermalParams::switching_loss_scale,
           &FetThermalParams::switching_energy_J_per_V_A,
           &FetThermalParams::switching_energy_J_per_A2,
           &FetThermalParams::deadtime_diode_V_s,
           &FetThermalParams::switching_energy_J,
           &FetThermalParams::switching_energy_J_per_V,
           &FetThermalParams::hard_switching_per_Hz }) {
    auto params = MakeN1LikeParams();
    BOOST_TEST(params.valid());
    params.*field = -1e-9f;
    BOOST_TEST(!params.valid());
    params.*field = std::numeric_limits<float>::quiet_NaN();
    BOOST_TEST(!params.valid());
  }
}

BOOST_AUTO_TEST_CASE(FetThermalModelRdsOnRange) {
  // Rds(on) is only evaluated from -55C to 200C, and the constants
  // are rejected if it would be negative anywhere in that range.
  auto params = MakeN1LikeParams();
  BOOST_TEST(params.rds_on_ohm(-100.0f) == params.rds_on_ohm(-55.0f));
  BOOST_TEST(params.rds_on_ohm(300.0f) == params.rds_on_ohm(200.0f));
  BOOST_TEST(params.rds_on_ohm(25.0f) == params.rds_on_25C_ohm);

  // A small negative tempco is allowed.
  params.rds_on_tempco_per_C = -0.001f;
  BOOST_TEST(params.valid());
  // This one is negative at 200C, 1 - 0.01 * 175.
  params.rds_on_tempco_per_C = -0.01f;
  BOOST_TEST(!params.valid());
  // And this one at -55C, 1 - 0.02 * 80.
  params.rds_on_tempco_per_C = 0.02f;
  BOOST_TEST(!params.valid());
  params.rds_on_tempco_per_C = std::numeric_limits<float>::quiet_NaN();
  BOOST_TEST(!params.valid());
}
