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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "mjlib/base/visitor.h"

#include "fw/math.h"

namespace moteus {

/// The physical gate drive currents currently configured in the gate
/// driver, in amperes.
struct GateDriveCurrents {
  float hs_source_A = 0.0f;
  float hs_sink_A = 0.0f;
  float ls_source_A = 0.0f;
  float ls_sink_A = 0.0f;
};

/// Per-board constants for the FET junction temperature estimator.
/// The silicon and package constants come from the FET datasheet.
/// The rest depend on board layout and are measured with a thermal
/// camera on each board design.
///
/// The model is the whole board, as rises over ambient: each of the 6
/// FETs has a junction node above a pad node (the copper under the
/// package), the pads conduct to each other, to ambient, and to the
/// copper at the thermistor, which conducts to ambient and to the bulk
/// of the board, and the thermistor reads that copper through a first
/// order lag:
///
///   L_i --> [junction i] --r_junction_pad_i--> [pad i]
///   c_j_i dJ_i/dt = L_i - q_i
///   c_pad dP_i/dt = q_i - g_amb_i P_i - g_th_i (P_i - T)
///                   - sum_j g_ij (P_i - P_j)
///   C_T dT/dt = sum_i g_th_i (P_i - T) - g_tb (T - B) - g_ta T
///   C_B dB/dt = g_tb (T - B) - g_ba B
///   lag dS/dt = T - S
///
/// where L_i is FET i's loss, q_i = (J_i - P_i) / r_junction_pad_i is
/// the heat its junction passes to its pad, and the junction's heat
/// capacity c_j_i is tau_junction_pad_i / r_junction_pad_i.
///
/// Each junction's estimate is the measured thermistor plus its
/// modeled rise over the modeled thermistor reading, J_i - S, which
/// does not depend on ambient, and never less than the thermistor.
/// The modeled rise can be negative, for a FET that sheds its heat to
/// ambient while its neighbours heat the thermistor, and for a minute
/// or so after the hot start below, while the heat leaving the pads
/// keeps the modeled reading above the cooling junctions.  Reading
/// the thermistor instead only errs high.
///
/// Optionally the board is run a second time with a heat spreader H on
/// its pads,
///
///   c_pad dP_i/dt = ... - g_ps (P_i - H)
///   C_H dH/dt = sum_i g_ps (P_i - H) - g_ha H
///
/// and each junction's estimate is the hotter of the two.  A board
/// without a spreader and one with a strong spreader are each worst
/// for different coolings and loads: the bare board when the pads
/// cannot shed their heat, the spreader when it carries heat from the
/// loaded FETs to the thermistor's far end.  Neither alone covers both
/// without reading far above the board.
///
/// The constants are fitted so that, for every cooling the fixture
/// measured and for simulated fans and heat spreaders on the FETs, the
/// estimate reads at or above the board in steady state and in its
/// response to a step of loss, and in closed loop with the default
/// derating keeps the true junction 10C or more below the FETs'
/// absolute maximum (see the mfg-moteus fixture tools).
///
/// The model starts hot: with nothing known of the FETs' state at
/// power up, every junction and pad starts at the start of derating,
/// and relaxes from there.  An estimate started at the thermistor
/// would let a board restarted while hot take a full cold board's
/// heating on top of hot FETs.  Since the estimate reads close to the
/// board, a board power cycled while driving its FETs into derating
/// can still restart with them hotter than that, which is accepted as
/// rare.  With Inputs::hot_start false, it instead starts with every
/// junction and pad at the thermistor's reading.
///
/// The 6 FETs are indexed as [channel * 2 + (0 = high side, 1 = low
/// side)], where "channel" is the physical current sense / PWM channel
/// number, 0-2.
///
/// If any field is invalid the model reports itself as not enabled
/// and produces no estimate; every board is expected to provide a
/// complete set.
struct FetThermalParams {
  static constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

  // Incremented whenever the loss or thermal network model changes,
  // so that tools reading recorded telemetry can tell which model
  // produced it.
  uint8_t model_version = 12;

  // Silicon.
  float rds_on_25C_ohm = kNaN;
  // Fractional increase in Rds(on) per degree C above 25C.
  float rds_on_tempco_per_C = 0.0f;
  // Gate switch charge, Qsw = Qgs2 + Qgd, in coulombs.  The
  // voltage/current overlap time during a hard switching edge is
  // approximated as Qsw divided by the gate drive current.
  float gate_switch_charge_coulomb = kNaN;

  // Junction to the pad node, per FET.  At its simplest this is the
  // datasheet Rth(ch-c) plus the solder joint, with the time constant
  // from the transient thermal impedance curve.  Measured on a board,
  // it also takes in the fast spreading into the copper right under
  // the package, which heats in a fraction of a second and differs
  // between FETs whose drain pad is on a large copper pour (high side,
  // on the bus) and on a small one (low side, the phase node).  Too
  // slow a time constant here is what lets a short, heavy pulse run
  // the silicon far ahead of the estimate.
  std::array<float, 6> r_junction_pad_C_per_W = {
    kNaN, kNaN, kNaN, kNaN, kNaN, kNaN };
  std::array<float, 6> tau_junction_pad_s = {
    kNaN, kNaN, kNaN, kNaN, kNaN, kNaN };

  // Each pad's conductance to the copper at the thermistor.  Dominated
  // by board layout; measured.  Zero for a pad that only reaches it
  // through its neighbours.
  std::array<float, 6> g_pad_thermistor_W_per_K = {
    kNaN, kNaN, kNaN, kNaN, kNaN, kNaN };
  // Each pad's conductance to ambient, in still air.
  std::array<float, 6> g_pad_ambient_W_per_K = {
    kNaN, kNaN, kNaN, kNaN, kNaN, kNaN };
  // Heat capacity of the pad node, the same for every FET.
  float c_pad_J_per_K = kNaN;

  // The copper at the thermistor and the bulk of the board: their heat
  // capacities, the conductance between them and each one's to
  // ambient, and the lag of the thermistor reading behind its copper.
  float c_thermistor_J_per_K = kNaN;
  float c_board_J_per_K = kNaN;
  float g_thermistor_board_W_per_K = kNaN;
  float g_thermistor_ambient_W_per_K = kNaN;
  float g_board_ambient_W_per_K = kNaN;
  float thermistor_lag_s = kNaN;

  // The heat spreader on the pads: each pad's conductance to it, its
  // conductance to ambient, and its heat capacity.  A conductance of
  // zero means the board is only run without one.
  float g_pad_spreader_W_per_K = 0.0f;
  float g_spreader_ambient_W_per_K = 0.0f;
  float c_spreader_J_per_K = 0.0f;

  // Calibration multiplier on the switching loss estimate.  The
  // Qsw / I_gate approximation is only good to a factor of ~1.5, but
  // the ratio between gate drive settings is much better than that.
  float switching_loss_scale = 1.0f;

  // The rest of the switching loss, as energies per PWM cycle, all
  // measured on the fixture and zero when not:
  //
  //  * In the FET that hard switches, energies that do not depend on
  //    the gate drive: switching_energy_J_per_V_A * bus_V * |i| (an
  //    overlap that the gate current does not set) and
  //    switching_energy_J_per_A2 * i^2 (stray inductance, a current
  //    dependent switching time).
  //  * In the other FET of the leg, its body diode conducting through
  //    the dead time at both edges: 2 * deadtime_diode_V_s * |i|, the
  //    diode's forward drop times the dead time.  It does not depend
  //    on the bus voltage, so it shows against the PWM rate.
  //  * Per leg, whatever the current, shared equally by its two FETs:
  //    switching_energy_J + switching_energy_J_per_V * bus_V (output
  //    capacitance and gate charge).
  float switching_energy_J_per_V_A = 0.0f;
  float switching_energy_J_per_A2 = 0.0f;
  float deadtime_diode_V_s = 0.0f;
  float switching_energy_J = 0.0f;
  float switching_energy_J_per_V = 0.0f;

  // The hard switching FET's loss per edge, through the gate drive
  // and the first two energies above, grows with the PWM rate by
  // 1 + hard_switching_per_Hz * pwm_rate_hz.  On the x1 the hard
  // switching FETs read 10 to 20 percent hotter at 60 kHz than at 15
  // kHz against a loss proportional to the rate.
  float hard_switching_per_Hz = 0.0f;

  // Additional series resistance added to Rds(on) in every FET's
  // conduction term.  It absorbs whatever dissipates as I^2 near the
  // FET beyond the datasheet Rds(on) and the shunt: copper in the
  // phase path, a higher effective Rds(on), and so on.  One value
  // for the board; the data does not resolve differences between
  // FETs, and any real difference is absorbed into that FET's pad
  // conductance.  Measured; zero means not yet calibrated.
  float r_extra_ohm = 0.0f;

  // Conductances between pairs of pads (physical neighbours), the
  // first network_couplings of each array.  Fitted from fixture runs
  // that read every FET at once.
  static constexpr int kMaxCouplings = 8;
  uint8_t network_couplings = 0;
  std::array<uint8_t, kMaxCouplings> coupling_a = {};
  std::array<uint8_t, kMaxCouplings> coupling_b = {};
  std::array<float, kMaxCouplings> coupling_W_per_K = {};

  // Published read-only as the "fet_model_params" telemetry struct
  // so that tools can record which constants a board is using.
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(model_version));
    a->Visit(MJ_NVP(rds_on_25C_ohm));
    a->Visit(MJ_NVP(rds_on_tempco_per_C));
    a->Visit(MJ_NVP(gate_switch_charge_coulomb));
    a->Visit(MJ_NVP(r_junction_pad_C_per_W));
    a->Visit(MJ_NVP(tau_junction_pad_s));
    a->Visit(MJ_NVP(g_pad_thermistor_W_per_K));
    a->Visit(MJ_NVP(g_pad_ambient_W_per_K));
    a->Visit(MJ_NVP(c_pad_J_per_K));
    a->Visit(MJ_NVP(c_thermistor_J_per_K));
    a->Visit(MJ_NVP(c_board_J_per_K));
    a->Visit(MJ_NVP(g_thermistor_board_W_per_K));
    a->Visit(MJ_NVP(g_thermistor_ambient_W_per_K));
    a->Visit(MJ_NVP(g_board_ambient_W_per_K));
    a->Visit(MJ_NVP(thermistor_lag_s));
    a->Visit(MJ_NVP(g_pad_spreader_W_per_K));
    a->Visit(MJ_NVP(g_spreader_ambient_W_per_K));
    a->Visit(MJ_NVP(c_spreader_J_per_K));
    a->Visit(MJ_NVP(switching_loss_scale));
    a->Visit(MJ_NVP(switching_energy_J_per_V_A));
    a->Visit(MJ_NVP(switching_energy_J_per_A2));
    a->Visit(MJ_NVP(deadtime_diode_V_s));
    a->Visit(MJ_NVP(switching_energy_J));
    a->Visit(MJ_NVP(switching_energy_J_per_V));
    a->Visit(MJ_NVP(hard_switching_per_Hz));
    a->Visit(MJ_NVP(r_extra_ohm));
    a->Visit(MJ_NVP(network_couplings));
    a->Visit(MJ_NVP(coupling_a));
    a->Visit(MJ_NVP(coupling_b));
    a->Visit(MJ_NVP(coupling_W_per_K));
  }

  // The Rds(on) temperature feedback is only evaluated between these
  // temperatures.  Above the upper one the device is destroyed
  // anyway, and limiting it keeps the loop gain bounded for any
  // input.  The lower one is the bottom of the FETs' rated range.
  // With both, valid() can check that Rds(on) is never negative.
  static constexpr float kMinRdsTemp_C = -55.0f;
  static constexpr float kMaxRdsTemp_C = 200.0f;

  float rds_on_ohm(float junction_C) const {
    const float t = std::clamp(junction_C, kMinRdsTemp_C, kMaxRdsTemp_C);
    return rds_on_25C_ohm * (1.0f + rds_on_tempco_per_C * (t - 25.0f));
  }

  bool valid() const {
    const auto positive = [](float v) { return v > 0.0f && std::isfinite(v); };
    // A negative conductance would make the network unstable.
    const auto conductance = [](float v) { return v >= 0.0f && std::isfinite(v); };
    // A negative loss would let the estimate cool the board.
    const auto loss = [](float v) { return v >= 0.0f && std::isfinite(v); };
    if (!loss(rds_on_25C_ohm) ||
        !std::isfinite(rds_on_tempco_per_C) ||
        !loss(gate_switch_charge_coulomb) ||
        !loss(switching_loss_scale) ||
        !positive(c_pad_J_per_K) ||
        !conductance(r_extra_ohm) ||
        !loss(switching_energy_J_per_V_A) ||
        !loss(switching_energy_J_per_A2) ||
        !loss(deadtime_diode_V_s) ||
        !loss(switching_energy_J) ||
        !loss(switching_energy_J_per_V) ||
        !loss(hard_switching_per_Hz) ||
        !positive(c_thermistor_J_per_K) ||
        !positive(c_board_J_per_K) ||
        !positive(thermistor_lag_s) ||
        !conductance(g_thermistor_board_W_per_K) ||
        !conductance(g_thermistor_ambient_W_per_K) ||
        !conductance(g_board_ambient_W_per_K) ||
        !conductance(g_pad_spreader_W_per_K) ||
        !conductance(g_spreader_ambient_W_per_K)) {
      return false;
    }
    // Rds(on) is linear in the temperature, so it is never negative
    // if it is non-negative at both ends of its range.
    if (!(rds_on_ohm(kMinRdsTemp_C) >= 0.0f) ||
        !(rds_on_ohm(kMaxRdsTemp_C) >= 0.0f)) {
      return false;
    }
    if (has_spreader() && !positive(c_spreader_J_per_K)) { return false; }
    for (int i = 0; i < 6; i++) {
      if (!positive(r_junction_pad_C_per_W[i]) ||
          !positive(tau_junction_pad_s[i]) ||
          !conductance(g_pad_thermistor_W_per_K[i]) ||
          !conductance(g_pad_ambient_W_per_K[i])) {
        return false;
      }
    }
    if (network_couplings > kMaxCouplings) { return false; }
    for (int k = 0; k < network_couplings; k++) {
      if (coupling_a[k] >= 6 || coupling_b[k] >= 6 ||
          coupling_a[k] == coupling_b[k] ||
          !conductance(coupling_W_per_K[k])) {
        return false;
      }
    }
    // Every node must conduct to ambient, directly or through others,
    // or its temperature would grow without bound.  Nodes 0-5 are the
    // pads, 6 the thermistor's copper, 7 the board.  The spreader
    // is tied to every pad, so it then reaches ambient too.
    std::array<bool, 8> reached = {};
    for (int i = 0; i < 6; i++) {
      reached[i] = g_pad_ambient_W_per_K[i] > 0.0f;
    }
    reached[6] = g_thermistor_ambient_W_per_K > 0.0f;
    reached[7] = g_board_ambient_W_per_K > 0.0f;
    const auto join = [&](int a, int b, float g) {
      if (!(g > 0.0f)) { return; }
      const bool either = reached[a] || reached[b];
      reached[a] = either;
      reached[b] = either;
    };
    for (int pass = 0; pass < 8; pass++) {
      for (int i = 0; i < 6; i++) { join(i, 6, g_pad_thermistor_W_per_K[i]); }
      for (int k = 0; k < network_couplings; k++) {
        join(coupling_a[k], coupling_b[k], coupling_W_per_K[k]);
      }
      join(6, 7, g_thermistor_board_W_per_K);
    }
    for (const bool r : reached) {
      if (!r) { return false; }
    }
    return true;
  }

  bool has_spreader() const { return g_pad_spreader_W_per_K > 0.0f; }
};

/// All state of the estimator, published as the "fet_model"
/// telemetry struct.
struct FetThermalStatus {
  bool enabled = false;

  // Estimated dissipation in each FET.
  std::array<float, 6> loss_W = {};
  // Estimated junction temperature of each FET.
  std::array<float, 6> junction_C = {};
  // Estimated temperature of the copper under each FET, from the run
  // (with or without the spreader) that gives its junction estimate.
  std::array<float, 6> pad_C = {};
  // The modeled rise of the thermistor's reading over ambient, in the
  // run with the hottest junction.  The estimates are the measured
  // thermistor plus each node's rise over this.
  float reading_rise_C = 0.0f;

  float max_junction_C = 0.0f;

  // How far the hottest junction is through its derating band: 0 at
  // the start of derating, 1 at the fault temperature.  With a
  // margin of 0 or less there is no band, and this is -infinity
  // below the fault temperature and +infinity above it.  NaN if the
  // model is disabled or the fault temperature is NaN.
  float junction_derate_fraction = std::numeric_limits<float>::quiet_NaN();

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(enabled));
    a->Visit(MJ_NVP(loss_W));
    a->Visit(MJ_NVP(junction_C));
    a->Visit(MJ_NVP(pad_C));
    a->Visit(MJ_NVP(reading_rise_C));
    a->Visit(MJ_NVP(max_junction_C));
    a->Visit(MJ_NVP(junction_derate_fraction));
  }
};

/// Estimate the junction temperature of each FET in a three phase
/// bridge from the per-phase currents and the board thermistor.
///
/// This is intended to be run at a slow rate (~1kHz) outside of the
/// control ISR.  The fastest mode of any board's network is about
/// 4.5ms, on the moteus-x1, whose FET 0 pad is tied closely to the
/// thermistor's copper; on the other boards it is the 20ms thermistor
/// lag.  A 1ms explicit Euler step is within the stability limit of
/// twice that, and stays within 0.3C of an exact solution through
/// steps and pulses of loss.
class FetThermalModel {
 public:
  struct Inputs {
    float dt_s = 0.001f;
    float thermistor_C = 0.0f;
    float bus_V = 0.0f;
    float pwm_rate_hz = 0.0f;

    // Physical channel currents.  Positive is current flowing out of
    // the phase node into the motor.
    std::array<float, 3> phase_A = {};
    // High side on-fraction of each physical channel, 0-1.
    std::array<float, 3> duty = {};

    // If true, phase_A and duty are ignored, and each FET's loss is
    // instead its average over an electrical revolution of a
    // sinusoidal phase current with this amplitude.
    bool averaged = false;
    float current_amplitude_A = 0.0f;

    GateDriveCurrents gate;

    // The low side current sense shunt resistance.  It is in series
    // with the low side FET's source, so it dissipates i^2 * R while
    // that FET conducts, right next to its pad.  That heat is
    // attributed to the low side FET.
    float shunt_ohm = 0.0f;

    // The junction derating thresholds.
    float junction_fault_C = 0.0f;
    float junction_margin_C = 0.0f;

    // If true, the model (re)initializes with every junction and pad
    // at the start of derating, otherwise at the thermistor's reading.
    bool hot_start = true;

    // Above this electrical frequency the losses are averaged over a
    // revolution.  The phase currents are sampled once per update, so
    // near multiples of the update rate those samples would see a
    // nearly frozen electrical angle, and the model a locked rotor at
    // an arbitrary angle.  Well above the reciprocal of the junction
    // time constant the average is what the junction sees anyway, so
    // the two agree where they switch over.
    static constexpr float kAveragedAboveHz = 50.0f;

    // Fill phase_A and duty, which are per physical channel so that
    // each FET's position on the board is fixed, from servo_stats
    // cur1_A-cur3_A and the logical phase PWM (a, b, c).  Logical
    // phases b and c drive physical channels 3 and 2, or 2 and 3 with
    // phase_invert, and servo_stats has already swapped cur2_A and
    // cur3_A when phase_invert is set.  See ISR_CalculateCurrentState
    // and DoPwmControl.
    //
    // Above kAveragedAboveHz electrical, the averaged model is used
    // instead, with the amplitude from the d and q currents (which are
    // amplitude invariant).
    //
    // When the bridge is off the current sense amplifiers are
    // disabled and their readings are meaningless (they sit at the
    // full negative range), and there is no loss either.
    void SetPhases(bool bridge_on, bool phase_invert,
                   float cur1_A, float cur2_A, float cur3_A,
                   float pwm_a, float pwm_b, float pwm_c,
                   float electrical_hz, float d_A, float q_A) {
      averaged = false;
      current_amplitude_A = 0.0f;
      if (!bridge_on) {
        phase_A = {};
        duty = {};
        return;
      }
      if (std::abs(electrical_hz) > kAveragedAboveHz) {
        averaged = true;
        current_amplitude_A = std::sqrt(d_A * d_A + q_A * q_A);
      }
      phase_A = { cur1_A,
                  phase_invert ? cur3_A : cur2_A,
                  phase_invert ? cur2_A : cur3_A };
      duty = { pwm_a,
               phase_invert ? pwm_b : pwm_c,
               phase_invert ? pwm_c : pwm_b };
    }
  };

  void Update(const FetThermalParams& params,
              const Inputs& in,
              FetThermalStatus* out) {
    // The constants are fixed for the life of the firmware, so they
    // are checked when the model (re)initializes rather than on every
    // step, where the checks cost as much as the thermal network.
    if (!initialized_) {
      valid_ = params.valid();
      if (valid_) {
        // With the constants fixed, the divisions of the junction
        // node are done once here rather than every step.
        for (int i = 0; i < 6; i++) {
          g_junction_pad_[i] = 1.0f / params.r_junction_pad_C_per_W[i];
          junction_rate_[i] =
              params.r_junction_pad_C_per_W[i] / params.tau_junction_pad_s[i];
        }
      }
    }
    if (!valid_ || !std::isfinite(in.thermistor_C)) {
      *out = {};
      initialized_ = false;
      return;
    }

    const int runs = params.has_spreader() ? 2 : 1;

    if (!initialized_) {
      // Start hot, at the start of derating (see FetThermalParams):
      // every junction and pad that far above the thermistor's
      // reading, and everything else at ambient.  Or, without a hot
      // start, everything at ambient.
      const float start_C = in.junction_fault_C - std::max(0.0f, in.junction_margin_C);
      const float rise = (in.hot_start && std::isfinite(start_C)) ?
          std::max(0.0f, start_C - in.thermistor_C) : 0.0f;
      for (auto& st : state_) {
        st = {};
        st.junction_rise.fill(rise);
        st.pad_rise.fill(rise);
      }
      initialized_ = true;
    }

    // Each FET's estimate, the hotter of the runs, from the state at
    // the start of the step; the losses' Rds(on) uses it.
    std::array<float, 6> junction_C = {};
    for (int i = 0; i < 6; i++) {
      junction_C[i] = junction(in, 0, i);
      if (runs > 1) { junction_C[i] = std::max(junction_C[i], junction(in, 1, i)); }
    }

    const float bus_V = std::isfinite(in.bus_V) ? in.bus_V : 0.0f;

    // First, the losses.
    //
    // When averaged over a revolution, the mean of i^2 is I^2 / 2,
    // and each FET conducts for half of it.  Each FET hard switches
    // for the half revolution where the current has its sign, during
    // which the mean of |i| is 2 I / pi and of i^2 is I^2 / 2, and
    // its body diode conducts in the dead time for the other half.
    // Only Rds(on) differs between FETs, so the rest is computed once.
    //
    // A leg that switches loses this much in each of its FETs
    // whatever the current.
    const float leg_W = 0.5f * in.pwm_rate_hz *
        (params.switching_energy_J + params.switching_energy_J_per_V * bus_V);
    float averaged_i2 = 0.0f;
    float averaged_hs_W = 0.0f;
    float averaged_ls_W = 0.0f;
    if (in.averaged) {
      const float i = in.current_amplitude_A;
      averaged_i2 = 0.25f * i * i;
      const float other_W = DiodeLoss(params, in.pwm_rate_hz, i / kPi) + leg_W;
      averaged_hs_W = SwitchingLoss(
          params, bus_V, in.pwm_rate_hz, i / kPi, averaged_i2,
          in.gate.hs_source_A, in.gate.hs_sink_A) + other_W;
      averaged_ls_W = SwitchingLoss(
          params, bus_V, in.pwm_rate_hz, i / kPi, averaged_i2,
          in.gate.ls_source_A, in.gate.ls_sink_A) + other_W;
    }

    std::array<float, 6> loss_W = {};
    for (int channel = 0; channel < 3; channel++) {
      const int hs = channel * 2;
      const int ls = hs + 1;
      const float rds_hs =
          params.rds_on_ohm(junction_C[hs]) + params.r_extra_ohm;
      const float rds_ls =
          params.rds_on_ohm(junction_C[ls]) + params.r_extra_ohm + in.shunt_ohm;

      if (in.averaged) {
        loss_W[hs] = averaged_i2 * rds_hs + averaged_hs_W;
        loss_W[ls] = averaged_i2 * rds_ls + averaged_ls_W;
        continue;
      }

      const float i = in.phase_A[channel];
      const float i2 = i * i;
      const float duty = std::max(0.0f, std::min(1.0f, in.duty[channel]));

      loss_W[hs] = i2 * rds_hs * duty;
      loss_W[ls] = i2 * rds_ls * (1.0f - duty);

      // Only one FET in each leg hard switches, the other has its
      // body diode conducting during the dead time and thus switches
      // at zero voltage, losing only the diode's conduction.  A leg
      // held fully on or off (as when braking) does not switch at
      // all.
      if (duty > 0.0f && duty < 1.0f) {
        const bool hs_hard = (i >= 0.0f);
        const float abs_i = std::abs(i);
        loss_W[hs_hard ? hs : ls] += SwitchingLoss(
            params, bus_V, in.pwm_rate_hz, abs_i, i2,
            hs_hard ? in.gate.hs_source_A : in.gate.ls_source_A,
            hs_hard ? in.gate.hs_sink_A : in.gate.ls_sink_A);
        loss_W[hs_hard ? ls : hs] += DiodeLoss(params, in.pwm_rate_hz, abs_i);
        loss_W[hs] += leg_W;
        loss_W[ls] += leg_W;
      }
    }

    // A non-finite loss, from a NaN current sample say, would poison
    // the state for good, since the bounds in Step() do not catch NaN.
    // Treat it like a missing thermistor: no estimate for this step,
    // and a hot start once the inputs are good again.
    for (const float w : loss_W) {
      if (!std::isfinite(w)) {
        *out = {};
        initialized_ = false;
        return;
      }
    }

    for (int r = 0; r < runs; r++) {
      Step(params, r == 1, in.dt_s, loss_W, &state_[r]);
    }

    float max_junction_C = -std::numeric_limits<float>::infinity();
    int hottest = 0;
    for (int i = 0; i < 6; i++) {
      int from = 0;
      out->junction_C[i] = junction(in, 0, i);
      if (runs > 1 && junction(in, 1, i) > out->junction_C[i]) {
        out->junction_C[i] = junction(in, 1, i);
        from = 1;
      }
      const auto& st = state_[from];
      out->pad_C[i] = in.thermistor_C + st.pad_rise[i] - st.reading_rise;
      if (out->junction_C[i] > max_junction_C) {
        max_junction_C = out->junction_C[i];
        hottest = from;
      }
    }

    out->enabled = true;
    out->loss_W = loss_W;
    out->reading_rise_C = state_[hottest].reading_rise;
    out->max_junction_C = max_junction_C;

    out->junction_derate_fraction = DerateFraction(
        max_junction_C, in.junction_fault_C, in.junction_margin_C);
  }

  // How far `temp_C` is through the derating band that ends at
  // `fault_C`: 0 at `fault_C - margin_C`, 1 at `fault_C`.  With a
  // margin of 0 or less there is no band, only a cutoff at `fault_C`,
  // so the result is -infinity below it and +infinity above.
  static float DerateFraction(float temp_C, float fault_C, float margin_C) {
    if (margin_C > 0.0f) {
      return (temp_C - (fault_C - margin_C)) / margin_C;
    }
    if (std::isnan(temp_C) || std::isnan(fault_C)) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return (temp_C > fault_C) ?
        std::numeric_limits<float>::infinity() :
        -std::numeric_limits<float>::infinity();
  }

 private:
  static constexpr float kMaxTemp_C = 1000.0f;

  // The loss of hard switching `abs_current_A` at every PWM edge, in
  // the FET that does: the voltage/current overlap through the gate
  // drive, with its time approximated as Qsw divided by the gate
  // drive current, and the energies that do not depend on the gate
  // drive, the second with `i2_A2` the square of the current, all
  // growing with the PWM rate by hard_switching_per_Hz.
  static float SwitchingLoss(const FetThermalParams& params,
                             float bus_V, float pwm_rate_hz,
                             float abs_current_A, float i2_A2,
                             float source_A, float sink_A) {
    const float rate_W = pwm_rate_hz *
        (1.0f + params.hard_switching_per_Hz * pwm_rate_hz);
    const float other_W = rate_W *
        (params.switching_energy_J_per_V_A * bus_V * abs_current_A +
         params.switching_energy_J_per_A2 * i2_A2);
    if (!(source_A > 0.0f && sink_A > 0.0f)) { return other_W; }
    const float overlap_s = params.gate_switch_charge_coulomb *
        (1.0f / source_A + 1.0f / sink_A);
    return 0.5f * bus_V * abs_current_A * overlap_s * rate_W *
        params.switching_loss_scale + other_W;
  }

  // The body diode of the FET that does not hard switch, conducting
  // `abs_current_A` through the dead time at both PWM edges.
  static float DiodeLoss(const FetThermalParams& params,
                         float pwm_rate_hz, float abs_current_A) {
    return 2.0f * pwm_rate_hz * params.deadtime_diode_V_s * abs_current_A;
  }

  // The state of one run of the board, as rises over ambient.
  struct State {
    std::array<float, 6> junction_rise = {};
    std::array<float, 6> pad_rise = {};
    float thermistor_rise = 0.0f;
    float board_rise = 0.0f;
    float spreader_rise = 0.0f;
    float reading_rise = 0.0f;
  };

  // Propagate one run, every flow from the state at the start of the
  // step.
  void Step(const FetThermalParams& params, bool spreader, float dt_s,
            const std::array<float, 6>& loss_W, State* st) const {
    // First the heat leaving each pad other than to its junction.
    std::array<float, 6> q_pad_out = {};
    float q_into_thermistor = 0.0f;
    float q_into_spreader = 0.0f;
    for (int i = 0; i < 6; i++) {
      const float q_th =
          params.g_pad_thermistor_W_per_K[i] * (st->pad_rise[i] - st->thermistor_rise);
      q_into_thermistor += q_th;
      q_pad_out[i] = q_th + params.g_pad_ambient_W_per_K[i] * st->pad_rise[i];
      if (spreader) {
        const float q_sp =
            params.g_pad_spreader_W_per_K * (st->pad_rise[i] - st->spreader_rise);
        q_into_spreader += q_sp;
        q_pad_out[i] += q_sp;
      }
    }
    for (int k = 0; k < params.network_couplings; k++) {
      const int a = params.coupling_a[k];
      const int b = params.coupling_b[k];
      const float q = params.coupling_W_per_K[k] * (st->pad_rise[a] - st->pad_rise[b]);
      q_pad_out[a] += q;
      q_pad_out[b] -= q;
    }
    const float q_board =
        params.g_thermistor_board_W_per_K * (st->thermistor_rise - st->board_rise);

    const float pad_step = dt_s / params.c_pad_J_per_K;
    for (int i = 0; i < 6; i++) {
      const float q_junction_pad =
          (st->junction_rise[i] - st->pad_rise[i]) * g_junction_pad_[i];

      st->junction_rise[i] +=
          dt_s * junction_rate_[i] * (loss_W[i] - q_junction_pad);
      st->pad_rise[i] += pad_step * (q_junction_pad - q_pad_out[i]);
    }
    const float reading_step =
        dt_s / params.thermistor_lag_s * (st->thermistor_rise - st->reading_rise);
    st->thermistor_rise += dt_s / params.c_thermistor_J_per_K * (
        q_into_thermistor - q_board -
        params.g_thermistor_ambient_W_per_K * st->thermistor_rise);
    st->board_rise += dt_s / params.c_board_J_per_K * (
        q_board - params.g_board_ambient_W_per_K * st->board_rise);
    if (spreader) {
      st->spreader_rise += dt_s / params.c_spreader_J_per_K * (
          q_into_spreader - params.g_spreader_ambient_W_per_K * st->spreader_rise);
    }
    st->reading_rise += reading_step;

    // Bound the state so that a bogus input (for instance current
    // sense readings while the driver is disabled) can never make the
    // estimate diverge or wrap.
    for (int i = 0; i < 6; i++) {
      st->junction_rise[i] = std::clamp(st->junction_rise[i], -kMaxTemp_C, kMaxTemp_C);
      st->pad_rise[i] = std::clamp(st->pad_rise[i], -kMaxTemp_C, kMaxTemp_C);
    }
    st->thermistor_rise = std::clamp(st->thermistor_rise, -kMaxTemp_C, kMaxTemp_C);
    st->board_rise = std::clamp(st->board_rise, -kMaxTemp_C, kMaxTemp_C);
    st->spreader_rise = std::clamp(st->spreader_rise, -kMaxTemp_C, kMaxTemp_C);
    st->reading_rise = std::clamp(st->reading_rise, -kMaxTemp_C, kMaxTemp_C);
  }

  // A junction's estimate from one run: the measured thermistor plus
  // its modeled rise over the modeled thermistor reading, but never
  // less than the thermistor (see FetThermalParams).
  float junction(const Inputs& in, int run, int i) const {
    const float rise = state_[run].junction_rise[i] - state_[run].reading_rise;
    return in.thermistor_C + std::max(0.0f, rise);
  }


  bool initialized_ = false;
  // Whether the constants are usable, as of the last initialization.
  bool valid_ = false;
  // 1 / r_junction_pad and 1 / junction heat capacity, per FET.
  std::array<float, 6> g_junction_pad_ = {};
  std::array<float, 6> junction_rate_ = {};
  // Without, then with the spreader.
  std::array<State, 2> state_ = {};
};

}
