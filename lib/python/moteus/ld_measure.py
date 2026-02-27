# Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""L_d saturation measurement protocol (device-coupled).

Implements the voltage-mode V_d injection sweep used to map out the
D-axis inductance as a function of d_A.  The motor is spun open-loop
with `d vdq 0 V_q`, then at each (speed, d_A) operating point an
integral V_d controller nudges the measured d_A to the target and
steady-state samples of y = q_V - R*q_A and omega_e are collected.

The caller supplies a duck-typed `device` exposing a narrow slice of
the `moteus_tool.Stream` API:

    async def command(cmd_str) -> Any
    async def read_servo_stats() -> Any        # .velocity, .d_A, .q_A
    async def read_data(channel) -> Any        # "servo_control": .q_V
    async def read_config_double(name) -> float
    async def is_config_supported(name) -> bool

Pure-math analysis lives in ``ld_saturation``; this module only runs
the device-facing protocol.
"""

import asyncio
import math
import time

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from moteus import ld_saturation


@dataclass(frozen=True)
class LdSweepParams:
    """Inputs needed to plan and run a full L_d sweep."""

    winding_resistance: float      # ohms
    unwrapped_position_scale: float
    pp: float                      # pole pairs (motor.poles / 2)
    v_per_hz: float                # from Kv calibration
    kv_cal_voltage: float          # from Kv calibration
    motor_power: float             # watts (--cal-motor-power)
    power_factor: float = 6.0      # --cal-ld-power-factor
    voltage_factor: float = 1.0    # --cal-ld-voltage-factor
    # Tunables (hard-coded in the original code):
    n_levels: int = 6
    n_speeds: int = 5
    vd_ki: float = 0.5
    vq_speed_ki: float = 2.0
    settle_time: float = 4.0
    sample_time: float = 0.5
    min_omega_e: float = 40.0


@dataclass(frozen=True)
class LdSweepPlan:
    """Derived sweep levels and limits from an LdSweepParams."""

    all_d_A: List[float]           # [0.0] + descending d_A targets
    V_q_levels: List[float]        # ascending
    startup_V_q: float
    vd_max: float
    max_speed_hz: float
    max_d_A_abs: float             # |most negative d_A|, for printing


def compute_sweep_plan(p: LdSweepParams) -> LdSweepPlan:
    """Pure derivation of the sweep levels from ``p``.

    Replaces the sweep-parameter math that used to be duplicated
    between ``moteus_tool.Stream.measure_ld_saturation`` and
    ``utils/measure_ld_data.py``.
    """
    power_limit = p.motor_power * p.power_factor
    max_d_A = math.sqrt(power_limit / (1.5 * p.winding_resistance))

    min_d_A = max_d_A * 0.2
    d_A_targets = [
        -min_d_A - i * (max_d_A - min_d_A) / max(p.n_levels - 1, 1)
        for i in range(p.n_levels)]
    all_d_A = [0.0] + d_A_targets

    max_vq_factor = p.voltage_factor
    min_vq_factor = max(0.5, max_vq_factor * 0.25)
    V_q_factors = [
        min_vq_factor + i * (max_vq_factor - min_vq_factor) /
        max(p.n_speeds - 1, 1)
        for i in range(p.n_speeds)]
    V_q_levels = [p.kv_cal_voltage * f for f in V_q_factors]

    vd_max = max(V_q_levels[-1] * 3.0,
                 p.winding_resistance * max_d_A * 5.0, 5.0)
    max_speed_hz = V_q_levels[-1] / p.v_per_hz
    startup_V_q = V_q_levels[-1]

    return LdSweepPlan(
        all_d_A=all_d_A,
        V_q_levels=V_q_levels,
        startup_V_q=startup_V_q,
        vd_max=vd_max,
        max_speed_hz=max_speed_hz,
        max_d_A_abs=max_d_A,
    )


def _speed_and_omega(stats, params: LdSweepParams) -> Tuple[float, float]:
    """Convert servo_stats velocity to (speed_hz, omega_e)."""
    speed_hz = stats.velocity / params.unwrapped_position_scale
    omega_e = speed_hz * 2.0 * math.pi * params.pp
    return speed_hz, omega_e


def _pi_update(V_d: float, V_q_cmd: float, d_A_meas: float,
               d_A_target: float, speed_hz: float, dt: float,
               params: LdSweepParams, plan: LdSweepPlan,
               V_q: float, V_q_min: float) -> Tuple[float, float]:
    """Apply one PI-update step to V_d (d_A tracking) and V_q_cmd
    (speed regulation).  Shared by the settle and sample loops in
    _ld_measure_point."""
    speed_err = abs(speed_hz) - plan.max_speed_hz
    V_q_cmd -= params.vq_speed_ki * speed_err * dt
    V_q_cmd = max(V_q_min, min(V_q, V_q_cmd))

    d_A_err = d_A_target - d_A_meas
    V_d += params.vd_ki * d_A_err * dt
    V_d = max(-plan.vd_max, min(plan.vd_max, V_d))
    return V_d, V_q_cmd


async def _ld_startup(device, params: LdSweepParams,
                      plan: LdSweepPlan) -> Optional[float]:
    """Ramp-start the motor, retrying up to 3 times for cogging.

    Returns speed_hz on success, None on failure.
    """
    startup_V_q = plan.startup_V_q
    omega_e = 0.0
    speed_hz = 0.0
    for _attempt in range(3):
        spinup_start = time.time()
        while time.time() - spinup_start < 2.0:
            elapsed = time.time() - spinup_start
            frac = min(1.0, elapsed / 1.5)
            await device.command(f"d vdq 0 {startup_V_q * frac:.4f}")
            await asyncio.sleep(0.05)
        await device.command(f"d vdq 0 {startup_V_q:.4f}")
        await asyncio.sleep(0.5)

        stats = await device.read_servo_stats()
        speed_hz, omega_e = _speed_and_omega(stats, params)
        if abs(omega_e) > params.min_omega_e:
            break
        await device.command("d stop")
        await asyncio.sleep(1.0)

    if abs(omega_e) < params.min_omega_e:
        print(f"WARNING: Motor not spinning after 3 attempts "
              f"(omega_e={omega_e:.1f})")
        await device.command("d stop")
        return None

    return speed_hz


async def _ld_measure_point(device, d_A_target: float, V_q: float,
                            V_q_min: float, params: LdSweepParams,
                            plan: LdSweepPlan):
    """Measure a single (d_A, V_q) operating point.

    Runs a settle loop with integral V_d controller to converge
    d_A, then samples omega_e and y at steady state.

    Returns (omega_median, y_median), 'stall', 'saturated', or None.
    """
    R = params.winding_resistance

    V_d = R * d_A_target if d_A_target != 0.0 else 0.0
    V_q_cmd = V_q
    last_time = time.time()
    speed_hz = 0.0
    d_A_meas = 0.0

    # Settle: iterate V_d until d_A converges.
    settle_start = time.time()
    while time.time() - settle_start < params.settle_time:
        now = time.time()
        dt = min(now - last_time, 0.1)
        last_time = now

        await device.command(f"d vdq {V_d:.4f} {V_q_cmd:.4f}")
        stats = await device.read_servo_stats()
        speed_hz, _omega = _speed_and_omega(stats, params)
        d_A_meas = stats.d_A

        V_d, V_q_cmd = _pi_update(
            V_d, V_q_cmd, d_A_meas, d_A_target, speed_hz, dt,
            params, plan, V_q, V_q_min)

    # Check stall.
    omega_now = speed_hz * 2.0 * math.pi * params.pp
    if abs(omega_now) < params.min_omega_e:
        print(f"    STALL at d_A={d_A_target:.1f}"
              f" (V_q={V_q:.2f}, V_d={V_d:.2f}, speed={speed_hz:.1f} Hz)")
        return 'stall'

    # Check V_d saturation.
    if (abs(abs(V_d) - plan.vd_max) < 0.01
            and abs(d_A_meas - d_A_target) > 1.0):
        return 'saturated'

    # Sample at steady state.
    omega_samples: List[float] = []
    y_samples: List[float] = []
    sample_start = time.time()
    while time.time() - sample_start < params.sample_time:
        now = time.time()
        dt = min(now - last_time, 0.1)
        last_time = now

        await device.command(f"d vdq {V_d:.4f} {V_q_cmd:.4f}")
        ctrl = await device.read_data("servo_control")
        stats = await device.read_servo_stats()

        speed_hz, omega_e = _speed_and_omega(stats, params)
        y = ctrl.q_V - R * stats.q_A

        omega_samples.append(omega_e)
        y_samples.append(y)

        d_A_meas = stats.d_A
        V_d, V_q_cmd = _pi_update(
            V_d, V_q_cmd, d_A_meas, d_A_target, speed_hz, dt,
            params, plan, V_q, V_q_min)

    if len(omega_samples) >= 3:
        med_omega = ld_saturation.median(omega_samples)
        med_y = ld_saturation.median(y_samples)
        if abs(med_omega) > params.min_omega_e:
            return (med_omega, med_y)

    return None


async def _ld_main_sweep(device, params: LdSweepParams,
                         plan: LdSweepPlan,
                         avg_data: Dict[float, List[Tuple[float, float]]]):
    """Sweep V_q and d_A levels, collecting steady-state data."""
    n_speeds = len(plan.V_q_levels)

    for vf_idx, V_q in enumerate(plan.V_q_levels):
        approx_omega = (V_q / params.v_per_hz) * 2.0 * math.pi * params.pp
        if abs(approx_omega) < params.min_omega_e:
            continue

        await device.command(f"d vdq 0 {V_q:.4f}")
        await asyncio.sleep(2.0)
        stats = await device.read_servo_stats()
        speed_hz, omega_e = _speed_and_omega(stats, params)

        if abs(omega_e) < params.min_omega_e:
            await device.command(f"d vdq 0 {plan.startup_V_q:.4f}")
            await asyncio.sleep(1.0)
            continue

        print(f"  speed {vf_idx + 1}/{n_speeds}: "
              f"V_q={V_q:.2f} speed={speed_hz:.1f} Hz")

        for d_A_target in plan.all_d_A:
            r = await _ld_measure_point(
                device, d_A_target, V_q, plan.V_q_levels[0],
                params, plan)
            if r == 'stall' or r == 'saturated':
                break
            if r is not None:
                avg_data[d_A_target].append(r)


async def _ld_supplemental_pass(
        device, params: LdSweepParams, plan: LdSweepPlan,
        avg_data: Dict[float, List[Tuple[float, float]]]):
    """Add low-speed data for levels with compressed omega spread."""
    OMEGA_SPREAD_MIN = 1.5
    needs_supplement = []
    for d_A in plan.all_d_A:
        pts = avg_data[d_A]
        if len(pts) < 3:
            continue
        omegas = [p[0] for p in pts]
        if (min(omegas) > 0
                and max(omegas) / min(omegas) < OMEGA_SPREAD_MIN):
            needs_supplement.append(d_A)

    if not needs_supplement:
        return

    print(f"  supplementing {len(needs_supplement)} levels "
          f"with low-speed data")
    supp_V_q = [plan.V_q_levels[0] * f for f in [0.15, 0.30, 0.50]]
    supp_V_q = [v for v in supp_V_q if v >= 0.05]

    for V_q in supp_V_q:
        for d_A_target in needs_supplement:
            r = await _ld_measure_point(
                device, d_A_target, V_q, plan.V_q_levels[0] * 0.1,
                params, plan)
            if r is not None and r not in ('stall', 'saturated'):
                avg_data[d_A_target].append(r)


async def collect_sweep_data(
        device, params: LdSweepParams,
        plan: Optional[LdSweepPlan] = None,
) -> Dict[float, List[Tuple[float, float]]]:
    """Run startup + main sweep + supplemental pass.

    Manages ``servopos.position_{min,max}`` via try/finally so the
    device's configured limits are restored even if the sweep aborts.

    Returns ``avg_data`` mapping d_A -> list of (omega_e, y) points.
    """
    if plan is None:
        plan = compute_sweep_plan(params)

    avg_data: Dict[float, List[Tuple[float, float]]] = {
        d_A: [] for d_A in plan.all_d_A}

    original_position_min = await device.read_config_double(
        "servopos.position_min")
    original_position_max = await device.read_config_double(
        "servopos.position_max")
    await device.command("conf set servopos.position_min NaN")
    await device.command("conf set servopos.position_max NaN")

    try:
        speed_hz = await _ld_startup(device, params, plan)
        if speed_hz is None:
            return avg_data
        print(f"  motor spinning: V_q={plan.startup_V_q:.2f} "
              f"speed={speed_hz:.1f} Hz")

        await _ld_main_sweep(device, params, plan, avg_data)
        await _ld_supplemental_pass(device, params, plan, avg_data)
    finally:
        await device.command("d stop")
        await asyncio.sleep(0.3)
        await device.command(
            f"conf set servopos.position_min {original_position_min}")
        await device.command(
            f"conf set servopos.position_max {original_position_max}")

    return avg_data


async def measure_and_fit(
        device, params: LdSweepParams,
        motor_poles: int,
) -> Optional[ld_saturation.LdFitDetails]:
    """plan -> collect -> analyze_detailed.

    Returns the structured fit result, or None if the sweep failed
    to produce enough data or the fit was singular.  The caller is
    responsible for writing the result to device config.
    """
    plan = compute_sweep_plan(params)

    print(f"\nMeasuring L_d vs d_A (voltage-mode method)")
    print(f"  max |d_A| = {plan.max_d_A_abs:.1f} A, "
          f"{len(plan.all_d_A) - 1} levels + d_A=0 ref, "
          f"{params.n_speeds} speeds")
    print(f"  R = {params.winding_resistance:.4f} ohm, "
          f"poles = {motor_poles}")

    avg_data = await collect_sweep_data(device, params, plan)

    return ld_saturation.analyze_detailed(avg_data, {
        'R': params.winding_resistance,
        'poles': motor_poles,
    })
