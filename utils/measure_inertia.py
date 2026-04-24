#!/usr/bin/python3 -B

# Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

"""Measure the moment of inertia of a system connected to a moteus controller.

Improvements over the original:
  - Bidirectional torque sweeps (CW + CCW) so friction cancels in regression.
  - Per-trace acceleration via a sliding-window best-R² segment finder:
    a fixed-width window (45 % of trace) is slid across every trace and the
    position that maximises R² of a linear fit to v(t) is chosen.  This
    automatically skips the planetary-gear backlash transient at the start
    without any threshold tuning.
  - Single global  τ = J·α + τ_friction  OLS regression across all traces.
  - Bootstrap 95 % confidence interval on J.
  - servo.pid_dq.max_desired_rate is restored via 'conf load' after the test.
  - Matplotlib summary: velocity traces with best window highlighted, plus
    global regression scatter with fit line and CI band.
"""

import argparse
import asyncio
import math
import moteus
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


# ── helpers ───────────────────────────────────────────────────────────────────

async def collect_trace(c, torque, count, sleep_s=0.001):
    """Apply constant feedforward torque for *count* steps.

    Returns list of (timestamp_s, velocity_rev_s).
    """
    trace = []
    for _ in range(count):
        now = time.time()
        result = await c.set_position(
            position=math.nan,
            kp_scale=0.0,
            kd_scale=0.0,
            ilimit_scale=0.0,
            feedforward_torque=torque,
            ignore_position_bounds=True,
            query=True,
        )
        vel = result.values[moteus.Register.VELOCITY]
        trace.append((now, vel))
        await asyncio.sleep(sleep_s)
    return trace


async def wait_for_stop(c, vel_threshold=0.1, settle_s=1.5):
    """Brake and block until motor velocity drops below threshold."""
    finish_time = None
    while True:
        now  = time.time()
        data = await c.set_brake(query=True)
        await asyncio.sleep(0.001)
        if abs(data.values[moteus.Register.VELOCITY]) < vel_threshold:
            if finish_time is None:
                finish_time = now + settle_s
        else:
            finish_time = None
        if finish_time and now > finish_time:
            return data


def _linear_r2(ts, vs):
    """R² of a linear fit to (ts, vs). Returns (alpha, r2)."""
    coeffs = np.polyfit(ts, vs, 1)
    vs_fit = np.polyval(coeffs, ts)
    ss_res = float(np.sum((vs - vs_fit) ** 2))
    ss_tot = float(np.sum((vs - vs.mean()) ** 2))
    r2     = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    return float(coeffs[0]), r2


def regression_accel(trace, window_frac=0.50, min_samples=6):
    """Find the most linear segment of v(t) via a sliding window.

    Strategy
    --------
    A window of `window_frac` × N samples slides one step at a time across
    the entire trace.  At each position the R² of a linear fit to v(t) is
    computed.  The position that maximises R² is chosen as the stable
    acceleration window — no velocity threshold or stiction sigma required.

    The window width is wide enough to average out noise but narrow enough
    that it cannot straddle both the backlash transient and the clean
    acceleration phase simultaneously.

    Returns (alpha_rev_s2, r_squared, (i_lo, i_hi)) or (None, None, None).
    """
    n      = len(trace)
    w      = max(min_samples, int(round(window_frac * n)))
    t0_abs = trace[0][0]

    if n < w:
        return None, None, None

    best_r2   = -np.inf
    best_i    = 0
    best_alpha = 0.0

    ts_all = np.array([t - t0_abs for t, _ in trace])
    vs_all = np.array([v          for _, v in trace])

    for i in range(n - w + 1):
        ts_seg = ts_all[i : i + w]
        vs_seg = vs_all[i : i + w]
        alpha, r2 = _linear_r2(ts_seg, vs_seg)
        if r2 > best_r2:
            best_r2    = r2
            best_i     = i
            best_alpha = alpha

    i_lo = best_i
    i_hi = best_i + w
    return best_alpha, best_r2, (i_lo, i_hi)


# ── main ──────────────────────────────────────────────────────────────────────

async def main():
    parser = argparse.ArgumentParser()
    moteus.make_transport_args(parser)

    parser.add_argument('--target',      '-t', default=1,     type=int,
                        help='Controller ID to target')
    parser.add_argument('--scale',             default=1.3,   type=float,
                        help='Torque multiplier between steps')
    parser.add_argument('--count',             default=30,    type=int,
                        help='CAN frames per torque step')
    parser.add_argument('--min-vel-std',       default=0.003, type=float,
                        help='Minimum velocity noise stddev (rev/s) to prevent false positives')
    parser.add_argument('--torque-limit',      default=10.0,  type=float,
                        help='Maximum torque magnitude [N·m]')

    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)
    c = moteus.Controller(id=args.target, transport=transport)
    s = moteus.Stream(c)

    # ── check PLL bandwidth ───────────────────────────────────────────────────
    pll_hz = float(await s.command(
        b'conf get motor_position.sources.0.pll_filter_hz',
        allow_any_response=True,
    ))
    if pll_hz < 400:
        raise RuntimeError(
            f'Controller must be calibrated with ≥ 400 Hz BW; measured {pll_hz} Hz'
        )

    # ── override dI/dt for measurement ─────────────────────
    await s.command(b'conf set servo.pid_dq.max_desired_rate 10000000')

    try:
        await run_test(args, c)
    finally:
        # 'conf load' discards all in-RAM conf changes made since the last
        # persistent save, restoring the controller to its stored configuration.
        print("\nRestoring controller config via 'conf load' …")
        await c.set_stop()
        await s.command(b'conf load')


async def run_test(args, c):
    await c.set_stop()
    await asyncio.sleep(1.0)

    # ── baseline velocity noise ───────────────────────────────────────────────
    vel_samples = [
        (await c.query()).values[moteus.Register.VELOCITY]
        for _ in range(50)
    ]
    velocity_std = max(args.min_vel_std, float(np.std(vel_samples)))
    print(f"Velocity noise stddev = {velocity_std:.5f} rev/s\n")

    scale_threshold = 0.5 * (args.scale - 1) + 1

    # ── bidirectional torque sweep ────────────────────────────────────────────
    all_results = []

    for direction in (+1, -1):
        dir_label = "CW " if direction > 0 else "CCW"
        print(f"{'─' * 62}")
        print(f"  {dir_label} sweep")
        print(f"{'─' * 62}")

        torque       = 0.01 * args.torque_limit
        last_end_vel = None

        while True:
            signed_torque = direction * torque
            print(f"  τ = {signed_torque:+7.4f} N·m  ", end='', flush=True)

            await c.set_stop()
            trace     = await collect_trace(c, signed_torque, args.count)
            stop_data = await wait_for_stop(c)

            vels      = [v for _, v in trace]
            delta_vel = max(vels) - min(vels)
            end_vel   = abs(vels[-1])
            fault     = stop_data.values[moteus.Register.MODE] == 1

            # ── termination conditions ────────────────────────────────────────
            stop_reason = None
            if fault:
                stop_reason = f"fault {stop_data.values[moteus.Register.FAULT]}"
            elif delta_vel > 4000 * velocity_std:
                stop_reason = "delta_v >> noise floor"
            elif (last_end_vel is not None
                  and delta_vel > 500 * velocity_std
                  and end_vel < scale_threshold * last_end_vel):
                stop_reason = "diminishing returns"

            # ── best-linear-segment regression ───────────────────────────────
            alpha, r2, win = regression_accel(trace)

            if alpha is not None and abs(alpha) > 10 * velocity_std:
                pct = 100 * (win[1] - win[0]) / len(trace)
                all_results.append({
                    "torque":    signed_torque,
                    "direction": dir_label.strip(),
                    "trace":     trace,
                    "win":       win,
                    "alpha_rev": alpha,
                    "alpha_rad": alpha * 2.0 * math.pi,
                    "r2":        r2,
                })
                print(
                    f"win=[{win[0]}:{win[1]}] ({pct:.0f}%)  "
                    f"α={alpha:+8.3f} rev/s²  R²={r2:.4f}",
                    end='',
                )
            else:
                print(f"  skip: α too small or fit failed (α={alpha})", end='')

            if stop_reason:
                print(f"  → {stop_reason}")
                break
            else:
                print()

            last_end_vel = end_vel
            torque      *= args.scale
            if torque > args.torque_limit:
                print(f"  Reached torque limit ({args.torque_limit} N·m)")
                break

    print()

    # ── global OLS:  τ = J·α + τ_friction ────────────────────────────────────
    if len(all_results) < 2:
        print("Insufficient data for global regression.")
        return

    torques    = np.array([r["torque"]    for r in all_results])
    alphas_rad = np.array([r["alpha_rad"] for r in all_results])

    A = np.column_stack([alphas_rad, np.ones(len(alphas_rad))])
    (J_est, tau_friction), *_ = np.linalg.lstsq(A, torques, rcond=None)

    tau_pred  = J_est * alphas_rad + tau_friction
    ss_res    = float(np.sum((torques - tau_pred) ** 2))
    ss_tot    = float(np.sum((torques - torques.mean()) ** 2))
    r2_global = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

    # Bootstrap 95 % CI
    rng    = np.random.default_rng(42)
    idx    = np.arange(len(torques))
    boot_J = []
    for _ in range(4000):
        si = rng.choice(idx, size=len(idx), replace=True)
        try:
            (Jb, _), *_ = np.linalg.lstsq(A[si], torques[si], rcond=None)
            boot_J.append(Jb)
        except Exception:
            pass
    boot_J         = np.array(boot_J)
    J_std          = float(np.std(boot_J))
    J_lo95, J_hi95 = np.percentile(boot_J, [2.5, 97.5])

    print("═" * 62)
    print(f"  Traces used        : {len(all_results)}")
    print(f"  Inertia  J         : {J_est:.6f} kg·m²")
    print(f"  Std dev (boot.)    : {J_std:.6f} kg·m²")
    print(f"  95 % CI            : [{J_lo95:.6f},  {J_hi95:.6f}] kg·m²")
    if J_est != 0:
        print(f"  CoV                : {J_std / J_est:.4f}")
    print(f"  τ_friction (bias)  : {tau_friction:.4f} N·m")
    print(f"  Global fit R²      : {r2_global:.5f}")
    print("═" * 62)

    _plot(all_results, alphas_rad, torques,
          J_est, tau_friction, J_lo95, J_hi95, J_std, r2_global)


# ── plotting ──────────────────────────────────────────────────────────────────

_FIG_BG  = "#1e1e1e"
_AX_BG   = "#252525"
_GRID_C  = "#3a3a3a"
_SPINE_C = "#484848"
_TEXT_C  = "#cccccc"


def _style_ax(ax):
    ax.set_facecolor(_AX_BG)
    ax.tick_params(colors=_TEXT_C, labelsize=8)
    ax.xaxis.label.set_color(_TEXT_C)
    ax.yaxis.label.set_color(_TEXT_C)
    ax.title.set_color(_TEXT_C)
    for sp in ax.spines.values():
        sp.set_edgecolor(_SPINE_C)
    ax.grid(True, color=_GRID_C, alpha=0.35, linewidth=0.7)


def _plot(all_results, alphas_rad, torques,
          J_est, tau_friction, J_lo95, J_hi95, J_std, r2_global):

    n      = len(all_results)
    cmap   = plt.get_cmap("tab20")
    colors = [cmap(i / max(n - 1, 1)) for i in range(n)]

    fig, (ax0, ax1) = plt.subplots(
        1, 2, figsize=(15, 6), facecolor=_FIG_BG,
    )
    fig.suptitle(
        "Inertia Measurement  —  Global Regression",
        color=_TEXT_C, fontsize=13, fontweight="bold",
    )
    _style_ax(ax0)
    _style_ax(ax1)

    # ── left: velocity traces ─────────────────────────────────────────────────
    legend_handles = []

    for i, (res, col) in enumerate(zip(all_results, colors)):
        trace  = res["trace"]
        t0_abs = trace[0][0]
        ts_ms  = np.array([(t - t0_abs) * 1e3 for t, _ in trace])
        vs     = np.array([v for _, v in trace])

        i_lo, i_hi = res["win"]

        # Full trace dimmed
        ax0.plot(ts_ms, vs, color=col, lw=0.9, alpha=0.30)

        # Best linear window bright
        ax0.plot(ts_ms[i_lo:i_hi], vs[i_lo:i_hi],
                 color=col, lw=1.6, alpha=0.90)

        # Regression fit line over the window
        seg_t  = np.array([(t - t0_abs) for t, _ in trace[i_lo:i_hi]])
        seg_v  = np.array([v for _, v in trace[i_lo:i_hi]])
        coeffs = np.polyfit(seg_t, seg_v, 1)
        fit_v  = np.polyval(coeffs, seg_t)
        ax0.plot(seg_t * 1e3, fit_v, color=col, lw=2.0, ls="--", alpha=1.0)

        # Window boundary markers removed — bright segment makes them redundant

        lbl = (f"{res['torque']:+.3f} N·m {res['direction']}"
               f"  α={res['alpha_rev']:+.2f} rev/s²  R²={res['r2']:.3f}")
        legend_handles.append(
            Line2D([0], [0], color=col, lw=1.8, label=lbl)
        )

    ax0.set_xlabel("Time (ms)")
    ax0.set_ylabel("Velocity (rev/s)")
    ax0.set_title(
        "Velocity traces  ·  dim=full  bright=best-R² window  dashed=fit"
    )
    ax0.legend(
        handles=legend_handles, fontsize=6.0, loc="best",
        facecolor="#303030", edgecolor=_SPINE_C, labelcolor=_TEXT_C,
    )
    ax0.axhline(0, color=_SPINE_C, lw=0.8)

    # ── right: τ vs α scatter + global fit + CI ───────────────────────────────
    for i, col in enumerate(colors):
        ax1.scatter(
            alphas_rad[i], torques[i],
            color=col, s=70, zorder=4,
            edgecolors="#ffffff", linewidths=0.5,
        )

    a_span   = np.linspace(alphas_rad.min(), alphas_rad.max(), 300)
    fit_line = J_est * a_span + tau_friction
    ax1.plot(
        a_span, fit_line, color="#ffffff", lw=2.0,
        label=(f"Fit  J = {J_est:.5f} kg·m²\n"
               f"τ_f = {tau_friction:+.4f} N·m   R² = {r2_global:.4f}"),
    )

    ci_lo = J_lo95 * a_span + tau_friction
    ci_hi = J_hi95 * a_span + tau_friction
    ax1.fill_between(
        a_span, ci_lo, ci_hi, color="#ffffff", alpha=0.10,
        label=f"95 % CI  [{J_lo95:.5f},  {J_hi95:.5f}] kg·m²",
    )

    ax1.axhline(0, color=_SPINE_C, lw=0.8)
    ax1.axvline(0, color=_SPINE_C, lw=0.8)
    ax1.set_xlabel("Angular acceleration  α  (rad/s²)")
    ax1.set_ylabel("Applied torque  τ  (N·m)")
    ax1.set_title(
        f"Global Regression  —  J = {J_est:.5f} ± {J_std:.5f} kg·m²"
    )
    ax1.legend(
        fontsize=8, facecolor="#303030", edgecolor=_SPINE_C,
        labelcolor=_TEXT_C, loc="best",
    )

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    asyncio.run(main())
