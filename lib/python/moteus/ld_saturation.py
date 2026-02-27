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

"""L_d saturation analysis: pure computation, no I/O.

Analyzes voltage-mode V_d injection data to extract the D-axis
inductance saturation curve L_d(d_A) = B + C*d_A from steady-state
measurements of y = q_V - R*q_A vs omega_e at multiple speed and
d_A levels.

The physical model is:

    y = lambda_m * omega + (B * d_A + C * d_A^2) * omega + offset_k

where offset_k absorbs per-level voltage offsets (dead time, etc.).
An omega^2-weighted least squares fit emphasizes high-speed points
where the offset contribution is small relative to the signal.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass
class LdFitDetails:
    """Full result of the L_d saturation fit pipeline."""
    B: float
    C: float
    lambda_m: float               # from global_fit (pre-fallback)
    slope_data: Dict[float, float]
    regression_data: Dict[float, Dict[str, float]]
    clean_d_A: List[float]
    fallback_applied: bool        # True if C>0 triggered median fallback


def median(x):
    """Return the median of a sequence."""
    s = sorted(x)
    n = len(s)
    if n == 0:
        raise ValueError("median of empty sequence")
    if n % 2:
        return s[n // 2]
    return (s[n // 2 - 1] + s[n // 2]) / 2.0


def per_level_regression(avg_data):
    """Per-d_A-level linear regression of y vs omega.

    For each d_A level with >= 3 points and sufficient omega spread,
    fits y = slope*omega + intercept.

    Args:
        avg_data: {d_A_float: [(omega, y), ...]}

    Returns:
        (slope_data, regression_data) where:
        - slope_data: {d_A: slope}
        - regression_data: {d_A: {'slope', 'intercept', 'r_squared', 'n'}}
    """
    all_d_A = sorted(avg_data.keys())
    slope_data = {}
    regression_data = {}

    for d_A in all_d_A:
        pts = avg_data[d_A]
        if len(pts) < 3:
            continue
        omegas = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        n = len(omegas)

        omega_min = min(omegas)
        omega_max = max(omegas)
        if (omega_min > 0 and
                omega_max / omega_min < 1.3):
            print(f"    skip d_A={d_A:6.1f}: "
                  f"omega spread "
                  f"{omega_max/omega_min:.2f}x "
                  f"< 1.3x")
            continue

        # 2-parameter regression: y = slope*omega + b.
        sx = sum(omegas)
        sy = sum(ys)
        sxy = sum(x * y for x, y in zip(omegas, ys))
        sxx = sum(x * x for x in omegas)
        denom = n * sxx - sx * sx
        if abs(denom) < 1e-30:
            continue
        slope = (n * sxy - sx * sy) / denom
        intercept = (sy - slope * sx) / n
        ss_res = sum((y - slope * x - intercept) ** 2
                     for x, y in zip(omegas, ys))
        mean_y = sy / n
        ss_tot = sum((y - mean_y) ** 2 for y in ys)
        r_sq = (1.0 - ss_res / ss_tot
                if ss_tot > 1e-30 else 0.0)
        slope_data[d_A] = slope
        regression_data[d_A] = {
            'slope': slope,
            'intercept': intercept,
            'r_squared': r_sq,
            'n': n,
        }

    return slope_data, regression_data


def reject_outliers(slope_data):
    """MAD-based outlier rejection on z-scores.

    Transforms slopes to z(d_A) = (slope(d_A) - slope(0)) / d_A,
    then rejects z outliers using MAD (median absolute deviation).

    Args:
        slope_data: {d_A: slope} (must include 0.0 key)

    Returns:
        list of clean non-zero d_A values, or None if slope(0) missing.
    """
    slope_0 = slope_data.get(0.0)
    if slope_0 is None:
        return None

    nonzero_d_A = [d for d in sorted(slope_data.keys())
                   if d != 0.0]
    z_data = {d: (slope_data[d] - slope_0) / d
              for d in nonzero_d_A}

    clean_d_A = list(nonzero_d_A)
    if len(nonzero_d_A) >= 4:
        z_vals = [z_data[d] for d in nonzero_d_A]
        median_z = median(z_vals)
        mad = median(
            [abs(z - median_z) for z in z_vals])
        if mad > 1e-12:
            sigma_est = mad / 0.6745
            threshold = 3.0 * sigma_est
            clean_d_A = [
                d for d in nonzero_d_A
                if (abs(z_data[d] - median_z)
                    <= threshold)]

    for d in nonzero_d_A:
        suffix = (" (rejected)"
                  if d not in clean_d_A else "")
        print(f"    d_A={d:6.1f}: "
              f"L_d={z_data[d]*1e6:.1f} uH{suffix}")

    return clean_d_A


def global_fit(avg_data, valid_d_A):
    """Unweighted least-squares fit of all data points.

    Fits y = lambda_m*omega + B*d_A*omega + C*d_A^2*omega + f_k
    where f_k is a per-level constant offset.

    Args:
        avg_data: {d_A_float: [(omega, y), ...]}
        valid_d_A: sorted list of d_A levels to include

    Returns:
        (lambda_m, B, C) or None if singular.
    """
    level_map = {d: i for i, d in enumerate(valid_d_A)}
    K = len(valid_d_A)
    NP = 3 + K
    XtX = [[0.0] * NP for _ in range(NP)]
    Xty = [0.0] * NP

    for d_A in valid_d_A:
        k = level_map[d_A]
        for omega, y in avg_data[d_A]:
            w = 1.0
            row = [0.0] * NP
            row[0] = omega
            row[1] = d_A * omega
            row[2] = d_A * d_A * omega
            row[3 + k] = 1.0
            for i in range(NP):
                for j in range(NP):
                    XtX[i][j] += w * row[i] * row[j]
                Xty[i] += w * row[i] * y

    aug = [XtX[i][:] + [Xty[i]] for i in range(NP)]
    for col in range(NP):
        max_row = col
        for row in range(col + 1, NP):
            if abs(aug[row][col]) > abs(aug[max_row][col]):
                max_row = row
        aug[col], aug[max_row] = aug[max_row], aug[col]
        if abs(aug[col][col]) < 1e-30:
            return None
        for row in range(col + 1, NP):
            f = aug[row][col] / aug[col][col]
            for j in range(col, NP + 1):
                aug[row][j] -= f * aug[col][j]

    beta = [0.0] * NP
    for i in range(NP - 1, -1, -1):
        beta[i] = aug[i][NP]
        for j in range(i + 1, NP):
            beta[i] -= aug[i][j] * beta[j]
        beta[i] /= aug[i][i]

    return (beta[0], beta[1], beta[2])


def analyze_detailed(avg_data, params) -> Optional[LdFitDetails]:
    """Run the full L_d saturation analysis pipeline.

    Same as ``analyze`` but returns a structured result that
    includes the per-level regressions and the pre-fallback
    ``lambda_m`` for callers that want to emit richer diagnostics
    (e.g. fixture-writing tools).

    Args:
        avg_data: {d_A_float: [(omega, y), ...]}
        params: {'R': float, 'poles': int}

    Returns:
        LdFitDetails, or None on failure.
    """
    all_d_A = sorted(avg_data.keys())

    total_pts = sum(len(avg_data[d]) for d in all_d_A)
    if total_pts < 10:
        print("WARNING: Not enough data points for fit")
        return None

    # Per-level regression (used for outlier rejection).
    slope_data, regression_data = per_level_regression(avg_data)

    clean_d_A = reject_outliers(slope_data)
    if clean_d_A is None:
        print("WARNING: No d_A=0 slope")
        return None

    # Global fit.
    valid_d_A = sorted([0.0] + clean_d_A)
    result = global_fit(avg_data, valid_d_A)
    if result is None:
        print("WARNING: Global fit failed")
        return None

    lambda_m, B, C = result
    fallback_applied = False

    if C > 0:
        slope_0 = slope_data[0.0]
        z_vals = [(slope_data[d] - slope_0) / d
                  for d in clean_d_A]
        B = median(z_vals)
        C = 0.0
        fallback_applied = True

    max_d_A = min(all_d_A)
    L_d_max = B + C * max_d_A
    print(f"  L_d(0)={B*1e6:.1f} uH"
          f"  L_d({max_d_A:.1f})={L_d_max*1e6:.1f} uH")

    return LdFitDetails(
        B=B,
        C=C,
        lambda_m=lambda_m,
        slope_data=slope_data,
        regression_data=regression_data,
        clean_d_A=clean_d_A,
        fallback_applied=fallback_applied,
    )


def analyze(avg_data, params):
    """Run the full L_d saturation analysis pipeline.

    Thin back-compat wrapper around ``analyze_detailed``.

    Args:
        avg_data: {d_A_float: [(omega, y), ...]}
        params: {'R': float, 'poles': int}

    Returns:
        (B, C) tuple where L_d(d_A) = B + C*d_A, or None on failure.
    """
    d = analyze_detailed(avg_data, params)
    return None if d is None else (d.B, d.C)
