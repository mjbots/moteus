#!/usr/bin/env python3

from __future__ import annotations

"""
From o3
"""

"""
infer_motor.py
==============

* Fit* proportionality constants (κ) for the power-law scaling relations
  𝑅, 𝐿, d₀, d₁, R_θa, C_th  ←  f(Kv, m, D, χ)

* Predict* the six hard-to-measure parameters for any new BLDC motor when
  you only know Kv, mass, diameter, and whether it is an outrunner.

The script keeps the physically-motivated exponents fixed (Step 3 of the
discussion) and solves **only** for κ with a one-line analytic fit.
"""

import numpy as np
import pandas as pd
import json
from pathlib import Path

# ────────────────────────────────────────────────────────────────
# 1.  Physics-derived power-law exponents  (Kvᵃ · mᵇ · Dᶜ · ξ)
# ────────────────────────────────────────────────────────────────
EXP = {
    # param : ( a      b      c )
    "R"   : (-2.0,  -1.0,  +2.0),
    "L"   : (-2.0,  +1.0,  +1.0),
    "d0"  : ( 0.0,  +1.0,  +0.0),
    "d1"  : ( 0.0,  +1.0,  +2.0),
    "Rth" : ( 0.0,  -1.0,  +1.0),
    "Cth" : ( 0.0,  +1.0,  +0.0),
}

# outrunner utilisation factor:  ξ = 1 – 0.30 ⋅ χ  (χ=1 outrunner, 0 inrunner)
def xi(chi: int, k_out: float = 0.30) -> float:
    return 1.0 - k_out * chi


# ────────────────────────────────────────────────────────────────
# 2.  Fixed-exponent analytic κ-fit
# ────────────────────────────────────────────────────────────────
def fit_kappa(df: pd.DataFrame, exp: dict = EXP) -> dict[str, float]:
    """
    Solve for κ in  y = κ · Kvᵃ · mᵇ · Dᶜ · ξ   with the exponents 'exp' fixed.
    Returns:  {"R": κ_R, "L": κ_L, ...}
    """
    kappa = {}
    for param, (a, b, c) in exp.items():
        log_base = (
            a * np.log(df["Kv"])
            + b * np.log(df["m"])
            + c * np.log(df["D"])
            + np.log(xi(df["chi"]))
        )
        log_kappa = np.mean(np.log(df[param]) - log_base)
        kappa[param] = float(np.exp(log_kappa))
    return kappa


# ────────────────────────────────────────────────────────────────
# 3.  Parameter estimation for a new motor
# ────────────────────────────────────────────────────────────────
def estimate(
    motor: dict, kappa: dict[str, float], exp: dict = EXP
) -> dict[str, float]:
    Kv, m, D, chi = motor["Kv"], motor["m"], motor["D"], motor["chi"]
    fac = xi(chi)
    out = {}
    for p, (a, b, c) in exp.items():
        out[p] = kappa[p] * (Kv**a) * (m**b) * (D**c) * fac
    return out


# ────────────────────────────────────────────────────────────────
# 4.  Demonstration
# ────────────────────────────────────────────────────────────────
def main() -> None:
    # --- sample calibration set (replace with your real data) ---
    sample_motors = [
        dict(name="mj5208", Kv=304, m=0.192, D=0.063, chi=1,
             R=0.047, L=28.6e-6, d0=0.01289, d1=0.0001469,
             Rth=4.55, Cth=164.59),
        dict(name="mad8318", Kv=120, m=0.646, D=0.091, chi=1,
             R=0.015, L=9.75e-6, d0=0.1272, d1=0.001829,
             Rth=1.516, Cth=735.5),
        dict(name="gbm5208", Kv=25.5, m=0.193, D=0.063, chi=1,
             R=7.545, L=2254.5e-6, d0=0.00822, d1=0.000183,
             Rth=4.55, Cth=165),
        dict(name="be8108", Kv=135, m=0.230, D=0.087, chi=1,
             R=0.0910, L=39.2e-6, d0=0.0300, d1=0.000691,
             Rth=2.74, Cth=210.9),
#        dict(name="hoverboard350", Kv=18.2, m=4.26, D=0.156, chi=1,
#             R=0.216, L=530e-6, d0=0.0892, d1=0.0131,
#             Rth=2.41, Cth=946.1),
#        dict(name="ht1105", Kv=1180, m=0.008, D=0.016, chi=1,
#             R=6.435, L=298.5e-6, d0=6.29e-5, d1=3.80e-6,
#             Rth=20.8, Cth=14.8),

    ]
    df = pd.DataFrame(sample_motors)

    # --- step 1: fit κ constants --------------------------------
    kappa = fit_kappa(df)
    print("Fitted κ constants:")
    for p, v in kappa.items():
        print(f"  κ_{p:<3}= {v:.4g}")
    Path("kappa.json").write_text(json.dumps(kappa, indent=2))
    print("\n(saved to kappa.json)\n")

    # --- step 2: estimate parameters for a new motor ------------
    test_motor = dict(name="test-6374-190Kv", Kv=190, m=0.88, D=0.063, chi=1)
    est = estimate(test_motor, kappa)

    print(f"Estimates for {test_motor['name']}:")
    units = {"R": "Ω", "L": "H", "d0": "W·s", "d1": "W·s²",
             "Rth": "K/W", "Cth": "J/K"}
    for p in ["R", "L", "d0", "d1", "Rth", "Cth"]:
        print(f"  {p:<4}= {est[p]:.4g} {units[p]}")

    # --- step 3: test fits for all sample database motors -------
    print(f"\n{'='*60}")
    print("Test fits for all sample database motors:")
    print(f"{'='*60}")
    print(f"{'Motor':<15} {'Param':<4} {'Measured':<12} {'Estimated':<12} {'Error %':<8}")
    print(f"{'-'*60}")

    for _, motor in df.iterrows():
        motor_dict = motor.to_dict()
        est = estimate(motor_dict, kappa)

        print(f"\n{motor['name']:<15}")
        for p in ["R", "L", "d0", "d1", "Rth", "Cth"]:
            measured = motor[p]
            estimated = est[p]
            error_pct = abs(estimated - measured) / measured * 100 if measured != 0 else 0

            print(f"{'':15} {p:<4} {measured:<12.4g} {estimated:<12.4g} {error_pct:<8.1f}")

    print(f"\n{'-'*60}")
    print("Summary statistics:")
    all_errors = []
    for _, motor in df.iterrows():
        motor_dict = motor.to_dict()
        est = estimate(motor_dict, kappa)
        for p in ["R", "L", "d0", "d1", "Rth", "Cth"]:
            measured = motor[p]
            estimated = est[p]
            if measured != 0:
                error_pct = abs(estimated - measured) / measured * 100
                all_errors.append(error_pct)

    print(f"Mean absolute error: {np.mean(all_errors):.1f}%")
    print(f"Median absolute error: {np.median(all_errors):.1f}%")
    print(f"Max absolute error: {np.max(all_errors):.1f}%")


if __name__ == "__main__":
    main()
