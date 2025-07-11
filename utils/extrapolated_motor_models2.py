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
        dict(name="6374-170Kv", Kv=170, m=0.92, D=0.063, chi=1,
             R=0.026, L=36e-6, d0=0.020, d1=1.5e-4, Rth=1.8, Cth=420),
        dict(name="6355-230Kv", Kv=230, m=0.69, D=0.063, chi=1,
             R=0.031, L=28e-6, d0=0.018, d1=1.4e-4, Rth=2.0, Cth=330),
        dict(name="5065-270Kv", Kv=270, m=0.43, D=0.050, chi=1,
             R=0.041, L=22e-6, d0=0.012, d1=7.0e-5, Rth=2.6, Cth=210),
        dict(name="80100-120Kv", Kv=120, m=1.50, D=0.080, chi=1,
             R=0.019, L=78e-6, d0=0.045, d1=3.7e-4, Rth=1.5, Cth=650),
        dict(name="inrunner-45Kv", Kv=45,  m=4.10, D=0.092, chi=0,
             R=0.015, L=230e-6, d0=0.080, d1=9.0e-4, Rth=0.9, Cth=1850),
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


if __name__ == "__main__":
    main()
