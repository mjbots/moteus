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

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

# Thresholds are tuned for the falling-edge ground-truth oracle in
# hall_filter_test.cc (velocity and position derived from the sharp
# falling hall edges only, so they are insensitive to the open
# collector rise/fall asymmetry).  pmetric/pmax/vmetric/vmax are
# bounded both above and below (the runner also fails under half the
# threshold), bracketing the measured result with ~20% headroom so a
# regression is caught tightly.
#
# These reflect the shipping hall velocity estimator: every edge feeds
# the PLL, the slow-mode velocity is the two-edge (rise+fall) average,
# and the PLL-band velocity has its DC bias removed online against that
# same two-edge velocity.  Both modes are steady-state unbiased, with
# hysteresis across the slow/PLL boundary.
#
# vbias% is the worst steady-state velocity bias over flat-speed
# windows (one-sided -- smaller is always better; no lower bound, 1.0%
# floor).  It is ~0 on the spindle and hardware captures (the operating
# regime); the reversal-heavy back-and-forth/bnforth stress cases keep
# a small residual.  20260618-spindle-hw is a real-motor capture used
# to validate the bias fix on hardware.
#
# ripple% is the edge-locked velocity ripple over the same flat-speed
# windows (one-sided; 0.3% floor): the peak-to-peak amplitude of the
# velocity error averaged synchronously onto phase bins spanning
# falling-edge brackets, as a percentage of true speed.  At constant
# speed that mean waveform should be flat; any deterministic
# edge-synchronous structure -- an anti-overrun decay firing before
# the next edge is actually late, per-edge rise/fall alternation, edge
# overshoot -- survives the averaging while noise cancels.  Such
# structure is nearly invisible to the sum-of-squares velocity_metric
# but directly excites a velocity-feedback (kd) term at a multiple of
# the hall edge rate.  The anti-overrun decay used to fire
# deterministically at 75% of every sector in slow mode (1.4-2%
# ripple on the hboard/new-slow/n1 captures); it now requires both
# dead-reckoning past the sector boundary and the edge being overdue
# in time.
#
# 20260715-hboard-n1-1hz is a moteus-n1 + hoverboard motor hardware
# capture at a constant 1 Hz (the regime that exposed the
# deterministic decay; ~2% edge-locked ripple before the fix).  It was
# captured at a 20 kHz control rate rather than the 30 kHz of the
# other datasets.
RATE_OVERRIDES_HZ = {
    '20260715-hboard-n1-1hz': 20000,
}

TESTS = [
    # test                        PLL   pmetric  pmax   vmetric  vmax    vbias% ripple%
    ('20250616-speed-cycle',          50,   0.129,   2.4,   430.0,   135.0,  1.0,   0.3),
    ('20250616-back-and-forth',       50,   0.195,   2.68,  140.0,   82.3,   1.0,   0.453),
    ('20250616-bnforth-highaccel',    50,   0.11,    2.4,   221.0,   140.0,  1.0,   0.3),
    ('20250616-hboard-cycle',         50,   0.0586,  1.8,   29.1,    38.6,   1.0,   0.73),
    ('20250616-hboard-manual',        50,   0.146,   2.4,   19.0,    32.2,   2.0,   0.3),
    ('20250615-new-slow',             50,   0.294,   2.81,  344.0,   128.0,  1.0,   0.751),
    ('20260618-spindle-sweep',        50,   0.0285,  1.88,  116.0,   67.0,   1.0,   1.07),
    ('20260618-spindle-hw',           50,   0.0306,  1.8,   650.0,   141.0,  1.0,   0.48),
    ('20260715-hboard-n1-1hz',        50,   0.00561, 1.8,   50.3,    117.0,  1.12,  0.316),

    ('20250616-speed-cycle',         124,   0.129,   2.4,   430.0,   136.0,  1.0,   0.3),
    ('20250616-back-and-forth',      124,   0.195,   2.68,  141.0,   82.3,   1.0,   0.303),
    ('20250616-bnforth-highaccel',   124,   0.11,    2.4,   218.0,   140.0,  1.0,   0.336),
    ('20250616-hboard-cycle',        124,   0.058,   1.8,   43.3,    38.6,   1.0,   0.31),
    ('20250616-hboard-manual',       124,   0.146,   2.4,   19.0,    32.2,   2.0,   0.3),
    ('20250615-new-slow',            124,   0.294,   2.81,  344.0,   128.0,  1.0,   0.751),
    ('20260618-spindle-sweep',       124,   0.0221,  1.88,  175.0,   77.7,   1.0,   2.51),
    ('20260618-spindle-hw',          124,   0.024,   1.8,   711.0,   148.0,  1.0,   0.86),
    ('20260715-hboard-n1-1hz',       124,   0.00561, 1.8,   50.3,    117.0,  1.12,  0.316),

    ('20250616-speed-cycle',         248,   0.13,    2.4,   446.0,   143.0,  1.0,   0.3),
    ('20250616-back-and-forth',      248,   0.195,   2.68,  141.0,   82.3,   1.0,   0.303),
    ('20250616-bnforth-highaccel',   248,   0.11,    2.4,   218.0,   140.0,  1.0,   0.336),
    ('20250616-hboard-cycle',        248,   0.058,   1.8,   43.3,    38.6,   1.0,   0.31),
    ('20250616-hboard-manual',       248,   0.146,   2.4,   19.0,    32.2,   2.0,   0.3),
    ('20250615-new-slow',            248,   0.294,   2.81,  344.0,   128.0,  1.0,   0.751),
    ('20260618-spindle-sweep',       248,   0.0247,  1.88,  187.0,   68.6,   1.0,   0.664),
    ('20260618-spindle-hw',          248,   0.0237,  1.8,   947.0,   169.0,  1.0,   1.36),
    ('20260715-hboard-n1-1hz',       248,   0.00561, 1.8,   50.3,    117.0,  1.12,  0.316),

    ('20250616-speed-cycle',         496,   0.129,   2.4,   714.0,   218.0,  1.0,   0.3),
    ('20250616-back-and-forth',      496,   0.195,   2.68,  141.0,   82.3,   1.0,   0.303),
    ('20250616-bnforth-highaccel',   496,   0.11,    2.4,   218.0,   140.0,  1.0,   0.336),
    ('20250616-hboard-cycle',        496,   0.058,   1.8,   43.3,    38.6,   1.0,   0.31),
    ('20250616-hboard-manual',       496,   0.146,   2.4,   19.0,    32.2,   2.0,   0.3),
    ('20250615-new-slow',            496,   0.294,   2.81,  344.0,   128.0,  1.0,   0.751),
    ('20260618-spindle-sweep',       496,   0.0247,  1.88,  187.0,   68.6,   1.0,   0.664),
    ('20260618-spindle-hw',          496,   0.0269,  1.8,   1340.0,  677.0,  1.0,   0.317),
    ('20260715-hboard-n1-1hz',       496,   0.00561, 1.8,   50.3,    117.0,  1.12,  0.316),

    ('20250616-speed-cycle',         992,   0.129,   2.4,   714.0,   218.0,  1.0,   0.3),
    ('20250616-back-and-forth',      992,   0.195,   2.68,  141.0,   82.3,   1.0,   0.303),
    ('20250616-bnforth-highaccel',   992,   0.11,    2.4,   218.0,   140.0,  1.0,   0.336),
    ('20250616-hboard-cycle',        992,   0.058,   1.8,   43.3,    38.6,   1.0,   0.31),
    ('20250616-hboard-manual',       992,   0.146,   2.4,   19.0,    32.2,   2.0,   0.3),
    ('20250615-new-slow',            992,   0.294,   2.81,  344.0,   128.0,  1.0,   0.751),
    ('20260618-spindle-sweep',       992,   0.0247,  1.88,  187.0,   68.6,   1.0,   0.664),
    ('20260618-spindle-hw',          992,   0.0276,  1.8,   1220.0,  218.0,  1.0,   0.317),
    ('20260715-hboard-n1-1hz',       992,   0.00561, 1.8,   50.3,    117.0,  1.12,  0.316),
]

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--binary')
    parser.add_argument('--datafile')

    args = parser.parse_args()

    datadir = os.path.dirname(args.datafile)

    tdir = tempfile.TemporaryDirectory(prefix='hall_regression-')

    tests_passed = 0
    tests_failed = 0

    for test_name, pll_filter_hz, pmetric, pmax, vmetric, vmax, vbias, vripple in TESTS:
        source_test_filename = f"{test_name}.dat.gz"
        shutil.copy(os.path.join(datadir, source_test_filename), tdir.name)

        subprocess.check_call(['gzip', '-k', '-f', '-d',
                               os.path.join(tdir.name, source_test_filename)])

        rate_hz = RATE_OVERRIDES_HZ.get(test_name, 30000)
        result_json = subprocess.check_output([args.binary, '--input', os.path.join(tdir.name, f"{test_name}.dat"), "--pll_filter_hz", f"{pll_filter_hz}", "--rate_hz", f"{rate_hz}"])

        result = json.loads(result_json)

        test_pass = True

        if result['velocity_metric'] >= vmetric:
            test_pass = False
        if result['velocity_metric'] < 0.5 * vmetric:
            test_pass = False
        if result['max_velocity_error'] >= vmax:
            test_pass = False
        if result['max_velocity_error'] < 0.5 * vmax:
            test_pass = False
        if result['position_metric'] >= pmetric:
            test_pass = False
        if result['position_metric'] < 0.5 * pmetric:
            test_pass = False
        if result['max_position_error'] >= pmax:
            test_pass = False
        if result['max_position_error'] < 0.5 * pmax:
            test_pass = False
        # vbias and ripple are one-sided: lower is always better, so
        # there is no lower bound (and a floor in the table keeps
        # near-zero cases from being brittle).
        if abs(result['max_velocity_bias_pct']) >= vbias:
            test_pass = False
        if result['edge_ripple_pct'] >= vripple:
            test_pass = False

        if test_pass:
            tests_passed += 1
            print(f"PASS: {test_name} {pll_filter_hz}Hz p={result['position_metric']}/{result['max_position_error']} v={result['velocity_metric']}/{result['max_velocity_error']} bias={result['max_velocity_bias_pct']:.2f}% ripple={result['edge_ripple_pct']:.2f}%")
        else:
            tests_failed += 1

            print(f"FAIL: {test_name} {pll_filter_hz}Hz {result}")

    print()
    print(f"PASS: {tests_passed}")
    print(f"FAIL: {tests_failed}")
    print()

    if tests_failed == 0:
        print("ALL PASS")
    else:
        print("SOME TESTS FAILED!")
        sys.exit(1)

if __name__ == '__main__':
    main()
