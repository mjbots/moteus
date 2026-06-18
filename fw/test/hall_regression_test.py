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
TESTS = [
    # test                         PLL   pmetric  pmax  vmetric  vmax  vbias%
    ('20250616-speed-cycle',       50,    0.13,   2.30,   421.0,  132.0,   1.0),
    ('20250616-back-and-forth',    50,    0.191,  2.56,   133.0,   82.3,   4.43),
    ('20250616-bnforth-highaccel', 50,    0.109,  2.30,   212.0,  140.0,   4.2),
    ('20250616-hboard-cycle',      50,    0.0585, 1.72,    29.4,   38.6,   1.03),
    ('20250616-hboard-manual',     50,    0.142,  2.30,    17.5,   32.2,   2.69),
    ('20250615-new-slow',          50,    0.289,  2.70,   320.0,  128.0,   1.0),
    ('20260618-spindle-sweep',     50,    0.033,  1.80,   117.0,   67.0,   1.0),
    ('20260618-spindle-hw',        50,    0.0397, 1.72,   666.0,  148.0,   1.0),

    ('20250616-speed-cycle',       124,   0.129,  2.30,   432.0,  135.0,   1.0),
    ('20250616-back-and-forth',    124,   0.191,  2.56,   132.0,   82.3,   1.0),
    ('20250616-bnforth-highaccel', 124,   0.109,  2.30,   205.0,  140.0,   1.0),
    ('20250616-hboard-cycle',      124,   0.0567, 1.72,    42.6,   38.6,   1.03),
    ('20250616-hboard-manual',     124,   0.142,  2.30,    17.5,   32.2,   2.69),
    ('20250615-new-slow',          124,   0.289,  2.70,   320.0,  128.0,   1.0),
    ('20260618-spindle-sweep',     124,   0.0226, 1.80,   178.0,   77.7,   1.0),
    ('20260618-spindle-hw',        124,   0.0253, 1.72,   715.0,  153.0,   1.0),

    ('20250616-speed-cycle',       248,   0.129,  2.30,   443.0,  149.0,   1.0),
    ('20250616-back-and-forth',    248,   0.191,  2.56,   132.0,   82.3,   1.0),
    ('20250616-bnforth-highaccel', 248,   0.109,  2.30,   205.0,  140.0,   1.0),
    ('20250616-hboard-cycle',      248,   0.0567, 1.72,    42.6,   38.6,   1.03),
    ('20250616-hboard-manual',     248,   0.142,  2.30,    17.5,   32.2,   2.69),
    ('20250615-new-slow',          248,   0.289,  2.70,   320.0,  128.0,   1.0),
    ('20260618-spindle-sweep',     248,   0.0246, 1.80,   186.0,   68.6,   1.0),
    ('20260618-spindle-hw',        248,   0.0232, 1.72,   956.0,  169.0,   1.0),

    ('20250616-speed-cycle',       496,   0.129,  2.30,   707.0,  218.0,   1.0),
    ('20250616-back-and-forth',    496,   0.191,  2.56,   132.0,   82.3,   1.0),
    ('20250616-bnforth-highaccel', 496,   0.109,  2.30,   205.0,  140.0,   1.0),
    ('20250616-hboard-cycle',      496,   0.0567, 1.72,    42.6,   38.6,   1.03),
    ('20250616-hboard-manual',     496,   0.142,  2.30,    17.5,   32.2,   2.69),
    ('20250615-new-slow',          496,   0.289,  2.70,   320.0,  128.0,   1.0),
    ('20260618-spindle-sweep',     496,   0.0246, 1.80,   186.0,   68.6,   1.0),
    ('20260618-spindle-hw',        496,   0.0264, 1.72,  1350.0,  677.0,   1.0),

    ('20250616-speed-cycle',       992,   0.129,  2.30,   707.0,  218.0,   1.0),
    ('20250616-back-and-forth',    992,   0.191,  2.56,   132.0,   82.3,   1.0),
    ('20250616-bnforth-highaccel', 992,   0.109,  2.30,   205.0,  140.0,   1.0),
    ('20250616-hboard-cycle',      992,   0.0567, 1.72,    42.6,   38.6,   1.03),
    ('20250616-hboard-manual',     992,   0.142,  2.30,    17.5,   32.2,   2.69),
    ('20250615-new-slow',          992,   0.289,  2.70,   320.0,  128.0,   1.0),
    ('20260618-spindle-sweep',     992,   0.0246, 1.80,   186.0,   68.6,   1.0),
    ('20260618-spindle-hw',        992,   0.0272, 1.72,  1220.0,  218.0,   1.0),
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

    for test_name, pll_filter_hz, pmetric, pmax, vmetric, vmax, vbias in TESTS:
        source_test_filename = f"{test_name}.dat.gz"
        shutil.copy(os.path.join(datadir, source_test_filename), tdir.name)

        subprocess.check_call(['gzip', '-k', '-f', '-d',
                               os.path.join(tdir.name, source_test_filename)])

        result_json = subprocess.check_output([args.binary, '--input', os.path.join(tdir.name, f"{test_name}.dat"), "--pll_filter_hz", f"{pll_filter_hz}"])

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
        # vbias is one-sided: lower bias is always better, so there is
        # no lower bound (and a floor in the table keeps near-zero
        # cases from being brittle).
        if abs(result['max_velocity_bias_pct']) >= vbias:
            test_pass = False

        if test_pass:
            tests_passed += 1
            print(f"PASS: {test_name} {pll_filter_hz}Hz p={result['position_metric']}/{result['max_position_error']} v={result['velocity_metric']}/{result['max_velocity_error']} bias={result['max_velocity_bias_pct']:.2f}%")
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
