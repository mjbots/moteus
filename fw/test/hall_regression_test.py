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
# vbias% is the worst steady-state velocity bias over flat-speed
# windows (one-sided -- smaller is always better; no lower bound, with
# a 1.0% floor so near-zero cases are not brittle).  It isolates the
# open-collector rise/fall velocity bias from per-edge ripple and
# acceleration lag.  20260618-spindle-hw is a real-motor capture.
TESTS = [
    # test                         PLL   pmetric  pmax  vmetric  vmax  vbias%
    ('20250616-speed-cycle',         50,     0.15,   2.4,    781.0,   160.0,   1.0),
    ('20250616-back-and-forth',      50,    0.205,  2.68,    155.0,    89.1,  4.21),
    ('20250616-bnforth-highaccel',   50,    0.114,   2.4,    212.0,   120.0,  3.77),
    ('20250616-hboard-cycle',        50,   0.0605,   1.8,     80.4,    39.3,  3.79),
    ('20250616-hboard-manual',       50,    0.151,   2.4,     20.8,    32.0,  1.46),
    ('20250615-new-slow',            50,    0.306,  2.81,    286.0,   119.0,   1.0),
    ('20260618-spindle-sweep',       50,   0.0359,  1.88,    507.0,    84.9,  4.14),
    ('20260618-spindle-hw',          50,   0.0426,   1.8,   1790.0,   185.0,  3.32),

    ('20250616-speed-cycle',        124,    0.149,   2.4,    596.0,   147.0,   1.0),
    ('20250616-back-and-forth',     124,    0.205,  2.68,    161.0,    89.1,   1.0),
    ('20250616-bnforth-highaccel',  124,    0.115,   2.4,    214.0,   120.0,   1.0),
    ('20250616-hboard-cycle',       124,   0.0592,   1.8,    403.0,   125.0,  8.55),
    ('20250616-hboard-manual',      124,    0.151,   2.4,     20.8,    32.0,  1.46),
    ('20250615-new-slow',           124,    0.306,  2.81,    286.0,   119.0,   1.0),
    ('20260618-spindle-sweep',      124,   0.0253,  1.88,   1470.0,   131.0,   7.5),
    ('20260618-spindle-hw',         124,   0.0269,   1.8,   1010.0,   183.0,  1.93),

    ('20250616-speed-cycle',        248,     0.15,   2.4,    562.0,   155.0,   1.0),
    ('20250616-back-and-forth',     248,    0.205,  2.68,    161.0,    89.1,   1.0),
    ('20250616-bnforth-highaccel',  248,    0.115,   2.4,    214.0,   120.0,   1.0),
    ('20250616-hboard-cycle',       248,   0.0614,   1.8,    954.0,   153.0,  4.13),
    ('20250616-hboard-manual',      248,    0.151,   2.4,     20.8,    32.0,  1.46),
    ('20250615-new-slow',           248,    0.306,  2.81,    286.0,   119.0,   1.0),
    ('20260618-spindle-sweep',      248,    0.034,  1.88,   9830.0,   351.0,  8.35),
    ('20260618-spindle-hw',         248,    0.025,   1.8,  21400.0,   502.0,  13.6),

    ('20250616-speed-cycle',        496,     0.15,   2.4,   1550.0,   312.0,   1.0),
    ('20250616-back-and-forth',     496,    0.205,  2.68,    161.0,    89.1,   1.0),
    ('20250616-bnforth-highaccel',  496,    0.115,   2.4,    214.0,   120.0,   1.0),
    ('20250616-hboard-cycle',       496,   0.0614,   1.8,    954.0,   153.0,  4.13),
    ('20250616-hboard-manual',      496,    0.151,   2.4,     20.8,    32.0,  1.46),
    ('20250615-new-slow',           496,    0.306,  2.81,    286.0,   119.0,   1.0),
    ('20260618-spindle-sweep',      496,   0.0379,  1.88,  12100.0,   460.0,   8.4),
    ('20260618-spindle-hw',         496,   0.0349,   1.8,  68900.0,   915.0,  7.88),

    ('20250616-speed-cycle',        992,     0.15,   2.4,   1800.0,   312.0,   1.0),
    ('20250616-back-and-forth',     992,    0.205,  2.68,    161.0,    89.1,   1.0),
    ('20250616-bnforth-highaccel',  992,    0.115,   2.4,    214.0,   120.0,   1.0),
    ('20250616-hboard-cycle',       992,   0.0614,   1.8,    954.0,   153.0,  4.13),
    ('20250616-hboard-manual',      992,    0.151,   2.4,     20.8,    32.0,  1.46),
    ('20250615-new-slow',           992,    0.306,  2.81,    286.0,   119.0,   1.0),
    ('20260618-spindle-sweep',      992,   0.0379,  1.88,  12100.0,   460.0,   8.4),
    ('20260618-spindle-hw',         992,   0.0409,   1.8,  78100.0,  1540.0,  7.88),
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
