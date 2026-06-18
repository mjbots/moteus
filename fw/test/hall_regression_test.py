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
TESTS = [
    # test                         PLL   pmetric  pmax  vmetric  vmax
    ('20250616-speed-cycle',         50,     0.15,   2.4,    781.0,   160.0),
    ('20250616-back-and-forth',      50,    0.205,  2.68,    155.0,    89.1),
    ('20250616-bnforth-highaccel',   50,    0.114,   2.4,    212.0,   120.0),
    ('20250616-hboard-cycle',        50,   0.0605,   1.8,     80.4,    39.3),
    ('20250616-hboard-manual',       50,    0.151,   2.4,     20.8,    32.0),
    ('20250615-new-slow',            50,    0.306,  2.81,    286.0,   119.0),

    ('20250616-speed-cycle',        124,    0.149,   2.4,    596.0,   147.0),
    ('20250616-back-and-forth',     124,    0.205,  2.68,    161.0,    89.1),
    ('20250616-bnforth-highaccel',  124,    0.115,   2.4,    214.0,   120.0),
    ('20250616-hboard-cycle',       124,   0.0592,   1.8,    403.0,   125.0),
    ('20250616-hboard-manual',      124,    0.151,   2.4,     20.8,    32.0),
    ('20250615-new-slow',           124,    0.306,  2.81,    286.0,   119.0),

    ('20250616-speed-cycle',        248,     0.15,   2.4,    562.0,   155.0),
    ('20250616-back-and-forth',     248,    0.205,  2.68,    161.0,    89.1),
    ('20250616-bnforth-highaccel',  248,    0.115,   2.4,    214.0,   120.0),
    ('20250616-hboard-cycle',       248,   0.0614,   1.8,    954.0,   153.0),
    ('20250616-hboard-manual',      248,    0.151,   2.4,     20.8,    32.0),
    ('20250615-new-slow',           248,    0.306,  2.81,    286.0,   119.0),

    ('20250616-speed-cycle',        496,     0.15,   2.4,   1550.0,   312.0),
    ('20250616-back-and-forth',     496,    0.205,  2.68,    161.0,    89.1),
    ('20250616-bnforth-highaccel',  496,    0.115,   2.4,    214.0,   120.0),
    ('20250616-hboard-cycle',       496,   0.0614,   1.8,    954.0,   153.0),
    ('20250616-hboard-manual',      496,    0.151,   2.4,     20.8,    32.0),
    ('20250615-new-slow',           496,    0.306,  2.81,    286.0,   119.0),

    ('20250616-speed-cycle',        992,     0.15,   2.4,   1800.0,   312.0),
    ('20250616-back-and-forth',     992,    0.205,  2.68,    161.0,    89.1),
    ('20250616-bnforth-highaccel',  992,    0.115,   2.4,    214.0,   120.0),
    ('20250616-hboard-cycle',       992,   0.0614,   1.8,    954.0,   153.0),
    ('20250616-hboard-manual',      992,    0.151,   2.4,     20.8,    32.0),
    ('20250615-new-slow',           992,    0.306,  2.81,    286.0,   119.0),
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

    for test_name, pll_filter_hz, pmetric, pmax, vmetric, vmax in TESTS:
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

        if test_pass:
            tests_passed += 1
            print(f"PASS: {test_name} {pll_filter_hz}Hz p={result['position_metric']}/{result['max_position_error']} v={result['velocity_metric']}/{result['max_velocity_error']}")
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
