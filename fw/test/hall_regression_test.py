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

TESTS = [
    # test                         PLL    pmetric  pmax  vmetric  vmax
    ('20250616-speed-cycle',       50,    0.09,    1.1,  1400.0,  280.0),
    ('20250616-back-and-forth',    50,    0.11,    1.1,  150.0,   70.0),
    ('20250616-bnforth-highaccel', 50,    0.09,    1.1,  190.0,   80.0),
    ('20250616-hboard-cycle',      50,    0.06,    1.1,  900.0,   130.0),
    ('20250616-hboard-manual',     50,    0.04,    1.1,  23.0,    20.0),

    ('20250616-speed-cycle',       124,   0.09,    1.1,  1400.0,  270.0),
    ('20250616-back-and-forth',    124,   0.11,    1.1,  190.0,   70.0),
    ('20250616-bnforth-highaccel', 124,   0.09,    1.1,  250.0,   80.0),
    ('20250616-hboard-cycle',      124,   0.06,    1.1,  1400.0,  160.0),
    ('20250616-hboard-manual',     124,   0.04,    1.1,  23.0,    20.0),

    ('20250616-speed-cycle',       248,   0.09,    1.1,  1400.0,  270.0),
    ('20250616-back-and-forth',    248,   0.11,    1.1,  190.0,   70.0),
    ('20250616-bnforth-highaccel', 248,   0.09,    1.1,  250.0,   80.0),
    ('20250616-hboard-cycle',      248,   0.06,    1.1,  2800.0,  180.0),
    ('20250616-hboard-manual',     248,   0.04,    1.1,  25.0,    20.0),

    ('20250616-speed-cycle',       496,   0.08,    1.1,  2800.0,  350.0),
    ('20250616-back-and-forth',    496,   0.11,    1.1,  190.0,   70.0),
    ('20250616-bnforth-highaccel', 496,   0.09,    1.1,  260.0,   80.0),
    ('20250616-hboard-cycle',      496,   0.06,    1.1,  2800.0,  180.0),
    ('20250616-hboard-manual',     496,   0.04,    1.1,  25.0,    20.0),

    ('20250616-speed-cycle',       992,   0.09,    1.1,  3500.0,  390.0),
    ('20250616-back-and-forth',    992,   0.11,    1.1,  200.0,   70.0),
    ('20250616-bnforth-highaccel', 992,   0.09,    1.1,  260.0,   80.0),
    ('20250616-hboard-cycle',      992,   0.06,    1.1,  2900.0,  180.0),
    ('20250616-hboard-manual',     992,   0.04,    1.1,  25.0,    20.0),
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
