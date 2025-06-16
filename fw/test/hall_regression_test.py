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
    ('20250616-speed-cycle',       50,    0.30,    1.1,  1700.0,  260.0),
    ('20250616-back-and-forth',    50,    0.25,    1.1,  450.0,   120.0),
    ('20250616-bnforth-highaccel', 50,    0.26,    1.1,  350.0,   130.0),
    ('20250616-hboard-cycle',      50,    0.30,    1.1,  1700.0,  140.0),
    ('20250616-hboard-manual',     50,    0.30,    1.1,  1200.0,  120.0),

    ('20250616-speed-cycle',       100,   0.30,    1.1,  3600.0,  260.0),
    ('20250616-back-and-forth',    100,   0.30,    1.1,  3000.0,  250.0),
    ('20250616-bnforth-highaccel', 100,   0.26,    1.1,  2600.0,  250.0),
    ('20250616-hboard-cycle',      100,   0.30,    1.1,  4100.0,  240.0),
    ('20250616-hboard-manual',     100,   0.35,    1.1,  4500.0,  240.0),

    ('20250616-speed-cycle',       200,   0.30,    1.1,  9600.0,  470.0),
    ('20250616-back-and-forth',    200,   0.30,    1.1, 17000.0,  470.0),
    ('20250616-bnforth-highaccel', 200,   0.30,    1.1, 18000.0,  470.0),
    ('20250616-hboard-cycle',      200,   0.35,    1.1, 13000.0,  470.0),
    ('20250616-hboard-manual',     200,   0.35,    1.1, 13000.0,  470.0),

    ('20250616-speed-cycle',       400,   0.30,    1.1, 34000.0, 1000.0),
    ('20250616-back-and-forth',    400,   0.30,    1.1, 57000.0, 1000.0),
    ('20250616-bnforth-highaccel', 400,   0.30,    1.1, 70000.0, 1000.0),
    ('20250616-hboard-cycle',      400,   0.35,    1.1, 50000.0, 1000.0),
    ('20250616-hboard-manual',     400,   0.35,    1.1, 30000.0, 1000.0),
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
        if result['position_metric'] >= pmetric:
            test_pass = False
        if result['position_metric'] < 0.5 * pmetric:
            test_pass = False
        if result['max_position_error'] >= pmax:
            test_pass = False

        if test_pass:
            tests_passed += 1
            print(f"PASS: {test_name} {result['position_metric']} {result['velocity_metric']}")
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
