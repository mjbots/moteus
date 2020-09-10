#!/usr/bin/python3

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

import datetime
import subprocess
import tempfile
import unittest

from bazel_tools.tools.python.runfiles import runfiles

RUNFILES = runfiles.Create()
MOTEUS_TOOL = RUNFILES.Rlocation(
    "com_github_mjbots_moteus/moteus/tool/moteus_tool")
DYNAMOMETER_DRIVE = RUNFILES.Rlocation(
    "com_github_mjbots_moteus/moteus/tool/dynamometer_drive")

def dyno(*args):
    tmp = tempfile.NamedTemporaryFile(
        prefix='{}-moteus_firmware_validate-'.format(
            datetime.datetime.now().isoformat()),
        delete = False)

    subprocess.run(args = [DYNAMOMETER_DRIVE,
                           '--torque_transducer', '/dev/ttyUSB0',
                           '--log', tmp.name] + list(args),
                   check = True)


class TestDyno(unittest.TestCase):
    def test_validate_pwm_mode(self):
        dyno('--validate_pwm_mode', '1')


if __name__ == '__main__':
    unittest.main()
