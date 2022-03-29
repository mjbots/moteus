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
import os
import re
import subprocess
import tempfile
import unittest

from bazel_tools.tools.python.runfiles import runfiles

RUNFILES = runfiles.Create()
MOTEUS_TOOL = RUNFILES.Rlocation(
    "com_github_mjbots_moteus/utils/moteus_tool")
DYNAMOMETER_DRIVE = RUNFILES.Rlocation(
    "com_github_mjbots_moteus/utils/dynamometer_drive")
TORQUE_RIPPLE = RUNFILES.Rlocation(
    "com_github_mjbots_moteus/utils/dyno_static_torque_ripple")

def dyno(*args, keep_log=False):
    tmp = tempfile.NamedTemporaryFile(
        prefix='{}-moteus_firmware_validate-'.format(
            datetime.datetime.now().isoformat()),
        delete = False)

    try:
        subprocess.run(args = [DYNAMOMETER_DRIVE,
                               '--torque_transducer', '/dev/ttyUSB0',
                               '--log', tmp.name] + list(args),
                       check = True)
        if not keep_log:
            os.remove(tmp.name)
            return None
        return tmp.name
    except:
        print("Failing log: {}".format(tmp.name))
        raise


class TestDynoFast(unittest.TestCase):
    def test_validate_pwm_mode(self):
        dyno('--validate_pwm_mode', '1')

    def test_pwm_cycle_overrun(self):
        dyno('--pwm_cycle_overrun', '1')

    def test_validate_current_mode(self):
        dyno('--validate_current_mode', '1')

    def test_validate_position_basic(self):
        dyno('--validate_position_basic', '1')

    def test_validate_position_pid(self):
        dyno('--validate_position_pid', '1')

    def test_validate_position_lowspeed(self):
        dyno('--validate_position_lowspeed', '1')

    def test_validate_position_wraparound(self):
        dyno('--validate_position_wraparound', '1')

    def test_validate_stay_within(self):
        dyno('--validate_stay_within', '1')

    def test_validate_max_slip(self):
        dyno('--validate_max_slip', '1')

    def test_validate_slip_stop_position(self):
        dyno('--validate_slip_stop_position', '1')

    def test_validate_slip_bounds(self):
        dyno('--validate_slip_bounds', '1')

    def test_validate_dq_ilimit(self):
        dyno('--validate_dq_ilimit', '1')

    def test_validate_power_limit(self):
        dyno('--validate_power_limit', '1')

    def test_validate_max_velocity(self):
        dyno('--validate_max_velocity', '1')

    def test_rezero(self):
        dyno('--validate_rezero', '1')

    def test_validate_voltage_mode_control(self):
        dyno('--validate_voltage_mode_control', '1')

    def test_validate_fixed_voltage_mode(self):
        dyno('--validate_fixed_voltage_mode', '1')

    def test_validate_brake_mode(self):
        dyno('--validate_brake_mode', '1')

    def test_validate_velocity_accel_limits(self):
        dyno('--validate_velocity_accel_limits', '1')


class TestDynoSlow(unittest.TestCase):
    def test_torque_ripple(self):
        log_file = dyno('--static_torque_ripple', '1', keep_log=True)
        result = subprocess.check_output(
            '{} --skip-plot --ripple {}'.format(TORQUE_RIPPLE, log_file),
            shell=True)
        data = re.search(
            r'\*\* ANALYSIS\n(.*?)\n\n', result.decode('utf8'),
            re.DOTALL).group(1).split('\n')
        print('\n'.join(data))
        for line in data:
            fields = line.split(' ')
            torque_name = fields[0:2]
            extra = fields[2:]
            data_dict = dict([x.split('=') for x in extra])
            self.assertLess(float(data_dict['std']), 0.015)
            self.assertLess(float(data_dict['pk-pk']), 0.060)


if __name__ == '__main__':
    print("Set PS to 24V/5A!")

    unittest.main()
