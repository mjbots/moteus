#!/usr/bin/python3 -B

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


import math
import unittest

import moteus.moteus as mot


class MoteusTest(unittest.TestCase):
    def test_make_position_empty(self):
        dut = mot.Controller()
        result = dut.make_position()
        self.assertEqual(result.data, bytes([0x01, 0x00, 0x0a]))
        self.assertEqual(result.destination, 1)
        self.assertEqual(result.source, 0)
        self.assertEqual(result.reply_required, False)

    def test_make_position(self):
        dut = mot.Controller()
        result = dut.make_position(
            position=math.nan,
            velocity=2.0,
            maximum_torque=2.0,
            stop_position=0.0,
            feedforward_torque=-.3,
            watchdog_timeout=0.0)
        self.assertEqual(result.data, bytes([
            0x01, 0x00, 0x0a,
            0x0f, 0x20,
            0x00, 0x00, 0xc0, 0x7f,
            0x00, 0x00, 0x00, 0x40,
            0x9a, 0x99, 0x99, 0xbe,
            0x0e, 0x26,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ]))

    def test_make_position_query(self):
        dut = mot.Controller()
        result = dut.make_position(query=True)
        self.assertEqual(
            result.data,
            bytes([0x01, 0x00, 0x0a,
                   0x14, 0x04, 0x00,
                   0x13, 0x0d]))
        self.assertEqual(result.reply_required, True)

        parsed = result.parse(bytes([]))
        self.assertEqual(parsed.id, 1)
        self.assertEqual(len(parsed.values), 0)

        parsed = result.parse(bytes([
            0x24, 0x04, 0x00,
            0x0a, 0x00, # mode
            0x10, 0x02, # position
            0x00, 0xfe, # velocity
            0x20, 0x00, # torque
            0x23, 0x0d,
            0x20,  # voltage
            0x30,  # temperature
            0x40,  # fault
        ]))

        self.assertEqual(
            parsed.values,
            {
                0: 10,
                1: 0.0528,
                2: -0.128,
                3: 0.32,
                13: 16.0,
                14: 48.0,
                15: 64,
            })

        self.assertEqual(repr(parsed), '1/{MODE(0x000): 10, POSITION(0x001): 0.0528, VELOCITY(0x002): -0.128, TORQUE(0x003): 0.32, VOLTAGE(0x00d): 16.0, TEMPERATURE(0x00e): 48.0, FAULT(0x00f): 64}')


if __name__ == '__main__':
    unittest.main()
