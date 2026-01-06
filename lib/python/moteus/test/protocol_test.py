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

import math
import unittest

import moteus.protocol as protocol
import moteus.multiplex as mp


class ProtocolTest(unittest.TestCase):
    def test_scale_register_position(self):
        # Position uses 0.01, 0.0001, 0.00001 scales
        self.assertAlmostEqual(
            protocol.scale_register(protocol.Register.POSITION, mp.INT8, 100),
            1.0)  # 100 * 0.01
        self.assertAlmostEqual(
            protocol.scale_register(protocol.Register.POSITION, mp.INT16, 10000),
            1.0)  # 10000 * 0.0001
        self.assertAlmostEqual(
            protocol.scale_register(protocol.Register.POSITION, mp.INT32, 100000),
            1.0)  # 100000 * 0.00001

    def test_scale_register_velocity(self):
        # Velocity uses 0.1, 0.00025, 0.00001 scales
        self.assertAlmostEqual(
            protocol.scale_register(protocol.Register.VELOCITY, mp.INT8, 10),
            1.0)  # 10 * 0.1
        self.assertAlmostEqual(
            protocol.scale_register(protocol.Register.VELOCITY, mp.INT16, 4000),
            1.0)  # 4000 * 0.00025

    def test_scale_register_torque(self):
        # Torque uses 0.5, 0.01, 0.001 scales
        self.assertAlmostEqual(
            protocol.scale_register(protocol.Register.TORQUE, mp.INT8, 2),
            1.0)  # 2 * 0.5
        self.assertAlmostEqual(
            protocol.scale_register(protocol.Register.TORQUE, mp.INT16, 100),
            1.0)  # 100 * 0.01

    def test_scale_register_nan(self):
        # NaN is represented as the minimum value for each type
        self.assertTrue(math.isnan(
            protocol.scale_register(protocol.Register.POSITION, mp.INT8, -128)))
        self.assertTrue(math.isnan(
            protocol.scale_register(protocol.Register.POSITION, mp.INT16, -32768)))
        self.assertTrue(math.isnan(
            protocol.scale_register(protocol.Register.POSITION, mp.INT32, -2147483648)))

    def test_scale_register_none(self):
        # None input should return None
        self.assertIsNone(protocol.scale_register(protocol.Register.POSITION, mp.INT8, None))

    def test_scale_register_command_position(self):
        # Command position uses same scales as position
        self.assertAlmostEqual(
            protocol.scale_register(protocol.Register.COMMAND_POSITION, mp.INT16, 10000),
            1.0)

    def test_parse_registers_response(self):
        # 0x25 = REPLY INT16, count 1, register 0x01 (POSITION), value 100
        data = bytes([0x25, 0x01, 0x64, 0x00])
        result = protocol.parse_registers(data)

        self.assertIsInstance(result, protocol.ParsedRegisters)
        self.assertEqual(len(result.command), 0)
        self.assertEqual(len(result.response), 1)
        self.assertEqual(len(result.query), 0)
        self.assertEqual(len(result.errors), 0)

        # Position 100 * 0.0001 (INT16 scale) = 0.01
        self.assertAlmostEqual(result.response[1], 0.01)

    def test_parse_registers_command(self):
        # 0x05 = WRITE INT16, count 1, register 0x20 (COMMAND_POSITION), value 200
        data = bytes([0x05, 0x20, 0xc8, 0x00])
        result = protocol.parse_registers(data)

        self.assertEqual(len(result.command), 1)
        self.assertEqual(len(result.response), 0)

        # Command position 200 * 0.0001 = 0.02
        self.assertAlmostEqual(result.command[0x20], 0.02)

    def test_parse_registers_query(self):
        # 0x11 = READ INT8, count 1, register 0x01
        data = bytes([0x11, 0x01])
        result = protocol.parse_registers(data)

        self.assertEqual(len(result.command), 0)
        self.assertEqual(len(result.response), 0)
        self.assertEqual(len(result.query), 1)
        self.assertEqual(result.query[0], (0x01, mp.INT8))

    def test_parse_registers_error(self):
        # 0x30 = WRITE_ERROR, register 0x05, error code 0x01
        data = bytes([0x30, 0x05, 0x01])
        result = protocol.parse_registers(data)

        self.assertEqual(len(result.errors), 1)
        self.assertEqual(result.errors[0], (0x05, 0x01))

    def test_parse_registers_mixed(self):
        # Response followed by query
        # 0x21 = REPLY INT8, count 1, register 0x00 (MODE), value 10
        # 0x11 = READ INT8, count 1, register 0x01
        data = bytes([0x21, 0x00, 0x0a, 0x11, 0x01])
        result = protocol.parse_registers(data)

        self.assertEqual(len(result.response), 1)
        self.assertEqual(result.response[0x00], 10)  # MODE is int
        self.assertEqual(len(result.query), 1)
        self.assertEqual(result.query[0], (0x01, mp.INT8))


if __name__ == '__main__':
    unittest.main()
