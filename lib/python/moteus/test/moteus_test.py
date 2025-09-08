#!/usr/bin/python3 -B

# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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
from moteus.device_info import DeviceAddress


class CanMessage:
    def __init__(self, arbitration_id=0x0100, data=b''):
        self.arbitration_id = arbitration_id
        self.data = data


class MoteusTest(unittest.TestCase):
    def test_make_position_empty(self):
        dut = mot.Controller()
        result = dut.make_position()
        self.assertEqual(result.data, bytes([0x01, 0x00, 0x0a]))
        self.assertEqual(result.destination, 1)
        self.assertEqual(result.source, 0)
        self.assertEqual(result.reply_required, False)
        self.assertEqual(result.expected_reply_size, 0)

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
            0x0f, 0x25,
            0x00, 0x00, 0x00, 0x40,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ]))
        self.assertEqual(result.expected_reply_size, 0)

        result = dut.make_position(
            position=math.nan, velocity=1, maximum_torque=0.0, stop_position=0.15)
        self.assertEqual(result.data, bytes([
            0x01, 0x00, 0x0a,
            0x0e, 0x20,
            0x00, 0x00, 0xc0, 0x7f,
            0x00, 0x00, 0x80, 0x3f,
            0x0e, 0x25,
            0x00, 0x00, 0x00, 0x00,
            0x9a, 0x99, 0x19, 0x3e,
        ]))
        self.assertEqual(result.expected_reply_size, 0)

    def test_make_position_query(self):
        dut = mot.Controller()
        result = dut.make_position(query=True)
        self.assertEqual(
            result.data,
            bytes([0x01, 0x00, 0x0a,
                   0x11, 0x00,
                   0x1f, 0x01,
                   0x13, 0x0d]))
        self.assertEqual(result.reply_required, True)
        self.assertEqual(result.expected_reply_size, 22)

        parsed = result.parse(CanMessage())
        self.assertEqual(parsed.id, 1)
        self.assertEqual(len(parsed.values), 0)

        parsed = result.parse(CanMessage(data=bytes([
            0x24, 0x04, 0x00,
            0x0a, 0x00, # mode
            0x10, 0x02, # position
            0x00, 0xfe, # velocity
            0x20, 0x00, # torque
            0x23, 0x0d,
            0x20,  # voltage
            0x30,  # temperature
            0x40,  # fault
        ])))

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

    def test_make_position_query_all(self):
        dut = mot.Controller()
        qr = mot.QueryResolution()
        qr.mode = mot.mp.INT16
        qr.position = mot.mp.INT16
        qr.velocity = mot.mp.INT16
        qr.torque = mot.mp.INT16
        qr.q_current = mot.mp.INT16
        qr.d_current = mot.mp.INT16
        qr.abs_position = mot.mp.INT16
        qr.power = mot.mp.INT16
        qr.motor_temperature = mot.mp.INT8
        qr.trajectory_complete = mot.mp.INT8
        qr.home_state = mot.mp.INT8
        qr.voltage = mot.mp.INT8
        qr.temperature = mot.mp.INT8
        qr.fault = mot.mp.INT8
        qr.aux1_gpio = mot.mp.INT8
        qr.aux2_gpio = mot.mp.INT8

        result = dut.make_position(query=True, query_override=qr)
        self.assertEqual(
            result.data.hex(),
            bytes([0x01, 0x00, 0x0a,
                   0x14, 0x08, 0x00,
                   0x10, 0x06, 0x0a,
                   0x12, 0x5e]).hex())
        self.assertEqual(result.reply_required, True)
        self.assertEqual(result.expected_reply_size, 32)

        parsed = result.parse(CanMessage())
        self.assertEqual(parsed.id, 1)
        self.assertEqual(len(parsed.values), 0)

        parsed = result.parse(CanMessage(data=bytes([
            0x24, 0x08, 0x00,
            0x0a, 0x00, # mode
            0x10, 0x02, # position
            0x00, 0xfe, # velocity
            0x20, 0x00, # torque
            0x30, 0x00, # q current
            0x40, 0x00, # d current
            0x50, 0x00, # abs position
            0x60, 0x00, # power
            0x20, 0x06, 0x0a,
            0x10,  # motor temp
            0x01,  # trajectory complete
            0x01,  # home state
            0x20,  # voltage
            0x30,  # temperature
            0x40,  # fault
        ])))

        self.assertEqual(
            parsed.values,
            {
                0: 10,
                1: 0.0528,
                2: -0.128,
                3: 0.32,
                4: 4.800000000000001,
                5: 6.4,
                6: 0.008,
                7: 4.800000000000001,
                10: 16.0,
                11: 1,
                12: 1,
                13: 16.0,
                14: 48.0,
                15: 64,
            })

        self.assertEqual(repr(parsed), '1/{MODE(0x000): 10, POSITION(0x001): 0.0528, VELOCITY(0x002): -0.128, TORQUE(0x003): 0.32, Q_CURRENT(0x004): 4.800000000000001, D_CURRENT(0x005): 6.4, ABS_POSITION(0x006): 0.008, POWER(0x007): 4.800000000000001, MOTOR_TEMPERATURE(0x00a): 16.0, TRAJECTORY_COMPLETE(0x00b): 1, REZERO_STATE(0x00c): 1, VOLTAGE(0x00d): 16.0, TEMPERATURE(0x00e): 48.0, FAULT(0x00f): 64}')

    def test_make_position_query_override(self):
        qr = mot.QueryResolution()
        qr.trajectory_complete = mot.mp.INT8

        dut = mot.Controller()
        result = dut.make_position(query_override=qr)

        self.assertEqual(
            result.data,
            bytes([0x01, 0x00, 0x0a,
                   0x11, 0x00,
                   0x1f, 0x01,
                   0x11, 0x0b,
                   0x13, 0x0d]))
        self.assertEqual(result.reply_required, True)
        self.assertEqual(result.expected_reply_size, 25)

    def test_make_query(self):
        qr = mot.QueryResolution()
        qr._extra = {
            0x030 : mot.mp.F32,
            0x031 : mot.mp.F32,
            0x032 : mot.mp.F32,
            0x033 : mot.mp.F32,
        }

        dut = mot.Controller(query_resolution=qr)
        result = dut.make_query()
        self.assertEqual(
            result.data,
            bytes([0x11, 0x00,
                   0x1f, 0x01,
                   0x13, 0x0d,
                   0x1c, 0x04, 0x30]))
        self.assertEqual(result.reply_required, True)
        self.assertEqual(result.expected_reply_size, 41)

    def test_make_query2(self):
        qr = mot.QueryResolution()
        qr.voltage = mot.mp.F32
        qr.temperature = mot.mp.F32
        qr.fault = mot.mp.INT8

        dut = mot.Controller(query_resolution=qr)
        result = dut.make_query()
        self.assertEqual(
            result.data,
            bytes([0x11, 0x00,
                   0x1f, 0x01,
                   0x1e, 0x0d,
                   0x11, 0x0f]))
        self.assertEqual(result.reply_required, True)
        self.assertEqual(result.expected_reply_size, 30)

    def test_make_diagnostic_read(self):
        dut = mot.Controller()
        result = dut.make_diagnostic_read()
        self.assertEqual(
            result.data,
            bytes([0x42, 0x01, 0x30]))
        self.assertEqual(result.reply_required, True)
        self.assertEqual(result.expected_reply_size, 51)

        parsed = result.parse(CanMessage())
        self.assertEqual(parsed.id, 1)
        self.assertEqual(parsed.data, None)

        parsed = result.parse(CanMessage(data=bytes([
            0x41, 0x01, 0x04,
            0x44, 0x45, 0x46, 0x47])))
        self.assertEqual(parsed.id, 1)
        self.assertEqual(parsed.data, b'DEFG')

    def test_make_vfoc(self):
        dut = mot.Controller()
        result = dut.make_vfoc(theta = math.pi, voltage=2.0, theta_rate = 2 * math.pi)
        self.assertEqual(
            result.data,
            bytes([0x01, 0x00, 0x07,
                   0x0e, 0x18,
                   0x00, 0x00, 0x80, 0x3f,
                   0x00, 0x00, 0x00, 0x40,
                   0x0d, 0x1e,
                   0x00, 0x00, 0x00, 0x40,
            ]))
        self.assertEqual(result.expected_reply_size, 0)

    def test_make_current(self):
        dut = mot.Controller()
        result = dut.make_current(d_A = 1.0, q_A = 2.0)
        self.assertEqual(
            result.data,
            bytes([0x01, 0x00, 0x09,
                   0x0e, 0x1c,
                   0x00, 0x00, 0x00, 0x40,
                   0x00, 0x00, 0x80, 0x3f,
            ]))
        self.assertEqual(result.expected_reply_size, 0)

    def test_make_brake(self):
        dut = mot.Controller()
        result = dut.make_brake()
        self.assertEqual(
            result.data,
            bytes([0x01, 0x00, 0x0f]))
        self.assertEqual(result.expected_reply_size, 0)

    def test_make_set_output_nearest(self):
        dut = mot.Controller()
        result = dut.make_set_output_nearest(position=2)
        self.assertEqual(
            result.data,
            bytes([0x0d, 0xb0, 0x02, 0x00, 0x00, 0x00, 0x40]))
        self.assertEqual(result.expected_reply_size, 0)

    def test_make_set_output_exact(self):
        dut = mot.Controller()
        result = dut.make_set_output_exact(position=2)
        self.assertEqual(
            result.data,
            bytes([0x0d, 0xb1, 0x02, 0x00, 0x00, 0x00, 0x40]))
        self.assertEqual(result.expected_reply_size, 0)

    def test_make_rezero(self):
        dut = mot.Controller()
        result = dut.make_rezero(rezero=2)
        self.assertEqual(
            result.data,
            bytes([0x0d, 0xb0, 0x02, 0x00, 0x00, 0x00, 0x40]))
        self.assertEqual(result.expected_reply_size, 0)

    def test_make_require_reindex(self):
        dut = mot.Controller()
        result = dut.make_require_reindex()
        self.assertEqual(
            result.data,
            bytes([0x01, 0xb2, 0x02, 0x01]))
        self.assertEqual(result.expected_reply_size, 0)

    def test_make_write_gpio(self):
        dut = mot.Controller()
        result = dut.make_write_gpio(aux1=3, aux2=5)
        self.assertEqual(
            result.data,
            bytes([0x02, 0x5c, 0x03, 0x05]))
        self.assertEqual(result.expected_reply_size, 0)

    def test_make_read_gpio(self):
        dut = mot.Controller()
        result = dut.make_read_gpio()
        self.assertEqual(
            result.data,
            bytes([0x12, 0x5e]))
        self.assertEqual(result.expected_reply_size, 22)

    def test_make_set_trim(self):
        dut = mot.Controller()
        result = dut.make_set_trim(trim=2)
        self.assertEqual(
            result.data.hex(),
            "097102000000")
        self.assertEqual(result.expected_reply_size, 0)

    def test_make_aux_pwm(self):
        dut = mot.Controller()
        result = dut.make_aux_pwm(aux1_pwm2=0.2,
                                  aux2_pwm4=0.4)
        self.assertEqual(
            result.data.hex(),
            "05779919057e3233")
        self.assertEqual(result.expected_reply_size, 0)

    def test_query_extra_disjoint(self):
        qr = mot.QueryResolution()
        qr._extra = {
            0x030 : mot.mp.F32,
            0x031 : mot.mp.F32,
            0x033 : mot.mp.F32,
            0x034 : mot.mp.F32,
            0x035 : mot.mp.F32,
            0x036 : mot.mp.F32,
        }

        dut = mot.Controller(query_resolution=qr)
        result = dut.make_query()
        self.assertEqual(
            result.data,
            bytes([
                0x11, 0x00,
                0x1f, 0x01,
                0x13, 0x0d,
                0x1e, 0x30,
                0x1c, 0x04, 0x33, ]))
        self.assertEqual(result.expected_reply_size, 51)

    def test_make_command_with_uuid_prefix(self):
        # Create a DeviceAddress with only a UUID (4-byte prefix)
        test_uuid = bytes.fromhex('01020304')
        device_address = DeviceAddress(uuid=test_uuid)

        dut = mot.Controller(id=device_address)
        result = dut.make_stop()

        # The result should use a DeviceAddress
        self.assertTrue(isinstance(result.destination, DeviceAddress))
        self.assertEqual(result.destination.can_id, None)
        self.assertEqual(result.destination.uuid, test_uuid)

        # Check for the exact UUID prefix that gets emitted
        expected_prefix_hex = "09d40201020304"
        actual_hex = result.data.hex()

        self.assertTrue(actual_hex.startswith(expected_prefix_hex),
                       f"Expected data to start with {expected_prefix_hex}, but got {actual_hex}")

    def test_source_can_id(self):
        # Test that a non-zero source CAN ID can be set and is used in
        # commands
        dut = mot.Controller(source_can_id=0x42)
        result = dut.make_stop()

        self.assertEqual(result.source, 0x42)


if __name__ == '__main__':
    unittest.main()
