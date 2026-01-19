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


import asyncio
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


class MockTransport:
    """A mock transport for testing move_to."""
    def __init__(self, responses=None):
        self.responses = responses or []
        self.call_count = 0
        self.commands = []

    async def cycle(self, commands):
        self.commands.append(commands)
        if self.call_count < len(self.responses):
            result = self.responses[self.call_count]
            self.call_count += 1
            return result
        return []


class MockResult:
    """A mock result for testing."""
    def __init__(self, servo_id, position=0.0, velocity=0.0, mode=10,
                 trajectory_complete=False, fault=0):
        self.id = servo_id
        self.arbitration_id = (servo_id << 8) | 0x100
        self.values = {
            mot.Register.MODE: mode,
            mot.Register.POSITION: position,
            mot.Register.VELOCITY: velocity,
            mot.Register.TRAJECTORY_COMPLETE: 1 if trajectory_complete else 0,
            mot.Register.FAULT: fault,
        }


class MoveToTest(unittest.TestCase):
    def test_move_to_single_servo_requires_position(self):
        """Test that single servo case requires position parameter."""
        controller = mot.Controller(id=1)

        async def test():
            with self.assertRaises(ValueError) as ctx:
                await mot.move_to(controller)
            self.assertIn("position required", str(ctx.exception))

        asyncio.run(test())

    def test_move_to_empty_list(self):
        """Test that empty target list returns empty result."""

        async def test():
            result = await mot.move_to([])
            self.assertEqual(result, [])

        asyncio.run(test())

    def test_move_to_single_servo_complete(self):
        """Test single servo move_to completes when trajectory_complete."""

        mock_transport = MockTransport(responses=[
            # First call (count=2->1): trajectory not complete
            [MockResult(1, position=0.5, trajectory_complete=False)],
            # Second call (count=1->0): trajectory still not complete
            [MockResult(1, position=0.8, trajectory_complete=False)],
            # Third call: trajectory complete
            [MockResult(1, position=1.0, trajectory_complete=True)],
        ])

        controller = mot.Controller(id=1, transport=mock_transport)

        async def test():
            result = await mot.move_to(controller, position=1.0)
            self.assertIsNotNone(result)
            self.assertEqual(result.id, 1)
            self.assertEqual(result.values[mot.Register.POSITION], 1.0)
            self.assertEqual(result.values[mot.Register.TRAJECTORY_COMPLETE], 1)
            self.assertEqual(mock_transport.call_count, 3)

        asyncio.run(test())

    def test_move_to_multi_servo_complete(self):
        """Test multi-servo move_to completes when all trajectories complete."""

        mock_transport = MockTransport(responses=[
            # First call: neither complete
            [MockResult(1, position=0.5, trajectory_complete=False),
             MockResult(2, position=-0.2, trajectory_complete=False)],
            # Second call: servo 1 complete, servo 2 not
            [MockResult(1, position=1.0, trajectory_complete=True),
             MockResult(2, position=-0.3, trajectory_complete=False)],
            # Third call: both complete
            [MockResult(1, position=1.2, trajectory_complete=True),
             MockResult(2, position=-0.5, trajectory_complete=True)],
        ])

        c1 = mot.Controller(id=1, transport=mock_transport)
        c2 = mot.Controller(id=2, transport=mock_transport)

        async def test():
            results = await mot.move_to([
                (c1, 1.0),
                (c2, -0.5),
            ])
            self.assertEqual(len(results), 2)

            # Results should be in same order as input
            self.assertEqual(results[0][0].id, 1)
            self.assertEqual(results[1][0].id, 2)
            self.assertEqual(results[0][1].values[mot.Register.TRAJECTORY_COMPLETE], 1)
            self.assertEqual(results[1][1].values[mot.Register.TRAJECTORY_COMPLETE], 1)
            self.assertEqual(results[0][1].values[mot.Register.POSITION], 1.2)
            self.assertEqual(results[1][1].values[mot.Register.POSITION], -0.5)

        asyncio.run(test())

    def test_move_to_with_duration(self):
        """Test move_to with duration parameter queries positions first."""

        mock_transport = MockTransport(responses=[
            # Initial query to get current positions
            [MockResult(1, position=0.0, trajectory_complete=False)],
            # First command cycle (count=2->1)
            [MockResult(1, position=0.5, trajectory_complete=True)],
            # Second command cycle (count=1->0)
            [MockResult(1, position=1.0, trajectory_complete=True)],
        ])

        controller = mot.Controller(id=1, transport=mock_transport)

        async def test():
            result = await mot.move_to(
                controller, position=1.0, duration=2.0)
            self.assertIsNotNone(result)
            # Should have made 3 calls: query + 2 position commands
            cmd_registers = mot.parse_registers(mock_transport.commands[1][0].data)
            self.assertEqual(cmd_registers.command.get(
                mot.Register.COMMAND_VELOCITY_LIMIT, None), 0.5)
            self.assertEqual(mock_transport.call_count, 3)

        asyncio.run(test())

    def test_move_to_fault_raises_error(self):
        """Test that move_to raises FaultError on servo fault."""

        mock_transport = MockTransport(responses=[
            # Servo reports fault
            [MockResult(1, position=0.5, mode=1, fault=5)],  # mode 1 = FAULT
        ])

        controller = mot.Controller(id=1, transport=mock_transport)

        async def test():
            with self.assertRaises(mot.FaultError):
                await mot.move_to(controller, position=1.0)

        asyncio.run(test())

    def test_move_to_timeout(self):
        """Test that move_to raises TimeoutError on timeout."""

        # Transport that never completes
        mock_transport = MockTransport(responses=[
            [MockResult(1, position=0.5, trajectory_complete=False)]
            for _ in range(100)  # Many incomplete responses
        ])

        controller = mot.Controller(id=1, transport=mock_transport)

        async def test():
            with self.assertRaises(asyncio.TimeoutError):
                await mot.move_to(
                    controller, position=1.0, timeout=0.05, period_s=0.01)

        asyncio.run(test())

    def test_move_to_nan_position_not_waited(self):
        """Test that NaN positions are commanded but not waited on."""

        mock_transport = MockTransport(responses=[
            # First call (count=2->1): servo 1 not complete
            [MockResult(1, position=0.5, trajectory_complete=False),
             MockResult(2, position=0.0, trajectory_complete=False)],
            # Second call (count=1->0): servo 1 still not complete
            [MockResult(1, position=0.8, trajectory_complete=False),
             MockResult(2, position=0.0, trajectory_complete=False)],
            # Third call: servo 1 complete - should finish even though
            # servo 2 never reported trajectory_complete
            [MockResult(1, position=1.0, trajectory_complete=True),
             MockResult(2, position=0.0, trajectory_complete=False)],
        ])

        c1 = mot.Controller(id=1, transport=mock_transport)
        c2 = mot.Controller(id=2, transport=mock_transport)

        async def test():
            results = await mot.move_to([
                (c1, 1.0),        # Move to 1.0 and wait
                (c2, math.nan),   # Hold position, don't wait
            ])
            # Should complete after servo 1 finishes, regardless of servo 2
            self.assertEqual(len(results), 2)
            self.assertEqual(results[0][0].id, 1)
            self.assertEqual(results[1][0].id, 2)
            # Servo 1 should show complete
            self.assertEqual(results[0][1].values[mot.Register.TRAJECTORY_COMPLETE], 1)
            # Servo 2 never completed but we got a result
            self.assertEqual(results[1][1].values[mot.Register.TRAJECTORY_COMPLETE], 0)
            self.assertEqual(mock_transport.call_count, 3)

        asyncio.run(test())

    def test_move_to_all_nan_completes_immediately(self):
        """Test that if all targets are NaN, function completes after two cycles."""

        mock_transport = MockTransport(responses=[
            # First cycle (count=2->1)
            [MockResult(1, position=0.0, trajectory_complete=False),
             MockResult(2, position=0.0, trajectory_complete=False)],
            # Second cycle (count=1->0)
            [MockResult(1, position=0.0, trajectory_complete=False),
             MockResult(2, position=0.0, trajectory_complete=False)],
        ])

        c1 = mot.Controller(id=1, transport=mock_transport)
        c2 = mot.Controller(id=2, transport=mock_transport)

        async def test():
            results = await mot.move_to([
                (c1, math.nan),
                (c2, math.nan),
            ])
            # Should complete after two cycles (count=2 mechanism)
            self.assertEqual(len(results), 2)
            self.assertEqual(mock_transport.call_count, 2)

        asyncio.run(test())

    def test_move_to_with_setpoint_object(self):
        """Test move_to with Setpoint object for velocity control."""

        mock_transport = MockTransport(responses=[
            # First cycle (count=2->1)
            [MockResult(1, position=0.5, trajectory_complete=True)],
            # Second cycle (count=1->0)
            [MockResult(1, position=1.0, trajectory_complete=True)],
        ])

        controller = mot.Controller(id=1, transport=mock_transport)

        async def test():
            # Use Setpoint object with velocity
            setpoint = mot.Setpoint(position=1.0, velocity=0.5)
            result = await mot.move_to([
                (controller, setpoint),
            ])
            self.assertEqual(len(result), 1)
            # Verify the command was built (we can check that it was called)
            self.assertEqual(mock_transport.call_count, 2)

        asyncio.run(test())

    def test_move_to_mixed_setpoint_types(self):
        """Test move_to with mix of numbers, NaN, and Setpoint objects."""

        mock_transport = MockTransport(responses=[
            # First cycle: c1 not complete
            [MockResult(1, position=0.5, trajectory_complete=False),
             MockResult(2, position=0.0, trajectory_complete=False),
             MockResult(3, position=-0.2, trajectory_complete=False)],
            # Second cycle: all that matter are complete
            [MockResult(1, position=1.0, trajectory_complete=True),
             MockResult(2, position=0.0, trajectory_complete=False),
             MockResult(3, position=-0.5, trajectory_complete=True)],
        ])

        c1 = mot.Controller(id=1, transport=mock_transport)
        c2 = mot.Controller(id=2, transport=mock_transport)
        c3 = mot.Controller(id=3, transport=mock_transport)

        async def test():
            results = await mot.move_to([
                (c1, 1.0),                                    # Plain number
                (c2, math.nan),                               # NaN - hold position
                (c3, mot.Setpoint(position=-0.5, velocity=0.2)), # Setpoint object
            ])
            self.assertEqual(len(results), 3)
            self.assertEqual(results[0][0].id, 1)
            self.assertEqual(results[1][0].id, 2)
            self.assertEqual(results[2][0].id, 3)

            # Verify the commands sent in the first cycle
            first_cycle_cmds = mock_transport.commands[0]
            self.assertEqual(len(first_cycle_cmds), 3)

            # c1: plain number position=1.0
            c1_regs = mot.parse_registers(first_cycle_cmds[0].data)
            self.assertAlmostEqual(
                c1_regs.command.get(mot.Register.COMMAND_POSITION), 1.0, places=4)
            self.assertNotIn(mot.Register.COMMAND_VELOCITY, c1_regs.command)

            # c2: NaN position (hold)
            c2_regs = mot.parse_registers(first_cycle_cmds[1].data)
            self.assertTrue(math.isnan(c2_regs.command.get(mot.Register.COMMAND_POSITION)))

            # c3: Setpoint with position=-0.5, velocity=0.2
            c3_regs = mot.parse_registers(first_cycle_cmds[2].data)
            self.assertAlmostEqual(
                c3_regs.command.get(mot.Register.COMMAND_POSITION), -0.5, places=4)
            self.assertAlmostEqual(
                c3_regs.command.get(mot.Register.COMMAND_VELOCITY), 0.2, places=4)

        asyncio.run(test())


if __name__ == '__main__':
    unittest.main()
