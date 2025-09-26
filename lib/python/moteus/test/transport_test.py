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

import asyncio
import struct
import unittest
import unittest.mock

import moteus.fdcanusb_device as fdcanusb
import moteus.transport as transport
import moteus.transport_device as transport_device
import moteus.command as command
from moteus.protocol import Register

from moteus.test.fdcanusb_test_base import FdcanusbTestBase
from moteus.test.mock_serial import MockSerial


class MockTransportDevice:
    """Mock TransportDevice for testing broadcast frame handling."""

    def __init__(self):
        self.transaction_calls = []
        self.responses_to_return = []

    def parent(self):
        """Return None to indicate no parent device."""
        return None

    async def transaction(self, requests, request_attitude=False, force_can_check=None):
        """Record the transaction call and return mock responses."""
        self.transaction_calls.append({
            'requests': requests[:],  # Make a copy
            'request_attitude': request_attitude,
            'force_can_check': force_can_check
        })

    def add_response(self, response):
        """Add a response to be returned by the next transaction."""
        self.responses_to_return.append(response)


class TransportTest(FdcanusbTestBase):

    def test_basic_construction_single_device(self):
        async def test():
            async with fdcanusb.FdcanusbDevice() as device:
                trans = transport.Transport(device)
                self.assertIsNotNone(trans)

        self.run_async(test())

    def test_basic_construction_device_list(self):
        async def test():
            async with fdcanusb.FdcanusbDevice() as device1:
                mock_serial2 = MockSerial()
                with unittest.mock.patch('moteus.fdcanusb_device.aioserial.AioSerial',
                                        side_effect=[self.mock_serial, mock_serial2]):
                    async with fdcanusb.FdcanusbDevice() as device2:
                        trans = transport.Transport([device1, device2])
                        self.assertIsNotNone(trans)

        self.run_async(test())

    def test_discover_multiple_devices(self):
        async def test():
            mock_serial1 = MockSerial()
            mock_serial2 = MockSerial()

            # Helper to create a simple response frame without UUID
            # (devices without UUIDs are still valid and discoverable)
            def make_simple_response(can_id):
                arbitration_id = (can_id << 8) | 0x00

                # We'll send a frame that consists of a single NOP,
                # just to test that we are able to respond.
                frame = b'rcv '
                frame += f'{arbitration_id:04x}'.encode('latin1')
                frame += b' '
                frame += b'50'
                frame += b' '  # Add empty flags field
                frame += b'\n'
                return frame

            # Add write hooks to respond to the broadcast query
            def respond_to_query1(serial, data):
                if b'can send' in data:
                    # Queue OK response first
                    serial.add_response(b"OK\n")
                    # Then queue responses from devices 1 and 2 on this fdcanusb
                    serial.add_response(make_simple_response(1))
                    serial.add_response(make_simple_response(2))

            def respond_to_query2(serial, data):
                if b'can send' in data:
                    # Queue OK response first
                    serial.add_response(b"OK\n")
                    # Then queue response from device 3 on this fdcanusb
                    serial.add_response(make_simple_response(3))

            mock_serial1.add_write_hook(b'can send', respond_to_query1)
            mock_serial2.add_write_hook(b'can send', respond_to_query2)

            with unittest.mock.patch('moteus.fdcanusb_device.aioserial.AioSerial',
                                    side_effect=[mock_serial1, mock_serial2]):
                async with fdcanusb.FdcanusbDevice() as device1:
                    async with fdcanusb.FdcanusbDevice() as device2:
                        dut = transport.Transport([device1, device2])

                        devices = await dut.discover(timeout=0.5)

                        # Should find 3 devices total
                        self.assertEqual(len(devices), 3)

                        # Check device 1 and 2 are on first fdcanusb
                        dev1 = [d for d in devices if d.can_id == 1][0]
                        self.assertIsNone(dev1.uuid)
                        self.assertEqual(dev1.transport_device, device1)
                        self.assertIsNotNone(dev1.address)
                        self.assertEqual(dev1.address.can_id, 1)

                        dev2 = [d for d in devices if d.can_id == 2][0]
                        self.assertIsNone(dev2.uuid)
                        self.assertEqual(dev2.transport_device, device1)
                        self.assertIsNotNone(dev2.address)
                        self.assertEqual(dev2.address.can_id, 2)

                        # Check device 3 is on second fdcanusb
                        dev3 = [d for d in devices if d.can_id == 3][0]
                        self.assertIsNone(dev3.uuid)
                        self.assertEqual(dev3.transport_device, device2)
                        self.assertIsNotNone(dev3.address)
                        self.assertEqual(dev3.address.can_id, 3)

        self.run_async(test())

    def test_single_broadcast_with_reply(self):
        """Test that a single broadcast frame expecting a reply is sent alone."""
        async def test():
            mock_device = MockTransportDevice()
            trans = transport.Transport(mock_device)

            # Create a broadcast command that expects a reply
            cmd = command.Command()
            cmd.destination = 0x7f  # Broadcast destination
            cmd.source = 1
            cmd.reply_required = True
            cmd.data = b'\x01\x02\x03'

            # Mock a response
            response_frame = transport_device.Frame(
                arbitration_id=0x101,
                data=b'\x04\x05\x06'
            )
            mock_device.add_response([response_frame])

            # Send the command through Transport
            result = await trans.cycle([cmd])

            # Verify we got exactly one transaction call
            self.assertEqual(len(mock_device.transaction_calls), 1)

            # Verify the broadcast was sent alone
            call = mock_device.transaction_calls[0]
            self.assertEqual(len(call['requests']), 1)
            # Check broadcast bit pattern (0x7f in lower 7 bits)
            self.assertEqual(call['requests'][0].frame.arbitration_id & 0x7f, 0x7f)

        self.run_async(test())

    def test_multiple_broadcasts_with_replies(self):
        """Test that multiple broadcast frames expecting replies are sent sequentially."""
        async def test():
            mock_device = MockTransportDevice()
            trans = transport.Transport(mock_device)

            # Create two broadcast commands that expect replies
            commands = []
            for i in range(2):
                cmd = command.Command()
                cmd.destination = 0x7f  # Broadcast
                cmd.source = i + 1
                cmd.reply_required = True
                cmd.data = bytes([i, i+1, i+2])
                commands.append(cmd)

                # Add mock response for each
                response = transport_device.Frame(
                    arbitration_id=0x100 + i,
                    data=bytes([i+3, i+4])
                )
                mock_device.add_response([response])

            # Send both commands
            result = await trans.cycle(commands, request_attitude=True, force_can_check='test')

            # The implementation creates separate transactions for
            # each broadcast with reply.
            self.assertEqual(len(mock_device.transaction_calls), 2)

            # Find the transactions that contain broadcast frames
            broadcast_calls = [call for call in mock_device.transaction_calls
                             if any(req.frame.arbitration_id & 0x7f == 0x7f
                                   for req in call['requests'])]

            # Should have exactly 2 broadcast transactions
            self.assertEqual(len(broadcast_calls), 2)

            # Each broadcast transaction should have exactly 1 frame
            for call in broadcast_calls:
                self.assertEqual(len(call['requests']), 1)

            # First broadcast transaction
            self.assertEqual(broadcast_calls[0]['requests'][0].frame.data, b'\x00\x01\x02')

            # Second broadcast transaction
            self.assertEqual(broadcast_calls[1]['requests'][0].frame.data, b'\x01\x02\x03')

        self.run_async(test())

    def test_broadcast_and_non_broadcast_mixed(self):
        """Test mixing broadcast frames (with replies) and non-broadcast frames."""
        async def test():
            mock_device = MockTransportDevice()
            trans = transport.Transport(mock_device)

            commands = []

            # Add a broadcast command with reply
            broadcast_cmd = command.Command()
            broadcast_cmd.destination = 0x7f  # Broadcast
            broadcast_cmd.source = 1
            broadcast_cmd.reply_required = True
            broadcast_cmd.data = b'\xaa\xbb'
            commands.append(broadcast_cmd)

            # Add two non-broadcast commands
            for i in range(2):
                cmd = command.Command()
                cmd.destination = i + 1  # Non-broadcast
                cmd.source = i
                cmd.reply_required = True
                cmd.data = bytes([i * 10, i * 10 + 1])
                commands.append(cmd)

            # Add responses for each transaction
            mock_device.add_response([transport_device.Frame(arbitration_id=0x200, data=b'\x01')])
            mock_device.add_response([
                transport_device.Frame(arbitration_id=0x201, data=b'\x02'),
                transport_device.Frame(arbitration_id=0x202, data=b'\x03')
            ])

            # Send all commands
            result = await trans.cycle(commands, request_attitude=True, force_can_check='check')

            # Should have 2 transactions: 1 for broadcast, 1 for both non-broadcasts
            self.assertEqual(len(mock_device.transaction_calls), 2)

            # First transaction: broadcast alone
            call1 = mock_device.transaction_calls[0]
            self.assertEqual(len(call1['requests']), 1)
            self.assertEqual(call1['requests'][0].frame.arbitration_id & 0x7f, 0x7f)
            self.assertTrue(call1['request_attitude'])
            self.assertEqual(call1['force_can_check'], 'check')

            # Second transaction: both non-broadcasts together
            call2 = mock_device.transaction_calls[1]
            self.assertEqual(len(call2['requests']), 2)
            # Non-broadcast IDs shouldn't have 0x7f in lower bits
            self.assertNotEqual(call2['requests'][0].frame.arbitration_id & 0x7f, 0x7f)
            self.assertNotEqual(call2['requests'][1].frame.arbitration_id & 0x7f, 0x7f)
            self.assertFalse(call2['request_attitude'])
            self.assertIsNone(call2['force_can_check'])

        self.run_async(test())

    def test_broadcast_without_reply(self):
        """Test that broadcast frames without frame_filter are batched normally."""
        async def test():
            mock_device = MockTransportDevice()
            trans = transport.Transport(mock_device)

            commands = []

            # Add broadcast commands WITHOUT reply_required (no reply expected)
            for i in range(2):
                cmd = command.Command()
                cmd.destination = 0x7f  # Broadcast
                cmd.source = i
                cmd.reply_required = False  # No reply expected
                cmd.data = bytes([i, i+1])
                commands.append(cmd)

            # Add a non-broadcast command
            normal_cmd = command.Command()
            normal_cmd.destination = 5
            normal_cmd.source = 3
            normal_cmd.reply_required = False
            normal_cmd.data = b'\xff\xfe'
            commands.append(normal_cmd)

            mock_device.add_response([])

            # Send all commands
            result = await trans.cycle(commands, force_can_check=True)

            # Should have only 1 transaction with all 3 frames
            self.assertEqual(len(mock_device.transaction_calls), 1)

            call = mock_device.transaction_calls[0]
            self.assertEqual(len(call['requests']), 3)
            # All frames should be in the same batch
            self.assertEqual(call['requests'][0].frame.arbitration_id & 0x7f, 0x7f)
            self.assertEqual(call['requests'][1].frame.arbitration_id & 0x7f, 0x7f)
            self.assertNotEqual(call['requests'][2].frame.arbitration_id & 0x7f, 0x7f)

        self.run_async(test())

    def test_non_broadcast_frames_batched(self):
        """Verify that multiple non-broadcast frames are batched in a single transaction."""
        async def test():
            mock_device = MockTransportDevice()
            trans = transport.Transport(mock_device)

            commands = []

            # Create several non-broadcast commands
            for i in range(5):
                cmd = command.Command()
                cmd.destination = i + 1  # Non-broadcast IDs (1-5)
                cmd.source = i
                cmd.reply_required = (i % 2 == 0)  # Some with replies, some without
                cmd.data = bytes([i, i*2, i*3])
                commands.append(cmd)

            # Add mock response
            responses = [transport_device.Frame(arbitration_id=0x300 + i, data=bytes([i])) for i in range(3)]
            mock_device.add_response(responses)

            # Send all commands
            result = await trans.cycle(commands, request_attitude=True, force_can_check='batch_check')

            # Should have exactly 1 transaction with all 5 frames
            self.assertEqual(len(mock_device.transaction_calls), 1)

            call = mock_device.transaction_calls[0]
            self.assertEqual(len(call['requests']), 5)
            self.assertTrue(call['request_attitude'])
            self.assertEqual(call['force_can_check'], 'batch_check')

            # Verify all frames are non-broadcast
            for i in range(5):
                self.assertNotEqual(call['requests'][i].frame.arbitration_id & 0x7f, 0x7f)
                self.assertEqual(call['requests'][i].frame.data, bytes([i, i*2, i*3]))

        self.run_async(test())


if __name__ == '__main__':
    unittest.main()
