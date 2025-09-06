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
from moteus.protocol import Register

from moteus.test.fdcanusb_test_base import FdcanusbTestBase
from moteus.test.mock_serial import MockSerial


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


if __name__ == '__main__':
    unittest.main()
