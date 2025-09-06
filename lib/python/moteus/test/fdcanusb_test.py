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
import unittest

import moteus
import moteus.fdcanusb_device as fdcanusb

from moteus.test.fdcanusb_test_base import FdcanusbTestBase


class FdcanusbTest(FdcanusbTestBase):

    def test_basic_construction(self):
        async def test():
            async with moteus.Fdcanusb() as dut:
                self.assertIsNotNone(dut)
        self.run_async(test())

    def test_send_basic(self):
        async def test():
            async with fdcanusb.FdcanusbDevice() as dut:
                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                self.mock_serial.queue_response(b"OK\n")

                await dut.send_frame(frame)

                expected_cmd = b"can send 0105 30313233 b\n"
                self.assertEqual(self.mock_serial.write_data, expected_cmd)

        self.run_async(test())

    def test_send_with_flags(self):
        async def test():
            async with fdcanusb.FdcanusbDevice() as dut:
                frame = fdcanusb.Frame(0x105, b'0123', is_extended_id=True,
                                       is_fd=True, bitrate_switch=True)
                self.mock_serial.queue_response(b"OK\n")
                await dut.send_frame(frame)

                flags = sorted(self.mock_serial.write_data.decode('latin1').split(' ')[-1].strip())
                self.assertEqual(flags, ['B', 'F'])
        self.run_async(test())

    def test_subscription_basic(self):
        async def test():
            async with fdcanusb.FdcanusbDevice() as dut:

                received_frames = []

                async def handler(frame):
                    received_frames.append(frame)

                def filter_func(frame):
                    return (frame.arbitration_id & 0xFF00) == 0x100

                sub = dut.subscribe(filter_func, handler)

                self.mock_serial.add_response(b"rcv 0105 01 F\n") # matches
                self.mock_serial.add_response(b"rcv 0205 02 F\n") # no match
                self.mock_serial.add_response(b"rcv 0110 03 F\n") # matches

                await asyncio.sleep(0.02)

                self.assertEqual(len(received_frames), 2)
                self.assertEqual(received_frames[0].arbitration_id, 0x105)
                self.assertEqual(received_frames[1].arbitration_id, 0x110)

                sub.cancel()
        self.run_async(test())

    def test_transaction(self):
        async def test():
            async with fdcanusb.FdcanusbDevice() as dut:
                self.mock_serial.queue_response(b"OK\n")
                self.mock_serial.queue_response(b"OK\n")

                def queue_responses(serial, data):
                    # Send the responses in a different order from the
                    # commands.  They should get sorted out when the
                    # final results come in.
                    serial.add_response(b"rcv 0186 240400 F\n")
                    serial.add_response(b"rcv 0185 240401 F\n")

                # Send our responses after the second frame is sent.
                self.mock_serial.add_write_hook(
                    b'can send 0106', queue_responses)

                requests = [
                    (fdcanusb.Frame(0x105, b'4567'), lambda f: f.arbitration_id == 0x185),
                    (fdcanusb.Frame(0x106, b'5678'), lambda f: f.arbitration_id == 0x186),
                ]

                results = await dut.transaction(requests, timeout=None)

                self.assertEqual(len(results), 2)
                self.assertIsNotNone(results[0])
                self.assertIsNotNone(results[1])
                self.assertEqual(results[0].arbitration_id, 0x185)
                self.assertEqual(results[1].arbitration_id, 0x186)

        self.run_async(test())


if __name__ == '__main__':
    unittest.main()
