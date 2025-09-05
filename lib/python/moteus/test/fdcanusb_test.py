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
import collections
import unittest
import unittest.mock

import moteus
import moteus.fdcanusb_device as fdcanusb


class MockSerial:
    def __init__(self):
        self.write_data = b''
        self.read_buffer = b''
        self.is_open = True

        self._read_waiters = []
        self._response_queue = collections.deque()

        # List of (pattern, callback) tuples
        self._write_hooks = []

    def add_write_hook(self, pattern, callback):
        """Add a hook that triggers when a write matches the pattern."""
        self._write_hooks.append((pattern, callback))

    def write(self, data):
        self.write_data += data

        # Check write hooks first, iterate over a copy in case a
        # handler modifies it.
        new_hooks = []
        for pattern, callback in self._write_hooks[:]:
            if pattern in data:
                callback(self, data)
            else:
                # By default, only execute hooks once.
                new_hooks.append((pattern, callback))

        self._write_hooks = new_hooks

        if data.startswith(b'can send'):
            if self._response_queue:
                receive_frame = self._response_queue.popleft()
                self.read_buffer += receive_frame
                self._notify_waiters()

    async def drain(self):
        pass

    async def read(self, size, block=True):
        while True:
            # If we are blocking, then we need the full data
            if ((self.read_buffer and not block) or
                len(self.read_buffer) > size):
                break

            # Wait for more data
            waiter = asyncio.Future()
            self._read_waiters.append(waiter)
            try:
                await waiter
            except asyncio.CancelledError:
                return b''

        to_return = self.read_buffer[:size]
        self.read_buffer = self.read_buffer[size:]
        return to_return

    def close(self):
        self.is_open = False
        for waiter in self._read_waiters:
            if not waiter.done():
                waiter.cancel()

        self._read_waiters.clear()

    def add_response(self, data):
        self.read_buffer += data
        self._notify_waiters()

    def queue_response(self, data):
        self._response_queue.append(data)

    def _notify_waiters(self):
        waiters_to_notify = [w for w in self._read_waiters if not w.done()]
        self._read_waiters.clear()
        for waiter in waiters_to_notify:
            waiter.set_result(None)


class FdcanusbTest(unittest.TestCase):
    def setUp(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        self.mock_serial = MockSerial()

        self.aioserial_patcher = unittest.mock.patch('moteus.fdcanusb_device.aioserial.AioSerial')
        self.mock_aioserial = self.aioserial_patcher.start()
        self.mock_aioserial.return_value = self.mock_serial

        self.detect_patcher = unittest.mock.patch('moteus.fdcanusb_device.FdcanusbDevice.detect_fdcanusb')
        self.mock_detect = self.detect_patcher.start()
        self.mock_detect.return_value = '/dev/mock_fdcanusb'

    def tearDown(self):
        pending = asyncio.all_tasks(self.loop)
        for task in pending:
            if not task.done():
                task.cancel()

        if pending:
            self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))

        self.aioserial_patcher.stop()
        self.detect_patcher.stop()
        self.loop.close()

    def run_async(self, coro):
        return self.loop.run_until_complete(asyncio.wait_for(coro, timeout=5.0))

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

                expected_cmd = b"can send 0105 30313233\n"
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
