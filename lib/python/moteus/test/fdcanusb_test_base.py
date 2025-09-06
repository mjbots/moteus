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
import unittest.mock

from moteus.test.mock_serial import MockSerial


class FdcanusbTestBase(unittest.TestCase):
    """Base test class with common setup for fdcanusb-based tests."""
    
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