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

import unittest
import unittest.mock

import moteus.fdcanusb_device as fdcanusb
import moteus.transport as transport

from moteus.test.fdcanusb_test_base import FdcanusbTestBase
from moteus.test.mock_serial import MockSerial


class TransportTest(FdcanusbTestBase):

    def test_basic_construction_single_device(self):
        async def test():
            # Test construction with a single FdcanusbDevice
            async with fdcanusb.FdcanusbDevice() as device:
                trans = transport.Transport(device)
                self.assertIsNotNone(trans)

        self.run_async(test())

    def test_basic_construction_device_list(self):
        async def test():
            # Test construction with a list of devices
            async with fdcanusb.FdcanusbDevice() as device1:
                # Create a second mock serial for the second device
                mock_serial2 = MockSerial()
                with unittest.mock.patch('moteus.fdcanusb_device.aioserial.AioSerial',
                                        side_effect=[self.mock_serial, mock_serial2]):
                    async with fdcanusb.FdcanusbDevice() as device2:
                        trans = transport.Transport([device1, device2])
                        self.assertIsNotNone(trans)

        self.run_async(test())


if __name__ == '__main__':
    unittest.main()
