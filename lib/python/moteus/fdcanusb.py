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

import typing

from .fdcanusb_device import FdcanusbDevice
from .transport_wrapper import TransportWrapper

class Fdcanusb(TransportWrapper):
    def __init__(self, path=None, *args, **kwargs):
        if path is None:
            path = Fdcanusb.detect_fdcanusb()

        device = FdcanusbDevice(path, *args, **kwargs)
        super().__init__(device)

    @staticmethod
    def detect_fdcanusb():
        return FdcanusbDevice.detect_fdcanusb()
