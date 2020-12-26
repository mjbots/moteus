# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

'''This is a low-performance implementation of an asyncio serial
wrapper that is compatible with windows which does not allow
non-blocking serial connections.  It uses threads and busy-looping to
emulate non-blocking operations while still supporting cancellation.

'''

import ctypes
import serial
import sys
from typing import List, Optional, Union

from moteus.aiostream import *

class AioSerial(AioStream):
    def __init__(self, *args, **kwargs):
        kwargs['timeout'] = 0.0
        self.serial = serial.Serial(*args, **kwargs)

        if sys.platform.startswith('win32'):
            import win32file
            # For Windows, many drivers treat all 0's as a special case that results
            # in better timings that are not possible to achieve through any other
            # means.
            timeouts = serial.win32.COMMTIMEOUTS()
            timeouts.ReadIntervalTimeout = 0
            timeouts.ReadTotalTimeoutMultiplier = 0
            timeouts.ReadTotalTimeoutConstant = 0
            timeouts.WriteTotalTimeoutMultiplier = 0
            timeouts.WriteTotalTimeoutConstant = 0
            serial.win32.SetCommTimeouts(self.serial._port_handle, ctypes.byref(timeouts))

        super(AioSerial, self).__init__(self.serial)
