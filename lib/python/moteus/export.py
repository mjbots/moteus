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

"""Classes and functions for interoperating with the moteus brushless
controller."""

ALL = [
    'aiostream',
    'make_transport_args', 'get_singleton_transport',
    'DeviceAddress', 'DeviceInfo',
    'Frame', 'FrameFilter', 'TransportDevice',
    'Fdcanusb', 'FdcanusbDevice', 'Controller', 'Register', 'Transport', 'TransportWrapper',
    'PythonCan', 'PythonCanDevice',
    'Mode', 'QueryResolution', 'PositionResolution', 'Command', 'CommandError',
    'Stream',
    'TRANSPORT_FACTORIES',
    'INT8', 'INT16', 'INT32', 'F32', 'IGNORE',
    'reader',
    'RegisterParser', 'QueryParser',
    'SubframeType', 'RegisterSubframe', 'ErrorSubframe', 'StreamSubframe',
    'Subframe', 'parse_frame',
    'ParsedRegisters', 'parse_registers', 'scale_register',
]
from moteus.command import Command
from moteus.device_info import DeviceAddress, DeviceInfo
from moteus.fdcanusb import Fdcanusb
from moteus.fdcanusb_device import FdcanusbDevice
from moteus.transport import Transport
from moteus.transport_wrapper import TransportWrapper
from moteus.pythoncan import PythonCan
from moteus.pythoncan_device import PythonCanDevice
from moteus.multiplex import (INT8, INT16, INT32, F32, IGNORE,
                              RegisterParser, QueryParser,
                              SubframeType, RegisterSubframe, ErrorSubframe,
                              StreamSubframe, Subframe, parse_frame)
from moteus.transport_device import Frame, FrameFilter, TransportDevice
import moteus.reader as reader
import moteus.aiostream as aiostream

from moteus.moteus import (
    CommandError,
    Controller, Register, Mode, QueryResolution, PositionResolution, Stream,
    make_transport_args, get_singleton_transport,
    TRANSPORT_FACTORIES)
from moteus.protocol import ParsedRegisters, parse_registers, scale_register

try:
    from moteus.version import VERSION
except ModuleNotFoundError:
    VERSION = "DEV"
