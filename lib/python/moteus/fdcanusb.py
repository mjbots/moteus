# Copyright 2020-2021 Josh Pieper, jjp@pobox.com.
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
import glob
import os
import serial
import serial.tools
import serial.tools.list_ports
import sys
import time

from . import aioserial


def _hexify(data):
    return ''.join(['{:02X}'.format(x) for x in data])


def _dehexify(data):
    result = b''
    for i in range(0, len(data), 2):
        result += bytes([int(data[i:i + 2], 16)])
    return result


class CanMessage:
    arbitration_id = 0
    is_extended_id = False
    dlc = 0
    data = b''
    is_fd = False
    bitrate_switch = False


class Fdcanusb:
    """Connects to a single mjbots fdcanusb."""

    def __init__(self, path=None, debug_log=None):
        """Constructor.

        Arguments:
          path: serial port where fdcanusb is located
        """
        if path is None:
            path = self.detect_fdcanusb()

        # A fdcanusb ignores the requested baudrate, so we'll just
        # pick something nice and random.
        self._serial = aioserial.AioSerial(port=path, baudrate=9600)
        self._stream_data = b''

        self._cycle_lock = asyncio.Lock()

        self._debug_log = None
        if debug_log:
            self._debug_log = open(debug_log, 'wb')

    async def _readline(self, stream):
        while True:
            offset = min((self._stream_data.find(c) for c in b"\r\n"
                          if c in self._stream_data), default=None)
            if offset is not None:
                to_return, self._stream_data = (
                    self._stream_data[0:offset+1],
                    self._stream_data[offset+1:])
                if offset > 0:
                    return to_return
                else:
                    continue
            else:
                self._stream_data += await stream.read(8192, block=False)

    def detect_fdcanusb(self):
        if sys.platform == 'win32':
            return self.win32_detect_fdcanusb()

        if os.path.exists('/dev/fdcanusb'):
            return '/dev/fdcanusb'
        maybe_list = glob.glob('/dev/serial/by-id/*fdcanusb*')
        if len(maybe_list):
            return sorted(maybe_list)[0]

        return self.pyserial_detect_fdcanusb()

    def win32_detect_fdcanusb(self):
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == 0x0483 and port.pid == 0x5740:
                return port.name

        raise RuntimeError('Could not detect fdcanusb')

    def pyserial_detect_fdcanusb(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == 0x0483 and port.pid == 0x5740:
                return port.device

        raise RuntimeError('Could not detect fdcanusb')

    async def cycle(self, commands):
        """Request that the given set of commands be sent to the fdcanusb, and
        any responses collated and returned, after being parsed by
        their command specific parsers.

        Each command instance must model moteus.Command
        """

        # Since the fdcanusb has state, we need to ensure it is kept
        # consistent.  Only one call to cycle at a time please.
        async with self._cycle_lock:
            # Since the fdcanusb can't send multiple things at once, we
            # just go through the commands one at a time and handle them
            # individually.
            return [await self._do_command(x) for x in commands]

    async def _do_command(self, command):
        await self.write(command)
        reply_required = command.reply_required

        # Get the OK response.
        ok_response = await self._readline(self._serial)
        if not ok_response.startswith(b"OK"):
            raise RuntimeError("fdcanusb lost synchronization, got: " +
                               ok_response.decode('latin1'))

        if reply_required:
            line = await self._readline(self._serial)

            if not line.startswith(b"rcv"):
                raise RuntimeError("unexpected fdcanusb response, got: " +
                                   line.decode('latin1'))

            if self._debug_log:
                self._debug_log.write(f'{time.time()} < '.encode('latin1') +
                                      line.rstrip() + b'\n')

            fields = line.split(b" ")
            message = CanMessage()
            message.data = _dehexify(fields[2])
            message.arbitration_id = int(fields[1], 16)

            # We are assuming that only one device is responding at a
            # time, thus we don't even need to look at the
            # source/destination or CAN prefix.

            return command.parse(message)

    async def write(self, command):
        # This merely sends a command and doesn't even wait for an OK
        # to come back.  It can *not* be intermixed with calls to
        # 'cycle'.
        bus_id = None
        if command.raw:
            bus_id = command.arbitration_id
        else:
            bus_id = (command.destination |
                      (0x8000 if command.reply_required else 0) |
                      (command.can_prefix << 16))
        cmd = "can send {:04x} {}\n".format(
            bus_id, _hexify(command.data)).encode('latin1')
        self._serial.write(cmd)
        if self._debug_log:
            self._debug_log.write(f'{time.time()} > '.encode('latin1') +
                                  cmd.rstrip() + b'\n')
        await self._serial.drain()

    async def read(self):
        # Read a single CAN message and do not parse it.
        while True:
            line = await self._readline(self._serial)
            if not line.startswith(b"rcv"):
                continue

            if self._debug_log:
                self._debug_log.write(f'{time.time()} < '.encode('latin1') +
                                      line.rstrip() + b'\n')

            fields = line.split(b" ")
            message = CanMessage()
            message.data = _dehexify(fields[2])
            message.arbitration_id = int(fields[1], 16)
            return message
