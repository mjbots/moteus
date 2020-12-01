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

import asyncio

from . import aioserial


def _hexify(data):
    return ''.join(['{:02X}'.format(x) for x in data])


def _dehexify(data):
    result = b''
    for i in range(0, len(data), 2):
        result += bytes([int(data[i:i + 2], 16)])
    return result


async def _readline(stream):
    result = bytearray()
    while True:
        char = await stream.read(1)
        if char == b'\r' or char == b'\n':
            if len(result):
                return result
        else:
            result += char


class Fdcanusb:
    """Connects to a single mjbots fdcanusb."""

    def __init__(self, path=None):
        """Constructor.

        Arguments:
          path: serial port where fdcanusb is located
        """
        if path is None:
            # TODO: Handle windows.
            path = '/dev/fdcanusb'

        # A fdcanusb ignores the requested baudrate, so we'll just
        # pick something nice and random.
        self._serial = aioserial.AioSerial(port=path, baudrate=9600)

    async def cycle(self, commands):
        """Request that the given set of commands be sent to the fdcanusb, and
        any responses collated and returned, after being parsed by
        their command specific parsers.

        Each command instance must model moteus.Command
        """

        # Since the fdcanusb can't send multiple things at once, we
        # just go through the commands one at a time and handle them
        # individually.
        return [await self._do_command(x) for x in commands]

    async def _do_command(self, command):
        destination = command.destination
        reply_required = command.reply_required

        bus_id = command.destination + (0x8000 if reply_required else 0)
        self._serial.write(
            "can send {:04x} {}\n".format(
                bus_id, _hexify(command.data)).encode('latin1'))
        await self._serial.drain()

        # Get the OK response.
        ok_response = await _readline(self._serial)
        if not ok_response.startswith(b"OK"):
            raise RuntimeError("fdcanusb lost synchronization, got: " +
                               ok_response.decode('latin1'))

        if reply_required:
            line = await _readline(self._serial)

            if not line.startswith("rcv"):
                raise RuntimeError("unexpected fdcanusb response, got: " + line)

            fields = line.split(b" ")
            return command.parse(id=fields[1], data=fields[2])
