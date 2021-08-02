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
import sys

can = None

class PythonCan:
    '''Implements a 'Transport' on top of python-can.'''

    def __init__(self, *args, **kwargs):
        '''All arguments pass through to can.Bus'''

        # We delay importing this until we need it, as it can take a
        # while.
        global can
        if not can:
            import can

        # We provide some defaults if they are not already
        # provided... this makes it more likely to just work out of
        # the box and can still be easily override with either
        # constructor arguments, or a config file.
        if 'interface' not in can.rc:
            can.rc['interface'] = 'socketcan'
        if 'channel' not in can.rc:
            can.rc['channel'] = 'can0'
        if 'fd' not in can.rc:
            can.rc['fd'] = True
        self._can = can.Bus(*args, **kwargs)
        self._setup = False

    def _maybe_setup(self):
        if self._setup:
            return

        self._reader = can.AsyncBufferedReader()
        self._notifier = can.Notifier(self._can, [self._reader],
                                      loop=asyncio.get_event_loop())
        self._setup = True

    async def cycle(self, commands):
        self._maybe_setup()

        result = []
        for x in commands:
            reply = await self._do_command(x)
            if reply is not None:
                result.append(reply)

        return result

    async def _do_command(self, command):
        await self.write(command)

        if not command.reply_required:
            return None

        reply = await self._reader.get_message()

        # We're assuming only one device will respond, so the source,
        # destination, and CAN prefix should all match without
        # checking.

        return command.parse(reply)

    async def write(self, command):
        reply_required = command.reply_required
        arbitration_id = (command.destination |
                          (0x8000 if reply_required else 0) |
                          (command.can_prefix << 16))
        message = can.Message(arbitration_id=arbitration_id,
                              is_extended_id=(arbitration_id >= 0x7ff),
                              dlc=len(command.data),
                              data=bytearray(command.data),
                              is_fd=True,
                              bitrate_switch=True)

        self._can.send(message)

    async def read(self):
        self._maybe_setup()
        return await self._reader.get_message()
