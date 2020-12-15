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
import can

class PythonCan:
    '''Implements a 'Transport' on top of python-can.'''

    def __init__(self, *args, **kwargs):
        '''All arguments pass through to can.Bus'''
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
        reply_required = command.reply_required
        arbitration_id = command.destination + (0x8000 if reply_required else 0)
        message = can.Message(arbitration_id=arbitration_id,
                              is_extended_id=(arbitration_id >= 0x7ff),
                              dlc=len(command.data),
                              data=bytearray(command.data),
                              is_fd=True,
                              bitrate_switch=True)

        self._can.send(message)
        if not command.reply_required:
            return None

        reply = await self._reader.get_message()

        return command.parse(reply.data)
