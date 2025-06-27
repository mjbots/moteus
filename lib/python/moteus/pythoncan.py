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
            try:
                can.rc = can.util.load_config()
            except can.CanInterfaceNotImplementedError as e:
                if 'Unknown interface type "None"' not in str(e):
                    raise

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

        self._disable_brs = False
        if 'disable_brs' in kwargs:
            self._disable_brs = kwargs['disable_brs']
            del kwargs['disable_brs']

        if ('timing' not in kwargs and
            kwargs.get('interface', can.rc['interface']) == 'pcan'):
            # We default to the timing that works with moteus and assume
            # an 80MHz base clock, which seems pretty typical for PCAN
            # interfaces.
            kwargs['timing'] = can.BitTimingFd.from_sample_point(
                nom_bitrate=1000000, data_bitrate=5000000,
                nom_sample_point=66, data_sample_point=66,
                f_clock=80000000)

        self._can = can.Bus(*args, **kwargs)
        self._setup = False

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def close(self):
        self._can.shutdown()

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

        while command.reply_required:
            reply = await self.read()

            moteus_id = (reply.arbitration_id >> 8) & 0x7f

            if command.raw or command.destination == moteus_id:
                return command.parse(reply)

            # We did not get a response from the device we were hoping
            # for, so just keep waiting.

    async def write(self, command):
        reply_required = command.reply_required
        arbitration_id = (command.destination |
                          (0x8000 if reply_required else 0) |
                          (command.can_prefix << 16))
        on_wire_size = self._round_up_dlc(len(command.data))
        full_message = (command.data +
                        bytes([0x50]) * (on_wire_size - len(command.data)))
        message = can.Message(arbitration_id=arbitration_id,
                              is_extended_id=(arbitration_id >= 0x7ff),
                              dlc=on_wire_size,
                              data=full_message,
                              is_fd=True,
                              bitrate_switch=not self._disable_brs)

        self._can.send(message)

    async def read(self):
        self._maybe_setup()
        while True:
            frame = await self._reader.get_message()
            if not frame.is_error_frame:
                return frame
            # Just ignore error frames entirely and keep reading until
            # we get a good one.

    def _round_up_dlc(self, size):
        if size <= 8:
            return size
        if size <= 12:
            return 12
        if size <= 16:
            return 16
        if size <= 20:
            return 20
        if size <= 24:
            return 24
        if size <= 32:
            return 32
        if size <= 48:
            return 48
        if size <= 64:
            return 64
        return size
