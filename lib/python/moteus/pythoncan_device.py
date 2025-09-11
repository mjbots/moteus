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
import collections
import os
import sys
import time
import typing

from .transport_device import Frame, FrameFilter, TransportDevice, Subscription

can = None


class PythonCanSubscription(Subscription):
    def __init__(self, device, subscription_id):
        self._device = device
        self._id = subscription_id

    def cancel(self):
        self._device._remove_subscription(self._id)


class PythonCanDevice(TransportDevice):
    '''Implements a 'Transport' on top of python-can.'''

    def __init__(self, *args, **kwargs):
        '''Nearly all arguments pass through to can.Bus'''

        super(PythonCanDevice, self).__init__(
            **{k: kwargs[k] for k in ['max_buffer_size',
                                      'padding_hex'] if k in kwargs})

        self._padding_byte = bytes.fromhex(self._padding_hex)
        self._debug_log = kwargs.pop('debug_log', None)

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

        if 'fd' not in can.rc:
            can.rc['fd'] = True

        self._disable_brs = False
        if 'disable_brs' in kwargs:
            self._disable_brs = kwargs['disable_brs']
            del kwargs['disable_brs']

        if ('timing' not in kwargs and
            kwargs.get('interface', can.rc.get('interface', None)) == 'pcan'):
            # We default to the timing that works with moteus and assume
            # an 80MHz base clock, which seems pretty typical for PCAN
            # interfaces.
            kwargs['timing'] = can.BitTimingFd.from_sample_point(
                nom_bitrate=1000000, data_bitrate=5000000,
                nom_sample_point=66, data_sample_point=66,
                f_clock=80000000)

        self._can = can.Bus(*args, **kwargs)
        self._setup = False

        self._notifier = None

        self._log_prefix = (
            getattr(self._can, 'channel', None) or
            kwargs.get('channel', None) or
            str(self._can))

    def __repr__(self):
        return f"PythonCan('{self._log_prefix}')"

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    def close(self):
        if self._notifier:
            self._notifier.stop()
            self._notifier = None

        self._can.shutdown()

    def _maybe_setup(self):
        if self._setup:
            return

        self._notifier = can.Notifier(
            self._can, [self._receive_handler],
            loop=asyncio.get_event_loop())
        self._setup = True

    async def _receive_handler(self, message):
        if message.is_error_frame:
            return

        frame = self._can_message_to_frame(message)

        if self._debug_log:
            self._write_log(f'< {frame.arbitration_id:04X} {frame.data.hex().upper()}'.encode('latin1'))

        await self._handle_received_frame(frame)

    def _frame_to_can_message(self, frame: Frame):
        dlc = self._round_up_dlc(len(frame.data))
        padding_bytes = dlc - len(frame.data)

        return can.Message(
            arbitration_id=frame.arbitration_id,
            is_extended_id=frame.is_extended_id,
            dlc=dlc,
            data=frame.data + bytes(self._padding_byte) * padding_bytes,
            is_fd=frame.is_fd,
            bitrate_switch=frame.bitrate_switch and not self._disable_brs,
        )

    def _can_message_to_frame(self, message) -> Frame:
        return Frame(
            arbitration_id=message.arbitration_id,
            data=message.data,
            dlc=message.dlc,
            is_extended_id=message.is_extended_id,
            is_fd=message.is_fd,
            bitrate_switch=getattr(message, 'bitrate_switch', False),
            channel=self,
        )

    async def send_frame(self, frame: Frame):
        self._maybe_setup()

        can_message = self._frame_to_can_message(frame)

        if self._debug_log:
            self._write_log(f'> {frame.arbitration_id:04x} {can_message.data.hex().upper()}'.encode('latin1'))

        self._can.send(self._frame_to_can_message(frame))

    async def receive_frame(self):
        self._maybe_setup()

        return await super(PythonCanDevice, self).receive_frame()

    async def transaction(
            self,
            requests: typing.List[TransportDevice.Request]):
        # We do not support child devices.
        assert not any([request.child_device is not None
                        for request in requests])

        self._maybe_setup()

        def make_subscription(request):
            future = asyncio.Future()

            async def handler(frame, request=request, future=future):
                if future.done():
                    return

                request.responses.append(frame)
                future.set_result(None)

            return self._subscribe(request.frame_filter, handler), future

        subscriptions = [
            make_subscription(request)
            for request in requests
            if request.frame_filter is not None
        ]

        try:
            for request in requests:
                await self.send_frame(request.frame)

            if subscriptions:
                await asyncio.gather(*[x[1] for x in subscriptions])
        finally:
            for x in subscriptions:
                x[0].cancel()

    def _write_log(self, output: bytes):
        assert self._debug_log is not None
        self._debug_log.write(f'{time.time():.6f}/{self._log_prefix} '.encode('latin1') + output + b'\n')

    @staticmethod
    def enumerate_devices(**kwargs):
        global can
        if not can:
            import can

        import logging

        # python-can spews a lot of logging during enumeration.
        # Squelch as much of that as we can.

        log = logging.getLogger("can")  # covers can.interface and sub-loggers
        prev_level, prev_prop = log.level, log.propagate
        log.setLevel(logging.CRITICAL)

        # Don't bubble to root handlers.
        log.propagate = False

        try:
            # Try to auto-enumerate.
            interfaces_to_check = os.getenv(
                "MJBOTS_PYTHONCAN", "socketcan,pcan,kvaser,vector").split(',')
            available_configs = can.detect_available_configs(interfaces_to_check)

            if not available_configs:
                raise RuntimeError("No python-can interfaces detected")

            def _merge_args(a, b):
                return {**a, **b}

            return [PythonCanDevice(**_merge_args(kwargs, x))
                    for x in available_configs]
        finally:
            # Restore logging to the previous state.
            log.setLevel(prev_level)
            log.propagate = prev_prop
