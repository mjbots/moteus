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

        self._debug_log = kwargs.pop('debug_log', None)
        self._max_buffer_size =  kwargs.pop('max_buffer_size', 1000)

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

        self._receive_queue = collections.deque()
        self._receive_waiters = []
        self._subscriptions = {}
        self._next_subscription_id = 0

        self._notifier = None

    def __repr__(self):
        return f'PythonCan({self._can})'

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

    def _notify_waiters(self, waitlist):
        waiters_to_notify = []
        for waiter in waitlist:
            if not waiter.done():
                waiters_to_notify.append(waiter)

        for waiter in waiters_to_notify:
            waiter.set_result(None)

    def _receive_handler(self, message):
        if message.is_error_frame:
            return

        frame = self._can_message_to_frame(message)

        self._receive_queue.append(frame)

        while len(self._receive_queue) > self._max_buffer_size:
            self._receive_queue.popleft()

        self._notify_waiters(self._receive_waiters)

        if self._debug_log:
            self._write_log(f'< {frame.arbitration_id:04X} {frame.data.hex().upper()}'.encode('latin1'))

        for sub_id, (filter_fn, handler) in self._subscriptions.items():
            if filter_fn(frame):
                asyncio.create_task(self._run_handler(handler, frame))

    async def _run_handler(self, handler, frame):
        try:
            await handler(frame)
        except Exception:
            import traceback, sys
            traceback.print_exc()
            sys.exit(1)

    def _frame_to_can_message(self, frame: Frame):
        dlc = self._round_up_dlc(len(frame.data))
        padding_bytes = dlc - len(frame.data)

        return can.Message(
            arbitration_id=frame.arbitration_id,
            is_extended_id=frame.is_extended_id,
            dlc=dlc,
            data=frame.data + bytes([0x50]) * padding_bytes,
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
        )

    async def send_frame(self, frame: Frame):
        self._maybe_setup()

        if self._debug_log:
            self._write_log(f'> {frame.arbitration_id:04x} {frame.data.hex().upper()}'.encode('latin1'))

        self._can.send(self._frame_to_can_message(frame))

    async def receive_frame(self):
        self._maybe_setup()

        if self._receive_queue:
            return self._receive_queue.popleft()

        while True:
            try:
                waiter = asyncio.Future()
                self._receive_waiters.append(waiter)

                await waiter

                if self._receive_queue:
                    return self._receive_queue.popleft()
            finally:
                startlen = len(self._receive_waiters)
                self._receive_waiters = [w for w in self._receive_waiters
                                         if w != waiter]

    def subscribe(self,
                  filter: FrameFilter,
                  handler: typing.Callable[[Frame], typing.Awaitable[None]]) -> Subscription:
        sub_id = self._next_subscription_id
        self._next_subscription_id += 1

        self._subscriptions[sub_id] = (filter, handler)

        return PythonCanSubscription(self, sub_id)

    def _remove_subscription(self, sub_id):
        self._subscriptions.pop(sub_id, None)

    async def transaction(
            self,
            requests: typing.List[typing.Tuple[Frame, FrameFilter]],
            timeout: typing.Optional[float] = None) -> typing.List[typing.Optional[Frame]]:
        self._maybe_setup()

        respones = [None] * len(requests)
        response_futures = []
        subscriptions = []
        waiting_indices = []

        for i, (_, response_filter) in enumerate(requests):
            if response_filter is not None:
                future = asyncio.Future()
                response_futures.append(future)
                waiting_indices.append(i)

                async def handler(frame, future=future):
                    if not future.done():
                        future.set_result(frame)

                sub = self.subscribe(response_filter, handler)
                subscriptions.append(sub)

        try:
            for request, _ in requests:
                await self.send_frame(request)

            responses = [None] * len(requests)
            if response_futures:
                done, pending = await asyncio.wait(
                    response_futures,
                    timeout=timeout,
                    return_when=asyncio.ALL_COMPLETED)

                for future_idx, orig_idx in enumerate(waiting_indices):
                    this_response = response_futures[future_idx]
                    if this_response.done():
                        try:
                            responses[orig_idx] = this_response.result()
                        except:
                            pass
            return responses

        finally:
            for sub in subscriptions:
                sub.cancel()

    def _write_log(self, output: bytes):
        assert self._debug_log is not None
        self._debug_log.write(f'{time.time():.6f} '.encode('latin1') + output + b'\n')
