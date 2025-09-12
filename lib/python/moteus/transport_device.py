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
from dataclasses import dataclass, field
import typing


@dataclass
class Frame:
    arbitration_id: int = 0
    data: bytes = b''
    dlc: int = 0
    is_extended_id: bool = False
    is_fd: bool = False
    bitrate_switch: bool = False

    '''Designates which device and sub-channel the frame was received
    on, or should be transmitted to.'''
    channel: typing.Any = None

    # This is a compatibility alias for channel that will be removed
    # in a future version.  It will only be set from calls to
    # 'Transport.cycle' or 'Transport.read'
    bus: typing.Any = None


FrameFilter = typing.Callable[[Frame], bool]


class Subscription:
    '''ABC for managing device subscriptions.'''
    def cancel(self):
        raise NotImplementedError()


class TransportDevice:
    '''Base for hardware-specific CAN devices'''
    def __init__(self, max_buffer_size=50, padding_hex='50'):
        self._max_buffer_size = max_buffer_size
        self._padding_hex = padding_hex

        self._receive_queue = []
        self._receive_waiters = []

        self._subscriptions = {}
        self._next_subscription_id = 0

    def parent(self):
        return None

    def close(self):
        pass

    def empty_bus_tx_safe(self):
        """Return True if it is safe to send frames to this
        TransportDevice if no physical hardware is connected.

        It is implied that if False, then attempts should not be made
        to discover devices attached to it, and broadcast frames
        should not be sent to it.
        """
        raise NotImplementedError()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    async def send_frame(self, frame: Frame):
        raise NotImplementedError()

    async def receive_frame(self) -> Frame:
        while True:
            if self._receive_queue:
                return self._receive_queue.pop(0)

            try:
                waiter = asyncio.Future()
                self._receive_waiters.append(waiter)

                await waiter
            finally:
                self._receive_waiters = [w for w in self._receive_waiters
                                         if w != waiter]

    @dataclass
    class Request:
        # The frame to send.
        frame: Frame

        # If this "TransportDevice" is a "parent" device with
        # sub-devices, which sub-device to use.
        child_device: typing.Optional[typing.Any] = None

        # Frames that match this filter will be returned.
        frame_filter: typing.Optional[FrameFilter] = None

        expected_reply_size: int = 0

        # Matching frames are returned in this list.
        responses: typing.List[Frame] = field(default_factory=list)

    async def transaction(
            self,
            requests: typing.List[Request]):
        """Send requests and wait for matching responses.

        Results are stored in place in each Request structure.

        If cancelled or interrupted with an asyncio timeout, any
        already received frames will be stored in the Request
        structures.
        """
        raise NotImplementedError()

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

    class _Subscription:
        def __init__(self, parent, subscription_id):
            self.parent = parent
            self.id = subscription_id

        def cancel(self):
            self.parent._subscriptions.pop(self.id, None)

    def _subscribe(self,
                   filter: FrameFilter,
                   handler: typing.Callable[[Frame], typing.Awaitable[None]]) -> Subscription:
        sub_id = self._next_subscription_id
        self._next_subscription_id += 1

        self._subscriptions[sub_id] = (filter, handler)

        return TransportDevice._Subscription(self, sub_id)

    async def _run_handler(self, handler, frame):
        try:
            await handler(frame)
        except Exception as e:
            import traceback, sys
            traceback.print_exc()
            sys.exit(1)

    def _notify_waiters(self, waitlist):
        waiters_to_notify = [w for w in waitlist if not w.done()]
        [w.set_result(None) for w in waiters_to_notify]

    async def _handle_received_frame(self, frame):
        any_subscription = False

        # Then dispatch to subscribers.
        for sub_id, (filter_fn, handler) in self._subscriptions.items():
            if filter_fn(frame):
                any_subscription = True
                asyncio.create_task(self._run_handler(handler, frame))

        if not any_subscription:
            self._receive_queue.append(frame)

            # Limit our maximum queue size.
            while len(self._receive_queue) > self._max_buffer_size:
                self._receive_queue.pop(0)

            self._notify_waiters(self._receive_waiters)
