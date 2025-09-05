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

from dataclasses import dataclass
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


FrameFilter = typing.Callable[[Frame], bool]


class Subscription:
    '''ABC for managing device subscriptions.'''
    def cancel(self):
        raise NotImplementedError()


class TransportDevice:
    '''ABC for hardware-specific CAN devices'''

    async def send_frame(self, frame: Frame):
        raise NotImplementedError()

    async def receive_frame(self) -> Frame:
        raise NotImplementedError()

    def subscribe(self, filter: FrameFilter,
                  handler: typing.Callable[[Frame], typing.Awaitable[None]]) -> Subscription:
        raise NotImplementedError()

    async def transaction(
            self,
            requests: typing.List[typing.Tuple[Frame, FrameFilter]],
            timeout: typing.Optional[float] = None) -> typing.List[typing.Optional[Frame]]:
        """Send requests and wait for matching responses.

        Args:
          requests: List of (frame, response_filter) tuples.  If a
            filter is None, do not wait for a response for that frame.
          timeout: Optional timeout in seconds for all responses
        """
        raise NotImplementedError()

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

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
