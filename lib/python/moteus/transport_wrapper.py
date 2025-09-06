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

import typing

from .transport_device import Frame, FrameFilter, Subscription
from .transport import Transport


class TransportWrapper:
    """Compatibility base class to implement old-style transports with
    a new TransportDevice."""

    def __init__(self, device):
        self._device = device
        self._transport = Transport(device)

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    def close(self):
        self._transport.close()

    async def cycle(self, *args, **kwargs):
        return await self._transport.cycle(*args, **kwargs)

    async def write(self, *args, **kwargs):
        return await self._transport.write(*args, **kwargs)

    async def read(self, *args, **kwargs):
        return await self._transport.read(*args, **kwargs)

    async def flush_read_queue(self, *args, **kwargs):
        return await self._transport.flush_read_queue(*args, **kwargs)

    async def discover(self, *args, **kwargs):
        return await self._transport.discover(*args, **kwargs)
