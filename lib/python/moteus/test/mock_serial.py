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


class MockSerial:
    """Mock serial device for testing fdcanusb and transport functionality."""

    def __init__(self):
        self.write_data = b''
        self.read_buffer = b''
        self.is_open = True

        self._read_waiters = []
        self._response_queue = collections.deque()

        # List of (pattern, callback) tuples
        self._write_hooks = []

    def add_write_hook(self, pattern, callback):
        """Add a hook that triggers when a write matches the pattern."""
        self._write_hooks.append((pattern, callback))

    def write(self, data):
        self.write_data += data

        # Check write hooks first, iterate over a copy in case a
        # handler modifies it.
        new_hooks = []
        for pattern, callback in self._write_hooks[:]:
            if pattern in data:
                callback(self, data)
            else:
                # By default, only execute hooks once.
                new_hooks.append((pattern, callback))

        self._write_hooks = new_hooks

        if data.startswith(b'can send'):
            if self._response_queue:
                receive_frame = self._response_queue.popleft()
                self.read_buffer += receive_frame
                self._notify_waiters()

    async def drain(self):
        pass

    async def read(self, size, block=True):
        while True:
            # If we have enough data, return it
            if len(self.read_buffer) >= size:
                break

            # If non-blocking, return whatever we have (even if empty)
            if not block:
                break

            # Wait for more data
            waiter = asyncio.Future()
            self._read_waiters.append(waiter)
            try:
                await waiter
            except asyncio.CancelledError:
                return b''

        to_return = self.read_buffer[:size]
        self.read_buffer = self.read_buffer[size:]
        return to_return

    def close(self):
        self.is_open = False
        for waiter in self._read_waiters:
            if not waiter.done():
                waiter.cancel()

        self._read_waiters.clear()

    def add_response(self, data):
        self.read_buffer += data
        self._notify_waiters()

    def queue_response(self, data):
        self._response_queue.append(data)

    def _notify_waiters(self):
        waiters_to_notify = [w for w in self._read_waiters if not w.done()]
        self._read_waiters.clear()
        for waiter in waiters_to_notify:
            waiter.set_result(None)
