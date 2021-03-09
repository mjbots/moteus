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

'''This wraps a python file-like object in an asynchronous file-like
object for use in coroutines.  It uses threads for maximum
portability, although not necessarily the best performance.'''

import asyncio
import queue
import threading
from typing import List, Optional, Union

async def _async_set_future(fut, value):
    if fut.done():
        # We must have been canceled.
        return
    fut.set_result(value)


def _run_queue(q):
    while True:
        try:
            # We use a timeout just so things like
            # KeyboardInterrupt can fire.
            item = q.get(block=True, timeout=0.05)
            item()
        except queue.Empty:
            pass


class AioStream:
    def __init__(self, fd):
        self.fd = fd

        self._write_data = b''
        self._read_queue = queue.Queue()
        self._write_queue = queue.Queue()
        self._read_lock = threading.Lock()
        self._write_lock = threading.Lock()

        self._read_thread = threading.Thread(
            target=self._read_child, daemon=True)
        self._read_thread.start()
        self._write_thread = threading.Thread(
            target=self._write_child, daemon=True)
        self._write_thread.start()

    async def read(self, size: int = 1, block=True) -> bytes:
        loop = asyncio.get_event_loop()
        remaining = size
        accumulated_result = b''

        while True:
            f = loop.create_future()
            skip = [False]

            def do_read():
                # This will run in the background thread.
                with self._read_lock:
                    if skip[0]:
                        return

                    result = self.fd.read(remaining)
                    asyncio.run_coroutine_threadsafe(_async_set_future(f, result), loop)

            self._read_queue.put_nowait(do_read)
            try:
                this_round = await f
            except asyncio.CancelledError:
                with self._read_lock:
                    skip[0] = True

                raise

            accumulated_result += this_round
            remaining -= len(this_round)
            if not block or remaining == 0:
                return accumulated_result

    def write(self, data: Union[bytearray, bytes, memoryview]) -> int:
        self._write_data += data

    async def drain(self, ) -> int:
        self._write_data, write_data = b'', self._write_data
        loop = asyncio.get_event_loop()
        f = loop.create_future()
        skip = [False]

        def do_write():
            with self._write_lock:
                if skip[0]:
                    return
                self.fd.write(write_data)
                asyncio.run_coroutine_threadsafe(_async_set_future(f, True), loop)

        self._write_queue.put_nowait(do_write)
        try:
            result = await f
        except asyncio.CancelledError:
            with self._write_lock:
                skip[0] = True
            raise

    def _read_child(self):
        _run_queue(self._read_queue)

    def _write_child(self):
        _run_queue(self._write_queue)
