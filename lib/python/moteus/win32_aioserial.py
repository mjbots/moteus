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

'''This is a low-performance implementation of an asyncio serial
wrapper that is compatible with windows which does not allow
non-blocking serial connections.  It uses threads and busy-looping to
emulate non-blocking operations while still supporting cancellation.

'''

import asyncio
import queue
import serial
import threading
from typing import List, Optional, Union


async def _async_set_future(fut, value):
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


class AioSerial:
    def __init__(self, *args, aioserial_poll_period=0.05, **kwargs):
        self.serial = serial.Serial(*args, **kwargs)
        self._write_data = b''
        self._read_queue = queue.Queue()
        self._write_queue = queue.Queue()
        self._read_thread = threading.Thread(
            target=self._read_child, daemon=True)
        self._read_thread.start()
        self._write_thread = threading.Thread(
            target=self._write_child, daemon=True)
        self._write_thread.start()
        self._aio_poll_period = aioserial_poll_period
        self.serial.timeout = aioserial_poll_period

    async def read(self, size: int = 1, block=True) -> bytes:
        loop = asyncio.get_event_loop()
        remaining = size
        accumulated_result = b''

        while True:
            f = loop.create_future()

            def do_read():
                result = self.serial.read(remaining)
                if len(result):
                    asyncio.run_coroutine_threadsafe(
                        _async_set_future(f, result), loop)
                    return

            self._read_queue.put_nowait(do_read)
            this_round = await f
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

        def do_write():
            self.serial.write(write_data)
            asyncio.run_coroutine_threadsafe(_async_set_future(f, True), loop)

        self._write_queue.put_nowait(do_write)
        result = await f

    def _read_child(self):
        _run_queue(self._read_queue)

    def _write_child(self):
        _run_queue(self._write_queue)
