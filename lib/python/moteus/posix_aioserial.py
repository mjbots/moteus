# Copyright 2018 Henry Chang
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

import array
import asyncio
import concurrent.futures
import sys
from typing import List, Optional, Union

import serial


class AioSerial:

    def __init__(
            self,
            port: Optional[str] = None,
            baudrate: int = 9600,
            bytesize: int = serial.EIGHTBITS,
            parity: str = serial.PARITY_NONE,
            stopbits: Union[float, int] = serial.STOPBITS_ONE,
            xonxoff: bool = False,
            rtscts: bool = False,
            write_timeout: Optional[float] = None,
            dsrdtr: bool = False,
            inter_byte_timeout: Optional[float] = None,
            exclusive: Optional[bool] = None,
            loop: Optional[asyncio.AbstractEventLoop] = None,
            **kwargs):
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=0,
            xonxoff=xonxoff,
            rtscts=rtscts,
            write_timeout=write_timeout,
            dsrdtr=dsrdtr,
            inter_byte_timeout=inter_byte_timeout,
            exclusive=exclusive,
            **kwargs)
        self._loop: Optional[asyncio.AbstractEventLoop] = loop

        self.loop.add_reader(self.serial.fileno(), self._handle_read)
        self._read_event = asyncio.Event()
        self._read_data = bytearray()
        self._write_data = bytearray()
        self.fd = self.serial.fileno()

    @property
    def loop(self) -> Optional[asyncio.AbstractEventLoop]:
        return self._loop if self._loop else asyncio.get_running_loop() \
                if sys.version_info >= (3, 7) else asyncio.get_event_loop()

    @loop.setter
    def loop(self, value: Optional[asyncio.AbstractEventLoop]):
        self.loop = value

    async def read(self, size: int = 1, block=True) -> bytes:
        result = bytearray()

        while True:
            while len(self._read_data) == 0:
                await self._read_event.wait()
                self._read_event.clear()

            read_size = min(size - len(result), len(self._read_data))
            assert read_size > 0
            to_return, self._read_data = (
                self._read_data[0:read_size], self._read_data[read_size:])

            result += to_return

            if not block or len(result) == size:
                return result

    def write(self, data: Union[bytearray, bytes, memoryview]) -> int:
        self._write_data += data

    async def drain(self, ) -> int:
        to_write, self._write_data = self._write_data, bytearray()
        self.serial.write(to_write)

    def _handle_read(self):
        self._read_data += self.serial.read(8192)
        self._read_event.set()
