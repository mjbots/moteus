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
import glob
import os
import serial
import serial.tools
import serial.tools.list_ports
import sys
import time
import typing

from . import aioserial
from .transport_device import Frame, FrameFilter, TransportDevice


def _hexify(data):
    return ''.join(['{:02X}'.format(x) for x in data])


def _dehexify(data):
    result = b''
    for i in range(0, len(data), 2):
        result += bytes([int(data[i:i + 2], 16)])
    return result


def _find_serial_number(path):
    """Attempt to find the USB serial number for a given device path.

    This function handles both direct device paths and symlinks to the
    actual device, like custom udev rules that create /dev/fdcanusb -> /dev/serial/by-id/foo.
    """
    if not path:
        return None

    try:
        # Resolve symlinks to get the actual device path
        real_path = os.path.realpath(path)

        # Get all ports including symlinks
        ports = serial.tools.list_ports.comports(include_links=True)

        for port in ports:
            # Check if either the device path or the resolved path matches
            if port.device == path or port.device == real_path:
                # Return serial number if available
                if hasattr(port, 'serial_number') and port.serial_number:
                    return port.serial_number

    except Exception:
        # If anything goes wrong, just return None
        pass

    return None


class FdcanusbDevice(TransportDevice):
    """Connects to a single mjbots fdcanusb."""

    def __init__(self, path=None, debug_log=None, disable_brs=False, **kwargs):
        """Constructor.

        Arguments:
          path: serial port where fdcanusb is located
        """
        super(FdcanusbDevice, self).__init__(**kwargs)

        self._disable_brs = disable_brs

        # A fdcanusb ignores the requested baudrate, so we'll just
        # pick something nice and random.
        self._serial = aioserial.AioSerial(port=path, baudrate=9600)

        # Attempt to discover the USB serial number associated with
        # this device for pretty-printing.
        self._serial_number = _find_serial_number(path)

        self._stream_data = b''

        self._ok_waiters = []

        self._reader_task = None
        self._running = False

        self._debug_log = debug_log

        # Start the reader if we can.
        self._start_reader()

    def __repr__(self):
        if self._serial_number:
            return f"Fdcanusb(sn='{self._serial_number}')"
        else:
            return 'Fdcanusb()'

    def close(self):
        if self._reader_task and not self._reader_task.done():
            self._reader_task.cancel()

        if hasattr(self._serial, 'close'):
            self._serial.close()

        self._running = False

    def empty_bus_tx_safe(self):
        return True

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    def _start_reader(self):
        self._running = True
        try:
            self._reader_task = asyncio.create_task(self._reader_loop())
        except RuntimeError:
            self._reader_task = None

    async def _ensure_reader_started(self):
        if self._reader_task is None and self._running:
            self._reader_task = asyncio.create_task(self._reader_loop())

    async def _reader_loop(self):
        try:
            while self._running:
                try:
                    line = await self._readline(self._serial)
                    if not line:
                        continue

                    if self._debug_log:
                        self._write_log(b'< ' + line.rstrip())

                    if line.startswith(b'rcv'):
                        frame = self._parse_frame(line)
                        await self._handle_received_frame(frame)
                    elif line.startswith(b'OK'):
                        await self._handle_ok_response(line)
                    else:
                        await self._handle_other_response(line)
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    raise
                    # Log and continue running.
                    if self._debug_log:
                        self._write_log(f'ERROR: {str(e)}'.encode('latin1'))

                    # Sleep briefly to prevent a tight error loop.
                    await asyncio.sleep(0.01)
        finally:
            self._running = False

    async def _handle_ok_response(self, line):
        # Just notify the first non-done OK waiter.  An OK received
        # with no waiters is assumed to be stale.
        #
        # Callers are *required* to enqueue their self._ok_waiters
        # *before* sending anything that could result in an OK being
        # emitted.
        for waiter in self._ok_waiters:
            if not waiter.done():
                waiter.set_result(None)
                return

    async def _handle_other_response(self, line):
        raise RuntimeError(f'{self} received error {line}')

    async def _readline(self, stream):
        while True:
            offset = min((self._stream_data.find(c) for c in b"\r\n"
                          if c in self._stream_data), default=None)
            if offset is not None:
                to_return, self._stream_data = (
                    self._stream_data[0:offset+1],
                    self._stream_data[offset+1:])
                if offset > 0:
                    return to_return
                else:
                    continue
            else:
                data = await stream.read(8192, block=False)
                if not data:
                    # If for some reason we got a completely empty
                    # response, which shouldn't happen, ensure we
                    # yield control to somebody else so that hopefully
                    # we eventually make forward progress.
                    await asyncio.sleep(0)
                else:
                    self._stream_data += data

    def _parse_frame(self, line):
        fields = line.split(b" ")
        frame = Frame()
        frame.data = _dehexify(fields[2])
        frame.dlc = len(frame.data)
        frame.arbitration_id = int(fields[1], 16)
        frame.channel = self

        for flag in fields[3:]:
            if flag == b'E':
                frame.is_extended_id = True
            if flag == b'B':
                frame.bitrate_switch = True
            if flag == b'F':
                frame.is_fd = True

        return frame

    async def _write_send_frame(self, frame):
        actual_brs = frame.bitrate_switch and not self._disable_brs

        hexdata = _hexify(frame.data)
        on_wire_size = self._round_up_dlc(len(frame.data))
        hexdata += self._padding_hex * (on_wire_size - len(frame.data))

        flags = ''
        if actual_brs:
            flags += 'B'
        else:
            flags += 'b'
        if frame.is_fd:
            flags += 'F'

        cmd = "can send {:04x} {}{}{}\n".format(
            frame.arbitration_id,
            hexdata,
            ' ' if len(flags) > 0 else '',
            flags).encode('latin1')

        self._serial.write(cmd)
        if self._debug_log:
            self._write_log(b'> ' + cmd.rstrip())

    async def send_frame(self, frame: Frame):
        await self._ensure_reader_started()

        try:
            ok_waiter = asyncio.Future()
            self._ok_waiters.append(ok_waiter)

            await self._write_send_frame(frame)
            await self._serial.drain()

            await ok_waiter
        finally:
            self._ok_waiters = [w for w in self._ok_waiters
                                if w != ok_waiter]

    async def receive_frame(self):
        await self._ensure_reader_started()

        return await super(FdcanusbDevice, self).receive_frame()

    async def transaction(
            self,
            requests: typing.List[TransportDevice.Request],
            **kwargs):

        # We do not support child devices.
        assert not any([request.child_device is not None
                        for request in requests])

        def make_subscription(request):
            future = asyncio.Future()

            async def handler(frame, request=request, future=future):
                if future.done():
                    # Stick it in our receive queue so that it isn't
                    # lost.
                    self._receive_queue.append(frame)

                    return

                request.responses.append(frame)
                # While we may receive more than one frame for a given
                # request, we only wait for one.
                future.set_result(None)

            return self._subscribe(request.frame_filter, handler), future

        subscriptions = [
            make_subscription(request)
            for request in requests
            if request.frame_filter is not None
        ]

        try:
            # Now we will send all our requests.
            ok_waiters = set([asyncio.Future()
                              for _ in range(len(requests))])
            try:
                self._ok_waiters.extend(list(ok_waiters))

                # TODO: Provide control over the amount of pipelining
                # like the C++ class.
                for request in requests:
                    await self._write_send_frame(request.frame)

                await self._serial.drain()

                await asyncio.gather(*ok_waiters)
            finally:
                self._ok_waiters = [w for w in self._ok_waiters
                                    if w not in ok_waiters]

            # If there any responses to wait for, do so.
            if subscriptions:
                await asyncio.gather(*[x[1] for x in subscriptions])
        finally:
            # Clean up all our subscriptions.
            for x in subscriptions:
                x[0].cancel()

    def _write_log(self, output: bytes):
        assert self._debug_log is not None
        self._debug_log.write(f'{time.time():.6f}/{self._serial_number} '.encode('latin1') + output + b'\n')

    @staticmethod
    def detect_fdcanusbs():
        if sys.platform == 'win32':
            return FdcanusbDevice.pyserial_detect_fdcanusbs()

        maybe_list = glob.glob('/dev/serial/by-id/*fdcanusb*')
        if len(maybe_list):
            return sorted(maybe_list)

        return FdcanusbDevice.pyserial_detect_fdcanusbs()

    @staticmethod
    def detect_fdcanusb():
        return FdcanusbDevice.detect_fdcanusbs()[0]

    @staticmethod
    def pyserial_detect_fdcanusbs():
        ports = serial.tools.list_ports.comports()

        return [x.device for x in ports
                if x.vid == 0x0483 and x.pid == 0x5740]
