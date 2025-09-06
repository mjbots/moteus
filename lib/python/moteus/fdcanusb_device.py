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
import glob
import os
import serial
import serial.tools
import serial.tools.list_ports
import sys
import time
import typing

from . import aioserial
from .transport_device import Frame, FrameFilter, TransportDevice, Subscription


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


class FdcanusbSubscription(Subscription):
    def __init__(self, transport, subscription_id):
        self._transport = transport
        self._id = subscription_id

    def cancel(self):
        self._transport._remove_subscription(self._id)


class FdcanusbDevice(TransportDevice):
    """Connects to a single mjbots fdcanusb."""

    def __init__(self, path=None, debug_log=None, disable_brs=False,
                 max_buffer_size=1000, padding_hex='50', channel=None):
        """Constructor.

        Arguments:
          path: serial port where fdcanusb is located
        """
        self._max_buffer_size = max_buffer_size
        self._padding_hex = padding_hex
        self._disable_brs = disable_brs
        self._channel = channel

        # A fdcanusb ignores the requested baudrate, so we'll just
        # pick something nice and random.
        self._serial = aioserial.AioSerial(port=path, baudrate=9600)

        # Attempt to discover the USB serial number associated with
        # this device for pretty-printing.
        self._serial_number = _find_serial_number(path)

        self._stream_data = b''

        self._receive_queue = collections.deque()
        self._receive_waiters = []
        self._subscriptions = {}
        self._next_subscription_id = 0

        self._ok_waiters = []
        self._ok_queue = collections.deque()

        self._reader_task = None
        self._running = False

        self._send_flags = 'b' if disable_brs else ''

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

    def _notify_waiters(self, waitlist):
        waiters_to_notify = []
        for waiter in waitlist:
            if not waiter.done():
                waiters_to_notify.append(waiter)

        for waiter in waiters_to_notify:
            waiter.set_result(None)

    async def _handle_received_frame(self, frame):
        self._receive_queue.append(frame)

        # Limit our maximum queue size.
        while len(self._receive_queue) > self._max_buffer_size:
            self._receive_queue.popleft()

        self._notify_waiters(self._receive_waiters)

        # Then dispatch to subscribers.
        for sub_id, (filter_fn, handler) in self._subscriptions.items():
            if filter_fn(frame):
                asyncio.create_task(self._run_handler(handler, frame))

    async def _handle_ok_response(self, line):
        self._ok_queue.append(line)

        self._notify_waiters(self._ok_waiters)

    async def _handle_other_responses(self, line):
        # TODO: Actually do something reasonable.
        await self._handle_ok_response(line)

    async def _wait_for_ok(self):
        if self._ok_queue:
            return self._ok_queue.popleft()

        while True:
            try:
                waiter = asyncio.Future()
                self._ok_waiters.append(waiter)

                await waiter

                if self._ok_queue:
                    return self._ok_queue.popleft()
            finally:
                self._ok_waiters = [w for w in self._ok_waiters if w != waiter]

    async def _run_handler(self, handler, frame):
        try:
            await handler(frame)
        except Exception as e:
            import traceback, sys
            traceback.print_exc()
            sys.exit(1)

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
        frame.channel = self._channel

        flags = fields[3] if len(fields) > 3 else b''
        if b'E' in flags:
            frame.is_extended_id = True
        if b'B' in flags:
            frame.bitrate_switch = True
        if b'F' in flags:
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

        await self._write_send_frame(frame)
        await self._serial.drain()
        await self._wait_for_ok()

    async def receive_frame(self):
        await self._ensure_reader_started()

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
                self._receive_waiters = [w for w in self._receive_waiters
                                         if w != waiter]

    def subscribe(self,
                  filter: FrameFilter,
                  handler: typing.Callable[[Frame], typing.Awaitable[None]]) -> Subscription:
        sub_id = self._next_subscription_id
        self._next_subscription_id += 1

        self._subscriptions[sub_id] = (filter, handler)

        return FdcanusbSubscription(self, sub_id)

    def _remove_subscription(self, sub_id):
        self._subscriptions.pop(sub_id, None)


    async def transaction(
            self,
            requests: typing.List[typing.Tuple[Frame, FrameFilter]],
            timeout: typing.Optional[float] = None) -> typing.List[typing.Optional[Frame]]:
        responses = [None] * len(requests)
        response_futures = []
        subscriptions = []
        waiting_indices = []

        # Create a subscription for each expected response.
        for i, (_, response_filter) in enumerate(requests):
            if response_filter is None:
                continue

            future = asyncio.Future()
            response_futures.append(future)
            waiting_indices.append(i)

            # We want to capture this into the closure.
            idx = i

            async def handler(frame, idx=idx, future=future):
                if not future.done():
                    future.set_result(frame)

            sub = self.subscribe(response_filter, handler)
            subscriptions.append(sub)

        try:
            # Now we will send all our requests.
            for request, _ in requests:
                await self._write_send_frame(request)

            await self._serial.drain()

            for request, _ in requests:
                await self._wait_for_ok()

            # If there any responses to wait for, do so.
            if response_futures:
                done, pending = await asyncio.wait(
                    response_futures,
                    timeout=timeout,
                    return_when=asyncio.ALL_COMPLETED)

                # Collect results:
                for future_idx, orig_idx in enumerate(waiting_indices):
                    this_future = response_futures[future_idx]
                    if this_future.done():
                        try:
                            responses[orig_idx] = this_future.result()
                        except:
                            pass

            return responses
        finally:
            # Clean up all our subscriptions.
            for sub in subscriptions:
                sub.cancel()

    def _write_log(self, output: bytes):
        assert self._debug_log is not None
        self._debug_log.write(f'{time.time():.6f} '.encode('latin1') + output + b'\n')

    @staticmethod
    def detect_fdcanusb():
        if sys.platform == 'win32':
            return FdcanusbDevice.win32_detect_fdcanusb()

        if os.path.exists('/dev/fdcanusb'):
            return '/dev/fdcanusb'
        maybe_list = glob.glob('/dev/serial/by-id/*fdcanusb*')
        if len(maybe_list):
            return sorted(maybe_list)[0]

        return FdcanusbDevice.pyserial_detect_fdcanusb()

    @staticmethod
    def win32_detect_fdcanusb():
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == 0x0483 and port.pid == 0x5740:
                return port.name

        raise RuntimeError('Could not detect fdcanusb')

    @staticmethod
    def pyserial_detect_fdcanusb():
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == 0x0483 and port.pid == 0x5740:
                return port.device

        raise RuntimeError('Could not detect fdcanusb')
