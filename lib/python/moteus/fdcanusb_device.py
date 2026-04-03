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
from .async_timeout import timeout
from .transport_device import Frame, FrameFilter, TransportDevice


# Default timeout/retry configuration
DEFAULT_COMMAND_TIMEOUT = 0.2      # Time to wait for OK (seconds)
DEFAULT_RESPONSE_TIMEOUT = 0.2    # Time to wait for rcv (seconds)
DEFAULT_MAX_RETRIES = 3           # Attempts before giving up
DEFAULT_RETRY_BACKOFF = 1.5       # Exponential backoff multiplier
DEFAULT_MAX_LINE_LENGTH = 512     # Buffer overflow protection (bytes)


# Support both old and new fdcanusb VID/PID
FDCANUSB_IDS = [
    (0x0483, 0x5740),  # Legacy fdcanusb
    (0x1209, 0x2323),  # New fdcanusb with gs_usb
]


def _hexify(data):
    return ''.join(['{:02X}'.format(x) for x in data])


# CRC-8 lookup table for polynomial 0x97, indexed by nybble
_CRC8_TABLE = [
    0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb,
    0x57, 0xc0, 0xee, 0x79, 0xb2, 0x25, 0x0b, 0x9c,
]


def _compute_crc8(data: bytes) -> int:
    """Compute CRC-8 using polynomial 0x97, nybble-at-a-time."""
    crc = 0
    for b in data:
        crc = _CRC8_TABLE[(crc ^ (b >> 4)) & 0x0f] ^ ((crc << 4) & 0xff)
        crc = _CRC8_TABLE[(crc ^ (b & 0x0f)) & 0x0f] ^ ((crc << 4) & 0xff)
    return crc


def _append_checksum(line: bytes) -> bytes:
    """Append ' *XX' checksum to a line (without trailing newline).

    The CRC is computed over the line content plus the space before the '*',
    since the firmware computes CRC over everything before the '*'.
    """
    line_with_space = line + b' '
    crc = _compute_crc8(line_with_space)
    return line_with_space + '*{:02X}'.format(crc).encode('latin1')


def _strip_and_validate_checksum(line: bytes) -> tuple:
    """Strip and validate checksum from a line.

    Returns:
        (stripped_line, had_checksum, checksum_valid)
        - stripped_line: line with checksum removed (and trailing spaces stripped)
        - had_checksum: True if a *XX pattern was found at end of line
        - checksum_valid: True if checksum matched (only meaningful if had_checksum)
    """
    # Find the last '*' in the line - must be exactly 3 chars from end (*XX)
    star_pos = line.rfind(b'*')
    if star_pos == -1 or star_pos + 3 != len(line):
        return (line, False, False)

    # Check if followed by exactly 2 hex digits
    hex_part = line[star_pos + 1:star_pos + 3]
    try:
        claimed_crc = int(hex_part, 16)
    except ValueError:
        return (line, False, False)

    # Compute CRC over everything before '*'
    content = line[:star_pos]
    computed_crc = _compute_crc8(content)

    # Strip trailing spaces from content
    content = content.rstrip(b' ')

    return (content, True, claimed_crc == computed_crc)


def _dehexify(data):
    result = b''
    for i in range(0, len(data), 2):
        result += bytes([int(data[i:i + 2], 16)])
    return result


def _get_fdcanusb_properties(path):
    """Attempt to find the USB serial number for a given device path.

    This function handles both direct device paths and symlinks to the
    actual device, like custom udev rules that create /dev/fdcanusb -> /dev/serial/by-id/foo.
    """

    result = {}
    if not path:
        return result

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
                    result['serial_number'] = port.serial_number

                if (port.vid, port.pid) in FDCANUSB_IDS:
                    result['is_fdcanusb'] = True

                return result

    except Exception:
        # If anything goes wrong, just bail.
        pass

    return result


class FdcanusbError(RuntimeError):
    def __init__(self, message):
        super(FdcanusbError, self).__init__(message)


class FdcanusbDevice(TransportDevice):
    """Connects to a single mjbots fdcanusb."""

    def __init__(self, path=None, debug_log=None, disable_brs=False, baudrate=None,
                 command_timeout=None, response_timeout=None,
                 max_retries=None, retry_backoff=None,
                 max_line_length=None, **kwargs):
        """Constructor.

        Arguments:
          path: serial port where fdcanusb is located
          debug_log: optional file-like object for debug logging
          disable_brs: disable bitrate switching
          baudrate: serial baud rate (default 1Mbps)
          command_timeout: time to wait for OK response (default 0.2s)
          response_timeout: time to wait for rcv response (default 0.2s)
          max_retries: number of retry attempts (default 3)
          retry_backoff: exponential backoff multiplier (default 1.5)
          max_line_length: maximum line buffer size (default 512)
        """
        super(FdcanusbDevice, self).__init__(**kwargs)

        self._disable_brs = disable_brs

        # Timeout/retry configuration
        self._command_timeout = command_timeout if command_timeout is not None else DEFAULT_COMMAND_TIMEOUT
        self._response_timeout = response_timeout if response_timeout is not None else DEFAULT_RESPONSE_TIMEOUT
        self._max_retries = max_retries if max_retries is not None else DEFAULT_MAX_RETRIES
        self._retry_backoff = retry_backoff if retry_backoff is not None else DEFAULT_RETRY_BACKOFF
        self._max_line_length = max_line_length if max_line_length is not None else DEFAULT_MAX_LINE_LENGTH

        # A fdcanusb or mjcanfd-usb-1x ignores the requested baudrate.
        # If none is specified, use 1Mbps, which is the default for
        # UART based communication with moteus.
        self._serial = aioserial.AioSerial(
            port=path, baudrate=baudrate or 921600)

        # Attempt to discover the USB serial number associated with
        # this device for pretty-printing.
        fdcanusb_props = _get_fdcanusb_properties(path)

        self._serial_number = fdcanusb_props.get('serial_number', None)
        self._is_fdcanusb = fdcanusb_props.get('is_fdcanusb', False)

        # Start out by enabling checksums for non-fdcanusb devices
        # (e.g., UART to moteus).
        #
        # These will also be enabled dynamically if we receive a
        # checksum error, indicating that the recipient is requiring
        # checksums.
        self._checksum_active = not self._is_fdcanusb

        self._stream_data = b''

        self._ok_waiters = []

        self._reader_task = None
        self._running = False

        self._debug_log = debug_log

        # Lock to serialize send_frame and transaction calls to prevent
        # interleaving of commands and responses from concurrent coroutines.
        self._command_lock = asyncio.Lock()

        # Start the reader if we can.
        self._start_reader()

    def __repr__(self):
        if self._serial_number:
            return f"Fdcanusb(sn='{self._serial_number}')"
        else:
            return 'Fdcanusb()'

    @property
    def serial_number(self):
        """Return the USB serial number of this device, if known."""
        return self._serial_number

    @property
    def supports_flash(self):
        """Return True if this device supports firmware flashing.

        UART-based connections do not support flashing because the bootloader
        requires CAN-FD for reliable bulk data transfer.
        """
        return self._is_fdcanusb

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
        if self._reader_task and self._reader_task.done():
            raise self._reader_task.exception()

        if self._reader_task is None and self._running:
            self._reader_task = asyncio.create_task(self._reader_loop())

    async def _process_one_line(self):
        """Read and process a single line from the serial port.

        Returns True if line was processed, False if it should be skipped.
        """
        line = await self._readline(self._serial)
        if not line:
            return False

        if self._debug_log:
            self._write_log(b'< ' + line.rstrip())

        # Strip and validate checksum if present
        stripped_line = line.rstrip()
        stripped_line, had_checksum, checksum_valid = \
            _strip_and_validate_checksum(stripped_line)

        if self._checksum_active:
            # When checksum mode is active, require valid checksums
            if not had_checksum:
                if self._debug_log:
                    self._write_log(b'< CHECKSUM MISSING, skipping')
                return False
            if not checksum_valid:
                if self._debug_log:
                    self._write_log(b'< CHECKSUM INVALID, skipping')
                return False
        elif had_checksum and not checksum_valid:
            # Checksum present but invalid - skip even if not in checksum mode
            if self._debug_log:
                self._write_log(b'< CHECKSUM INVALID, skipping')
            return False

        if line.startswith(b'rcv'):
            frame = self._parse_frame(stripped_line)
            await self._handle_received_frame(frame)
        elif line.startswith(b'OK'):
            await self._handle_ok_response(stripped_line)
        elif line.startswith(b'ERR'):
            await self._handle_err_response(stripped_line)
        else:
            await self._handle_other_response(stripped_line)

        return True

    async def _reader_loop(self):
        try:
            while self._running:
                try:
                    await self._process_one_line()
                except asyncio.CancelledError:
                    break
        finally:
            self._running = False

    async def _handle_err_response(self, line):
        # Check for checksum error and enable checksum mode
        if b'checksum' in line.lower():
            self._checksum_active = True

        # Pass this error to the first non-done waiter.
        for waiter in self._ok_waiters:
            if not waiter.done():
                waiter.set_exception(FdcanusbError(f'Error: {line}'))
                return

        # If there is no one waiting, then just ignore this error.

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
                # Check for buffer overflow - if we have too much data
                # without a newline, discard it to prevent memory issues
                # and potential corruption from a garbled line.
                if len(self._stream_data) > self._max_line_length:
                    if self._debug_log:
                        self._write_log(b'< LINE OVERFLOW, discarding buffer')
                    self._stream_data = b''

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
        fields = line.strip().split(b" ")
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

        cmd = "can send {:04x} {}{}{}".format(
            frame.arbitration_id,
            hexdata,
            ' ' if len(flags) > 0 else '',
            flags).encode('latin1')

        if self._checksum_active:
            cmd = _append_checksum(cmd)

        cmd = cmd + b'\n'

        self._serial.write(cmd)
        if self._debug_log:
            self._write_log(b'> ' + cmd.rstrip())

    async def _send_frame_helper(self, frame: Frame):
        try:
            ok_waiter: asyncio.Future[None] = asyncio.Future()
            self._ok_waiters.append(ok_waiter)

            await self._write_send_frame(frame)
            await self._serial.drain()

            await ok_waiter
        finally:
            self._ok_waiters = [w for w in self._ok_waiters
                                if w != ok_waiter]

    async def _with_uart_retry(self, operation, description,
                               allow_retry=True):
        """Run an async operation with timeout and retry for UART errors.

        For fdcanusb (CAN-FD USB adapters), the operation is run
        directly without timeout.  For UART connections, a timeout
        and retry with exponential backoff is applied.

        Retries on FdcanusbError containing 'checksum' or 'unknown
        command', and on TimeoutError.  Other errors are raised
        immediately.
        """
        current_timeout = self._command_timeout
        retries = 0

        while True:
            try:
                if self._is_fdcanusb:
                    return await operation()
                else:
                    async with timeout(current_timeout):
                        return await operation()
            except FdcanusbError as e:
                str_error = str(e).lower()
                if (allow_retry and
                    ('checksum' in str_error or
                     'unknown command' in str_error)):
                    retries += 1
                    if retries > self._max_retries:
                        raise
                    await asyncio.sleep(0.010)
                    current_timeout *= self._retry_backoff
                    continue
                raise
            except asyncio.TimeoutError:
                retries += 1
                if not allow_retry or retries > self._max_retries:
                    raise RuntimeError(
                        f"{description} failed after "
                        f"{retries} retries")
                current_timeout *= self._retry_backoff
                continue

    async def send_frame(self, frame: Frame):
        await self._ensure_reader_started()

        async with self._command_lock:
            await self._with_uart_retry(
                lambda: self._send_frame_helper(frame),
                "send_frame")


    async def receive_frame(self):
        await self._ensure_reader_started()

        return await super(FdcanusbDevice, self).receive_frame()

    async def _transaction_unlocked(self, requests, subscriptions):
        """Execute a transaction without holding the lock.

        Args:
            requests: List of Request objects to send.
            subscriptions: List of (subscription, future) tuples for responses.
        """

        async def send_and_wait_ok():
            ok_waiters: set[asyncio.Future[None]] = set(
                [asyncio.Future() for _ in range(len(requests))])
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

        await self._with_uart_retry(
            send_and_wait_ok, "transaction",
            allow_retry=(len(requests) == 1))

        # If there any responses to wait for, do so.
        if subscriptions:
            if self._is_fdcanusb:
                await asyncio.gather(*[x[1] for x in subscriptions])
            else:
                # Use timeout for waiting on rcv responses
                try:
                    async with timeout(self._response_timeout):
                        await asyncio.gather(*[x[1] for x in subscriptions])
                except asyncio.TimeoutError:
                    # Timeout waiting for responses - proceed with
                    # whatever we got.
                    pass

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

        async with self._command_lock:
            subscriptions = [
                make_subscription(request)
                for request in requests
                if request.frame_filter is not None
            ]

            try:
                await self._transaction_unlocked(requests, subscriptions)
            finally:
                # Clean up all our subscriptions.
                for x in subscriptions:
                    x[0].cancel()

    def _write_log(self, output: bytes):
        assert self._debug_log is not None
        self._debug_log.write(f'{time.time():.6f}/{self._serial_number} '.encode('latin1') + output + b'\n')

    @staticmethod
    def detect_fdcanusbs():
        """
        Returns:
            (list[str]): list of filesystem paths that can be
                used to construct FdcanusbDevice objects
        """
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
                if (x.vid, x.pid) in FDCANUSB_IDS]
