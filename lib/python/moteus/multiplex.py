# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

import enum
import math
import struct
from typing import NamedTuple, Union, Iterator, Optional, Any

"""Constants and helper functions used for constructing and parsing
frames in the mjlib multiplex protocol."""

# From https://github.com/mjbots/moteus/blob/main/docs/reference.md#a1-subframes
INT8 = 0
INT16 = 1
INT32 = 2
F32 = 3
IGNORE = 4

WRITE_BASE = 0x00
WRITE_INT8 = 0x00
WRITE_INT16 = 0x04
WRITE_INT32 = 0x08
WRITE_F32 = 0x0c
READ_BASE = 0x10
REPLY_BASE = 0x20
WRITE_ERROR = 0x30
READ_ERROR = 0x31
STREAM_CLIENT_DATA = 0x40
STREAM_SERVER_DATA = 0x41
STREAM_CLIENT_POLL = 0x42
NOP = 0x50


TYPES = [
    struct.Struct('<b'),
    struct.Struct('<h'),
    struct.Struct('<i'),
    struct.Struct('<f'),
]

TYPES_MAX = [
    127,
    32767,
    2147483647,
    math.nan,
]

def read_varuint(offset, buf):
    result = 0
    shift = 0

    for i in range(5):
        if (offset + i) >= len(buf):
            return None, offset + i

        this_byte = buf[offset + i]
        result |= (this_byte & 0x7f) << shift
        shift += 7

        if (this_byte & 0x80) == 0:
            return result, offset + i + 1

    return None, offset + 5


def saturate(value, resolution, scale):
    if not math.isfinite(value):
        if resolution == INT8:
            return -128
        elif resolution == INT16:
            return -32768
        elif resolution == INT32:
            return -2147483648
        elif resolution == F32:
            return value

    scaled = value / scale
    m = TYPES_MAX[resolution]
    double_m = float(m)

    result = scaled
    if scaled < -double_m:
        result = -m
    elif scaled > double_m:
        result = m

    if resolution == F32:
        return result
    return int(result)


def resolution_size(resolution):
    return [1, 2, 4, 4][resolution]


class SubframeType(enum.Enum):
    WRITE = 1
    READ = 2
    RESPONSE = 3
    WRITE_ERROR = 4
    READ_ERROR = 5
    STREAM_CLIENT_TO_SERVER = 6
    STREAM_SERVER_TO_CLIENT = 7
    STREAM_CLIENT_POLL_SERVER = 8


class RegisterSubframe(NamedTuple):
    """A WRITE, READ, or RESPONSE subframe."""
    type: SubframeType       # WRITE, READ, or RESPONSE
    register: int            # Register number
    resolution: int          # INT8/INT16/INT32/F32
    value: Optional[Any]     # Decoded value (None for READ)


class ErrorSubframe(NamedTuple):
    """A WRITE_ERROR or READ_ERROR subframe."""
    type: SubframeType       # WRITE_ERROR or READ_ERROR
    register: int            # Register that caused error
    error_code: int          # Error code


class StreamSubframe(NamedTuple):
    """A tunneled stream subframe."""
    type: SubframeType       # STREAM_CLIENT_TO_SERVER, STREAM_SERVER_TO_CLIENT, STREAM_CLIENT_POLL_SERVER
    channel: int             # Stream channel number
    data: bytes              # Stream payload (empty for POLL)


Subframe = Union[RegisterSubframe, ErrorSubframe, StreamSubframe]


class Stream:
    def __init__(self, data):
        self._data = data
        self._offset = 0

    def remaining(self):
        return len(self._data) - self._offset

    def read_int8(self):
        value = TYPES[INT8].unpack_from(self._data, self._offset)[0]
        self._offset += 1
        return value

    def read_int16(self):
        value = TYPES[INT16].unpack_from(self._data, self._offset)[0]
        self._offset += 2
        return value

    def read_int32(self):
        value = TYPES[INT32].unpack_from(self._data, self._offset)[0]
        self._offset += 4
        return value

    def read_f32(self):
        value = TYPES[F32].unpack_from(self._data, self._offset)[0]
        self._offset += 4
        return value

    def read_varuint(self):
        result, self._offset = read_varuint(self._offset, self._data)
        if result is None:
            raise RuntimeError('Invalid varuint')
        return result

    def read_type(self, typecode):
        if typecode == INT8:
            return self.read_int8()
        elif typecode == INT16:
            return self.read_int16()
        elif typecode == INT32:
            return self.read_int32()
        elif typecode == F32:
            return self.read_f32()
        raise RuntimeError(f'Unknown type: {typecode}')

    def read_bytes(self, count):
        result = self._data[self._offset:self._offset + count]
        self._offset += count
        return result


def parse_frame(data: bytes) -> Iterator[Subframe]:
    """Parse a multiplex frame, yielding subframes.

    Args:
        data: Raw multiplex frame bytes (or any bytes-like object)

    Yields:
        Subframe objects (RegisterSubframe, ErrorSubframe, or StreamSubframe)

    Example:
        for subframe in parse_frame(data):
            if isinstance(subframe, RegisterSubframe):
                print(f"Register {subframe.register} = {subframe.value}")
            elif isinstance(subframe, ErrorSubframe):
                print(f"Error on register {subframe.register}: {subframe.error_code}")
    """
    # Ensure we have bytes
    if not isinstance(data, bytes):
        data = bytes(data)
    stream = Stream(data)

    while stream.remaining():
        cmd = stream.read_int8() & 0xff  # ensure unsigned

        upper = cmd & 0xf0
        maybe_type = (cmd & 0b00001100) >> 2
        maybe_num = cmd & 0b00000011

        if upper == READ_BASE:
            # READ frame: requests register values
            if maybe_num > 0:
                length = maybe_num
            else:
                length = stream.read_varuint()
            if length == 0:
                # Empty read, skip
                continue
            current_reg = stream.read_varuint()
            for i in range(length):
                yield RegisterSubframe(
                    type=SubframeType.READ,
                    register=current_reg + i,
                    resolution=maybe_type,
                    value=None)

        elif upper == WRITE_BASE or upper == REPLY_BASE:
            # WRITE or RESPONSE frame: contains register values
            if maybe_num > 0:
                length = maybe_num
            else:
                length = stream.read_varuint()
            if length == 0:
                # Empty write/response, skip
                continue
            current_reg = stream.read_varuint()
            frame_type = SubframeType.WRITE if upper == WRITE_BASE else SubframeType.RESPONSE
            for i in range(length):
                yield RegisterSubframe(
                    type=frame_type,
                    register=current_reg + i,
                    resolution=maybe_type,
                    value=stream.read_type(maybe_type))

        elif cmd == WRITE_ERROR or cmd == READ_ERROR:
            # Error frame
            register = stream.read_varuint()
            error_code = stream.read_varuint()
            yield ErrorSubframe(
                type=SubframeType.WRITE_ERROR if cmd == WRITE_ERROR else SubframeType.READ_ERROR,
                register=register,
                error_code=error_code)

        elif cmd == STREAM_CLIENT_DATA:
            # Stream from client to server: channel (1 byte), size (1 byte), data
            channel = stream.read_varuint()
            size = stream.read_varuint()
            data_bytes = stream.read_bytes(size)
            yield StreamSubframe(
                type=SubframeType.STREAM_CLIENT_TO_SERVER,
                channel=channel,
                data=data_bytes)

        elif cmd == STREAM_SERVER_DATA:
            # Stream from server to client: channel (1 byte), size (varuint), data
            channel = stream.read_varuint()
            size = stream.read_varuint()
            data_bytes = stream.read_bytes(size)
            yield StreamSubframe(
                type=SubframeType.STREAM_SERVER_TO_CLIENT,
                channel=channel,
                data=data_bytes)

        elif cmd == STREAM_CLIENT_POLL:
            # Poll for stream data: channel (1 byte), max_length (1 byte)
            channel = stream.read_varuint()
            max_length = stream.read_varuint()
            # Store max_length in data as a single byte for reference
            yield StreamSubframe(
                type=SubframeType.STREAM_CLIENT_POLL_SERVER,
                channel=channel,
                data=bytes([max_length]))

        elif cmd == NOP:
            # No operation, skip
            continue

        else:
            # Unknown command, stop parsing to avoid corruption
            break


class RegisterParser:
    """This can be used as a helper for parsing multiplex formatted
    register replies."""

    def __init__(self, data):
        """
        Arguments:

         data: a 'bytes' containing multiplex response data
        """
        # Create generator and consume incrementally
        self._parser = parse_frame(data)
        self._current = None
        self._exhausted = False

    def next(self):
        """
        Return metadata about the next available register value in the stream.

         Return, tuple: (valid, register_number, resolution)

        valid: boolean, False if no more register data is present
        register_number: integer register number
        resolution: One of INT8/INT16/INT32/F32
        """
        if self._exhausted:
            return (False, 0, INT8)

        # Find the next RESPONSE subframe
        while True:
            try:
                subframe = next(self._parser)
            except StopIteration:
                self._exhausted = True
                return (False, 0, INT8)

            if (isinstance(subframe, RegisterSubframe) and
                subframe.type == SubframeType.RESPONSE):
                self._current = subframe
                return (True, subframe.register, subframe.resolution)
            # Skip non-RESPONSE subframes

    def read(self, resolution):
        """Consume and return a single value from the stream.

        Note: The resolution parameter is kept for API compatibility but
        the value is already decoded based on the resolution in the frame.
        """
        if self._current is None:
            raise RuntimeError("must call next() before read()")
        return self._current.value

    def nanify(self, value, resolution):
        """The multiplex register protocol uses the negative-most value to
        represent NaN.  Convert from raw integers to integers with
        possible NaN.
        """

        if resolution == INT8 and value == -128:
            return math.nan
        elif resolution == INT16 and value == -32768:
            return math.nan
        elif resolution == INT32 and value == -2147483648:
            return math.nan
        return value

    def read_mapped(self, resolution, int8_scale, int16_scale, int32_scale):
        """Read a single register value, where the scale value for each
        resolution is known.
        """

        scales = [int8_scale, int16_scale, int32_scale, 1.0]
        return self.nanify(self.read(resolution), resolution) * scales[resolution]

    def read_int(self, resolution):
        """Read a single register value, which is assumed to represent an
        integer value.
        """

        return int(self.read(resolution))


class QueryParser:
    '''Parse a query to see what fields will be queried and at what
    resolution.'''

    def __init__(self, data):
        # Create generator and consume incrementally
        self._parser = parse_frame(data)
        self._exhausted = False

    @staticmethod
    def parse(data):
        qp = QueryParser(data)
        return [x for x in qp]

    def __iter__(self):
        return self

    def __next__(self):
        if self._exhausted:
            raise StopIteration()

        # Find the next READ subframe
        while True:
            try:
                subframe = next(self._parser)
            except StopIteration:
                self._exhausted = True
                raise

            if (isinstance(subframe, RegisterSubframe) and
                subframe.type == SubframeType.READ):
                return (subframe.register, subframe.resolution)


class WriteFrame:
    """Provides helper methods for writing structured data into a byte
    buffer."""

    def __init__(self, buf):
        self._buf = buf
        self._size = 0

    def size(self):
        return self._size

    def write(self, value, resolution):
        self._buf.write(TYPES[resolution].pack(value))
        self._size += TYPES[resolution].size

    def write_int8(self, value):
        self._buf.write(TYPES[INT8].pack(value))
        self._size += 1

    def write_int16(self, value):
        self._buf.write(TYPES[INT16].pack(value))
        self._size += 2

    def write_int32(self, value):
        self._buf.write(TYPES[INT32].pack(value))
        self._size += 4

    def write_f32(self, value):
        self._buf.write(TYPES[F32].pack(value))
        self._size += 4

    def write_varuint(self, value):
        while True:
            this_byte = value & 0x7f
            value >>= 7
            this_byte |= 0x80 if value else 0x00
            self._buf.write(bytes([this_byte]))
            self._size += 1

            if value == 0:
                break

    def write_mapped(self, value, int8_scale, int16_scale, int32_scale, resolution):
        scales = [int8_scale, int16_scale, int32_scale, 1.0]
        self.write(saturate(value, resolution, scales[resolution]), resolution)


class WriteCombiner:
    """Determines how to group registers when encoding them to minimize
    the required bytes.  This just uses a heuristic, but does a decent
    job still.
    """

    def __init__(self, writer, base_command, start_register, resolutions):
        self.writer = writer
        self.base_command = base_command
        self.start_register = start_register
        self.resolutions = resolutions
        self.reply_size = 0
        self._offset = 0
        self._current_resolution = -1

    def size(self):
        return len(self.resolutions)

    def maybe_write(self):
        this_offset = self._offset
        self._offset += 1

        if self._current_resolution == self.resolutions[this_offset]:
            # We don't need to write any register operations here, and
            # the value should go out only if requested.
            return self._current_resolution != IGNORE

        # We need to do some kind of framing.  See how far ahead the
        # new resolution goes.
        new_resolution = self.resolutions[this_offset]
        self._current_resolution = new_resolution

        if new_resolution == IGNORE:
            # We are now in a block of ignores.
            return False

        count = 1
        i = this_offset + 1
        while True:
            if not (i < len(self.resolutions) and self.resolutions[i] == new_resolution):
                break
            i += 1
            count += 1

        write_command = [0x00, 0x04, 0x08, 0x0c][new_resolution] | self.base_command

        start_size = self.writer.size()

        if count <= 3:
            # Use the shorthand formulation.
            self.writer.write_int8(write_command + count)
        else:
            # Nope, the long form.
            self.writer.write_int8(write_command)
            self.writer.write_int8(count)

        self.writer.write_varuint(self.start_register + this_offset)

        end_size = self.writer.size()

        self.reply_size += end_size - start_size
        self.reply_size += count * resolution_size(new_resolution)

        return True
