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

import math
import struct

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


class RegisterParser:
    """This can be used as a helper for parsing multiplex formatted
    register replies."""

    def __init__(self, data):
        """
        Arguments:

         data: a 'byte' containing multiplex response data
        """
        self.data = data
        self.size = len(data)

        self._offset = 0
        self._remaining = 0
        self._current_register = 0
        self._current_resolution = INT8

    def next(self):
        """
        Return metadata about the next available register value in the stream.

         Return, tuple: (valid, register_number, resolution)

        valid: boolean, False if no more register data is present
        register_number: integer register number
        resolution: One of INT8/INT16/INT32/F32
        """

        if self._offset >= self.size:
            return (False, 0, INT8)

        if self._remaining:
            self._remaining -= 1
            this_register = self._current_register
            self._current_register += 1

            # Do we actually have enough data?
            if self._offset + resolution_size(self._current_resolution) > self.size:
                return (False, 0, INT8)

            return (True, this_register, self._current_resolution)

        # We need to look for more data.
        while self._offset < self.size:
            cmd = self.data[self._offset]
            self._offset += 1

            if cmd == NOP:
                continue

            if self._offset >= self.size:
                # We are all out.
                break

            if cmd >= 0x20 and cmd < 0x30:
                # This is a regular reply of some sort.
                id = (cmd >> 2) & 0x03
                self._current_resolution = id

                count = cmd & 0x03
                if count == 0:
                    count = self.data[self._offset]
                    self._offset += 1

                    if self._offset >= self.size:
                        # Guess this was malformed.  We'll be done now
                        # anyways.
                        break

                if count == 0:
                    # Empty, guess we can ignore.
                    continue

                maybe_current_register, self._offset = read_varuint(self._offset, self.data)
                if maybe_current_register is None:
                    return (False, 0, INT8)

                self._current_register = maybe_current_register
                self._remaining = count - 1

                if self._offset + resolution_size(self._current_resolution) > self.size:
                    return (False, 0, INT8)

                result_register = self._current_register
                self._current_register += 1
                return (True, result_register, self._current_resolution)

            # For anything else, we'll just assume it is an error of some
            # sort and stop parsing.
            self._offset = self.size
            break

        return (False, 0, INT8)

    def read(self, resolution):
        """Consume and return a single value from the stream."""

        size = resolution_size(resolution)
        if self._offset + size > self.size:
            raise RuntimeError("overrun")

        start = self._offset
        self._offset += size
        return TYPES[resolution].unpack(self.data[start:start+size])[0]

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


class WriteFrame:
    """Provides helper methods for writing structured data into a byte
    buffer."""

    def __init__(self, buf):
        self._buf = buf

    def write(self, value, resolution):
        self._buf.write(TYPES[resolution].pack(value))

    def write_int8(self, value):
        self._buf.write(TYPES[INT8].pack(value))

    def write_int16(self, value):
        self._buf.write(TYPES[INT16].pack(value))

    def write_int32(self, value):
        self._buf.write(TYPES[INT32].pack(value))

    def write_f32(self, value):
        self._buf.write(TYPES[F32].pack(value))

    def write_varuint(self, value):
        while True:
            this_byte = value & 0x7f
            value >>= 7
            this_byte |= 0x80 if value else 0x00
            self._buf.write(bytes([this_byte]))

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
        self._offset = 0
        self._current_resolution = -1

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

        if count <= 3:
            # Use the shorthand formulation.
            self.writer.write_int8(write_command + count)
        else:
            # Nope, the long form.
            self.writer.write_int8(write_command)
            self.writer.write_int8(count)

        self.writer.write_varuint(self.start_register + this_offset)
        return True
