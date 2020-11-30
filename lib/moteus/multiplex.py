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

import struct

"""Constants and helper functions used for constructing and parsing
frames in the mjlib multiplex protocol."""

INT8 = 0
INT16 = 1
INT32 = 2
F32 = 3

WRITE_BASE = 0x00
READ_BASE = 0x10
REPLY_BASE = 0x20
WRITE_ERROR = 0x30
READ_ERROR = 0x31
STREAM_CLIENT_DATA = 0x40
STREAM_SERVER_DATA = 0x41
STREAM_CLIENT_POLL = 0x42
NOP = 0x50


TYPES = {
    INT8: struct.Struct('<b'),
    INT16: struct.Struct('<h'),
    INT32: struct.Struct('<i'),
    F32: struct.Struct('<f'),
}

def read_varuint(stream):
    result = 0
    shift = 0

    for i in range(5):
        data = stream.read(1)
        if len(data) < 1:
            return None
        this_byte, = struct.unpack('<B', data)
        result |= (this_byte & 0x7f) << shift
        shift += 7

        if (this_byte & 0x80) == 0:
            return result

    assert False


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
            if self._offset + resolution_size(self._current_resolution) > size_:
                return (False, 0, INT8)

            return (True, this_register, self._current_resolution)

        # We need to look for more data.
        while self._offset < self._size:
            cmd = self.data[self._offset]
            self._offset += 1

            if cmd == NOP:
                continue

            if self._offset >= self._size:
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

                    if self._offset >= size:
                        # Guess this was malformed.  We'll be done now
                        # anyways.
                        break

                if count == 0:
                    # Empty, guess we can ignore.
                    continue

                self._current_register = self.data[self._offset]
                self._offset += 1
                self._remaining = count - 1

                if self._offset + resolution_size(self._current_register) > self._size:
                    return (False, 0, INT8)

                result_register = self._current_register
                self._current_register += 1
                return (True, result_register, self._current_resolution)

            # For anything else, we'll just assume it is an error of some
            # sort and stop parsing.
            self._offset = self._size
            break

        return (False, 0, INT8)

    def read(self, resolution):
        """Consume and return a single value from the stream."""

        size = resolution_size(resolution)
        if self._offset + size > self._size:
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
        resolution is known."""

        scales = [int8_scale, int16_scale, int32_scale, 1.0]
        return self.nanify(self.read(resolution), resolution) * scales[resolution]

    def read_int(self, resolution):
        return int(self.read(resolution))
