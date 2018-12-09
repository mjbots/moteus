#!/usr/bin/python3 -B

# Copyright 2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

'''Plot debug binary data emitted from the moteus controller.'''

import pylab
import struct
import sys

def main():
    data = open(sys.argv[1], "rb").read()

    command = []
    measured = []
    timestamps = []

    offset = 0
    ts = 0
    while offset + 4 < len(data):
        if data[offset] != 0x5a:
            offset += 1
            continue

        s1, s2 = struct.unpack('<bh', data[offset+1:offset+4])

        offset += 4

        timestamps.append(ts)
        command.append(s1 / 10.0)
        measured.append(s2 / 1000.0)

        ts += 1/40000.0

    pylab.plot(timestamps, command, label="command")
    pylab.plot(timestamps, measured, label="measured")

    pylab.show()


if __name__ == '__main__':
    main()
