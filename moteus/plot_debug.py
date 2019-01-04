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

import collections
import math
import numpy
import pylab
import struct
import sys

DATA_FORMAT = struct.Struct('<bbhhhhb')
DATA_NT = collections.namedtuple(
    'Data',
    ['etheta',
     'command_d_A', 'measured_d_A',
     'pid_d_p', 'pid_d_i', 'control_d_V',
     'velocity'])

def main():
    data = open(sys.argv[1], "rb").read()

    timestamps = []
    sampled = []

    offset = 0
    ts = 0
    while offset + DATA_FORMAT.size < len(data):
        if data[offset] != 0x5a:
            offset += 1
            continue

        raw_data = DATA_NT._make(DATA_FORMAT.unpack(
            data[offset + 1:offset + 1 + DATA_FORMAT.size]))
        scaled_data = DATA_NT(
            etheta = raw_data.etheta / 255.0 * (2.0 * math.pi),
            command_d_A = raw_data.command_d_A / 2.0,
            measured_d_A = raw_data.measured_d_A / 500.0,
            pid_d_p = raw_data.pid_d_p / 32767.0 * 12.0,
            pid_d_i = raw_data.pid_d_i / 32767.0 * 12.0,
            control_d_V = raw_data.control_d_V / 32767.0 * 12.0,
            velocity = raw_data.velocity / 127.0 * 10.0,
        )

        offset += 1 + DATA_FORMAT.size

        timestamps.append(ts)
        sampled.append(scaled_data)

        ts += 1 / 40000.0

    fig, ax1 = pylab.subplots()
    pylab.margins(xmargin=0.2)

    ax1.plot(timestamps, [x.command_d_A for x in sampled], 'r', label="command")
    ax1.plot(timestamps, [x.measured_d_A for x in sampled], 'y', label="measured")
    ax1.legend(loc='upper left')

    ax2 = ax1.twinx()

    ax2.plot(timestamps, [x.etheta for x in sampled], 'blue', label="etheta")
    ax2.plot(timestamps, [x.velocity for x in sampled], 'black', label='velocity')
    ax2.legend(loc='upper right')

    ax3 = ax1.twinx()
    ax3.spines['right'].set_position(('outward', 30))
    ax3.plot(timestamps, [x.pid_d_p for x in sampled], label='pid.p')
    ax3.plot(timestamps, [x.pid_d_i for x in sampled], label='pid.i')
    ax3.plot(timestamps, [x.control_d_V for x in sampled], label='cmd')
    ax3.legend(loc='lower right')


    pylab.show()


if __name__ == '__main__':
    main()
