#!/usr/bin/python3 -B

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

import argparse
import matplotlib.pyplot as plt
import numpy as np
import struct


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', type=str)
    parser.add_argument('--pwm-hz', default=30000, type=int)
    parser.add_argument('-s', '--struct', default='hhh')

    args = parser.parse_args()

    s = struct.Struct('<b' + args.struct)

    data = open(args.file, "rb").read()

    for i in range(s.size + 1):
        maybe_all_header = data[slice(i, s.size * 20, s.size)]
        if maybe_all_header == bytes([0x5a]) * len(maybe_all_header):
            print(f"skipping first {i} bytes")
            data = data[i:]
            break
    else:
        raise RuntimeError("Could not locate header in highrate data")

    parsed = [s.unpack(data[i:i+s.size])
              for i in range(0, len(data) - s.size, s.size)]

    def fmt_tuple(l):
        return ','.join([f"{x:04x}" for x in l])

    for i, row in enumerate(parsed):
        if row[0] != 0x5a:
            print(f"Desynchronization at row {i}")
            print(fmt_tuple(parsed[i-1]))
            print(fmt_tuple(parsed[i]))

    size = len(parsed)
    xvals = np.arange(0, size / args.pwm_hz, 1.0 / args.pwm_hz)

    for i, _ in enumerate(parsed[0]):
        if i == 0:
            continue

        plt.plot(xvals, [x[i] for x in parsed],
                 label=f'item {i}')

    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
