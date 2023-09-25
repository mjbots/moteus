#!/usr/bin/python3

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
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plot-encoder', action='store_true')
    parser.add_argument('file')

    args = parser.parse_args()

    data = [x.strip() for x in open(args.file).readlines()[1:-1]]

    fields = [x.split(' ') for x in data]

    def get(desired_key, data):
        result = []
        for line in data:
            if isinstance(desired_key, int):
                result.append(int(line[desired_key]))
            else:
                for field in line:
                    if '=' not in field:
                        continue
                    key, val = field.split('=')
                    if key == desired_key:
                        result.append(int(val))
                        break
                else:
                    raise RuntimeError(f"Key {desired_key} not found")
        return result

    ax1 = plt.subplot(211 if args.plot_encoder else 111)
    ax1.plot(get("i1", fields), label='cur1')
    ax1.plot(get("i2", fields), label='cur2')
    ax1.plot(get("i3", fields), label='cur3')
    ax1.legend()

    if args.plot_encoder:
        ax2 = plt.subplot(212)
        ax2.plot(get(2, fields), label='enc')
        ax2.legend()

    plt.show()


if __name__ == '__main__':
    main()
