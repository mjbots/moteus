#!/usr/bin/python3

# Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

"""Compares cycle counts measured on hardware against those the
emulator's timing model predicts.

  ./compare.py hardware_cycles.txt [model_cycles.txt]
"""

import os
import sys


def read(path):
    result = {}
    for line in open(path):
        if line.startswith('#') or not line.strip():
            continue
        name, cycles = line.split()
        result[name] = int(cycles)
    return result


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    here = os.path.dirname(os.path.abspath(__file__))
    hardware = read(sys.argv[1])
    model = read(sys.argv[2] if len(sys.argv) > 2
                 else os.path.join(here, 'model_cycles.txt'))

    wrong = 0
    print(f'{"benchmark":28} {"hardware":>8} {"model":>8} {"error":>6}')
    for name, measured in hardware.items():
        predicted = model.get(name)
        if predicted is None:
            print(f'{name:28} {measured:8} {"-":>8}')
            continue
        error = predicted - measured
        wrong += error != 0
        flag = '' if error == 0 else f'{error:+6}'
        print(f'{name:28} {measured:8} {predicted:8} {flag}')
    print()
    print(f'{wrong} of {len(hardware)} differ')


if __name__ == '__main__':
    main()
