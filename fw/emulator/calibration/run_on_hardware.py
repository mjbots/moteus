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

"""Runs the timing benchmarks on a real STM32G474 through an stlink
and openocd, printing the measured cycles in the same format as
model_cycles.txt.

The controller is reset and halted, the benchmark program is loaded
into RAM and run in place of the firmware, and then the controller is
reset again.  Nothing in flash is changed.  The motor outputs stay
disabled throughout, as the firmware never runs.

  ./run_on_hardware.py > hardware_cycles.txt

"""

import argparse
import os
import re
import subprocess
import sys
import tempfile

# These must match make_benchmarks.py.
CODE = 0x10000000
RESULTS = 0x20000400
STACK = 0x20002000
END_MARKER = 0xd0e5d0e5

THUMB_STATE = 0x01000000


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmarks',
                        default=os.path.join(here, 'benchmarks.txt'))
    parser.add_argument('--openocd', default='openocd')
    parser.add_argument('--interface', default='interface/stlink.cfg')
    parser.add_argument('--target', default='target/stm32g4x.cfg')
    args = parser.parse_args()

    # Each program is a list of names and the bytes to run.
    programs = []
    for line in open(args.benchmarks):
        kind, _, rest = line.strip().partition(' ')
        if kind == 'program':
            programs.append(([], bytearray()))
        elif kind == 'name':
            programs[-1][0].append(rest)
        elif kind == 'code':
            programs[-1][1].extend(bytes.fromhex(rest))

    print('# Measured by run_on_hardware.py.')
    for names, program in programs:
        for name, cycles in run(args, names, program):
            print(f'{name} {cycles}')


def run(args, names, program):
    '''Run one program, returning the name and cycle count of each of
    its benchmarks, less the first.'''
    count = len(names) + 1

    with tempfile.TemporaryDirectory() as tmp:
        binary = os.path.join(tmp, 'benchmarks.bin')
        with open(binary, 'wb') as f:
            f.write(program)

        commands = [
            'init',
            'reset halt',
            f'load_image {binary} 0x{CODE:08x} bin',
            f'reg sp 0x{STACK:08x}',
            f'reg pc 0x{CODE:08x}',
            f'reg xPSR 0x{THUMB_STATE:08x}',
            'resume',
            'sleep 500',
            'halt',
            f'mdw 0x{RESULTS:08x} {count}',
            'reset run',
            'shutdown',
        ]
        command = [args.openocd, '-f', args.interface, '-f', args.target]
        for x in commands:
            command += ['-c', x]
        output = subprocess.run(
            command, check=True, capture_output=True, text=True)

    # mdw prints lines such as "0x20000400: 00000004 00000005 ..."
    words = []
    for line in (output.stdout + output.stderr).splitlines():
        match = re.match(r'^0x[0-9a-fA-F]{8}:((?: [0-9a-fA-F]{8})+)', line)
        if match:
            words += [int(x, 16) for x in match.group(1).split()]

    if len(words) < count or words[len(names)] != END_MARKER:
        sys.stderr.write(output.stdout + output.stderr)
        sys.exit('The benchmarks did not run to completion.')

    return [(name, cycles - words[0]) for name, cycles in zip(names, words)]


if __name__ == '__main__':
    main()
