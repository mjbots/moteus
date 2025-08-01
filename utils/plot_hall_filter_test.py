#!/usr/bin/python3 -B

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

import argparse
import matplotlib.pyplot as plt
import sys

def plot(ax, ax2, fname, plot_all, suffix, lw):
    lines = open(fname).readlines()
    header = lines[0].strip().split(',')

    data = [tuple(float(y) for y in x.strip().split(','))
            for x in lines[1:]]

    def get(name, line):
        return line[header.index(name)]

    x_time = [get('time', x) for x in data]

    if plot_all:
        ax.plot(x_time, [get('truth_vel', x) for x in data], '--', color='aquamarine', label='truth_vel')

    ax.plot(x_time, [get('velocity', x) for x in data], label=f'velocity{suffix or ''}', lw=lw)

    if ax2:
        if plot_all:
            ax2.plot(x_time, [get('compensated', x) for x in data], '.', label='compensated')
            ax2.plot(x_time, [get('truth_pos', x) for x in data], '--', label='truth_pos')

        ax2.plot(x_time, [get('filtered', x) for x in data], label=f'filtered{suffix or ''}', lw=lw)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('file', nargs='+')
    parser.add_argument('--skip-position', action='store_true')
    parser.add_argument('--title')

    args = parser.parse_args()

    fig1, ax = plt.subplots()
    if args.skip_position:
        ax2 = None
    else:
        ax2 = ax.twinx()

    first = True
    lw = len(args.file)
    for fname in args.file:
        suffix = None
        if ':' in fname:
            fname, suffix = fname.split(':')
        plot(ax, ax2, fname, first, suffix, lw=lw)
        first = False
        lw -= 1

    ax.legend(loc='upper right')

    ax.set_ylabel('velocity (counts/s)')
    ax.set_xlabel('time (s)')

    if ax2:
        ax2.legend(loc='upper left')
        ax2.set_ylabel('position (counts)')

    if args.title:
        plt.title(args.title)
    plt.show()


if __name__ == '__main__':
    main()
