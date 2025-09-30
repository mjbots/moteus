#!/usr/bin/python

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

"""Given a measurement series consisting of current vs measured
torque, calculate an appropriate saturation model to be used with
moteus controllers."""


import argparse
from dataclasses import dataclass
import math
import matplotlib.pyplot as plt
import numpy as np
import scipy
import sys


@dataclass
class SaturationModel:
    '''This is the model that moteus uses to calculate torque.  It is
    linear up until rotation_current_cutoff A, and then logarithmic.
    '''

    torque_constant: float
    rotation_current_cutoff: float
    rotation_current_scale: float
    rotation_torque_scale: float

    def evaluate(self, current):
        kt_current = current * self.torque_constant
        if current < self.rotation_current_cutoff:
            return kt_current

        return (self.rotation_current_cutoff * self.torque_constant +
                self.rotation_torque_scale *
                (math.log2(max(1 + (current - self.rotation_current_cutoff) *
                               self.rotation_current_scale, 0.00001))))


def fit_saturation(current, torque):
    # Pick initial values, they don't have to be particularly close,
    # just in the right ballpark.
    kt = torque[-1] / current[-1]

    rotation_current_cutoff = current[-1] * 0.5
    rotation_current_scale = 0.01
    rotation_torque_scale = 5

    max_current = current[-1]

    def current_weight(i):
        if i / max_current < 0.5:
            return 1

        # Apply larger weights as you get into the saturation region,
        # as that is the part you actually want to fit closely.
        return 5 * (i / max_current) ** 3

    def metric(x, *args):
        sm = SaturationModel(*x)

        return sum([current_weight(i) * (sm.evaluate(i) - t) ** 2
                    for i, t in zip(current, torque)])

    maybe_result = scipy.optimize.minimize(
        metric,
        [
            kt,
            rotation_current_cutoff,
            rotation_current_scale,
            rotation_torque_scale,
        ])

    return SaturationModel(*maybe_result.x)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--title', default=None)
    parser.add_argument('--current-header', '-c', default='measured_current')
    parser.add_argument('--torque-header', '-t', default='dyno_torque')

    parser.add_argument('file')

    args = parser.parse_args()

    # We read a CSV containing the data and expect the first line to
    # be a header with column titles.
    all_lines = open(args.file).readlines()
    header = all_lines[0].strip().split(',')
    data = all_lines[1:]
    xdata = [
        tuple(float(x) for x in line.strip().split(','))
        for line in data
    ]

    torque_index = header.index(args.torque_header)
    current_index = header.index(args.current_header)

    # Extract our data of interest and sort it by current.
    ct = list(sorted([(x[current_index], x[torque_index]) for x in xdata]))
    currents = [x[0] for x in ct]
    torques = [x[1] for x in ct]

    # Fit the model and print it to the console.
    sm = fit_saturation(currents, torques)
    print(sm)
    Kv = 8.3 / sm.torque_constant
    print(f"Kv: {Kv}")

    # Now plot.
    plt.plot(currents, torques, label='data')

    max_x = max(currents) + 1
    fitx = np.arange(0, max_x, max_x / 100)
    plt.plot(fitx, [sm.evaluate(i) for i in fitx], label='model fit')
    plt.xlabel('Q axis current (A)')
    plt.ylabel('Torque (N*m)')
    plt.legend()
    plt.grid()

    if args.title:
        plt.title(args.title)

    plt.show()


if __name__ == '__main__':
    main()
