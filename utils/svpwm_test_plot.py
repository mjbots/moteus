#!/usr/bin/python

# Copyright 2024 mjbots Robotic Systems, LLC.  info@mjbots.com
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
import math
import matplotlib.pyplot as plt
import numpy
import sys


SQRT3_4 = math.sqrt(3 / 4)


class SinCos:
    def __init__(self, theta):
        self.s = math.sin(theta)
        self.c = math.cos(theta)


class InverseDqTransform:
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    @staticmethod
    def from_dq(sc, d, q):
        a = sc.c * d - sc.s * q
        b = (SQRT3_4 * sc.s - 0.5 * sc.c) * d - (-SQRT3_4 * sc.c - 0.5 * sc.s) * q
        c = (-SQRT3_4 * sc.s - 0.5 * sc.c) * d - (SQRT3_4 * sc.c - 0.5 * sc.s) * q

        return InverseDqTransform(a, b, c)

    @staticmethod
    def from_values(a, b, c):
        return InverseDqTransform(a, b, c)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--vq', type=float, default=0)
    parser.add_argument('--vd', type=float, default=0)
    parser.add_argument('--title', type=str, default=None)
    parser.add_argument('--offset-type', type=str, default=None)

    args = parser.parse_args()

    if args.offset_type is None:
        args.offset_type = 'spwm'

    limit = 2 * math.pi * 2
    thetas = list(numpy.arange(0, limit, math.pi / 200))

    def apply_pwm(theta):
        if args.offset_type == 'spwm':
            # Sinusoidal PWM
            x = InverseDqTransform.from_dq(SinCos(theta), args.vd, args.vq)
            return InverseDqTransform.from_values(x.a + 0.5,
                                                  x.b + 0.5,
                                                  x.c + 0.5)
        elif args.offset_type == 'balanced':
            # This is the approach used by moteus prior to SVPWM,
            # which works out to be exactly the same as SPWM.
            x = InverseDqTransform.from_dq(SinCos(theta), args.vd, args.vq)
            center = (x.a + x.b + x.c) / 3
            return InverseDqTransform.from_values(x.a + 0.5 - center,
                                                  x.b + 0.5 - center,
                                                  x.c + 0.5 - center)
        elif args.offset_type == 'minmax':
            # The "minmax" approach used to implement SVPWM in moteus.
            x = InverseDqTransform.from_dq(SinCos(theta), args.vd, args.vq)
            low = min(x.a, x.b, x.c)
            high = max(x.a, x.b, x.c)
            mid = 0.5 * (low + high)
            return InverseDqTransform.from_values(x.a + 0.5 - mid,
                                                  x.b + 0.5 - mid,
                                                  x.c + 0.5 - mid)

        raise RuntimeError(f'unknown offset type: {args.offset_type}')


    phases = [apply_pwm(t) for t in thetas]
    plt.plot(thetas, [x.a for x in phases], lw=3, label='a')
    plt.plot(thetas, [x.b for x in phases], lw=3, label='b')
    plt.plot(thetas, [x.c for x in phases], lw=3, label='c')

    def motor_power(idt):
        return (idt.a - idt.b) ** 2 + (idt.a - idt.c) ** 2 + (idt.b - idt.c) ** 2

    plt.plot(thetas, [motor_power(x) for x in phases], lw=2, label='normalized power')

    plt.xlabel('Electrical Angle (radians)')
    plt.ylabel('Phase PWM')

    plt.xticks(numpy.arange(0, limit, math.pi/4))
    plt.yticks([0, 0.25, 0.5, 0.75, 1.0])

    plt.legend(loc='upper right')
    plt.grid()

    if args.title is not None:
        plt.title(args.title)

    plt.show()




if __name__ == '__main__':
    main()
