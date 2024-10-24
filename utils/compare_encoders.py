#!/usr/bin/python3 -B

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
import asyncio
import math
import matplotlib
import matplotlib.pyplot as plt
import moteus
import numpy
import time

import histogram

PRINT_DURATION = 0.1

def wrap_half(value):
    while value > 0.5:
        value -= 1.0
    while value < -0.5:
        value += 1.0
    return value


def wrap_zero_one(value):
    while value > 1.0:
        value -= 1.0
    while value < 0.0:
        value += 1.0
    return value


def get_encoder(item, number):
    if number == 0:
        return item.values[moteus.Register.ENCODER_0_POSITION]
    elif number == 1:
        return item.values[moteus.Register.ENCODER_1_POSITION]
    elif number == 2:
        return item.values[moteus.Register.ENCODER_2_POSITION]


async def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--target', '-t', type=int, default=1)

    parser.add_argument('--reference-encoder', '-r', type=int, default=1)
    parser.add_argument('--measure-encoder', '-m', type=int, default=0)
    parser.add_argument('--velocity', '-v', type=float, default=0.1)
    parser.add_argument('--revolutions', type=float, default=2)
    parser.add_argument('--period', type=float, default=0.005)
    parser.add_argument('--kp-scale', type=float, default=1.0)
    parser.add_argument('--r-measure', action='store_true')
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--output', '-o', type=str, default=None)

    args = parser.parse_args()

    encoders = {}
    encoders[args.reference_encoder] = True
    encoders[args.measure_encoder] = True

    if len(encoders) != 2:
        print("Two distinct encoders must be selected")
        return

    qr = moteus.QueryResolution()

    qr._extra[moteus.Register.ENCODER_0_POSITION] = moteus.F32
    qr._extra[moteus.Register.ENCODER_1_POSITION] = moteus.F32
    qr._extra[moteus.Register.ENCODER_2_POSITION] = moteus.F32

    m = moteus.Controller(id=args.target, query_resolution=qr)
    s = moteus.Stream(m)

    await s.write_message(b"tel stop")
    await s.flush_read()
    await s.command(b"d stop")


    position_min = await histogram.read_config_double(s, "servopos.position_min")
    position_max = await histogram.read_config_double(s, "servopos.position_max")

    await s.command(b'conf set servopos.position_min nan')
    await s.command(b'conf set servopos.position_max nan')

    try:
        results = await run(args, m)
    finally:
        await s.flush_read()
        await m.set_stop()
        await s.command(
            f'conf set servopos.position_min {position_min}'.encode('utf8'))
        await s.command(
            f'conf set servopos.position_max {position_max}'.encode('utf8'))

    reference_values = [get_encoder(x, args.reference_encoder) for x in results]
    measure_values = [get_encoder(x, args.measure_encoder) for x in results]

    offset = numpy.mean([wrap_zero_one(r - m) for r, m in zip(reference_values, measure_values)])

    error_values = [-wrap_half(r - offset - m) for r, m in zip(reference_values, measure_values)]

    if args.output:
        with open(args.output, 'w') as out:
            for r, m in zip(reference_values, measure_values):
                print(r, m, file=out)
        return

    fig, ax = plt.subplots()
    x_values = reference_values if args.r_measure else measure_values
    xencoder = args.reference_encoder if args.r_measure else args.measure_encoder
    ax.plot(x_values, error_values, '+')

    ax.yaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(xmax=1))
    ax.set_ylabel('position deviation')

    ax.set_xlabel(f'encoder {xencoder} position')

    max_error = max([abs(x) for x in error_values])
    stddev_error = numpy.std(error_values)

    print()
    print(f"Max error: {max_error*100:.3f}%")
    print(f"Stddev error: {stddev_error*100:.3f}%")

    if args.plot:
        plt.show()


async def run(args, m):
    results = []
    start_time = time.time()
    last_print = start_time
    total_duration = args.revolutions / args.velocity

    print(f"Running for {total_duration:.0f}s")
    while True:
        results.append(await m.set_position(
            position=math.nan, velocity=args.velocity,
            kp_scale=args.kp_scale,
            query=True))
        last_result = results[-1]

        fault = last_result.values[moteus.Register.FAULT]
        if fault != 0:
            raise RuntimeError(f"Fault: {fault}")

        await asyncio.sleep(args.period)

        now = time.time()
        duration = now - start_time
        if duration > total_duration:
            break

        if now - last_print > PRINT_DURATION:
            last_print = now
            print(f"r:{get_encoder(last_result, args.reference_encoder):6.3f} " +
                  f"m:{get_encoder(last_result, args.measure_encoder):6.3f}",
                  end='\r', flush=True)

    print()

    return results


if __name__ == '__main__':
    asyncio.run(main())
