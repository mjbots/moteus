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
import asyncio
import json
import matplotlib.pyplot as plt
import moteus
import numpy
import tempfile

import histogram


async def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--target', '-t', type=int, default=1)
    parser.add_argument('--verbose', '-v', action='store_true')
    parser.add_argument('--limit', '-l', type=float, default=0.0)
    parser.add_argument('--measure-velocity', action='store_true')
    parser.add_argument('--measure-error', action='store_true')

    args = parser.parse_args()


    m = moteus.Controller(id=args.target)
    s = moteus.Stream(m, verbose=args.verbose)

    await s.write_message(b"tel stop")
    await s.flush_read()
    await s.command(b"d stop")
    await s.command(b'conf set servopos.position_min nan')
    await s.command(b'conf set servopos.position_max nan')

    TEST_VELOCITIES = [4, 2, 1, 0.5, 0.2, 0.05, 0.01]
    if args.measure_error:
        HIST_OPTIONS = ["ye"]  # error between position and control position
    else:
        HIST_OPTIONS = []  # encoder 0 compensated derivative

    for velocity in TEST_VELOCITIES:
        if velocity < args.limit:
            break
        await s.command(f'd pos nan {velocity} 0.5 a5'.encode('utf8'))
        await asyncio.sleep(2)

        values1, _ = await histogram.capture_histogram(
            stream=s,
            hist_options=HIST_OPTIONS,
            split_count=1,
            sample_time = max(2, abs(2.0 / velocity)))

        await s.command(b'd stop')
        await asyncio.sleep(2)

        await s.command(f'd pos nan {-velocity} 0.5 a5'.encode('utf8'))
        await asyncio.sleep(2)

        values2, _ = await histogram.capture_histogram(
            stream=s,
            hist_options=HIST_OPTIONS,
            split_count=1,
            sample_time = max(2, abs(2.0 / velocity)))

        await s.command(b'd stop')
        await asyncio.sleep(2)

        std1 = numpy.std(values1)
        std2 = numpy.std(values2)
        std = max(std1, std2)

        print(f'{velocity:0.2f} - {std}  ({std1}, {std2})')


    await s.command(b'd stop')


if __name__ == '__main__':
    asyncio.run(main())
