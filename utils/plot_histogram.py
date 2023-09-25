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
import matplotlib.pyplot as plt
import moteus
import numpy

import histogram

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--time', type=float, default=3)
    parser.add_argument('-s', '--split', type=int, default=1)
    parser.add_argument('-o', '--output')
    parser.add_argument('hist_options', nargs='*')
    args = parser.parse_args()

    c = moteus.Controller()
    s = moteus.Stream(c, verbose=True)

    await s.write_message(b"tel stop")
    await s.flush_read()

    values, splits = await histogram.capture_histogram(
        s,
        split_count=args.split,
        hist_options=args.hist_options,
        sample_time=args.time,
    )

    if args.output:
        with open(args.output, 'w') as f:
            f.write(''.join([f'{x}\n' for x in values]))

    mean = numpy.mean(values)
    stddev = numpy.std(values)

    fig, ax = plt.subplots()
    plt.plot(values)
    disp = ax.transData
    plt.annotate(f"mean={mean}", (10, 10), xycoords="figure pixels")
    plt.annotate(f"stddev={stddev}", (10, 40), xycoords="figure pixels")
    if splits:
        plt.plot(splits, [mean] * len(splits), '+')
    plt.show()


if __name__ == '__main__':
    asyncio.run(main())
