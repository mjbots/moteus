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


COMPENSATION_SIZE = 32


def integrate(values):
    v = 0
    result = []
    for x in values:
        v += x / len(values)
        result.append(v)
    return result


def smooth(values, poles):
    window_size = 2 * len(values) // poles

    def get_index(i):
        if i >= len(values):
            i -= len(values)
        return values[i]

    def get_window(i):
        return [get_index(i + x)
                for x in range(-window_size // 2, window_size // 2 + 1)]

    result = [
        sum(get_window(i)) / len(get_window(i))
        for i in range(len(values))
    ]
    return result


def sample(values, num_bins):
    xvals = []
    result = []

    for i in range(num_bins):
        start = i * len(values) // num_bins
        end = (i + 1) * len(values) // num_bins

        xvals.append(0.5 * (start + end))
        result.append(sum(values[start:end]) / (end - start))

    return xvals, result


async def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--target', '-t', type=int, default=1)
    parser.add_argument('--encoder-channel', '-c', type=int, default=0)
    parser.add_argument('--voltage', type=float, default=0.4)
    parser.add_argument('--split-count', type=int, default=1)
    parser.add_argument('--plot-original', action='store_true')
    parser.add_argument('--plot-results', action='store_true')
    parser.add_argument('--analyze', action='store_true',
                        help='show the effectiveness of current cal')
    parser.add_argument('--zero', action='store_true',
                        help='force the on-device table to 0s')
    parser.add_argument('--store', action='store_true')
    parser.add_argument('--verbose', '-v', action='store_true')

    args = parser.parse_args()

    if args.store and args.analyze:
        print("Cannot analyze and store simultaneously")
        return

    m = moteus.Controller(id=args.target)
    s = moteus.Stream(m, verbose=args.verbose)

    await s.write_message(b"tel stop")
    await s.flush_read()
    await s.command(b"d stop")

    poles = int(await histogram.read_config_double(s, "motor.poles"))

    await histogram.can_compensate_encoder(s, args.encoder_channel)

    await asyncio.sleep(1.0)

    await s.command(f"d vdq 0 {args.voltage}".encode('utf8'))
    await asyncio.sleep(1.0)

    tap = 'o' if not args.analyze else 'c'

    values, splits = await histogram.capture_histogram(
        stream=s,
        # Get the "offset_value" from the encoder channel, in
        # numerically differentiated form.
        hist_options=[
            f"xo{args.encoder_channel}",
            f"y{tap}{args.encoder_channel}d",
        ],
        split_count=args.split_count,
        sample_time=8)

    await s.command(b"d stop")


    # We want to discard any commutation specific anomalies, so just
    # do a moving average with a size equal to the pole count of the
    # motor.
    smoothed = smooth(values, poles)

    mean = sum(smoothed) / len(smoothed)

    fig, ax = plt.subplots()
    ax2 = ax.twinx()

    if args.plot_original:
        ax.plot(values)
        ax.plot(smoothed)

    unbiased = [(x - mean) / mean for x in smoothed]
    integrated = integrate(unbiased)
    integrated_mean = sum(integrated) / len(integrated)
    unbiased_integrated = [x - integrated_mean for x in integrated]

    xsampled, sampled = sample(unbiased_integrated, COMPENSATION_SIZE)

    if args.plot_results:
        ax.plot([x - mean for x in values], label='raw unbiased')
        ax.plot(unbiased, label='smoothed')

        ax2.plot(unbiased_integrated, label='integrated')
        ax2.plot(xsampled, sampled, label='sampled')

    ax.legend(loc="upper left")
    ax2.legend(loc="upper right")

    if args.store:
        print("Saving new encoder compensation to device")

        for i, x in enumerate(sampled):
            value = -x if not args.zero else 0.0
            await s.command(f"conf set motor_position.sources.{args.encoder_channel}.compensation_table.{i} {value}".encode('utf8'))

        await s.command(b'conf write')
    else:
        print("WARNING: Values not stored to device, --store not specified")

    if args.plot_original or args.plot_results:
        plt.show()


if __name__ == '__main__':
    asyncio.run(main())
