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
import math
import matplotlib.pyplot as plt
import moteus
import numpy
import sys
import tempfile

import histogram

COGGING_TABLE_SIZE = 1024

async def read_data(args, s, speed=None):
    if args.input:
        with open(args.input) as inf:
            return json.load(inf)

    print('Ensure the motor can move freely with no load')

    estimated_time_min = args.split_count * (args.average_count / speed) * 2 / 60
    print(f'This will take approximately {estimated_time_min:.1f}min')

    await asyncio.sleep(3.0)

    position_min = await histogram.read_config_double(s, "servopos.position_min")
    position_max = await histogram.read_config_double(s, "servopos.position_max")
    output_scale = await histogram.read_config_double(s, "motor_position.rotor_to_output_ratio")

    await s.command(b'conf set servopos.position_min nan')
    await s.command(b'conf set servopos.position_max nan')

    # And we'll operate both forward and backwards.
    result = {}
    for velocity in [-speed, speed]:
        print(f'vel={velocity}')
        await s.command(f"d pos nan {velocity * output_scale} nan a4".encode('utf8'))
        await asyncio.sleep(2)

        values, _ = await histogram.capture_histogram(
            stream=s,
            hist_options=["yq"],
            split_count=args.split_count,
            sample_time = int(args.average_count / speed))

        await s.command(b"d stop")
        await asyncio.sleep(2)

        if any([not math.isfinite(x) for x in values]):
            print(f'Compensation failed.  Ensure that PID values are set for smooth motion at speed={speed * output_scale}')
            sys.exit(1)

        if velocity < 0.0:
            result['reverse'] = values
        else:
            result['forward'] = values

    await s.command(
        f'conf set servopos.position_min {position_min}'.encode('utf8'))
    await s.command(
        f'conf set servopos.position_max {position_max}'.encode('utf8'))

    return result

async def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--input', '-i')
    parser.add_argument('--output', '-o', type=str,
                        default=tempfile.NamedTemporaryFile(
                            prefix='cogging-', suffix='.log').name)
    parser.add_argument('--target', '-t', type=int, default=1)
    parser.add_argument('--verbose', '-v', action='store_true')
    parser.add_argument('--average-count', type=int, default=1)
    parser.add_argument('--speed', type=float, default=2)
    parser.add_argument('--split-count', type=int, default=8)

    parser.add_argument('--plot-source', action='store_true')
    parser.add_argument('--plot-results', action='store_true')

    parser.add_argument('--store', action='store_true')

    args = parser.parse_args()

    m = moteus.Controller(id=args.target)
    s = moteus.Stream(m, verbose=args.verbose)

    await s.write_message(b"tel stop")
    await s.flush_read()
    await s.command(b"d stop")

    poles = int(await histogram.read_config_double(s, "motor.poles"))

    commutation_source = int(await histogram.read_config_double(
        s, "motor_position.commutation_source"))

    await histogram.can_compensate_encoder(s, commutation_source)

    speed = args.speed / poles

    data = await read_data(args, s, speed=speed)

    print('Output saved to: ', args.output)
    with open(args.output, 'w') as of:
        json.dump(data, of)

    fig, ax = plt.subplots()

    if args.plot_source:
        ax.plot(data['reverse'], label='fwd')
        ax.plot(data['forward'], label='rev')

    source_length = len(data['reverse'])
    assert len(data['forward']) == source_length

    indices = []
    average = []

    for i in range(COGGING_TABLE_SIZE):
        start = i * source_length // COGGING_TABLE_SIZE
        end = (i + 1) * source_length // COGGING_TABLE_SIZE

        indices.append(0.5 * (start + end))

        fwd_window = data['forward'][start:end]
        rev_window = data['reverse'][start:end]

        average.append((sum(fwd_window) + sum(rev_window)) /
                       (len(fwd_window) + len(rev_window)))

    if args.plot_results:
        ax.plot(indices, average, label='avg')

    if args.store:
        print("Saving new cogging compensation to device")

        scale = max(abs(max(average)), abs(min(average))) / 127
        scaled_average = [int(x / scale) for x in average]

        await s.command(f'conf set motor.cogging_dq_scale {scale}'.encode('utf8'))
        for i, d in enumerate(scaled_average):
            await s.command(f'conf set motor.cogging_dq_comp.{i} {d}'.encode('utf8'))

        await s.command(b'conf write')
    else:
        print("WARNING: Values not stored to device, --store not specified")

    ax.legend()

    if args.plot_results or args.plot_source:
        plt.show()


if __name__ == '__main__':
    asyncio.run(main())
