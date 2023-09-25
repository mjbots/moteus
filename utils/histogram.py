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

import asyncio
import moteus
import numpy
import sys

async def capture_histogram(stream,
                            sample_time=None,
                            hist_options=[],
                            split_count=1):
    values = []

    splits = []
    for split_index in range(split_count):
        split_str = ''
        if split_count > 1:
            start = split_index / split_count
            end = (split_index + 1) / split_count
            split_str = f'm{start} M{end}'
            splits.append(len(values))
        cmd_args = [x for x in [
            "d hstart",
            ' '.join(hist_options),
            split_str]
                    if x.strip() != '']
        await stream.command(' '.join(cmd_args).encode('utf8'))
        await asyncio.sleep(sample_time)

        results = (await stream.command(b"d hend")).decode('utf8')
        data = [
            [float(a) for a in x.strip().split(' ')]
            for x in results.split('\n')
            if x.strip() != '']
        values += [x[1] for x in data]

    return values, splits


async def read_config_double(s, name):
    return float((await s.command(
        f"conf get {name}".encode('utf8'),
        allow_any_response=True)).decode('utf8'))


async def can_compensate_encoder(s, encoder_channel):
    motor_position = await s.read_data("motor_position")
    if (not motor_position.sources[encoder_channel].active_velocity or
        not motor_position.sources[encoder_channel].active_theta):
        print(f"Encoder f{encoder_channel} is not ready")
        sys.exit(1)
    config_type = await read_config_double(
        s, f"motor_position.sources.{encoder_channel}.type")
    if config_type == 4:
        print(f"Hall encoders are not suitable for compensation")
        sys.exit(1)
