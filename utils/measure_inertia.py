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

'''Measure the moment of inertia of a system connected to a moteus
controller.

For motors with only hall sensors (and some others), it is often
necessary to use non-standard parameters to get a meaningful
measurement.

'''


import argparse
import asyncio
import math
import moteus
import numpy as np
import time


async def main():
    parser = argparse.ArgumentParser()

    moteus.make_transport_args(parser)

    parser.add_argument('--target', '-t', default=1, type=int)
    parser.add_argument('--scale', default=1.3, type=float)
    parser.add_argument('--count', default=30, type=int)
    parser.add_argument('--min-vel-std', default=0.003, type=float)

    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)
    c = moteus.Controller(id=args.target, transport=transport)
    s = moteus.Stream(c)

    pll_filter_hz = float(await s.command(b'conf get motor_position.sources.0.pll_filter_hz', allow_any_response=True))
    if pll_filter_hz < 400:
        raise RuntimeError(f'The controller must be calibrated with >= 400Hz BW, measured {pll_filter_hz}')

    # We do not want any max desired rate interfering with our tests.
    await s.command(b'conf set servo.pid_dq.max_desired_rate 10000000')

    try:
        await run_test(args, c)
    finally:
        await c.set_stop()


async def run_test(args, c):
    scale_threshold = 0.5 * (args.scale - 1) + 1

    await c.set_stop()

    await asyncio.sleep(1)

    # Measure the noise in velocity.
    velocity_readings = [(await c.query()).values[moteus.Register.VELOCITY] for _ in range(50)]
    velocity_std = max(args.min_vel_std, np.std(velocity_readings))

    print(f"Velocity stddev = {velocity_std}")


    torque = 0.001

    results = {}
    last_end_velocity = None

    while True:
        print(f"trying torque={torque} ", end='', flush=True)
        await c.set_stop()

        trace = []
        for _ in range(args.count):
            now = time.time()
            trace.append((now, await c.set_position(position=math.nan, kp_scale=0.0, kd_scale=0.0, ilimit_scale=0.0, feedforward_torque=torque, ignore_position_bounds=True, query=True)))
            await asyncio.sleep(0.001)

        print(f"len(trace)={len(trace)}")

        finish_time = None
        while True:
            now = time.time()
            data = await c.set_brake(query=True)

            await asyncio.sleep(0.001)

            if abs(data.values[moteus.Register.VELOCITY]) < 0.1 and finish_time is None:
                finish_time = now + 0.5

            if finish_time and now > finish_time:
                break

        # If the velocity difference over our interval is more than
        # X times the noise level, then save this one.
        these_velocities = [r.values[moteus.Register.VELOCITY] for ts, r in trace]
        delta_velocity = max(these_velocities) - min(these_velocities)
        end_velocity = trace[-1][1].values[moteus.Register.VELOCITY]

        if last_end_velocity is not None:
            velocity_threshold = 500 * velocity_std
            if (delta_velocity > velocity_threshold and
                end_velocity < scale_threshold * last_end_velocity):
                print(f"Finish because delta_velocity ({delta_velocity}) > ({velocity_threshold}) and end_velocity < {scale_threshold} * last")
                break

        if data.values[moteus.Register.MODE] == 1:
            # This ended in a fault, so stop here too.
            print(f"Finish because of fault: {data.values[moteus.Register.FAULT]}")
            break

        velocity_threshold = 4000 * velocity_std
        if delta_velocity > velocity_threshold:
            # We definitely have enough data.
            print(f"Finish because delta_velocity ({delta_velocity}) > {velocity_threshold}")
            break

        results[torque] = trace

        last_end_velocity = end_velocity

        torque = torque * args.scale

    print()

    # Find the largest velocity change among all our traces, then
    # discard anything that has less than X% of that change.
    def velocity_change(trace):
        start_index = len(trace) // 4
        end_index = len(trace) // 2

        return abs(trace[end_index][1].values[moteus.Register.VELOCITY] -
                   trace[start_index][1].values[moteus.Register.VELOCITY])

    largest_velocity_change = max(
        [velocity_change(trace) for _, trace in results.items()])

    results = {
        key: trace
        for key, trace in results.items()
        if velocity_change(trace) > 0.10 * largest_velocity_change
    }

    def estimate_accel_from_trace(trace):
        # In order to avoid effects caused by saturation and startup,
        # we look to capture the acceleration during the period
        # between 25% and 50% of the captured trace.

        start_index = len(trace) // 4
        end_index = len(trace) // 2
        end_time, end_data = trace[end_index]
        start_time, start_data = trace[start_index]

        delta_time = end_time - start_time
        end_velocity = end_data.values[moteus.Register.VELOCITY]

        accel = end_velocity / delta_time

        print(f"end_velocity={end_velocity} delta_time={delta_time} accel={accel}")

        return accel

    inertias = []
    for torque, trace in results.items():
        accel_hz_s = estimate_accel_from_trace(trace)

        if accel_hz_s == 0:
            continue

        inertia = torque / accel_hz_s / (2 * math.pi)
        print(f"torque={torque} inertia={inertia}")

        inertias.append(inertia)

    inertias.sort()
    print()
    print(f"Inertia: {np.median(inertias)}")


if __name__ == '__main__':
    asyncio.run(main())
