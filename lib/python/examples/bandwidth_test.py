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


"""This example can be used to determine a rough maximal bandwidth
capability of a given system for a position/query loop.
"""

import argparse
import asyncio
import moteus
import moteus.moteus_tool
import math
import time


STATUS_PERIOD_S = 0.1


async def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('-t', '--target', action='append',
                        help='one or more targets')
    parser.add_argument('--minimal-format', action='store_true',
                        help='use a minimal CAN frame size with lower resolution')

    moteus.make_transport_args(parser)

    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)

    targets = (
        moteus.moteus_tool.expand_targets(args.target)
        if args.target else
        [x.address for x in (await transport.discover())])

    print(f"Testing with {len(targets)} targets:")
    for t in targets:
        print(f" * {t}")
    print()

    qr = moteus.QueryResolution()
    pr = moteus.PositionResolution()

    if args.minimal_format:
        pr.position = moteus.INT16
        pr.velocity = moteus.INT16

        qr.mode = moteus.INT16
        qr.position = moteus.INT16
        qr.velocity = moteus.INT16
        qr.torque = moteus.INT16

    controllers = {
        address: moteus.Controller(
            transport=transport,
            id=address,
            query_resolution=qr,
            position_resolution=pr)
        for address in targets
    }

    hz_count = 0
    status_time = time.time() + STATUS_PERIOD_S

    while True:
        hz_count += 1

        commands = [
            c.make_position(position=math.nan, velocity=0.0, query=True)
            for c in controllers.values()
        ]

        responses = await transport.cycle(commands)

        count = len(responses)

        now = time.time()
        if now > status_time:
            print(f"{hz_count / STATUS_PERIOD_S:6.1f}Hz  rx_count={count}  ",
                  end='\r', flush=True)
            hz_count = 0
            status_time += STATUS_PERIOD_S

        await asyncio.sleep(0)


if __name__ == '__main__':
    asyncio.run(main())
