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

'''Show a single data field repeatedly.'''

import argparse
import asyncio
import moteus
import time


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', '-n', default='servo_stats')
    parser.add_argument('--period', '-p', type=float, default=0.1)
    parser.add_argument('--filter', '-f', default=None)
    args = parser.parse_args()

    c = moteus.Controller()
    s = moteus.Stream(c)

    await s.command(b"tel text")

    next_iteration = time.time()

    while True:
        now = time.time()
        await asyncio.sleep(max(next_iteration - now, 0))

        result = await s.command(f"tel get {args.name}".encode('utf8'))
        print()
        print()
        lines = result.decode('utf8').split('\n')
        if args.filter:
            lines = [x for x in lines if args.filter in x]
        print('\n'.join(lines))

        next_iteration += args.period


if __name__ == '__main__':
    asyncio.run(main())
