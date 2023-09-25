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
import moteus
import time


async def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('-t', '--target', type=int, default=1)

    moteus.make_transport_args(parser)

    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)
    c = moteus.Controller(args.target, transport=transport)
    s = moteus.Stream(c)

    old_time = None
    old_us = None

    for i in range(5):
        us = (await s.read_data('system_info')).ms_count * 1000
        now = time.time()

        if old_time is not None:
            print((us - old_us) / (now - old_time))

        old_time = now
        old_us = us

        await asyncio.sleep(1.0)


if __name__ == '__main__':
    asyncio.run(main())
