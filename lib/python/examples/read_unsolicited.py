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


"""Transport.cycle(read_unsolited=...) can be used to read from
devices which emit unsolicited data.
"""

import argparse
import asyncio
import moteus


async def main():
    parser = argparse.ArgumentParser()

    moteus.make_transport_args(parser)

    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)

    # We will assume 1 moteus controller exists and another device is
    # emitting unsolicited data both on the first transport device.

    controller = moteus.Controller(id=1)
    unsolicited_td = transport.devices()[0]

    while True:
        commands = [
            controller.make_query(),
        ]

        results = await transport.cycle(
            commands, read_unsolicited=[unsolicited_td])

        print(results)

        await asyncio.sleep(0.2)


if __name__ == '__main__':
    asyncio.run(main())
