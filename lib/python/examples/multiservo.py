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

"""This example commands multiple servos connected to a system.  It
uses the uses the .cycle() method in order to optimally use bandwidth
across any connected CAN-FD interfaces.
"""

import argparse
import asyncio
import math
import moteus
import time

async def main():
    parser = argparse.ArgumentParser()
    moteus.make_transport_args(parser)
    args = parser.parse_args()

    # Explicitly make a transport, so that we can call cycle onit.
    transport = moteus.get_singleton_transport(args)

    # Discover all connected devices.
    #
    # This will return a list of moteus.DeviceInfo structures.
    devices = await transport.discover()

    # You could alternately hard-code a list of servos, either by
    # specifying integer CAN IDs, or by using the
    # 'moteus.DeviceAddress' structure.
    addresses = [x.address for x in devices]

    # We create one 'moteus.Controller' instance for each servo.  It
    # is not strictly required to pass a 'transport' since we do not
    # intend to use any 'set_*' methods, but it doesn't hurt.
    #
    # This syntax is a python "dictionary comprehension":
    # https://docs.python.org/3/tutorial/datastructures.html#dictionaries
    servos = [moteus.Controller(id=address, transport=transport)
              for address in addresses]

    # We will start by sending a 'stop' to all servos, in the event
    # that any had a fault.
    await transport.cycle([x.make_stop() for x in servos])

    while True:
        # The 'cycle' method accepts a list of commands, each of which
        # is created by calling one of the `make_foo` methods on
        # Controller.  The most common thing will be the
        # `make_position` method.

        now = time.time()

        # For now, we will just construct a position command for each
        # of the servos, each of which consists of a sinusoidal
        # velocity command starting from wherever the servo was at to
        # begin with.
        #
        # 'make_position' accepts optional keyword arguments that
        # correspond to each of the available position mode registers
        # in the moteus reference manual.
        commands = [
            servo.make_position(position=math.nan,
                                velocity=0.1 * math.sin(now + i),
                                query=True)
            for i, servo in enumerate(servos)
        ]

        # By sending all commands to the transport in one go, the
        # library can send out commands and retrieve responses
        # simultaneously from all interfaces.  It can also pipeline
        # commands and responses for multiple servos on the same bus.
        results = await transport.cycle(commands)

        # The result is a list of 'moteus.Result' types, each of which
        # identifies the servo it came from, and has a 'values' field
        # that allows access to individual register results.
        #
        # Note: It is possible to not receive responses from all
        # servos for which a query was requested.
        #
        # Here, we'll just print the ID, position, and velocity of
        # each servo for which a reply was returned.
        print(", ".join(
            f"({result.arbitration_id:04X} " +
            f"{result.values[moteus.Register.POSITION]} " +
            f"{result.values[moteus.Register.VELOCITY]})"
            for result in results))

        # We will wait 20ms between cycles.  By default, each servo
        # has a watchdog timeout, where if no CAN command is received
        # for 100ms the controller will enter a latched fault state.
        await asyncio.sleep(0.02)


if __name__ == '__main__':
    asyncio.run(main())
