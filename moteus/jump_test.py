#!/usr/bin/python3 -B

# Copyright 2019 Josh Pieper, jjp@pobox.com.
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

'''Jump a 3dof leg continuously'''

import argparse
import asyncio
import collections
import enum
import struct
import time


import mjlib.micro.multiplex_protocol as mp
import mjlib.micro.aioserial as aioserial


SERVO_STATS = struct.Struct('<iiHHHHHHHffffHffffiff')
SERVO_STATS_NC = collections.namedtuple('servo_stats', '''mode errc
     adc1_raw adc2_raw adc3_raw
     position_raw fet_temp_raw
     adc1_offset adc2_offset
     cur1_A cur2_A busV filt_bus_V
     position
     fet_temp_C
     electrical_theta
     d_A q_A
     unwrapped_position_raw
     unwrapped_position
     velocity''')

FEMUR_START = 0.09
TIBIA_START = -0.11

async def readline(stream):
    result = bytearray()
    while True:
        char = await stream.read(1)
        if char == b'\r' or char == b'\n':
            if len(result):
                return result
        else:
            result += char


async def read_response(stream):
    result = await readline(stream)
    _ = await stream.read(1)  # ignore the extra '\n' we know the servos always send
    return result


class ServoSet:
    def __init__(self, aio_serial, ids):
        self.aio_serial = aio_serial
        self.manager = mp.MultiplexManager(self.aio_serial)
        self.channels = {
            key: mp.MultiplexClient(
                self.manager,
                timeout=0.04,
                destination_id=key,
                channel=1)
            for key in ids
        }

    async def get_servo_stats(self, id):
        channel = self.channels[id]

        channel.write(b"tel get servo_stats\n")
        await channel.drain()

        line = (await read_response(channel)).strip()
        size_str = await channel.read(4)  # size
        size, = struct.unpack('<i', size_str)
        data = await channel.read(size)

        return SERVO_STATS_NC(*SERVO_STATS.unpack(data[0:SERVO_STATS.size]))


    async def command(self, commands):
        remaining = list(commands.keys())
        results = {}
        # Keep trying to send the commands to each servo until we get
        # a result.
        while len(remaining):
            for key in remaining:
                self.channels[key].write(commands[key])

            await self.manager.drain()

            still_left = remaining[:]
            for key in still_left:
                try:
                    this_result = await asyncio.wait_for(
                        read_response(self.channels[key]), 0.06)
                    results[key] = this_result
                    remaining.remove(key)
                except asyncio.TimeoutError:
                    # We won't have removed this from 'remaining', so
                    # we'll keep trying.
                    pass

        return results


class State(enum.Enum):
    INIT = 1
    WAIT_FOR_INIT = 2
    JUMPING = 3
    RETRACTING = 4
    FALLING = 5
    LANDING = 6
    FAULT = 7


async def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('-d', '--device', type=str, default='/dev/ttyUSB0',
                        help='serial device')
    parser.add_argument('-b', '--baud', type=int, default=3000000,
                        help='baud rate')
    parser.add_argument('--really-run', action='store_true')

    args = parser.parse_args()

    serial = aioserial.AioSerial(port=args.device, baudrate=args.baud)
    # Empty out anything still on the bus.


    try:
        _ = await asyncio.wait_for(serial.read(8192), 0.1)
    except asyncio.TimeoutError:
        pass

    servo_set = ServoSet(serial, [1, 2, 3])

    state = State.INIT

    while True:
        data = {}
        for key in [1, 2, 3]:
            while True:
                try:
                    data[key] = await asyncio.wait_for(
                        servo_set.get_servo_stats(key), 0.04)
                    break
                except asyncio.TimeoutError:
                    # We retry infinitely.
                    pass

        print('{:.6f} {}'.format(time.time(), state))
        for id in [1, 2, 3]:
            print(' {}: {:3d} p {:7.4f}  v {:7.3f}  i {:5.1f}  t {:2.0f}'.format(
                id, data[id].mode,
                data[id].unwrapped_position, data[id].velocity,
                data[id].q_A, data[id].fet_temp_C))
        print()

        for key, value in data.items():
            if value.mode == 1:
                # We have a fault on one servo.  Stop the state machine.
                state = State.FAULT
                break

        if state == State.INIT:
            # Command a low current maneuver to the starting position.
            if args.really_run:
                await servo_set.command({
                    1: 'd pos {} 0 15\n'.format(FEMUR_START).encode('utf8'),
                    2: 'd pos {} 0 15\n'.format(TIBIA_START).encode('utf8'),
                    3: b'd pos -0.005 0 40\n',
                })

            state = State.WAIT_FOR_INIT

        elif state == State.WAIT_FOR_INIT:
            # Wait for both channels to be operating and for the
            # position to be roughly there, then switch to landing.
            done = (
                data[1].mode == 9 and
                data[2].mode == 9 and
                abs(data[1].velocity) < 0.1 and
                abs(data[2].velocity) < 0.1 and
                abs(data[3].velocity) < 0.1
            )
            if done:
                state = State.LANDING

        elif state == State.LANDING:
            # Wait until the velocity is either zero, or the leg is
            # rising up.

            stopped = (
                data[1].velocity < 0.1 and
                data[2].velocity > -0.1)

            if stopped:
                # Start the jump.
                if args.really_run:
                    await servo_set.command({
                        1: b'd pos nan -9 30\n',
                        2: b'd pos nan 9 25\n',
                    })
                state = State.JUMPING
        elif state == State.FALLING:
            # Wait for contact, by watching for current to be non-zero.
            in_contact = (
                abs(data[1].q_A) > 10 or
                abs(data[2].q_A) > 10)
            if in_contact:
                # Nothing to do now, except wait for us to bottom out.
                state = State.LANDING

        elif state == State.RETRACTING:
            # Wait for servos to be roughly in position and the
            # current to be mostly stabilized.
            in_position = (
                abs(data[1].unwrapped_position - FEMUR_START) < 0.01 and
                abs(data[2].unwrapped_position - TIBIA_START) < 0.01 and
                abs(data[1].q_A) < 5 and
                abs(data[2].q_A) < 5)

            if in_position:
                # We don't have to do anything here but wait for the
                # landing event.
                state = State.FALLING

        elif state == State.JUMPING:
            # Wait for servos to be vertical.
            vertical = (
                abs(data[1].unwrapped_position) < 0.01 and
                abs(data[2].unwrapped_position) < 0.01)
            if vertical:
                # Start our retract.
                if args.really_run:
                    await servo_set.command({
                        1: 'd pos {} 0 25\n'.format(FEMUR_START).encode('latin1'),
                        2: 'd pos {} 0 25\n'.format(TIBIA_START).encode('latin1'),
                    })
                state = State.RETRACTING


if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(main())
