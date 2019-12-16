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
import csv
import datetime
import enum
import io
import struct
import time


import mjlib.multiplex.stream_helpers as sh
import mjlib.multiplex.multiplex_protocol as mp
import mjlib.multiplex.aioserial as aioserial

_STREAM_CLIENT_TO_SERVER = 0x40
_STREAM_SERVER_TO_CLIENT = 0x41
G_VERBOSE = False


SERVO_STATS = struct.Struct('<iiHHHHHHHfffffHffffifff')
SERVO_STATS_NC = collections.namedtuple('servo_stats', '''mode errc
     adc1_raw adc2_raw adc3_raw
     position_raw fet_temp_raw
     adc1_offset adc2_offset
     cur1_A cur2_A busV filt_bus_V filt_1ms_bus_V
     position
     fet_temp_C
     electrical_theta
     d_A q_A
     unwrapped_position_raw
     unwrapped_position
     velocity
     torque_Nm''')

RETRACT_POS = -.1
THROW_POS = -0.223
RETRACT_TORQUE = 4
THROW_TORQUE = 30.0

def hexify(data):
    return ''.join(['{:02x}'.format(x) for x in data])


def dehexify(data):
    result = b''
    for i in range(0, len(data), 2):
        result += bytes([int(data[i:i+2], 16)])
    return result


def _pack_frame(source, dest, payload):
    return 'can send {:x} {}\n'.format(
        (source << 8) | dest,
        hexify(payload)).encode('latin1')


async def read_ok(stream, context = 'DEV'):
    if G_VERBOSE:
        print(' -- waiting_ok({}):start'.format(context))
    result = []
    while True:
        line = await read_response(stream, context)
        if G_VERBOSE:
            print(' -- waiting_ok({}) line: {}'.format(context, line))
        if line.startswith(b'OK'):
            if G_VERBOSE:
                print(' -- waiting_ok({}) DONE'.format(context))
            return result
        result.append(line)


async def readline(stream):
    result = bytearray()
    while True:
        char = await stream.read(1)
        if char == b'\r' or char == b'\n':
            if len(result):
                return result
        else:
            result += char


async def read_response(stream, _=None):
    result = await readline(stream)
    _ = await stream.read(1)  # ignore the extra '\n' we know the servos always send
    return result


class FdcanusbManager:
    def __init__(self, stream, source_id=0):
        self.stream = stream
        self.source_id = source_id
        self.lock = asyncio.Lock()
        self._write_data = bytearray()

    async def write(self, data):
        if G_VERBOSE:
            print(' -- CAN write:', data)
        self.stream.write(data)

        await self.stream.drain()

        # Wait for the OK
        await read_ok(self.stream, 'CAN')

    async def read_frame(self, only_from=None):
        line = await readline(self.stream)
        if G_VERBOSE:
            print(' -- CAN1 read:', line)

        if not line.startswith(b'rcv'):
            return None

        fields = line.decode('latin1').strip().split(' ')
        address = int(fields[1], 16)
        dest = address & 0xff
        source = (address >> 8) & 0xff

        if only_from and source != only_from:
            return None

        payload = dehexify(fields[2])

        return payload


class FdcanusbClient:
    def __init__(self, manager, destination_id,
                 channel=1,
                 poll_rate_s=0.01,
                 timeout=0.05):
        self._manager = manager
        self._destination_id = destination_id
        self._channel = channel
        self._poll_rate_s = poll_rate_s
        self._timeout = timeout
        self._read_data = bytearray()

    async def write(self, data, **kwargs):
        payload = struct.pack(
            '<BBB',
            _STREAM_CLIENT_TO_SERVER,
            self._channel,
            len(data)) + data

        await self._manager.write('can send {:x} {}\n'.format(
            self._manager.source_id << 8 | self._destination_id,
            hexify(payload)).encode('latin1'))

    async def read(self, size):
        # Poll repeatedly until we have enough.
        while len(self._read_data) < size:
            async with self._manager.lock:
                result = await self._try_one_poll()
                if result is not None:
                    self._read_data += result

            if len(self._read_data) >= size:
                break

            # We didn't get anything, so wait our polling period and
            # try again.
            await asyncio.sleep(self._poll_rate_s)

        to_return, self._read_data = (
            self._read_data[0:size], self._read_data[size:])
        return to_return

    def _make_stream_client_to_server(self, response, data):
        '''Returns a tuple of (target_message, remaining_data)'''
        write_size = min(len(data), 100)
        payload = struct.pack(
            '<BBB',
            _STREAM_CLIENT_TO_SERVER,
            self._channel,
            write_size) + data[0:write_size]

        # So that we don't need a varuint.
        assert len(payload) < 127

        return (_pack_frame(
            ((0x80 if response else 0x00) | self._manager.source_id),
            self._destination_id,
            payload),
                data[write_size:])

    async def _try_one_poll(self):
        assert self._manager.lock.locked()

        frame, _ = self._make_stream_client_to_server(True, b'')
        await self._manager.write(frame)

        # Now we can look for a reply from the device.
        result = b''
        timeout = self._timeout

        # We loop multiple times in case a previous poll timed out,
        # and replies are in-flight.
        while True:
            try:
                payload = await asyncio.wait_for(
                    self._manager.read_frame(only_from=self._destination_id),
                    timeout=timeout)
                if payload is None:
                    break
            except asyncio.TimeoutError:
                # We treat a timeout, for now, the same as if the client
                # came back with no data at all.
                break

            payload_stream = sh.AsyncStream(io.BytesIO(payload))
            subframe_id = await mp.read_varuint(payload_stream)
            channel = await mp.read_varuint(payload_stream)
            server_len = await mp.read_varuint(payload_stream)

            if subframe_id is None or channel is None or server_len is None:
                break

            if subframe_id != _STREAM_SERVER_TO_CLIENT:
                break

            cur_pos = payload_stream.tell()
            result += payload[cur_pos:cur_pos+server_len]

            # On subsequent tries through this, we barely want to wait
            # at all, we're just looking to see if something is
            # already there.
            timeout = 0.0001

        return result

    async def register_query(self, request):
        '''request should be a RegisterRequest object
        returns a dict from ParseRegisterReply
        '''
        async with self._manager.lock:
            await self._manager.write(_pack_frame(
                self._manager.source_id | 0x80,
                self._destination_id,
                request.data.getbuffer()))

            return await ParseRegisterReply(await self._manager.read_frame())

    async def register_write(self, request):
        async with self._manager.lock:
            await self._manager.write(_pack_frame(
                self._manager.source_id,
                self._destination_id,
                request.data.getbuffer()))


class ServoSet:
    def __init__(self, aio_serial, ids, fdcanusb):
        self.aio_serial = aio_serial

        manager = FdcanusbManager if fdcanusb else mp.MultiplexManager
        self.manager = manager(self.aio_serial)

        client = FdcanusbClient if fdcanusb else mp.MultiplexClient

        self.channels = {
            key: client(
                self.manager,
                timeout=0.02,
                destination_id=key,
                channel=1)
            for key in ids
        }

    async def get_servo_stats(self, id):
        channel = self.channels[id]

        await channel.write(b"tel get servo_stats\n")

        while True:
            line = (await read_response(channel)).strip()
            if line.startswith(b"emit servo_stats"):
                break

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
                await self.channels[key].write(commands[key])

            still_left = remaining[:]
            for key in still_left:
                try:
                    this_result = await asyncio.wait_for(
                        read_response(self.channels[key]), 0.05)
                    if not this_result.startswith(b"OK"):
                        # We must have gotten a response for some
                        # previous message.  Just try again.
                        continue
                    results[key] = this_result
                    remaining.remove(key)
                except asyncio.TimeoutError:
                    # We won't have removed this from 'remaining', so
                    # we'll keep trying.
                    pass

        return results


class State(enum.Enum):
    RETRACTING = 1
    THROWING = 2
    FAULT = 3

async def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('-d', '--device', type=str, default='/dev/ttyUSB0',
                        help='serial device')
    parser.add_argument('-b', '--baud', type=int, default=3000000,
                        help='baud rate')
    parser.add_argument('-c', '--fdcanusb', action='store_true')
    parser.add_argument('-o', '--output', default='/tmp')
    parser.add_argument('--really-run', action='store_true')

    args = parser.parse_args()

    output_filename = '{}/jump-{}.csv'.format(
        args.output, datetime.datetime.now().isoformat())
    fields = [
        'time',
        'state',
        '1_mode',
        '1_position',
        '1_velocity',
        '1_current',
        '1_temp_C',
        '2_mode',
        '2_position',
        '2_velocity',
        '2_current',
        '2_temp_C',
        '3_mode',
        '3_position',
        '3_velocity',
        '3_current',
        '3_temp_C',
    ]

    output = csv.DictWriter(open(output_filename, 'w'), fields)
    output.writeheader()

    serial = aioserial.AioSerial(port=args.device, baudrate=args.baud)
    # Empty out anything still on the bus.


    try:
        _ = await asyncio.wait_for(serial.read(8192), 0.1)
    except asyncio.TimeoutError:
        pass

    IDS = [1]
    servo_set = ServoSet(serial, IDS, fdcanusb=args.fdcanusb)

    state_start_s = time.time()
    state = State.RETRACTING
    old_state = state

    while True:
        if state != old_state:
            state_start_s = time.time()
            old_state = state
        since_state_s = time.time() - state_start_s

        data = {}
        for key in IDS:
            while True:
                try:
                    data[key] = await asyncio.wait_for(
                        servo_set.get_servo_stats(key), 0.15)
                    break
                except asyncio.TimeoutError:
                    # We retry infinitely.
                    pass

        now = time.time()
        print('{:.6f} {}'.format(now, state))
        for id in [1]:
            print(' {}: {:3d} p {:7.4f}  v {:7.3f}  t {:5.1f}  t {:2.0f}'.format(
                id, data[id].mode,
                data[id].unwrapped_position, data[id].velocity,
                data[id].torque_Nm, data[id].fet_temp_C))
        print()

        csv_data = {
            'time': now,
            'state': str(state),
            '1_mode': data[1].mode,
            '1_position': data[1].position,
            '1_velocity': data[1].velocity,
            '1_current': data[1].q_A,
            '1_temp_C': data[1].fet_temp_C,
        }
        output.writerow(csv_data)

        for key, value in data.items():
            if value.mode == 1:
                # We have a fault on one servo.  Stop the state machine.
                state = State.FAULT
                break

        if state == State.RETRACTING:
            if args.really_run:
                await servo_set.command({
                    1: 'd pos nan 0.3 {} s{}\n'.format(RETRACT_TORQUE, RETRACT_POS).encode('utf8'),
                    })

            # Wait until enough time has passed.
            if since_state_s > 0.6:
                state = State.THROWING

        elif state == State.THROWING:
            if args.really_run:
                await servo_set.command({
                    1: 'd pos nan 5.0 {} s{}\n'.format(THROW_TORQUE, THROW_POS).encode('utf8'),
                    })
            if since_state_s > 2.0:
                state = State.RETRACTING

        elif state == State.FAULT:
            # Nothing to do here, we stay here indefinitely.
            pass



if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(main())
