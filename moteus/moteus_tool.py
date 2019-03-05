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

'''Configures, calibrates, and manipulates the moteus brushless servo.'''

import argparse
import array
import asyncio
import fcntl
import io
import math
import termios
import sys


import mjlib.micro.multiplex_protocol as mp
import mjlib.micro.aioserial as aioserial

import moteus.calibrate_encoder


G_VERBOSE = False


def set_serial_low_latency(fd):
    buf = array.array('i', [0] * 32)
    fcntl.ioctl(fd, termios.TIOCGSERIAL, buf)
    buf[4] |= 0x2000
    fcntl.ioctl(fd, termios.TIOCSSERIAL, buf)


async def readline(stream):
    result = bytearray()
    while True:
        char = await stream.read(1)
        if char == b'\r' or char == b'\n':
            if len(result):
                return result
        else:
            result += char


async def readbytes(stream):
    while True:
        await stream.read(1)


async def read_response(stream):
    result = await readline(stream)
    _ = await stream.read(1)  # ignore the extra '\n' we know the servos always send
    return result


async def read_ok(stream):
    if G_VERBOSE:
        print(' -- waiting_ok')
    result = []
    while True:
        line = await read_response(stream)
        if G_VERBOSE:
            print(' -- waiting_ok:', line)
        if line.startswith(b'OK'):
            return result
        result.append(line)


async def write_command(stream, line):
    if G_VERBOSE:
        print(' -- writing:', line)
    stream.write(line + b'\n')
    await stream.drain()


async def command(stream, line):
    await write_command(stream, line)
    return await read_ok(stream)


async def find_online_targets(manager):
    result = []
    for maybe_id in range(128):
        client = mp.MultiplexClient(
            manager, timeout=0.1, destination_id=maybe_id, channel=1)
        # TODO(jpieper): Create a way to expose a poll mechanism at
        # the multiplex-protocol level.  For now, we will just use an
        # effectively no-op telemetry command which we would need to
        # do to put the device in a known state anyways.
        await write_command(client, b'tel stop\n')

        try:
            await asyncio.wait_for(read_ok(client), 0.01)
            result.append(maybe_id)
        except asyncio.TimeoutError:
            pass

    return result


async def read_data(client, channel):
    await command(client, 'tel fmt {} 1'.format(channel).encode('utf8'))
    result = await command(client, 'tel get {}'.format(channel).encode('utf8'))

    return dict([x.decode('utf8').split(' ') for x in result])

async def find_current(client, voltage):
    assert voltage < 0.6
    assert voltage >= 0.0
    # Start the current.
    await command(client, 'd pwm 0 {:.3f}'.format(voltage).encode('utf8'))

    # Wait a tiny bit for it to stabilize.
    await asyncio.sleep(0.3)

    # Now request the servo_stats telemetry channel in text in order
    # to get the D and Q currents.
    data = await read_data(client, 'servo_stats')
    print('data:', data)

    d_cur = float(data['servo_stats.d_A'])
    q_cur = float(data['servo_stats.q_A'])

    # Now stop the current.
    await command(client, b'd stop')

    # And sleep a tiny bit before returning.
    await asyncio.sleep(0.1)

    return math.sqrt(d_cur ** 2 + q_cur ** 2)


def calculate_winding_resistance(voltages, currents):
    assert len(voltages) == len(currents)
    assert voltages[-1] == max(voltages)
    assert currents[-1] == max(currents)

    # Dead simple approximation for now.
    return voltages[-1] / currents[-1]


async def do_calibrate(client, args):
    print('This will move the motor, ensure it can spin freely!')

    # We have 3 things to calibrate.
    #  1) the encoder to phase mapping
    #  2) the winding resistance
    #  3) the kV rating of the motor

    # We start with the encoder mapping.  For that to work, we first
    # want to get it locked into zero phase.
    await command(client, b'd pwm 0 0.3')
    await asyncio.sleep(3.0)

    await command(client, b'd stop')
    await asyncio.sleep(0.1)

    print('Starting calibration process')

    # Now we start the calibration process and record the results.
    await write_command(client, b'd cal 0.3\n')

    encoder_lines = []
    while True:
        line = (await readline(client)).strip().decode('utf8')
        print(line)
        encoder_lines.append(line)
        if line.startswith('CAL done'):
            break

    calibrate_data = moteus.calibrate_encoder.read_file(
        io.StringIO(''.join(['1>{}\n'.format(x) for x in encoder_lines])))
    calibration = moteus.calibrate_encoder.perform_calibration(
        calibrate_data, args.show_plots)

    if 'error' in calibration:
        print('Error calibrating:', calibration['error'])
        return

    print(calibration)

    print('Storing encoder config')

    # Now store this part of the config.
    await command(client, 'conf set motor.poles {}'.format(
        calibration['poles']).encode('utf8'))
    await command(client, 'conf set motor.invert {}'.format(
        1 if calibration['invert'] else 0).encode('utf8'))
    await command(client, 'conf set servo.pid_position.sign {}'.format(
        -1 if calibration['invert'] else 1).encode('utf8'))
    for index, value in enumerate(calibration['offset']):
        await command(client, 'conf set motor.offset.{} {}'.format(
            index, value).encode('utf8'))

    # Next we figure out the winding resistance.
    print('Calculating winding resistance')

    if True:
        voltages = [0.2, 0.3, 0.4]
        currents = [await find_current(client, x) for x in voltages]

        print('Measured voltage/current:', list(zip(voltages, currents)))

        winding_resistance = calculate_winding_resistance(voltages, currents)
        await command(client, 'conf set motor.resistance_ohm {}'.format(
            winding_resistance).encode('utf8'))

    # And now the kV rating.
    speed = 2.0
    await command(client, b'conf set motor.v_per_hz 0')
    await command(client, b'd index 0')
    await command(client, 'd pos nan {} 5'.format(speed).encode('utf8'))

    # Wait for it to stabilize.
    await asyncio.sleep(2.0)

    async def read_q_v():
        servo_control = await read_data(client, 'servo_control')
        await asyncio.sleep(0.1)
        return float(servo_control['servo_control.q_V'])

    q_Vs = [await read_q_v() for i in range(10)]
    v_per_hz = sum(q_Vs) / len(q_Vs) / speed

    await command(client, b'd stop')
    await asyncio.sleep(0.5)

    await command(client, 'conf set motor.v_per_hz {}'.format(v_per_hz).encode('utf8'))

    # Finally, write all this configuration to the device.
    await command(client, b'conf write')

    print('Calibration complete')


async def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        '-t', '--target', type=int, action='append', default=None,
        help='destination address(es) (default: autodiscover)')
    parser.add_argument(
        '-d', '--device', type=str, default='/dev/ttyUSB0',
        help='serial device')
    parser.add_argument(
        '-b', '--baud', type=int, default=3000000, help='baud rate')
    parser.add_argument('-v', '--verbose', action='store_true')

    parser.add_argument('--show-plots', action='store_true',
                        help='only valid with --calibrate')

    # The different commands that we can do.  No more than one can be
    # specified at a time.
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        '--stop', action='store_true',
        help='command the servos to stop')
    group.add_argument(
        '--dump-config', action='store_true',
        help='emit all configuration to the console')
    group.add_argument(
        '--set-id', type=int, default=None,
        help='configure the device to use the given multiplex ID')
    group.add_argument(
        '--calibrate', action='store_true',
        help='calibrate the motor, requires full freedom of motion')

    args = parser.parse_args()

    if args.verbose:
        global G_VERBOSE
        G_VERBOSE = True

    serial = aioserial.AioSerial(port=args.device, baudrate=args.baud)
    set_serial_low_latency(serial.fd)

    try:
        _ = await asyncio.wait_for(serial.read(8192), 0.1)
    except asyncio.TimeoutError:
        pass

    manager = mp.MultiplexManager(serial)

    # If we don't know which target to use, then we search to see who
    # is on the bus.
    if args.target is None:
        args.target = await find_online_targets(manager)
        if len(args.target) == 0:
            print('No devices found.')
            sys.exit(1)

        print('Auto-detected device ids: ', args.target)

    clients = { key: mp.MultiplexClient(
        manager, timeout=0.02, destination_id=key, channel=1)
                for key in args.target }

    for key, client in clients.items():
        if len(clients) > 0:
            print('*** ID: {}'.format(key))

        # Read anything that might have been sitting on the channel
        # first.
        try:
            await write_command(client, b'tel stop')
            _ = await(asyncio.wait_for(readbytes(client), 0.2))
        except asyncio.TimeoutError:
            pass

        if args.stop:
            await command(client, b'd stop')

        if args.dump_config:
            await write_command(client, b'conf enumerate')

            while True:
                line = (await readline(client)).strip()
                if line.startswith(b'OK'):
                    break
                print(line.decode('utf8'))

        if args.calibrate:
            await do_calibrate(client, args)


if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(main())
