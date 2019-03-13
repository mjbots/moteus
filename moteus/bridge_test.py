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

'''Engineering test tool for imu_junction bridge board'''

import argparse
import array
import fcntl
import termios
import asyncio


import mjlib.multiplex.multiplex_protocol as mp
import mjlib.multiplex.aioserial as aioserial


def set_serial_low_latency(fd):
    buf = array.array('i', [0] * 32)
    fcntl.ioctl(fd, termios.TIOCGSERIAL, buf)
    buf[4] |= 0x2000
    fcntl.ioctl(fd, termios.TIOCSSERIAL, buf)


async def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        '-d', '--device', type=str, default='/dev/ttyUSB0',
        help='serial device')
    parser.add_argument(
        '-b', '--baud', type=int, default=3000000, help='baud rate')

    args = parser.parse_args()

    serial = aioserial.AioSerial(port=args.device, baudrate=args.baud)
    set_serial_low_latency(serial.fd)
    manager = mp.MultiplexManager(serial)

    clients = {
        key: mp.MultiplexClient(
            manager, timeout=0.02, destination_id=key, channel=1)
        for key in [1, 2, 3] }

    for key, client in clients.items():
        client.write(b'tel stop\n')

    await manager.drain()

    await asyncio.sleep(2.0)


if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(main())
