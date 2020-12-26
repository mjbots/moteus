#!/usr/bin/python3 -B

# Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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
import asyncio
import sys

from . import moteus
from . import aiostream

def _expand_targets(targets):
    result = set()

    for item in targets:
        fields = item.split(',')
        for field in fields:
            if '-' in field:
                first, last = field.split('-')
                result |= set(range(int(first), int(last) + 1))
            else:
                result |= { int(field) }

    return sorted(list(result))


async def _copy_stream(inp, out):
    while True:
        data = await inp.read(4096, block=False)
        out.write(data)
        await out.drain()


class Stream:
    def __init__(self, args, target_id, transport):
        self.controller = moteus.Controller(target_id, transport=transport)
        self.stream = moteus.Stream(self.controller, verbose=args.verbose)

    async def do_console(self):
        console_stdin = aiostream.AioStream(sys.stdin.buffer.raw)
        console_stdout = aiostream.AioStream(sys.stdout.buffer.raw)
        dir1 = asyncio.create_task(_copy_stream(self.stream, console_stdout))
        dir2 = asyncio.create_task(_copy_stream(console_stdin, self.stream))
        await asyncio.wait([dir1, dir2], return_when=asyncio.FIRST_COMPLETED)

    async def command(self, message):
        await self.stream.command(message)


class Runner:
    def __init__(self, args):
        self.args = args
        self.cmdline_targets = _expand_targets(args.target)

        # Was our target list found through discovery?
        self._discovered = False

    async def start(self):
        self.transport = moteus.get_singleton_transport(self.args)
        targets = await self.find_targets()

        for target in targets:
            if self._discovered:
                print(f"Target: {target}")
            await self.run_action(target)

    async def find_targets(self):
        if self.cmdline_targets:
            return self.cmdline_targets

        result = []
        self._discovered = True

        for i in range(1, 127):
            c = moteus.Controller(id=i, transport=self.transport)
            try:
                _ = await asyncio.wait_for(c.query(), 0.01)
                result.append(i)
            except asyncio.TimeoutError:
                pass

        return result

    async def run_action(self, target_id):
        stream = Stream(self.args, target_id, self.transport)

        if self.args.console:
            await stream.do_console()
        elif self.args.stop:
            await stream.command(b'd stop')
        else:
            raise RuntimeError("No action specified")


async def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        '-t', '--target', type=str, action='append', default=[],
        help='destination address(es) (default: autodiscover)')
    parser.add_argument('-v', '--verbose', action='store_true')

    moteus.make_transport_args(parser)

    group = parser.add_mutually_exclusive_group()

    group.add_argument('-s', '--stop', action='store_true',
                       help='command the servos to stop')
    group.add_argument('-i', '--info', action='store_true',
                       help='display information from the servo')
    group.add_argument('-c', '--console', action='store_true',
                       help='create a serial console')
    group.add_argument('--dump-config', action='store_true',
                       help='emit all configuration to the console')
    group.add_argument('--write-config', metavar='FILE',
                       help='write the given configuration')
    group.add_argument('--flash', metavar='FILE',
                       help='write the given elf file to flash')

    parser.add_argument('--no-restore-config', action='store_true',
                        help='do not restore config after flash')

    group.add_argument('--calibrate', action='store_true',
                        help='calibrate the motor, requires full freedom of motion')
    parser.add_argument('--cal-no-update', action='store_true',
                        help='do not store calibration results on motor')
    parser.add_argument('--cal-power', metavar='V', type=float,
                        help='voltage to use during calibration')
    parser.add_argument('--cal-speed', metavar='HZ', type=float,
                        help='speed in electrical rps')
    parser.add_argument('--cal-voltage', metavar='V', type=float,
                        help='maximum voltage when measuring resistance')
    parser.add_argument('--cal-raw', metavar='FILE', type=str,
                        help='write raw calibration data')

    group.add_argument('--restore-cal', metavar='FILE', type=str,
                        help='restore calibration from logged data')
    group.add_argument('--zero-offset', action='store_true',
                        help='set the motor\'s position offset')

    args = parser.parse_args()

    runner = Runner(args)
    await runner.start()


if __name__ == '__main__':
    asyncio.run(main())
