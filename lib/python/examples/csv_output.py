#!/usr/bin/python3 -B

# Copyright 2024 mjbots Robotic Systems, LLC.  info@mjbots.com
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


"""
This example shows how to write a CSV output file while executing
trajectory motions.
"""

import asyncio
import math
import moteus
import time


class CsvController(moteus.Controller):
    """Operates a controller, while writing data to a CSV file.

    Every operation which results in a query, will emit a single line
    to the CSV file.  The set of values to be written will be those
    which are configured in the `query_resolution` kwarg parameter.

    This is a context manager, and can be used with `with` statements.

    """

    def __init__(self,
                 filename=None,
                 *args,
                 relative_time=True,
                 **kwargs):
        super(CsvController, self).__init__(*args, **kwargs)

        self.fields = [field for field, resolution in
                       moteus.QueryParser.parse(self.make_query().data)]

        self.fd = open(filename, "w")
        self.relative_time = relative_time
        self._start_time = time.time()

        def format_reg(x):
            try:
                return moteus.Register(x).name
            except TypeError:
                return f'0x{x:03x}'

        print(",".join(["time"] + list(map(format_reg, self.fields))),
              file=self.fd)

    # Override the base `execute` method, but if a result is returned,
    # emit a line to the CSV file.
    async def execute(self, command):
        result = await super(CsvController, self).execute(command)

        if result is not None:
            now = time.time()
            output_time = now - self._start_time if self.relative_time else now
            print(",".join(list(map(str,
                                    [output_time] +
                                    [result.values.get(x, 0)
                                     for x in self.fields]))),
                  file=self.fd)

        return result

    def __enter__(self):
        self.fd.__enter__()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return self.fd.__exit__(exc_type, exc_value, traceback)


async def wait_stopped(controller, time_s, period_s=0.025):
    '''Hold the controller at the current position for a given amount of
    time.'''

    start = time.time()
    while True:
        now = time.time()
        if (now - start) > time_s:
            return

        await controller.set_position(position=math.nan, velocity=0.0)
        await asyncio.sleep(period_s)


async def main():
    with CsvController("output.csv") as c:
        await c.set_stop()

        while True:
            await c.set_position_wait_complete(position=30.0, accel_limit=1.0)
            await wait_stopped(c, 1.0)

            await c.set_position_wait_complete(position=0.0, accel_limit=1.0)
            await wait_stopped(c, 1.0)


if __name__ == '__main__':
    asyncio.run(main())
