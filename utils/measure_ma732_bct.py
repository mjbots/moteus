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

'''This tool can be used to select an optimal BCT and trim axis for
MA600/MA732 off axis encoders.

Before running, the MA600/MA732 must be enabled in the corresponding
aux port, and at least one motor_position source must be configured to
use it as an input.

'''

import argparse
import asyncio
import math
import matplotlib.pyplot as plt
import moteus
import numpy

import histogram


class Searcher:
    def __init__(self):
        self.low = -248
        self.high = 248
        self.count = 16

        self.start_range()

    def start_range(self):
        step = max(2, (self.high - self.low) // self.count)
        self.search_start = list(range(self.low, self.high + 1, step))
        self.results = []

    def __call__(self, x_val=None, metric=None):
        if x_val is not None:
            self.results.append((x_val, metric))

        if len(self.search_start):
            to_return = self.search_start[0]
            del self.search_start[0]
            return to_return, None

        # We need to figure out our best result in the result set,
        # and pick our low and high accordingly.
        best_pair = min(sorted([(b, a) for a, b in self.results]))
        best_value = best_pair[1]
        best_metric = best_pair[0]

        index = [(i, pair) for i, pair in enumerate(self.results)
                 if pair[0] == best_value][0][0]
        before = max(0, index - 1)
        after = min(len(self.results) - 1, index + 1)

        self.low = self.results[before][0]
        self.high = self.results[after][0]

        if self.high - self.low < 8:
            return None, (best_value, best_metric)

        self.start_range()
        return self()



class BctDetector:
    def __init__(self, args):
        self.args = args

    async def command(self, message):
        if type(message) == str:
            message = message.encode('utf8')

        return await self.stream.command(message)

    async def run(self):
        m = moteus.Controller(id=self.args.target)
        self.stream = moteus.Stream(m, verbose=self.args.verbose)

        await self.stream.write_message(b"tel stop")
        await self.stream.flush_read()

        await self.command("d stop")


        self.auxstr = f"aux{self.args.aux}"

        start_bct = await histogram.read_config_double(
            self.stream, f"{self.auxstr}.spi.bct")
        start_trim = await histogram.read_config_double(
            self.stream, f"{self.auxstr}.spi.trim")
        num_poles = await histogram.read_config_double(
            self.stream, f"motor.poles")
        position_min = await histogram.read_config_double(
            self.stream, f"servopos.position_min")
        position_max = await histogram.read_config_double(
            self.stream, f"servopos.position_max")

        self.espeed = self.args.speed
        self.eaccel = self.args.acceleration

        print(f"start_bct={start_bct} start_trim={start_trim}")

        # Figure out what voltage we are going to use by searching for our
        # desired power.
        voltage = await self.find_open_loop_voltage()

        self.source = await self.find_source()

        await self.command(f"conf set {self.auxstr}.spi.bct 0")
        await self.command(f"conf set {self.auxstr}.spi.trim 0")

        await self.command(f"conf set motor.poles 2")
        await self.command(f"conf set servopos.position_min nan")
        await self.command(f"conf set servopos.position_max nan")

        try:
            # Now we will hunt through BCT in X and Y to see which gives the
            # best value.
            best_bct, best_metric = await self.find_best_bct(voltage)

            await self.command("d stop")

            print(f"Selecting kTrim{'X' if best_bct > 0 else 'Y'} bct={abs(best_bct)}")
            await self.command(f"conf set {self.auxstr}.spi.trim {1 if best_bct > 0 else 2}")
            await self.command(f"conf set {self.auxstr}.spi.bct {best_bct}")

        finally:
            # Try to stop and clean up.
            await self.command("d stop")

            # Restore our motor pole count.
            await self.command(f"conf set motor.poles {num_poles}")

            await self.command(f"conf set servopos.position_min {position_min}")
            await self.command(f"conf set servopos.position_max {position_max}")

        # We'll get here if we had no exceptions.
        await self.command("conf write")

    async def find_current(self, voltage):

        await self.command(f"d pwm 0 {voltage:.3f}")

        await asyncio.sleep(0.15)

        def extract(f):
            return math.hypot(f.d_A, f.q_A)

        data = [extract(await self.stream.read_data("servo_stats"))
                for _ in range(10)]

        await self.command("d stop")

        await asyncio.sleep(0.05)
        return sum(data) / len(data)

    async def find_open_loop_voltage(self):
        if self.args.force_voltage is not None:
            return self.args.force_voltage

        voltage = 0.01

        while True:
            print(f"Testing {voltage:.3f}V", end='\r', flush=True)
            this_current = await self.find_current(voltage)
            power = this_current * voltage
            if power > self.args.power:
                print()
                return voltage

            voltage *= 1.1


    async def get_config(self, name):
        raw_config = await self.command(f"conf enumerate {name}")
        config = dict([line.split(' ')
                       for line in raw_config.decode('utf8').split('\n')
                       if ' ' in line])

        return config

    async def find_source(self):
        # Look through all sources to see which references our desired
        # AUX port with SPI.

        auxconfig = await self.get_config(self.auxstr)
        if not int(auxconfig[f'{self.auxstr}.spi.mode']) in [4, 5]:
            raise RuntimeError(f'{self.auxstr} not configured for MA600/MA732')

        config = await self.get_config("motor_position")

        maybe_result = -1
        while True:
            maybe_result += 1


            auxstr = f'motor_position.sources.{maybe_result}.aux_number'
            if auxstr not in config:
                break

            if int(config[auxstr]) != self.args.aux:
                continue

            typestr = f'motor_position.sources.{maybe_result}.type'
            if int(config[typestr]) != 1:  # SPI
                continue

            cprstr = f'motor_position.sources.{maybe_result}.cpr'
            if int(config[cprstr]) != 65536:
                raise RuntimeError('MA732 is not configured with CPR=65536')

            return maybe_result

        raise RuntimeError(f'No motor_position source found which uses MA732 on {self.auxstr}')


    async def find_best_bct(self, voltage):
        searcher = Searcher()

        to_try, _ = searcher()

        while True:
            trim_axis = 1 if to_try > 0 else 2

            await self.command(f"conf set {self.auxstr}.spi.trim {trim_axis}")
            await self.command(f"conf set {self.auxstr}.spi.bct {abs(to_try)}")

            await self.command(f"d pos nan {self.espeed} nan o{voltage} a{self.eaccel}")
            await asyncio.sleep(1.0)

            values, splits = await histogram.capture_histogram(
                stream=self.stream,
                hist_options=[
                    f"xo{self.source}",
                    f"yo{self.source}d",
                    ],
                split_count=self.args.split_count,
                sample_time=2)

            await self.command("d stop")

            await asyncio.sleep(0.5)

            finite_values = [x for x in values if math.isfinite(x)]

            if len(finite_values) < 0.3 * len(values):
                to_try, _ = searcher()
            else:
                metric = numpy.std(finite_values)

                print(f"tried trim={trim_axis} bct={abs(to_try)} metric={metric}")
                to_try, best = searcher(to_try, metric)

                if best is not None:
                    return best


async def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--target', '-t', type=int, default=1)
    parser.add_argument('--aux', '-a', type=int, default=2)
    parser.add_argument('--speed', '-s', type=float, default=10.0)
    parser.add_argument('--acceleration', type=float, default=20.0)
    parser.add_argument('--power', '-p', type=float, default=3.0)
    parser.add_argument('--force-voltage', type=float, default=None)
    parser.add_argument('--split-count', type=int, default=2)

    parser.add_argument('--verbose', action='store_true')

    args = parser.parse_args()

    detector = BctDetector(args)
    await detector.run()



if __name__ == '__main__':
    asyncio.run(main())
