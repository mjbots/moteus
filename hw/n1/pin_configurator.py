#!/usr/bin/python3

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
import enum
import re
import sys


class MatchStatus(enum.IntEnum):
    VALID = 1
    INCONCLUSIVE = 0
    IMPOSSIBLE = -1


def generic_search(data, to_find):
    found = [d.find(to_find) >= 0 for d in data]
    if any(found):
        return found.index(True)
    return None

def find_and_remove(data, to_find, compare=generic_search):
    hit = compare(data, to_find)
    if hit is not None:
        del(data[hit])
        return 1
    return 0


class Desired5V:
    def __init__(self, args):
        self.args = args

    def __call__(self, pins):
        for pin in pins:
            for subpin in pin.split('/'):
                if subpin.find('5V') < 0:
                    return MatchStatus.IMPOSSIBLE
        if len(pins) == self.args.pins:
            return MatchStatus.VALID
        return MatchStatus.INCONCLUSIVE


class DesiredADC:
    def __init__(self, args):
        self.args = args
        self.available_adcs = [
            x for x in ['ADC1', 'ADC2', 'ADC3', 'ADC4']
            if x not in self.args.exclude
        ]

    def all_analog(self, maybe_pair):
        pins = maybe_pair.split('/')
        return all([x.find('ANALOG') >= 0 for x in pins])

    def find_adc(self, data, to_find):
        found = [d.find(to_find) >= 0 and self.all_analog(d) for d in data]
        if any(found):
            return found.index(True)
        return None

    def __call__(self, pins):
        # This is not strictly correct, as it will count multiple
        # channels on the same pin as multiple ADCs.  We'll fix that
        # in an ad-hoc way if it matters.

        adc_present = []
        pins_copy = pins[:]
        for adc in self.available_adcs:
            adc_present.append(find_and_remove(pins_copy, adc,
                                               compare=self.find_adc))

        count = sum(adc_present)
        if count == 2:
            return MatchStatus.VALID
        elif (2 - count) + len(pins) > self.args.pins:
            return MatchStatus.IMPOSSIBLE
        else:
            return MatchStatus.INCONCLUSIVE


class DesiredUART:
    def __init__(self, args):
        self.args = args
        self.available_uarts = [
            x for x in ['USART1', 'USART2', 'USART3', 'UART4', 'UART5', 'LPUART1']
            if x not in self.args.exclude
        ]

    def __call__(self, pins):

        results = []
        for uart in self.available_uarts:
            pins_copy = pins[:]
            tx = find_and_remove(pins_copy, f'{uart}_TX')
            rx = find_and_remove(pins_copy, f'{uart}_RX')

            if tx > 0 and rx > 0:
                return MatchStatus.VALID
            elif (2 - tx - rx) + len(pins) > self.args.pins:
                results.append(MatchStatus.IMPOSSIBLE)
            else:
                results.append(MatchStatus.INCONCLUSIVE)
        return max(results)


class DesiredI2C:
    def __init__(self, args):
        self.args = args
        self.available_i2c = [
            x for x in ['I2C1', 'I2C2', 'I2C3', 'I2C4', 'I2C5']
            if x not in self.args.exclude
        ]

    def __call__(self, pins):
        results = []
        for i2c in self.available_i2c:
            pins_copy = pins[:]
            sda = find_and_remove(pins_copy, f'{i2c}_SDA')
            scl = find_and_remove(pins_copy, f'{i2c}_SCL')

            if sda > 0 and scl > 0:
                return MatchStatus.VALID
            elif (2 - sda - scl) + len(pins) > self.args.pins:
                results.append(MatchStatus.IMPOSSIBLE)
            else:
                results.append(MatchStatus.INCONCLUSIVE)

        return max(results)


class DesiredQuadrature:
    def __init__(self, args):
        self.args = args
        self.available_timers = [
            x for x in ['TIM1', 'TIM2', 'TIM3', 'TIM4', 'TIM5', 'TIM8', 'TIM20']
            if x not in self.args.exclude
        ]

    def __call__(self, pins):
        results = []

        for timer in self.available_timers:
            pins_copy = pins[:]
            ch1 = find_and_remove(pins_copy, f'{timer}_CH1')
            ch2 = find_and_remove(pins_copy, f'{timer}_CH2')

            if ch1 > 0 and ch2 > 0:
                return MatchStatus.VALID
            elif (2 - ch1 - ch1) + len(pins) > self.args.pins:
                results.append(MatchStatus.IMPOSSIBLE)
            else:
                results.append(MatchStatus.INCONCLUSIVE)
        return max(results)


class DesiredSpi:
    def __init__(self, args):
        self.args = args
        self.available_spi = [
            x for x in ['SPI1', 'SPI2', 'SPI3', 'SPI4', 'SPI5']
            if x not in self.args.exclude
        ]

    def any_non_five_volt(self, pins_copy):
        for pin_pair in pins_copy:
            for pin in pin_pair.split('/'):
                if pin.find('5V') < 0:
                    return True
        return False

    def __call__(self, pins):
        results = []

        # Just check all 5 SPI types.
        for spi in self.available_spi:
            sck = 0
            miso = 0
            mosi = 0

            pins_copy = pins[:]

            sck = find_and_remove(pins_copy, f'{spi}_SCK')
            miso = find_and_remove(pins_copy, f'{spi}_MISO')
            mosi = find_and_remove(pins_copy, f'{spi}_MOSI')

            if self.args.nonspi_five_volt:
                # All remaining pins must be 5V.
                if self.any_non_five_volt(pins_copy):
                    results.append(MatchStatus.IMPOSSIBLE)
                    continue


            if sck > 0 and miso > 0 and mosi > 0:
                return MatchStatus.VALID
            elif (3 - sck - miso - mosi) + len(pins) > self.args.pins:
                results.append(MatchStatus.IMPOSSIBLE)
            else:
                results.append(MatchStatus.INCONCLUSIVE)
        return max(results)

    def find_nonspi(self, pins):
        results = []

        # Just check all 5 SPI types.
        for spi in self.available_spi:
            sck = 0
            miso = 0
            mosi = 0

            pins_copy = pins[:]

            sck = find_and_remove(pins_copy, f'{spi}_SCK')
            miso = find_and_remove(pins_copy, f'{spi}_MISO')
            mosi = find_and_remove(pins_copy, f'{spi}_MOSI')

            if sck > 0 and miso > 0 and mosi > 0:
                return pins_copy
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pins', type=int, default=5)
    parser.add_argument('--pinout', default='pinout.txt')
    parser.add_argument('--five-volt', action='store_true')
    parser.add_argument('--verbose', '-v', action='store_true')
    parser.add_argument('--exclude', '-e', action='append', default=[])
    parser.add_argument('--nonspi-five-volt', action='store_true')
    parser.add_argument('--max-doubled', default=10, type=int)

    parser.add_argument('--action', '-a', default='search', type=str)

    parser.add_argument('--verify', '-f', type=str)

    args = parser.parse_args()

    lines = [x.strip() for x in open(args.pinout).readlines()
             if (not x.startswith('#') and
                 x.split(' ')[0] not in args.exclude)]

    if args.action == 'search':
        do_search(args, lines)
    if args.action == 'verify':
        do_verify(args, lines)

def do_verify(args, lines):
    def get_pin(pin):
        return [x for x in lines if x.startswith(pin + ' ')][0]

    pins = []
    pin_pairs = args.verify.split(',')
    for pair in pin_pairs:
        pins.append(' / '.join([get_pin(pin) for pin in pair.split('/')]))

    to_check = [
        ('SPI', DesiredSpi(args)),
        ('QUAD', DesiredQuadrature(args)),
        ('I2C', DesiredI2C(args)),
        ('UART', DesiredUART(args)),
        ('ADC', DesiredADC(args)),
    ]

    for check in to_check:
        print(f"{check[0]} - {check[1](pins)}")

    desired_spi = DesiredSpi(args)
    nonspi_desired = [
        DesiredQuadrature(args),
        DesiredI2C(args),
        DesiredUART(args),
        DesiredADC(args),
    ]

    nonspi_pins = desired_spi.find_nonspi(pins)
    bonus = [d(nonspi_pins)
             for d in nonspi_desired].count(MatchStatus.VALID) * 2

    print(f"bonus {bonus}")



def do_search(args, lines):
    desired_spi = DesiredSpi(args)

    nonspi_desired = [
        DesiredQuadrature(args),
        DesiredI2C(args),
        DesiredUART(args),
        DesiredADC(args),
    ]

    desired = [ desired_spi ] + nonspi_desired

    if args.five_volt:
        desired.append(Desired5V(args))

    current = [0]
    tries = 0

    best_score = 1000
    best_doubled = 1000

    while True:
        tries += 1
        these_pins_doubled = [lines[x] for x in current]
        short_doubled = [x.split(' ')[0] for x in these_pins_doubled]
        these_pins = []
        while len(these_pins_doubled):
            maybe_pair = these_pins_doubled[0:2]

            if len(maybe_pair) == 1 or maybe_pair[0] == maybe_pair[1]:
                these_pins.append(maybe_pair[0])
            else:
                these_pins.append(' / '.join(maybe_pair))
            del(these_pins_doubled[0:2])

        #print(f"TRYING: {these_pins}")
        these_pins_short = []
        while len(short_doubled):
            short_pair = short_doubled[0:2]
            if len(short_pair) == 2 and short_pair[0] == short_pair[1]:
                del(short_pair[1])
            these_pins_short.append('/'.join(short_pair))
            del(short_doubled[0:2])

        doubled = ','.join(these_pins_short).count('/')

        print(f"{tries}   {','.join(these_pins_short)}     ",
              end='\n' if args.verbose else '\r')

        if tries == 116803:
            a = 1

        if doubled <= args.max_doubled:
            results = [d(these_pins) for d in desired]
        else:
            results = [MatchStatus.IMPOSSIBLE]

        b = 1

        if all([x == MatchStatus.VALID for x in results]):
            pin_count = len(current)
            nonspi_pins = desired_spi.find_nonspi(these_pins)
            bonus = [d(nonspi_pins)
                     for d in nonspi_desired].count(MatchStatus.VALID) * 2
            score = pin_count + doubled * 2 - bonus
            if doubled < best_doubled:
                best_doubled = doubled
            if score <= best_score:
                best_score = score
                print()
                print()
                print(f"FOUND after {tries} tries w/ {pin_count} pins {doubled} doubled {score} score - {bonus} bonus")
                print('\n'.join(these_pins))
                print()

        if (any([x == MatchStatus.IMPOSSIBLE for x in results]) or
              len(current) >= args.pins * 2 or
              (current[-1] + 1) >= len(lines)):
            # We need to advance the current index.  If that exceeds
            # our limit, then backtrack if possible.
            while True:
                current[-1] += 1
                if len(current) >=2 and current[-1] == current[-2]:
                    continue
                if current[-1] < len(lines):
                    break
                current.pop()
                if len(current) == 0:
                    print("IMPOSSIBLE")
                    sys.exit(1)
        else:
            if len(current) % 2 == 0:
                current.append(current[-2] + 1)
            else:
                current.append(current[-1])



if __name__ == '__main__':
    main()
