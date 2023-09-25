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

"""Accept a file which is the output of a moteus encoder calibration
run.  Display useful graphs, and emit valid calibration parameters."""

import argparse
import math
import matplotlib.pyplot as pyplot
import numpy
import sys


def wrap_negpi_pi(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def read_line(fd):
    line = fd.readline().strip()

    return line


def wrap_int16(value):
    if value > 32767:
        return value - 65536
    if value < -32768:
        return 65536 + value
    return value


def read_file(fd):
    start = read_line(fd)
    if not start.startswith('CAL start'):
        raise RuntimeError('Does not start with magic line:' + start)

    phase_up = []
    phase_down = []

    data = []
    while True:
        line = read_line(fd)
        if line == 'CAL done':
            break

        fields = [int(x) for x in line.strip().split(' ')[0:3]]
        item = fields[1:]
        if fields[0] == 1:
            phase_up.append(item)
        elif fields[0] == 2:
            phase_down.append(item)

    return {
        'phase_up': phase_up,
        'phase_down': phase_down,
    }


_DEFAULT_CALIBRATION = {
    'invert': False,
    'poles': 0,
    'offset': [],
    'error': None,
}


def _make_error(message):
    print("ERROR:", message)
    result = {
        "error": message,
    }
    result.update(_DEFAULT_CALIBRATION)
    return result


def perform_calibration(data, show_plots=False):
    # Things we need to figure out from this data:
    #  * whether it is inverted or not
    #  * number of poles
    #  * phase offset as a function of encoder position
    phase_up = data['phase_up']
    phase_down = data['phase_down']

    total_delta = 0
    for first_pair, second_pair in zip(phase_up[:-1], phase_up[1:]):
        total_delta += wrap_int16(second_pair[1] - first_pair[1])

    # The magnitude of our delta should be approximately a full 16
    # bits.

    if abs(abs(total_delta) - 65536) > 5000:
        return _make_error("phase_up did not traverse sufficient encoder distance")

    result = {'invert': False}

    # Figure out inversion.

    if total_delta < 0:
        result['invert'] = True
        phase_up = [(a, 65535-b) for a, b in phase_up]
        phase_down = [(a, 65535-b) for a, b in phase_down]
        total_delta *= -1


    # Next, figure out number of poles.  We compare the total encoder
    # delta, to the total phase delta.
    total_phase = 0
    for first_pair, second_pair in zip(phase_up[:-1], phase_up[1:]):
        total_phase += wrap_int16(second_pair[0] - first_pair[0])

    # These should be close to an integral multiple, or we have a
    # problem.
    ratio = total_phase / total_delta
    remainder = abs(round(ratio) - ratio)

    if abs(remainder) > 0.10:
        return _make_error("encoder not an integral multiple of phase, {} > {}"
                           .format(remainder, 0.10))

    print("total_phase:", total_phase)
    print("total_delta:", total_delta)
    print("ratio:", ratio)
    result['poles'] = int(round(ratio) * 2)
    print("poles:", result['poles'])

    # Now we need to figure out the phase offset at select points.  We
    # interpolate and average the phase up and phase down sections.
    phase_up_inverted = list(sorted([(b, a) for a, b in phase_up]))
    phase_down_inverted = list(sorted([(b, a) for a, b in reversed(phase_down)]))

    offset = phase_down_inverted[0][1] - phase_up_inverted[0][1]
    if abs(offset) > 32767:
        # We need to shift it so that they start from the same place.
        change = -65536 * round(offset / 65536)
        phase_down_inverted = [(a, b + change) for a, b in phase_down_inverted]

    interp_x = numpy.linspace(0, 65535, 10000)
    phase_up_encoder = numpy.array([x[0] for x in phase_up_inverted])
    phase_up_phase = numpy.unwrap((2.0 * numpy.pi / 65536.0) *
                                  numpy.array([x[1] for x in phase_up_inverted]))

    phase_down_encoder = numpy.array([x[0] for x in phase_down_inverted])
    phase_down_phase = numpy.unwrap((2.0 * numpy.pi / 65536.0) *
                                    numpy.array([x[1] for x in phase_down_inverted]))


    pu_interp = numpy.interp(interp_x, phase_up_encoder, phase_up_phase)
    pd_interp = numpy.interp(interp_x, phase_down_encoder, phase_down_phase)
    avg_interp = 0.5 * (pu_interp + pd_interp)

    expected = (2.0 * math.pi / 65536.0) * (result['poles'] / 2) * interp_x
    err = [wrap_negpi_pi(x) for x in avg_interp - expected]
    # Make the error seem reasonable, so unwrap if we happen to span
    # the pi boundary.
    if (max(err) - min(err)) > 1.5 * math.pi:
        err = [x if x > 0 else x + 2 * math.pi for x in err]

    def windowed_avg(data, index, size):
        start = max(0, int(index - size // 2))
        end = min(len(data) - 1, int(index + size // 2))
        subdata = data[start:end]
        errs = [wrap_negpi_pi(x - subdata[0]) for x in subdata[1:]]
        return subdata[0] + sum(errs) / len(subdata)

    avg_window = int(len(err) / result['poles'])
    avg_err = [windowed_avg(err, i, avg_window) for i in range(len(err))]

    offset_x = list(range(0, 65536, 65536 // 64))
    offset_y = numpy.interp(offset_x, interp_x, avg_err)

    result['offset'] = list(offset_y)

    if show_plots:
        ax = pyplot.subplot(211)
        ax.plot(phase_up_encoder, phase_up_phase,
                    label='phase_up')
        ax.plot(phase_down_encoder, phase_down_phase,
                    label='phase_down')
        ax.plot(interp_x, expected, label='expected')
        ax.plot(interp_x, avg_interp, label='avg')
        ax.legend()

        ax = pyplot.subplot(212)

        ax.plot(interp_x, err, label='err')
        ax.plot(interp_x, avg_err, label='avg_err')
        ax.legend()

        pyplot.show()

    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--poles', type=float, default=24, help='number of poles')
    parser.add_argument('file', help='input file')
    parser.add_argument('-o', '--output', default='-',
                        help='file with output configuration')

    args = parser.parse_args()

    with open(args.file, 'r') as fd:
        data = read_file(fd)

    calibration = perform_calibration(data, show_plots=True)
    print(calibration)

    # Now print out the commands necessary to install this config.
    stream = open(args.output, 'w') if args.output != '-' else sys.stdout

    print('conf set motor.poles {}'.format(calibration['poles']), file=stream)
    print('conf set motor.invert {}'.format(1 if calibration['invert'] else 0),
          file=stream)
    for index, value in enumerate(calibration['offset']):
        print('conf set motor.offset.{} {}'.format(index, value), file=stream)

    print('conf write', file=stream)

if __name__ == '__main__':
    main()
