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
import matplotlib.pyplot as pyplot
import sys

import calibrate_encoder as ce


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('file', type=str)

    parser.add_argument('--cal-max-remainder', metavar='F',
                        type=float, default=0.1,
                        help='maximum allowed error in calibration')
    parser.add_argument('--cal-force-optimize', action='store_true',
                        help='require nonlinear commutation optimization')
    parser.add_argument('--cal-disable-optimize', action='store_true',
                        help='prevent nonlinear commutation optimization')


    args = parser.parse_args()

    f = ce.parse_file(open(args.file, "rb"))
    r = ce.calibrate(f,
                     max_remainder_error=args.cal_max_remainder,
                     allow_optimize=not args.cal_disable_optimize,
                     force_optimize=args.cal_force_optimize)

    print(r)

    ax = pyplot.subplot(211)
    ax.plot(r.debug['phase_up_encoder'], r.debug['phase_up_phase'],
            label='phase_up')
    ax.plot(r.debug['phase_down_encoder'], r.debug['phase_down_phase'],
            label='phase_down')
    ax.plot(r.debug['xpos'], r.debug['expected'], label='expected')
    ax.plot(r.debug['xpos'], r.debug['avg_interp'], label='avg')
    ax.legend()

    ax = pyplot.subplot(212)

    ax.plot(r.debug['xpos'], r.debug['sample_errors'], label='fit error',
             color='magenta')
    ax.legend(loc='upper right')

    ax2 = ax.twinx()
    ax2.plot(r.debug['xpos'], r.debug['err'], label='err')
    ax2.plot(r.debug['xpos'], r.debug['avg_err'], label='avg_err')
    ax2.plot(r.debug['offset_x'], r.debug['offset'], label='offset')
    ax2.legend(loc='upper left')

    pyplot.show()


if __name__ == '__main__':
    main()
