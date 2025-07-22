#!/usr/bin/python3 -B

# Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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
import math
import subprocess
import sys


# From ChatGPT o3 and https://www.dsprelated.com/showarticle/973.php
def rms_gain_robertson(f_test, f_c, zeta):
    """
    Continuous-time magnitude |H_CL(jω)| for the closed-loop PLL response
    described in Neil Robertson’s article (DSPRelated #973).

    Parameters
    ----------
    f_test : float   – frequency of the test sine wave  [Hz]
    f_c    : float   – 3-dB (-3 dB) cut-off frequency of the loop [Hz]
    zeta   : float   – damping coefficient ζ

    Returns
    -------
    gain : float     – linear gain; multiply by input-RMS to predict output-RMS
    """

    # ---- 1.  Find r = (ω_c / ω_n) from |H_CL(jω_c)|² = 1/2 ------------------
    # This gives the quadratic      x² − (4 ζ² + 2) x − 1 = 0 ,
    # with x = r²  and the positive root keeps x ≥ 0.
    B = 4.0 * zeta * zeta + 2.0
    x = 0.5 * (B + math.sqrt(B * B + 4.0))    # positive root
    r = math.sqrt(x)                          # r = ω_c / ω_n

    # ---- 2.  Evaluate |H_CL(jω)| at the desired test frequency --------------
    Omega = f_test / f_c          # normalised test frequency  ω / ω_c
    q      = r * Omega            # = ω / ω_n

    num = 1.0 + (2.0 * zeta * q)**2                # |numerator|²
    den = (1.0 - q*q)**2 + (2.0 * zeta * q)**2     # |denominator|²
    return math.sqrt(num / den)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--binary', '-b')
    parser.add_argument('--plot', '-p', action='store_true')
    parser.add_argument('--save', type=str, default=None)

    args = parser.parse_args()

    fail = False

    for hall in [False, True]:
        for filter_3db_hz in [10, 50, 100, 200, 400]:
            result = subprocess.check_output(
                [args.binary, '--filter_hz', str(filter_3db_hz),
                 '--frequency_min_hz', '0.2',
                 '--frequency_max_hz', '1000.0' if not hall else '400.0',
                 '--frequency_step', '1.05'] +
                (['--hall', '1'] if hall else [])).decode('latin1')
            lines = result.split('\n')
            data = [tuple(float(x) for x in line.strip().split(' '))
                    for line in lines
                    if line.strip() != '']

            if args.save:
                fname = f'{args.save}/encoder_bw_{"hall" if hall else "abs"}_{filter_3db_hz}Hz.csv'
                with open(fname, 'w') as outf:
                    outf.write('FrequencyHz,code,truth\n')
                    for f, c in data:
                        t = rms_gain_robertson(f, filter_3db_hz, 1.0)
                        outf.write(f'{f},{c},{t}\n')

            if args.plot:
                import matplotlib.pyplot as plt

                plt.loglog()
                plt.grid(which='both')
                plt.plot([x[0] for x in data], [x[1] for x in data],
                         label=f'code {filter_3db_hz}Hz')
                plt.plot([x[0] for x in data], [rms_gain_robertson(x[0], filter_3db_hz, 1.0) for x in data],
                         label=f'ground truth {filter_3db_hz}Hz')
                plt.xlabel('Frequency (Hz)')
                plt.ylabel('Gain')
                plt.legend()
                plt.show()

            if not args.plot and not args.save:
                for freq, gain in data:
                    expected_gain = rms_gain_robertson(freq, filter_3db_hz, 1.0)
                    TOLERANCE = (
                        0.05 if not hall else
                        0.12 if filter_3db_hz <= 248
                        else 0.13)
                    if abs(expected_gain / gain - 1.0) > TOLERANCE:
                        print(f'Gain mismatch: filt={filter_3db_hz}Hz hall={hall} {freq}Hz {expected_gain} != {gain} (TOL={TOLERANCE})')
                        fail = True

    if fail:
        raise RuntimeError('at least one failure')


if __name__ == '__main__':
    main()
