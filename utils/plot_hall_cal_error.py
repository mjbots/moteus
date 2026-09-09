#!/usr/bin/python3 -B

# Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

"""Plot the commutation angle error resulting from hall calibration.

Simulates a motor whose hall transitions are equally spaced (every 60
electrical deg) but displaced from the electrical phase by a uniform "hall
offset", then runs both the pre-2026 moteus_tool hall calibration and
the current one against it.  The y axis is the electrical angle the
firmware believes the rotor is at minus where it actually is, evaluated
at every hall transition of every pole pair.

The old approach only looked at which hall state was active at
commanded phase 0 and mapped that sector to count 0, so the firmware
placed the sector's lower boundary at 0 electrical deg regardless of where it
really was.  The current approach sweeps forward and backward in 4
electrical deg steps and averages every observed transition, so the residual
is bounded by half a step.

The new-approach curve is produced by the actual code in
moteus.calibrate_encoder; the old approach is small enough to be
reproduced inline.
"""

import argparse
import math
import os
import sys

import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, '..', 'lib', 'python'))

import moteus.calibrate_encoder as ce  # noqa: E402


# Inverse of the firmware's kHallMapping: sector count -> raw bits.
COUNT_TO_BITS = {0: 1, 1: 3, 2: 2, 3: 6, 4: 4, 5: 5}

STEPS_PER_CYCLE_NEW = 90
OLD_STEPS = 24

# Span of hall offsets to plot, electrical deg.
OFFSET_RANGE_DEG = 120.0

# With uniformly spaced halls the result is independent of pole count:
# every electrical cycle observes the same transitions and the
# motor.offset[] table comes out constant, so the firmware's
# interpolation is exact regardless of how many hall counts each
# table bin spans.  Any small value will do.
POLES = 14


def wrap_deg(x):
    return (x + 180.0) % 360.0 - 180.0


class SimulatedHall:
    """Ideal 120 deg halls whose transition into sector k, when the
    rotor moves in the positive direction, occurs at electrical angle
    (offset_deg + 60 * k + hysteresis_deg).  Moving in the negative
    direction the same transition is observed at
    (offset_deg + 60 * k - hysteresis_deg).  The hysteresis term
    stands in for both hall switching hysteresis and the rotor's
    settling lag behind the commanded phase, both of which are
    anti-symmetric in direction of travel."""

    def __init__(self, offset_deg, hysteresis_deg=0.0):
        self.offset_deg = offset_deg
        self.hysteresis_deg = hysteresis_deg

    def true_boundary(self, k):
        return self.offset_deg + 60.0 * k

    def count(self, theta_deg, direction):
        best = None
        for k in range(6):
            boundary = self.true_boundary(k) + direction * self.hysteresis_deg
            dist = (theta_deg - boundary) % 360.0
            if best is None or dist < best[0]:
                best = (dist, k)
        return best[1]

    def raw(self, theta_deg, direction):
        return COUNT_TO_BITS[self.count(theta_deg, direction)]


def sweep(hall, steps_per_cycle, cycles, direction):
    """Mirror moteus_tool's hall_sweep: (phase_rad, raw_bits) samples,
    unwrapped phase, in the order they would be recorded."""
    n = steps_per_cycle * cycles
    step_range = range(n + 1) if direction > 0 else range(n, -1, -1)
    result = []
    for i in step_range:
        phase_deg = i / steps_per_cycle * 360.0
        result.append((math.radians(phase_deg),
                       hall.raw(phase_deg % 360.0, direction)))
    return result


def firmware_lerp(table, ratio):
    """MotorPosition::lerp on the motor.offset[] table."""
    n = len(table)
    left = min(n - 1, int(ratio * n))
    right = (left + 1) % n
    fraction = (ratio - left / n) * n
    return table[left] * (1.0 - fraction) + table[right] * fraction


def firmware_errors(hall, offset, sign, polarity, table, poles):
    """Firmware electrical theta minus true electrical theta at the
    instant of every sector transition, for every pole pair.  The
    firmware places entry into count k at k * 60 electrical deg plus the
    interpolated motor.offset[] correction."""
    cpr = 3 * poles
    errors = []
    for pole_pair in range(poles // 2):
        for k_true in range(6):
            k_fw = ce.hall_bits_to_count(
                COUNT_TO_BITS[k_true], offset, sign, polarity)
            ratio = (pole_pair * 6 + k_fw) / cpr
            theta_fw = k_fw * 60.0 + math.degrees(firmware_lerp(table, ratio))
            errors.append(wrap_deg(theta_fw - hall.true_boundary(k_true)))
    return errors


def worst(errors):
    return max(errors, key=abs)


def old_calibration_error(hall, poles):
    """The pre-fine-calibration moteus_tool: 24 samples at 15 electrical deg,
    the state seen at phase 0 becomes count 0, the sign comes from the
    first transition, and motor.offset[] is zeroed."""
    data = sweep(hall, OLD_STEPS, 1, +1)[:OLD_STEPS]
    counts = [ce._HALL_BITS_TO_COUNT[raw] for _, raw in data]
    offset = -counts[0]
    next_count = counts[0]
    for c in counts[1:]:
        next_count = c
        if next_count != counts[0]:
            break
    sign = 1 if ((next_count + 6 + 3 - counts[0]) % 6 - 3) > 0 else -1
    table = [0.0] * ce.HALL_OFFSET_TABLE_SIZE
    return worst(firmware_errors(hall, offset, sign, 0, table, poles))


def new_calibration_error(hall, poles, cycles):
    """The current moteus_tool: 90 samples per cycle, forward then
    reverse, over min(cycles, poles / 2) electrical cycles, with the
    motor.offset[] table built from the circular mean of every
    observed transition."""
    cycles = min(cycles, poles // 2)
    fwd = sweep(hall, STEPS_PER_CYCLE_NEW, cycles, +1)
    rev = sweep(hall, STEPS_PER_CYCLE_NEW, cycles, -1)
    result = ce.calibrate_hall(fwd)
    table, _, _ = ce.build_hall_offset_table_multi(
        [fwd, rev], result, poles=poles)
    return worst(firmware_errors(
        hall, result.offset, result.sign, result.polarity, table, poles))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--cycles', type=int, default=3,
                        help='electrical cycles per sweep direction for '
                        'the new approach (default %(default)s)')
    parser.add_argument('--hysteresis', type=float, default=0.0,
                        help='hall hysteresis plus rotor settling lag, '
                        'electrical deg, applied in the direction of travel '
                        '(default %(default)s)')
    parser.add_argument('--resolution', type=float, default=0.25,
                        help='hall offset grid spacing in electrical deg '
                        '(default %(default)s)')
    parser.add_argument('--output', metavar='FILE',
                        help='write the figure to FILE instead of '
                        'displaying it')
    args = parser.parse_args()

    # Two full sector widths, so the periodicity of the old error is
    # visible.  Stay strictly off multiples of the resolution so no
    # sample lands exactly on a transition, where the simulated hall
    # state would be arbitrary.
    steps = int(round(OFFSET_RANGE_DEG / args.resolution))
    offsets = [(i + 0.5) * args.resolution for i in range(steps)]

    old_errors = []
    new_errors = []
    for offset in offsets:
        hall = SimulatedHall(offset, args.hysteresis)
        old_errors.append(old_calibration_error(hall, POLES))
        new_errors.append(new_calibration_error(
            hall, POLES, args.cycles))

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(offsets, old_errors, color='#2a78d6', linewidth=2,
            label='Pre 1.0.0 calibration')
    ax.plot(offsets, new_errors, color='#eb6834', linewidth=2,
            label='New: fine forward/reverse sweep, averaged')
    ax.axhline(0.0, color='#52514e', linewidth=0.8)

    ax.set_xlim(0.0, OFFSET_RANGE_DEG)
    ax.set_xticks(range(0, int(OFFSET_RANGE_DEG) + 1, 10))
    ax.set_xlabel(
        'Hall transition offset from electrical phase (electrical deg)')
    ax.set_ylabel('Commutation angle error (electrical deg)')
    title = 'Hall calibration commutation error'
    if args.hysteresis != 0.0:
        title += f', {args.hysteresis:g} electrical-degree hysteresis'
    ax.set_title(title)
    ax.grid(True, color='#dddcd8', linewidth=0.6)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    # Below the axes, where neither curve can collide with it.
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.18),
              frameon=False)
    fig.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=150)
    else:
        plt.show()


if __name__ == '__main__':
    main()
