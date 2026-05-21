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

import json
import math
import scipy.optimize

BIN_COUNT = 64

class Entry:
    direction = 0
    phase = 0
    encoder = 0
    i1 = 0
    i2 = 0
    i3 = 0

class File:
    phase_up = None
    phase_down = None


def _parse_entry(line):
    result = Entry()

    fields = line.split(' ')
    if len(fields) < 3:
        raise RuntimeError("malformed line: " + line)

    result.direction = int(fields[0])
    result.phase = int(fields[1])
    result.encoder = int(fields[2])
    for field in fields[3:]:
        key, val = field.split('=')
        if key == 'i1':
            result.i1 = float(val) * 0.001
        elif key == 'i2':
            result.i2 = float(val) * 0.001
        elif key == 'i3':
            result.i3 = float(val) * 0.001

    return result


def check_line(cmd):
    return ' '.join(cmd.strip().split(' ')[0:2])


def parse_file(fp):
    lines = [x.decode('latin1') for x in fp.readlines()]
    if not check_line(lines[0]) in ["CAL start", "CALI start"]:
        raise RuntimeError("calibration does not start with magic line")
    if not check_line(lines[-1]) in ["CAL done", "CALI done"]:
        raise RuntimeError("calibration does not end with magic line")

    lines = lines[1:-1]

    entries = [_parse_entry(line) for line in lines]

    result = File()
    result.phase_up = [x for x in entries if x.direction == 1 or x.direction == 3]
    result.phase_down = [x for x in entries if x.direction == 2 or x.direction == 4]

    return result


def _wrap_uint16(value):
    while value < 0:
        value += 65536
    while value > 65535:
        value -= 65536
    return value


def _wrap_int16(value):
    while value > 32767:
        value -= 65536
    while value < -32768:
        value += 65536
    return value


def _wrap_neg_pi_to_pi(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def _unwrap(value):
    result = []
    for item in value:
        if len(result) == 0:
            result.append(item)
        else:
            result.append(result[-1] + _wrap_neg_pi_to_pi(item - result[-1]))
    return result


def _linspace(start, end, count):
    result = []
    for i in range(count):
        result.append(end if (i + 1) == count else
                      i * (end - start) / (count - 1) + start)

    return result


def _interpolate(sample_points, x, y):
    assert len(x) > 1
    assert len(x) == len(y)

    xindex = 0

    result = [0] * len(sample_points)
    for i in range(len(sample_points)):
        point = sample_points[i]

        if point < x[xindex]:
            value = y[xindex]
        else:
            while ((xindex + 2) < len(x) and
                   point >= x[xindex + 1]):
                xindex += 1

            if point > x[xindex + 1]:
                # We're past the end
                value = y[xindex + 1]
            else:
                # Linearly interpolate.
                length = x[xindex + 1] - x[xindex]
                if length == 0.0:
                    value = y[xindex + 1]
                else:
                    ratio = (point - x[xindex]) / length
                    value = (y[xindex + 1] - y[xindex]) * ratio + y[xindex]

        result[i] = value

    return result


def _window_average(values, window_size):
    def wrap(v):
        if v < 0:
            return v + len(values)
        if v >= len(values):
            return v - len(values)
        return v

    result = [0] * len(values)
    for i in range(len(values)):
        start = i - window_size // 2
        end = i + window_size // 2
        errs = [0] * (end - start)
        for j in range(start, end):
            errs[j - start] = values[wrap(j)]
        result[i] = sum(errs) / len(errs)

    return result


class CalibrationResult:
    def __init__(self):
        self.invert = None
        self.phase_invert = None
        self.poles = None
        self.offset = None

        self.total_phase = None
        self.total_delta = None
        self.ratio = None

        self.fit_metric = None

        self.current_quality_factor = None

        self.errors = []

    def __repr__(self):
        return json.dumps({
            "invert": self.invert,
            "phase_invert": self.phase_invert,
            "poles": self.poles,
            "offset": self.offset,
            "fit_metric": self.fit_metric,
            "current_quality_factor": self.current_quality_factor,
            "errors": self.errors,
            })

    def to_json(self):
        return {
            'invert': self.invert,
            'phase_invert': self.phase_invert,
            'poles': self.poles,
            'offset': self.offset,
            'fit_metric': self.fit_metric,
            'current_quality_factor': self.current_quality_factor,
        }


def calibrate(parsed,
              desired_direction=1,
              max_remainder_error=0.1,
              allow_phase_invert=True,
              allow_optimize=True,
              force_optimize=False):
    '''Calibrate the motor.

    :param desired_direction: For positive unwrapped_position, should
      the encoder increase (positive value), or decrease (negative
      value)

    :param allow_phase_invert: If False, then phase_invert must remain
      false (presumably because the firmware does not support it.
    '''
    if (len(parsed.phase_up) < 2 or
        len(parsed.phase_down) < 2):
        raise RuntimeError("one or more phases were empty")

    # We discard the first few entries from the phase up side, since
    # they will be bogus.
    del(parsed.phase_up[0:4])

    total_delta = sum([
        _wrap_int16(b.encoder - a.encoder) for a, b in
        zip(parsed.phase_up[0:], parsed.phase_up[1:])])

    result = CalibrationResult()

    if abs(abs(total_delta)- 65536) > 5000:
        result.errors.append("phase_up did not traverse appropriate encoder distance")

    # Figure out inversion.
    if desired_direction > 0 and total_delta > 0:
        # This is the "norminal" case.
        result.invert = False
        result.phase_invert = False
    elif desired_direction < 0 and total_delta < 0:
        # Here, we want to be moving backwards, so we flip the encoder
        # sign.
        result.invert = True
        result.phase_invert = False
    elif desired_direction < 0 and total_delta > 0:
        result.invert = True
        result.phase_invert = True
    elif desired_direction > 0 and total_delta < 0:
        result.invert = False
        result.phase_invert = True
    else:
        raise RuntimeError(
            f"Invalid desired_direction={desired_direction} " +
            f"and total_delta={total_delta}")

    if result.phase_invert:
        # We need to flip phase.
        if not allow_phase_invert:
            raise RuntimeError(
                "Requested motor direction not possible with " +
                "current firmware version")
        for item in parsed.phase_up:
            item.phase = _wrap_uint16(-item.phase)
            item.i2, item.i3 = item.i3, item.i2
        for item in parsed.phase_down:
            item.phase = _wrap_uint16(-item.phase)
            item.i2, item.i3 = item.i3, item.i2

    if result.invert:
        for item in parsed.phase_up:
            item.encoder = 65535 - item.encoder
        for item in parsed.phase_down:
            item.encoder = 65535 - item.encoder
        total_delta *= -1

    # Next, figure out the number of poles.  We compare the total
    # encoder delta to the total phase delta.
    total_phase = float(sum([
        _wrap_int16(b.phase - a.phase) for a, b in
        zip(parsed.phase_up[0:], parsed.phase_up[1:])]))

    ratio = total_phase / total_delta
    remainder = abs(round(ratio) - ratio);

    if remainder > max_remainder_error:
        result.errors.append(
            f"encoder not an integral multiple of phase, " +
            f"{remainder} > {max_remainder_error}")

    result.total_phase = total_phase
    result.total_delta = total_delta
    result.ratio = ratio

    result.poles = int(round(ratio) * 2)

    # Now we need to figure out the phase offset at select points.  We
    # interpolate and average the phase up and phase down sections.
    phase_up_by_encoder = sorted(parsed.phase_up, key=lambda x: x.encoder)
    phase_down_by_encoder = sorted(parsed.phase_down, key=lambda x: x.encoder)

    offset = phase_down_by_encoder[0].phase - phase_up_by_encoder[0].phase
    if abs(offset) > 32767:
        # We need to shift it so that they start from the same place.
        change = int(-65536 * round(offset / 65536.0))
        for item in phase_down_by_encoder:
            item.phase += change

    phase_up_encoder = [x.encoder for x in phase_up_by_encoder]
    phase_up_phase = [2.0 * math.pi / 65536.0 * x.phase for x in phase_up_by_encoder]
    phase_up_phase = _unwrap(phase_up_phase)

    phase_down_encoder = [x.encoder for x in phase_down_by_encoder]
    phase_down_phase = [2.0 * math.pi / 65536.0 * x.phase for x in phase_down_by_encoder]
    phase_down_phase = _unwrap(phase_down_phase)

    xpos = _linspace(0, 65535.0, 10000)

    pu_interp = _interpolate(xpos, phase_up_encoder, phase_up_phase)
    pd_interp = _interpolate(xpos, phase_down_encoder, phase_down_phase)
    avg_interp = [0.5 * (a + b) for a, b in zip(pu_interp, pd_interp)]

    expected = [(2.0 * math.pi / 65536.0) * (result.poles / 2) * x for x in xpos]

    err = [a - b for a, b in zip(avg_interp, expected)]

    # Make the error balanced about 0.
    mean_err = sum(err) / len(err)
    wrapped_mean_err = _wrap_neg_pi_to_pi(mean_err)
    delta_mean_err = mean_err - wrapped_mean_err
    err = [x - delta_mean_err for x in err]

    avg_window = int(len(err) / result.poles)
    avg_err = _window_average(err, avg_window)

    offset_x = list(range(0, 65536, 65536 // BIN_COUNT))
    offset = _interpolate(offset_x, xpos, avg_err)

    MAX_ERROR = 0.8

    # The firmware will complain if individual steps are more
    # than this apart.
    MAX_STEP_CHANGE = 3.5

    # Penalize solutions that have error greater than this.
    PENALTY_RATIO = 0.80

    def full_metric(x, *args):
        errors = 0
        result = 0
        resampled = _interpolate(xpos, offset_x + [65536],
                                 list(x) + [x[0]])
        for a, b in zip(err, resampled):
            this_err = abs(_wrap_neg_pi_to_pi(a - b))
            # Heavily penalize if we would have an error.
            if this_err > PENALTY_RATIO * MAX_ERROR:
                this_err = (PENALTY_RATIO * MAX_ERROR +
                            (this_err - PENALTY_RATIO * MAX_ERROR) * 10)
            if this_err > MAX_ERROR:
                errors += 1
            result += this_err ** 2

        for i in range(0, len(list(x))):
            nexti = (i + 1) % len(list(x))
            delta = abs(x[nexti] - x[i])
            if delta > (PENALTY_RATIO * MAX_STEP_CHANGE):
                result += (
                    10 * (delta - PENALTY_RATIO * MAX_STEP_CHANGE) ** 2)
        return result, errors

    def metric(x, *args):
        result = full_metric(x, *args)[0]
        print(f"optimizing - metric={result:.5f}     ", end='\r', flush=True)
        return result

    starting_metric, starting_errors = full_metric(offset)

    if (force_optimize or (
            allow_optimize and
            (starting_metric > 30 or
             starting_errors > 0))):
        print()
        print(f"Initial metric: {starting_metric}")
        # Optimize these initial offsets.
        optimres = scipy.optimize.minimize(metric, offset, tol=1e1)

        if not optimres.success:
            result.errors.append(
                f"optimization failed {optimres.message}")

        print()
        offset = list(optimres.x)

    result.fit_metric, _ = full_metric(offset)

    # Now double check our results.
    resampled_offset = _interpolate(xpos, offset_x + [65536],
                                    offset + [offset[0]])
    any_sample_error = False
    sample_errors = []
    for a, b in zip(err, resampled_offset):
        sample_error = _wrap_neg_pi_to_pi(a - b)
        sample_errors.append(sample_error)
        if not any_sample_error and abs(sample_error) > MAX_ERROR:
            result.errors.append(
                f"excessive error in curve fit |{sample_error}| > {MAX_ERROR}")
            any_sample_error = True

    result.offset = offset

    result.debug = {
        'phase_up_encoder': phase_up_encoder,
        'phase_up_phase' : phase_up_phase,
        'phase_down_encoder' : phase_down_encoder,
        'phase_down_phase' : phase_down_phase,
        'xpos' : xpos,
        'expected' : expected,
        'avg_interp' : avg_interp,
        'err' : err,
        'avg_err' : avg_err,
        'offset_x' : offset_x,
        'offset' : offset,
        'sample_errors' : sample_errors,
    }

    return result


class HallCalibrationResult:
    def __init__(self):
        self.offset = None
        self.sign = None
        self.polarity = None
        self.phase_invert = None

        self.errors = []

    def __repr__(self):
        return json.dumps({
            "offset": self.offset,
            "sign": self.sign,
            "polarity": self.polarity,
            "phase_invert": self.phase_invert,
            })

    def to_json(self):
        return {
            'offset': self.offset,
            'sign': self.sign,
            'polarity': self.polarity,
            'phase_invert': self.phase_invert,
        }


# The bits-to-sector-count mapping used by the moteus firmware
# (kHallMapping in fw/aux_common.h).
_HALL_BITS_TO_COUNT = [0, 0, 2, 1, 4, 5, 3, 0]

# Size of the motor.offset[] commutation correction table in the
# firmware (fw/bldc_servo.h).
HALL_OFFSET_TABLE_SIZE = 64


def calibrate_hall(data,
                   desired_direction=1,
                   allow_phase_invert=True):
    result = HallCalibrationResult()

    states_seen = [x[1] for x in data]
    counts = {}
    for x in states_seen:
        counts[x] = counts.get(x, 0) + 1
    if len(counts) != 6:
        raise RuntimeError(f"{len(counts)} hall states seen, expected 6")

    # For now, assume 120 deg separation.
    if counts.get(0, 0) != 0 or counts.get(7, 0) != 0:
        raise RuntimeError(
            "We currently only support halls with 120 degree separation")

    # Find the offset.
    closest_to_zero = min([(abs(_wrap_neg_pi_to_pi(x[0])), x[1]) for x in data])
    result.offset = -_HALL_BITS_TO_COUNT[closest_to_zero[1]]

    start_count = _HALL_BITS_TO_COUNT[data[0][1]]
    next_count = start_count
    for x in data[1:]:
        next_count = _HALL_BITS_TO_COUNT[x[1]]
        if next_count != start_count:
            break

    result.sign = 1 if ((next_count + 6 + 3 - start_count) % 6 - 3) > 0 else -1

    result.polarity = 0
    result.phase_invert = 0

    if desired_direction == -1:
        if not allow_phase_invert:
            raise RuntimeError("Requested motor direction not possible "
                               "with current firmware version")

        result.sign *= -1
        result.phase_invert = 1

    return result


def hall_bits_to_count(raw_bits, offset, sign, polarity):
    """Apply the basic hall calibration (polarity / offset / sign) to
    a raw hall.bits reading and return the resulting "sector count" in
    [0, 6) -- the same value the firmware uses for slow-mode tracking.
    """
    base = _HALL_BITS_TO_COUNT[raw_bits ^ polarity]
    return ((base + offset) * sign + 6) % 6


def find_hall_boundary_phases(samples):
    """Given a list of (phase_rad, sector_count) samples taken at
    monotonically increasing phases across [0, 2*pi), locate each of
    the 6 sector boundaries.

    boundary_phase[k] is the lower-phase edge of the contiguous
    "count == k" interval in sweep order, regardless of whether the
    count sequence is locally increasing or decreasing.  Equivalently,
    it is the midpoint phase of the +-1 transition that lands on
    count == k.

    Returns a list of 6 boundary phases (in radians in [0, 2*pi)).
    Raises if any sector boundary was not observed -- usually a sign
    the step size is too coarse.
    """
    boundary_phase = [None] * 6
    n = len(samples)
    for i in range(n):
        a_phase, a_count = samples[i]
        b_phase, b_count = samples[(i + 1) % n]
        # Wrap b_phase forward across the 0/2pi seam.
        if (i + 1) == n:
            b_phase = b_phase + 2 * math.pi
        # Balanced delta in [-3, 3].  A single-step transition (in
        # either direction) is one we can use; everything else is
        # either no transition or a multi-step jump from a coarse
        # sample interval (the latter is reported via the missing-
        # sectors check below).
        delta = ((b_count - a_count + 3) % 6) - 3
        if abs(delta) != 1:
            continue
        midpoint = 0.5 * (a_phase + b_phase) % (2 * math.pi)
        if boundary_phase[b_count] is None:
            boundary_phase[b_count] = midpoint

    missing = [k for k, v in enumerate(boundary_phase) if v is None]
    if missing:
        raise RuntimeError(
            f"Hall transition scan missed sectors {missing}; "
            f"reduce step size or raise encoder voltage.")
    return boundary_phase


def compute_hall_offset_table(boundary_phases, cpr,
                              table_size=HALL_OFFSET_TABLE_SIZE):
    """Compute the motor.offset[] electrical-angle correction table
    (in radians) from per-sector boundary phases.

    boundary_phases[k] is the measured rotor electrical angle (radians
    in [0, 2*pi)) at which the firmware will report "we just entered
    sector k".  The firmware would otherwise place this transition at
    the ideal angle k * (2*pi/6); the per-boundary correction is
    delta_k = boundary_phases[k] - k * pi/3, wrapped to [-pi, pi].

    The motor.offset[] table is indexed by the rotor's filtered_value
    / cpr ratio (where cpr = 3 * poles for hall sources).  Each table
    entry's equivalent hall-sector position is filtered_value mod 6,
    and the offset there is the linear interpolation of the
    surrounding two boundary deltas.

    Returns a list of `table_size` floats in radians.
    """
    deltas = []
    for k in range(6):
        ideal = (k / 6.0) * 2 * math.pi
        err = ((boundary_phases[k] - ideal + math.pi)
               % (2 * math.pi)) - math.pi
        deltas.append(err)

    out = []
    for i in range(table_size):
        ratio = (i + 0.5) / table_size
        # ratio is in (0, 1) so mod is always in [0, 6) and
        # int(mod) is always in [0, 5].
        mod = (ratio * cpr) % 6.0
        sector = int(mod)
        assert sector <= 5
        frac = mod - sector
        next_sector = (sector + 1) % 6
        out.append((1.0 - frac) * deltas[sector] +
                   frac * deltas[next_sector])
    return out


def build_hall_offset_table(hall_cal_data, cal_result, poles,
                            table_size=HALL_OFFSET_TABLE_SIZE):
    """Build the motor.offset[] table from a raw hall sweep and the
    `calibrate_hall` result.

    hall_cal_data is a list of (phase_rad, raw_bits) pairs spanning
    one electrical cycle.  The natural (pre --cal-invert) sign is
    used internally so the table reflects the physical hall layout
    rather than the post-flip firmware view; the same table is
    correct regardless of cal_result.phase_invert.

    Returns (offset_table, boundary_phases) where offset_table is the
    list of `table_size` motor.offset[] values and boundary_phases is
    the per-sector measurement that produced it (useful for logging).
    """
    natural_sign = (-cal_result.sign if cal_result.phase_invert
                    else cal_result.sign)
    sector_samples = [
        (phase, hall_bits_to_count(
            raw_bits, offset=cal_result.offset,
            sign=natural_sign,
            polarity=cal_result.polarity))
        for (phase, raw_bits) in hall_cal_data]
    boundary_phases = find_hall_boundary_phases(sector_samples)
    offset_table = compute_hall_offset_table(
        boundary_phases, cpr=3 * poles, table_size=table_size)
    return offset_table, boundary_phases
