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

import json
import math

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


def parse_file(fp):
    lines = [x.decode('latin1') for x in fp.readlines()]
    if not lines[0].startswith("CAL start"):
        raise RuntimeError("calibration does not start with magic line")
    if not lines[-1].startswith("CAL done"):
        raise RuntimeError("calibration does not end with magic line")

    lines = lines[1:-1]

    entries = [_parse_entry(line) for line in lines]

    result = File()
    result.phase_up = [x for x in entries if x.direction == 1]
    result.phase_down = [x for x in entries if x.direction == 2]

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
            errs[j - start] = _wrap_neg_pi_to_pi(values[wrap(j)] - values[wrap(start)])
        result[i] = values[wrap(start)] + (sum(errs) / len(errs))

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

        self.errors = []

    def __repr__(self):
        return json.dumps({
            "invert": self.invert,
            "phase_invert": self.phase_invert,
            "poles": self.poles,
            "offset": self.offset,
            "errors": self.errors,
            })

    def to_json(self):
        return {
            'invert': self.invert,
            'phase_invert': self.phase_invert,
            'poles': self.poles,
            'offset': self.offset,
        }


def calibrate(parsed,
              desired_direction=1,
              max_remainder_error=0.1,
              allow_phase_invert=True):
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

    err = [_wrap_neg_pi_to_pi(a - b) for a, b in zip(avg_interp, expected)]

    # Make the error seem reasonable, so unwrap if we happen to span
    # the pi boundary.
    if (max(err) - min(err)) > 1.5 * math.pi:
        err = [x if x > 0 else x + 2 * math.pi for x in err]

    avg_window = int(len(err) / result.poles)
    avg_err = _window_average(err, avg_window)

    offset_x = list(range(0, 65536, 1024))
    offset = _interpolate(offset_x, xpos, avg_err)

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


def calibrate_hall(data,
                   desired_direction=1,
                   allow_phase_invert=True):
    result = HallCalibrationResult()

    hall_mapping = [
        0, 0, 2, 1, 4, 5, 3, 0,
    ]

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
    result.offset = -hall_mapping[closest_to_zero[1]]

    start_count = hall_mapping[data[0][1]]
    next_count = start_count
    for x in data[1:]:
        next_count = hall_mapping[x[1]]
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
