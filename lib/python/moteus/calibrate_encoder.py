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
        self.poles = None
        self.offset = None

        self.total_phase = None
        self.total_delta = None
        self.ratio = None

        self.errors = []

    def __repr__(self):
        return json.dumps({
            "invert": self.invert,
            "poles": self.poles,
            "offset": self.offset,
            "errors": self.errors,
            })

    def to_json(self):
        return {
            'invert': self.invert,
            'poles': self.poles,
            'offset': self.offset,
        }


def calibrate(parsed):
    if (len(parsed.phase_up) < 2 or
        len(parsed.phase_down) < 2):
        raise RuntimeError("one or more phases were empty")

    # We discard the first few entries from the phase up side, since
    # they will be bogus.
    del(parsed.phase_up[0:4])

    total_delta = sum([
        _wrap_int16(b.encoder - a.encoder) for a, b in
        zip(parsed.phase_up[0:], parsed.phase_up[1:])])

    if abs(abs(total_delta)- 65536) > 5000:
        result.errors.append("phase_up did not traverse appropriate encoder distance")

    result = CalibrationResult()

    # Figure out inversion.
    if total_delta < 0:
        result.invert = True
        for item in parsed.phase_up:
            item.encoder = 65535 - item.encoder
        for item in parsed.phase_down:
            item.encoder = 65535 - item.encoder
        total_delta *= -1
    else:
        result.invert = False

    # Next, figure out the number of poles.  We compare the total
    # encoder delta to the total phase delta.
    total_phase = float(sum([
        _wrap_int16(b.phase - a.phase) for a, b in
        zip(parsed.phase_up[0:], parsed.phase_up[1:])]))

    ratio = total_phase / total_delta
    remainder = abs(round(ratio) - ratio);

    MAX_REMAINDER_ERROR = 0.1
    if remainder > MAX_REMAINDER_ERROR:
        result.errors.append(
            f"encoder not an integral multiple of phase, " +
            f"{remainder} > {MAX_REMAINDER_ERROR}")

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
