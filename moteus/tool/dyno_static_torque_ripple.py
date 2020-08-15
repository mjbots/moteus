#!/usr/bin/python3

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

"""Process a .log file from a static torque ripply dynamometer run and
produce a result plot with some useful metrics."""

import bisect
import matplotlib.pyplot as plt
import sys

import mjlib.telemetry.file_reader as file_reader


def wrap_int16(value):
    while value > 32767:
        value -= 65536
    while value < -32768:
        value += 65536
    return value


def bisect_left(data, timestamp):
    timestamps = [x.timestamp for x in data]
    return bisect.bisect_left(timestamps, timestamp)


def find_regions(data, predicate):
    '''Given a list @p data, return a list of lists, where each result is
    a contiguous block from the original 'data' for which @p predicate
    returned truthy.
    '''

    result = []
    current = []
    old_p = False
    for d in data:
        this_p = predicate(d)
        if this_p:
            current.append(d)
        if not this_p and old_p:
            result.append(current)
            current = []
        old_p = this_p

    if len(current):
        result.append(current)
    return result


def main():
    fr = file_reader.FileReader(sys.argv[1])
    data = fr.get(["torque", "dut_servo_cmd", "dut_servo_stats", "dut_git"])

    # Figure out which torques were tested.
    dut_servo_cmd = data["dut_servo_cmd"]
    dut_servo_stats = data["dut_servo_stats"]
    tested_torques = set([x.data.feedforward_Nm for x in dut_servo_cmd])

    def find_full_encoder(start):
        # Then we want to go until just before a full encoder
        # revolution has taken place.
        start_position_index = bisect_left(dut_servo_stats, start)
        total_position_delta = 0
        cur_position = None
        last_timestamp = None
        for item in dut_servo_stats[start_position_index:]:
            this_position = item.data.position
            if cur_position:
                delta = wrap_int16(this_position - cur_position)
                if abs(total_position_delta + delta) >= 65536:
                    return last_timestamp

                total_position_delta += delta
            cur_position = this_position
            last_timestamp = item.timestamp
        raise RuntimeError("could not find full revolution")

    # For each torque, find the range of times that constitutes a
    # test.
    def find_test(torque_Nm):
        # First, pick the largest region with the given torque.
        regions = find_regions(
            dut_servo_cmd,
            lambda x: (x.data.mode.value == 10 and
                       x.data.feedforward_Nm == torque_Nm))

        regions.sort(reverse=True, key=lambda x: len(x))
        biggest_region = regions[0]

        # Skip 2s from the start
        begin = biggest_region[0].timestamp + 2.0
        end = find_full_encoder(begin)

        return {
            "torque" : torque_Nm,
            "begin" : begin,
            "end" : end,
            }
    tests = [find_test(x) for x in tested_torques]
    print("tests: ", tests)

    ax = plt.subplot(111)

    for test in tests:
        si = bisect_left(data["torque"], test["begin"])
        se = bisect_left(data["torque"], test["end"])

        timed_torque = [(x.timestamp, x.data.torque_Nm)
                        for x in data["torque"][si:se]]
        to_plot = [
            (dut_servo_stats[bisect_left(dut_servo_stats, x[0])].data.position, x[1])
            for x in timed_torque]
        to_plot.sort()
        ax.plot([x[0] for x in to_plot], [(x[1] - test["torque"]) for x in to_plot],
                label=f'{test["torque"]:.2f} Nm')

    ax.grid()
    ax.legend()
    ax.set_title("Torque Ripple vs Encoder Position")

    plt.show()


if __name__ == '__main__':
    main()
