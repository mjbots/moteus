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

"""Process a .log file from a static torque ripply dynamometer run and
produce a result plot with some useful metrics."""

import argparse
import bisect
import datetime
import math
import matplotlib.pyplot as plt
import numpy
import sys

import mjlib.telemetry.file_reader as file_reader



def wrap_one(value):
    while value > 0.5:
        value -= 1.0
    while value < -0.5:
        value += 1.0
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--ripple', '-r', action='store_true')
    parser.add_argument('--sign', type=float, default=1.0)
    parser.add_argument('--skip-plot', action='store_true')
    parser.add_argument('filename', type=str, nargs=1)
    args = parser.parse_args()


    fr = file_reader.FileReader(args.filename[0])
    data = fr.get(["torque", "dut_servo_cmd", "dut_servo_stats", "dut_git"])

    # Figure out which torques were tested.
    dut_servo_cmd = data["dut_servo_cmd"]
    dut_servo_stats = data["dut_servo_stats"]
    tested_torques = set(
        [float('nan')] + [x.data.feedforward_Nm for x in dut_servo_cmd])

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
                delta = wrap_one(this_position - cur_position)
                if abs(total_position_delta + delta) >= 1.0:
                    return last_timestamp

                total_position_delta += delta
            cur_position = this_position
            last_timestamp = item.timestamp
        raise RuntimeError("could not find full revolution")

    # For each torque, find the range of times that constitutes a
    # test.
    def find_test(torque_Nm):
        # First, pick the largest region with the given torque.
        if math.isnan(torque_Nm):
            regions = find_regions(
                dut_servo_cmd,
                lambda x: (x.data.mode.value == 0))
        else:
            regions = find_regions(
                dut_servo_cmd,
                lambda x: (x.data.mode.value == 10 and
                           x.data.feedforward_Nm == torque_Nm))

        regions.sort(reverse=True, key=lambda x: len(x))
        biggest_region = regions[0]

        # Skip from the start to move past startup transients.
        begin = biggest_region[0].timestamp + 4.0
        end = find_full_encoder(begin)

        return {
            "torque" : torque_Nm if not math.isnan(torque_Nm) else 'stop',
            "begin" : begin,
            "end" : end,
            }
    tests = [find_test(x) for x in tested_torques]
    tests.sort(key=lambda x: abs(x["torque"]) if type(x["torque"]) == float else -1)
    print("** TESTS")
    print(tests)
    print()

    analysis = []

    if not args.skip_plot:
        ax = plt.subplot(111)
        cmap = plt.get_cmap("tab10")

    for i, test in enumerate(tests):
        si = bisect_left(data["torque"], test["begin"])
        se = bisect_left(data["torque"], test["end"])

        timed_torque = [(x.timestamp, args.sign * x.data.torque_Nm)
                        for x in data["torque"][si:se]]
        offset = sum([x[1] for x in timed_torque]) / len(timed_torque)

        compare_torque = (offset if args.ripple else (
            test["torque"] if type(test["torque"]) == float else 0.0))

        to_plot = [
            (wrap_one(dut_servo_stats[bisect_left(dut_servo_stats, x[0])].data.position), x[1])
            for x in timed_torque]
        to_plot.sort()

        torque_name = (f'{test["torque"]:.2f} Nm'
                       if type(test["torque"]) == float else "stop Nm")
        if not args.skip_plot:
            color = cmap(i)
            color = (color[0], color[1], color[2], 1.0 - float(i) / len(tests))
            ax.plot([x[0] for x in to_plot],
                    [(x[1] - compare_torque) for x in to_plot],
                    label=torque_name,
                    color=color)

        torques = [x[1] for x in to_plot]
        std = numpy.std(torques)
        pkpk = max(torques) - min(torques)
        analysis.append(f'{torque_name} std={std:.3f} pk-pk={pkpk:.3f}')

    if not args.skip_plot:
        ax.grid()
        ax.legend()
        ax.set_title("Torque {} vs Encoder Position".format(
            "Error" if not args.ripple else "Ripple"))
        ax.set_ylabel("Transducer measured N*m")
        ax.set_xlabel("Magnetic Encoder Position")

    git_data = data["dut_git"][0].data
    git_hash = ''.join(['{:02x}'.format(x) for x in git_data.hash])
    test_timestr = datetime.datetime.utcfromtimestamp(
        data["dut_git"][0].timestamp).isoformat()
    test_info = f'git: {git_hash} dirty: {git_data.dirty} time: {test_timestr}'

    print("** ANALYSIS")
    print('\n'.join(analysis))
    print()
    print("** GIT")
    print(test_info)

    if not args.skip_plot:
        ax.text(0.95, 0.01, test_info,
                horizontalalignment='right', transform=ax.transAxes)
        ax.text(0.95, 0.04, '\n'.join(analysis),
                horizontalalignment='right',
                verticalalignment='bottom',
                transform=ax.transAxes)

        plt.show()


if __name__ == '__main__':
    main()
