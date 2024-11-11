#!/usr/bin/python3 -B

# Copyright 2024 mjbots Robotic Systems, LLC.  info@mjbots.com
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


"""This utility can be used to generate encoder compensation tables
for moteus controllers.  There are two possible methods:

1. "inertial mode": The inertia of the system is used to attempt to get
the motor to spin at a constant speed, then deviations from this
constant speed can be assumed to be caused by encoder non-linearity.
This can be used even if there is only a single encoder configured.

However, if a more accurate reference encoder is available even
temporarily, it is recommend to first set it as the commutation
encoder, calibrate, then perform compensation on the target encoder.

2. "reference mode": Here, the controller is advanced slowly through
several full revolutions while in position mode.  It can be invoked by
specifying '--reference-encoder X'.  The reference encoder is assumed
to be perfect and must be used for commutation.  The encoder
compensation table for the target encoder is adjusted to match the
reference encoder.  With a quality reference encoder, this produces
the highest quality compensation results.  It does require that PID
values be somewhat tuned before performing the calibration, or that
--reference-kp-scale is used to get at least the kp to a usable state.


Note: If the encoder being compensated is currently being used for
commutation, then this process will invalidate the current commutation
calibration, requiring a subsequent invocation of:

`moteus_tool -t X --calibrate`.

"""

import argparse
import asyncio
import math
import matplotlib
import matplotlib.pyplot as plt
import moteus
import numpy
import sys
import time

import histogram


PRINT_DURATION = 0.1


def parse_config(data):
    lines = data.split(b'\n')
    return dict(line.decode('latin1').split(' ', 1) for line in lines
                if line.strip() != b'')

def find_compensation_size(motor_position_config):
    source0_comp = [
        x for x in motor_position_config.keys()
        if x.startswith("motor_position.sources.0.compensation_table")
    ]
    size = len(source0_comp)
    bits = int(math.log2(size))
    if 2 ** bits != size:
        raise RuntimeError("could not determine size of compensation table")

    return size


def mean_value(items):
    return sum([x[1] for x in items]) / len(items)


def integrate(xyvalues):
    v = 0

    result = [(xyvalues[0][0], 0.0)]
    for i in range(1, len(xyvalues) + 1):
        cur = xyvalues[i % len(xyvalues)]
        old = xyvalues[i - 1]

        delta = math.fmod(cur[0] - old[0] + 1.0, 1.0) * 0.5 * (cur[1] + old[1])

        v += delta

        lastx = result[-1][0]

        result.append((lastx + math.fmod(1.0 + cur[0] - old[0], 1.0), v))

    return result


def sample(items, num_bins):
    result = []

    xpoints = numpy.arange(0.0, 1.0, 1.0 / num_bins)

    ypoints = list(numpy.interp(xpoints,
                                [x[0] for x in items],
                                [x[1] for x in items],
                                period=1.0))

    return list(zip(list(xpoints), list(ypoints)))


def unpack_plot(items):
    return [[x[0] for x in items], [x[1] for x in items]]


def wrap_half(value):
    while value > 0.5:
        value -= 1.0
    while value < -0.5:
        value += 1.0
    return value


def wrap_zero_one(value):
    while value > 1.0:
        value -= 1.0
    while value < 0.0:
        value += 1.0
    return value


def get_encoder(item, number):
    if number == 0:
        return item.values[moteus.Register.ENCODER_0_POSITION]
    elif number == 1:
        return item.values[moteus.Register.ENCODER_1_POSITION]
    elif number == 2:
        return item.values[moteus.Register.ENCODER_2_POSITION]


async def run_reference_compensation(args, m, s, ax):
    results = []
    start_time = time.time()
    last_print = start_time
    total_duration = args.reference_revolutions / args.reference_velocity + args.reference_initial_time

    print(f"Running for {total_duration:.0f}s")
    while True:
        this_result = await m.set_position(
            position=math.nan, velocity=args.reference_velocity,
            kp_scale=args.reference_kp_scale,
            query=True)

        last_result = this_result

        now = time.time()
        duration = now - start_time
        if duration > total_duration:
            break

        if duration > args.reference_initial_time:
            results.append(this_result)

        mode = last_result.values[moteus.Register.MODE]
        fault = last_result.values[moteus.Register.FAULT]
        if mode == 11 or fault != 0:
            raise RuntimeError(f"controller mode {mode} / fault: {fault}")

        await asyncio.sleep(args.reference_period)

        if now - last_print > PRINT_DURATION:
            last_print = now
            print(f"r:{get_encoder(last_result, args.reference_encoder):6.3f} " +
                  f"m:{get_encoder(last_result, args.encoder_channel):6.3f}",
                  end='\r', flush=True)

    await m.set_stop()

    reference_values = [get_encoder(x, args.reference_encoder) for x in results]
    measure_values = [get_encoder(x, args.encoder_channel) for x in results]

    offset = numpy.mean([wrap_zero_one(r - m) for r, m in zip(reference_values, measure_values)])

    error_values = [-wrap_half(r - offset - m) for r, m in zip(reference_values, measure_values)]

    if args.plot or args.analyze:
        ax.yaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(xmax=1))
        ax.set_ylabel('position deviation')

        ax.plot(measure_values, error_values, '+', label='measured deviation')

    max_error = max([abs(x) for x in error_values])
    stddev_error = numpy.std(error_values)

    print()
    print(f"Max error: {max_error*100:.3f}%")
    print(f"Stddev error: {stddev_error*100:.3f}%")

    if args.analyze:
        plt.show()
        sys.exit()

    return sorted(zip(measure_values, error_values))


async def run_inertial_compensation(args, m, s, ax):
    await asyncio.sleep(1.0)

    await s.command(f"d vdq 0 {args.voltage}".encode('utf8'))
    await asyncio.sleep(1.0)

    servo_stats = await s.read_data('servo_stats')
    if servo_stats.mode != 8:
        # We special case the uncalibrated motor fault, as that is something
        if servo_stats.fault == 36:
            print("Controller is not yet calibrated.  Run:")
            print()
            print(f"  moteus_tool -t {args.target} --calibrate")
            print()
            print("before attempting encoder compensation.")

            sys.exit()

        raise RuntimeError(f"controller mode {servo_stats.mode} / fault: {servo_stats.fault}")

    # We operate by using the onboard histogram function.  The x value
    # of the histogram is always the "offset" value, which is post
    # configured offset, but pre-compensation.
    #
    # If we are either analyzing, or performing an incremental update,
    # then the y value will be 'c' for "compensated" value, which is
    # the value post-compensation.  If we are computing a value from
    # scratch, then the y value will be 'o' for "offset" value, or
    # pre-compensation.
    tap = 'o' if not (args.analyze or not args.absolute) else 'c'

    sample_channel = args.encoder_channel

    measured_velocity_values, ignored_splits = await histogram.capture_histogram(
        stream=s,
        hist_options=[
            f"xo{args.encoder_channel}",
            # Our y value uses the 'd' flag to specify numerical
            # differentiation of the sampled value.
            f"y{tap}{sample_channel}d",
        ],
        split_count=args.split_count,
        sample_time=8)

    await s.command(b"d stop")

    # The histogram is reported with no X value, as it presumes to
    # cover the full range of possible values.  Turn it into a tuple
    # of (x, y), where the x is the fractional encoder reading and the
    # y is the sampled velocity at that position.
    measured_velocities = [
        ((i + 0.5) / len(measured_velocity_values), x)
        for i, x in enumerate(measured_velocity_values)]

    ax.yaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(xmax=1))
    ax.set_ylabel('velocity deviation')

    mean_velocity = mean_value(measured_velocities)

    # Subtract the mean velocity.
    unbiased_velocities = [
        (x, ((v / mean_velocity) - 1.0))
        for x, v in measured_velocities]

    print(f"Analysis results {'(post-compensation)' if tap == 'c' else '(pre-compensation)'}:")
    print(f" mean velocity: {mean_velocity}")
    print(f" stddev: {numpy.std([v for x, v in unbiased_velocities])}")

    if args.write_unbiased:
        with open(args.write_unbiased, 'w') as out:
            for r, m in unbiased_velocities:
                print(r, m, file=out)

    ax.plot(*unpack_plot(unbiased_velocities),
            label='measured velocity deviation')

    if args.analyze:
        plt.show()

        sys.exit()

    integrated_velocities = integrate(unbiased_velocities)
    integrated_mean = mean_value(integrated_velocities)

    unbiased_integrated_velocities = [
        (x, i - integrated_mean) for x, i in integrated_velocities
    ]

    return unbiased_integrated_velocities


async def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--target', '-t', type=int, default=1)

    # If set, then no actual compensation will be performed.  Instead,
    # the on-device table for the desired encoder channel will just be
    # set to all 0s.
    parser.add_argument('--zero', action='store_true',
                        help='force the on-device table to 0s and do no other work')

    # This encoder will be the one that is compensated.
    parser.add_argument('--encoder-channel', '-c', type=int, default=0)

    # If specified, then use "reference mode" instead of "inertial
    # mode".  Implies --absolute.
    parser.add_argument('--reference-encoder', '-r', type=int, default=None)

    ##############################################3
    # Additional parameters for reference mode.
    parser.add_argument('--reference-revolutions', type=float, default=2)
    parser.add_argument('--reference-velocity', type=float, default=0.1)
    parser.add_argument('--reference-initial-time', type=float, default=1.0)
    parser.add_argument('--reference-period', type=float, default=0.005)
    parser.add_argument('--reference-kp-scale', type=float, default=1.0)


    ##############################################3
    # Additional parameters for inertial mode.

    # This may need to be much larger for gimbal or other low Kv
    # motors.
    parser.add_argument('--voltage', type=float, default=0.8)

    parser.add_argument('--split-count', type=int, default=2)


    ##############################################3
    # Common parameters for both modes.
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--analyze', action='store_true',
                        help='instead of saving results, show the effectiveness of current compensation')
    parser.add_argument('--no-store', action='store_true',
                        help='do not save values to device')
    parser.add_argument('--incremental-factor', type=float, default=0.7)

    # Adjust the existing compensation table by a fraction of the
    # observed error.
    parser.add_argument('--absolute', action='store_true')

    parser.add_argument('--write-unbiased', type=str, default=None)
    parser.add_argument('--write-integrated', type=str, default=None)

    parser.add_argument('--verbose', '-v', action='store_true')

    args = parser.parse_args()

    if args.reference_encoder is not None:
        args.absolute = True

    qr = moteus.QueryResolution()

    qr._extra[moteus.Register.ENCODER_0_POSITION] = moteus.F32
    qr._extra[moteus.Register.ENCODER_1_POSITION] = moteus.F32
    qr._extra[moteus.Register.ENCODER_2_POSITION] = moteus.F32

    m = moteus.Controller(id=args.target, query_resolution=qr)
    s = moteus.Stream(m, verbose=args.verbose)

    await s.write_message(b"tel stop")
    await s.flush_read()
    await s.command(b"d stop")

    conf_motor_position = parse_config(await s.command(b"conf enumerate motor_position"))
    COMPENSATION_SIZE = find_compensation_size(conf_motor_position)

    # Is the target encoder currently used for commutation.
    used_for_commutation = (
        int(conf_motor_position['motor_position.commutation_source']) ==
        args.encoder_channel)
    has_compensation_scale = 'motor_position.sources.0.compensation_scale' in conf_motor_position


    if used_for_commutation and args.reference_encoder is not None:
        print("cannot perform reference calibration when target is used for commutation")
        return

    if args.zero or (args.reference_encoder is not None and not args.analyze):
        # The reference calibration mode requires that the encoder
        # compensation table be all zeros to start, so we mostly
        # duplicate the zeroing logic here.
        print(f"Zeroing table for encoder {args.encoder_channel}")
        for i in range(COMPENSATION_SIZE):
            await s.command(f"conf set motor_position.sources.{args.encoder_channel}.compensation_table.{i} 0".encode('utf8'))

        if has_compensation_scale:
            await s.command(f"conf set motor_position.sources.{args.encoder_channel}.compensation_scale 0".encode('utf8'))

        if used_for_commutation:
            print("Compensated encoder used for commutation, invalidating motor calibration")
            await s.command(b"conf set motor.poles 0")

        await s.command(b'conf write')

        if args.reference_encoder is None:
            return

    print(f"Compensation table size: {COMPENSATION_SIZE}")

    # Verify the selected encoder is one we can actually compensate.
    await histogram.can_compensate_encoder(s, args.encoder_channel)

    fig, ax = plt.subplots()

    ax.set_xlabel(f'encoder {args.encoder_channel} position')

    position_min = await histogram.read_config_double(s, "servopos.position_min")
    position_max = await histogram.read_config_double(s, "servopos.position_max")

    await s.command(b'conf set servopos.position_min nan')
    await s.command(b'conf set servopos.position_max nan')

    try:
        if args.reference_encoder is not None:
            unbiased_integrated_velocities = await run_reference_compensation(args, m, s, ax)
        else:
            unbiased_integrated_velocities = await run_inertial_compensation(args, m, s, ax)
    finally:
        await s.flush_read()
        await m.set_stop()
        await s.command(
            f'conf set servopos.position_min {position_min}'.encode('utf8'))
        await s.command(
            f'conf set servopos.position_max {position_max}'.encode('utf8'))


    if args.write_integrated:
        with open(args.write_integrated, 'w') as out:
            for r, m in unbiased_integrated_velocities:
                print(r, m, file=out)

    sampled = sample(unbiased_integrated_velocities, COMPENSATION_SIZE)

    ax2 = ax.twinx()
    ax2.yaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(xmax=1))
    ax2.set_ylabel('position deviation')

    if args.plot:
        ax2.plot(*unpack_plot(unbiased_integrated_velocities), label='integrated position deviation', color='red')
        ax2.plot(*unpack_plot(sampled), label='downsampled position deviation', color='green')

    if not args.no_store:
        print("Saving new encoder compensation to device")

        scale = 1.0
        if has_compensation_scale:
            scale = float(conf_motor_position[f"motor_position.sources.{args.encoder_channel}.compensation_scale"]) / 127.0

        table = [float(conf_motor_position[f"motor_position.sources.{args.encoder_channel}.compensation_table.{i}"]) * scale
                 for i in range(COMPENSATION_SIZE)]

        for i, x in enumerate(sampled):
            update = -x[1]
            if not args.absolute:
                table[i] += args.incremental_factor * update
            else:
                table[i] = update

        if has_compensation_scale:
            new_scale = max([abs(x) for x in table])
            for i in range(COMPENSATION_SIZE):
                int_value = int(127 * table[i] / new_scale)
                await s.command(f"conf set motor_position.sources.{args.encoder_channel}.compensation_table.{i} {int_value}".encode('utf8'))
            await s.command(f"conf set motor_position.sources.{args.encoder_channel}.compensation_scale {new_scale}".encode('utf8'))
        else:
            for i in range(COMPENSATION_SIZE):
                await s.command(f"conf set motor_position.sources.{args.encoder_channel}.compensation_table.{i} {table[i]}".encode('utf8'))

        if used_for_commutation:
            print("Compensated encoder used for commutation, invalidating motor calibration")
            await s.command(b"conf set motor.poles 0")

        await s.command(b'conf write')
    else:
        print("--no-store specified, not saving values to device")

    if args.plot:
        ax.legend(loc="upper left")
        ax2.legend(loc="upper right")

        plt.show()

if __name__ == '__main__':
    asyncio.run(main())
