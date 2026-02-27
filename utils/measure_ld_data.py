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

"""Standalone L_d saturation data collection tool.

Collects raw voltage-mode V_d injection data for L_d saturation
analysis and writes it in the test fixture JSON format consumed by
ld_saturation_test.py.  Uses the ld_measure and ld_saturation
modules directly, without running the full calibration flow.

Example usage:

    utils/measure_ld_data.py --target 1 \\
        --resistance 0.049 \\
        --v-per-hz 0.109 \\
        --kv-cal-voltage 1.72 \\
        -o ld_mymotor.json
"""

import argparse
import asyncio
import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, '..', 'lib', 'python'))

import moteus
from moteus import moteus_tool, ld_measure, ld_saturation


async def main():
    parser = argparse.ArgumentParser(
        description='Collect L_d saturation data and write test JSON.')

    parser.add_argument(
        '-t', '--target', type=int, default=1,
        help='CAN target ID (default: 1)')
    parser.add_argument('--can-prefix', type=int, default=0)
    parser.add_argument('-v', '--verbose', action='store_true')

    parser.add_argument(
        '--resistance', type=float, required=True,
        help='winding resistance in ohms')
    parser.add_argument(
        '--v-per-hz', type=float, required=True,
        help='volts per electrical Hz (from Kv calibration)')
    parser.add_argument(
        '--kv-cal-voltage', type=float, required=True,
        help='Kv calibration voltage')

    parser.add_argument(
        '--poles', type=int, default=None,
        help='motor poles (default: read from device)')
    parser.add_argument(
        '--unwrapped-position-scale', type=float, default=None,
        help='unwrapped position scale (default: read from device)')

    parser.add_argument(
        '--power-factor', type=float, default=6.0,
        help='multiple of motor-power for d_A range (default: 6.0)')
    parser.add_argument(
        '--voltage-factor', type=float, default=1.0,
        help='max multiple of kv-cal-voltage for speed sweep '
        '(default: 1.0)')
    parser.add_argument(
        '--motor-power', type=float, default=7.5,
        help='motor power in watts (default: 7.5)')

    parser.add_argument(
        '-o', '--output', type=str, required=True,
        help='output JSON file path')

    moteus.make_transport_args(parser)
    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)

    stream_args = argparse.Namespace(
        can_prefix=args.can_prefix,
        verbose=args.verbose,
        diagnostic_channel=1)
    stream = moteus_tool.Stream(stream_args, args.target, transport)

    await stream.write_message("tel stop")
    await stream.flush_read()

    # Resolve motor.poles and unwrapped_position_scale.
    if args.poles is not None:
        motor_poles = args.poles
    else:
        motor_poles = await stream.read_config_int("motor.poles")
    if motor_poles <= 0:
        print("ERROR: motor.poles not set, use --poles")
        return

    if args.unwrapped_position_scale is not None:
        unwrapped_position_scale = args.unwrapped_position_scale
    elif await stream.is_config_supported(
            "motor.unwrapped_position_scale"):
        unwrapped_position_scale = await stream.read_config_double(
            "motor.unwrapped_position_scale")
    elif await stream.is_config_supported(
            "motor_position.rotor_to_output_ratio"):
        unwrapped_position_scale = await stream.read_config_double(
            "motor_position.rotor_to_output_ratio")
    else:
        print("ERROR: cannot read unwrapped_position_scale, "
              "use --unwrapped-position-scale")
        return

    params = ld_measure.LdSweepParams(
        winding_resistance=args.resistance,
        unwrapped_position_scale=unwrapped_position_scale,
        pp=motor_poles / 2.0,
        v_per_hz=args.v_per_hz,
        kv_cal_voltage=args.kv_cal_voltage,
        motor_power=args.motor_power,
        power_factor=args.power_factor,
        voltage_factor=args.voltage_factor,
    )
    plan = ld_measure.compute_sweep_plan(params)

    print(f"\nMeasuring L_d vs d_A (voltage-mode method)")
    print(f"  max |d_A| = {plan.max_d_A_abs:.1f} A, "
          f"{len(plan.all_d_A) - 1} levels + d_A=0 ref, "
          f"{params.n_speeds} speeds")
    print(f"  R = {params.winding_resistance:.4f} ohm, "
          f"poles = {motor_poles}")

    avg_data = await ld_measure.collect_sweep_data(stream, params, plan)

    # Build raw data in test fixture format.
    raw = {}
    for d_A in sorted(avg_data.keys()):
        pts = avg_data[d_A]
        if pts:
            raw[str(d_A)] = [
                {'omega': omega, 'y': y} for omega, y in pts]

    # Run analysis pipeline once; LdFitDetails exposes everything
    # needed for the fixture format.
    fit = {}
    regressions = {}
    details = ld_saturation.analyze_detailed(avg_data, {
        'R': params.winding_resistance,
        'poles': motor_poles,
    })
    if details is not None:
        fit = {
            'lambda_m': details.lambda_m,
            'B': details.B,
            'C': details.C,
        }
        for d_A, reg in details.regression_data.items():
            regressions[str(d_A)] = reg

    output = {
        'fit': fit,
        'regressions': regressions,
        'raw': raw,
        'params': {
            'R': params.winding_resistance,
            'poles': motor_poles,
            'v_per_hz': params.v_per_hz,
        },
    }

    with open(args.output, 'w') as f:
        json.dump(output, f, indent=2)
        f.write('\n')

    print(f"\nWrote {args.output}")
    total_pts = sum(len(v) for v in avg_data.values())
    print(f"  {total_pts} data points across "
          f"{sum(1 for v in avg_data.values() if v)} levels")
    if fit:
        print(f"  lambda_m = {fit['lambda_m']:.6f}")
        print(f"  B = {fit['B']*1e6:.1f} uH, "
              f"C = {fit['C']*1e6:.2f} uH/A")


asyncio.run(main())
