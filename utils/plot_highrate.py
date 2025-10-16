#!/usr/bin/python3 -B

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
from dataclasses import dataclass
import matplotlib.pyplot as plt
import numpy as np
import struct


EMIT_DEBUG = [
    (1 << 0, 'aux1_spi', 1 / 4, 'counts'),
    (1 << 1, 'servo_stats.velocity', 100 / 32767, 'A'),
    (1 << 2, 'servo_stats.d_A', 100 / 32767, 'A'),
    (1 << 3, 'servo_stats.q_A', 100 / 32767, 'A'),
    (1 << 4, 'servo_stats.adc_cur1_raw', 1, 'counts'),
    (1 << 5, 'servo_stats.adc_cur2_raw', 1, 'counts'),
    (1 << 6, 'servo_stats.adc_cur3_raw', 1, 'counts'),
    (1 << 7, 'servo_stats.cur1_A', 100 / 32767, 'A'),
    (1 << 8, 'servo_stats.cur2_A', 100 / 32767, 'A'),
    (1 << 9, 'servo_stats.cur3_A', 100 / 32767, 'A'),
    (1 << 10, 'servo_control.torque_Nm', 30 / 32767, 'Nm'),
    (1 << 11, 'motor_position.sources[0].raw', 1, 'counts'),
    (1 << 12, 'motor_position.sources[1].raw', 1, 'counts'),
    (1 << 13, 'motor_position.sources[2].raw', 1, 'counts'),
    (1 << 14, 'servo_stats.final_timer', 1, 'counts'),
    (1 << 15, 'servo_control.d_V', 64 / 32767, 'V'),
    (1 << 16, 'servo_control.q_V', 64 / 32767, 'V'),
    (1 << 17, 'servo_control.pwm.a', 1 / 32767, 'duty'),
    (1 << 18, 'servo_control.pwm.b', 1 / 32767, 'duty'),
    (1 << 19, 'servo_control.pwm.c', 1 / 32767, 'duty'),
    (1 << 20, 'servo_stats.torque_Nm', 30 / 32767, 'Nm'),
    (1 << 21, 'servo_stats.power_W', 3000 / 32767, 'W'),
]


class PlotConfig:
    struct = None
    fields = []

    @staticmethod
    def parse_emit_debug(emit_debug):
        result = PlotConfig()

        structdef = ''
        fields = []

        for emit_debug_option in EMIT_DEBUG:
            if emit_debug_option[0] & emit_debug != 0:
                structdef += 'h'
                fields.append(emit_debug_option)

        result.struct = struct.Struct('<b' + structdef)
        result.fields = fields

        return result

    @staticmethod
    def parse_struct(structdef):
        result = PlotConfig()
        result.struct = struct.Struct('<b' + structdef)

        return result

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', type=str)
    parser.add_argument('--pwm-hz', default=30000, type=int)

    parser.add_argument('--struct', '-s', default='hhh')

    # If specified, this takes precedence over the --struct option.
    parser.add_argument('--emit-debug', '-e', default=None, type=int,
                        help="servo.emit_debug used to generate data")

    parser.add_argument('--plot-mode', default='subplots',
                        choices=['subplots', 'twinx'],
                        help="plotting mode: twinx (multiple y-axes on one plot) "
                             "or subplots (separate subplot per unit)")

    args = parser.parse_args()

    if args.emit_debug:
        plot_config = PlotConfig.parse_emit_debug(args.emit_debug)
    else:
        plot_config = PlotConfig.parse_struct(args.struct)

    s = plot_config.struct

    data = open(args.file, "rb").read()

    for i in range(s.size + 1):
        maybe_all_header = data[slice(i, s.size * 20, s.size)]
        if maybe_all_header == bytes([0x5a]) * len(maybe_all_header):
            print(f"skipping first {i} bytes")
            data = data[i:]
            break
    else:
        raise RuntimeError("Could not locate header in highrate data")

    parsed = [s.unpack(data[i:i+s.size])
              for i in range(0, len(data) - s.size, s.size)]

    def fmt_tuple(l):
        return ','.join([f"{x:04x}" for x in l])

    for i, row in enumerate(parsed):
        if row[0] != 0x5a:
            print(f"Desynchronization at row {i}")
            print(fmt_tuple(parsed[i-1]))
            print(fmt_tuple(parsed[i]))

    size = len(parsed)
    xvals = np.arange(0, size / args.pwm_hz, 1.0 / args.pwm_hz)

    if args.emit_debug:
        # Scale data and group by units
        scaled_data = {}
        units_map = {}

        for field_idx, field_info in enumerate(plot_config.fields):
            # field_info = (bit_mask, name, scale, unit)
            _, name, scale, unit = field_info
            # Data index is field_idx + 1 (skip header byte at index 0)
            data_idx = field_idx + 1

            # Apply scaling
            raw_values = np.array([x[data_idx] for x in parsed])
            scaled_values = raw_values * scale

            # Store scaled data and track units
            scaled_data[name] = (scaled_values, unit)
            if unit not in units_map:
                units_map[unit] = []
            units_map[unit].append(name)

        # Plot based on mode
        if args.plot_mode == 'twinx':
            # Single plot with multiple y-axes
            fig, ax1 = plt.subplots()
            axes = {}
            colors = plt.cm.tab10.colors
            color_idx = 0

            for unit_idx, (unit, field_names) in enumerate(units_map.items()):
                # Create axis for this unit
                if unit_idx == 0:
                    ax = ax1
                else:
                    ax = ax1.twinx()
                    # Offset additional axes
                    if unit_idx > 1:
                        ax.spines['right'].set_position(('outward', 60 * (unit_idx - 1)))

                axes[unit] = ax
                ax.set_ylabel(unit)

                # Plot all fields with this unit
                for field_name in field_names:
                    scaled_values, _ = scaled_data[field_name]
                    color = colors[color_idx % len(colors)]
                    ax.plot(xvals, scaled_values, label=field_name, color=color)
                    color_idx += 1

            ax1.set_xlabel('Time (s)')

            # Combine all legends
            lines = []
            labels = []
            for ax in [ax1] + [axes[u] for u in list(units_map.keys())[1:]]:
                ax_lines, ax_labels = ax.get_legend_handles_labels()
                lines.extend(ax_lines)
                labels.extend(ax_labels)
            ax1.legend(lines, labels, loc='best')

        else:  # subplots mode
            # Separate subplot for each unit
            num_units = len(units_map)
            fig, axes = plt.subplots(num_units, 1, figsize=(10, 3 * num_units),
                                     sharex=True)

            # Handle case of single unit
            if num_units == 1:
                axes = [axes]

            for ax, (unit, field_names) in zip(axes, units_map.items()):
                for field_name in field_names:
                    scaled_values, _ = scaled_data[field_name]
                    ax.plot(xvals, scaled_values, label=field_name)

                ax.set_ylabel(unit)
                ax.legend(loc='best')
                ax.grid(True, alpha=0.3)

            axes[-1].set_xlabel('Time (s)')

        plt.tight_layout()
        plt.show()

    else:
        # Backward compatibility: simple plotting without scaling
        for i, _ in enumerate(parsed[0]):
            if i == 0:
                continue

            plt.plot(xvals, [x[i] for x in parsed],
                     label=f'item {i}')

        plt.legend()
        plt.show()


if __name__ == '__main__':
    main()
