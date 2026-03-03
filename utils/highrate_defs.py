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

"""Shared high-rate debug field definitions and parser.

Used by plot_highrate.py and fw_hil_test.py.
"""

import struct


EMIT_DEBUG = [
    (1 << 0, 'aux1_spi', 1 / 4, 'counts'),
    (1 << 1, 'servo_stats.velocity', 100 / 32767, 'rev/s'),
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

    def parse_data(self, data):
        """Parse raw binary data into list of tuples.

        Finds the sync byte (0x5a) alignment that matches the most
        records, then returns only records with a valid sync byte.
        This tolerates occasional dropped bytes in the serial stream.
        """
        s = self.struct

        # Find the offset with the most sync bytes across the file.
        best_offset = 0
        best_count = 0
        for i in range(s.size):
            count = sum(1 for j in range(i, len(data), s.size)
                        if data[j] == 0x5a)
            if count > best_count:
                best_count = count
                best_offset = i

        if best_count < 10:
            raise RuntimeError("Could not locate header in highrate data")

        data = data[best_offset:]

        # Only include records whose sync byte is valid.
        return [s.unpack(data[i:i+s.size])
                for i in range(0, len(data) - s.size, s.size)
                if data[i] == 0x5a]

    def scaled_fields(self, parsed):
        """Return dict mapping field name to scaled numpy-compatible list.

        ``parsed`` is the output of ``parse_data``.
        """
        result = {}
        for field_idx, field_info in enumerate(self.fields):
            _, name, scale, unit = field_info
            data_idx = field_idx + 1
            values = [x[data_idx] * scale for x in parsed]
            result[name] = values
        return result
