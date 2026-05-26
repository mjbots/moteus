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

import unittest

import moteus.moteus_tool as mt


def parse_config(data):
    result = {}
    for line in data.split(b'\n'):
        if b' ' not in line:
            continue
        key, value = line.split(b' ')
        result[key.decode('utf8')] = value.decode('utf8')
    return result


# Minimal config covering the keys the 0x0109 -> 0x010e upgrade chain
# dereferences without a default.
_CHAIN_CONFIG = {
    'motor.v_per_hz': '0.0978',
    'servo.pwm_rate_hz': '15000',
    'servo.flux_brake_min_voltage': '50.0',
    'servo.max_voltage': '56.0',
    'servo.derate_temperature': '90.0',
    'servo.fault_temperature': '105.0',
}


def _chain_config(**overrides):
    cfg = dict(_CHAIN_CONFIG, **overrides)
    return '\n'.join(f'{k} {v}' for k, v in cfg.items()).encode('utf8')


class MaxPowerUpgradeTest(unittest.TestCase):
    """0x010a max_power_W migration -- regression for an x1 (family 3)
    KeyError because the family->default table lacked family 3."""

    def _upgrade(self, family, maxpower):
        # 0x010a is where max_power_W gained per-family defaults; testing the
        # 0x0109 -> 0x010e chain exercises the fix independently of any later
        # ABI version.
        up = mt.FirmwareUpgrade(0x0109, 0x010e, board_family=family)
        return parse_config(up.fix_config(
            _chain_config(**{'servo.max_power_W': maxpower})))

    def test_x1_nonfinite_power(self):
        # x1 defaulted max_power_W to the board limit (non-finite) -> NaN.
        out = self._upgrade(3, 'nan')          # must not raise
        self.assertEqual(out['servo.max_power_W'], 'nan')

    def test_x1_finite_power_scaled(self):
        out = self._upgrade(3, '300.0')
        self.assertAlmostEqual(float(out['servo.max_power_W']),
                               300.0 * 15000 / 40000, places=3)

    def test_known_family_default_to_nan(self):
        # A family-0 value at its old 450W default still maps to NaN.
        out = self._upgrade(0, '450.0')
        self.assertEqual(out['servo.max_power_W'], 'nan')


if __name__ == '__main__':
    unittest.main()
