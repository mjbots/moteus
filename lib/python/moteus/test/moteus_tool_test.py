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


# The 1.0.0 gate-drive defaults for each board, as written by firmware
# <= 0x010000.  (family, hwrev) identifies the board.
OLD_DEFAULTS = {
    'r4.5': (0, 5, {
        'idrivep_hs_ma': 370, 'idriven_hs_ma': 740,
        'idrivep_ls_ma': 370, 'idriven_ls_ma': 740,
        'ocp_deg_us': 4, 'vds_lvl_mv': 260,
    }),
    'r4.8': (0, 7, {
        'idrivep_hs_ma': 50, 'idriven_hs_ma': 100,
        'idrivep_ls_ma': 50, 'idriven_ls_ma': 100,
        'ocp_deg_us': 4, 'vds_lvl_mv': 450,
    }),
    'r4.11': (0, 8, {
        'idrivep_hs_ma': 100, 'idriven_hs_ma': 200,
        'idrivep_ls_ma': 100, 'idriven_ls_ma': 200,
        'ocp_deg_us': 4, 'vds_lvl_mv': 700,
    }),
    'n1': (1, 0, {
        'idrivep_hs_ma': 150, 'idriven_hs_ma': 300,
        'idrivep_ls_ma': 150, 'idriven_ls_ma': 300,
        'ocp_deg_us': 4, 'vds_lvl_mv': 700,
    }),
    'c1': (2, 0, {
        'idrivep_hs_ma': 80, 'idriven_hs_ma': 60,
        'idrivep_ls_ma': 60, 'idriven_ls_ma': 20,
        'ocp_deg_us': 4, 'vds_lvl_mv': 700,
    }),
    'x1': (3, 0, {
        'idrivep_hs_ma': 300, 'idriven_hs_ma': 200,
        'idrivep_ls_ma': 850, 'idriven_ls_ma': 600,
        'ocp_deg_us': 4, 'vds_lvl_mv': 700,
    }),
}


def make_config(values):
    lines = [f'drv8323_conf.{k} {v}' for k, v in values.items()]
    # Include an unrelated key to confirm it passes through untouched.
    lines.append('id.id 1')
    return '\n'.join(lines).encode('utf8')


def parse_config(data):
    result = {}
    for line in data.split(b'\n'):
        if b' ' not in line:
            continue
        key, value = line.split(b' ')
        result[key.decode('utf8')] = value.decode('utf8')
    return result


def register_code(family, hwrev, version, field, value):
    '''A faithful model of the firmware reg3/reg4/reg5 gate-drive code
    selection, so the test can assert the migration preserves the code
    actually written to the chip.'''
    if version >= 0x010100:
        drv8323 = (family == 0 and hwrev <= 6) or (family == 2)
    else:
        # firmware <= 0x010000 (post-c1-commit) had n1 on the drv8323
        # path and c1 on the drv8353 path.
        drv8323 = (family == 0 and hwrev <= 6) or (family == 1)

    if field in ('idrivep_hs_ma', 'idrivep_ls_ma'):
        table = (mt.IDRIVEP_TABLE_DRV8323 if drv8323
                 else mt.IDRIVEP_TABLE_DRV8353)
    elif field == 'idriven_hs_ma':
        table = (mt.IDRIVEN_TABLE_DRV8323 if drv8323
                 else mt.IDRIVEN_TABLE_DRV8353)
    elif field == 'idriven_ls_ma':
        if drv8323:
            table = mt.IDRIVEN_TABLE_DRV8323
        elif version >= 0x010100:
            table = mt.IDRIVEN_TABLE_DRV8353
        else:
            # The reg4 bug (Defect A): idriven_ls used the IDRIVEP table.
            table = mt.IDRIVEP_TABLE_DRV8353
    elif field == 'ocp_deg_us':
        table = (mt.DEGLITCH_TABLE_DRV8323 if drv8323
                 else mt.DEGLITCH_TABLE_DRV8353)
    elif field == 'vds_lvl_mv':
        table = (mt.VDS_LVL_TABLE_DRV8323 if drv8323
                 else mt.VDS_LVL_TABLE_DRV8353)
    else:
        raise KeyError(field)

    return mt._map_choice(table, int(value))


class FirmwareUpgradeGateDriveTest(unittest.TestCase):
    def _upgrade(self, family, hwrev, values):
        upgrade = mt.FirmwareUpgrade(0x010000, 0x010100, family, hwrev)
        return parse_config(upgrade.fix_config(make_config(values)))

    def _downgrade(self, family, hwrev, values):
        downgrade = mt.FirmwareUpgrade(0x010100, 0x010000, family, hwrev)
        return parse_config(downgrade.fix_config(make_config(values)))

    def test_upgrade_exact_values(self):
        # The migrated value of each board's 1.0.0 default must equal the
        # new 1.1.0 firmware default.
        expected = {
            'r4.5': {'idriven_ls_ma': '740'},   # unchanged (drv8323 path)
            'r4.8': {'idriven_ls_ma': '200'},
            'r4.11': {'idriven_ls_ma': '600'},
            # n1 was untouched, so it adopts the new conservative defaults.
            'n1': {
                'idrivep_hs_ma': '150', 'idriven_hs_ma': '200',
                'idrivep_ls_ma': '150', 'idriven_ls_ma': '100',
            },
            'c1': {
                'idrivep_hs_ma': '60', 'idriven_hs_ma': '20',
                'idrivep_ls_ma': '60', 'idriven_ls_ma': '20',
                # ocp_deg_us is intentionally left at 4 (not preserved).
                'ocp_deg_us': '4', 'vds_lvl_mv': '940',
            },
            'x1': {'idriven_ls_ma': '1200'},
        }
        for name, (family, hwrev, values) in OLD_DEFAULTS.items():
            result = self._upgrade(family, hwrev, values)
            for field, want in expected[name].items():
                self.assertEqual(
                    result[f'drv8323_conf.{field}'], want,
                    f'{name}.{field}')

    def test_upgrade_preserves_or_restores_register_code(self):
        # c1/r4.8/r4.11/x1: the chip register code is preserved across
        # the upgrade.  (n1 deliberately adopts new defaults when untouched
        # and is covered by the dedicated n1 tests below.)
        for name, (family, hwrev, values) in OLD_DEFAULTS.items():
            if name == 'n1':
                continue
            result = self._upgrade(family, hwrev, values)
            for field, old_value in values.items():
                # c1's deglitch is intentionally not preserved; its
                # register code changes from the drv8353 to the drv8323
                # encoding of the same 4 setting.
                if name == 'c1' and field == 'ocp_deg_us':
                    continue
                new_value = int(result[f'drv8323_conf.{field}'])
                new_code = register_code(family, hwrev, 0x010100,
                                         field, new_value)
                old_code = register_code(family, hwrev, 0x010000,
                                         field, old_value)
                self.assertEqual(new_code, old_code, f'{name}.{field}')

    # The 0x010100 n1 defaults, as written by the new firmware on the
    # (corrected) DRV8353 path.
    N1_NEW = {
        'idrivep_hs_ma': 150, 'idriven_hs_ma': 200,
        'idrivep_ls_ma': 150, 'idriven_ls_ma': 100,
        'ocp_deg_us': 4, 'vds_lvl_mv': 700,
    }

    def test_n1_untouched_upgrade_adopts_new_defaults(self):
        family, hwrev, values = OLD_DEFAULTS['n1']
        result = self._upgrade(family, hwrev, values)
        for field in ('idrivep_hs_ma', 'idriven_hs_ma',
                      'idrivep_ls_ma', 'idriven_ls_ma'):
            self.assertEqual(int(result[f'drv8323_conf.{field}']),
                             self.N1_NEW[field], field)

    def test_n1_untouched_downgrade_restores_old_defaults(self):
        family, hwrev, _ = OLD_DEFAULTS['n1']
        result = self._downgrade(family, hwrev, self.N1_NEW)
        old = OLD_DEFAULTS['n1'][2]
        for field in ('idrivep_hs_ma', 'idriven_hs_ma',
                      'idrivep_ls_ma', 'idriven_ls_ma'):
            self.assertEqual(int(result[f'drv8323_conf.{field}']),
                             old[field], field)

    def test_n1_modified_preserves_register_code(self):
        # When the n1 gate drive has been changed from the defaults, the
        # migration preserves the register code across the path change
        # rather than jumping to the new defaults.
        family, hwrev, _ = OLD_DEFAULTS['n1']
        modified = {
            'idrivep_hs_ma': 820, 'idriven_hs_ma': 1640,
            'idrivep_ls_ma': 440, 'idriven_ls_ma': 880,
            'ocp_deg_us': 4, 'vds_lvl_mv': 700,
        }
        gate_fields = ('idrivep_hs_ma', 'idriven_hs_ma',
                       'idrivep_ls_ma', 'idriven_ls_ma')
        result = self._upgrade(family, hwrev, modified)
        for field in gate_fields:
            new_value = int(result[f'drv8323_conf.{field}'])
            self.assertEqual(
                register_code(family, hwrev, 0x010100, field, new_value),
                register_code(family, hwrev, 0x010000, field, modified[field]),
                field)
        # ...and it round-trips back to the original code on downgrade.
        upgraded = {k[len('drv8323_conf.'):]: int(v)
                    for k, v in result.items()
                    if k.startswith('drv8323_conf.')}
        down = self._downgrade(family, hwrev, upgraded)
        for field in gate_fields:
            down_value = int(down[f'drv8323_conf.{field}'])
            self.assertEqual(
                register_code(family, hwrev, 0x010000, field, down_value),
                register_code(family, hwrev, 0x010000, field, modified[field]),
                field)

    def test_downgrade_round_trips_register_code(self):
        # Upgrade then downgrade must reproduce the original 1.0.0
        # register code for every field.
        for name, (family, hwrev, values) in OLD_DEFAULTS.items():
            upgraded = self._upgrade(family, hwrev, values)
            upgraded_values = {
                k[len('drv8323_conf.'):]: int(v)
                for k, v in upgraded.items()
                if k.startswith('drv8323_conf.')
            }
            downgraded = self._downgrade(family, hwrev, upgraded_values)
            for field, old_value in values.items():
                down_value = int(downgraded[f'drv8323_conf.{field}'])
                self.assertEqual(
                    register_code(family, hwrev, 0x010000, field, down_value),
                    register_code(family, hwrev, 0x010000, field, old_value),
                    f'{name}.{field}')

    def test_unrelated_key_passthrough(self):
        family, hwrev, values = OLD_DEFAULTS['c1']
        result = self._upgrade(family, hwrev, values)
        self.assertEqual(result['id.id'], '1')

    def test_missing_hwrev_assumes_drv8323(self):
        # A family 0 board reporting no hwrev is only reachable from old
        # firmware, which ran on early (drv8323-path) r4.x boards.  So we
        # assume the drv8323 path and leave idriven_ls_ma untouched.
        upgrade = mt.FirmwareUpgrade(0x010000, 0x010100, 0, None)
        result = parse_config(upgrade.fix_config(
            make_config({'idriven_ls_ma': 200})))
        self.assertEqual(result['drv8323_conf.idriven_ls_ma'], '200')


# Minimal config covering the keys the 0x0109 -> 0x010100 upgrade chain
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
