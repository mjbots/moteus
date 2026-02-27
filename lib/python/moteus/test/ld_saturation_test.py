#!/usr/bin/env python3

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

"""Tests for ld_saturation analysis module."""

import json
import os
import unittest

from moteus import ld_saturation


TESTDATA_DIR = os.path.join(
    os.path.dirname(__file__), 'testdata')


def _load_test_data(name):
    """Load test data and return (avg_data, params, expected)."""
    path = os.path.join(TESTDATA_DIR, f'ld_{name}.json')
    with open(path) as f:
        data = json.load(f)
    raw = data['raw']
    avg_data = {}
    for d_A_str, points in raw.items():
        d_A = float(d_A_str)
        avg_data[d_A] = [(p['omega'], p['y']) for p in points]
    return avg_data, data['params'], data


class TestMedian(unittest.TestCase):
    def test_odd(self):
        self.assertEqual(ld_saturation.median([3, 1, 2]), 2)

    def test_even(self):
        self.assertEqual(ld_saturation.median([4, 1, 3, 2]), 2.5)

    def test_single(self):
        self.assertEqual(ld_saturation.median([42]), 42)

    def test_empty_raises(self):
        with self.assertRaises(ValueError):
            ld_saturation.median([])


class TestPerLevelRegression(unittest.TestCase):
    def test_mj5208(self):
        avg_data, params, expected = _load_test_data('mj5208')
        slope_data, regression_data = (
            ld_saturation.per_level_regression(avg_data))

        # All 7 d_A levels should produce slopes.
        self.assertEqual(len(slope_data), 7)
        self.assertIn(0.0, slope_data)

        # Check slopes match expected regressions.
        for d_A_str, reg in expected['regressions'].items():
            d_A = float(d_A_str)
            self.assertAlmostEqual(
                slope_data[d_A], reg['slope'], places=15)
            self.assertAlmostEqual(
                regression_data[d_A]['r_squared'],
                reg['r_squared'], places=15)


class TestRejectOutliers(unittest.TestCase):
    def test_mj5208_no_outliers(self):
        avg_data, _, _ = _load_test_data('mj5208')
        slope_data, _ = ld_saturation.per_level_regression(avg_data)
        clean_d_A = ld_saturation.reject_outliers(slope_data)

        # mj5208 has no outliers: all 6 nonzero levels retained.
        nonzero = [d for d in sorted(slope_data.keys()) if d != 0.0]
        self.assertEqual(clean_d_A, nonzero)

    def test_hoverboard_rejects_outlier(self):
        avg_data, _, _ = _load_test_data('hoverboard')
        slope_data, _ = ld_saturation.per_level_regression(avg_data)
        clean_d_A = ld_saturation.reject_outliers(slope_data)

        # Hoverboard rejects d_A=-2.28... (the closest to zero).
        nonzero = [d for d in sorted(slope_data.keys()) if d != 0.0]
        self.assertEqual(len(clean_d_A), len(nonzero) - 1)
        rejected = set(nonzero) - set(clean_d_A)
        self.assertEqual(len(rejected), 1)
        # The rejected level is the one closest to zero.
        rejected_d_A = rejected.pop()
        self.assertAlmostEqual(
            rejected_d_A, -2.283574025657326, places=10)

    def test_no_slope_zero(self):
        result = ld_saturation.reject_outliers({-5.0: 0.001})
        self.assertIsNone(result)


class TestGlobalFit(unittest.TestCase):
    def test_mj5208(self):
        avg_data, _, _ = _load_test_data('mj5208')
        slope_data, _ = ld_saturation.per_level_regression(avg_data)
        clean_d_A = ld_saturation.reject_outliers(slope_data)
        valid_d_A = sorted([0.0] + clean_d_A)

        result = ld_saturation.global_fit(avg_data, valid_d_A)
        self.assertIsNotNone(result)
        lm, B, C = result
        self.assertAlmostEqual(
            lm, 0.0025517617575494082, places=15)
        self.assertAlmostEqual(
            B, 2.8459612307808206e-05, places=20)
        self.assertAlmostEqual(
            C, -1.1680501066905844e-06, places=21)


class TestAnalyze(unittest.TestCase):
    def _run_motor(self, name):
        avg_data, params, expected = _load_test_data(name)
        result = ld_saturation.analyze(avg_data, params)
        self.assertIsNotNone(result)

        B, C = result
        exp_fit = expected['fit']
        self.assertEqual(
            B, exp_fit['B'],
            f"{name} B: {B!r} != {exp_fit['B']!r}")
        self.assertEqual(
            C, exp_fit['C'],
            f"{name} C: {C!r} != {exp_fit['C']!r}")

        return result

    def test_analyze_mj5208(self):
        self._run_motor('mj5208')

    def test_analyze_mad8318(self):
        self._run_motor('mad8318')

    def test_analyze_be8108(self):
        self._run_motor('be8108')

    def test_analyze_custom(self):
        self._run_motor('custom')

    def test_analyze_hoverboard(self):
        self._run_motor('hoverboard')

    def test_analyze_spindle(self):
        self._run_motor('spindle')

    def test_analyze_flipsky6374(self):
        self._run_motor('flipsky6374')

    def test_insufficient_data(self):
        avg_data = {
            0.0: [(100.0, 0.25), (200.0, 0.50)],
            -5.0: [(100.0, 0.20), (200.0, 0.45)],
        }
        params = {'R': 0.05, 'poles': 14}
        result = ld_saturation.analyze(avg_data, params)
        self.assertIsNone(result)

    def test_no_d_a_zero(self):
        # Enough total points but d_A=0 has < 3 points so no slope.
        avg_data = {
            0.0: [(100.0, 0.25)],
            -5.0: [(100.0, 0.20), (200.0, 0.40),
                   (300.0, 0.60), (400.0, 0.80)],
            -10.0: [(100.0, 0.15), (200.0, 0.30),
                    (300.0, 0.45), (400.0, 0.60)],
            -15.0: [(100.0, 0.10), (200.0, 0.20)],
        }
        params = {'R': 0.05, 'poles': 14}
        result = ld_saturation.analyze(avg_data, params)
        self.assertIsNone(result)


if __name__ == '__main__':
    unittest.main()
