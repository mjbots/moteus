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

"""Test that the mpat CLI tool runs and produces output."""

import json
import subprocess
import unittest

from bazel_tools.tools.python.runfiles import runfiles


class MpatTest(unittest.TestCase):
    def setUp(self):
        self.runfiles = runfiles.Create()
        self.mpat = self.runfiles.Rlocation(
            "com_github_mjbots_moteus/utils/mpat")

    def run_mpat(self, *args):
        """Run the mpat CLI with the given arguments and return the
        subprocess result."""
        result = subprocess.run(
            [self.mpat] + list(args),
            capture_output=True,
            text=True
        )
        return result

    def run_mpat_json(self, *args):
        """Run the mpat CLI with --json and return parsed JSON output.

        Returns None if the analysis returned null."""
        result = self.run_mpat('--json', *args)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        return data

    def assertFieldAlmostEqual(self, data, field, expected, places=1):
        """Assert that a field in the JSON output is approximately equal
        to the expected value."""
        self.assertIsNotNone(data, "Analysis returned null")
        self.assertIn(field, data)
        self.assertAlmostEqual(data[field], expected, places=places,
                               msg=f"{field}: {data[field]} != {expected}")

    def assertFieldGreater(self, data, field, value):
        """Assert that a field in the JSON output is greater than a
        value."""
        self.assertIsNotNone(data, "Analysis returned null")
        self.assertIn(field, data)
        self.assertGreater(data[field], value,
                           msg=f"{field}: {data[field]} not > {value}")

    def assertFieldLess(self, data, field, value):
        """Assert that a field in the JSON output is less than a
        value."""
        self.assertIsNotNone(data, "Analysis returned null")
        self.assertIn(field, data)
        self.assertLess(data[field], value,
                        msg=f"{field}: {data[field]} not < {value}")

    def assertFieldEqual(self, data, field, expected):
        """Assert that a field in the JSON output is exactly equal to
        the expected value."""
        self.assertIsNotNone(data, "Analysis returned null")
        self.assertIn(field, data)
        self.assertEqual(data[field], expected,
                         msg=f"{field}: {data[field]} != {expected}")

    def test_help(self):
        """Test that --help produces output."""
        result = self.run_mpat("--help")
        self.assertEqual(result.returncode, 0)
        self.assertIn("moteus Performance Analysis Tool", result.stdout)
        self.assertIn("--analysis", result.stdout)

    def test_max_torque(self):
        """Test max_torque analysis produces valid output."""
        result = self.run_mpat(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "48",
            "--velocity", "10")
        self.assertEqual(result.returncode, 0)
        self.assertIn("Nm", result.stdout)

    def test_json_output(self):
        """Test JSON output format."""
        data = self.run_mpat_json(
            "--analysis", "max_current",
            "--controller", "moteus-x1",
            "--voltage", "48")
        self.assertIsNotNone(data)
        self.assertIn("phase_current", data)

    def test_fw_off_max_velocity(self):
        """FW off: max_velocity is back-EMF limited."""
        data = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--torque", "0",
            "--field_weakening", "off",
            "--motor_cooling", "max")
        self.assertFieldAlmostEqual(data, "velocity", 100.7, places=0)

    def test_fw_on_max_velocity_extended(self):
        """FW on with max cooling: velocity extends beyond base speed."""
        data = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--torque", "0",
            "--field_weakening", "on",
            "--motor_cooling", "max")
        self.assertFieldGreater(data, "velocity", 101)

    def test_fw_on_below_base_speed_no_effect(self):
        """FW on at low speed: same torque as FW off."""
        off = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "48",
            "--velocity", "10",
            "--field_weakening", "off")
        on = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "48",
            "--velocity", "10",
            "--field_weakening", "on")
        self.assertAlmostEqual(off["torque"], on["torque"], places=2)

    def test_fw_off_above_base_speed_null(self):
        """FW off above base speed: analysis returns null."""
        data = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "150",
            "--field_weakening", "off")
        self.assertIsNone(data)

    def test_fw_on_above_base_speed_torque(self):
        """FW on above base speed: non-zero torque is achievable."""
        data = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "150",
            "--field_weakening", "on",
            "--motor_cooling", "max",
            "--time", "2")
        self.assertFieldGreater(data, "torque", 0)

    def test_fw_d_current_above_base_speed(self):
        """Operating point above base speed has non-zero d_current."""
        data = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "120",
            "--torque", "0.1",
            "--field_weakening", "on")
        self.assertFieldLess(data, "d_current", 0)
        self.assertFieldGreater(data, "q_current", 0)
        # phase_current = sqrt(d^2 + q^2) > q_current
        self.assertGreater(data["phase_current"], data["q_current"])

    def test_fw_d_current_below_base_speed(self):
        """Operating point below base speed has zero d_current."""
        data = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "50",
            "--torque", "0.1",
            "--field_weakening", "on")
        self.assertFieldEqual(data, "d_current", 0)

    def test_fw_copper_loss_increases_above_base(self):
        """Copper loss is higher with FW above base speed due to d-axis."""
        below = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "50",
            "--torque", "0.1",
            "--field_weakening", "on")
        above = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "120",
            "--torque", "0.1",
            "--field_weakening", "on")
        self.assertGreater(above["copper_loss"], below["copper_loss"])

    def test_fw_high_torque_above_base_speed_infeasible(self):
        """FW on: operating point with high torque in FW region is
        infeasible due to voltage limit."""
        # At velocity=130 (above base ~101 Hz), the voltage needed for
        # torque=0.45 exceeds the available phase voltage.
        data = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "130",
            "--torque", "0.45",
            "--field_weakening", "on")
        self.assertIsNotNone(data)
        self.assertIn("infeasible_reason", data)

    def test_fw_max_torque_reduced_above_base_speed(self):
        """FW on: max torque in FW region is less than below base speed."""
        below = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "50",
            "--field_weakening", "on",
            "--motor_cooling", "max",
            "--time", "2")
        above = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "130",
            "--field_weakening", "on",
            "--motor_cooling", "max",
            "--time", "2")
        self.assertIsNotNone(below)
        self.assertIsNotNone(above)
        self.assertGreater(below["torque"], above["torque"])

    def test_fw_max_velocity_lower_with_torque(self):
        """FW on: max velocity with significant torque is lower than with
        zero torque.  Uses enough torque (0.5 Nm) to ensure the thermal
        or voltage feasibility limit is binding rather than both cases
        hitting the board voltage ceiling."""
        zero = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "14",
            "--torque", "0",
            "--field_weakening", "on",
            "--motor_cooling", "max")
        nonzero = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "14",
            "--torque", "0.5",
            "--field_weakening", "on",
            "--motor_cooling", "max")
        self.assertIsNotNone(zero)
        self.assertIsNotNone(nonzero)
        self.assertGreater(zero["velocity"], nonzero["velocity"])

    def test_fw_odrive_no_field_weakening(self):
        """ODrive controllers don't support field weakening, so
        FW-on and FW-off should give identical results."""
        off = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "odrive-s1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--torque", "0",
            "--field_weakening", "off",
            "--motor_cooling", "max")
        on = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "odrive-s1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--torque", "0",
            "--field_weakening", "on",
            "--motor_cooling", "max")
        self.assertIsNotNone(off)
        self.assertIsNotNone(on)
        self.assertAlmostEqual(off["velocity"], on["velocity"], places=4)


    # --- Inductance saturation tests ---

    def test_saturation_increases_max_velocity(self):
        """Inductance saturation should extend max FW velocity because
        higher effective L_d at negative i_d increases id_char."""
        no_sat = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "moteus-x1",
            "--motor", "model",
            "--motor.kv", "304", "--motor.r", "0.094",
            "--motor.l_d", "57.2e-6",
            "--motor.thermal_r", "4.55", "--motor.thermal_c", "164.59",
            "--motor.poles", "14",
            "--voltage", "14",
            "--torque", "0",
            "--field_weakening", "on",
            "--motor_cooling", "none",
            "--motor.ld_scale", "0")
        sat = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "moteus-x1",
            "--motor", "model",
            "--motor.kv", "304", "--motor.r", "0.094",
            "--motor.l_d", "57.2e-6",
            "--motor.thermal_r", "4.55", "--motor.thermal_c", "164.59",
            "--motor.poles", "14",
            "--voltage", "14",
            "--torque", "0",
            "--field_weakening", "on",
            "--motor_cooling", "none",
            "--motor.ld_scale", "-3.6e-6")
        self.assertIsNotNone(no_sat)
        self.assertIsNotNone(sat)
        self.assertFieldAlmostEqual(no_sat, "velocity", 58.1, places=0)
        self.assertFieldAlmostEqual(sat, "velocity", 67.5, places=0)

    def test_saturation_zero_scale_matches_base(self):
        """With ld_scale=0, results should match the unsaturated model."""
        base = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "model",
            "--motor.kv", "304", "--motor.r", "0.094",
            "--motor.l_d", "57.2e-6",
            "--motor.thermal_r", "4.55", "--motor.thermal_c", "164.59",
            "--motor.poles", "14",
            "--voltage", "24", "--velocity", "50",
            "--field_weakening", "on")
        with_zero = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "model",
            "--motor.kv", "304", "--motor.r", "0.094",
            "--motor.l_d", "57.2e-6",
            "--motor.thermal_r", "4.55", "--motor.thermal_c", "164.59",
            "--motor.poles", "14",
            "--voltage", "24", "--velocity", "50",
            "--field_weakening", "on",
            "--motor.ld_scale", "0")
        self.assertIsNotNone(base)
        self.assertIsNotNone(with_zero)
        self.assertAlmostEqual(base["torque"], with_zero["torque"],
                               places=4)

    def test_saturation_below_base_speed_no_effect(self):
        """Below base speed, i_d=0, so saturation has no effect."""
        no_sat = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "model",
            "--motor.kv", "304", "--motor.r", "0.094",
            "--motor.l_d", "57.2e-6",
            "--motor.thermal_r", "4.55", "--motor.thermal_c", "164.59",
            "--motor.poles", "14",
            "--voltage", "24", "--velocity", "50", "--torque", "0.1",
            "--field_weakening", "on",
            "--motor.ld_scale", "0")
        sat = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "model",
            "--motor.kv", "304", "--motor.r", "0.094",
            "--motor.l_d", "57.2e-6",
            "--motor.thermal_r", "4.55", "--motor.thermal_c", "164.59",
            "--motor.poles", "14",
            "--voltage", "24", "--velocity", "50", "--torque", "0.1",
            "--field_weakening", "on",
            "--motor.ld_scale", "-3.6e-6")
        self.assertIsNotNone(no_sat)
        self.assertIsNotNone(sat)
        self.assertAlmostEqual(no_sat["torque"], sat["torque"], places=4)
        self.assertFieldEqual(no_sat, "d_current", 0)
        self.assertFieldEqual(sat, "d_current", 0)

    def test_saturation_reduces_d_current_magnitude(self):
        """With saturation, L_d increases at negative i_d, increasing
        id_char, which means less i_d is needed for the same speed ratio."""
        no_sat = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "model",
            "--motor.kv", "304", "--motor.r", "0.094",
            "--motor.l_d", "57.2e-6",
            "--motor.thermal_r", "4.55", "--motor.thermal_c", "164.59",
            "--motor.poles", "14",
            "--voltage", "24", "--velocity", "120", "--torque", "0.1",
            "--field_weakening", "on",
            "--motor.ld_scale", "0")
        sat = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "model",
            "--motor.kv", "304", "--motor.r", "0.094",
            "--motor.l_d", "57.2e-6",
            "--motor.thermal_r", "4.55", "--motor.thermal_c", "164.59",
            "--motor.poles", "14",
            "--voltage", "24", "--velocity", "120", "--torque", "0.1",
            "--field_weakening", "on",
            "--motor.ld_scale", "-3.6e-6")
        self.assertIsNotNone(no_sat)
        self.assertIsNotNone(sat)
        # Saturation increases id_char, so less i_d needed (smaller magnitude)
        self.assertFieldAlmostEqual(no_sat, "d_current", -27.4, places=0)
        self.assertFieldAlmostEqual(sat, "d_current", -14.4, places=0)


    # --- Voltage-limited torque tests ---

    def test_voltage_limits_torque_near_max_speed(self):
        """Near max speed, voltage headroom limits the achievable torque
        well below the thermal limit.  At 95% of max speed, the
        max_torque should be substantially less than at low speed."""
        low_speed = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-r4",
            "--motor", "mj5208",
            "--voltage", "14",
            "--velocity", "10")
        # max_velocity is ~57 Hz; 95% is ~54 Hz
        high_speed = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-r4",
            "--motor", "mj5208",
            "--voltage", "14",
            "--velocity", "54")
        self.assertIsNotNone(low_speed)
        self.assertIsNotNone(high_speed)
        # At 54 Hz, voltage limiting should reduce torque to less than
        # half of the low-speed value.
        self.assertGreater(low_speed["torque"], 0.1)
        self.assertLess(high_speed["torque"], low_speed["torque"] * 0.6)
        self.assertGreater(high_speed["torque"], 0)

    def test_zero_torque_speed_matches_max_velocity(self):
        """The speed at which max_torque reaches zero should match the
        max_velocity analysis at zero torque."""
        max_vel = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "moteus-r4",
            "--motor", "mj5208",
            "--voltage", "14",
            "--torque", "0")
        self.assertIsNotNone(max_vel)
        # Query max_torque just below max_velocity (round down to avoid
        # exceeding the getMaxVelocity gate).
        vel_str = f"{max_vel['velocity'] * 0.99:.1f}"
        at_limit = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-r4",
            "--motor", "mj5208",
            "--voltage", "14",
            "--velocity", vel_str)
        self.assertIsNotNone(at_limit)
        # Torque should be near zero at 99% of max_velocity.
        self.assertLess(at_limit["torque"], 0.05)

    def test_torque_monotonically_decreases_with_speed(self):
        """The max torque should decrease monotonically with speed."""
        prev_torque = None
        for vel in [0, 10, 20, 30, 40, 50, 54, 56]:
            data = self.run_mpat_json(
                "--analysis", "max_torque",
                "--controller", "moteus-r4",
                "--motor", "mj5208",
                "--voltage", "14",
                "--velocity", str(vel))
            self.assertIsNotNone(data)
            if prev_torque is not None:
                self.assertLessEqual(data["torque"], prev_torque,
                    f"Torque increased from {prev_torque} to "
                    f"{data['torque']} at {vel} Hz")
            prev_torque = data["torque"]

    def test_higher_voltage_increases_high_speed_torque(self):
        """Higher bus voltage provides more voltage headroom, allowing
        more torque at high speed.  At 55 Hz the MJ5208 is deeply
        voltage-limited at 14V but has headroom at 24V."""
        low_v = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-r4",
            "--motor", "mj5208",
            "--voltage", "14",
            "--velocity", "55")
        high_v = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-r4",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "55")
        self.assertIsNotNone(low_v)
        self.assertIsNotNone(high_v)
        self.assertGreater(high_v["torque"], low_v["torque"])


    # --- ODrive back-EMF limited velocity regression tests ---

    def test_odrive_max_velocity_regression(self):
        """ODrive controllers should match empirically validated
        back-EMF limited velocities at 12V, time=2."""
        cases = [
            ("odrive-pro", "mj5208", 54.72),
            ("odrive-pro", "mad8318", 21.60),
            ("odrive-pro", "be8108", 24.30),
            ("odrive-s1", "mj5208", 45.60),
            ("odrive-s1", "mad8318", 18.00),
            ("odrive-s1", "be8108", 20.25),
            ("odrive-micro", "mj5208", 45.60),
            ("odrive-micro", "mad8318", 18.00),
            ("odrive-micro", "be8108", 20.25),
        ]
        for ctrl, motor, expected_vel in cases:
            with self.subTest(controller=ctrl, motor=motor):
                data = self.run_mpat_json(
                    "--analysis", "max_velocity",
                    "--controller", ctrl,
                    "--motor", motor,
                    "--voltage", "12",
                    "--torque", "0",
                    "--time", "2")
                self.assertIsNotNone(data,
                    f"{ctrl} {motor}: returned null")
                self.assertAlmostEqual(
                    data["velocity"], expected_vel, places=1,
                    msg=f"{ctrl} {motor}: {data['velocity']:.2f} != {expected_vel}")


if __name__ == "__main__":
    unittest.main()
