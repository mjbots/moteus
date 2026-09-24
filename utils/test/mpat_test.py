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
import math
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

    def assertLimitingFactor(self, data, expected):
        """Assert that the analysis reports the given limiting
        factor."""
        self.assertFieldEqual(data, "limiting_factor", expected)

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
        self.assertLimitingFactor(data, "controller_temp")

    def test_json_output_no_limiting_factor(self):
        """An operating point has no limiting factor, so none is
        reported."""
        data = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "5",
            "--torque", "0.1")
        self.assertIsNotNone(data)
        self.assertNotIn("limiting_factor", data)

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
        self.assertFieldAlmostEqual(data, "velocity", 102.0, places=0)
        self.assertLimitingFactor(data, "back_emf")

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
        self.assertEqual(off["limiting_factor"], on["limiting_factor"])

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
        # torque=0.9 exceeds the available phase voltage, while 0.8
        # still fits.  The short duration with maximum cooling keeps
        # the thermal checks, which are reported in preference to the
        # voltage limit, out of the way.
        args = ["--analysis", "operating_point",
                "--controller", "moteus-x1",
                "--motor", "mj5208",
                "--voltage", "24",
                "--velocity", "130",
                "--field_weakening", "on",
                "--motor_cooling", "max",
                "--time", "2"]
        feasible = self.run_mpat_json(*args, "--torque", "0.8")
        infeasible = self.run_mpat_json(*args, "--torque", "0.9")
        self.assertIsNotNone(feasible)
        self.assertIsNotNone(infeasible)
        self.assertNotIn("infeasible_reason", feasible)
        self.assertFieldEqual(infeasible, "infeasible_reason", "back_emf")

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
        self.assertLimitingFactor(above, "back_emf")

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
        self.assertLimitingFactor(zero, "back_emf")
        self.assertLimitingFactor(nonzero, "max_controller_temp")

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
        self.assertLimitingFactor(off, "back_emf")
        self.assertLimitingFactor(on, "back_emf")


    # --- Winding resistance temperature tests ---

    COPPER_TEMPCO = 0.00393

    # Phase resistance 0.1 ohm, 2 C/W, 100 J/C.
    WINDING_MOTOR = [
        "--controller", "moteus-x1",
        "--motor", "model",
        "--motor.kv", "100", "--motor.r", "0.2", "--motor.l_d", "1e-4",
        "--motor.thermal_r", "2", "--motor.thermal_c", "100",
        "--motor.poles", "14",
        "--voltage", "24"]

    # Phase resistance 1 ohm at a low voltage, so the resistive drop
    # limits the current at zero velocity.
    IR_LIMITED_MOTOR = [
        "--controller", "moteus-x1",
        "--motor", "model",
        "--motor.kv", "100", "--motor.r", "2", "--motor.l_d", "1e-4",
        "--motor.thermal_r", "2", "--motor.thermal_c", "100",
        "--motor.poles", "14",
        "--voltage", "12"]

    # A moteus-c1 at 48V, whose 150W power limit is reached before its
    # 20A current limit or the motor temperature limit.
    POWER_LIMITED_MOTOR = [
        "--controller", "moteus-c1",
        "--controller_cooling", "max",
        "--motor", "model",
        "--motor.kv", "100", "--motor.r", "1.0", "--motor.l_d", "1e-4",
        "--motor.thermal_r", "0.5", "--motor.thermal_c", "1000",
        "--motor.poles", "14",
        "--voltage", "48"]

    def resistance_ratio(self, temp):
        """The winding resistance at `temp` relative to that at 25C."""
        return 1 + self.COPPER_TEMPCO * (temp - 25)

    def electrical_power(self, data):
        return (data["copper_loss"] + data["iron_loss"] +
                data["mechanical_power"])

    def test_winding_copper_loss_at_temperature(self):
        """Copper loss uses the resistance at the winding temperature
        the operating point reaches, from the 25C value."""
        data = self.run_mpat_json(
            "--analysis", "operating_point", *self.WINDING_MOTOR,
            "--velocity", "0", "--torque", "1")
        self.assertIsNotNone(data)
        current = data["phase_current"]
        temp = data["motor_temp"]
        r25 = 0.2 / 2
        expected = 1.5 * current * current * r25 * self.resistance_ratio(temp)
        self.assertAlmostEqual(data["copper_loss"], expected, places=6)
        # At zero velocity the copper loss is all of it, and in steady
        # state the rise is that loss times the thermal resistance.
        self.assertAlmostEqual(temp, 25 + data["copper_loss"] * 2, places=6)
        self.assertGreater(temp, 70)

    def test_winding_temperature_with_ambient_and_iron_loss(self):
        """With an ambient other than 25C and iron loss, the winding
        temperature and copper loss are still self consistent."""
        data = self.run_mpat_json(
            "--analysis", "operating_point", *self.WINDING_MOTOR,
            "--motor.d0", "0.01",
            "--ambient_temp", "40",
            "--velocity", "5", "--torque", "1")
        self.assertIsNotNone(data)
        current = data["phase_current"]
        temp = data["motor_temp"]
        self.assertGreater(data["iron_loss"], 0.1)
        self.assertAlmostEqual(
            data["copper_loss"],
            1.5 * current * current * 0.1 * self.resistance_ratio(temp),
            places=6)
        self.assertAlmostEqual(
            temp, 40 + (data["copper_loss"] + data["iron_loss"]) * 2,
            places=6)

    def test_winding_transient_temperature(self):
        """Over a finite time, the rise is the loss at the final
        temperature times the transient thermal gain."""
        data = self.run_mpat_json(
            "--analysis", "operating_point", *self.WINDING_MOTOR,
            "--velocity", "0", "--torque", "1", "--time", "60")
        self.assertIsNotNone(data)
        current = data["phase_current"]
        temp = data["motor_temp"]
        gain = 2 * (1 - math.exp(-60 / (2 * 100)))
        self.assertAlmostEqual(
            data["copper_loss"],
            1.5 * current * current * 0.1 * self.resistance_ratio(temp),
            places=6)
        self.assertAlmostEqual(temp, 25 + data["copper_loss"] * gain,
                               places=6)
        self.assertLess(temp, 50)

    def test_winding_tempco_zero_matches_fixed_resistance(self):
        """With no tempco the copper loss uses the 25C resistance."""
        data = self.run_mpat_json(
            "--analysis", "operating_point", *self.WINDING_MOTOR,
            "--velocity", "0", "--torque", "1", "--motor.r_tempco", "0")
        self.assertIsNotNone(data)
        current = data["phase_current"]
        self.assertAlmostEqual(data["copper_loss"],
                               1.5 * current * current * 0.1, places=6)

    def test_winding_hot_reduces_thermally_limited_torque(self):
        """When the motor temperature is the limit, the winding is at
        its maximum temperature, so less torque is available than with
        the 25C resistance."""
        hot = self.run_mpat_json(
            "--analysis", "max_torque", *self.WINDING_MOTOR, "--velocity", "1")
        fixed = self.run_mpat_json(
            "--analysis", "max_torque", *self.WINDING_MOTOR, "--velocity", "1",
            "--motor.r_tempco", "0")
        self.assertIsNotNone(hot)
        self.assertIsNotNone(fixed)
        self.assertLimitingFactor(hot, "max_motor_temperature")
        self.assertLimitingFactor(fixed, "max_motor_temperature")
        self.assertAlmostEqual(hot["motor_temp"], 80, places=0)
        self.assertLess(hot["torque"], 0.95 * fixed["torque"])

    def test_winding_thermal_limit_exact_at_zero_velocity(self):
        """At zero velocity the thermally limited current is solved
        exactly, so the winding ends at exactly the maximum
        temperature, including with an ambient other than 25C."""
        data = self.run_mpat_json(
            "--analysis", "max_torque", *self.WINDING_MOTOR,
            "--velocity", "0", "--ambient_temp", "40")
        self.assertIsNotNone(data)
        self.assertLimitingFactor(data, "max_motor_temperature")
        self.assertAlmostEqual(data["motor_temp"], 80, places=6)
        self.assertAlmostEqual(data["copper_loss"], (80 - 40) / 2, places=6)
        current = data["phase_current"]
        self.assertAlmostEqual(
            data["copper_loss"],
            1.5 * current * current * 0.1 * self.resistance_ratio(80),
            places=6)

    def test_winding_cool_voltage_limited_unchanged(self):
        """A voltage limited point over a short duration leaves the
        winding near ambient, so the tempco barely matters there."""
        args = ["--analysis", "max_torque", *self.WINDING_MOTOR,
                "--velocity", "30", "--time", "0.1"]
        hot = self.run_mpat_json(*args)
        fixed = self.run_mpat_json(*args, "--motor.r_tempco", "0")
        self.assertIsNotNone(hot)
        self.assertIsNotNone(fixed)
        self.assertLimitingFactor(hot, "back_emf")
        self.assertLimitingFactor(fixed, "back_emf")
        self.assertLess(hot["motor_temp"], 26)
        self.assertAlmostEqual(hot["torque"], fixed["torque"], places=2)

    def test_winding_ir_limited_current_at_zero_velocity(self):
        """At zero velocity with the resistive drop as the limit, the
        drop at the winding temperature that current produces is the
        same as the drop at 25C with no tempco, which is the available
        voltage."""
        args = ["--analysis", "max_torque", *self.IR_LIMITED_MOTOR,
                "--velocity", "0", "--time", "60"]
        hot = self.run_mpat_json(*args)
        fixed = self.run_mpat_json(*args, "--motor.r_tempco", "0")
        self.assertIsNotNone(hot)
        self.assertIsNotNone(fixed)
        self.assertLimitingFactor(hot, "back_emf")
        self.assertLimitingFactor(fixed, "back_emf")
        self.assertGreater(hot["motor_temp"], 35)
        self.assertAlmostEqual(
            hot["phase_current"] * self.resistance_ratio(hot["motor_temp"]),
            fixed["phase_current"], places=6)

    def test_winding_controller_power_limited_at_zero_velocity(self):
        """At zero velocity with the controller power as the limit,
        the copper loss is exactly that power, at the winding
        temperature it produces."""
        args = ["--analysis", "max_torque", *self.POWER_LIMITED_MOTOR,
                "--velocity", "0", "--time", "10"]
        hot = self.run_mpat_json(*args)
        fixed = self.run_mpat_json(*args, "--motor.r_tempco", "0")
        self.assertIsNotNone(hot)
        self.assertIsNotNone(fixed)
        self.assertLimitingFactor(hot, "max_controller_power")
        self.assertLimitingFactor(fixed, "max_controller_power")
        self.assertAlmostEqual(hot["copper_loss"], 150, places=6)
        self.assertAlmostEqual(fixed["copper_loss"], 150, places=6)
        self.assertAlmostEqual(
            hot["phase_current"],
            fixed["phase_current"] /
            math.sqrt(self.resistance_ratio(hot["motor_temp"])),
            places=6)

    def test_winding_max_torque_continuous_at_zero_velocity(self):
        """The exact zero velocity solution agrees with the numerical
        one used at any other velocity, for each limit."""
        cases = [
            ("thermal", self.WINDING_MOTOR, [],
             "max_motor_temperature"),
            ("ir_drop", self.IR_LIMITED_MOTOR, ["--time", "60"],
             "back_emf"),
            ("controller_power", self.POWER_LIMITED_MOTOR, ["--time", "10"],
             "max_controller_power"),
        ]
        for name, motor, extra, limit in cases:
            with self.subTest(limit=name):
                args = ["--analysis", "max_torque", *motor, *extra]
                zero = self.run_mpat_json(*args, "--velocity", "0")
                moving = self.run_mpat_json(*args, "--velocity", "0.001")
                self.assertIsNotNone(zero)
                self.assertIsNotNone(moving)
                self.assertLimitingFactor(zero, limit)
                self.assertLimitingFactor(moving, limit)
                self.assertAlmostEqual(zero["torque"], moving["torque"],
                                       places=2)

    def test_winding_controller_power_limited_max_velocity(self):
        """The controller power limited velocity accounts for the
        copper loss at the winding temperature that velocity produces,
        so the electrical power there is exactly the limit, and the
        velocity is lower than with no tempco."""
        args = ["--analysis", "max_velocity",
                *self.POWER_LIMITED_MOTOR,
                "--motor.r", "0.4",
                "--motor.thermal_r", "1", "--motor.thermal_c", "300",
                "--motor.d0", "0.01", "--motor.d1", "0.0005",
                "--torque", "0.5"]
        for fw, places in [("off", 6), ("on", 1)]:
            with self.subTest(field_weakening=fw):
                hot = self.run_mpat_json(*args, "--field_weakening", fw)
                fixed = self.run_mpat_json(
                    *args, "--field_weakening", fw, "--motor.r_tempco", "0")
                self.assertIsNotNone(hot)
                self.assertIsNotNone(fixed)
                self.assertLimitingFactor(hot, "max_controller_power")
                self.assertLimitingFactor(fixed, "max_controller_power")
                self.assertIsNone(hot.get("infeasible_reason"))
                self.assertAlmostEqual(self.electrical_power(hot), 150,
                                       places=places)
                self.assertAlmostEqual(self.electrical_power(fixed), 150,
                                       places=places)
                self.assertGreater(hot["motor_temp"], 40)
                self.assertLess(hot["velocity"], fixed["velocity"] - 0.1)

    def test_winding_thermal_runaway_infeasible(self):
        """If the copper loss grows with temperature faster than the
        cooling removes it, there is no steady temperature."""
        data = self.run_mpat_json(
            "--analysis", "operating_point", *self.WINDING_MOTOR,
            "--motor.thermal_r", "1000",
            "--velocity", "0", "--torque", "1")
        self.assertIsNotNone(data)
        self.assertIsNone(data["motor_temp"])
        self.assertIn("Motor thermal runaway", data["infeasible_reason"])

    def test_winding_thermal_runaway_finite_time_infeasible(self):
        """Over a finite time, a runaway is reported as exceeding the
        maximum temperature."""
        data = self.run_mpat_json(
            "--analysis", "operating_point", *self.WINDING_MOTOR,
            "--motor.thermal_r", "1000",
            "--velocity", "0", "--torque", "1", "--time", "2000")
        self.assertIsNotNone(data)
        self.assertIsNone(data["motor_temp"])
        self.assertEqual(
            data["infeasible_reason"],
            "Motor temperature exceeds maximum 80.0 (thermal runaway)")

    # --- Inductance saturation tests ---

    def test_saturation_increases_max_velocity(self):
        """Inductance saturation should extend max FW velocity because
        higher effective L_d at negative i_d increases id_char.  Less
        i_d is then needed at a given speed, so the d axis copper loss
        that sets the motor temperature limit is reached later."""
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
            "--motor.r_tempco", "0",
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
            "--motor.r_tempco", "0",
            "--motor.ld_scale", "-3.6e-6")
        self.assertIsNotNone(no_sat)
        self.assertIsNotNone(sat)
        self.assertFieldAlmostEqual(no_sat, "velocity", 58.8, places=0)
        self.assertFieldAlmostEqual(sat, "velocity", 68.4, places=0)
        self.assertLimitingFactor(no_sat, "max_motor_temp")
        self.assertLimitingFactor(sat, "max_motor_temp")

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
        self.assertFieldAlmostEqual(no_sat, "d_current", -26.6, places=0)
        self.assertFieldAlmostEqual(sat, "d_current", -14.4, places=0)


    def test_operating_point_max_controller_temp_override(self):
        """The operating point is checked against the configured
        controller temperature limit, not the controller default."""
        args = ["--analysis", "operating_point",
                "--controller", "moteus-n1",
                "--torque", "4", "--time", "10"]
        default = self.run_mpat_json(*args)
        lowered = self.run_mpat_json(*args, "--max_controller_temp", "35")
        self.assertIsNotNone(default)
        self.assertIsNotNone(lowered)
        self.assertIn("exceeds maximum 60.0", default["infeasible_reason"])
        self.assertIn("exceeds maximum 35.0", lowered["infeasible_reason"])

        raised = self.run_mpat_json(*args, "--max_controller_temp", "100")
        self.assertIsNotNone(raised)
        self.assertNotIn("Controller temperature",
                         raised.get("infeasible_reason") or "")

    def test_operating_point_max_motor_temp_override(self):
        """The operating point is checked against the configured motor
        temperature limit, not the motor default.  Only the last
        failing check is reported, so the controller limit is raised
        out of the way."""
        data = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-n1",
            "--motor", "mj5208",
            "--voltage", "24", "--velocity", "5", "--torque", "0.5",
            "--max_controller_temp", "500",
            "--max_motor_temp", "20")
        self.assertIsNotNone(data)
        self.assertIn("Motor temperature", data["infeasible_reason"])
        self.assertIn("exceeds maximum 20.0", data["infeasible_reason"])

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
        self.assertLimitingFactor(low_speed, "max_motor_temperature")
        self.assertLimitingFactor(high_speed, "back_emf")

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
        self.assertLimitingFactor(max_vel, "back_emf")
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
        self.assertLimitingFactor(low_v, "back_emf")
        self.assertLimitingFactor(high_v, "max_motor_temperature")


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
                self.assertLimitingFactor(data, "back_emf")


    # --- Gear reduction tests ---
    #
    # gear_reduction is the rotor-to-output ratio: output_vel =
    # rotor_vel / gear_reduction and output_torque = rotor_torque *
    # gear_reduction.  Motor parameters (Kv, R, L_d, poles, d0, d1)
    # and phase currents are rotor-space; the CLI's --velocity and
    # --torque are output-space.  The physics are identical at a
    # matched rotor operating point regardless of gear ratio, so a
    # configuration at (gear=N, out_vel=V, out_torque=N*T) should
    # report the same rotor-space quantities as (gear=1, out_vel=N*V,
    # out_torque=T).

    def test_gear_reduction_scales_max_velocity_inversely(self):
        """Zero-torque max_velocity (back-EMF limited) at gear=2 is
        half the direct-drive value."""
        direct = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--torque", "0",
            "--gear_reduction", "1",
            "--field_weakening", "off",
            "--motor_cooling", "max")
        geared = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--torque", "0",
            "--gear_reduction", "2",
            "--field_weakening", "off",
            "--motor_cooling", "max")
        self.assertIsNotNone(direct)
        self.assertIsNotNone(geared)
        self.assertAlmostEqual(
            geared["velocity"], direct["velocity"] / 2.0, places=3)
        self.assertLimitingFactor(direct, "back_emf")
        self.assertLimitingFactor(geared, "back_emf")

    def test_gear_reduction_rotor_invariance_below_base_speed(self):
        """Matched rotor conditions via different gear ratios produce
        identical phase currents and losses when operating below base
        speed (no field weakening engaged)."""
        direct = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "48",
            "--velocity", "10",
            "--torque", "0.1",
            "--gear_reduction", "1",
            "--field_weakening", "on")
        geared = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "48",
            "--velocity", "5",
            "--torque", "0.2",
            "--gear_reduction", "2",
            "--field_weakening", "on")
        self.assertIsNotNone(direct)
        self.assertIsNotNone(geared)
        # Rotor-space quantities must match exactly.
        self.assertAlmostEqual(
            geared["q_current"], direct["q_current"], places=6)
        self.assertEqual(geared["d_current"], 0)
        self.assertEqual(direct["d_current"], 0)
        self.assertAlmostEqual(
            geared["phase_current"], direct["phase_current"], places=6)
        self.assertAlmostEqual(
            geared["copper_loss"], direct["copper_loss"], places=6)
        self.assertAlmostEqual(
            geared["iron_loss"], direct["iron_loss"], places=6)

    def test_gear_reduction_scales_output_torque(self):
        """max_torque scales with gear_reduction at matched rotor
        speed (output torque equals rotor torque times gear ratio)."""
        direct = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "48",
            "--velocity", "10",
            "--gear_reduction", "1",
            "--field_weakening", "off",
            "--motor_cooling", "max",
            "--time", "2")
        geared = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "48",
            "--velocity", "5",
            "--gear_reduction", "2",
            "--field_weakening", "off",
            "--motor_cooling", "max",
            "--time", "2")
        self.assertIsNotNone(direct)
        self.assertIsNotNone(geared)
        self.assertAlmostEqual(
            geared["torque"], 2.0 * direct["torque"], places=3)
        # Same phase current since the rotor is at an identical
        # operating point in both cases.
        self.assertAlmostEqual(
            geared["phase_current"], direct["phase_current"], places=6)
        self.assertLimitingFactor(direct, "max_controller_power")
        self.assertLimitingFactor(geared, "max_controller_power")

    def test_gear_reduction_fw_engages_at_scaled_output_velocity(self):
        """Field weakening onset is governed by rotor (not output)
        speed.  At output velocity 55 Hz with gear=2 the rotor is at
        110 Hz and FW engages; with gear=1 the rotor is only at 55 Hz
        and FW stays dormant."""
        direct = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "55",
            "--torque", "0.1",
            "--gear_reduction", "1",
            "--field_weakening", "on")
        geared = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "55",
            "--torque", "0.1",
            "--gear_reduction", "2",
            "--field_weakening", "on")
        self.assertFieldEqual(direct, "d_current", 0)
        self.assertFieldLess(geared, "d_current", 0)

    def test_gear_reduction_fw_rotor_invariance_above_base(self):
        """Matched rotor conditions above base speed yield identical
        d_current, q_current, and losses across gear ratios."""
        direct = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "120",
            "--torque", "0.1",
            "--gear_reduction", "1",
            "--field_weakening", "on")
        geared = self.run_mpat_json(
            "--analysis", "operating_point",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "60",
            "--torque", "0.2",
            "--gear_reduction", "2",
            "--field_weakening", "on")
        self.assertIsNotNone(direct)
        self.assertIsNotNone(geared)
        self.assertAlmostEqual(
            geared["q_current"], direct["q_current"], places=6)
        self.assertAlmostEqual(
            geared["d_current"], direct["d_current"], places=6)
        self.assertAlmostEqual(
            geared["phase_current"], direct["phase_current"], places=6)
        self.assertAlmostEqual(
            geared["copper_loss"], direct["copper_loss"], places=6)
        self.assertAlmostEqual(
            geared["iron_loss"], direct["iron_loss"], places=6)

    def test_gear_reduction_fw_off_above_base_null_regardless_of_gear(self):
        """With FW off, max_torque above the gear-scaled base speed
        returns null whether gear_reduction is 1 or 2."""
        direct = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "120",
            "--gear_reduction", "1",
            "--field_weakening", "off",
            "--motor_cooling", "max",
            "--time", "2")
        geared = self.run_mpat_json(
            "--analysis", "max_torque",
            "--controller", "moteus-x1",
            "--motor", "mj5208",
            "--voltage", "24",
            "--velocity", "60",
            "--gear_reduction", "2",
            "--field_weakening", "off",
            "--motor_cooling", "max",
            "--time", "2")
        self.assertIsNone(direct)
        self.assertIsNone(geared)

    def test_gear_reduction_max_velocity_rotor_invariance(self):
        """At matched rotor conditions, max_velocity_output scales as
        1/gear_reduction even when the non-FW analytical
        controller-power path in Motor.getVelocityFromThermalPowerAndTorque
        is evaluated (mad8318 at 24V, torque=3 on gear=1 is back-EMF
        limited at ~40.26 Hz; gear=2 at output_torque=6 must yield
        ~20.13 Hz)."""
        direct = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "moteus-x1",
            "--motor", "mad8318",
            "--voltage", "24",
            "--torque", "3",
            "--gear_reduction", "1",
            "--field_weakening", "off",
            "--motor_cooling", "max",
            "--time", "5")
        geared = self.run_mpat_json(
            "--analysis", "max_velocity",
            "--controller", "moteus-x1",
            "--motor", "mad8318",
            "--voltage", "24",
            "--torque", "6",
            "--gear_reduction", "2",
            "--field_weakening", "off",
            "--motor_cooling", "max",
            "--time", "5")
        self.assertIsNotNone(direct)
        self.assertIsNotNone(geared)
        self.assertAlmostEqual(
            geared["velocity"], direct["velocity"] / 2.0, places=2)
        self.assertLimitingFactor(direct, "back_emf")
        self.assertLimitingFactor(geared, "back_emf")


if __name__ == "__main__":
    unittest.main()
