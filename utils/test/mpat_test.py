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

import subprocess
import unittest

from bazel_tools.tools.python.runfiles import runfiles


class MpatTest(unittest.TestCase):
    def setUp(self):
        self.runfiles = runfiles.Create()
        self.mpat = self.runfiles.Rlocation(
            "com_github_mjbots_moteus/utils/mpat")

    def test_help(self):
        """Test that --help produces output."""
        result = subprocess.run(
            [self.mpat, "--help"],
            capture_output=True,
            text=True
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("moteus Performance Analysis Tool", result.stdout)
        self.assertIn("--analysis", result.stdout)

    def test_max_torque(self):
        """Test max_torque analysis produces valid output."""
        result = subprocess.run(
            [self.mpat,
             "--analysis", "max_torque",
             "--controller", "moteus-x1",
             "--motor", "mj5208",
             "--voltage", "48",
             "--velocity", "10"],
            capture_output=True,
            text=True
        )
        self.assertEqual(result.returncode, 0)
        # Output format depends on --output default; just verify we get a torque value
        self.assertIn("Nm", result.stdout)

    def test_json_output(self):
        """Test JSON output format."""
        result = subprocess.run(
            [self.mpat,
             "--analysis", "max_current",
             "--controller", "moteus-x1",
             "--voltage", "48",
             "--json"],
            capture_output=True,
            text=True
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("phase_current", result.stdout)
        # Verify it's valid JSON by checking for braces
        self.assertIn("{", result.stdout)
        self.assertIn("}", result.stdout)


if __name__ == "__main__":
    unittest.main()
