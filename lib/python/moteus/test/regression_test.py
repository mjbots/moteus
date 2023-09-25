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

import unittest

import moteus.regression as regression

DUT = regression.linear_regression


class RouterTest(unittest.TestCase):
    def test_basic1(self):
        self.assertEqual(DUT([0, 2, 4], [0, 2, 4]), (0, 1))
        self.assertEqual(DUT([0, 2, 4], [4, 2, 0]), (4, -1))

        result = DUT([0, 2, 4], [0, 3, 5])
        self.assertAlmostEqual(result[0], 0.16666666)
        self.assertAlmostEqual(result[1], 1.25)


if __name__ == '__main__':
    unittest.main()
