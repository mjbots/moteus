#!/usr/bin/python3

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

import pylab

# Phase voltage, phase current, peak-peak ripple measured at XT30

# The following data was captured on 2020-08-11 at 24V w/ eXtreme with
# an r4.3 board, a 5008 motor, and firmware version
# 3db3983b8a60bab0750ddd07eec0aebe3222c3cc
DATA = [
    (0.8, 15, 0.50),
    (1.0, 18, 0.66),
    (1.5, 27, 1.14),
    (2.0, 37, 1.63),
    (2.5, 48, 2.50),
    (3.0, 57, 3.56),
    (3.5, 65, 4.44),
    (4.0, 75, 5.44),
    (4.5, 82, 6.84),
    (5.0, 92, 7.92),
    (5.5, 100, 8.96),
    (6.0, 107, 10.7),
    (6.5, 114, 13.2),
]

# The following data was captured on 2020-10-22 at 40V w/ Chroma with
# a r4.5 board, a 5008 motor, and firmware version
# 56a9bf61bb8873ae9df84bae69fad4d589e947ab
DATA_40V = [
    (0.8, 14, 0.50),
    (1.0, 17.5, 0.61),
    (1.5, 26, 0.860),
    (2.0, 35, 1.40),
    (2.5, 43, 1.97),
    (3.0, 52, 2.72),
    (3.5, 59, 3.26),
    (4.0, 65, 4.02),
    (4.5, 74, 5.00),
    (5.0, 84, 6.76),
]

# The following data was captured on 2020-10-24 at 44V w/ Chroma with
# a r4.5 board w/ double caps, a 5008 motor
DATA_40V = [
    (1.0, 18, 0.330),
    (1.5, 26, 0.650),
    (2.0, 35, 0.930),
    (2.5, 43, 1.43),
    (3.0, 52, 1.90),
    (3.5, 59, 2.50),
    (4.0, 65, 3.28),
    (4.5, 74, 4.30),
    (5.0, 84, 4.40),
]

# The following data was captured on 2020-12-18 at 44V w/ 1.8k supply
# w/ a r4.5 board w/ double caps and 5008 motor.
DATA_40V = [
    (1.0, 20.5, 0.238),
    (1.5, 29.8, 0.482),
    (2.0, 41.0, 0.912),
    (2.5, 52.0, 1.32),
    (3.0, 63.6, 1.85),
    (3.5, 80.0, 2.34),
    (4.0, 83.0, 2.92),
    (4.5, 97.0, 3.64),
    (5.0, 107.0, 4.72),
    (5.5, 118.0, 5.32),
]

power = [x[0] * x[1] for x in DATA]

pylab.plot(power, [x[2] for x in DATA])
pylab.xlabel('Power (W)')
pylab.ylabel('Peak-Peak Ripple (V)')
pylab.grid()
pylab.title('Ripple vs Power for moteus r4.3')

pylab.show()
