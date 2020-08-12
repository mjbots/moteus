#!/usr/bin/python3

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

# The following data was captured on 2020-08-11 with an r4.3 board, a
# 5008 motor, and firmware version
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

power = [x[0] * x[1] for x in DATA]

pylab.plot(power, [x[2] for x in DATA])
pylab.xlabel('Power (W)')
pylab.ylabel('Peak-Peak Ripple (V)')
pylab.grid()
pylab.title('Ripple vs Power for moteus r4.3')

pylab.show()
