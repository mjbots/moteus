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

import matplotlib.pyplot as plt
import sys

data = [tuple(float(y) for y in x.strip().split(','))
        for x in open(sys.argv[1]).readlines()]


fig1, ax = plt.subplots()

ax2 = ax.twinx()

ax2.plot([x[0] for x in data], [x[1] for x in data], color='yellow', label='x')
ax2.legend(loc='upper right')

ax.plot([x[0] for x in data], [x[2] for x in data], label='v')
ax.plot([x[0] for x in data], [x[3] for x in data], label='a')
ax.plot([x[0] for x in data], [x[4] for x in data], label='D')

ax.legend(loc='upper left')
plt.show()
