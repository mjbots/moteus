#!/usr/bin/python3 -B

# Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

lines = open(sys.argv[1]).readlines()
header = lines[0].strip().split(',')

data = [tuple(float(y) for y in x.strip().split(','))
        for x in lines[1:]]

def get(name, line):
    return line[header.index(name)]

fig1, ax = plt.subplots()

ax2 = ax.twinx()

x_time = [get('time', x) for x in data]

ax2.plot(x_time, [get('velocity', x) for x in data], color='red', label='velocity')
ax2.plot(x_time, [get('truth_vel', x) for x in data], '--', color='salmon', label='truth_vel')
ax2.legend(loc='upper right')

ax.plot(x_time, [get('compensated', x) for x in data], '.', label='compensated')
ax.plot(x_time, [get('filtered', x) for x in data], label='filtered')
ax.plot(x_time, [get('truth_pos', x) for x in data], '--', label='truth_pos')

ax.legend(loc='upper left')
plt.show()
