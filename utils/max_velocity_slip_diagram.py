#!/usr/bin/python3 -B

# Copyright 2024 mjbots Robotic Systems, LLC.  info@mjbots.com
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

fig1, axes = plt.subplots(3, 2)

axes[0, 0].plot([0, 1, 1, 10], [0, 0, 2, 2], lw=2)
axes[0, 0].set_ylabel('velocity command')
axes[0, 0].set_xticks([])
axes[0, 0].set_title('max_velocity_slip=nan')

axes[1, 0].plot([0, 1, 3, 10], [0, 0, 2, 2], lw=2)
axes[1, 0].set_ylabel('control velocity')
axes[1, 0].set_xticks([])

axes[2, 0].plot([0, 1, 3, 5, 5.5,  7, 7.1, 7.2,  7.4,  7.6, 7.8, 8.5, 10],
                [0, 0, 2, 2, 0.0,  0, 2.6, 2.65, 2.55, 2.4, 2.2, 2,   2],
                lw=2)
axes[2, 0].set_ylabel('actual velocity')


axes[0, 1].plot([0, 1, 1, 10], [0, 0, 2, 2], lw=2)
axes[0, 1].set_xticks([])
axes[0, 1].set_title('max_velocity_slip=0.5')

axes[1, 1].plot([0, 1, 3, 5.2, 5.5, 7,   8.5, 10],
                [0, 0, 2, 2,   0.5, 0.5, 2,   2],
                lw=2)
axes[1, 1].set_xticks([])

axes[2, 1].plot([0, 1, 3, 5, 5.5, 7, 7.1, 8.5, 10],
                [0, 0, 2, 2, 0,   0, 0.5, 2,   2],
                lw=2)

for col in [0, 1]:
    axes[2, col].annotate(
        'external brake engaged',
        xy=(5, -0.3),
        xytext=(-13, -3),
        textcoords='offset fontsize',
        arrowprops=dict(facecolor='black', shrink=0.05))
    axes[2, col].annotate(
        'external brake released',
        xy=(7, -0.3),
        xytext=(-13, -5),
        textcoords='offset fontsize',
        arrowprops=dict(facecolor='black', shrink=0.05))


for ax in axes.flatten():
    ax.set_xbound(0, 10)
    ax.set_ylim(-0.3, 2.9)
    ax.vlines([5, 7], -0.3, 3.0, 'black', ls='-.')

fig1.suptitle('Effect of max_velocity_slip with external torque that exceeds maximum')

plt.show()
