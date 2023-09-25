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

import matplotlib.pyplot as plt
import pylab

# Calculated motor temp (to be converted into power) versus measured
# FET temperature.
TEMP_24V_40KHZ = [
    (27.4, 39),
    (29.9, 44),
    (35.9, 54),
    (44.4, 64),
    (54.7, 75),
    (65.3, 86),
]

TEMP_24V_15KHZ = [
    (28.6, 36),
    (34.2, 42),
    (47.3, 52),
    (57.8, 58),
    (70.1, 65),
    (83.9, 72),
    (100.0, 80),
]

TEMP_24V_50KHZ = [
    (27.4, 38),
    (31.5, 50),
    (38.3, 62),
    (47.5, 75),
]

TEMP_36V_15KHZ = [
    (26.4, 35),
    (34.3, 43),
    (46.9, 55),
    (57.2, 63),
    (68.3, 71),
    (80.5, 78),
    (90.5, 84),
]

TEMP_36V_40KHZ = [
    (28.4, 42),
    (38.3, 61),
    (44.1, 75),
    (53.9, 88),
]

TEMP_36V_50KHZ = [
    (29.0, 45),
    (30.9, 51),
    (34.6, 60),
    (42.0, 75),
    (51.7, 91),
]

def motor_temp_to_power(x):
    # thermal conducivity is roughly 4C/W
    return (x - 25) / 4.0


def plot_temp(ax, data, label):
    ax.plot([motor_temp_to_power(x[0]) for x in data],
            [x[1] for x in data],
            label=label)


figure, axes = plt.subplots(nrows=1, ncols=2)

plot_temp(axes[0], TEMP_24V_50KHZ, label='50kHz')
plot_temp(axes[0], TEMP_24V_40KHZ, label='40kHz')
plot_temp(axes[0], TEMP_24V_15KHZ, label='15kHz')
axes[0].set_ylabel('FET temperature (C)')
axes[0].grid()
axes[0].set_title('24V')
axes[0].set_ylim(32, 95)
axes[0].set_xlim(0, 19)
axes[0].text(2, 72, '50kHz')
axes[0].text(7, 82, '40kHz')
axes[0].text(10, 67, '15kHz')

plot_temp(axes[1], TEMP_36V_50KHZ, label='50kHz')
plot_temp(axes[1], TEMP_36V_40KHZ, label='40kHz')
plot_temp(axes[1], TEMP_36V_15KHZ, label='15kHz')

axes[1].grid()
axes[1].set_title('36V')
axes[1].set_ylim(32, 95)
axes[1].set_xlim(0, 19)
axes[1].text(2.5, 75, '50kHz')
axes[1].text(5, 72, '40kHz')
axes[1].text(10, 75, '15kHz')

figure.suptitle('FET temperature versus PWM Frequency - moteus r4.8')
figure.supxlabel('Motor Power (W)')
plt.show()
