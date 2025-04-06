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
import numpy



COLOR_TABLE = {
    'r411' : 'blue',
    'c1'   : 'orange',
    'n1'   : 'green',
    'x1'   : 'red',
}

MOTOR_TABLE = {
    'mj5208' : 'mj5208',
    'mad8318' : 'MAD 8318',
    'gl80' : 'T-Motor GL80',
    'gbm5208' : 'GBM5208',
}

OLD_MT_OLD_FW_DATA = [
    # motor, controller, R_mean/std, L_mean/std, Kv_mean/std
    ('mj5208',  'r411', 11.38, 1.07, -17.12, 1.73, -4.37, 0.69),
    ('mj5208',  'c1',  -12.29, 0.66, -20.10, 6.36, 10.07, 0.43),
    ('mj5208',  'n1',   -8.56, 1.55, -24.85, 0.51,  6.29, 0.38),

    ('mad8318', 'r411', 24.83, 1.35, -8.42, 1.19, -14.17, 2.15),
    ('mad8318', 'c1',   -9.33, 0.42, -3.35, 1.57, 11.87, 0.65),
    ('mad8318', 'n1',    8.67, 0.66, -11.26, 1.64, -15.01, 0.13),

    ('gbm5208', 'r411', -2.70, 7.98, 60.93, 8.39, -9.46, 0.50),
    ('gbm5208', 'c1',  -26.88, 5.04, 18.45, 4.47, 22.08, 0.22),
    ('gbm5208', 'n1',   1.94, 5.97, 55.12, 4.67, -11.25, 0.51),

    ('ht1105',  'r411', 13.22, 7.09, 1153.5, 14.96, -14.98, 14.78),
    ('ht1105',  'c1',  -12.78, 10.99, 701.09, 14.56, 35.55, 8.66),
    ('ht1105',  'n1',  10.74, 9.39, 1063.40, 4.41, 9.02, 0.24),

    ('gl80', 'c1', -26.65, 2.16, -15.73, 1.18, 25.93, 4.42),
    ('gl80', 'n1', 6.91, 3.41, -.14, 5.86, -7.12, 2.92),
]


NEW_MT_OLD_FW_DATA = [
    # motor, controller, R_mean/std, L_mean/std, Kv_mean/std
    ('mj5208', 'r411', 17.87, 1.34,   9.04, 15.63, -3.59, 1.19),
    ('mj5208', 'c1',  -11.28, 0.53, -22.28, 1.93,   1.06, 0.70),
    ('mj5208', 'n1',    2.23, 1.36, -26.64, 14.40, -4.08, 0.78),

    ('mad8318', 'r411', 17.87, 1.34, 9.04, 15.63, -3.59, 1.19),
    ('mad8318', 'c1',  -11.28, 0.53, -22.28, 1.93, 1.06, 0.70),
    ('mad8318', 'n1',    2.23, 1.36, -26.64, 14.40, -4.08, 0.78),

    ('gbm5208', 'r411', 3.86, 4.77, 38.60, 11.01, 2.07, 1.16),
    ('gbm5208', 'c1',   2.07, 1.16, -25.14, 2.88, -26.18, 11.14),
    ('gbm5208', 'n1',  -3.93, 18.03, 9.38, 6.70, -.11, 1.56),

    ('ht1105', 'r411', 5.48, 19.67, 67.49, 0.50, 12.27, 3.62),
    ('ht1105', 'c1',  -20.73, 16.74, 17.09, 2.10, 8.13, 3.14),
    ('ht1105', 'n1',  2.59, 21.77, 67.35, 1.21, 15.02, 0.88),

    ('gl80', 'c1', -25.88, .99, -2.05, 5.56, 9.56, 8.49),
    ('gl80', 'n1', 10.51, 5.04, 2.27, 5.94, 8.74, 11.43),
]

NEW_MT_NEW_FW_DATA = [
    # motor, controller, R_mean/std, L_mean/std, Kv_mean/std
    ('mj5208', 'r411', 5.76, 1.10, -23.51, 8.24, -0.43, 0.76),
    ('mj5208', 'c1',   0.34, 2.41, -20.07, 8.99,  0.15, 0.65),
    ('mj5208', 'n1',  -0.17, 1.73, -29.48, 2.51, -4.08, 0.62),
    ('mj5208', 'x1',   2.08, 0.84, -13.80, 6.72, -2.13, 0.55),

    ('mad8318', 'r411', 30.22, 1.31, 4.35, 6.81, 1.04, 2.52),
    ('mad8318', 'c1',   55.19, 2.50, 16.72, 4.61, 2.61, 0.79),
    ('mad8318', 'n1',   10.27, 1.57, 2.25, 8.94, -.20, 0.33),
    ('mad8318', 'x1',   2.28, 7.0, 15.38, 5.89, -7.87, 1.01),

    ('gbm5208', 'r411', -.90, 12.29, -23.98, 19.13, 5.20, 2.04),
    ('gbm5208', 'c1',  -14.21, 16.38, -23.20, 7.72, 2.84, 0.39),
    ('gbm5208', 'n1',  -36.19, 21.81, -24.05, 13.31, 4.51, 0.41),
    ('gbm5208', 'x1',   -1.08, 7.52, .94, .73, 2.11, 1.49),

    ('ht1105',  'r411', 18.06, 6.85, 71.63, 2.35, 15.18, 2.75),
    ('ht1105',  'c1',  -14.36, 29.78, 73.14, 1.25, 21.77, 2.12),
    ('ht1105',  'n1',   2.21, 23.86, 71.08, 1.70, 20.10, 0.91),
    ('ht1105',  'x1',   17.68, 22.23, 124.53, 1.44, 22.05, 3.48),

    ('gl80', 'c1', 1.92, 0.82, 0.82, 1.49, 11.04, 8.00),
    ('gl80', 'n1', 6.88, 2.72, 1.83, 0.54, 23.05, 10.13),
    ('gl80', 'x1', 6.76, 1.29, 12.16, 2.40, 6.48, 4.56),
]

MOTORS = {x[0] : True for x in OLD_MT_OLD_FW_DATA}.keys()

fig, axs = plt.subplots(ncols=3, nrows=len(MOTORS), sharey='row', sharex='all')

def plot(ax, x, data, index, *args, **kwargs):
    ax.errorbar(
        [x], [data[2+index*2]], [data[3+index*2]],
        *args, linewidth=3, capsize=6, color=COLOR_TABLE[data[1]],
        **kwargs)

def plot_all(ax, xbounds, data, index, motor, *args, **kwargs):
    mdata = [x for x in data if x[0] == motor]
    step = 1/len(mdata) if len(mdata) > 1 else 1
    for drow, x in zip(mdata, numpy.arange(xbounds[0], xbounds[1], step)):
        plot(ax, x, drow, index)


axs[0, 0].set_title('Resistance')
axs[0, 1].set_title('Inductance')
axs[0, 2].set_title('Kv')

for col in [0, 1, 2]:
    axs[-1, col].set_xticks([0.5, 2.5, 4.5], labels=['old tool /\nold fw', 'new tool /\nold fw', 'new tool /\nnew fw'])

for row, motor in enumerate(MOTORS):
    axs[row, 0].set_ylabel(f'{MOTOR_TABLE.get(motor, motor)}\nPercent (%) error')
    for parameter in [0, 1, 2]:
        ax = axs[row, parameter]

        plot_all(ax, (0, 1), OLD_MT_OLD_FW_DATA, parameter, motor)
        plot_all(ax, (2, 3), NEW_MT_OLD_FW_DATA, parameter, motor)
        plot_all(ax, (4, 5), NEW_MT_NEW_FW_DATA, parameter, motor)

        # Ensure y=0 is on the plot and is highlighted.
        ylim = ax.get_ylim()
        if ylim[0] > 0:
            ax.set_ylim(bottom=-1)
        if ylim[1] < 0:
            ax.set_ylim(top=1)

        ax.grid(axis='y')
        ax.axhline(y=0, lw=3, color='k', zorder=-2)

    if motor == 'ht1105':
        axs[row, 0].set_yscale('symlog')


plt.show()
