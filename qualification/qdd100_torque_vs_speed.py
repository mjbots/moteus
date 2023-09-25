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

import matplotlib
import matplotlib.pyplot as plt


def main():
    plt.plot([0, 1300, 3700],
             [16, 16, 0])
    plt.xlabel('Speed (dps)')
    plt.ylabel('Torque (Nm)')
    plt.grid()
    plt.title('qdd100 beta 2 Peak Torque vs Speed at 36V')

    plt.show()


if __name__ == '__main__':
    main()
