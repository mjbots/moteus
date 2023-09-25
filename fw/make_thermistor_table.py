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


import math

def temp(counts):
    v = 3.3 * max(1, counts) / 4096.0
    B = 4050.0
    R = 10000.0

    r_t = 3.3 * R / v - R
    return 1.0 / (1.0 / (273.15 + 25.0) + (1.0 / B) * math.log(r_t / 47000)) - 273.15


def main():
    for x in range(0, 4096, 128):
        print("  {:.2f}f, // {}".format(temp(x), x))


if __name__ == '__main__':
    main()
