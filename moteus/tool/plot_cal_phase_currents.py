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

import matplotlib.pylab as pylab
import sys


def main():
    data = [x.strip() for x in open(sys.argv[1]).readlines()[1:-1]]

    fields = [x.split(' ') for x in data]

    def get(desired_key, data):
        result = []
        for line in data:
            for field in line:
                if '=' not in field:
                    continue
                key, val = field.split('=')
                if key == desired_key:
                    result.append(int(val))
                    break
            else:
                raise RuntimeError(f"Key {desired_key} not found")
        return result
    pylab.plot(get("i1", fields), label='cur1')
    pylab.plot(get("i2", fields), label='cur2')
    pylab.plot(get("i3", fields), label='cur3')
    pylab.legend()
    pylab.show()


if __name__ == '__main__':
    main()
