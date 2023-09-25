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

import asyncio
import matplotlib.pyplot as plt
import moteus

async def main():
    c = moteus.Controller()
    s = moteus.Stream(s)

    await s.command(b"d enc-scal 0")
    await asyncio.sleep(3.0)
    results = (await s.command(b"d enc-ecal")).decode('utf8')
    data = [[float(a) for a in x.strip().split(' ')] for x in results.split('\n')]
    indices = [x[0] for x in data]
    values = [x[1] for x in data]
    plt.plot(indices, values)
    plt.show()


if __name__ == '__main__':
    asyncio.run(main())
