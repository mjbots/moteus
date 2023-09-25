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
import unittest

from moteus import Router, Command

class FakeTarget:
    def __init__(self, nonce):
        self.nonce = nonce

    async def cycle(self, commands):
        return [(self.nonce, x) for x in commands]


class RouterTest(unittest.TestCase):
    async def run_basic(self):
        a1 = FakeTarget('n1')
        a2 = FakeTarget('n2')
        dut = Router([(a1, [3, 6]), (a2, [1, 4])])

        cmd1 = Command()
        cmd1.data = 'foo3'
        cmd1.destination = 3

        cmd2 = Command()
        cmd2.data = 'foo1'
        cmd2.destination = 1

        cmd3 = Command()
        cmd3.data = 'foo4'
        cmd3.destination = 4

        result = await dut.cycle([cmd1, cmd2, cmd3])

        self.assertEqual(result, [('n1', cmd1), ('n2', cmd2), ('n2', cmd3)])

    def test_basic(self):
        asyncio.get_event_loop().run_until_complete(self.run_basic())


if __name__ == '__main__':
    unittest.main()
