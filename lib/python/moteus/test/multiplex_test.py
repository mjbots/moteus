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


import io
import math
import unittest

import moteus.multiplex as mp


def consume_register_parser(parser):
    result = []
    while True:
        item = parser.next()
        if item[0]:
            value = parser.read(item[2])
            result.append((item, value))
        else:
            break
    return result


def eval_regparse(data):
    dut = mp.RegisterParser(bytes(data))
    return consume_register_parser(dut)


class MultiplexTest(unittest.TestCase):
    def test_register_parser(self):
        e = eval_regparse
        self.assertEqual(e([]), [])
        self.assertEqual(
            e([0x21, 0x03, 0x96]),
            [ ( (True, 0x03, mp.INT8), -106) ])
        self.assertEqual(
            e([0x22, 0x03, 0x96, 0x15]),
            [ ( (True, 0x03, mp.INT8), -106),
              ( (True, 0x04, mp.INT8), 21)])
        self.assertEqual(
            e([0x20, 0x05, 0x09, 0x02, 0x03, 0x04, 0x05, 0x06]),
            [ ( (True, 0x09, mp.INT8), 2),
              ( (True, 0x0a, mp.INT8), 3),
              ( (True, 0x0b, mp.INT8), 4),
              ( (True, 0x0c, mp.INT8), 5),
              ( (True, 0x0d, mp.INT8), 6),
              ])
        self.assertEqual(
            e([0x25, 0x05, 0x01, 0x02]),
            [ ( (True, 0x05, mp.INT16), 0x0201) ])
        self.assertEqual(
            e([0x24, 0x00, 0x25, 0x05, 0x01, 0x02]),
            [ ( (True, 0x05, mp.INT16), 0x0201) ])

    def test_saturate(self):
        self.assertEqual(mp.saturate(-1000.0, mp.INT8, 1.0), -127)
        self.assertEqual(mp.saturate(1000.0, mp.INT8, 1.0), 127)
        self.assertEqual(mp.saturate(math.nan, mp.INT8, 1.0), -128)
        self.assertEqual(mp.saturate(10.0, mp.INT8, 1.0), 10)
        self.assertEqual(mp.saturate(-15.0, mp.INT8, 1.0), -15)
        self.assertEqual(mp.saturate(0.0, mp.INT8, 1.0), 0)
        self.assertEqual(mp.saturate(10, mp.INT8, 0.1), 100)

        self.assertEqual(mp.saturate(-1000000, mp.INT16, 1.0), -32767)
        self.assertEqual(mp.saturate(math.nan, mp.INT16, 1.0), -32768)
        self.assertEqual(mp.saturate(123, mp.INT16, 1.0), 123)

    def test_write_frame(self):
        buf = io.BytesIO()
        dut = mp.WriteFrame(buf)
        dut.write_mapped(2, 2, 1, 0.5, mp.INT8)
        self.assertEqual(buf.getvalue(), bytes([1]))


if __name__ == '__main__':
    unittest.main()
