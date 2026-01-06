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

    def test_query_parser(self):
       self.assertEqual(mp.QueryParser.parse([]), [])
       self.assertEqual(mp.QueryParser.parse([0x11, 0x01]), [(0x001, 0)])
       self.assertEqual(mp.QueryParser.parse([0x12, 0x01]),
                        [(0x001, 0), (0x002, 0)])
       self.assertEqual(mp.QueryParser.parse([0x17, 0x81, 0x03]),
                        [(0x181, 1), (0x182, 1), (0x183, 1)])
       self.assertEqual(mp.QueryParser.parse([0x1f, 0x02, 0x11, 0x08]),
                        [(0x002, 3), (0x003, 3), (0x004, 3), (0x008, 0)])

    def test_parse_frame_empty(self):
        result = list(mp.parse_frame(b''))
        self.assertEqual(result, [])

    def test_parse_frame_response(self):
        # Test RESPONSE frame: 0x21 = REPLY_BASE + INT8 + count 1
        result = list(mp.parse_frame([0x21, 0x03, 0x96]))
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], mp.RegisterSubframe)
        self.assertEqual(result[0].type, mp.SubframeType.RESPONSE)
        self.assertEqual(result[0].register, 0x03)
        self.assertEqual(result[0].resolution, mp.INT8)
        self.assertEqual(result[0].value, -106)

    def test_parse_frame_response_int16(self):
        # Test RESPONSE frame with INT16: 0x25 = REPLY_BASE + INT16 + count 1
        result = list(mp.parse_frame([0x25, 0x05, 0x01, 0x02]))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].type, mp.SubframeType.RESPONSE)
        self.assertEqual(result[0].register, 0x05)
        self.assertEqual(result[0].resolution, mp.INT16)
        self.assertEqual(result[0].value, 0x0201)

    def test_parse_frame_write(self):
        # Test WRITE frame: 0x01 = WRITE_BASE + INT8 + count 1
        result = list(mp.parse_frame([0x01, 0x0a, 0x05]))
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], mp.RegisterSubframe)
        self.assertEqual(result[0].type, mp.SubframeType.WRITE)
        self.assertEqual(result[0].register, 0x0a)
        self.assertEqual(result[0].resolution, mp.INT8)
        self.assertEqual(result[0].value, 5)

    def test_parse_frame_query(self):
        # Test QUERY frame: 0x11 = READ_BASE + INT8 + count 1
        result = list(mp.parse_frame([0x11, 0x01]))
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], mp.RegisterSubframe)
        self.assertEqual(result[0].type, mp.SubframeType.READ)
        self.assertEqual(result[0].register, 0x01)
        self.assertEqual(result[0].resolution, mp.INT8)
        self.assertIsNone(result[0].value)

    def test_parse_frame_multiple_queries(self):
        # Test QUERY frame with count 3: 0x13 = READ_BASE + INT8 + count 3
        result = list(mp.parse_frame([0x13, 0x05]))
        self.assertEqual(len(result), 3)
        for i, sf in enumerate(result):
            self.assertEqual(sf.type, mp.SubframeType.READ)
            self.assertEqual(sf.register, 0x05 + i)
            self.assertIsNone(sf.value)

    def test_parse_frame_write_error(self):
        # Test WRITE_ERROR frame: 0x30 followed by register and error code
        result = list(mp.parse_frame([0x30, 0x05, 0x01]))
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], mp.ErrorSubframe)
        self.assertEqual(result[0].type, mp.SubframeType.WRITE_ERROR)
        self.assertEqual(result[0].register, 0x05)
        self.assertEqual(result[0].error_code, 0x01)

    def test_parse_frame_read_error(self):
        # Test READ_ERROR frame: 0x31 followed by register and error code
        result = list(mp.parse_frame([0x31, 0x0a, 0x02]))
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], mp.ErrorSubframe)
        self.assertEqual(result[0].type, mp.SubframeType.READ_ERROR)
        self.assertEqual(result[0].register, 0x0a)
        self.assertEqual(result[0].error_code, 0x02)

    def test_parse_frame_stream_client_to_server(self):
        # Test STREAM_CLIENT_DATA: 0x40, channel, size, data...
        result = list(mp.parse_frame([0x40, 0x01, 0x03, 0x41, 0x42, 0x43]))
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], mp.StreamSubframe)
        self.assertEqual(result[0].type, mp.SubframeType.STREAM_CLIENT_TO_SERVER)
        self.assertEqual(result[0].channel, 0x01)
        self.assertEqual(result[0].data, b'ABC')

    def test_parse_frame_stream_server_to_client(self):
        # Test STREAM_SERVER_DATA: 0x41, channel, size (varuint), data...
        result = list(mp.parse_frame([0x41, 0x02, 0x02, 0x58, 0x59]))
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], mp.StreamSubframe)
        self.assertEqual(result[0].type, mp.SubframeType.STREAM_SERVER_TO_CLIENT)
        self.assertEqual(result[0].channel, 0x02)
        self.assertEqual(result[0].data, b'XY')

    def test_parse_frame_stream_poll(self):
        # Test STREAM_CLIENT_POLL: 0x42, channel, max_length
        result = list(mp.parse_frame([0x42, 0x03, 0x40]))
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], mp.StreamSubframe)
        self.assertEqual(result[0].type, mp.SubframeType.STREAM_CLIENT_POLL_SERVER)
        self.assertEqual(result[0].channel, 0x03)
        self.assertEqual(result[0].data, bytes([0x40]))  # max_length stored in data

    def test_parse_frame_nop(self):
        # NOP (0x50) should be skipped
        result = list(mp.parse_frame([0x50, 0x50, 0x21, 0x03, 0x05, 0x50]))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].type, mp.SubframeType.RESPONSE)
        self.assertEqual(result[0].register, 0x03)
        self.assertEqual(result[0].value, 5)

    def test_parse_frame_mixed(self):
        # Test a frame with multiple subframes of different types
        # 0x21 0x01 0xff = RESPONSE reg 1, value -1 (INT8)
        # 0x11 0x05 = QUERY reg 5 (INT8)
        data = [0x21, 0x01, 0xff, 0x11, 0x05]
        result = list(mp.parse_frame(data))
        self.assertEqual(len(result), 2)

        self.assertEqual(result[0].type, mp.SubframeType.RESPONSE)
        self.assertEqual(result[0].register, 0x01)
        self.assertEqual(result[0].value, -1)

        self.assertEqual(result[1].type, mp.SubframeType.READ)
        self.assertEqual(result[1].register, 0x05)
        self.assertIsNone(result[1].value)

    def test_parse_frame_empty_response_skipped(self):
        # Empty response (count=0) should be skipped
        # 0x24 0x00 = REPLY INT16 with extended count 0 (empty, skipped)
        # 0x25 0x05 0x01 0x02 = REPLY INT16 count 1, reg 5, value 0x0201
        result = list(mp.parse_frame([0x24, 0x00, 0x25, 0x05, 0x01, 0x02]))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].register, 0x05)
        self.assertEqual(result[0].value, 0x0201)


if __name__ == '__main__':
    unittest.main()
