#!/usr/bin/python3 -B

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

'''Demonstrates how to interact with the moteus controller using a
fdcanusb transport and the multiplex register protocol.

It commands a position sine wave while reporting the status of the
device.
'''

import argparse
import enum
import io
import math
import serial
import struct
import time

# These constants can be found in:
# https://github.com/mjbots/moteus/blob/master/docs/reference.md under
# "register command set".

MP_INT8 = 0
MP_INT16 = 1
MP_INT32 = 2
MP_F32 = 3

MP_WRITE_BASE = 0x00
MP_READ_BASE = 0x10
MP_REPLY_BASE = 0x20
MP_WRITE_ERROR = 0x30
MP_READ_ERROR = 0x31
MP_NOP = 0x50

_TYPE_STRUCTS = {
    MP_INT8: struct.Struct('<b'),
    MP_INT16: struct.Struct('<h'),
    MP_INT32: struct.Struct('<i'),
    MP_F32: struct.Struct('<f'),
}

MOTEUS_REG_MODE = 0x000
MOTEUS_REG_POSITION = 0x001
MOTEUS_REG_VELOCITY = 0x002
MOTEUS_REG_TORQUE = 0x003
MOTEUS_REG_Q_A = 0x004
MOTEUS_REG_D_A = 0x005
MOTEUS_REG_V = 0x00d
MOTEUS_REG_TEMP_C = 0x00e
MOTEUS_REG_FAULT = 0x00f

MOTEUS_REG_POS_POSITION = 0x20
MOTEUS_REG_POS_VELOCITY = 0x21
MOTEUS_REG_POS_TORQUE = 0x22

class MoteusMode(enum.IntEnum):
    STOPPED = 0
    FAULT = 1
    PWM = 5
    VOLTAGE = 6
    VOLTAGE_FOC = 7
    VOLTAGE_DQ = 8
    CURRENT = 9
    POSITION = 10
    TIMEOUT = 11
    ZERO_VEL = 12


def hexify(data):
    return ''.join(['{:02x}'.format(x) for x in data])


def dehexify(data):
    result = b''
    for i in range(0, len(data), 2):
        result += bytes([int(data[i:i+2], 16)])
    return result


def readline(stream):
    result = bytearray()
    while True:
        char = stream.read(1)
        if char == b'\n':
            if len(result):
                return result
        else:
            result += char


def read_varuint(stream):
    result = 0
    shift = 0

    for i in range(5):
        data = stream.read(1)
        if len(data) < 1:
            return None
        this_byte, = struct.unpack('<B', data)
        result |= (this_byte & 0x7f) << shift
        shift += 7

        if (this_byte & 0x80) == 0:
            return result

    assert False


def read_type(stream, field_type):
    s = _TYPE_STRUCTS[field_type]
    data = stream.read(s.size)
    return s.unpack(data)[0]


def parse_register_reply(data):
    stream = io.BytesIO(data)
    result = {}

    while True:
        opcode = read_varuint(stream)
        if opcode is None:
            break
        opcode_base = opcode & ~0x0f
        if opcode_base == MP_REPLY_BASE:
            field_type = (opcode & 0x0c) >> 2
            size = opcode & 0x03
            if size == 0:
                size = read_varuint(stream)
            start_reg = read_varuint(stream)
            for i in range(size):
                result[start_reg + i] = read_type(stream, field_type)
        elif opcode_base == MP_WRITE_ERROR:
            reg = read_varuint(stream)
            err = read_varuint(stream)
            result[reg] = 'werr {}'.format(err)
        elif opcode_base == MP_READ_ERROR:
            reg = read_varuint(stream)
            err = read_varuint(stream)
            result[reg] = 'rerr {}'.format(err)
        elif opcode_base == MP_NOP:
            pass
        else:
            # Unknown opcode.  Just bail.
            break

    return result


class Spinner:
    def __init__(self, args):
        self.serial = serial.Serial(port=args.device)
        self.target = args.target

        # Send a stop to begin with, in case we have a fault or
        # something.  The fault states are latching, and require a
        # stop command in order to make the device move again.
        self.send_can_frame(self.construct_stop(), reply=False)

        # Read the "OK" response from the fdcanusb.
        readline(self.serial)

    def construct_stop(self):
        buf = io.BytesIO()
        buf.write(struct.pack(
            "<bbb",
            0x01,  # write int8 1x
            MOTEUS_REG_MODE,
            MoteusMode.STOPPED))
        return buf.getvalue()

    def construct_position(self):
        buf = io.BytesIO()
        buf.write(struct.pack(
            "<bbb",
            0x01,  # write int8 1x
            MOTEUS_REG_MODE,
            MoteusMode.POSITION))
        buf.write(struct.pack(
            "<bbfff",
            0x0f,  # write float32 3x
            MOTEUS_REG_POS_POSITION,
            self.angle_deg / 360.0,  # position
            self.velocity_dps / 360.0,  # velocity
            0.0,  # feedforward torque
            ))
        buf.write(struct.pack(
            "<bbb",
            0x1c,  # read float32 (variable number)
            4,     # 4 registers
            0x00   # starting at 0
            ))
        buf.write(struct.pack(
            "<bb",
            0x13,  # read int8 3x
            MOTEUS_REG_V))
        return buf.getvalue()

    def step(self):
        self.phase = time.time() % (2. * math.pi);
        self.angle_deg = 20.0 * math.sin(self.phase)
        self.velocity_dps = 20.0 * math.cos(self.phase)

        raw_frame = self.construct_position()
        self.send_can_frame(raw_frame, reply=True)

        # Read (and discard) the adapters response.
        ok_response = readline(self.serial)
        if not ok_response.startswith(b"OK"):
            raise RuntimeError("fdcanusb responded with: " +
                               ok_response.decode('latin1'))

        # Read the devices response.
        device = readline(self.serial)

        if not device.startswith(b"rcv"):
            raise RuntimeError("unexpected response")

        fields = device.split(b" ")
        response = dehexify(fields[2])
        response_data = parse_register_reply(response)

        print("Mode: {: 2d}  Pos: {: 6.2f}deg  Vel: {: 6.2f}dps  "
              "Torque: {: 6.2f}Nm  Temp: {: 3d}C  Voltage: {: 3.1f}V    ".format(
                  int(response_data[MOTEUS_REG_MODE]),
                  response_data[MOTEUS_REG_POSITION] * 360.0,
                  response_data[MOTEUS_REG_VELOCITY] * 360.0,
                  response_data[MOTEUS_REG_TORQUE],
                  response_data[MOTEUS_REG_TEMP_C],
                  response_data[MOTEUS_REG_V] * 0.5),
              end = '\r')

    def send_can_frame(self, frame, reply):
        self.serial.write("can send {:02x}{:02x} {}\n".format(
            0x80 if reply else 0x00,
            self.target, hexify(frame)).encode('latin1'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('-d', '--device', type=str, default='/dev/fdcanusb',
                        help='serial device')
    parser.add_argument('-t', '--target', type=int, default=1,
                        help='ID of target device')
    args = parser.parse_args()

    spinner = Spinner(args)

    while True:
        # Just run as fast as we can.
        spinner.step()


if __name__ == '__main__':
    main()
