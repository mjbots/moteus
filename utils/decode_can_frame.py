#!/usr/bin/python3 -B

# Copyright 2022 Josh Pieper, jjp@pobox.com.
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

import argparse
import enum
import moteus
import struct

class Command(enum.IntEnum):
    WRITE_REGISTERS = 0x00
    READ_REGISTERS = 0x10
    REPLY = 0x20
    WRITE_ERROR = 0x30
    READ_ERROR = 0x31
    STREAM_CLIENT_TO_SERVER = 0x40
    STREAM_SERVER_TO_CLIENT = 0x41
    STREAM_CLIENT_POLL_SERVER = 0x42
    NOP = 0x50

class Type(enum.IntEnum):
    INT8 = 0
    INT16 = 1
    INT32 = 2
    F32 = 3

class Stream:
    def __init__(self, data):
        self.data = data

    def remaining(self):
        return len(self.data)

    def _read_byte(self):
        result, self.data = self.data[0:1], self.data[1:]
        return result[0]

    def _read_value(self, size):
        result, self.data = self.data[0:size], self.data[size:]
        return result

    def read_struct(self, fmt):
        s = struct.Struct(fmt)
        data = self._read_value(s.size)
        return data, s.unpack(data)[0]

    def read_int8(self):
        return self.read_struct('<b')

    def read_int16(self):
        return self.read_struct('<h')

    def read_int32(self):
        return self.read_struct('<i')

    def read_f32(self):
        return self.read_struct('<f')

    def read_varuint(self):
        result_number = 0
        result_data = bytes([])
        shift = 0
        for i in range(5):
            this_byte = self._read_byte()
            result_data = result_data + bytes([this_byte])
            result_number |= (this_byte & 0x7f) << shift
            shift += 7

            if (this_byte & 0x80) == 0:
                return result_data, result_number

        raise RuntimeError(f'Invalid varuint {result_data.hex()}')

    def read_type(self, typecode):
        if typecode == int(Type.INT8):
            return self.read_int8()
        elif typecode == int(Type.INT16):
            return self.read_int16()
        elif typecode == int(Type.INT32):
            return self.read_int32()
        elif typecode == int(Type.F32):
            return self.read_f32()
        raise RuntimeError(f'Unknown type: {typecode}')


def format_reg(reg):
    try:
        typedreg = moteus.Register(reg)
        return f'0x{reg:03x}({typedreg.name})'
    except TypeError:
        return f'0x{reg:03x}'

def IsNan(typecode, value):
    if typecode == Type.INT8 and value == -(2**7):
        return True
    elif typecode == Type.INT16 and value == -(2**15):
        return True
    elif typecode == Type.INT32 and value == -(2**31):
        return True
    return False


def format_scaled(typecode, value, int8_scale, int16_scale, int32_scale):
    if IsNan(typecode, value):
        return f'{value} (NaN)'
    if typecode == Type.INT8:
        return f'{value} ({value * int8_scale})'
    elif typecode == Type.INT16:
        return f'{value} ({value * int16_scale})'
    elif typecode == Type.INT32:
        return f'{value} ({value * int32_scale})'
    else:
        return f'{value}'

class ScaleType:
    def __init__(self, registers, int8_scale, int16_scale, int32_scale):
        self.registers = registers
        self.int8_scale = int8_scale
        self.int16_scale = int16_scale
        self.int32_scale = int32_scale

SCALE_TYPES = [
    ScaleType([moteus.Register.POSITION,
               moteus.Register.ABS_POSITION,
               moteus.Register.COMMAND_POSITION,
               moteus.Register.COMMAND_STOP_POSITION,
               moteus.Register.COMMAND_WITHIN_LOWER_BOUND,
               moteus.Register.ENCODER_0_POSITION,
               moteus.Register.ENCODER_1_POSITION,
               moteus.Register.ENCODER_2_POSITION,],
              0.01, 0.0001, 0.00001),
    ScaleType([moteus.Register.VELOCITY,
               moteus.Register.COMMAND_VELOCITY,
               moteus.Register.COMMAND_VELOCITY_LIMIT,
               moteus.Register.ENCODER_0_VELOCITY,
               moteus.Register.ENCODER_1_VELOCITY,
               moteus.Register.ENCODER_2_VELOCITY,],
              0.1, 0.00025, 0.00001),
    ScaleType([moteus.Register.TORQUE,
               moteus.Register.COMMAND_FEEDFORWARD_TORQUE,
               moteus.Register.COMMAND_POSITION_MAX_TORQUE,
               moteus.Register.POSITION_FEEDFORWARD,
               moteus.Register.POSITION_COMMAND,
               moteus.Register.COMMAND_WITHIN_FEEDFORWARD_TORQUE,
               moteus.Register.COMMAND_WITHIN_MAX_TORQUE],
              0.5, 0.01, 0.001),
    ScaleType([moteus.Register.Q_CURRENT,
               moteus.Register.D_CURRENT,
               moteus.Register.COMMAND_Q_CURRENT,
               moteus.Register.COMMAND_D_CURRENT],
              1.0, 0.1, 0.001),
    ScaleType([moteus.Register.VOLTAGE,
               moteus.Register.VOLTAGE_PHASE_A,
               moteus.Register.VOLTAGE_PHASE_B,
               moteus.Register.VOLTAGE_PHASE_C,
               moteus.Register.VFOC_VOLTAGE,
               moteus.Register.VOLTAGEDQ_D,
               moteus.Register.VOLTAGEDQ_Q],
              0.5, 0.1, 0.001),
    ScaleType([moteus.Register.TEMPERATURE],
              1.0, 0.1, 0.001),
    ScaleType([moteus.Register.PWM_PHASE_A,
               moteus.Register.PWM_PHASE_B,
               moteus.Register.PWM_PHASE_C,
               moteus.Register.COMMAND_KP_SCALE,
               moteus.Register.COMMAND_KD_SCALE,
               moteus.Register.COMMAND_WITHIN_KP_SCALE,
               moteus.Register.COMMAND_WITHIN_KD_SCALE],
              1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0),
    ScaleType([moteus.Register.COMMAND_ACCEL_LIMIT],
              0.05, 0.001, 0.00001),
    ScaleType([moteus.Register.COMMAND_TIMEOUT,
               moteus.Register.COMMAND_WITHIN_TIMEOUT],
              0.01, 0.001, 0.000001),
]



def format_value(reg, typecode, value):
    if reg == 0:
        try:
            return f'{value}({moteus.Mode(value).name})'
        except ValueError:
            return f'{value}'
        except TypeError:
            return f'{value}'
    for scale_type in SCALE_TYPES:
        if reg in scale_type.registers:
            return format_scaled(typecode, value,
                                 scale_type.int8_scale,
                                 scale_type.int16_scale,
                                 scale_type.int32_scale)
    return f'{value}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('hexcan', nargs='*',
                        help='Hex encoded CAN frame')
    args = parser.parse_args()

    stream = Stream(bytes.fromhex(''.join(args.hexcan)))

    while stream.remaining():
        data, cmd = stream.read_int8()

        print(f'{data.hex()} - ', end='')
        upper = cmd & 0xf0
        maybe_type = (cmd & 0b00001100) >> 2
        maybe_num = cmd & 0b00000011

        if upper == int(Command.READ_REGISTERS):
            print(f'READ_REGISTERS - {Type(maybe_type).name} ', end='')
            if maybe_num > 0:
                length = maybe_num
                print(f'{length} registers')
            else:
                length_data, length = stream.read_varuint()
                print(f'\n {length_data.hex()} - {length} registers')
            start_reg_data, start_reg = stream.read_varuint()
            print(f'  {start_reg_data.hex()} - Starting at reg {format_reg(start_reg)}')
        elif (upper == int(Command.WRITE_REGISTERS) or
              upper == int(Command.REPLY)):
            print(f'{Command(upper).name} - {str(Type(maybe_type))} ', end='')
            if maybe_num > 0:
                length = maybe_num
                print(f'{length} registers')
            else:
                length_data, length = stream.read_varuint()
                print(f'\n {length_data.hex()} - {length} registers')

            start_reg_data, start_reg = stream.read_varuint()
            print(f'  {start_reg_data.hex()} - Starting at reg {format_reg(start_reg)}')

            cur_reg = start_reg
            for i in range(length):
                data, value = stream.read_type(maybe_type)
                print(f'   {data.hex()} - Reg {format_reg(cur_reg)} = {format_value(cur_reg, maybe_type, value)}')
                cur_reg += 1
        elif (cmd == Command.READ_ERROR or
              cmd == Command.WRITE_ERROR):
            print(f'{Command(cmd).name}')
            data, value = stream.read_varuint()
            print(f'  {data.hex()} - register {format_reg(value)}')
            data, value = stream.read_varuint()
            print(f'  {data.hex()} - error {value}')
        elif (cmd == Command.STREAM_CLIENT_TO_SERVER or
              cmd == Command.STREAM_SERVER_TO_CLIENT):
            print(f'{Command(cmd).name}')
            channel_data, channel = stream.read_varuint()
            print(f'  {channel_data.hex()} - channel {channel}')
            nbytes_data, nbytes = stream.read_varuint()
            print(f'  {nbytes_data.hex()} - {nbytes} bytes')
            data = stream._read_value(nbytes)
            print(f'  {data.hex()} - {data}')
        elif cmd == Command.STREAM_CLIENT_POLL_SERVER:
            print(f'{Command(cmd).name}')
            channel_data, channel = stream.read_varuint()
            print(f'  {channel_data.hex()} - channel {channel}')
            nbytes_data, nbytes = stream.read_varuint()
            print(f'  {nbytes_data.hex()} - at most {nbytes} bytes')
        elif cmd == Command.NOP:
            print(f'{Command(cmd).name}')


if __name__ == '__main__':
    main()
