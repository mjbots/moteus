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

import enum
import io
import math
import struct
from typing import NamedTuple, Any, Dict, List, Tuple

from . import multiplex as mp

class Register(enum.IntEnum):
    """These are the registers which are exposed for reading or writing
    from the moteus controller.

    The full list can be found at:
    https://github.com/mjbots/moteus/blob/main/docs/reference.md#a2b-registers

    """

    MODE = 0x000
    POSITION = 0x001
    VELOCITY = 0x002
    TORQUE = 0x003
    Q_CURRENT = 0x004
    D_CURRENT = 0x005
    ABS_POSITION = 0x006
    POWER = 0x007
    MOTOR_TEMPERATURE = 0x00a
    TRAJECTORY_COMPLETE = 0x00b
    REZERO_STATE = 0x00c
    HOME_STATE = 0x00c
    VOLTAGE = 0x00d
    TEMPERATURE = 0x00e
    FAULT = 0x00f

    PWM_PHASE_A = 0x010
    PWM_PHASE_B = 0x011
    PWM_PHASE_C = 0x012

    VOLTAGE_PHASE_A = 0x014
    VOLTAGE_PHASE_B = 0x015
    VOLTAGE_PHASE_C = 0x016

    VFOC_THETA = 0x018
    VFOC_VOLTAGE = 0x019
    VOLTAGEDQ_D = 0x01a
    VOLTAGEDQ_Q = 0x01b

    COMMAND_Q_CURRENT = 0x01c
    COMMAND_D_CURRENT = 0x01d

    VFOC_THETA_RATE = 0x01e

    COMMAND_POSITION = 0x020
    COMMAND_VELOCITY = 0x021
    COMMAND_FEEDFORWARD_TORQUE = 0x022
    COMMAND_KP_SCALE = 0x023
    COMMAND_KD_SCALE = 0x024
    COMMAND_POSITION_MAX_TORQUE = 0x025
    COMMAND_STOP_POSITION = 0x026
    COMMAND_TIMEOUT = 0x027
    COMMAND_VELOCITY_LIMIT = 0x028
    COMMAND_ACCEL_LIMIT = 0x029
    COMMAND_FIXED_VOLTAGE_OVERRIDE = 0x02a
    COMMAND_ILIMIT_SCALE = 0x02b
    COMMAND_FIXED_CURRENT_OVERRIDE = 0x02c
    COMMAND_IGNORE_POSITION_BOUNDS = 0x02d

    POSITION_KP = 0x030
    POSITION_KI = 0x031
    POSITION_KD = 0x032
    POSITION_FEEDFORWARD = 0x033
    POSITION_COMMAND = 0x034

    CONTROL_POSITION = 0x038
    CONTROL_VELOCITY = 0x039
    CONTROL_TORQUE = 0x03a
    POSITION_ERROR = 0x03b
    VELOCITY_ERROR = 0x03c
    TORQUE_ERROR = 0x03d

    COMMAND_WITHIN_LOWER_BOUND = 0x040
    COMMAND_WITHIN_UPPER_BOUND = 0x041
    COMMAND_WITHIN_FEEDFORWARD_TORQUE = 0x042
    COMMAND_WITHIN_KP_SCALE = 0x043
    COMMAND_WITHIN_KD_SCALE = 0x044
    COMMAND_WITHIN_MAX_TORQUE = 0x045
    COMMAND_WITHIN_TIMEOUT = 0x046
    COMMAND_WITHIN_ILIMIT_SCALE = 0x047
    COMMAND_WITHIN_IGNORE_POSITION_BOUNDS = 0x048

    ENCODER_0_POSITION = 0x050
    ENCODER_0_VELOCITY = 0x051
    ENCODER_1_POSITION = 0x052
    ENCODER_1_VELOCITY = 0x053
    ENCODER_2_POSITION = 0x054
    ENCODER_2_VELOCITY = 0x055

    ENCODER_VALIDITY = 0x058
    AUX1_GPIO_COMMAND = 0x05c
    AUX2_GPIO_COMMAND = 0x05d
    AUX1_GPIO_STATUS = 0x05e
    AUX2_GPIO_STATUS = 0x05f

    AUX1_ANALOG_IN1 = 0x060
    AUX1_ANALOG_IN2 = 0x061
    AUX1_ANALOG_IN3 = 0x062
    AUX1_ANALOG_IN4 = 0x063
    AUX1_ANALOG_IN5 = 0x064

    AUX2_ANALOG_IN1 = 0x068
    AUX2_ANALOG_IN2 = 0x069
    AUX2_ANALOG_IN3 = 0x06a
    AUX2_ANALOG_IN4 = 0x06b
    AUX2_ANALOG_IN5 = 0x06c

    MILLISECOND_COUNTER = 0x070
    CLOCK_TRIM = 0x071

    AUX1_PWM1 = 0x076,
    AUX1_PWM2 = 0x077,
    AUX1_PWM3 = 0x078,
    AUX1_PWM4 = 0x079,
    AUX1_PWM5 = 0x07a,
    AUX2_PWM1 = 0x07b,
    AUX2_PWM2 = 0x07c,
    AUX2_PWM3 = 0x07d,
    AUX2_PWM4 = 0x07e,
    AUX2_PWM5 = 0x07f,

    MODEL_NUMBER = 0x0100
    FIRMWARE_VERSION = 0x101
    REGISTER_MAP_VERSION = 0x102
    MULTIPLEX_ID = 0x110

    SERIAL_NUMBER = 0x120
    SERIAL_NUMBER1 = 0x120
    SERIAL_NUMBER2 = 0x121
    SERIAL_NUMBER3 = 0x122

    REZERO = 0x130
    SET_OUTPUT_NEAREST = 0x130
    SET_OUTPUT_EXACT = 0x131
    REQUIRE_REINDEX = 0x132
    RECAPTURE_POSITION_VELOCITY = 0x133

    DRIVER_FAULT1 = 0x140
    DRIVER_FAULT2 = 0x141

    UUID1 = 0x150
    UUID2 = 0x151
    UUID3 = 0x152
    UUID4 = 0x153

    UUID_MASK1 = 0x0154
    UUID_MASK2 = 0x0155
    UUID_MASK3 = 0x0156
    UUID_MASK4 = 0x0157

    UUID_MASK_CAPABLE = 0x0158


class Mode(enum.IntEnum):
    """Valid values for the Register.MODE register"""

    STOPPED = 0
    FAULT = 1
    PWM = 5
    VOLTAGE = 6
    VOLTAGE_FOC = 7
    VOLTAGE_DQ = 8
    CURRENT = 9
    POSITION = 10
    TIMEOUT = 11
    ZERO_VELOCITY = 12
    STAY_WITHIN = 13
    MEASURE_IND = 14
    BRAKE = 15


class Writer(mp.WriteFrame):
    def write_position(self, value, resolution):
        self.write_mapped(value, 0.01, 0.0001, 0.00001, resolution)

    def write_velocity(self, value, resolution):
        self.write_mapped(value, 0.1, 0.00025, 0.00001, resolution)

    def write_accel(self, value, resolution):
        self.write_mapped(value, 0.05, 0.001, 0.00001, resolution)

    def write_torque(self, value, resolution):
        self.write_mapped(value, 0.5, 0.01, 0.001, resolution)

    def write_pwm(self, value, resolution):
        self.write_mapped(value,
                          1.0 / 127.0,
                          1.0 / 32767.0,
                          1.0 / 2147483647.0,
                          resolution)

    def write_voltage(self, value, resolution):
        self.write_mapped(value, 0.5, 0.1, 0.001, resolution)

    def write_temperature(self, value, resolution):
        self.write_mapped(value, 1.0, 0.1, 0.001, resolution)

    def write_time(self, value, resolution):
        self.write_mapped(value, 0.01, 0.001, 0.000001, resolution)

    def write_current(self, value, resolution):
        self.write_mapped(value, 1.0, 0.1, 0.001, resolution)

    def write_power(self, value, resolution):
        self.write_mapped(value, 10.0, 0.05, 0.0001, resolution)

    def write_int(self, value, resolution):
        self.write_mapped(int(value), 1, 1, 1, resolution)


def _nanify(value, resolution):
    """Convert the protocol's special "minimum value" to NaN."""
    if resolution == mp.INT8 and value == -128:
        return math.nan
    elif resolution == mp.INT16 and value == -32768:
        return math.nan
    elif resolution == mp.INT32 and value == -2147483648:
        return math.nan
    return value


def _scale_mapped(value, resolution, int8_scale, int16_scale, int32_scale):
    """Apply scaling to a raw value based on resolution."""
    scales = [int8_scale, int16_scale, int32_scale, 1.0]
    return _nanify(value, resolution) * scales[resolution]


def scale_register(register, resolution, value):
    """Apply moteus-specific scaling to a raw register value.

    Args:
        register: Register number (e.g., Register.POSITION)
        resolution: Resolution type (INT8, INT16, INT32, or F32)
        value: Raw decoded value from the frame

    Returns:
        Scaled value in physical units (revolutions, rev/s, Nm, etc.)
    """
    if value is None:
        return None

    if register == Register.MODE:
        return int(value)
    elif register == Register.POSITION:
        return _scale_mapped(value, resolution, 0.01, 0.0001, 0.00001)
    elif register == Register.VELOCITY:
        return _scale_mapped(value, resolution, 0.1, 0.00025, 0.00001)
    elif register == Register.TORQUE:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.Q_CURRENT:
        return _scale_mapped(value, resolution, 1.0, 0.1, 0.001)
    elif register == Register.D_CURRENT:
        return _scale_mapped(value, resolution, 1.0, 0.1, 0.001)
    elif register == Register.ABS_POSITION:
        return _scale_mapped(value, resolution, 0.01, 0.0001, 0.00001)
    elif register == Register.POWER:
        return _scale_mapped(value, resolution, 10.0, 0.05, 0.0001)
    elif register == Register.TRAJECTORY_COMPLETE:
        return int(value)
    elif register == Register.HOME_STATE or register == Register.REZERO_STATE:
        return int(value)
    elif register == Register.VOLTAGE:
        return _scale_mapped(value, resolution, 0.5, 0.1, 0.001)
    elif register == Register.MOTOR_TEMPERATURE:
        return _scale_mapped(value, resolution, 1.0, 0.1, 0.001)
    elif register == Register.TEMPERATURE:
        return _scale_mapped(value, resolution, 1.0, 0.1, 0.001)
    elif register == Register.FAULT:
        return int(value)
    elif register == Register.POSITION_KP:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.POSITION_KI:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.POSITION_KD:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.POSITION_FEEDFORWARD:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.POSITION_COMMAND:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.CONTROL_POSITION:
        return _scale_mapped(value, resolution, 0.01, 0.0001, 0.00001)
    elif register == Register.CONTROL_VELOCITY:
        return _scale_mapped(value, resolution, 0.1, 0.00025, 0.00001)
    elif register == Register.CONTROL_TORQUE:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.POSITION_ERROR:
        return _scale_mapped(value, resolution, 0.01, 0.0001, 0.00001)
    elif register == Register.VELOCITY_ERROR:
        return _scale_mapped(value, resolution, 0.1, 0.00025, 0.00001)
    elif register == Register.TORQUE_ERROR:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.ENCODER_0_POSITION:
        return _scale_mapped(value, resolution, 0.01, 0.0001, 0.00001)
    elif register == Register.ENCODER_0_VELOCITY:
        return _scale_mapped(value, resolution, 0.1, 0.00025, 0.00001)
    elif register == Register.ENCODER_1_POSITION:
        return _scale_mapped(value, resolution, 0.01, 0.0001, 0.00001)
    elif register == Register.ENCODER_1_VELOCITY:
        return _scale_mapped(value, resolution, 0.1, 0.00025, 0.00001)
    elif register == Register.ENCODER_2_POSITION:
        return _scale_mapped(value, resolution, 0.01, 0.0001, 0.00001)
    elif register == Register.ENCODER_2_VELOCITY:
        return _scale_mapped(value, resolution, 0.1, 0.00025, 0.00001)
    elif register == Register.ENCODER_VALIDITY:
        return int(value)
    elif register == Register.AUX1_GPIO_COMMAND:
        return int(value)
    elif register == Register.AUX2_GPIO_COMMAND:
        return int(value)
    elif register == Register.AUX1_GPIO_STATUS:
        return int(value)
    elif register == Register.AUX2_GPIO_STATUS:
        return int(value)
    elif (register == Register.AUX1_ANALOG_IN1 or
          register == Register.AUX1_ANALOG_IN2 or
          register == Register.AUX1_ANALOG_IN3 or
          register == Register.AUX1_ANALOG_IN4 or
          register == Register.AUX1_ANALOG_IN5 or
          register == Register.AUX2_ANALOG_IN1 or
          register == Register.AUX2_ANALOG_IN2 or
          register == Register.AUX2_ANALOG_IN3 or
          register == Register.AUX2_ANALOG_IN4 or
          register == Register.AUX2_ANALOG_IN5):
        return _scale_mapped(value, resolution, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0)
    elif register == Register.MILLISECOND_COUNTER:
        return int(value)
    elif register == Register.CLOCK_TRIM:
        return int(value)
    elif (register >= Register.AUX1_PWM1 and
          register <= Register.AUX2_PWM5):
        return _scale_mapped(value, resolution, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0)
    # Command registers (0x020-0x02d)
    elif register == Register.COMMAND_POSITION:
        return _scale_mapped(value, resolution, 0.01, 0.0001, 0.00001)
    elif register == Register.COMMAND_VELOCITY:
        return _scale_mapped(value, resolution, 0.1, 0.00025, 0.00001)
    elif register == Register.COMMAND_FEEDFORWARD_TORQUE:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.COMMAND_KP_SCALE:
        return _scale_mapped(value, resolution, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0)
    elif register == Register.COMMAND_KD_SCALE:
        return _scale_mapped(value, resolution, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0)
    elif register == Register.COMMAND_POSITION_MAX_TORQUE:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.COMMAND_STOP_POSITION:
        return _scale_mapped(value, resolution, 0.01, 0.0001, 0.00001)
    elif register == Register.COMMAND_TIMEOUT:
        return _scale_mapped(value, resolution, 0.01, 0.001, 0.000001)
    elif register == Register.COMMAND_VELOCITY_LIMIT:
        return _scale_mapped(value, resolution, 0.1, 0.00025, 0.00001)
    elif register == Register.COMMAND_ACCEL_LIMIT:
        return _scale_mapped(value, resolution, 0.05, 0.001, 0.00001)
    elif register == Register.COMMAND_FIXED_VOLTAGE_OVERRIDE:
        return _scale_mapped(value, resolution, 0.5, 0.1, 0.001)
    elif register == Register.COMMAND_ILIMIT_SCALE:
        return _scale_mapped(value, resolution, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0)
    # Stay-within command registers (0x040-0x048)
    elif register == Register.COMMAND_WITHIN_LOWER_BOUND:
        return _scale_mapped(value, resolution, 0.01, 0.0001, 0.00001)
    elif register == Register.COMMAND_WITHIN_UPPER_BOUND:
        return _scale_mapped(value, resolution, 0.01, 0.0001, 0.00001)
    elif register == Register.COMMAND_WITHIN_FEEDFORWARD_TORQUE:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.COMMAND_WITHIN_KP_SCALE:
        return _scale_mapped(value, resolution, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0)
    elif register == Register.COMMAND_WITHIN_KD_SCALE:
        return _scale_mapped(value, resolution, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0)
    elif register == Register.COMMAND_WITHIN_MAX_TORQUE:
        return _scale_mapped(value, resolution, 0.5, 0.01, 0.001)
    elif register == Register.COMMAND_WITHIN_TIMEOUT:
        return _scale_mapped(value, resolution, 0.01, 0.001, 0.000001)
    elif register == Register.COMMAND_WITHIN_ILIMIT_SCALE:
        return _scale_mapped(value, resolution, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0)
    else:
        # Unknown register, return raw value with NaN handling
        return _nanify(value, resolution)


class ParsedRegisters(NamedTuple):
    """Result of parsing a multiplex frame with scaled register values.

    Attributes:
        command: WRITE subframes as {register: scaled_value}
        response: RESPONSE subframes as {register: scaled_value}
        query: QUERY subframes as [(register, resolution), ...]
        errors: Error subframes as [(register, error_code), ...]
    """
    command: Dict[int, Any]
    response: Dict[int, Any]
    query: List[Tuple[int, int]]
    errors: List[Tuple[int, int]]


def parse_registers(data) -> ParsedRegisters:
    """Parse a multiplex frame, returning scaled register values.

    This function parses all subframes in a multiplex frame and applies
    moteus-specific scaling to register values.

    Args:
        data: Raw multiplex frame bytes

    Returns:
        ParsedRegisters with command, response, query, and errors dicts/lists

    Example:
        result = parse_registers(frame_data)
        if result.response:
            position = result.response.get(Register.POSITION)
            velocity = result.response.get(Register.VELOCITY)
        if result.command:
            cmd_position = result.command.get(Register.COMMAND_POSITION)
    """
    command = {}
    response = {}
    query = []
    errors = []

    for subframe in mp.parse_frame(data):
        if isinstance(subframe, mp.RegisterSubframe):
            if subframe.type == mp.SubframeType.WRITE:
                scaled = scale_register(subframe.register, subframe.resolution, subframe.value)
                command[subframe.register] = scaled
            elif subframe.type == mp.SubframeType.RESPONSE:
                scaled = scale_register(subframe.register, subframe.resolution, subframe.value)
                response[subframe.register] = scaled
            elif subframe.type == mp.SubframeType.READ:
                query.append((subframe.register, subframe.resolution))
        elif isinstance(subframe, mp.ErrorSubframe):
            errors.append((subframe.register, subframe.error_code))

    return ParsedRegisters(command, response, query, errors)


def parse_reply(data):
    return parse_registers(data).response


class Result:
    id = None
    arbitration_id = None
    bus = None
    values = {}

    def __repr__(self):
        value_str = ', '.join(['{}(0x{:03x}): {}'.format(Register(key).name, key, value)
                              for key, value in self.values.items()])
        return f'{self.id}/{{{value_str}}}'

def parse_message(message):
    result = Result()
    result.id = (message.arbitration_id >> 8) & 0x7f
    result.values = parse_reply(message.data)

    # We store these things just for reference, so that our
    # results look a bit like CAN responses too.
    result.arbitration_id = message.arbitration_id
    if hasattr(message, 'bus'):
        result.bus = message.bus
    else:
        result.bus = 1
    result.data = message.data

    return result

def make_uuid_prefix(id):
    if isinstance(id, int) or id.uuid is None:
        return b''

    buf = io.BytesIO()
    writer = Writer(buf)

    reg_count = len(id.uuid) // 4
    assert len(id.uuid) % 4 == 0
    assert len(id.uuid) <= 16 and len(id.uuid) > 0

    combiner = mp.WriteCombiner(
        writer, 0x00, Register.UUID_MASK1, [mp.INT32] * reg_count)

    # Convert UUID prefix bytes to list of 32-bit integers
    # (little-endian)
    i32_data = []
    for i in range(0, len(id.uuid), 4):
        chunk = id.uuid[i:i+4]
        i32_val = struct.unpack('<i', chunk)[0]
        i32_data.append(i32_val)

    for i32_val in i32_data:
        if combiner.maybe_write():
            writer.write_int32(i32_val)

    return buf.getvalue()
