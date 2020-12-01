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

import enum
import io

from . import multiplex as mp
from . import command as cmd
from . import fdcanusb

def get_fdcanusb_router():
    """Return a single per-process instance of something that models a
    Router.  This is intended to be monkey-patched in environments
    where the default Fdcanusb isn't appropriate.
    """

    global _global_router

    if _global_router:
        return _global_router

    _global_router = fdcanusb.Fdcanusb()
    return _global_router


_global_router = None
_global_router_factory = get_fdcanusb_router


def get_singleton_router():
    return _global_router_factory()


def set_router_factory(x):
    """Platform specific modules can use this to change the default router
    factory."""

    global _global_router_factory
    _global_router_factory = x


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
    REZERO_STATE = 0x00c
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

    COMMAND_POSITION = 0x020
    COMMAND_VELOCITY = 0x021
    COMMAND_FEEDFORWARD_TORQUE = 0x022
    COMMAND_KP_SCALE = 0x023
    COMMAND_KD_SCALE = 0x024
    COMMAND_POSITION_MAX_TORQUE = 0x025
    COMMAND_STOP_POSITION = 0x026
    COMMAND_TIMEOUT = 0x027

    POSITION_KP = 0x030
    POSITION_KI = 0x031
    POSITION_KD = 0x032
    POSITION_FEEDFORWARD = 0x033
    POSITION_COMMAND = 0x034

    REGISTER_MAP_VERSION = 0x102
    SERIAL_NUMBER = 0x120
    SERIAL_NUMBER1 = 0x120
    SERIAL_NUMBER2 = 0x121
    SERIAL_NUMBER3 = 0x122

    REZERO = 0x130


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


class QueryResolution:
    mode = mp.INT16
    position = mp.INT16
    velocity = mp.INT16
    torque = mp.INT16
    q_current = mp.IGNORE
    d_current = mp.IGNORE
    rezero_state = mp.IGNORE
    voltage = mp.INT8
    temperature = mp.INT8
    fault = mp.INT8


class PositionResolution:
    position = mp.F32
    velocity = mp.F32
    feedforward_torque = mp.F32
    kp_scale = mp.F32
    kd_scale = mp.F32
    maximum_torque = mp.IGNORE
    stop_position = mp.F32
    watchdog_timeout = mp.F32


class Parser(mp.RegisterParser):
    def read_position(self, resolution):
        return self.read_mapped(resolution, 0.01, 0.0001, 0.00001)

    def read_velocity(self, resolution):
        return self.read_mapped(resolution, 0.1, 0.00025, 0.00001)

    def read_torque(self, resolution):
        return self.read_mapped(resolution, 0.5, 0.01, 0.001)

    def read_pwm(self, resolution):
        return self.read_mapped(
            resolution, 1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0)

    def read_voltage(self, resolution):
        return self.read_mapped(resolution, 0.5, 0.1, 0.001)

    def read_temperature(self, resolution):
        return self.read_mapped(resolution, 1.0, 0.1, 0.001)

    def read_time(self, resolution):
        return self.read_mapped(resolution, 0.01, 0.001, 0.000001)

    def read_current(self, resolution):
        return self.read_mapped(resolution, 1.0, 0.1, 0.001)

    def ignore(self, resolution):
        self._offset += mp.resolution_size(resolution)


class Writer(mp.WriteFrame):
    def write_position(self, value, resolution):
        self.write_mapped(value, 0.01, 0.0001, 0.00001, resolution)

    def write_velocity(self, value, resolution):
        self.write_mapped(value, 0.1, 0.00025, 0.00001, resolution)

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


def parse_register(parser, register, resolution):
    if register == Register.MODE:
        return parser.read_int(resolution)
    elif register == Register.POSITION:
        return parser.read_position(resolution)
    elif register == Register.VELOCITY:
        return parser.read_velocity(resolution)
    elif register == Register.TORQUE:
        return parser.read_torque(resolution)
    elif register == Register.Q_CURRENT:
        return parser.read_current(resolution)
    elif register == Register.D_CURRENT:
        return parser.read_current(resolution)
    elif register == Register.REZERO_STATE:
        return parser.read_int(resolution)
    elif register == Register.VOLTAGE:
        return parser.read_voltage(resolution)
    elif register == Register.TEMPERATURE:
        return parser.read_temperature(resolution)
    elif register == Register.FAULT:
        return parser.read_int(resolution)


def parse_reply(data):
    parser = Parser(data)
    result = {}
    while True:
        item = parser.next()
        if not item[0]:
            break
        resolution = item[2]
        register = item[1]
        result[register] = parse_register(parser, register, resolution)
    return result


class Result:
    id = None
    values = []

    def __repr__(self):
        value_str = ', '.join(['{}(0x{:03x}): {}'.format(Register(key).name, key, value)
                              for key, value in self.values.items()])
        return f'{self.id}/{{{value_str}}}'


def make_parser(id):
    def parse(data):
        result = Result()
        result.id = id
        result.values = parse_reply(data)
        return result
    return parse


class Controller:
    """Operates a single moteus controller across some communication
    medium.

    Attributes:
      id: bus ID of the controller
      query_resolution: an instance of moteus.QueryResolution
      position_resolution: an instance of moteus.PositionResolution
      router: something modeling moteus.Router to send commands through
    """

    def __init__(self, id=1,
                 query_resolution=QueryResolution(),
                 position_resolution=PositionResolution(),
                 router=None):
        self.id = id
        self.query_resolution = query_resolution
        self.position_resolution = position_resolution
        self.router = router
        self._parser = make_parser(id)

        # Pre-compute our query string.
        self._query_data = self._make_query_data()

    def _get_router(self):
        if self.router:
            return self.router

        # Try to construct a global singleton using some agreed upon
        # method that is hookable.
        self.router = get_singleton_router()
        return self.router

    def _make_query_data(self):
        buf = io.BytesIO()
        writer = Writer(buf)
        qr = self.query_resolution
        c1 = mp.WriteCombiner(writer, 0x10, int(Register.MODE), [
            qr.mode,
            qr.position,
            qr.velocity,
            qr.torque,
            qr.q_current,
            qr.d_current,
            ])
        for i in range(6):
            c1.maybe_write()

        c2 = mp.WriteCombiner(writer, 0x10, int(Register.REZERO_STATE), [
            qr.rezero_state,
            qr.voltage,
            qr.temperature,
            qr.fault,
        ])
        for i in range(4):
            c2.maybe_write()

        return buf.getvalue()

    def _make_command(self, *, query):
        result = cmd.Command()

        result.destination = self.id
        result.source = 0
        result.reply_required = query
        result.parse = self._parser

        return result

    def make_query(self):
        result = self._make_command(query=True)
        result.data = self._query_data
        return result;

    async def query(self, **kwargs):
        return await self._get_router().cycle([self.make_query(**kwargs)])

    def make_stop(self, *, query=False):
        """Return a moteus.Command structure with data necessary to send a
        stop mode command."""

        result = self._make_command(query=query)

        data_buf = io.BytesIO()
        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_int8(int(Register.MODE))
        writer.write_int8(int(Mode.STOPPED))

        if query:
            data_buf.write(self._query_data)

        result.data = data_buf.getvalue()

        return result

    async def set_stop(self, *args, **kwargs):
        return await self._get_router().cycle([self.make_stop(**kwargs)])

    def make_position(self,
                      *,
                      position=None,
                      velocity=None,
                      feedforward_torque=None,
                      kp_scale=None,
                      kd_scale=None,
                      maximum_torque=None,
                      stop_position=None,
                      watchdog_timeout=None,
                      query=False):
        """Return a moteus.Command structure with data necessary to send a
        position mode command with the given values."""

        result = self._make_command(query=query)

        pr = self.position_resolution
        resolutions = [
            pr.position if position else mp.IGNORE,
            pr.velocity if velocity else mp.IGNORE,
            pr.feedforward_torque if feedforward_torque else mp.IGNORE,
            pr.kp_scale if kp_scale else mp.IGNORE,
            pr.kd_scale if kd_scale else mp.IGNORE,
            pr.maximum_torque if maximum_torque else mp.IGNORE,
            pr.stop_position if stop_position else mp.IGNORE,
            pr.watchdog_timeout if watchdog_timeout else mp.IGNORE,
        ]

        data_buf = io.BytesIO()

        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_int8(int(Register.MODE))
        writer.write_int8(int(Mode.POSITION))

        combiner = mp.WriteCombiner(
            writer, 0x00, int(Register.COMMAND_POSITION), resolutions)

        if combiner.maybe_write():
            writer.write_position(position, pr.position)
        if combiner.maybe_write():
            writer.write_velocity(velocity, pr.velocity)
        if combiner.maybe_write():
            writer.write_torque(feedforward_torque, pr.feedforward_torque)
        if combiner.maybe_write():
            writer.write_pwm(kp_scale, pr.kp_scale)
        if combiner.maybe_write():
            writer.write_pwm(kd_scale, pr.kd_scale)
        if combiner.maybe_write():
            writer.write_torque(maximum_torque, pr.maximum_torque)
        if combiner.maybe_write():
            writer.write_position(stop_position, pr.stop_position)
        if combiner.maybe_write():
            writer.write_time(watchdog_timeout, pr.watchdog_timeout)

        if query:
            data_buf.write(self._query_data)

        result.data = data_buf.getvalue()

        return result

    async def set_position(self, *args, **kwargs):
        return await self._get_router().cycle([self.make_position(**kwargs)])
