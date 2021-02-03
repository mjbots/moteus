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

import asyncio
import argparse
import enum
import importlib_metadata
import io
import struct

from . import multiplex as mp
from . import command as cmd
from . import fdcanusb
from . import pythoncan

import moteus.reader

class FdcanusbFactory:
    PRIORITY = 10

    name = 'fdcanusb'

    def add_args(self, parser):
        parser.add_argument('--fdcanusb', type=str, metavar='FILE',
                            help='path to fdcanusb device')

    def is_args_set(self, args):
        return args and args.fdcanusb

    def __call__(self, args):
        kwargs = {}
        if args and args.fdcanusb:
            kwargs['path'] = args.fdcanusb
        return fdcanusb.Fdcanusb(**kwargs)


class PythonCanFactory:
    PRIORITY = 11

    name = 'pythoncan'

    def add_args(self, parser):
        parser.add_argument('--can-iface', type=str, metavar='IFACE',
                            help='pythoncan "interface" (default: socketcan)')
        parser.add_argument('--can-chan', type=str, metavar='CHAN',
                            help='pythoncan "channel" (default: can0)')

    def is_args_set(self, args):
        return args and (args.can_iface or args.can_chan)

    def __call__(self, args):
        kwargs = {}
        if args:
            if args.can_iface:
                kwargs['interface'] = args.can_iface
            if args.can_chan:
                kwargs['channel'] = args.can_chan
        return pythoncan.PythonCan(**kwargs)


'''External callers may insert additional factories into this list.'''
TRANSPORT_FACTORIES = [
    FdcanusbFactory(),
    PythonCanFactory(),
] + [ep.load()() for ep in
     importlib_metadata.entry_points().get('moteus.transports', [])]


GLOBAL_TRANSPORT = None


def make_transport_args(parser):
    for factory in TRANSPORT_FACTORIES:
        if hasattr(factory, 'add_args'):
            factory.add_args(parser)

    parser.add_argument(
        '--force-transport', type=str,
        choices=[x.name for x in TRANSPORT_FACTORIES],
        help='Force the given transport type to be used')


def get_singleton_transport(args=None):
    global GLOBAL_TRANSPORT

    if GLOBAL_TRANSPORT:
        return GLOBAL_TRANSPORT

    maybe_result = None
    to_try = sorted(TRANSPORT_FACTORIES, key=lambda x: x.PRIORITY)
    if args and args.force_transport:
        to_try = [x for x in to_try if x.name == args.force_transport]
    elif args:
        # See if any transports have options set.  If so, then limit
        # to just those that do.
        if any([x.is_args_set(args) for x in TRANSPORT_FACTORIES]):
            to_try = [x for x in to_try if x.is_args_set(args)]

    errors = []
    for factory in to_try:
        try:
            maybe_result = factory(args)
            break
        except Exception as e:
            errors.append((factory, str(e)))
            pass

    if maybe_result is None:
        raise RuntimeError("Unable to find a default transport, tried: {}".format(
            ','.join([str(x) for x in errors])))

    GLOBAL_TRANSPORT = maybe_result
    return GLOBAL_TRANSPORT


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
    mode = mp.INT8
    position = mp.F32
    velocity = mp.F32
    torque = mp.F32
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
    maximum_torque = mp.F32
    stop_position = mp.F32
    watchdog_timeout = mp.F32


class CurrentResolution:
    d_A = mp.F32
    q_A = mp.F32


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

    def write_current(self, value, resolution):
        self.write_mapped(value, 1.0, 0.1, 0.001, resolution)


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
    def parse(message):
        result = Result()
        result.id = id
        result.values = parse_reply(message.data)
        return result
    return parse


def parse_diagnostic_data(message):
    data = message.data

    if len(data) < 3:
        return None

    if data[0] != mp.STREAM_SERVER_DATA:
        return None
    if data[1] != 1:
        return None
    datalen, nextoff = mp.read_varuint(2, data)
    if datalen is None:
        return None

    if datalen > (len(data) - nextoff):
        return None
    return data[nextoff:nextoff+datalen]


class DiagnosticResult:
    id = None
    data = b''

    def __repr__(self):
        return f'{self.id}/{self.data}'


def make_diagnostic_parser(id):
    def parse(data):
        result = DiagnosticResult()
        result.id = id
        result.data = parse_diagnostic_data(data)
        return result
    return parse


class Controller:
    """Operates a single moteus controller across some communication
    medium.

    Attributes:
      id: bus ID of the controller
      query_resolution: an instance of moteus.QueryResolution
      position_resolution: an instance of moteus.PositionResolution
      transport: something modeling moteus.Transport to send commands through
    """

    def __init__(self, id=1,
                 query_resolution=QueryResolution(),
                 position_resolution=PositionResolution(),
                 current_resolution=CurrentResolution(),
                 transport=None):
        self.id = id
        self.query_resolution = query_resolution
        self.position_resolution = position_resolution
        self.current_resolution = current_resolution
        self.transport = transport
        self._parser = make_parser(id)
        self._diagnostic_parser = make_diagnostic_parser(id)

        # Pre-compute our query string.
        self._query_data = self._make_query_data()

    def _get_transport(self):
        if self.transport:
            return self.transport

        # Try to construct a global singleton using some agreed upon
        # method that is hookable.
        self.transport = get_singleton_transport()
        return self.transport

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

    def _extract(self, value):
        if len(value):
            return value[0]
        return None

    def _make_command(self, *, query, source=0):
        result = cmd.Command()

        result.destination = self.id
        result.source = source
        result.reply_required = query
        result.parse = self._parser

        return result

    def make_query(self):
        result = self._make_command(query=True)
        result.data = self._query_data
        return result;

    async def query(self, **kwargs):
        return self._extract(await self._get_transport().cycle(
            [self.make_query(**kwargs)]))

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
        return self._extract(await self._get_transport().cycle(
            [self.make_stop(**kwargs)]))

    def make_rezero(self, *,
                    rezero=0.0,
                    query=False):
        """Return a moteus.Command structure with data necessary to send a
        rezero command."""

        result = self._make_command(query=query)

        data_buf = io.BytesIO()
        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_F32 | 0x01)
        writer.write_varuint(Register.REZERO)
        writer.write_f32(rezero)

        if query:
            data_buf.write(self._query_data)

        result.data = data_buf.getvalue()
        return result

    async def set_rezero(self, *args, **kwargs):
        return self._extract(await self._get_transport().cycle(
            [self.make_rezero(**kwargs)]))

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
            pr.position if position is not None else mp.IGNORE,
            pr.velocity if velocity is not None else mp.IGNORE,
            pr.feedforward_torque if feedforward_torque is not None else mp.IGNORE,
            pr.kp_scale if kp_scale is not None else mp.IGNORE,
            pr.kd_scale if kd_scale is not None else mp.IGNORE,
            pr.maximum_torque if maximum_torque is not None else mp.IGNORE,
            pr.stop_position if stop_position is not None else mp.IGNORE,
            pr.watchdog_timeout if watchdog_timeout is not None else mp.IGNORE,
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
        return self._extract(await self._get_transport().cycle(
            [self.make_position(**kwargs)]))

    def make_current(self,
                     *,
                     d_A,
                     q_A,
                     query=False):
        """Return a moteus.Command structure with data necessary to send a
        current mode command.
        """

        result = self._make_command(query=query)
        cr = self.current_resolution
        resolutions = [
            cr.d_A if d_A is not None else mp.IGNORE,
            cr.q_A if q_A is not None else mp.IGNORE,
        ]

        data_buf = io.BytesIO()

        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_int8(int(Register.MODE))
        writer.write_int8(int(Mode.CURRENT))

        # Yes, annoyingly the register mapping as of version 4 still
        # has the Q current first in this grouping, unlike everywhere
        # else where D current is first.
        combiner = mp.WriteCombiner(
            writer, 0x00, int(Register.COMMAND_Q_CURRENT), resolutions)

        if combiner.maybe_write():
            writer.write_current(q_A, cr.q_A)
        if combiner.maybe_write():
            writer.write_current(d_A, cr.d_A)

        if query:
            data_buf.write(self._query_data)

        result.data = data_buf.getvalue()

        return result

    async def set_current(self, *args, **kwargs):
        return self._extract(await self._get_transport().cycle(
            [self.make_current(**kwargs)]))

    def make_diagnostic_write(self, data):
        result = self._make_command(query=False)

        # CAN-FD frames can be at most 64 bytes long
        assert len(data) <= 61

        data_buf = io.BytesIO()
        writer = Writer(data_buf)
        writer.write_int8(mp.STREAM_CLIENT_DATA)
        writer.write_int8(1)  # channel
        writer.write_int8(len(data))
        data_buf.write(data)

        result.data = data_buf.getvalue()
        return result

    async def send_diagnostic_write(self, *args, **kwargs):
        await self._get_transport().cycle([self.make_diagnostic_write(**kwargs)])

    def make_diagnostic_read(self, max_length=48, source=0):
        result = self._make_command(query=True)

        data_buf = io.BytesIO()
        writer = Writer(data_buf)
        writer.write_int8(mp.STREAM_CLIENT_POLL)
        writer.write_int8(1)
        writer.write_int8(max_length)

        result.parse = self._diagnostic_parser

        result.data = data_buf.getvalue()
        return result

    async def diagnostic_read(self, *args, **kwargs):
        return await self._get_transport().cycle(
            [self.make_diagnostic_read(**kwargs)])


class CommandError(RuntimeError):
    def __init__(self, message):
        super(CommandError, self).__init__("Error response:" + message)
        self.message = message


class Stream:
    """Presents a python file-like interface to the diagnostic stream of a
    moteus controller."""

    def __init__(self, controller, verbose=False):
        self.controller = controller
        self.verbose = verbose

        self.lock = asyncio.Lock()
        self._read_data = b''
        self._write_data = b''

        self._readers = {}

    def write(self, data):
        self._write_data += data

    async def drain(self):
        while len(self._write_data):
            to_write, self._write_data = self._write_data[0:61], self._write_data[61:]

            async with self.lock:
                await self.controller.send_diagnostic_write(data=to_write)

    async def read(self, size, block=True):
        while ((block == True and len(self._read_data) < size)
               or len(self._read_data) == 0):
            bytes_to_request = min(61, size - len(self._read_data))

            async with self.lock:
                these_results = await self.controller.diagnostic_read(bytes_to_request)

            this_data = b''.join(x.data for x in these_results if x.data)

            self._read_data += this_data

            if len(this_data) == 0:
                # Wait a bit before asking again.
                await asyncio.sleep(0.01)

        to_return, self._read_data = self._read_data[0:size], self._read_data[size:]
        return to_return

    async def flush_read(self, timeout=0.2):
        self._read_data = b''

        try:
            await asyncio.wait_for(self.read(65536), timeout)
            raise RuntimeError("More data to flush than expected")
        except asyncio.TimeoutError:
            # This is the expected path.
            pass

        self._read_data = b''

    async def _read_maybe_empty_line(self):
        while b'\n' not in self._read_data and b'\r' not in self._read_data:
            async with self.lock:
                these_results = await self.controller.diagnostic_read(61)

            this_data = b''.join(x.data for x in these_results if x.data)

            self._read_data += this_data

            if len(this_data) == 0:
                await asyncio.sleep(0.01)

        first_newline = min((self._read_data.find(c) for c in b'\r\n'
                             if c in self._read_data), default=None)
        to_return, self._read_data = (
            self._read_data[0:first_newline+1],
            self._read_data[first_newline+1:])
        return to_return

    async def readline(self):
        while True:
            line = (await self._read_maybe_empty_line()).rstrip()
            if len(line) > 0:
                if self.verbose:
                    print(f"< {line}")
                return line

    async def read_until_OK(self):
        result = b''
        while True:
            line = await self.readline()
            if line.startswith(b'OK'):
                return result
            if line.startswith(b'ERR'):
                raise CommandError(line.decode('latin1'))
            result += (line + b'\n')

    async def command(self, data, allow_any_response=False):
        await self.write_message(data)

        if allow_any_response:
            result = await self.readline()
        else:
            result = await self.read_until_OK()
        return result

    async def write_message(self, data):
        if self.verbose:
            print(f"> {data}")

        self.write(data + b'\n')
        await self.drain()

    async def read_binary_blob(self):
        size_bytes = await self.read(5, block=True)
        if size_bytes[0] != 0x0a:
            raise RuntimeError("missing newline before blob")
        size, = struct.unpack('<I', size_bytes[1:])
        return await self.read(size, block=True)

    async def read_data(self, name):
        if name not in self._readers:
            await self.write_message(f"tel schema {name}".encode('latin1'))

            maybe_schema_announce = await self.readline()
            if maybe_schema_announce != f"schema {name}".encode('latin1'):
                raise RuntimeError(
                    f"Unexpected schema announce for '{name}' " +
                    f": '{maybe_schema_announce}'")

            schema = await self.read_binary_blob()
            self._readers[name] = moteus.reader.Type.from_binary(io.BytesIO(schema))

            # Set this to be emitted as binary
            await self.command(f"tel fmt {name} 0".encode('latin1'))

        reader = self._readers[name]
        await self.write_message(f"tel get {name}".encode('latin1'))
        maybe_data_announce = await self.readline()

        if maybe_data_announce != f"emit {name}".encode('latin1'):
            raise RuntimeError(
                f"Invalid data announce for '{name}' : " +
                f"'{maybe_data_announce}'")

        data = await self.read_binary_blob()
        return reader.read(moteus.reader.Stream(io.BytesIO(data)))
