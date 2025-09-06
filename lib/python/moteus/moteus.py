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

import asyncio
import argparse
import copy
import enum
import importlib_metadata
import io
import math
import struct

from . import multiplex as mp
from . import command as cmd
from . import fdcanusb
from . import pythoncan
from .protocol import Register, Mode, Writer, parse_reply, Result, parse_message

import moteus.reader

class FdcanusbFactory:
    PRIORITY = 10

    name = 'fdcanusb'

    def add_args(self, parser):
        try:
            parser.add_argument('--can-disable-brs', action='store_true',
                                help='do not set BRS')
        except argparse.ArgumentError:
            # It must already be set.
            pass
        parser.add_argument('--fdcanusb', type=str, metavar='FILE',
                            help='path to fdcanusb device')
        parser.add_argument('--fdcanusb-debug', type=str, metavar='DEBUG',
                            help='write debug log')

    def is_args_set(self, args):
        return args and (args.fdcanusb or args.fdcanusb_debug)

    def __call__(self, args):
        kwargs = {}
        if args and args.fdcanusb:
            kwargs['path'] = args.fdcanusb
        if args and args.fdcanusb_debug:
            kwargs['debug_log'] = open(args.fdcanusb_debug, 'wb')
        if args and args.can_disable_brs:
            kwargs['disable_brs'] = True
        return fdcanusb.Fdcanusb(**kwargs)


class PythonCanFactory:
    PRIORITY = 11

    name = 'pythoncan'

    def add_args(self, parser):
        try:
            parser.add_argument('--can-disable-brs', action='store_true',
                                help='do not set BRS')
        except argparse.ArgumentError:
            # It must already be set.
            pass
        parser.add_argument('--can-iface', type=str, metavar='IFACE',
                            help='pythoncan "interface" (default: socketcan)')
        parser.add_argument('--can-chan', type=str, metavar='CHAN',
                            help='pythoncan "channel" (default: can0)')
        parser.add_argument('--can-debug', type=str, metavar='DEBUG',
                            help='write debug log')

    def is_args_set(self, args):
        return args and (args.can_iface or args.can_chan)

    def __call__(self, args):
        kwargs = {}
        if args:
            if args.can_iface:
                kwargs['interface'] = args.can_iface
            if args.can_chan:
                kwargs['channel'] = args.can_chan
            if args.can_disable_brs:
                kwargs['disable_brs'] = True
            if args.can_debug:
                kwargs['debug_log'] = open(args.can_debug, 'wb')
        return pythoncan.PythonCan(**kwargs)


'''External callers may insert additional factories into this list.'''
TRANSPORT_FACTORIES = [
    FdcanusbFactory(),
    PythonCanFactory(),
] + [ep.load()() for ep in
     importlib_metadata.entry_points().select(group='moteus.transports')]


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


def _merge_resolutions(a, b):
    if a == mp.IGNORE:
        return b
    if b == mp.IGNORE:
        return a
    return max(a, b)


class QueryResolution:
    mode = mp.INT8
    position = mp.F32
    velocity = mp.F32
    torque = mp.F32
    q_current = mp.IGNORE
    d_current = mp.IGNORE
    abs_position = mp.IGNORE
    power = mp.IGNORE
    motor_temperature = mp.IGNORE
    trajectory_complete = mp.IGNORE
    rezero_state = mp.IGNORE
    home_state = mp.IGNORE
    voltage = mp.INT8
    temperature = mp.INT8
    fault = mp.INT8

    aux1_gpio = mp.IGNORE
    aux2_gpio = mp.IGNORE

    # Additional registers can be queried by enumerating them as keys
    # in this dictionary, with the resolution as the matching value.
    _extra = {
        # 0x020 : mp.F32, ...
    }


class PositionResolution:
    position = mp.F32
    velocity = mp.F32
    feedforward_torque = mp.F32
    kp_scale = mp.F32
    kd_scale = mp.F32
    maximum_torque = mp.F32
    stop_position = mp.F32
    watchdog_timeout = mp.F32
    velocity_limit = mp.F32
    accel_limit = mp.F32
    fixed_voltage_override = mp.F32
    ilimit_scale = mp.F32
    fixed_current_override = mp.F32
    ignore_position_bounds = mp.F32


class VFOCResolution:
    theta = mp.F32
    voltage = mp.F32
    theta_rate = mp.F32


class CurrentResolution:
    d_A = mp.F32
    q_A = mp.F32


class PwmResolution:
    aux1_pwm1 = mp.INT16
    aux1_pwm2 = mp.INT16
    aux1_pwm3 = mp.INT16
    aux1_pwm4 = mp.INT16
    aux1_pwm5 = mp.INT16
    aux2_pwm1 = mp.INT16
    aux2_pwm2 = mp.INT16
    aux2_pwm3 = mp.INT16
    aux2_pwm4 = mp.INT16
    aux2_pwm5 = mp.INT16


class ZeroVelocityResolution:
    kd_scale = mp.F32


def make_parser(id):
    return parse_message


def parse_diagnostic_message(message, channel):
    data = message.data

    if len(data) < 3:
        return None

    if data[0] != mp.STREAM_SERVER_DATA:
        return None
    if data[1] != channel:
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


def make_diagnostic_parser(channel):
    def parse(message):
        result = DiagnosticResult()
        result.id = (message.arbitration_id >> 8) & 0x7f
        result.data = parse_diagnostic_message(message, channel)
        return result
    return parse


class FaultError(RuntimeError):
    def __init__(self, mode, code):
        super(FaultError, self).__init__(f"Fault mode={mode} code={code}")


class Controller:
    """Operates a single moteus controller across some communication
    medium.

    Attributes:
      id: bus ID of the controller or DeviceAddress structure
      query_resolution: an instance of moteus.QueryResolution
      position_resolution: an instance of moteus.PositionResolution
      transport: something modeling moteus.Transport to send commands through
    """

    def __init__(self, id=1,
                 query_resolution=QueryResolution(),
                 position_resolution=PositionResolution(),
                 vfoc_resolution=VFOCResolution(),
                 current_resolution=CurrentResolution(),
                 pwm_resolution=PwmResolution(),
                 zero_velocity_resolution=ZeroVelocityResolution(),
                 transport=None,
                 source_can_id=0,
                 can_prefix=0x0000):
        self.id = id
        self.source_can_id = source_can_id
        self.query_resolution = query_resolution
        self.position_resolution = position_resolution
        self.vfoc_resolution = vfoc_resolution
        self.current_resolution = current_resolution
        self.pwm_resolution = pwm_resolution
        self.zero_velocity_resolution = zero_velocity_resolution
        self.transport = transport
        self._parser = make_parser(id)
        self._can_prefix = can_prefix

        # Pre-compute our query string.
        self._query_data, self._default_query_reply_size = self._make_query_data()
        self._make_uuid_prefix_data()
        self.max_diagnostic_write = 64 - len(self._uuid_prefix_data) - 3

    def _get_transport(self):
        if self.transport:
            return self.transport

        # Try to construct a global singleton using some agreed upon
        # method that is hookable.
        self.transport = get_singleton_transport()
        return self.transport

    async def flush_transport(self):
        try:
            await asyncio.wait_for(self.transport.read(), 0.02)
        except asyncio.TimeoutError:
            pass

    def _make_uuid_prefix_data(self):
        if isinstance(self.id, int) or self.id.uuid is None:
            self._uuid_prefix_data = b''
            return

        buf = io.BytesIO()
        writer = Writer(buf)

        reg_count = len(self.id.uuid) // 4
        assert len(self.id.uuid) % 4 == 0
        assert len(self.id.uuid) <= 16 and len(self.id.uuid) > 0

        combiner = mp.WriteCombiner(
            writer, 0x00, Register.UUID_MASK1, [mp.INT32] * reg_count)

        # Convert UUID prefix bytes to list of 32-bit integers
        # (little-endian)
        i32_data = []
        for i in range(0, len(self.id.uuid), 4):
            chunk = self.id.uuid[i:i+4]
            i32_val = struct.unpack('<i', chunk)[0]
            i32_data.append(i32_val)

        for i32_val in i32_data:
            if combiner.maybe_write():
                writer.write_int32(i32_val)

        self._uuid_prefix_data = buf.getvalue()

    def _make_query_data(self, query_resolution=None):
        if query_resolution is None:
            query_resolution = self.query_resolution

        expected_reply_size = 0

        buf = io.BytesIO()
        writer = Writer(buf)
        qr = query_resolution
        c1 = mp.WriteCombiner(writer, 0x10, int(Register.MODE), [
            qr.mode,
            qr.position,
            qr.velocity,
            qr.torque,
            qr.q_current,
            qr.d_current,
            qr.abs_position,
            qr.power,
            ])
        for i in range(c1.size()):
            c1.maybe_write()

        expected_reply_size += c1.reply_size

        c2 = mp.WriteCombiner(writer, 0x10, int(Register.MOTOR_TEMPERATURE), [
            qr.motor_temperature,
            qr.trajectory_complete,
            _merge_resolutions(qr.rezero_state, qr.home_state),
            qr.voltage,
            qr.temperature,
            qr.fault,
        ])
        for i in range(c2.size()):
            c2.maybe_write()

        expected_reply_size += c2.reply_size

        c3 = mp.WriteCombiner(writer, 0x10, int(Register.AUX1_GPIO_STATUS), [
            qr.aux1_gpio,
            qr.aux2_gpio,
        ])
        for i in range(c3.size()):
            c3.maybe_write()

        expected_reply_size += c3.reply_size

        if len(qr._extra):
            min_val = int(min(qr._extra.keys()))
            max_val = int(max(qr._extra.keys()))
            c4 = mp.WriteCombiner(
                writer, 0x10, min_val,
                [qr._extra.get(i, mp.IGNORE)
                 for i in range(min_val, max_val +1)])
            for _ in range(c4.size()):
                c4.maybe_write()
            expected_reply_size += c4.reply_size

        return buf.getvalue(), expected_reply_size

    def _format_query(self, query, query_override, data_buf, result):
        if query_override is not None:
            query_data, expected_reply_size = \
                self._make_query_data(query_override)
            data_buf.write(query_data)
            result.expected_reply_size = expected_reply_size
        elif query:
            data_buf.write(self._query_data)
            result.expected_reply_size = self._default_query_reply_size

    def _make_command(self, *, query, query_override=None):
        result = cmd.Command()

        if isinstance(self.id, int):
            result.destination = self.id
        else:
            if self.id.can_id:
                result.destination = self.id.can_id
            elif self.id.uuid:
                # We will use the broadcast address
                result.destination = 0x7f

                # And then prefix our data with a request to mask
                # based on this UUID.
                result.data = self._uuid_prefix_data

            else:
                # This destination is not addressable.
                raise RuntimeError("Attempting to send data to a controller with neither a unique CAN ID nor UUID")

            result.channel = self.id.transport_device

        result.source = self.source_can_id
        result.reply_required = query or (query_override is not None)
        result.parse = self._parser
        result.can_prefix = self._can_prefix
        result.expected_reply_size = self._default_query_reply_size if query else 0

        # Create and properly position BytesIO for data construction
        data_buf = io.BytesIO(result.data if result.data else b'')
        data_buf.seek(0, io.SEEK_END)

        return result, data_buf

    def make_query(self, query_override=None):
        result, _ = self._make_command(
            query=True, query_override=query_override)
        if query_override:
            result.data, result.expected_reply_size = \
                self._make_query_data(query_override)
        else:
            result.data = self._query_data
            result.expected_reply_size = self._default_query_reply_size
        return result;

    async def query(self, **kwargs):
        return await self.execute(self.make_query(**kwargs))

    def make_custom_query(self, to_query_fields):
        """Return a moteus.Command structure with data required to query the
        registers given by the 'to_query_fields' dictionary of
        registers to resolutions.
        """

        result, data_buf = self._make_command(query=True)

        writer = Writer(data_buf)

        min_val = int(min(to_query_fields.keys()))
        max_val = int(max(to_query_fields.keys()))
        c = mp.WriteCombiner(writer, 0x10, min_val,
                             [to_query_fields.get(i, mp.IGNORE)
                              for i in range(min_val, max_val + 1)])
        for _ in range(min_val, max_val + 1):
            c.maybe_write()

        result.data = data_buf.getvalue()
        result.expected_reply_size = c.reply_size
        return result

    async def custom_query(self, *args, **kwargs):
        return await self.execute(self.make_custom_query(*args, **kwargs))

    def make_stop(self, *, query=False, query_override=None):
        """Return a moteus.Command structure with data necessary to send a
        stop mode command."""

        result, data_buf = self._make_command(
            query=query, query_override=query_override)

        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_int8(int(Register.MODE))
        writer.write_int8(int(Mode.STOPPED))

        self._format_query(query, query_override, data_buf, result)

        result.data = data_buf.getvalue()

        return result

    async def set_stop(self, *args, **kwargs):
        return await self.execute(self.make_stop(**kwargs))

    def make_zero_velocity(self,
                           *,
                           kd_scale=None,
                           query=False,
                           query_override=None):
        """Return a moteus.Command structure with data necessary to send a
        zero velocity mode command."""

        result, data_buf = self._make_command(
            query=query, query_override=query_override)

        zr = self.zero_velocity_resolution
        resolutions = [
            zr.kd_scale if kd_scale is not None else mp.IGNORE,
        ]

        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_int8(int(Register.MODE))
        writer.write_int8(int(Mode.ZERO_VELOCITY))

        # Only write kd_scale if it's not None
        if kd_scale is not None:
            combiner = mp.WriteCombiner(
                writer, 0x00, int(Register.COMMAND_KD_SCALE), resolutions)

            if combiner.maybe_write():
                writer.write_pwm(kd_scale, zr.kd_scale)

        self._format_query(query, query_override, data_buf, result)

        result.data = data_buf.getvalue()

        return result

    async def set_zero_velocity(self, *args, **kwargs):
        return await self.execute(self.make_zero_velocity(**kwargs))

    def make_set_output(self, *args,
                        position=0.0,
                        query=False,
                        query_override=None,
                        cmd=None
    ):
        """Return a moteus.Command structure with data necessary to send a
        set output nearest command."""

        if len(args):
            raise ValueError(f'unexpected positional arguments: {args}')

        result, data_buf = self._make_command(
            query=query, query_override=query_override)

        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_F32 | 0x01)
        writer.write_varuint(cmd)
        writer.write_f32(position)

        self._format_query(query, query_override, data_buf, result)

        result.data = data_buf.getvalue()
        return result

    def make_set_output_nearest(self, *args,
                                position=0.0,
                                query=False,
                                query_override=None):
        return self.make_set_output(
            *args,
            position=position, query=query, query_override=query_override,
            cmd=Register.SET_OUTPUT_NEAREST)

    def make_set_output_exact(self, *args,
                              position=0.0,
                              query=False,
                              query_override=None):
        return self.make_set_output(
            *args,
            position=position, query=query, query_override=query_override,
            cmd=Register.SET_OUTPUT_EXACT)

    async def set_output(self, *args, cmd=None, **kwargs):
        return await self.execute(self.make_set_output(*args, **kwargs, cmd=cmd))

    async def set_output_nearest(self, *args, **kwargs):
        return await self.set_output(*args, cmd=Register.SET_OUTPUT_NEAREST, **kwargs)

    async def set_output_exact(self, *args, **kwargs):
        return await self.set_output(*args, cmd=Register.SET_OUTPUT_EXACT, **kwargs)


    # For backwards compatibility, "*_output_nearest" used to be named
    # "make/set_rezero".
    def make_rezero(self, *args,
                    rezero=0.0,
                    query=False,
                    query_override=None):
        return self.make_set_output(
            *args,
            position=rezero, query=query, query_override=query_override,
            cmd=Register.SET_OUTPUT_NEAREST)

    async def set_rezero(self, *args, **kwargs):
        return await self.execute(self.make_rezero(**kwargs))

    def make_require_reindex(self,
                             query=False,
                             query_override=None):
        result, data_buf = self._make_command(
            query=query, query_override=query_override)

        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_varuint(Register.REQUIRE_REINDEX)
        writer.write_int8(1)

        result.data = data_buf.getvalue()
        return result

    async def set_require_reindex(self, query=False, query_override=None):
        return await self.execute(self.make_require_reindex(
            query=query, query_override=query_override))

    def make_recapture_position_velocity(self,
                                         query=False,
                                         query_override=None):
        result, data_buf = self._make_command(
            query=query, query_override=query_override)

        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_varuint(Register.RECAPTURE_POSITION_VELOCITY)
        writer.write_int8(1)

        result.data = data_buf.getvalue()
        return result

    async def set_recapture_position_velocity(self,
                                              query=False,
                                              query_override=None):
        return await self.execute(self.make_recapture_position_velocity(
            query=query, query_override=query_override))

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
                      velocity_limit=None,
                      accel_limit=None,
                      fixed_voltage_override=None,
                      ilimit_scale=None,
                      fixed_current_override=None,
                      ignore_position_bounds=None,
                      query=False,
                      query_override=None):
        """Return a moteus.Command structure with data necessary to send a
        position mode command with the given values."""

        result, data_buf = self._make_command(
            query=query, query_override=query_override)

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
            pr.velocity_limit if velocity_limit is not None else mp.IGNORE,
            pr.accel_limit if accel_limit is not None else mp.IGNORE,
            pr.fixed_voltage_override if fixed_voltage_override is not None else mp.IGNORE,
            pr.ilimit_scale if ilimit_scale is not None else mp.IGNORE,
            pr.fixed_current_override if fixed_current_override is not None else mp.IGNORE,
            pr.ignore_position_bounds if ignore_position_bounds is not None else mp.IGNORE,
        ]

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
        if combiner.maybe_write():
            writer.write_velocity(velocity_limit, pr.velocity_limit)
        if combiner.maybe_write():
            writer.write_accel(accel_limit, pr.accel_limit)
        if combiner.maybe_write():
            writer.write_voltage(fixed_voltage_override, pr.fixed_voltage_override)
        if combiner.maybe_write():
            writer.write_pwm(ilimit_scale, pr.ilimit_scale)
        if combiner.maybe_write():
            writer.write_current(fixed_current_override, pr.fixed_current_override)
        if combiner.maybe_write():
            writer.write_int(ignore_position_bounds, pr.ignore_position_bounds)

        self._format_query(query, query_override, data_buf, result)

        result.data = data_buf.getvalue()

        return result

    async def set_position(self, *args, **kwargs):
        return await self.execute(self.make_position(**kwargs))

    async def set_position_wait_complete(
            self,
            period_s=0.025,
            query_override=None,
            *args, **kwargs):
        """Repeatedly send a position mode command to a device until it
        reports that the trajectory has been completed.

        If the controller is unresponsive, this method will never return.

        If the controller reports a fault or position mode timeout, a
        FaultError exception will be raised.
        """

        if query_override is None:
            query_override = copy.deepcopy(self.query_resolution)
        else:
            query_override = copy.deepcopy(query_override)

        if query_override.mode == mp.IGNORE:
            query_override.mode = mp.INT8
        if query_override.fault == mp.IGNORE:
            query_override.fault = mp.INT8
        query_override.trajectory_complete = mp.INT8

        count = 2
        while True:
            result = await self.set_position(
                query_override=query_override, *args, **kwargs)

            if result is not None:
                count = max(count - 1, 0)

            if (count == 0 and
                result is not None and
                result.values[Register.TRAJECTORY_COMPLETE]):
                return result

            current_mode = result.values.get(Register.MODE, Mode.STOPPED)
            fault_code = result.values.get(Register.FAULT, 0)

            if current_mode == Mode.FAULT or current_mode == Mode.TIMEOUT:
                raise FaultError(current_mode, fault_code)

            await asyncio.sleep(period_s)

    def make_vfoc(self,
                  *,
                  theta,
                  voltage,
                  theta_rate=0.0,
                  query=False,
                  query_override=None):
        """Return a moteus.Command structure with data necessary to send a
        voltage mode FOC command."""

        result, data_buf = self._make_command(
            query=query, query_override=query_override)
        cr = self.vfoc_resolution
        resolutions = [
            cr.theta if theta is not None else mp.IGNORE,
            cr.voltage if voltage is not None else mp.IGNORE,
            mp.IGNORE,
            mp.IGNORE,
            mp.IGNORE,
            mp.IGNORE,
            cr.theta_rate if (theta_rate != 0.0 and theta_rate is not None) else mp.IGNORE,
        ]

        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_int8(int(Register.MODE))
        writer.write_int8(int(Mode.VOLTAGE_FOC))

        combiner = mp.WriteCombiner(
            writer, 0x00, int(Register.VFOC_THETA), resolutions)

        if combiner.maybe_write():
            writer.write_pwm(theta / math.pi, cr.theta)
        if combiner.maybe_write():
            writer.write_voltage(voltage, cr.voltage)
        if combiner.maybe_write():
            assert False
        if combiner.maybe_write():
            assert False
        if combiner.maybe_write():
            assert False
        if combiner.maybe_write():
            assert False
        if combiner.maybe_write():
            writer.write_velocity(theta_rate / math.pi, cr.theta_rate)

        self._format_query(query, query_override, data_buf, result)

        result.data = data_buf.getvalue()

        return result

    async def set_vfoc(self, *args, **kwargs):
        return await self.execute(self.make_vfoc(**kwargs))

    def make_current(self,
                     *,
                     d_A,
                     q_A,
                     query=False,
                     query_override=None):
        """Return a moteus.Command structure with data necessary to send a
        current mode command.
        """

        result, data_buf = self._make_command(
            query=query, query_override=query_override)
        cr = self.current_resolution
        resolutions = [
            cr.d_A if d_A is not None else mp.IGNORE,
            cr.q_A if q_A is not None else mp.IGNORE,
        ]

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

        self._format_query(query, query_override, data_buf, result)

        result.data = data_buf.getvalue()

        return result

    async def set_current(self, *args, **kwargs):
        return await self.execute(self.make_current(**kwargs))

    def make_stay_within(
            self,
            *,
            lower_bound=None,
            upper_bound=None,
            feedforward_torque=None,
            kp_scale=None,
            kd_scale=None,
            maximum_torque=None,
            stop_position=None,
            watchdog_timeout=None,
            ilimit_scale=None,
            ignore_position_bounds=None,
            query=False,
            query_override=None):
        """Return a moteus.Command structure with data necessary to send a
        within mode command with the given values."""

        result, data_buf = self._make_command(
            query=query, query_override=query_override)

        pr = self.position_resolution
        resolutions = [
            pr.position if lower_bound is not None else mp.IGNORE,
            pr.position if upper_bound is not None else mp.IGNORE,
            pr.feedforward_torque if feedforward_torque is not None else mp.IGNORE,
            pr.kp_scale if kp_scale is not None else mp.IGNORE,
            pr.kd_scale if kd_scale is not None else mp.IGNORE,
            pr.maximum_torque if maximum_torque is not None else mp.IGNORE,
            pr.watchdog_timeout if watchdog_timeout is not None else mp.IGNORE,
            pr.ilimit_scale if ilimit_scale is not None else mp.IGNORE,
            pr.ignore_position_bounds if ignore_position_bounds is not None else mp.IGNORE,
        ]

        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_int8(int(Register.MODE))
        writer.write_int8(int(Mode.STAY_WITHIN))

        combiner = mp.WriteCombiner(
            writer, 0x00, int(Register.COMMAND_WITHIN_LOWER_BOUND),
            resolutions)

        if combiner.maybe_write():
            writer.write_position(lower_bound, pr.position)
        if combiner.maybe_write():
            writer.write_position(upper_bound, pr.position)
        if combiner.maybe_write():
            writer.write_torque(feedforward_torque, pr.feedforward_torque)
        if combiner.maybe_write():
            writer.write_pwm(kp_scale, pr.kp_scale)
        if combiner.maybe_write():
            writer.write_pwm(kd_scale, pr.kd_scale)
        if combiner.maybe_write():
            writer.write_torque(maximum_torque, pr.maximum_torque)
        if combiner.maybe_write():
            writer.write_time(watchdog_timeout, pr.watchdog_timeout)
        if combiner.maybe_write():
            writer.write_pwm(ilimit_scale, pr.ilimit_scale)
        if combiner.maybe_write():
            writer.write_int(ignore_position_bounds, pr.ignore_position_bounds)

        self._format_query(query, query_override, data_buf, result)

        result.data = data_buf.getvalue()

        return result

    async def set_stay_within(self, *args, **kwargs):
        return await self.execute(self.make_stay_within(**kwargs))

    def make_brake(self, *, query=False, query_override=None):
        result, data_buf = self._make_command(
            query=query, query_override=query_override)

        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_int8(int(Register.MODE))
        writer.write_int8(int(Mode.BRAKE))

        self._format_query(query, query_override, data_buf, result)

        result.data = data_buf.getvalue()

        return result

    async def set_brake(self, *args, **kwargs):
        return await self.execute(self.make_brake(**kwargs))

    def make_write_gpio(self, aux1=None, aux2=None,
                        query=False, query_override=None):
        """Return a moteus.Command structure with data necessary to set one or
        more GPIO registers.

        aux1/aux2 are an optional integer bitfield, where the least
        significant bit is pin 0 on the respective port.
        """

        result, data_buf = self._make_command(
            query=query, query_override=query_override)

        writer = Writer(data_buf)

        combiner = mp.WriteCombiner(
            writer, 0x00, int(Register.AUX1_GPIO_COMMAND), [
                mp.INT8 if aux1 is not None else mp.IGNORE,
                mp.INT8 if aux2 is not None else mp.IGNORE,
        ])

        if combiner.maybe_write():
            writer.write_int8(aux1)
        if combiner.maybe_write():
            writer.write_int8(aux2)

        self._format_query(query, query_override, data_buf, result)

        result.data = data_buf.getvalue()
        return result

    async def set_write_gpio(self, *args, **kwargs):
        return await self.execute(self.make_write_gpio(**kwargs))

    def make_read_gpio(self):
        """Return a moteus.Command structure with data necessary to read all
        GPIO digital inputs."""

        result, data_buf = self._make_command(query=True)
        writer = Writer(data_buf)

        combiner = mp.WriteCombiner(
            writer, 0x10, int(Register.AUX1_GPIO_STATUS), [
                mp.INT8,
                mp.INT8,
        ])

        for i in range(2):
            combiner.maybe_write()

        result.data = data_buf.getvalue()
        return result

    async def read_gpio(self):
        """Return a bytes() object with an int8 for each auxiliary port.  The
        pins for each port are represented as bits, with the least significant
        bit being pin 0.

        None can be returned if no response is received.
        """

        results = await self._get_transport().cycle([self.make_read_gpio()])
        if len(results) == 0:
            return None
        result = results[0]
        return bytes([result.values[Register.AUX1_GPIO_STATUS],
                      result.values[Register.AUX2_GPIO_STATUS]])

    def make_diagnostic_write(self, data, channel=1):
        result, data_buf = self._make_command(query=False)

        # CAN-FD frames can be at most 64 bytes long
        assert len(data) <= 61

        writer = Writer(data_buf)
        writer.write_int8(mp.STREAM_CLIENT_DATA)
        writer.write_int8(channel)  # channel
        writer.write_int8(len(data))
        data_buf.write(data)

        result.data = data_buf.getvalue()
        return result

    async def send_diagnostic_write(self, *args, **kwargs):
        await self._get_transport().cycle([self.make_diagnostic_write(**kwargs)])

    def make_diagnostic_read(self, max_length=48, channel=1):
        result, data_buf = self._make_command(query=True)

        writer = Writer(data_buf)
        writer.write_int8(mp.STREAM_CLIENT_POLL)
        writer.write_int8(channel)
        writer.write_int8(max_length)

        result.parse = make_diagnostic_parser(channel)

        result.data = data_buf.getvalue()
        result.expected_reply_size = 3 + max_length
        return result

    async def diagnostic_read(self, *args, **kwargs):
        return await self._get_transport().cycle(
            [self.make_diagnostic_read(**kwargs)])

    def make_set_trim(self, *, trim=0):
        result, data_buf = self._make_command(query=False)

        writer = Writer(data_buf)
        writer.write_int8(mp.WRITE_INT32 | 0x01)
        writer.write_varuint(Register.CLOCK_TRIM)
        writer.write_int32(trim)

        result.data = data_buf.getvalue()
        return result

    async def set_trim(self, *args, **kwargs):
        return await self.execute(self.make_set_trim(*args, **kwargs))

    def make_aux_pwm(self, *,
                     aux1_pwm1=None,
                     aux1_pwm2=None,
                     aux1_pwm3=None,
                     aux1_pwm4=None,
                     aux1_pwm5=None,
                     aux2_pwm1=None,
                     aux2_pwm2=None,
                     aux2_pwm3=None,
                     aux2_pwm4=None,
                     aux2_pwm5=None,
                     query=False,
                     query_override=None):
        result, data_buf = self._make_command(query=query, query_override=query_override)

        pr = self.pwm_resolution
        resolutions = [
            pr.aux1_pwm1 if aux1_pwm1 is not None else mp.IGNORE,
            pr.aux1_pwm2 if aux1_pwm2 is not None else mp.IGNORE,
            pr.aux1_pwm3 if aux1_pwm3 is not None else mp.IGNORE,
            pr.aux1_pwm4 if aux1_pwm4 is not None else mp.IGNORE,
            pr.aux1_pwm5 if aux1_pwm5 is not None else mp.IGNORE,
            pr.aux2_pwm1 if aux2_pwm1 is not None else mp.IGNORE,
            pr.aux2_pwm2 if aux2_pwm2 is not None else mp.IGNORE,
            pr.aux2_pwm3 if aux2_pwm3 is not None else mp.IGNORE,
            pr.aux2_pwm4 if aux2_pwm4 is not None else mp.IGNORE,
            pr.aux2_pwm5 if aux2_pwm5 is not None else mp.IGNORE,
        ]

        writer = Writer(data_buf)
        combiner = mp.WriteCombiner(
            writer, 0x00, int(Register.AUX1_PWM1), resolutions)

        if combiner.maybe_write():
            writer.write_pwm(aux1_pwm1, pr.aux1_pwm1)
        if combiner.maybe_write():
            writer.write_pwm(aux1_pwm2, pr.aux1_pwm2)
        if combiner.maybe_write():
            writer.write_pwm(aux1_pwm3, pr.aux1_pwm3)
        if combiner.maybe_write():
            writer.write_pwm(aux1_pwm4, pr.aux1_pwm4)
        if combiner.maybe_write():
            writer.write_pwm(aux1_pwm5, pr.aux1_pwm5)
        if combiner.maybe_write():
            writer.write_pwm(aux2_pwm1, pr.aux2_pwm1)
        if combiner.maybe_write():
            writer.write_pwm(aux2_pwm2, pr.aux2_pwm2)
        if combiner.maybe_write():
            writer.write_pwm(aux2_pwm3, pr.aux2_pwm3)
        if combiner.maybe_write():
            writer.write_pwm(aux2_pwm4, pr.aux2_pwm4)
        if combiner.maybe_write():
            writer.write_pwm(aux2_pwm5, pr.aux2_pwm5)

        self._format_query(query, query_override, data_buf, result)

        result.data = data_buf.getvalue()

        return result

    async def set_aux_pwm(self, *args, **kwargs):
        return await self.execute(self.make_aux_pwm(*args, **kwargs))

    def _extract(self, value):
        if len(value):
            return value[0]
        return None

    async def execute(self, command):
        return self._extract(await self._get_transport().cycle([command]))


class CommandError(RuntimeError):
    def __init__(self, message):
        super(CommandError, self).__init__("Error response:" + message)
        self.message = message


class Stream:
    """Presents a python file-like interface to the diagnostic stream of a
    moteus controller."""

    def __init__(self, controller, verbose=False, channel=1):
        self.controller = controller
        self.verbose = verbose
        self.channel = channel

        self.lock = asyncio.Lock()
        self._read_data = b''
        self._write_data = b''

        self._readers = {}
        self._maxlen = controller.max_diagnostic_write

    def write(self, data):
        self._write_data += data

    async def drain(self):
        while len(self._write_data):
            to_write, self._write_data = self._write_data[0:self._maxlen], self._write_data[self._maxlen:]

            async with self.lock:
                await self.controller.send_diagnostic_write(
                    data=to_write, channel=self.channel)

    async def read(self, size, block=True):
        while ((block == True and len(self._read_data) < size)
               or len(self._read_data) == 0):
            bytes_to_request = min(self._maxlen, size - len(self._read_data))

            async with self.lock:
                these_results = await self.controller.diagnostic_read(
                    bytes_to_request, channel=self.channel)

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

        # Now flush anything from the underlying transport if applicable.
        await self.controller.flush_transport()

    async def _read_maybe_empty_line(self):
        while b'\n' not in self._read_data and b'\r' not in self._read_data:
            async with self.lock:
                these_results = await self.controller.diagnostic_read(
                    61, channel=self.channel)

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
