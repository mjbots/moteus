#!/usr/bin/python3 -B

# Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

'''Configures, calibrates, and manipulates the moteus brushless servo.'''

import argparse
import asyncio
import datetime
import elftools
import elftools.elf.elffile
import json
import io
import math
import os
import struct
import sys
import tempfile

from . import moteus
from . import aiostream
from . import regression
from . import calibrate_encoder as ce

MAX_FLASH_BLOCK_SIZE = 32


class FirmwareUpgrade:
    '''This encodes "magic" rules about upgrading firmware, largely about
    how to munge configuration options so as to not cause behavior
    change upon firmware changes.
    '''

    def __init__(self, old, new):
        self.old = old
        self.new = new

        if new > 0x0101:
            raise RuntimeError("Firmware to be flashed has a newer version than we support")

    def fix_config(self, old_config):
        lines = old_config.split(b'\n')
        items = dict([line.split(b' ') for line in lines if b' ' in line])

        #### 0x0101
        #
        # This version was the first to have the correct sign applied
        # to the velocity component of servo.feedforward_scale.  The
        # old versions applied a negative value, which acted opposite
        # to its desired intention.  The default value of the scale
        # was 1.0, which could cause velocity instability when used
        # with the new correct algorithm.
        #
        # So, when upgrading, if the value was at its default, we move
        # it to the new default value.  If it had been modified in any
        # way, we zero it out, which is safe, although it will result
        # in decreased performance.

        if self.new >= 0x0101 and self.old <= 0x0100:
            # If the old firmware had the default feedforward term of
            # 1.0, then switch it to be the new default of 0.5.
            if float(items[b'servo.feedforward_scale']) == 1.0:
                items[b'servo.feedforward_scale'] = b'0.5'
                print("Changing servo.feedforward_scale from 1.0 to 0.5 for version 0x0101")
            else:
                items[b'servo.feedforward_scale'] = b'0.0'
                print("Changing servo.feedforward_scale to 0.0 for version 0x0101")

        if self.new <= 0x0100 and self.old >= 0x0101:
            # To get back to identical behavior, we apply the inverse
            # mapping.
            if float(items[b'servo.feedforward_scale']) == 0.5:
                items[b'servo.feedforward_scale'] = b'1.0'
                print("Reverting servo.feedforward_scale from 0.5 to 1.0 for version 0x0101")

        lines = [key + b' ' + value for key, value in items.items()]
        return b'\n'.join(lines)


def _get_log_directory():
    moteus_cal_dir = os.environ.get("MOTEUS_CAL_DIR", None)
    if moteus_cal_dir:
        return moteus_cal_dir

    home = os.environ.get("HOME")
    if home:
        maybe_dir = os.path.join(home, "moteus-cal")
        if os.path.exists(maybe_dir):
            return maybe_dir

    return "."


def _expand_targets(targets):
    result = set()

    for item in targets:
        fields = item.split(',')
        for field in fields:
            if '-' in field:
                first, last = field.split('-')
                result |= set(range(int(first), int(last) + 1))
            else:
                result |= { int(field) }

    return sorted(list(result))


def _base64_serial_number(s1, s2, s3):
    serial_num = (s1 << 64) | (s2 << 32) | s3;
    digits = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    assert len(digits) == 64

    result = [0] * 16

    for i in range(16):
        digit_num = serial_num % 64
        result[15 - i] = digits[digit_num]
        serial_num //= 64

    return ''.join(result)


def _make_git_hash(hash):
    return ''.join(f"{x:02x}" for x in hash)


class ElfMapping:
    virtual_address = 0
    physical_address = 0
    size = 0


class ElfMappings:
    def __init__(self, fp):
        self._mappings = []

        for segment in fp.iter_segments():
            mapping = ElfMapping()
            mapping.virtual_address = segment['p_vaddr']
            mapping.physical_address = segment['p_paddr']
            mapping.size = segment['p_memsz']

            self._mappings.append(mapping)

    def logical_to_physical(self, address, size):
        for mapping in self._mappings:
            if (address >= mapping.virtual_address and
                (address + size) <= (mapping.virtual_address + mapping.size)):
                return address - mapping.virtual_address + mapping.physical_address
        raise RuntimeError(f"no mapping found for {address:x}")


class ElfData:
    sections = []
    firmware_version = None


def _read_elf(filename, sections):
    fp = elftools.elf.elffile.ELFFile(open(filename, "rb"))

    mappings = ElfMappings(fp)

    sections_to_find = set(sections)

    sections = {}

    for section in fp.iter_sections():
        if section.name not in sections_to_find:
            continue

        sections_to_find -= { section.name }

        data = section.data()

        physical_address = mappings.logical_to_physical(
            section['sh_addr'], section['sh_size'])
        sections[physical_address] = data

    result = ElfData()
    result.sections = []

    for key in sorted(sections.keys()):
        result.sections.append((key, sections[key]))

    # Try to find the firmware version.
    symtab = fp.get_section_by_name('.symtab')
    if symtab:
        maybe_version = symtab.get_symbol_by_name('kMoteusFirmwareVersion')
        if maybe_version:
            version = maybe_version[0]
            if version['st_size'] != 4:
                raise RuntimeError('Version in firmware image corrupt')

            symbol_address = version['st_value']
            symbol_section = version['st_shndx']

            sec = fp.get_section(symbol_section)
            sec_start = symbol_address - sec['sh_addr']
            ver, = struct.unpack('<i', sec.data()[sec_start:sec_start+4])

            result.firmware_version = ver

    if result.firmware_version is None:
        print("WARNING: Could not find firmware version in elf file, assuming 0x00000100")
        result.firmware_version = 0x00000100

    return result


async def _copy_stream(inp, out):
    while True:
        data = await inp.read(4096, block=False)
        if len(data) == 0:
            # EOF
            exit(0)
        out.write(data)
        await out.drain()


def _calculate_slope(x, y):
    return regression.linear_regression(x, y)[1]


def _calculate_winding_resistance(voltages, currents):
    return 1.0 / _calculate_slope(voltages, currents)


class FlashDataBlock:
    def __init__(self, address=-1, data=b""):
        self.address = address
        self.data = data


class FlashContext:
    def __init__(self, elf):
        self.elf = elf
        self.current_address = -1

    def get_next_block(self):
        # Find the next pair which contains something greater than
        # the current address.
        for address, data in self.elf:
            if address > self.current_address:
                # This is definitely it.
                return FlashDataBlock(
                    address, data[0:MAX_FLASH_BLOCK_SIZE])
            # We might be inside a block that has more data.
            end_of_this_block = address + len(data)
            if end_of_this_block > self.current_address:
                begin = self.current_address - address
                end = min(end_of_this_block - self.current_address,
                          MAX_FLASH_BLOCK_SIZE)
                return FlashDataBlock(
                    self.current_address, data[begin:begin+end])
        return FlashDataBlock()

    def advance_block(self):
        this_block = self.get_next_block()
        self.current_address = this_block.address + len(this_block.data)
        return self.get_next_block().address < 0


def _verify_blocks(expected, message):
    fields = message.decode('latin1').split(' ')
    if len(fields) != 2:
        raise RuntimeError(f"verify returned wrong field count {len(fields)} != 2")

    actual_address = int(fields[0], 16)
    if actual_address != expected.address:
        raise RuntimeError(f"verify returned wrong address {actual_address:x} != {expected.address:x}")

    actual_data = fields[1].strip().lower()
    if expected.data.hex() != actual_data:
        raise RuntimeError(f"verify returned wrong data at {expected.address:x}, {expected.data.hex()} != {actual_data}")


class Stream:
    def __init__(self, args, target_id, transport):
        self.args = args
        self.controller = moteus.Controller(target_id, transport=transport)
        self.stream = moteus.Stream(self.controller, verbose=args.verbose)

    async def do_console(self):
        console_stdin = aiostream.AioStream(sys.stdin.buffer.raw)
        console_stdout = aiostream.AioStream(sys.stdout.buffer.raw)
        dir1 = asyncio.create_task(_copy_stream(self.stream, console_stdout))
        dir2 = asyncio.create_task(_copy_stream(console_stdin, self.stream))
        done, pending = await asyncio.wait(
            [dir1, dir2], return_when=asyncio.FIRST_EXCEPTION)

    async def command(self, command_str, **kwargs):
        command_bytes = (command_str if type(command_str) == bytes else
                         command_str.encode('latin1'))
        return await self.stream.command(command_bytes, **kwargs)

    async def write_message(self, command_str, **kwargs):
        command_bytes = (command_str if type(command_str) == bytes else
                         command_str.encode('latin1'))
        return await self.stream.write_message(command_bytes, **kwargs)

    async def read_data(self, *args, **kwargs):
        return await self.stream.read_data(*args, **kwargs)

    async def read_config_double(self, name):
        result = await self.command(f"conf get {name}", allow_any_response=True)
        return float(result)

    async def get_device_info(self):
        firmware = await self.read_data("firmware")
        git = await self.read_data("git")

        result = {}
        result['serial_number'] = _base64_serial_number(
            firmware.serial_number[0],
            firmware.serial_number[1],
            firmware.serial_number[2])
        result['model'] = f"{firmware.model:x}"
        result['git_hash'] = _make_git_hash(git.hash)
        result['git_dirty'] = getattr(git, 'dirty', 0) != 0
        result['git_timestamp'] = getattr(git, 'timestamp', 0)

        return result

    async def info(self):
        print(json.dumps(await self.get_device_info(), indent=2))

    async def do_zero_offset(self):
        servo_stats = await self.read_data("servo_stats")
        position_raw = servo_stats.position_raw
        await self.command(f"conf set motor.position_offset {-position_raw:d}")
        await self.command("conf write")
        await self.command("d rezero")

    async def do_write_config(self, config_file):
        fp = open(config_file, "rb")
        await self.write_config_stream(fp)

    async def write_config_stream(self, fp):
        errors = []

        for line in fp.readlines():
            line = line.rstrip()
            if len(line) == 0:
                continue

            try:
                await self.command(line)
            except moteus.CommandError as ce:
                errors.append(line.decode('latin1'))

        if len(errors):
            print("\nSome config could not be set:")
            for line in errors:
                print(f" {line}")
            print()

    async def do_flash(self, elffile):
        elf = _read_elf(elffile, [".text", ".ARM.extab", ".ARM.exidx",
                                  ".data", ".ccmram", ".isr_vector"])
        count_bytes = sum([len(section) for address, section in elf.sections])

        fw = '0x{:08x}'.format(elf.firmware_version)
        print(f"Read ELF file version {fw}: {count_bytes} bytes")

        old_firmware = None

        if not self.args.bootloader_active:
            await self.write_message("tel stop")

            # Discard anything that might have been en route.
            await self.stream.flush_read()

            # Get the current firmware version.
            old_firmware = await self.read_data("firmware")

        upgrade = FirmwareUpgrade(
            elf.firmware_version
            if old_firmware is None else
            old_firmware.version,
            elf.firmware_version)

        if not self.args.bootloader_active and not self.args.no_restore_config:
            # Read our old config.
            old_config = await self.command("conf enumerate")

            # We will just leave this around in a temporary location.
            # Who knows, maybe it will be useful before the temp
            # directory gets erased?
            captured_config_file = tempfile.NamedTemporaryFile(
                prefix='moteus-config-', suffix='.cfg', delete=False)
            captured_config_file.write(old_config)
            captured_config_file.close()

            print(f"Captured old config to {captured_config_file.name}")

        # This will enter the bootloader.
        await self.write_message("d flash")
        await self.stream.readline()

        await self.command("unlock")
        await self.write_flash(elf.sections)
        await self.command("lock")
        # This will reset the controller, so we don't expect a response.
        await self.write_message("reset")

        await asyncio.sleep(1.0)

        if not self.args.bootloader_active and not self.args.no_restore_config:
            await self.restore_config(upgrade.fix_config(old_config))

    def _emit_flash_progress(self, ctx, type):
        if self.args.verbose:
            return
        print(f"flash: {type:15s}  {ctx.current_address:08x}", end="\r", flush=True)

    async def write_flash(self, elfs):
        write_ctx = FlashContext(elfs)
        while True:
            next_block = write_ctx.get_next_block()
            cmd = f"w {next_block.address:x} {next_block.data.hex()}"

            result = await self.command(cmd)
            self._emit_flash_progress(write_ctx, "flashing")
            done = write_ctx.advance_block()
            if done:
                break

        verify_ctx = FlashContext(elfs)
        while True:
            expected_block = verify_ctx.get_next_block()
            cmd = f"r {expected_block.address:x} {len(expected_block.data):x}"
            result = await self.command(cmd, allow_any_response=True)
            # Emit progress first, to make it easier to see where
            # things go wrong.
            self._emit_flash_progress(verify_ctx, "verifying")
            _verify_blocks(expected_block, result)
            done = verify_ctx.advance_block()
            if done:
                break

    async def check_for_fault(self):
        servo_stats = await self.read_data("servo_stats")
        if servo_stats.mode == 1:
            raise RuntimeError(f"Controller reported fault: {int(servo_stats.fault)}")

    async def restore_config(self, old_config):
        new_config = []
        for line in old_config.split(b'\n'):
            if len(line.strip()) == 0:
                continue
            new_config.append(b'conf set ' + line + b'\n')
        await self.write_config_stream(io.BytesIO(b''.join(new_config)))

    async def do_calibrate(self):
        print("This will move the motor, ensure it can spin freely!")
        await asyncio.sleep(2.0)

        unwrapped_position_scale = \
            await self.read_config_double("motor.unwrapped_position_scale")

        # We have 3 things to calibrate.
        #  1) The encoder to phase mapping
        #  2) The winding resistance
        #  3) The Kv rating of the motor.

        print("Starting calibration process")
        await self.check_for_fault()

        cal_result = await self.calibrate_encoder_mapping()
        await self.check_for_fault()

        winding_resistance = await self.calibrate_winding_resistance()
        await self.check_for_fault()

        v_per_hz = await self.calibrate_kv_rating(unwrapped_position_scale)
        await self.check_for_fault()

        # Rezero the servo since we just spun it a lot.
        await self.command("d rezero")

        if not self.args.cal_no_update:
            print("Saving to persistent storage")
            await self.command("conf write")

        print("Calibration complete")

        device_info = await self.get_device_info()

        now = datetime.datetime.utcnow()

        report = {
            'timestamp' : now.strftime('%Y-%m-%d %H:%M:%S.%f'),
            'device_info' : device_info,
            'calibration' : cal_result.to_json(),
            'winding_resistance' : winding_resistance,
            'v_per_hz' : v_per_hz,
            # We measure voltage to the center, not peak-to-peak, thus
            # the extra 0.5.
            'kv' : (0.5 * 60.0 / v_per_hz),
            'unwrapped_position_scale' : unwrapped_position_scale
        }

        log_filename = f"moteus-cal-{device_info['serial_number']}-{now.strftime('%Y%m%dT%H%M%S.%f')}.log"

        print(f"REPORT: {log_filename}")
        print(f"------------------------")

        print(json.dumps(report, indent=2))

        print()

        with open(os.path.join(_get_log_directory(), log_filename), "w") as fp:
            json.dump(report, fp, indent=2)
            fp.write("\n")

    async def calibrate_encoder_mapping(self):
        await self.command(f"d pwm 0 {self.args.cal_power}")
        await asyncio.sleep(3.0)

        await self.command("d stop")
        await asyncio.sleep(0.1)
        await self.write_message(
            (f"d cal {self.args.cal_power} s{self.args.cal_speed}"))

        cal_data = b''
        index = 0
        while True:
            line = (await self.stream.readline()).strip()
            if not self.args.verbose:
                print("Calibrating {} ".format("/-\\|"[index]), end='\r', flush=True)
                index = (index + 1) % 4
            cal_data += (line + b'\n')
            if line.startswith(b'CAL done'):
                break
            if line.startswith(b'CAL start'):
                continue
            if line.startswith(b'CAL'):
                # Some problem
                raise RuntimeError(f'Error calibrating: {line}')

        if self.args.cal_raw:
            with open(self.args.cal_raw, "wb") as f:
                f.write(cal_data)

        cal_file = ce.parse_file(io.BytesIO(cal_data))
        cal_result = ce.calibrate(cal_file)

        if cal_result.errors:
            raise RuntimeError(f"Error(s) calibrating: {cal_result.errors}")

        if not self.args.cal_no_update:
            print("\nStoring encoder config")
            await self.command(f"conf set motor.poles {cal_result.poles}")
            await self.command("conf set motor.invert {}".format(
                1 if cal_result.invert else 0))
            for i, offset in enumerate(cal_result.offset):
                await self.command(f"conf set motor.offset.{i} {offset}")

        return cal_result

    async def find_current(self, voltage):
        assert voltage < 3.0
        assert voltage >= 0.0

        await self.command(f"d pwm 0 {voltage:.3f}")

        # Wait a bit for it to stabilize.
        await asyncio.sleep(0.3)

        # Now get the servo_stats telemetry channel to read the D and Q
        # currents.
        data = await self.read_data("servo_stats")

        # Stop the current.
        await self.command("d stop");

        # Sleep a tiny bit before returning.
        await asyncio.sleep(0.1);

        d_cur = data.d_A;
        q_cur = data.q_A;

        current_A = math.hypot(d_cur, q_cur)
        print(f"{voltage}V - {current_A}A")

        return current_A

    async def calibrate_winding_resistance(self):
        print("Calculating winding resistance")

        ratios = [ 0.5, 0.6, 0.7, 0.85, 1.0 ]
        voltages = [x * self.args.cal_voltage for x in ratios]
        currents = [await self.find_current(voltage) for voltage in voltages]

        winding_resistance = _calculate_winding_resistance(voltages, currents)

        if not self.args.cal_no_update:
            await self.command(f"conf set motor.resistance_ohm {winding_resistance}")

        return winding_resistance

    async def find_speed(self, voltage):
        assert voltage < 1.0
        assert voltage >= 0.0

        await self.command(f"d vdq 0 {voltage:.3f}")

        # Wait for it to stabilize.
        await asyncio.sleep(1.0)

        data = await self.read_data("servo_stats")
        velocity = data.velocity

        print(f"{voltage}V - {velocity}Hz")

        return velocity

    async def calibrate_kv_rating(self, unwrapped_position_scale):
        print("Calculating Kv rating")

        original_position_min = await self.read_config_double("servopos.position_min")
        original_position_max = await self.read_config_double("servopos.position_max")

        await self.command("conf set servopos.position_min NaN")
        await self.command("conf set servopos.position_max NaN")
        await self.command("d index 0")

        voltages = [ 0.0, 0.2, 0.4, 0.6, 0.8 ]
        speed_hzs = [ await self.find_speed(voltage) for voltage in voltages]

        await self.command("d stop")

        await asyncio.sleep(0.5)

        geared_v_per_hz = 1.0 / _calculate_slope(voltages, speed_hzs)

        v_per_hz = geared_v_per_hz * unwrapped_position_scale
        print(f"v_per_hz (pre-gearbox)={v_per_hz}")

        if not self.args.cal_no_update:
            await self.command(f"conf set motor.v_per_hz {v_per_hz}")

        await self.command(f"conf set servopos.position_min {original_position_min}")
        await self.command(f"conf set servopos.position_max {original_position_max}")

        return v_per_hz

    async def do_restore_calibration(self, filename):
        report = json.load(open(filename, "r"))

        # Verify that the serial number matches.
        device_info = await self.get_device_info()
        if device_info['serial_number'] != report['device_info']['serial_number']:
            raise RuntimeError(
                f"Serial number in calibration ({report['serial_number']}) " +
                f"does not match device ({device_info['serial_number']})")

        cal_result = report['calibration']

        await self.command(
            f"conf set motor.poles {cal_result['poles']}")
        await self.command(
            f"conf set motor.invert {1 if cal_result['invert'] else 0}")
        for index, offset in enumerate(cal_result['offset']):
            await self.command(f"conf set motor.offset.{index} {offset}")

        await self.command(f"conf set motor.resistance_ohm {report['winding_resistance']}")
        await self.command(f"conf set motor.v_per_hz {report['v_per_hz']}")
        await self.command("conf write")

        print("Calibration restored")


class Runner:
    def __init__(self, args):
        self.args = args
        self.cmdline_targets = _expand_targets(args.target)

        # Was our target list found through discovery?
        self._discovered = False

    async def start(self):
        self.transport = moteus.get_singleton_transport(self.args)
        targets = await self.find_targets()

        for target in targets:
            if self._discovered:
                print(f"Target: {target}")
            await self.run_action(target)

    async def find_targets(self):
        if self.cmdline_targets:
            return self.cmdline_targets

        result = []
        self._discovered = True

        for i in range(1, 127):
            c = moteus.Controller(id=i, transport=self.transport)
            try:
                _ = await asyncio.wait_for(c.query(), 0.01)
                result.append(i)
            except asyncio.TimeoutError:
                pass

        return result

    async def run_action(self, target_id):
        stream = Stream(self.args, target_id, self.transport)

        if self.args.console:
            await stream.do_console()
        elif self.args.stop:
            await stream.command("d stop")
        elif self.args.dump_config:
            print((await stream.command("conf enumerate")).decode('latin1'))
        elif self.args.info:
            await stream.info()
        elif self.args.zero_offset:
            await stream.do_zero_offset()
        elif self.args.write_config:
            await stream.do_write_config(self.args.write_config)
        elif self.args.flash:
            await stream.do_flash(self.args.flash)
        elif self.args.calibrate:
            await stream.do_calibrate()
        elif self.args.restore_cal:
            await stream.do_restore_calibration(self.args.restore_cal)
        else:
            raise RuntimeError("No action specified")


async def async_main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        '-t', '--target', type=str, action='append', default=[],
        help='destination address(es) (default: autodiscover)')
    parser.add_argument('-v', '--verbose', action='store_true')

    moteus.make_transport_args(parser)

    group = parser.add_mutually_exclusive_group()

    group.add_argument('-s', '--stop', action='store_true',
                       help='command the servos to stop')
    group.add_argument('-i', '--info', action='store_true',
                       help='display information from the servo')
    group.add_argument('-c', '--console', action='store_true',
                       help='create a serial console')
    group.add_argument('--dump-config', action='store_true',
                       help='emit all configuration to the console')
    group.add_argument('--write-config', metavar='FILE',
                       help='write the given configuration')
    group.add_argument('--flash', metavar='FILE',
                       help='write the given elf file to flash')

    parser.add_argument('--no-restore-config', action='store_true',
                        help='do not restore config after flash')
    parser.add_argument('--bootloader-active', action='store_true',
                        help='bootloader is already active')

    group.add_argument('--calibrate', action='store_true',
                        help='calibrate the motor, requires full freedom of motion')
    parser.add_argument('--cal-no-update', action='store_true',
                        help='do not store calibration results on motor')
    parser.add_argument('--cal-power', metavar='V', type=float, default=0.4,
                        help='voltage to use during calibration')
    parser.add_argument('--cal-speed', metavar='HZ', type=float, default=1.0,
                        help='speed in electrical rps')
    parser.add_argument('--cal-voltage', metavar='V', type=float, default=0.45,
                        help='maximum voltage when measuring resistance')
    parser.add_argument('--cal-raw', metavar='FILE', type=str,
                        help='write raw calibration data')

    group.add_argument('--restore-cal', metavar='FILE', type=str,
                        help='restore calibration from logged data')
    group.add_argument('--zero-offset', action='store_true',
                        help='set the motor\'s position offset')

    args = parser.parse_args()

    runner = Runner(args)
    await runner.start()


def main():
    asyncio.run(async_main())


if __name__ == '__main__':
    main()
