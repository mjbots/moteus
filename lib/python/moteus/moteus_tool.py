#!/usr/bin/python3 -B

# Copyright 2019-2022 Josh Pieper, jjp@pobox.com.
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
import time
import uuid

from . import moteus
from . import aiostream
from . import regression
from . import calibrate_encoder as ce

MAX_FLASH_BLOCK_SIZE = 32


# For whatever reason, Windows can't reliably timeout with very
# short intervals.
FIND_TARGET_TIMEOUT = 0.01 if sys.platform != 'win32' else 0.05


class FirmwareUpgrade:
    '''This encodes "magic" rules about upgrading firmware, largely about
    how to munge configuration options so as to not cause behavior
    change upon firmware changes.
    '''

    def __init__(self, old, new):
        self.old = old
        self.new = new

        if new > 0x0105:
            raise RuntimeError("Firmware to be flashed has a newer version than we support")

    def fix_config(self, old_config):
        lines = old_config.split(b'\n')
        items = dict([line.split(b' ') for line in lines if b' ' in line])

        if self.new <= 0x0104 and self.old >= 0x0105:
            # For now, we will only attempt to downgrade a config
            # where there is only an onboard encoder and nothing else.
            if (int(items[b'aux1.spi.mode']) != 0 or
                int(items[b'motor_position.sources.0.type']) != 1 or
                int(items[b'motor_position.commutation_source']) != 0 or
                int(items[b'motor_position.output.source']) != 0 or
                int(items[b'motor_position.output.reference_source']) != -1 or
                float(items[b'motor_position.sources.0.offset']) != 0 or
                int(items[b'motor_position.output.sign']) != 1):
                print("WARNING: Cannot map 0x0105 encoder settings downgrading to 0x0104. " +
                      "Default onboard AS5047P will be used.")
            else:
                # Do our best to migrate the basic config.
                items[b'motor.invert'] = b'1' if int(items.pop(b'motor_position.sources.0.sign')) == -1 else b'0'
                new_offset = float(items[b'motor_position.output.offset'])

                scale = float(items[b'motor_position.rotor_to_output_ratio'])
                old_offset = int(new_offset / scale * 65536)
                items[b'motor.position_offset'] = str(old_offset).encode('utf-8')
                items[b'motor.unwrapped_position_scale'] = items.pop(b'motor_position.rotor_to_output_ratio')


            [items.pop(key) for key in
             [x for x in list(items.keys())
              if (x.startswith(b'motor_position.') or
                  x.startswith(b'aux1.') or
                  x.startswith(b'aux2.') or
                  x.startswith(b'icpz1.') or
                  x.startswith(b'icpz2.') or
                  x.startswith(b'motor.cogging_dq') or
                  x.startswith(b'servo.current_feedforward'))]]

        if self.new <= 0x0103 and self.old >= 0x0104:
            if (float(items.get(b'servo.pwm_comp_mag', 0.0)) == 0.005 and
                float(items.get(b'servo.pwm_comp_off', 0.0)) == 0.055):
                items[b'servo.pwm_comp_off'] = b'0.048'
                items[b'servo.pwm_comp_mag'] = b'0.011'
                items[b'servo.pwm_scale'] = b'1.15'
                print("Reverting PWM compensation for version 0x0103")

        if self.new <= 0x0102 and self.old >= 0x0103:
            if (float(items.get(b'servo.pwm_scale', 0.0)) == 1.15 and
                float(items.get(b'servo.pwm_comp_off', 0.0)) == 0.048):
                items[b'servo.pwm_comp_off'] = b'0.048'
                items[b'servo.pwm_comp_mag'] = b'0.003'
                # servo.pwm_scale doesn't exist in version 0x0102 and earlier
                print("Reverting servo.pwm_comp_mag from 0.011 to 0.003 for version 0x0102")
            if int(items[b'motor.phase_invert']) != 0:
                print("** WARNING ** Current calibration is not compatible, " +
                      "re-calibration required")
                items[b'motor.poles'] = b'0'

        if self.new <= 0x0100 and self.old == 0x0101:
            # To get back to identical behavior, we apply the inverse
            # mapping.
            if float(items[b'servo.feedforward_scale']) == 0.5:
                items[b'servo.feedforward_scale'] = b'1.0'
                print("Reverting servo.feedforward_scale from 0.5 to 1.0 for version 0x0101")

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

        if self.new == 0x0101 and self.old <= 0x0100:
            # If the old firmware had the default feedforward term of
            # 1.0, then switch it to be the new default of 0.5.
            if float(items[b'servo.feedforward_scale']) == 1.0:
                items[b'servo.feedforward_scale'] = b'0.5'
                print("Changing servo.feedforward_scale from 1.0 to 0.5 for version 0x0101")
            else:
                items[b'servo.feedforward_scale'] = b'0.0'
                print("Changing servo.feedforward_scale to 0.0 for version 0x0101")

        if self.new >= 0x0103 and self.old <= 0x0102:
            if float(items.get(b'servo.pwm_comp_mag', 0.0)) == 0.003:
                items[b'servo.pwm_scale'] = b'1.15'
                items[b'servo.pwm_comp_off'] = b'0.048'
                items[b'servo.pwm_comp_mag'] = b'0.011'
                print("Upgrading servo.pwm_scale to 1.15 for version 0x0103")

        if self.new >= 0x0104 and self.old <= 0x0103:
            if (float(items.get(b'servo.pwm_comp_mag', 0.0)) == 0.011 and
                float(items.get(b'servo.pwm_comp_off', 0.0)) == 0.048):
                items[b'servo.pwm_comp_off'] = b'0.055'
                items[b'servo.pwm_comp_mag'] = b'0.005'
                items[b'servo.pwm_scale'] = b'1.00'
                print("Upgrading PWM compensation for version 0x0104")

        if self.new >= 0x0105 and self.old <= 0x0104:
            # The encoder and auxiliary port subsystems changed with
            # 0x0105.  We have to map a possible external primary
            # encoder, or a possible auxiliary I2C encoder onto the
            # new possibilities.
            if b'abs_port.i2c_mode' in items:
                items[b'aux2.i2c.i2c_hz'] = items[b'abs_port.i2c_hz']
                items[b'aux2.i2c.i2c_mode'] = items[b'abs_port.i2c_mode']
                if int(items[b'abs_port.mode']) != 0:
                    items[b'aux2.i2c.devices.0.type'] = items[b'abs_port.mode']
                    items[b'aux2.i2c.devices.0.address'] = items[b'abs_port.encoder_i2c_address']
                    items[b'aux2.i2c.devices.0.poll_ms'] = items[b'abs_port.encoder_poll_ms']
                    items[b'aux2.pins.0.mode'] = b'13' # kI2C
                    items[b'aux2.pins.1.mode'] = b'13' # kI2C
                    items[b'aux2.pins.0.pull'] = b'1' # kPullUp
                    items[b'aux2.pins.1.pull'] = b'1' # kPullUp
                if int(items.get(b'servo.rezero_from_abs', 0)) != 0:
                    items[b'motor_position.sources.1.type'] = b'7'  # I2C
                    items[b'motor_position.sources.1.aux_number'] = b'2'
                    items[b'motor_position.sources.1.i2c_device'] = b'0'
                    items[b'motor_position.sources.1.offset'] = items[b'abs_port.position_offset']
                    items[b'motor_position.sources.1.reference'] = b'1' # output
                    items[b'motor_position.sources.1.pll_filter_hz'] = b'10'
                    abs_mode = int(items[b'abs_port.mode'])

                    # The AS5600 has a CPR of 16384, whereas the
                    # AS5048A has a CPR of 65536
                    items[b'motor_position.sources.1.cpr'] = (
                        b'65536' if abs_mode == 1 else b'16384')
                    if float(items[b'abs_port.position_scale']) == -1.0:
                        items[b'motor_position.sources.1.sign'] = b'-1'
                    elif float(items[b'abs_port.position_scale']) != 1.0:
                        print("WARNING: CANNOT MAP abs_port.position_scale != +-1")
                    items[b'motor_position.output.reference_source'] = b'1'

                # Remove the old defunct config items.
                [items.pop(key) for key in
                 [x for x in list(items.keys()) if x.startswith(b'abs_port.')]]
                items.pop(b'servo.rezero_from_abs')

            if b'encoder.mode' in items:
                # Now check to see if an external encoder was configured.
                if int(items.get(b'encoder.mode', 0)) != 0:
                    # The only other type that was supported previously
                    # was an external AS5047.

                    items[b'aux1.spi.mode'] = b'2'  # external as5047
                    items[b'aux1.pins.0.mode'] = b'2' # kSpiCs
                    items[b'aux1.pins.1.mode'] = b'1' # kSpi
                    items[b'aux1.pins.2.mode'] = b'1' # kSpi
                    items[b'aux1.pins.3.mode'] = b'1' # kSpi

                items.pop(b'encoder.mode')

            old_offset = float(items.pop(b'motor.position_offset'))
            scale = float(items[b'motor.unwrapped_position_scale'])
            new_offset = old_offset / 65536.0 * scale
            items[b'motor_position.output.offset'] = str(new_offset).encode('utf-8')
            items[b'motor_position.sources.0.sign'] = b'-1' if int(items.pop(b'motor.invert', 0)) else b'1'
            items[b'motor_position.rotor_to_output_ratio'] = items.pop(b'motor.unwrapped_position_scale')

            if b'servo.velocity_filter_length' in items:
                items.pop(b'servo.velocity_filter_length')

            if b'servo.encoder_filter.kp' in items:
                kp = float(items.pop(b'servo.encoder_filter.kp'))
                ki = float(items.pop(b'servo.encoder_filter.ki'))
                en = int(items.pop(b'servo.encoder_filter.enabled'))
                if en == 0:
                    pll_hz = 0
                else:
                    w_3db = kp / 2
                    pll_hz = w_3db / (2 * math.pi)
                items[b'motor_position.sources.0.pll_filter_hz'] = str(pll_hz).encode('utf-8')
                debug_override = int(items.pop(b'servo.encoder_filter.debug_override'))
                if debug_override >= 0:
                    items[b'motor_position.sources.0.debug_override'] = str(debug_override).encode('utf-8')

            # The default PWM rate changed from 40kHz to 30kHz, and
            # any values above 30kHz now result in 1/2 the control
            # rate.  Thus, for any rates that appear to be unchanged
            # from the default, we drop them to the new default of
            # 30kHz.
            if b'servo.pwm_rate_hz' in items:
                if int(items[b'servo.pwm_rate_hz']) == 40000:
                    print('Lowering PWM rate to 30kHz for version 0x0105')
                    items[b'servo.pwm_rate_hz'] = b'30000'

            print('Upgraded encoder configuration for version 0x0105')


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


def _round_nearest_4v(input_V):
    # We assume a minimum of 12V.
    if input_V < 14.0:
        return 12.0
    return round(input_V / 4) * 4


def expand_targets(targets):
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
        self.controller = moteus.Controller(target_id, transport=transport,
                                            can_prefix=args.can_prefix)
        self.stream = moteus.Stream(self.controller, verbose=args.verbose,
                                    channel=args.diagnostic_channel)

    async def do_console(self):
        console_stdin = aiostream.AioStream(sys.stdin.buffer.raw)
        console_stdout = aiostream.AioStream(sys.stdout.buffer.raw)
        dir1 = asyncio.create_task(_copy_stream(self.stream, console_stdout))
        dir2 = asyncio.create_task(_copy_stream(console_stdin, self.stream))
        done, pending = await asyncio.wait(
            [dir1, dir2], return_when=asyncio.FIRST_EXCEPTION)
        for i in done:
            e = i.exception()
            if e:
                raise e

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

    async def flush_read(self, *args, **kwargs):
        return await self.stream.flush_read(*args, **kwargs)

    async def read_config_int(self, name):
        result = await self.command(f"conf get {name}", allow_any_response=True)
        return int(result)

    async def read_config_double(self, name):
        result = await self.command(f"conf get {name}", allow_any_response=True)
        return float(result)

    async def is_config_supported(self, name):
        result = await self.command(f"conf get {name}", allow_any_response=True)
        if result.startswith(b'ERR'):
            if not (b'error reading' in result or b'unknown group' in result):
                raise RuntimeError(
                    f'Unknown error checking for {name}: {result}')
            return False
        return True

    async def read_uuid(self):
        try:
            text_data = await self.command("conf enumerate uuid")
            b = bytes([int(x.split(b' ')[1])
                       for x in text_data.split(b'\n') if len(x)])
            return uuid.UUID(bytes=b)
        except moteus.CommandError as ce:
            if not 'unknown group' in ce.message:
                raise
            return None

    async def get_device_info(self):
        firmware = await self.read_data("firmware")
        git = await self.read_data("git")
        uuid = await self.read_uuid()

        result = {}
        result['serial_number'] = _base64_serial_number(
            firmware.serial_number[0],
            firmware.serial_number[1],
            firmware.serial_number[2])
        result['model'] = f"{firmware.model:x}"
        result['git_hash'] = _make_git_hash(git.hash)
        result['git_dirty'] = getattr(git, 'dirty', 0) != 0
        result['git_timestamp'] = getattr(git, 'timestamp', 0)
        result['uuid'] = str(uuid) if uuid is not None else ''

        return result

    async def info(self):
        print(json.dumps(await self.get_device_info(), indent=2))

    async def do_set_offset(self, value):
        # First try the new way.
        try:
            await self.command(f"d cfg-set-output {value}")
        except moteus.CommandError:
            if value != 0.0:
                raise RuntimeError("Old firmware only supports 0 offset")
            # If that doesn't work, then try the old way.
            servo_stats = await self.read_servo_stats()
            position_raw = servo_stats.position_raw
            await self.command(f"conf set motor.position_offset {-position_raw:d}")

        await self.command("conf write")
        await self.command(f"d rezero {value}")


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
            # Get the current firmware version.
            try:
                old_firmware = await self.read_data("firmware")
            except:
                print("Could not read firmware version.  --bootloader-active may be necessary")
                raise

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
        next_block = None
        while True:
            next_block = write_ctx.get_next_block()
            cmd = f"w {next_block.address:x} {next_block.data.hex()}"

            result = await self.command(cmd)
            self._emit_flash_progress(write_ctx, "flashing")
            done = write_ctx.advance_block()
            if done:
                break

        # Write enough ff's to ensure we get to an even 8 byte
        # boundary.  Otherwise the bootloader may not actually flush
        # our writes out.
        final_address = next_block.address + len(next_block.data)
        remaining_to_flush = 8 - (final_address & 0x07)
        if remaining_to_flush:
            cmd = f"w {final_address:x} {'ff' * remaining_to_flush}"
            result = await self.command(cmd)

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

    async def read_servo_stats(self):
        servo_stats = await self.read_data("servo_stats")
        if servo_stats.mode == 1:
            raise RuntimeError(f"Controller reported fault: {int(servo_stats.fault)}")
        return servo_stats

    async def check_for_fault(self):
        await self.read_servo_stats()

    async def restore_config(self, old_config):
        new_config = []
        for line in old_config.split(b'\n'):
            if len(line.strip()) == 0:
                continue
            new_config.append(b'conf set ' + line + b'\n')
        await self.write_config_stream(io.BytesIO(b''.join(new_config)))
        await self.command("conf write")

        # Reset the controller so we're sure any config has taken
        # effect.
        #
        # We won't get a response to this, so don't look for one.
        await self.write_message("d reset")

    def calculate_calibration_parameters(self):
        # Check for deprecated arguments.
        def handle_deprecated(new_name, old_name):
            old_attr_name = old_name.replace('-', '_')
            new_attr_name = new_name.replace('-', '_')

            if ((getattr(self.args, old_attr_name) is not None) and
                (getattr(self.args, new_attr_name) is not None)):
                raise RuntimeError(f'Both the old deprecated --{old_name} and the new --{new_name} were specified')

            if (getattr(self.args, old_attr_name) is not None):
                print(f'WARNING: Using deprecated --{old_name}.  It will be removed soon, prefer --{new_name}')
                setattr(self.args, new_attr_name,
                        getattr(self.args, old_attr_name))

        handle_deprecated('cal-ll-encoder-voltage', 'cal-power')
        handle_deprecated('cal-ll-encoder-speed', 'cal-speed')
        handle_deprecated('cal-ll-resistance-voltage', 'cal-voltage')
        handle_deprecated('cal-ll-kv-voltage', 'cal-kv-voltage')

    async def find_resistance_cal_voltage(self, input_V):
        if self.args.cal_ll_resistance_voltage:
            return self.args.cal_ll_resistance_voltage
        else:
            # Progressively increase this value to roughly achieve our
            # desired power.
            cal_voltage = 0.01
            while True:
                print(f"Testing {cal_voltage:.3f}V for resistance",
                      end='\r', flush=True)
                this_current = await self.find_current(cal_voltage)
                power = this_current * cal_voltage
                if (power > self.args.cal_motor_power or
                    cal_voltage > (0.4 * input_V)):
                    break
                cal_voltage *= 1.1
            print()

            return cal_voltage

    async def do_calibrate(self):
        # Determine what our calibration parameters are.
        self.calculate_calibration_parameters()

        print("This will move the motor, ensure it can spin freely!")
        await asyncio.sleep(2.0)

        # Clear any faults that may be there.
        await self.command("d stop")

        if await self.is_config_supported("motor.unwrapped_position_scale"):
            unwrapped_position_scale = \
                await self.read_config_double("motor.unwrapped_position_scale")
        elif await self.is_config_supported("motor_position.rotor_to_output_ratio"):
            unwrapped_position_scale = \
                await self.read_config_double("motor_position.rotor_to_output_ratio")

        if await self.is_config_supported("servo.pwm_rate_hz"):
            pwm_rate_hz = await self.read_config_double("servo.pwm_rate_hz")
            control_rate_hz = pwm_rate_hz if pwm_rate_hz <= 40000 else pwm_rate_hz / 2
        else:
            # Supported firmware versions that are not configurable
            # are all 40kHz.
            control_rate_hz = 40000

        # The rest of the calibration procedure assumes that
        # phase_invert is 0.
        try:
            await self.command("conf set motor.phase_invert 0")
        except moteus.CommandError as e:
            # It is possible this firmware is too old to support
            # selecting the phase inversion.
            if not 'error setting' in e.message:
                raise
            pass

        # We have 3 things to calibrate.
        #  1) The encoder to phase mapping
        #  2) The winding resistance
        #  3) The Kv rating of the motor.
        input_V = _round_nearest_4v(
            (await self.read_servo_stats()).filt_bus_V)

        print("Starting calibration process")
        await self.check_for_fault()

        resistance_cal_voltage = await self.find_resistance_cal_voltage(input_V)
        print(f"Using {resistance_cal_voltage:.3f} V for resistance and inductance calibration")

        winding_resistance = await self.calibrate_winding_resistance(resistance_cal_voltage)
        await self.check_for_fault()

        cal_result = await self.calibrate_encoder_mapping(
            input_V, winding_resistance)
        await self.check_for_fault()

        # We use a larger voltage for inductance measurement to get a
        # more accurate value.  Since we switch back and forth at a
        # high rate, this doesn't actually use all that much power no
        # matter what we choose.
        inductance = await self.calibrate_inductance(2.0 * resistance_cal_voltage)
        await self.check_for_fault()

        kp, ki, torque_bw_hz = None, None, None
        if inductance:
            kp, ki, torque_bw_hz = \
                await self.calculate_bandwidth(winding_resistance, inductance,
                                               control_rate_hz)

            await self.command(f"conf set servo.pid_dq.kp {kp}")
            await self.command(f"conf set servo.pid_dq.ki {ki}")

            await self.check_for_fault()

        enc_kp, enc_ki, enc_bw_hz = await self.set_encoder_filter(
            torque_bw_hz, control_rate_hz=control_rate_hz)
        await self.check_for_fault()

        v_per_hz = await self.calibrate_kv_rating(
            input_V, unwrapped_position_scale)
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
            'inductance' : inductance,
            'pid_dq_kp' : kp,
            'pid_dq_ki' : ki,
            'torque_bw_hz' : torque_bw_hz,
            'encoder_filter_bw_hz' : enc_bw_hz,
            'encoder_filter_kp' : enc_kp,
            'encoder_filter_ki' : enc_ki,
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

    async def find_encoder_cal_voltage(self, input_V, winding_resistance):
        if self.args.cal_ll_encoder_voltage:
            return self.args.cal_ll_encoder_voltage

        # We're going to try and select a voltage to roughly achieve
        # "--cal-motor-power".
        return min(0.4 * input_V,
                   math.sqrt(self.args.cal_motor_power * winding_resistance))

    async def calibrate_encoder_mapping(self, input_V, winding_resistance):
        # Figure out what voltage to use for encoder calibration.
        encoder_cal_voltage = await self.find_encoder_cal_voltage(
            input_V, winding_resistance)
        self.encoder_cal_voltage = encoder_cal_voltage

        hall_configured = False

        # Verify that we have a non-hall absolute encoder as our
        # commutation sensor.
        if await self.is_config_supported("motor_position.commutation_source"):
            commutation_source = await self.read_config_int(
                "motor_position.commutation_source")
            encoder_type = await self.read_config_int(
                f"motor_position.sources.{commutation_source}.type")
            if encoder_type == 4:  # hall
                hall_configured = True

        if self.args.cal_hall:
            if not hall_configured:
                raise RuntimeError("--cal-hall specified, but hall sensors " +
                                   "not configured on device")
            return await self.calibrate_encoder_mapping_hall(encoder_cal_voltage)
        else:
            if hall_configured:
                raise RuntimeError(
                    "Cannot perform encoder mapping with hall sensors, " +
                    "use --cal-hall")
            try:
                return await self.calibrate_encoder_mapping_absolute(encoder_cal_voltage)
            except:
                # At least try to stop.
                await self.command("d stop")
                raise

    async def calibrate_encoder_mapping_hall(self, encoder_cal_voltage):
        if self.args.cal_motor_poles is None:
            raise RuntimeError(
                'hall effect calibration requires specifying --cal-motor-poles')

        commutation_source = await self.read_config_int(
            "motor_position.commutation_source")
        aux_number = await self.read_config_int(
            f"motor_position.sources.{commutation_source}.aux_number")

        hall_cal_data = []
        STEPS = 24
        for i in range(STEPS):
            phase = i / STEPS * 2 * math.pi
            await self.command(f"d pwm {phase} {encoder_cal_voltage}")
            await asyncio.sleep(0.5)
            motor_position = await self.read_data("motor_position")
            hall_cal_data.append(
                (phase, motor_position.sources[commutation_source].raw))

        if self.args.cal_raw:
            with open(self.args.cal_raw, "wb") as f:
                f.write(json.dumps(hall_cal_data, indent=2).encode('utf8'))

        await self.command("d stop")

        # See if we support phase_invert.
        allow_phase_invert = \
            await self.is_config_supported("motor.phase_invert")

        cal_result = ce.calibrate_hall(
            hall_cal_data,
            desired_direction=1 if not self.args.cal_invert else -1,
            allow_phase_invert=allow_phase_invert)

        if not self.args.cal_no_update:
            await self.command(f"conf set motor.poles {self.args.cal_motor_poles}")
            await self.command(f"conf set motor_position.sources.{commutation_source}.sign {cal_result.sign}")
            await self.command(f"conf set motor_position.sources.{commutation_source}.offset {cal_result.offset}")
            await self.command(f"conf set aux{aux_number}.hall.polarity {cal_result.polarity}")
            if allow_phase_invert:
                await self.command(
                    f"conf set motor.phase_invert {1 if cal_result.phase_invert else 0}")

            for i in range(64):
                await self.command(f"conf set motor.offset.{i} 0")

        return cal_result

    async def find_index(self, encoder_cal_voltage):
        print("Searching for index")
        theta_speed = self.args.cal_ll_encoder_speed * 2 * math.pi
        await self.command(f"d pwm 0 {encoder_cal_voltage} {theta_speed}")
        start_time = time.time()
        while True:
            await asyncio.sleep(0.5)
            motor_position = await self.read_data("motor_position")
            if motor_position.homed != 0 or motor_position.theta_valid:
                break
            now = time.time()
            if (now - start_time) > 40.0:
                raise RuntimeError("Timeout searching for index")
        await self.command("d stop")

    async def ensure_valid_theta(self, encoder_cal_voltage):
        try:
            # We might need to have some sense of pole count first.
            if await self.read_config_double("motor.poles") == 0:
                # Pick something arbitrary for now.
                await self.command(f"conf set motor.poles 2")

            motor_position = await self.read_data("motor_position")
            if motor_position.homed == 0 and not motor_position.theta_valid:
                # We need to find an index.
                await self.find_index(encoder_cal_voltage)
        except RuntimeError:
            # Odds are we don't support motor_position, in which case
            # theta is always valid for the older versions that don't
            # support it.
            pass

    async def calibrate_encoder_mapping_absolute(self, encoder_cal_voltage):
        await self.ensure_valid_theta(encoder_cal_voltage)

        await self.command(f"d pwm 0 {encoder_cal_voltage}")
        await asyncio.sleep(3.0)

        await self.write_message(
            (f"d cal {encoder_cal_voltage} s{self.args.cal_ll_encoder_speed}"))

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
            if line.startswith(b'ERR'):
                raise RuntimeError(f'Error calibrating: {line}')
            if line.startswith(b'CAL'):
                # Some problem
                raise RuntimeError(f'Error calibrating: {line}')

        if self.args.cal_raw:
            with open(self.args.cal_raw, "wb") as f:
                f.write(cal_data)

        cal_file = ce.parse_file(io.BytesIO(cal_data))

        # See if we support phase_invert.
        allow_phase_invert = \
            await self.is_config_supported("motor.phase_invert")

        cal_result = ce.calibrate(
            cal_file,
            desired_direction=1 if not self.args.cal_invert else -1,
            max_remainder_error=self.args.cal_max_remainder,
            allow_phase_invert=allow_phase_invert)

        if cal_result.errors:
            raise RuntimeError(f"Error(s) calibrating: {cal_result.errors}")

        if self.args.cal_motor_poles is not None:
            if self.args.cal_motor_poles != cal_result.poles:
                raise RuntimeError(
                    f"Auto-detected pole count ({cal_result.poles}) != " +
                    f"cmdline specified ({self.args.cal_motor_poles})")

        if not self.args.cal_no_update:
            print("\nStoring encoder config")
            await self.command(f"conf set motor.poles {cal_result.poles}")

            if await self.is_config_supported("motor_position.sources.0.sign"):
                await self.command("conf set motor_position.sources.0.sign {}".format(
                    -1 if cal_result.invert else 1))
            else:
                await self.command("conf set motor.invert {}".format(
                    1 if cal_result.invert else 0))
            if allow_phase_invert:
                await self.command("conf set motor.phase_invert {}".format(
                    1 if cal_result.phase_invert else 0))
            for i, offset in enumerate(cal_result.offset):
                await self.command(f"conf set motor.offset.{i} {offset}")

        return cal_result

    async def find_current(self, voltage):
        assert voltage < 20.0
        assert voltage >= 0.0

        await self.command(f"d pwm 0 {voltage:.3f}")

        # Wait a bit for it to stabilize.
        await asyncio.sleep(0.15)

        def extract(f):
            return math.hypot(f.d_A, f.q_A)

        # Now get the servo_stats telemetry channel to read the D and Q
        # currents.
        data = [extract(await self.read_servo_stats()) for _ in range(10)]

        # Stop the current.
        await self.command("d stop");

        # Sleep a tiny bit before returning.
        await asyncio.sleep(0.05);

        current_A = sum(data) / len(data)

        return current_A

    async def find_current_and_print(self, voltage):
        result = await self.find_current(voltage)
        print(f"{voltage:.3f}V - {result:.3f}A")
        return result

    async def calibrate_winding_resistance(self, cal_voltage):
        print("Calculating winding resistance")

        ratios = [ 0.5, 0.6, 0.7, 0.85, 1.0 ]
        voltages = [x * cal_voltage for x in ratios]
        currents = [await self.find_current_and_print(voltage)
                    for voltage in voltages]

        winding_resistance = _calculate_winding_resistance(voltages, currents)

        if winding_resistance < 0.001:
            raise RuntimeError(
                f'Winding resistance too small ({winding_resistance} < 0.001)' +
                f', try adjusting --cal-voltage')

        if not self.args.cal_no_update:
            await self.command(f"conf set motor.resistance_ohm {winding_resistance}")

        return winding_resistance

    async def calibrate_inductance(self, cal_voltage):
        print("Calculating motor inductance")

        try:
            await asyncio.wait_for(
                self.command(f"d ind {cal_voltage} 4"), 0.25)
        except moteus.CommandError as e:
            # It is possible this is an old firmware that does not
            # support inductance measurement.
            if not 'unknown command' in e.message:
                raise
            print("Firmware does not support inductance measurement")
            return None
        except asyncio.TimeoutError:
            print("Firmware does not support inductance measurement")
            return None

        start = time.time()
        await asyncio.sleep(1.0)
        await self.command(f"d stop")
        end = time.time()
        data = await self.read_servo_stats()

        delta_time = end - start
        inductance = (cal_voltage /
                      (data.meas_ind_integrator / delta_time))

        if inductance < 1e-6:
            raise RuntimeError(f'Inductance too small ({inductance} < 1e-6)')

        print(f"Calculated inductance: {inductance}H")
        return inductance

    async def set_encoder_filter(self, torque_bw_hz, control_rate_hz = None):
        # Check to see if our firmware supports encoder filtering.
        motor_position_style = await self.is_config_supported("motor_position.sources.0.pll_filter_hz")
        servo_style = await self.is_config_supported("servo.encoder_filter.enabled")
        if not motor_position_style and not servo_style:
            return None, None, None

        if self.args.encoder_bw_hz:
            desired_encoder_bw_hz = self.args.encoder_bw_hz
        else:
            if self.args.cal_hall:
                # Hall effect configurations require a low encoder BW
                # if the hall effects are also used for position and
                # velocity control.  Since that is one of the most
                # common ways of using hall effects, we by default cap
                # the bw at 20Hz and use a lower one if the torque bw
                # would otherwise have been lower.
                desired_encoder_bw_hz = min(20, 2 * torque_bw_hz)
            else:
                # We default to an encoder bandwidth of 50Hz, or 2x the
                # torque bw, whichever is larger.
                desired_encoder_bw_hz = max(50, 2 * torque_bw_hz)

        # And our bandwidth with the filter can be no larger than
        # 1/10th the control rate.
        encoder_bw_hz = min(control_rate_hz / 10, desired_encoder_bw_hz)

        if encoder_bw_hz != desired_encoder_bw_hz:
            print(f"Warning: using lower encoder filter than "+
                  f"requested: {encoder_bw_hz:.1f}Hz")

        w_3db = encoder_bw_hz * 2 * math.pi
        kp = 2 * w_3db
        ki = w_3db * w_3db

        if servo_style:
            await self.command(f"conf set servo.encoder_filter.enabled 1")
            await self.command(f"conf set servo.encoder_filter.kp {kp}")
            await self.command(f"conf set servo.encoder_filter.ki {ki}")
        elif motor_position_style:
            await self.command(f"conf set motor_position.sources.0.pll_filter_hz {encoder_bw_hz}")
        else:
            assert False
        return kp, ki, encoder_bw_hz

    async def calculate_bandwidth(self, resistance, inductance, control_rate_hz = None):
        twopi = 2 * math.pi

        if await self.is_config_supported("servo.current_sense_ohm"):
            current_sense_ohm = await self.read_config_double(
                "servo.current_sense_ohm")
        else:
            current_sense_ohm = 0.0005

        current_sense_scale = current_sense_ohm / 0.0005

        # We have several factors that can limit the bandwidth:

        # First, is that the controller operates its control loop at a
        # fixed rate.  We will limit the max bandwidth to 30x less
        # than that for now, so that we do not need to consider
        # discretization.
        board_limit_rad_s = (control_rate_hz / 30) * 2 * math.pi

        # Second, we limit the bandwidth such that the Kp value is not
        # too large.  The current sense noise on the moteus controller
        # limits how large Kp can get before the loop becomes unstable
        # or very noisy.
        kp_limit_rad_s = current_sense_scale * 0.20 / inductance

        # Finally, we limit the bandwidth such that the Ki value is
        # not too large.
        ki_limit_rad_s = current_sense_scale * 500.0 / resistance

        cal_bw_rad_s = self.args.cal_bw_hz * twopi

        w_3db = min(board_limit_rad_s,
                    kp_limit_rad_s,
                    ki_limit_rad_s,
                    cal_bw_rad_s)

        if w_3db != cal_bw_rad_s:
            print(f"Warning: using lower bandwidth " +
                  f"than requested: {w_3db/twopi:.1f}Hz")

        kp = w_3db * inductance
        ki = w_3db * resistance

        print(f"Calculated kp/ki: {kp}/{ki}")

        return kp, ki, w_3db / twopi

    async def find_speed(self, voltage, sleep_time=0.5):
        assert voltage < 20.0
        assert voltage >= 0.0

        await self.command(f"d vdq 0 {voltage:.3f}")

        # Wait for it to stabilize.
        await asyncio.sleep(sleep_time)

        start_time = time.time()

        def sign(x):
            return 1 if x >= 0 else -1

        old_change = None
        old_vel = None
        while True:
            data = await self.read_servo_stats()
            velocity = data.velocity

            # As a fallback, timeout after 1s of waiting.
            if (time.time() - start_time) > 2.0:
                return velocity

            if abs(velocity) < 0.2:
                return velocity

            if old_vel is not None:
                change = velocity - old_vel
                if old_change is not None:
                    if sign(old_change) != sign(change):
                        return velocity
                old_change = change

            old_vel = velocity

            await asyncio.sleep(0.1)

        return velocity

    async def find_speed_and_print(self, voltage, **kwargs):
        result = await self.find_speed(voltage, **kwargs)
        print(f"{voltage:.3f}V - {result:.3f}Hz")
        return result

    async def find_kv_cal_voltage(self, input_V, unwrapped_position_scale):
        if self.args.cal_ll_kv_voltage:
            return self.args.cal_ll_kv_voltage

        # Otherwise, we start small, and increase until we hit a
        # reasonable speed.
        maybe_result = 0.01
        while True:
            print(f"Testing {maybe_result:.3f}V for Kv",
                  end='\r', flush=True)
            if maybe_result > (0.2 * input_V):
                return maybe_result

            this_speed = await self.find_speed(maybe_result) / unwrapped_position_scale
            # Aim for this many Hz
            if abs(this_speed) > self.args.cal_motor_speed:
                break
            maybe_result *= 1.1

        print()
        return maybe_result

    async def calibrate_kv_rating(self, input_V, unwrapped_position_scale):
        print("Calculating Kv rating")

        await self.ensure_valid_theta(self.encoder_cal_voltage)

        original_position_min = await self.read_config_double("servopos.position_min")
        original_position_max = await self.read_config_double("servopos.position_max")

        await self.command("conf set servopos.position_min NaN")
        await self.command("conf set servopos.position_max NaN")
        await self.command("d index 0")

        kv_cal_voltage = await self.find_kv_cal_voltage(input_V, unwrapped_position_scale)
        await self.stop_and_idle()

        voltages = [x * kv_cal_voltage for x in [
            0.0, 0.25, 0.5, 0.75, 1.0 ]]
        speed_hzs = [ await self.find_speed_and_print(voltage, sleep_time=2)
                      for voltage in voltages]

        await self.stop_and_idle()

        await asyncio.sleep(0.5)

        geared_v_per_hz = 1.0 / _calculate_slope(voltages, speed_hzs)

        v_per_hz = geared_v_per_hz * unwrapped_position_scale
        print(f"v_per_hz (pre-gearbox)={v_per_hz}")

        if not self.args.cal_no_update:
            await self.command(f"conf set motor.v_per_hz {v_per_hz}")

        await self.command(f"conf set servopos.position_min {original_position_min}")
        await self.command(f"conf set servopos.position_max {original_position_max}")

        return v_per_hz

    async def stop_and_idle(self):
        await self.command("d stop")

        stop_count = 0
        while True:
            servo_stats = await self.read_servo_stats()
            if hasattr(servo_stats, 'velocity_filt'):
                if abs(servo_stats.velocity_filt) < 0.3:
                    stop_count += 1
                else:
                    stop_count = 0
            else:
                if abs(servo_stats.velocity) < 0.3:
                    stop_count += 1
                else:
                    stop_count = 0
            if stop_count > 5:
                return
            await asyncio.sleep(0.2)


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
        if await self.is_config_supported("motor_position.sources.0.sign"):
            await self.command(f"conf set motor_position.sources.0.sign {-1 if cal_result['invert'] else 1}")
        else:
            await self.command(
                f"conf set motor.invert {1 if cal_result['invert'] else 0}")
        if await self.is_config_supported("motor.phase_invert"):
            phase_invert = cal_result.get('phase_invert', False)
            await self.command(
                f"conf set motor.phase_invert {1 if phase_invert else 0}")
        for index, offset in enumerate(cal_result['offset']):
            await self.command(f"conf set motor.offset.{index} {offset}")

        await self.command(f"conf set motor.resistance_ohm {report['winding_resistance']}")
        await self.command(f"conf set motor.v_per_hz {report['v_per_hz']}")

        pid_dq_kp = report.get('pid_dq_kp', None)
        if pid_dq_kp is not None:
            await self.command(f"conf set servo.pid_dq.kp {pid_dq_kp}")

        pid_dq_ki = report.get('pid_dq_ki', None)
        if pid_dq_ki is not None:
            await self.command(f"conf set servo.pid_dq.ki {pid_dq_ki}")

        enc_kp = report.get('encoder_filter_kp', None)
        enc_ki = report.get('encoder_filter_ki', None)
        enc_hz = report.get('encoder_filter_bw_hz', None)
        if (enc_hz and
            await self.is_config_supported(
                "motor_position.sources.0.pll_filter_hz")):
            await self.command(
                f"conf set motor_position.sources.0.pll_filter_hz {enc_hz}")
        elif await self.is_config_supported("servo.encoder_filter.kp"):
            if enc_kp:
                await self.command(f"conf set servo.encoder_filter.kp {enc_kp}")
            if enc_ki:
                await self.command(f"conf set servo.encoder_filter.ki {enc_ki}")

        await self.command("conf write")

        print("Calibration restored")


class Runner:
    def __init__(self, args):
        self.args = args
        self.cmdline_targets = expand_targets(args.target)

        # Was our target list found through discovery?
        self._discovered = False

    async def start(self):
        self.transport = moteus.get_singleton_transport(self.args)
        targets = await self.find_targets()

        for target in targets:
            if self._discovered or len(targets) > 1:
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
                response = await asyncio.wait_for(
                    c.query(), FIND_TARGET_TIMEOUT)
                if response:
                    result.append(i)
            except asyncio.TimeoutError:
                pass

        return result

    def default_tel_stop(self):
        # The user might want to see what the device is spewing.
        if self.args.console:
            return False

        # If the bootloader is active, then it doesn't even know about
        # "tel stop".
        if self.args.flash and self.args.bootloader_active:
            return False

        # If we're just sending a "d stop" command, keep this simple so
        # we are more likely to actually stop the device.
        if self.args.stop:
            return False

        return True

    async def run_action(self, target_id):
        stream = Stream(self.args, target_id, self.transport)

        tel_stop = self.default_tel_stop()
        if self.args.tel_stop:
            tel_stop = True
        if self.args.no_tel_stop:
            tel_stop = False

        if tel_stop:
            await stream.write_message("tel stop")

            # Discard anything that might have been en route.
            await stream.flush_read()


        if self.args.console:
            await stream.do_console()
        elif self.args.stop:
            await stream.command("d stop")
        elif self.args.dump_config:
            print((await stream.command("conf enumerate")).decode('latin1'))
        elif self.args.info:
            await stream.info()
        elif self.args.zero_offset:
            await stream.do_set_offset(0.0)
        elif self.args.set_offset:
            await stream.do_set_offset(self.args.set_offset)
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
    parser.add_argument('--can-prefix', type=int, default=0)
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('--tel-stop', action='store_true',
                        help='force sending a "tel stop"')
    parser.add_argument('--no-tel-stop', action='store_true',
                        help='prevent sending a "tel stop"')

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

    group.add_argument('--restore-cal', metavar='FILE', type=str,
                        help='restore calibration from logged data')
    group.add_argument('--zero-offset', action='store_true',
                        help='set the motor\'s position offset')
    group.add_argument('--set-offset', metavar='O',
                       type=float,
                       help='set the motor\'s position offset')


    parser.add_argument('--diagnostic-channel', type=int, default=1,
                        help='diagnostic channel to use for --console')

    # Top level calibration parameters.
    parser.add_argument('--cal-invert', action='store_true',
                        help='if set, then commands and encoder will oppose')
    parser.add_argument('--cal-hall', action='store_true',
                        help='calibrate a motor with hall commutation sensors')

    parser.add_argument('--cal-bw-hz', metavar='HZ', type=float,
                        default=100.0,
                        help='configure current loop bandwidth in Hz')
    parser.add_argument('--encoder-bw-hz', metavar='HZ', type=float,
                        default=None,
                        help='override the encoder filter bandwidth in Hz')
    parser.add_argument('--cal-no-update', action='store_true',
                        help='do not store calibration results on motor')

    # These calibration values are low-level ones.  They are mostly
    # all correlated, and it is not that easy for users to even know
    # which of these would be useful to change.  We're leaving them
    # here for now so that the defaults can be overridden if
    # necessary.
    parser.add_argument('--cal-ll-encoder-voltage',
                        metavar='V', type=float,
                        help='voltage to use during calibration')
    parser.add_argument('--cal-ll-encoder-speed',
                        metavar='HZ', type=float, default=1.0,
                        help='speed in electrical rps')
    parser.add_argument('--cal-ll-resistance-voltage',
                        metavar='V', type=float,
                        help='maximum voltage when measuring resistance')
    parser.add_argument('--cal-ll-kv-voltage',
                        metavar='V', type=float,
                        help='maximum voltage when measuring Kv')


    # These are the "legacy" names of the low-level parameters.
    parser.add_argument('--cal-power', metavar='V', type=float,
                        help='[DEPRECATED] voltage to use during calibration')
    parser.add_argument('--cal-speed',
                        metavar='HZ', type=float,
                        help='[DEPRECATED] speed in electrical rps')
    parser.add_argument('--cal-voltage', metavar='V', type=float,
                        help='[DEPRECATED] maximum voltage when measuring resistance')
    parser.add_argument('--cal-kv-voltage', metavar='V', type=float,
                        help='[DEPRECATED] maximum voltage when measuring Kv')



    # These calibration are intended to be "higher level".
    # Internally, the above values are derived from these, combined
    # with the approximate input voltage to the controller.
    parser.add_argument('--cal-motor-power', metavar='W', type=float,
                        default=5.0,
                        help='motor power in W to use for encoder cal')
    parser.add_argument('--cal-motor-speed', metavar='Hz', type=float,
                        default=6.0,
                        help='max motor mechanical speed to use for kv cal')

    parser.add_argument('--cal-motor-poles', metavar='N', type=int,
                        default=None,
                        help='number of motor poles (2x pole pairs)')


    parser.add_argument('--cal-max-remainder', metavar='F',
                        type=float, default=0.1,
                        help='maximum allowed error in calibration')
    parser.add_argument('--cal-raw', metavar='FILE', type=str,
                        help='write raw calibration data')

    args = parser.parse_args()

    runner = Runner(args)
    await runner.start()


def main():
    asyncio.run(async_main())


if __name__ == '__main__':
    main()
