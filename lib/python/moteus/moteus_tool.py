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

# By default, we will only use current mode calibration if our
# expected maximum current is X times the sense noise in current.
CURRENT_QUALITY_MIN = 20

# We switch to voltage mode control if the ratio of maximum possible
# current to current noise is less than this amount.
VOLTAGE_MODE_QUALITY_MIN = 40

def _wrap_neg_pi_to_pi(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def lerp(array, ratio):
    array_size = len(array)

    left_index = int(math.floor(ratio * array_size))
    right_index = (left_index + 1) % array_size
    fraction = (ratio - left_index / array_size) * array_size
    left_comp = array[left_index]
    right_comp = array[right_index]

    return (left_comp * (1.0 - fraction)) + (right_comp * fraction)


def stddev(data):
    if len(data) == 0:
        return 0

    mean = sum(data) / len(data)
    return math.sqrt(sum((x - mean) ** 2 for x in data) / len(data))

SUPPORTED_ABI_VERSION = 0x010a

# Old firmwares used a slightly incorrect definition of Kv/v_per_hz
# that didn't match with vendors or oscilloscope tests.
V_PER_HZ_FUDGE_010a = 1.09

class FirmwareUpgrade:
    '''This encodes "magic" rules about upgrading firmware, largely about
    how to munge configuration options so as to not cause behavior
    change upon firmware changes.
    '''

    def __init__(self, old, new, board_family):
        self.old = old
        self.new = new
        self.board_family = board_family

        if new > SUPPORTED_ABI_VERSION:
            raise RuntimeError(f"\nmoteus_tool needs to be upgraded to support this firmware\n\n (likely 'python -m pip install --upgrade moteus')\n\nThe provided firmare is ABI version 0x{new:04x} but this moteus_tool only supports up to 0x{SUPPORTED_ABI_VERSION:04x}")

        if old > SUPPORTED_ABI_VERSION:
            raise RuntimeError(f"\nmoteus_tool needs to be upgraded to support this board\n\n (likely 'python -m pip install --upgrade moteus')\n\nThe board firmware is ABI version 0x{old:04x} but this moteus_tool only supports up to 0x{SUPPORTED_ABI_VERSION:04x}")

    def fix_config(self, old_config):
        lines = old_config.split(b'\n')
        items = dict([line.split(b' ') for line in lines if b' ' in line])

        if self.new <= 0x0109 and self.old >= 0x010a:
            kv = float(items.pop(b'motor.Kv'))

            v_per_hz = 0 if kv == 0 else ((V_PER_HZ_FUDGE_010a * 0.5 * 60) / kv)
            items[b'motor.v_per_hz'] = str(v_per_hz).encode('utf8')

            print(f"Downgrading motor.Kv to motor.v_per_hz and fixing fudge: old Kv={kv} v_per_hz={v_per_hz}")

            if b'servo.max_power_W' in items:
                newer_max_power_W = float(items.get(b'servo.max_power_W'))
                # If this is NaN, then we'll set it back to the board
                # default for the older firmware version.
                if not math.isfinite(newer_max_power_W):
                    items[b'servo.max_power_W'] = {
                        0 : b'450.0',
                        1 : b'450.0',
                        2 : b'100.0',
                        }[self.board_family or 0]
                else:
                    # If it was finite, then we'll try to set it
                    # appropriately based on what the PWM rate was.
                    # Firmware version 0x0109 and earlier used the
                    # configured power as if the PWM rate were 40kHz.
                    pwm_rate = float(items.get(b'servo.pwm_rate_hz'))

                    items[b'servo.max_power_W'] = str(newer_max_power_W * (40000.0 / pwm_rate)).encode('utf8')

                print(f"Downgrading servo.max_power_W to {items[b'servo.max_power_W'].decode('utf8')} for firmware <= 0x0109")


        if self.new <= 0x0108 and self.old >= 0x0109:
            # When downgrading, we should warn if a motor thermistor
            # value other than 47k is configured and enabled.
            if (int(items.get(b'servo.enable_motor_temperature', '0')) != 0 and
                int(float(items.get(b'servo.motor_thermistor_ohm', b'0.0'))) != 47000):
                print("Motor thermistor values other than 47000 ohm not supported in firmware <= 0x0108, disabling")
                items[b'servo.enable_motor_temperature'] = b'0'

            items.pop(b'servo.motor_thermistor_ohm')

            # Aux PWM out is not supported on <= 0x0108.
            items.pop(b'aux1.pwm_period_us')
            items.pop(b'aux2.pwm_period_us')

            def make_key(mpsource, index):
                return f'motor_position.sources.{mpsource}.compensation_table.{index}'.encode('utf8')

            if make_key(0, 0) in items:
                # Downsample encoder compensation bins.
                print("Downsampling encoder compensation tables from version >= 0x0109")
                for mpsource in range(0, 3):
                    bins = []

                    scale = float(items.pop(f'motor_position.sources.{mpsource}.compensation_scale'.encode('utf8'), 0.0)) / 127.0
                    new_size = 256
                    old_size = 32
                    ratio = new_size // old_size

                    for i in range(0, new_size):
                        key = make_key(mpsource, i)
                        bins.append(float(items.get(key)) * scale)
                        del items[key]

                    for i in range(0, old_size):
                        items[make_key(mpsource, i)] = str(bins[i*ratio]).encode('utf8')

        if self.new <= 0x0107 and self.old >= 0x0108:
            if float(items.get(b'servo.bemf_feedforward', '0')) == 0.0:
                print("Reverting servo.bemf_feedforward to 1.0")
                items[b'servo.bemf_feedforward'] = b'1.0'

        if self.new <= 0x0106 and self.old >= 0x0107:
            # motor_position.output.sign was broken in older versions.
            if int(items[b'motor_position.output.sign']) != 1:
                print("WARNING: motor_position.output.sign==-1 is broken in order versions, disabling")
                items[b'motor_position.output.sign'] = b'1'
            pass

        if self.new <= 0x0105 and self.old >= 0x0106:
            # Downgrade the I2C polling rate.
            for aux in [1, 2]:
                for i2c_device in [0, 1, 2]:
                    poll_rate_us_key = f'aux{aux}.i2c.devices.{i2c_device}.poll_rate_us'.encode('utf8')
                    poll_ms_key = f'aux{aux}.i2c.devices.{i2c_device}.poll_ms'.encode('utf8')

                    poll_rate_us = items[poll_rate_us_key]
                    items.pop(poll_rate_us_key)
                    items[poll_ms_key] = str(max(1, int(poll_rate_us) // 1000)).encode('utf8')
            print("Reverting I2C rates for version 0x0105")

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

        if self.new >= 0x0106 and self.old <= 0x0105:
            # We gave I2C encoders more timing resolution.
            for aux in [1, 2]:
                for i2c_device in [0, 1, 2]:
                    poll_rate_us_key = f'aux{aux}.i2c.devices.{i2c_device}.poll_rate_us'.encode('utf8')
                    poll_ms_key = f'aux{aux}.i2c.devices.{i2c_device}.poll_ms'.encode('utf8')

                    if poll_rate_us_key not in items:
                        continue

                    poll_ms = items[poll_ms_key]
                    items.pop(poll_ms_key)
                    items[poll_rate_us_key] = str(int(poll_ms) * 1000).encode('utf8')
            print("Upgrading I2C rates for version 0x0106")

        if self.new >= 0x0107 and self.old <= 0x0106:
            if int(items.get(b'motor_position.output.sign', 1)) == -1:
                print("Upgrading from motor_position.output.sign == -1, " +
                      "this was broken before, behavior will change")

            # No actual configuration updating is required here.
            pass

        if self.new >= 0x0108 and self.old <= 0x0107:
            if float(items.get(b'servo.bemf_feedforward', 1.0)) == 1.0:
                print("Upgrading servo.bemf_feedforward to 0.0")
                items[b'servo.bemf_feedforward'] = b'0.0'

        if self.new >= 0x0109 and self.old <= 0x0108:
            # Try to fix up the motor commutation offset tables.
            old_offsets = []
            i = 0
            while True:
                key = f'motor.offset.{i}'.encode('utf8')
                if key not in items:
                    break
                old_offsets.append(float(items.get(key)))
                i += 1

            offsets = old_offsets[:]

            # Unwrap this, then re-center the whole thing around 0.
            for i in range(1, len(offsets)):
                offsets[i] = (offsets[i - 1] +
                              _wrap_neg_pi_to_pi(offsets[i] -
                                                 offsets[i - 1]))
            mean_offset = sum(offsets) / len(offsets)
            delta = mean_offset - _wrap_neg_pi_to_pi(mean_offset)
            offsets = [x - delta for x in offsets]

            if any([abs(a - b) > 0.01
                    for a, b in zip(offsets, old_offsets)]):
                print("Re-wrapping motor commutation offsets")
                for i in range(len(offsets)):
                    key = f'motor.offset.{i}'.encode('utf8')
                    items[key] = f'{offsets[i]}'.encode('utf8')

        if self.new >= 0x0109 and self.old <= 0x0108:
            # If we had a motor thermistor enabled in previous
            # versions, it was with a value of 47000.
            if int(items.get(b'servo.enable_motor_temperature', b'0')) != 0:
                print("Thermistor from <= 0x0109 assumed to be 47000")
                items[b'servo.motor_thermistor_ohm'] = b'47000'

            def make_key(mpsource, index):
                return f'motor_position.sources.{mpsource}.compensation_table.{index}'.encode('utf8')

            new_size = 256
            old_size = 32
            ratio = new_size // old_size

            if make_key(0, 0) in items:
                print("Upsampling encoder compensation tables for version >= 0x0109")
                for mpsource in range(0, 3):
                    old_bins = [float(items.get(make_key(mpsource, i)).decode('latin1')) for i in range(0, 32)]
                    scale = max([abs(x) for x in old_bins])
                    bins = []
                    for i in range(new_size):
                        if scale != 0.0:
                            old_i = i // ratio
                            value = lerp(old_bins, i / new_size)
                            int_value = int(127 * value / scale)
                        else:
                            int_value = 0
                        items[make_key(mpsource, i)] = str(int_value).encode('utf8')

                    items[f'motor_position.sources.{mpsource}.compensation_scale'.encode('utf8')] = str(scale).encode('utf8')

        if self.new >= 0x010a and self.old <= 0x0109:
            v_per_hz = float(items.pop(b'motor.v_per_hz'))

            kv = 0 if v_per_hz == 0 else V_PER_HZ_FUDGE_010a * 0.5 * 60 / v_per_hz
            items[b'motor.Kv'] = str(kv).encode('utf8')

            print(f"Upgraded motor.v_per_hz to new motor.Kv and fixed fudge: old v_per_hz={v_per_hz} new Kv={kv}")

            if b'servo.max_power_W' in items:
                # If we had a power setting that was not the board
                # family default, then try to keep it going forward.
                # Otherwise switch it to NaN.
                board_default = {
                    0 : 450.0,
                    1 : 450.0,
                    2 : 100.0,
                }[self.board_family or 0]

                old_max_power = float(items[b'servo.max_power_W'])
                if old_max_power == board_default:
                    items[b'servo.max_power_W'] = b'nan'
                else:
                    pwm_rate = float(items.get(b'servo.pwm_rate_hz', 40000))
                    # The old value was set for 40kHz rate but the new
                    # value is absolute.  Scale it appropriately.
                    items[b'servo.max_power_W'] = str(old_max_power * (pwm_rate / 40000)).encode('utf8')
                print(f"Upgraded servo.max_power_W to {items[b'servo.max_power_W'].decode('utf8')} for 0x010a")

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


def _average(x):
    return sum(x) / len(x)


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

    async def do_restore_config(self, config_file):
        errors = []

        with open(config_file, "r") as fp:
            for line in fp.readlines():
                if '#' in line:
                    line = line[0:line.index('#')]
                line = line.rstrip()
                if len(line) == 0:
                    continue

                try:
                    await self.command(f'conf set {line}'.encode('latin1'))
                except moteus.CommandError as ce:
                    errors.append(line)

        await self.command(b'conf write')

        if len(errors):
            print("\nSome config could not be set:")
            for line in errors:
                print(f" {line}")
            print()

    async def do_write_config(self, config_file):
        fp = open(config_file, "rb")
        await self.write_config_stream(fp)

    async def write_config_stream(self, fp):
        errors = []

        config_lines = fp.readlines()

        for i, line in enumerate(config_lines):
            if i % 20 == 0:
                print(f"Writing config {100*i/len(config_lines):.0f}%  ",
                      end='\r', flush=True)

            line = line.rstrip()
            if len(line) == 0:
                continue

            try:
                await self.command(line)
            except moteus.CommandError as ce:
                errors.append(line.decode('latin1'))

        print()

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
            elf.firmware_version,
            None if old_firmware is None else old_firmware.family
        )

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

    async def clear_motor_offsets(self):
        i = 0
        while True:
            try:
                await self.command(f"conf set motor.offset.{i} 0")
            except moteus.CommandError as ce:
                if 'error setting' in ce.message:
                    # This means we hit the end of the offsets.
                    break
                else:
                    raise
            i += 1

    async def do_calibrate(self):
        self.firmware = await self.read_data("firmware")

        old_config = None
        if self.args.cal_no_update:
            print("Capturing old config for --cal-no-update")
            old_config = await self.command("conf enumerate")

        if self.firmware.version > SUPPORTED_ABI_VERSION:
            raise RuntimeError(f"\nmoteus_tool needs to be upgraded to support this firmware\n\n (likely python -m pip install --upgrade moteus')\n\nThe existing board has firmware 0x{self.firmware.version:04x} but this moteus_tool only supports up to 0x{SUPPORTED_ABI_VERSION:04x}")

        # Verify that commutation is from source 0.  It wouldn't be
        # too hard to support other sources, but for now this is
        # easier than doing so, and there is no real reason any end
        # user can't swap things around to get the commutation source
        # on slot 0.
        if await self.is_config_supported("motor_position.commutation_source"):
            commutation_source = await self.read_config_int("motor_position.commutation_source")
            if commutation_source != 0:
                raise RuntimeError("Automatic calibration only supported with commutation source of 0")

        # Determine what our calibration parameters are.
        self.calculate_calibration_parameters()

        print("This will move the motor, ensure it can spin freely!")
        await asyncio.sleep(2.0)

        # Force all existing offsets to 0, that way if we had a
        # discontinuous offset error, sending the stop will be able to
        # clear it (and we are about to overwrite them anyway).
        await self.clear_motor_offsets()

        # Clear any faults that may be there.
        await self.command("d stop")

        if await self.is_config_supported("motor.unwrapped_position_scale"):
            unwrapped_position_scale = \
                await self.read_config_double("motor.unwrapped_position_scale")
            motor_output_sign = 1.0
        elif await self.is_config_supported("motor_position.rotor_to_output_ratio"):
            unwrapped_position_scale = \
                await self.read_config_double("motor_position.rotor_to_output_ratio")
            motor_output_sign = \
                await self.read_config_double("motor_position.output.sign")


        if await self.is_config_supported("servo.pwm_rate_hz"):
            pwm_rate_hz = await self.read_config_double("servo.pwm_rate_hz")
            control_rate_hz = pwm_rate_hz if pwm_rate_hz <= 40000 else pwm_rate_hz / 2
        else:
            # Supported firmware versions that are not configurable
            # are all 40kHz.
            control_rate_hz = 40000

        # The rest of the calibration procedure assumes that
        # phase_invert is 0 and that the commutation encoder has a
        # positive sign.
        try:
            await self.command("conf set motor.phase_invert 0")
        except moteus.CommandError as e:
            # It is possible this firmware is too old to support
            # selecting the phase inversion.
            if not 'error setting' in e.message:
                raise
            pass

        try:
            await self.command("conf set motor_position.sources.0.sign 1")
        except moteus.CommandError as e:
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

        winding_resistance, highest_voltage, current_noise = (
            await self.calibrate_winding_resistance2(input_V))
        await self.check_for_fault()

        resistance_cal_voltage = highest_voltage

        # Determine our inductance.
        inductance = await self.calibrate_inductance(
            resistance_cal_voltage, input_V)
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
            torque_bw_hz, inductance,
            control_rate_hz=control_rate_hz)
        await self.check_for_fault()

        cal_result = await self.calibrate_encoder_mapping(
            input_V, winding_resistance, current_noise)
        await self.check_for_fault()

        motor_kv = await self.calibrate_kv_rating(
            input_V, unwrapped_position_scale, motor_output_sign)
        await self.check_for_fault()

        # Rezero the servo since we just spun it a lot.
        await self.command("d rezero")

        voltage_mode_control = False

        if await self.is_config_supported("servo.voltage_mode_control"):
            # See if we should be in voltage control mode or not.  The
            # heuristic is based the ratio of maximum possible phase
            # current given input voltage and phase resistance compared to
            # the phase noise.  Note that this is slightly different from
            # the heuristic used to switch to voltage mode calibration.
            # There we only look at the current that would be used for
            # calibration, not the maximum possible current.  Thus our
            # threshold is a bit different.
            max_possible_current = (0.5 * input_V / winding_resistance)
            max_current_quality = max_possible_current / current_noise

            voltage_mode_control = max_current_quality < VOLTAGE_MODE_QUALITY_MIN
            if voltage_mode_control:
                print(f"Using voltage mode control: \n  max possible current ({max_possible_current:.1f}) / current noise ({current_noise:.3f}) = {max_current_quality:.1f} < {VOLTAGE_MODE_QUALITY_MIN}")

            await self.command(f"conf set servo.voltage_mode_control {1 if voltage_mode_control else 0}")


        device_info = await self.get_device_info()

        if not self.args.cal_no_update:
            print("Saving to persistent storage")
            await self.command("conf write")
        else:
            # Restore our baseline configuration.
            print("Restoring baseline configuration for --cal-no-update")

            await self.restore_config(old_config)

        print("Calibration complete")

        try:
            now = datetime.datetime.now(datetime.timezone.utc)
        except:
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
            'kv' : motor_kv,
            'unwrapped_position_scale' : unwrapped_position_scale,
            'motor_position_output_sign' : motor_output_sign,
            'abi_version' : self.firmware.version,
            'voltage_mode_control' : voltage_mode_control,
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
            return self.args.cal_ll_encoder_voltage,

        # We're going to try and select a voltage to roughly achieve
        # "--cal-motor-power".
        return min(0.4 * input_V,
                   math.sqrt((self.args.cal_motor_power / 1.5) *
                             winding_resistance))

    async def calibrate_encoder_mapping(self, input_V, winding_resistance, current_noise):
        # Figure out what voltage to use for encoder calibration.
        encoder_cal_voltage = \
            await self.find_encoder_cal_voltage(input_V, winding_resistance)
        encoder_cal_current = encoder_cal_voltage / winding_resistance
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

        if self.args.cal_hall or hall_configured:
            if not hall_configured:
                raise RuntimeError("--cal-hall specified, but hall sensors " +
                                   "not configured on device")
            return await self.calibrate_encoder_mapping_hall(encoder_cal_voltage)
        else:
            old_output_sign = None
            old_voltage_mode_control = None
            try:
                if await self.is_config_supported("motor_position.output.sign"):
                    # Some later parts of our calibration procedure can
                    # handle a negative sign, but not the absolute encoder
                    # mapping when using the current mode.  Thus for now
                    # we just force it to 1 and set it back when complete.
                    old_output_sign = await self.read_config_int(
                        "motor_position.output.sign")
                    await self.command("conf set motor_position.output.sign 1")

                    old_voltage_mode_control = await self.read_config_int(
                        "servo.voltage_mode_control")

                return await self.calibrate_encoder_mapping_absolute(
                    encoder_cal_voltage, encoder_cal_current, current_noise)
            finally:
                # At least try to stop.
                await self.command("d stop")

                if old_output_sign is not None:
                    await self.command(
                        f"conf set motor_position.output.sign {old_output_sign}")

                    await self.command(
                        f"conf set servo.voltage_mode_control {old_voltage_mode_control}")


    async def calibrate_encoder_mapping_hall(self, encoder_cal_voltage):
        if self.args.cal_motor_poles is None:
            raise RuntimeError(
                'hall effect calibration requires specifying --cal-motor-poles')

        if (self.args.cal_motor_poles % 2) == 1:
            raise RuntimeError(
                'only motors with even numbers of poles are supported')

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

        if self.args.cal_write_raw:
            with open(self.args.cal_write_raw, "wb") as f:
                f.write(json.dumps(hall_cal_data, indent=2).encode('utf8'))

        await self.command("d stop")

        # See if we support phase_invert.
        allow_phase_invert = \
            await self.is_config_supported("motor.phase_invert")

        cal_result = ce.calibrate_hall(
            hall_cal_data,
            desired_direction=1 if not self.args.cal_invert else -1,
            allow_phase_invert=allow_phase_invert)

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
        # We might need to have some sense of pole count first.
        if await self.read_config_double("motor.poles") == 0:
            # Pick something arbitrary for now.
            await self.command(f"conf set motor.poles 2")

        try:
            motor_position = await self.read_data("motor_position")
        except RuntimeError:
            # Odds are we don't support motor_position, in which case
            # theta is always valid for the older versions that don't
            # support it.
            return

        if motor_position.error != 0:
            raise RuntimeError(
                f"encoder error: {repr(motor_position.error)}")

        if motor_position.homed == 0 and not motor_position.theta_valid:
            # We need to find an index.
            await self.find_index(encoder_cal_voltage)

    async def calibrate_encoder_mapping_absolute(
            self, encoder_cal_voltage, encoder_cal_current, current_noise):
        await self.ensure_valid_theta(encoder_cal_voltage)

        current_quality_factor = encoder_cal_current / current_noise
        use_current_for_quality = (
            current_quality_factor > CURRENT_QUALITY_MIN or
            self.args.cal_force_encoder_current_mode)
        use_current_for_firmware_version = self.firmware.version >= 0x010a

        use_current_calibration = (
            use_current_for_quality and use_current_for_firmware_version)

        if use_current_for_firmware_version and not use_current_for_quality:
            print(f"Using voltage mode calibration, current quality factor {current_quality_factor:.1f} < {CURRENT_QUALITY_MIN:.1f}")


        old_motor_poles = None

        if use_current_calibration:
            old_motor_poles = await self.read_config_int("motor.poles")
            await self.command("conf set motor.poles 2")
            await self.command(f"d pos nan 0 nan c{encoder_cal_current} b1")
            await asyncio.sleep(3.0)

            await self.write_message(
                (f"d cali i{encoder_cal_current} s{self.args.cal_ll_encoder_speed}"))
        else:
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
            if line.startswith(b'CAL done') or line.startswith(b'CALI done'):
                break
            if line.startswith(b'CAL start') or line.startswith(b'CALI start'):
                continue
            if line.startswith(b'ERR'):
                raise RuntimeError(f'Error calibrating: {line}')
            if line.startswith(b'CAL'):
                # Some problem
                raise RuntimeError(f'Error calibrating: {line}')

        if self.args.cal_write_raw:
            with open(self.args.cal_write_raw, "wb") as f:
                f.write(cal_data)

        cal_file = ce.parse_file(io.BytesIO(cal_data))

        # See if we support phase_invert.
        allow_phase_invert = \
            await self.is_config_supported("motor.phase_invert")

        cal_result = ce.calibrate(
            cal_file,
            desired_direction=1 if not self.args.cal_invert else -1,
            max_remainder_error=self.args.cal_max_remainder,
            allow_phase_invert=allow_phase_invert,
            allow_optimize=not self.args.cal_disable_optimize,
            force_optimize=self.args.cal_force_optimize,
        )

        if cal_result.errors:
            raise RuntimeError(f"Error(s) calibrating: {cal_result.errors}")

        if self.args.cal_motor_poles is not None:
            if self.args.cal_motor_poles != cal_result.poles:
                raise RuntimeError(
                    f"Auto-detected pole count ({cal_result.poles}) != " +
                    f"cmdline specified ({self.args.cal_motor_poles})")

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

        cal_result.current_quality_factor = current_quality_factor

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
        data = [extract(await self.read_servo_stats()) for _ in range(20)]

        # Stop the current.
        await self.command("d stop");

        # Sleep a tiny bit before returning.
        await asyncio.sleep(0.05);

        current_A = sum(data) / len(data)
        noise_A = stddev(data)

        return current_A, noise_A

    async def find_current_and_print(self, voltage):
        result, noise = await self.find_current(voltage)
        print(f"{voltage:.3f}V - {result:.3f}A")
        return result

    async def calibrate_winding_resistance2(self, input_V):
        print("Calculating winding resistance")

        # Depending upon the switching rate, there will be a region
        # around 0 current where the measured resistance is much
        # higher than actual and highly non-linear.  We also want to
        # limit the maximum amount of power put into the motor per the
        # user's request.  So, to balance those requirements, we start
        # at very low voltages and geometrically increase until we
        # reach the desired user power.  After that point, we attempt
        # to select a region from the largest currents that is roughly
        # linear.

        cal_voltage = 0.01

        # This will be a list of:
        #  ( voltage,
        #    current,
        #    step_resistance,
        #    noise,
        #  )
        results = []

        while True:
            this_current, this_noise = await self.find_current(cal_voltage)

            this_resistance = None
            if len(results):
                this_resistance = ((cal_voltage - results[-1][0]) /
                                   (this_current - results[-1][1]))

            if not self.args.verbose:
                print(f"Tested {cal_voltage:6.3f}V resistance {this_resistance or 0:6.3f}ohms  noise={this_noise:5.3f}A  ",
                      end='\r', flush=True)
            else:
                print(f" V={cal_voltage} I={this_current} R={this_resistance}")

            results.append((cal_voltage, this_current, this_resistance, this_noise))

            power = this_current * cal_voltage * 1.5
            if (power > self.args.cal_motor_power or
                cal_voltage > (0.4 * input_V)):
                break

            cal_voltage *= 1.1

        print()

        # If we had infinite precision, the "most correct" answer
        # would be the very last step_resistance we measured.
        # However, current noise will mean it is useful to incorporate
        # a reading from a more distant point.  This is hard because
        # as we get closer to 0, the results will become highly
        # non-linear, corrupting the result.

        # What we'll do is take the very last result, and the last
        # result that is less than 70% of the current of the last
        # result.

        last_result = results[-1]

        less_than = [x for x in results if x[1] < 0.60 * last_result[1]][-1]


        resistance = ((last_result[0] - less_than[0]) /
                      (last_result[1] - less_than[1]))

        print(f"Resistance {resistance:.3f} ohms")

        await self.command(f"conf set motor.resistance_ohm {resistance}")

        return resistance, last_result[0], last_result[3]


    async def _test_inductance_period(self, cal_voltage, input_V, ind_period):
        ind_voltage = cal_voltage

        if self.firmware.version < 0x010a:
            await asyncio.wait_for(
                self.command(f"d ind {cal_voltage} {ind_period}"), 0.25)
        else:
            # Our device supports inductance measurement from a
            # voltage offset.
            offset = min(0.2 * input_V, cal_voltage)
            ind_voltage = min(0.15 * input_V, 0.80 * cal_voltage)
            await asyncio.wait_for(
                self.command(f"d ind {ind_voltage} {ind_period} o{offset}"), 0.25)


        start = time.time()
        await asyncio.sleep(1.0)

        if self.firmware.version < 0x010a:
            await self.command(f"d stop")
        else:
            # Hold the same position and fixed voltage.
            await self.command(f"d pos nan 0 nan o{offset} b1")

        end = time.time()
        data = await self.read_servo_stats()

        delta_time = end - start
        di_dt = data.meas_ind_integrator / delta_time

        if self.args.verbose:
            print(f" inductance period={ind_period} v={cal_voltage} di_dt={di_dt}")

        return di_dt, ind_voltage


    async def calibrate_inductance(self, cal_voltage, input_V):
        print("Calculating motor inductance")

        old_motor_poles = await self.read_config_int("motor.poles")
        if old_motor_poles == 0:
            await self.command("conf set motor.poles 2")

        # Sweep through a range of inductance measurement frequencies
        # until we have a peak or near peak.  Rationale:
        #
        # For low inductance/low resistance motors, we can't actually
        # test very low frequencies without generating overly large
        # currents.  Thus we start out at a high frequency and stop
        # when it looks like we have gotten near enough to the peak.
        # Similarly, for high resistance/low inductance motors, we can
        # only effectively measure the inductance at high frequencies,
        # otherwise the current will saturate at Vbus/resistance.  For
        # high resistance / high inductance motors, we need to get all
        # the way to low frequencies before the measured current is
        # actually large enough to detect.

        periods_to_test = [2, 3, 4, 6, 8, 10, 12, 16, 20, 24, 32]

        highest_di_dt = None
        highest_cal_voltage = None
        since_highest = None

        try:
            for period in periods_to_test:
                this_di_dt, this_ind_voltage = (
                    await self._test_inductance_period(
                        cal_voltage, input_V, period))

                if highest_di_dt is None or this_di_dt > highest_di_dt:
                    highest_di_dt = this_di_dt
                    highest_cal_voltage = this_ind_voltage
                    since_highest = 0
                else:
                    if since_highest is not None:
                        since_highest += 1

                if self.args.verbose:
                    print(f"inductance period {period} di_dt={this_di_dt}")

                if (highest_di_dt > 0 and
                    (this_di_dt < 0.5 * highest_di_dt or since_highest > 2)):
                    # We stop early to avoid causing excessive ripple
                    # current on low resistance / low inductance motors.
                    break
            await self.command("d stop")
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

        inductance = (highest_cal_voltage / highest_di_dt)

        if inductance < 1e-6:
            raise RuntimeError(f'Inductance too small ({inductance} < 1e-6)')

        print(f"Calculated inductance: {inductance}H")
        return inductance

    async def set_encoder_filter(self, torque_bw_hz, inductance, control_rate_hz = None):
        # Check to see if our firmware supports encoder filtering.
        motor_position_style = await self.is_config_supported("motor_position.sources.0.pll_filter_hz")
        servo_style = await self.is_config_supported("servo.encoder_filter.enabled")
        if not motor_position_style and not servo_style:
            return None, None, None

        if self.args.encoder_bw_hz:
            desired_encoder_bw_hz = self.args.encoder_bw_hz
        else:
            # Don't let the encoder bandwidth be less than 10Hz by default.
            desired_encoder_bw_hz = max(10, torque_bw_hz)

            # If we are using hall effects as the output encoder, then
            # limit the bandwidth based on inductance as a rough
            # heuristic.
            hall_output = False
            if await self.is_config_supported("motor_position.rotor_to_output_ratio"):
                output_encoder = await self.read_config_int("motor_position.output.source")
                output_type = await self.read_config_int(f"motor_position.sources.{output_encoder}.type")
                if output_type == 4:  # kHall
                    hall_output = True

            if inductance and hall_output:
                desired_encoder_bw_hz = min(
                    desired_encoder_bw_hz, 2e-2 / inductance)

            # Also, limit the bandwidth for halls based on the number
            # of poles and the estimated calibration speed.
            if hall_output:
                max_pole_bandwidth_hz = (
                    0.5 * self.args.cal_motor_poles *
                    self.args.cal_motor_speed)
                desired_encoder_bw_hz = min(
                    desired_encoder_bw_hz, max_pole_bandwidth_hz)


        # And our bandwidth with the filter can be no larger than
        # 1/30th the control rate.
        encoder_bw_hz = min(control_rate_hz / 30, desired_encoder_bw_hz)

        if encoder_bw_hz != desired_encoder_bw_hz:
            print(f"Warning: using lower encoder bandwidth than "+
                  f"requested: {encoder_bw_hz:.1f}Hz")

        w_3db = encoder_bw_hz * 2 * math.pi
        kp = 2 * w_3db
        ki = w_3db * w_3db

        if servo_style:
            await self.command(f"conf set servo.encoder_filter.enabled 1")
            await self.command(f"conf set servo.encoder_filter.kp {kp}")
            await self.command(f"conf set servo.encoder_filter.ki {ki}")
        elif motor_position_style:
            commutation_source = await self.read_config_int("motor_position.commutation_source")
            await self.command(f"conf set motor_position.sources.{commutation_source}.pll_filter_hz {encoder_bw_hz}")
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

        cal_bw_rad_s = self.args.cal_bw_hz * twopi

        w_3db = min(board_limit_rad_s,
                    cal_bw_rad_s)

        if w_3db != cal_bw_rad_s:
            print(f"Warning: using lower torque bandwidth " +
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

        SAMPLE_PERIOD_S = 0.02
        AVERAGE_PERIOD_S = 0.10

        AVERAGE_COUNT = int(AVERAGE_PERIOD_S / SAMPLE_PERIOD_S)

        def sign(x):
            return 1 if x >= 0 else -1

        velocity_samples = []
        power_samples = []

        while True:
            data = await self.read_servo_stats()

            total_current_A = math.hypot(data.d_A, data.q_A)
            total_power_W = voltage * total_current_A * 1.5

            power_samples.append(total_power_W)

            if len(power_samples) > AVERAGE_COUNT:
                del power_samples[0]

            velocity_samples.append(data.velocity)

            if len(velocity_samples) > (3 * AVERAGE_COUNT):
                del velocity_samples[0]

            recent_average = _average(velocity_samples[-AVERAGE_COUNT:])

            # As a fallback, timeout after a fixed amount of waiting.
            if (time.time() - start_time) > 2.0:
                return recent_average

            if len(power_samples) >= AVERAGE_COUNT:
                average_power_W = sum(power_samples) / len(power_samples)
                max_power_W = (self.args.cal_max_kv_power_factor *
                               self.args.cal_motor_power)

                # This is a safety.  During speed measurement, current
                # should always be near 0.  However, if the encoder
                # commutation calibration failed, we can sometimes
                # trigger large currents during the Kv detection phase
                # while not actually moving.
                if (abs(recent_average) < 0.2 and
                    average_power_W > max_power_W):
                    await self.command("d stop")

                    raise RuntimeError(
                        f"Motor failed to spin, {average_power_W} > " +
                        f"{max_power_W}")

            if (len(velocity_samples) >= AVERAGE_COUNT and
                abs(recent_average) < 0.2):
                return recent_average

            if len(velocity_samples) == 3 * AVERAGE_COUNT:
                average_1 = _average(velocity_samples[:AVERAGE_COUNT])
                average_2 = _average(velocity_samples[AVERAGE_COUNT:2*AVERAGE_COUNT])
                average_3 = recent_average

                if sign(average_3 - average_2) != sign(average_2 - average_1):
                    return recent_average

            await asyncio.sleep(SAMPLE_PERIOD_S)

        return velocity

    async def find_speed_and_print(self, voltage, **kwargs):
        result = await self.find_speed(voltage, **kwargs)
        print(f"{voltage:.3f}V - {result:.3f}Hz")
        return result

    async def find_kv_cal_voltage(self, input_V, unwrapped_position_scale):
        if self.args.cal_ll_kv_voltage:
            return self.args.cal_ll_kv_voltage

        first_nonzero_speed_voltage = None

        # Otherwise, we start small, and increase until we hit a
        # reasonable speed, but at least twice what it takes to get a
        # non-zero speed.
        maybe_result = 0.01
        while True:
            print(f"Testing {maybe_result:.3f}V for Kv",
                  end='\r', flush=True)
            if maybe_result > (0.2 * input_V):
                return maybe_result

            this_speed = await self.find_speed(maybe_result) / unwrapped_position_scale

            if (abs(this_speed) > (0.1 * self.args.cal_motor_speed) and
                first_nonzero_speed_voltage is None):
                first_nonzero_speed_voltage = maybe_result

            # Aim for this many Hz
            if (first_nonzero_speed_voltage is not None and
                maybe_result > (2 * first_nonzero_speed_voltage) and
                abs(this_speed) > self.args.cal_motor_speed):
                break

            maybe_result *= 1.1

        print()
        return maybe_result

    async def calibrate_kv_rating(self, input_V, unwrapped_position_scale,
                                  motor_output_sign):
        if self.args.cal_force_kv is None:
            print("Calculating Kv rating")

            await self.ensure_valid_theta(self.encoder_cal_voltage)

            original_position_min = await self.read_config_double("servopos.position_min")
            original_position_max = await self.read_config_double("servopos.position_max")

            await self.command("conf set servopos.position_min NaN")
            await self.command("conf set servopos.position_max NaN")
            await self.command("d index 0")

            kv_cal_voltage = await self.find_kv_cal_voltage(
                input_V, unwrapped_position_scale)
            await self.stop_and_idle()

            voltages = [x * kv_cal_voltage for x in [
                0.0, 0.25, 0.5, 0.75, 1.0 ]]
            voltage_speed_hzs = list(zip(
                voltages, [ await self.find_speed_and_print(voltage, sleep_time=2)
                            for voltage in voltages]))

            await self.stop_and_idle()

            await asyncio.sleep(0.5)

            await self.command(f"conf set servopos.position_min {original_position_min}")
            await self.command(f"conf set servopos.position_max {original_position_max}")

            # Drop any measurements that are too slow.  This will
            # include (hopefully) our initial zero voltage
            # measurement, but that lets us get a more accurate read
            # on the slope.
            speed_threshold = abs(0.45 * voltage_speed_hzs[-1][1])
            voltage_speed_hzs = [(v, s) for v, s in voltage_speed_hzs
                                 if abs(s) > speed_threshold]

            geared_v_per_hz = 1.0 / _calculate_slope(
                [x[0] for x in voltage_speed_hzs],
                [x[1] for x in voltage_speed_hzs])

            v_per_hz = (geared_v_per_hz *
                        unwrapped_position_scale)
            if self.firmware.version <= 0x0106:
                v_per_hz *= motor_output_sign

            print(f"v_per_hz (pre-gearbox)={v_per_hz}")

            if v_per_hz < 0.0:
                raise RuntimeError(
                    f"v_per_hz measured as negative ({v_per_hz}), something wrong")

            # Experimental verification of Kv using this protocol
            # typically results in a determination of Kv roughly 14%
            # below what an open circuit spin measures with an
            # oscilloscope.  That is probably due to friction in the
            # system and other non-linearities.
            FUDGE = 1.14
            motor_kv = FUDGE * 0.5 * 60 / v_per_hz
        else:
            motor_kv = self.args.cal_force_kv
            print(f"Using forced Kv: {self.args.cal_force_kv}")

        if self.firmware.version >= 0x010a:
            await self.command(f"conf set motor.Kv {motor_kv}")
        else:
            if self.firmware.family == 2:
                # moteus-c1 in older firmwares had additional
                # scaling.
                motor_kv /= 1.38

            v_per_hz = (V_PER_HZ_FUDGE_010a * 0.5 * 60 / motor_kv)
            await self.command(f"conf set motor.v_per_hz {v_per_hz}")

        if motor_kv < 0:
            raise RuntimeError(f'Kv value ({motor_kv}) is negative')

        return motor_kv

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
        if await self.is_config_supported("motor.v_per_hz"):
            await self.command(f"conf set motor.v_per_hz {report['v_per_hz']}")
        elif await self.is_config_supported("motor.Kv"):
            await self.command(f"conf set motor.Kv {report['kv']}")

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
        self.transport = None

        # Was our target list found through discovery?
        self._discovered = False

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        if self.transport and hasattr(self.transport, 'close'):
            self.transport.close()

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
        elif self.args.restore_config:
            await stream.do_restore_config(self.args.restore_config)
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
    group.add_argument('--restore-config', metavar='FILE',
                       help='restore a config saved with --dump-config')
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
                        default=7.5,
                        help='motor power in W to use for encoder cal')
    parser.add_argument('--cal-motor-speed', metavar='Hz', type=float,
                        default=12.0,
                        help='max motor mechanical speed to use for kv cal')

    parser.add_argument('--cal-motor-poles', metavar='N', type=int,
                        default=None,
                        help='number of motor poles (2x pole pairs)')
    parser.add_argument('--cal-force-kv', metavar='Kv', type=float,
                        default=None,
                        help='do not calibrate Kv, but use the specified value')
    parser.add_argument('--cal-force-optimize', action='store_true',
                        help='require nonlinear commutation optimization')
    parser.add_argument('--cal-disable-optimize', action='store_true',
                        help='prevent nonlinear commutation optimization')


    parser.add_argument('--cal-max-remainder', metavar='F',
                        type=float, default=0.1,
                        help='maximum allowed error in calibration')
    parser.add_argument('--cal-max-kv-power-factor', type=float,
                        default=1.25)
    parser.add_argument('--cal-write-raw', metavar='FILE', type=str,
                        help='write raw calibration data')
    parser.add_argument('--cal-force-encoder-current-mode', action='store_true',
                        help='always use encoder current mode calibration if supported')

    args = parser.parse_args()

    with Runner(args) as runner:
        await runner.start()


def main():
    asyncio.run(async_main())


if __name__ == '__main__':
    main()
