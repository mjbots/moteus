#!/usr/bin/python3 -B

# Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

# Helpers for the moteus firmware version constants.
#
# The on-wire firmware version (register 0x101) is a 32-bit integer
# encoded as 0x00MMmmpp where MM/mm/pp are the major/minor/patch bytes.
# This module converts between human-readable semver strings and that
# packed encoding, and provides an in-place rewrite of the constants
# in fw/moteus_hw.h and lib/python/moteus/moteus_tool.py.

import argparse
import pathlib
import re
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent


def parse_version(version):
    core = version.split('-', 1)[0].split('+', 1)[0]
    parts = core.split('.')
    if len(parts) != 3:
        raise ValueError(f'expected MAJOR.MINOR.PATCH, got {version!r}')
    major, minor, patch = (int(p) for p in parts)
    for name, value in zip(('major', 'minor', 'patch'), (major, minor, patch)):
        if not (0 <= value <= 255):
            raise ValueError(f'{name} {value} does not fit in one byte')
    return major, minor, patch


def encode(version):
    major, minor, patch = parse_version(version)
    return (major << 16) | (minor << 8) | patch


def decode(packed):
    return f'{(packed >> 16) & 0xff}.{(packed >> 8) & 0xff}.{packed & 0xff}'


def _rewrite(path, pattern, replacement):
    text = path.read_text()
    updated = re.sub(pattern, replacement, text, count=1, flags=re.MULTILINE)
    if updated == text:
        return False
    path.write_text(updated)
    return True


def update_firmware_constant(version):
    return _rewrite(
        REPO_ROOT / 'fw' / 'moteus_hw.h',
        r'^#define MOTEUS_FIRMWARE_VERSION 0x[0-9a-fA-F]+$',
        f'#define MOTEUS_FIRMWARE_VERSION 0x{encode(version):06x}',
    )


def update_supported_abi(version):
    return _rewrite(
        REPO_ROOT / 'lib' / 'python' / 'moteus' / 'moteus_tool.py',
        r'^SUPPORTED_ABI_VERSION = 0x[0-9a-fA-F]+$',
        f'SUPPORTED_ABI_VERSION = 0x{encode(version):06x}',
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('encode', help='print the packed integer for a version')
    p.add_argument('version')

    p = sub.add_parser('decode', help='print the version string for a packed integer')
    p.add_argument('packed', help='hex (0x...) or decimal')

    p = sub.add_parser('update', help='rewrite version constants in place')
    p.add_argument('--firmware', metavar='VERSION',
                   help='update MOTEUS_FIRMWARE_VERSION in fw/moteus_hw.h')
    p.add_argument('--supported-abi', metavar='VERSION',
                   help='update SUPPORTED_ABI_VERSION in moteus_tool.py')

    args = parser.parse_args()
    if args.cmd == 'encode':
        print(f'0x{encode(args.version):06x}')
    elif args.cmd == 'decode':
        print(decode(int(args.packed, 0)))
    elif args.cmd == 'update':
        if not (args.firmware or args.supported_abi):
            sys.exit('need at least one of --firmware/--supported-abi')
        if args.firmware:
            changed = update_firmware_constant(args.firmware)
            print(f'fw/moteus_hw.h: {"updated" if changed else "unchanged"}')
        if args.supported_abi:
            changed = update_supported_abi(args.supported_abi)
            print(f'lib/python/moteus/moteus_tool.py: {"updated" if changed else "unchanged"}')


if __name__ == '__main__':
    main()
