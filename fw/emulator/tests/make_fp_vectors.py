#!/usr/bin/python3

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

"""Generates fp_vectors.txt, the expected results of single precision
floating point instructions, from the host's IEEE-754 arithmetic.

Requires numpy and arm-none-eabi-as.

  ./make_fp_vectors.py > fp_vectors.txt
"""

import random
import re
import struct
import subprocess
import tempfile

import numpy as np

f32 = np.float32

VALUES = [f32(x) for x in [
    0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 1.5, -1.5, 2.5, -2.5, 3.75, -3.75,
    16384.0, -16384.0, 24576.0, 8191.7, -8191.7, 1e-3, 6.1035156e-05,
    1e10, -1e10, 3.0e9, -3.0e9, 2147483648.0, -2147483904.0,
    4294967296.0, 0.99999994, 123456.789,
    float('inf'), float('-inf'), float('nan')]]

INTEGERS = [0, 1, -1, 5, -5, 16384, 2147483647, -2147483648,
            16777217, -16777217, 123456789]


def bits(x):
    return struct.unpack('<I', struct.pack('<f', float(x)))[0]


def fma(d, n, m):
    # The product of two floats is exact in double precision.
    return f32(np.float64(d) + np.float64(n) * np.float64(m))


def to_s32(x):
    if np.isnan(x):
        return 0
    x = float(x)
    if x >= 2147483647.0:
        return 0x7fffffff
    if x <= -2147483648.0:
        return 0x80000000
    return int(x) & 0xffffffff


def to_u32(x):
    if np.isnan(x):
        return 0
    x = float(x)
    if x >= 4294967295.0:
        return 0xffffffff
    if x <= 0:
        return 0
    return int(x)


def to_fixed(x, fraction_bits, signed):
    '''Convert to fixed point, truncating and saturating.'''
    if np.isnan(x):
        return 0
    low, high = (-2**31, 2**31 - 1) if signed else (0, 2**32 - 1)
    if np.isinf(x):
        return (high if x > 0 else low) & 0xffffffff
    # Scaling by a power of two is exact in double precision.
    scaled = int(np.float64(x) * 2.0**fraction_bits)
    return max(low, min(high, scaled)) & 0xffffffff


def from_fixed(i, fraction_bits):
    return f32(np.float64(i) / 2.0**fraction_bits)


def compare(a, b):
    '''Returns the resulting NZCV flags.'''
    if np.isnan(a) or np.isnan(b):
        return 0b0011
    if a == b:
        return 0b0110
    if a < b:
        return 0b1000
    return 0b0010


# Each is (assembly, kind, function of (s0, s1, s2)).
BINARY = {
    'vadd.f32': lambda a, b: a + b,
    'vsub.f32': lambda a, b: a - b,
    'vmul.f32': lambda a, b: a * b,
    'vdiv.f32': lambda a, b: a / b,
    'vnmul.f32': lambda a, b: -(a * b),
}

UNARY = {
    'vneg.f32': lambda a: -a,
    'vabs.f32': lambda a: np.abs(a),
    'vsqrt.f32': lambda a: np.sqrt(a),
}

# Here s0 and s1 are multiplied and s2 is the accumulator.
ACCUMULATE = {
    'vfma.f32': lambda n, m, d: fma(d, n, m),
    'vfms.f32': lambda n, m, d: fma(d, -n, m),
    'vfnms.f32': lambda n, m, d: fma(-d, n, m),
    'vfnma.f32': lambda n, m, d: fma(-d, -n, m),
}


def encode(instructions):
    '''Returns the two halfwords of each of a list of instructions.'''
    with tempfile.TemporaryDirectory() as tmp:
        source = tmp + '/enc.s'
        obj = tmp + '/enc.o'
        with open(source, 'w') as f:
            f.write('.syntax unified\n.cpu cortex-m4\n'
                    '.fpu fpv4-sp-d16\n.thumb\n')
            f.write(''.join(x + '\n' for x in instructions))
        subprocess.check_call(['arm-none-eabi-as', '-o', obj, source])
        listing = subprocess.check_output(
            ['arm-none-eabi-objdump', '-d', obj]).decode()
    result = re.findall(
        r'^\s+[0-9a-f]+:\s+([0-9a-f]{4}) ([0-9a-f]{4})\s', listing, re.M)
    assert len(result) == len(instructions)
    return result


def main():
    np.seterr(all='ignore')
    random.seed(1)

    pairs = [(a, b) for a in VALUES for b in VALUES]
    triples = [(a, b, c)
               for a in VALUES[:24] for b in VALUES[:24] for c in VALUES[:24]]
    random.shuffle(triples)
    sampled_pairs = list(pairs)
    random.shuffle(sampled_pairs)

    # Each case is (name, assembly, kind, inputs, expected)
    cases = []
    for name, fn in BINARY.items():
        exhaustive = name in ['vadd.f32', 'vsub.f32']
        for a, b in (pairs if exhaustive else sampled_pairs[:120]):
            cases.append((name, f'{name} s2, s0, s1', 'value',
                          [bits(a), bits(b), 0], bits(fn(a, b))))
    for name, fn in UNARY.items():
        for a in VALUES:
            cases.append((name, f'{name} s2, s0', 'value',
                          [bits(a), 0, 0], bits(fn(a))))
    for name, fn in ACCUMULATE.items():
        for n, m, d in triples[:150]:
            cases.append((name, f'{name} s2, s0, s1', 'value',
                          [bits(n), bits(m), bits(d)], bits(fn(n, m, d))))
    for a in VALUES:
        cases.append(('vcvt.s32.f32', 'vcvt.s32.f32 s2, s0', 'value',
                      [bits(a), 0, 0], to_s32(a)))
        cases.append(('vcvt.u32.f32', 'vcvt.u32.f32 s2, s0', 'value',
                      [bits(a), 0, 0], to_u32(a)))
        cases.append(('vcmpe0', 'vcmpe.f32 s0, #0', 'flags',
                      [bits(a), 0, 0], compare(a, f32(0))))
        cases.append(('vcmp0', 'vcmp.f32 s0, #0', 'flags',
                      [bits(a), 0, 0], compare(a, f32(0))))
    # The fixed point conversions operate in place.
    for fraction_bits in [1, 8, 16, 24, 31, 32]:
        for a in VALUES + [f32(x) for x in [
                0.3, -0.3, 0.999, 127.99, -127.99, 6.0e-8, 255.5, 65535.7]]:
            for kind, signed in [('s32', True), ('u32', False)]:
                cases.append((
                    f'vcvt.{kind}.f32.fixed',
                    f'vcvt.{kind}.f32 s2, s2, #{fraction_bits}', 'value',
                    [0, 0, bits(a)], to_fixed(a, fraction_bits, signed)))
        for i in INTEGERS:
            u = i & 0xffffffff
            cases.append((
                'vcvt.f32.s32.fixed',
                f'vcvt.f32.s32 s2, s2, #{fraction_bits}', 'value',
                [0, 0, u], bits(from_fixed(i, fraction_bits))))
            cases.append((
                'vcvt.f32.u32.fixed',
                f'vcvt.f32.u32 s2, s2, #{fraction_bits}', 'value',
                [0, 0, u], bits(from_fixed(u, fraction_bits))))
    for a, b in pairs:
        for name in ['vcmp', 'vcmpe']:
            cases.append((name, f'{name}.f32 s0, s1', 'flags',
                          [bits(a), bits(b), 0], compare(a, b)))
    for i in INTEGERS:
        u = i & 0xffffffff
        cases.append(('vcvt.f32.s32', 'vcvt.f32.s32 s2, s0', 'value',
                      [u, 0, 0], bits(f32(np.int32(i)))))
        cases.append(('vcvt.f32.u32', 'vcvt.f32.u32 s2, s0', 'value',
                      [u, 0, 0], bits(f32(np.uint32(u)))))

    assembly = sorted(set(x[1] for x in cases))
    encodings = dict(zip(assembly, encode(assembly)))

    print('# Generated by make_fp_vectors.py from host IEEE-754 arithmetic.')
    print('#')
    print('# name hw1 hw2 kind s0 s1 s2 expected')
    print('#')
    print("# For kind 'value', expected is s2 after the instruction.  For")
    print("# 'flags', it is FPSCR[31:28].")
    for name, asm, kind, inputs, expected in cases:
        hw1, hw2 = encodings[asm]
        print(f'{name} {hw1} {hw2} {kind} '
              f'{inputs[0]:08x} {inputs[1]:08x} {inputs[2]:08x} '
              f'{expected:08x}')


if __name__ == '__main__':
    main()
