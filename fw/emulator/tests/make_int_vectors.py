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

"""Generates int_vectors.txt, the expected results of integer
instructions, from an independent model of their documented behavior
in the ARMv7-M Architecture Reference Manual.

Requires arm-none-eabi-as.

  ./make_int_vectors.py > int_vectors.txt
"""

import random
import re
import subprocess
import tempfile

M = 0xffffffff

N, Z, C, V, Q = 16, 8, 4, 2, 1

EDGES = [0, 1, 2, 0x7f, 0x80, 0xff, 0x100, 0x7fff, 0x8000, 0xffff,
         0x10000, 0x7fffffff, 0x80000000, 0x80000001, 0xfffffffe,
         0xffffffff, 0x12345678, 0x87654321, 0x00ff00ff, 0xaaaaaaaa]


def signed(x):
    return x - (1 << 32) if x & 0x80000000 else x


def nz(result):
    return (N if result & 0x80000000 else 0) | (Z if result == 0 else 0)


def add_with_carry(x, y, carry):
    unsigned_sum = x + y + carry
    signed_sum = signed(x) + signed(y) + carry
    result = unsigned_sum & M
    flags = nz(result)
    if unsigned_sum != result:
        flags |= C
    if signed(result) != signed_sum:
        flags |= V
    return result, flags


def shift_c(value, kind, amount, carry):
    '''Returns (result, carry out).'''
    if kind == 'rrx':
        return ((carry << 31) | (value >> 1)), value & 1
    if amount == 0:
        return value, carry
    if kind == 'lsl':
        if amount > 32:
            return 0, 0
        extended = value << amount
        return extended & M, (extended >> 32) & 1
    if kind == 'lsr':
        if amount > 32:
            return 0, 0
        return (value >> amount) & M, (value >> (amount - 1)) & 1
    if kind == 'asr':
        amount = min(amount, 32)
        return (signed(value) >> amount) & M, \
            (signed(value) >> (amount - 1)) & 1
    if kind == 'ror':
        amount %= 32
        result = ((value >> amount) | (value << (32 - amount))) & M
        return result, result >> 31
    raise RuntimeError(kind)


def expand_imm_carry(const, carry):
    '''The carry out of ThumbExpandImm_C for a given constant.'''
    low = const & 0xff
    high = (const >> 8) & 0xff
    if const in [low, low | (low << 16), (high << 8) | (high << 24),
                 low | (low << 8) | (low << 16) | (low << 24)]:
        # These forms are not rotated, and leave the carry alone.
        return carry
    return const >> 31


class Case:
    def __init__(self, name, asm, inputs, flags_in, r2, r3, flags_out):
        self.name = name
        self.asm = asm
        self.inputs = inputs
        self.flags_in = flags_in
        self.r2 = r2 & M
        self.r3 = r3 & M
        self.flags_out = flags_out


cases = []


def operands(count):
    result = [(a, b) for a in EDGES[:12] for b in EDGES[:12]]
    random.shuffle(result)
    result = result[:count // 2]
    while len(result) < count:
        result.append((random.getrandbits(32), random.getrandbits(32)))
    return result


def emit(name, asm, fn, count=24):
    '''fn takes (r0, r1, r2, r3, carry in) and returns (r2, r3, flags)
    where flags is None if they are unaffected, or else a tuple of
    (new flags, mask of those affected).'''
    for a, b in operands(count):
        r2 = random.choice(EDGES + [random.getrandbits(32)])
        r3 = random.getrandbits(32)
        flags_in = random.choice([0, C, N | V, Z | C, N | Z | C | V])
        carry = 1 if flags_in & C else 0
        new_r2, new_r3, flags = fn(a, b, r2, r3, carry)
        if flags is None:
            flags_out = flags_in
        else:
            value, mask = flags
            flags_out = (flags_in & ~mask) | (value & mask)
        cases.append(Case(name, asm, [a, b, r2, r3], flags_in,
                          new_r2, new_r3, flags_out))


NZCV = N | Z | C | V


def arithmetic(op):
    def fn(x, y, carry):
        if op in ['add', 'cmn']:
            return add_with_carry(x, y, 0)
        if op == 'adc':
            return add_with_carry(x, y, carry)
        if op in ['sub', 'cmp']:
            return add_with_carry(x, ~y & M, 1)
        if op == 'sbc':
            return add_with_carry(x, ~y & M, carry)
        if op == 'rsb':
            return add_with_carry(~x & M, y, 1)
        raise RuntimeError(op)
    return fn


def logical(op):
    return {
        'and': lambda x, y: x & y,
        'tst': lambda x, y: x & y,
        'orr': lambda x, y: x | y,
        'eor': lambda x, y: x ^ y,
        'teq': lambda x, y: x ^ y,
        'bic': lambda x, y: x & ~y & M,
        'orn': lambda x, y: (x | ~y) & M,
    }[op]


COMPARES = ['cmp', 'cmn', 'tst', 'teq']
SHIFTS = [('lsl', 1), ('lsl', 13), ('lsl', 31), ('lsr', 1), ('lsr', 17),
          ('lsr', 32), ('asr', 1), ('asr', 9), ('asr', 32), ('ror', 1),
          ('ror', 21), ('rrx', 0)]
CONSTANTS = [0, 1, 0xff, 0xab00ab, 0xcd00cd00, 0x55555555, 0xff00,
             0x3fc00, 0xff000000, 0x80000000, 0x1fe, 0x7f800000]


def shift_text(kind, amount):
    return ', rrx' if kind == 'rrx' else f', {kind} #{amount}'


def make_data_processing():
    for op in ['add', 'adc', 'sub', 'sbc', 'rsb', 'cmp', 'cmn']:
        fn = arithmetic(op)
        compare = op in COMPARES

        def reg(shift, set_flags, fn=fn, compare=compare):
            def result(a, b, r2, r3, carry):
                kind, amount = shift
                shifted, _ = shift_c(b, kind, amount, carry)
                value, flags = fn(a, shifted, carry)
                return (r2, r3 if compare else value,
                        (flags, NZCV) if set_flags else None)
            return result

        def imm(const, set_flags, fn=fn, compare=compare):
            def result(a, b, r2, r3, carry):
                value, flags = fn(a, const, carry)
                return (r2, r3 if compare else value,
                        (flags, NZCV) if set_flags else None)
            return result

        for shift in [('lsl', 0)] + SHIFTS:
            suffix = '' if shift == ('lsl', 0) else shift_text(*shift)
            if compare:
                emit(op, f'{op}.w r0, r1{suffix}', reg(shift, True), 10)
            else:
                emit(op + 's', f'{op}s.w r3, r0, r1{suffix}',
                     reg(shift, True), 10)
                emit(op, f'{op}.w r3, r0, r1{suffix}', reg(shift, False), 4)
        for const in CONSTANTS:
            if compare:
                emit(op, f'{op}.w r0, #{const}', imm(const, True), 8)
            else:
                emit(op + 's', f'{op}s.w r3, r0, #{const}',
                     imm(const, True), 8)
                emit(op, f'{op}.w r3, r0, #{const}', imm(const, False), 3)

    for op in ['and', 'orr', 'eor', 'bic', 'orn', 'tst', 'teq']:
        fn = logical(op)
        compare = op in COMPARES

        def reg(shift, set_flags, fn=fn, compare=compare):
            def result(a, b, r2, r3, carry):
                kind, amount = shift
                shifted, carry_out = shift_c(b, kind, amount, carry)
                value = fn(a, shifted)
                flags = nz(value) | (C if carry_out else 0)
                return (r2, r3 if compare else value,
                        (flags, N | Z | C) if set_flags else None)
            return result

        def imm(const, set_flags, fn=fn, compare=compare):
            def result(a, b, r2, r3, carry):
                value = fn(a, const)
                carry_out = expand_imm_carry(const, carry)
                flags = nz(value) | (C if carry_out else 0)
                return (r2, r3 if compare else value,
                        (flags, N | Z | C) if set_flags else None)
            return result

        for shift in [('lsl', 0)] + SHIFTS:
            suffix = '' if shift == ('lsl', 0) else shift_text(*shift)
            if compare:
                emit(op, f'{op}.w r0, r1{suffix}', reg(shift, True), 10)
            else:
                emit(op + 's', f'{op}s.w r3, r0, r1{suffix}',
                     reg(shift, True), 10)
                emit(op, f'{op}.w r3, r0, r1{suffix}', reg(shift, False), 4)
        for const in CONSTANTS:
            if compare:
                emit(op, f'{op}.w r0, #{const}', imm(const, True), 8)
            else:
                emit(op + 's', f'{op}s.w r3, r0, #{const}',
                     imm(const, True), 8)
                emit(op, f'{op}.w r3, r0, #{const}', imm(const, False), 3)

    for const in CONSTANTS:
        def mvn(a, b, r2, r3, carry, const=const):
            value = ~const & M
            carry_out = expand_imm_carry(const, carry)
            return r2, value, (nz(value) | (C if carry_out else 0), N | Z | C)
        emit('mvns', f'mvns.w r3, #{const}', mvn, 3)

    for const in [0, 1, 0xfff, 0x800, 0x123]:
        emit('addw', f'addw r3, r0, #{const}',
             lambda a, b, r2, r3, c, k=const: (r2, (a + k) & M, None), 6)
        emit('subw', f'subw r3, r0, #{const}',
             lambda a, b, r2, r3, c, k=const: (r2, (a - k) & M, None), 6)
    for const in [0, 0xffff, 0x1234, 0x8000]:
        emit('movw', f'movw r3, #{const}',
             lambda a, b, r2, r3, c, k=const: (r2, k, None), 2)
        emit('movt', f'movt r3, #{const}',
             lambda a, b, r2, r3, c, k=const:
             (r2, (r3 & 0xffff) | (k << 16), None), 3)


def make_shifts():
    for kind in ['lsl', 'lsr', 'asr', 'ror']:
        def by_register(set_flags, kind=kind):
            def result(a, b, r2, r3, carry):
                value, carry_out = shift_c(a, kind, b & 0xff, carry)
                flags = nz(value) | (C if carry_out else 0)
                return r2, value, (flags, N | Z | C) if set_flags else None
            return result

        # Use small shift amounts most of the time.
        for a, b in operands(40):
            amount = random.choice(
                [0, 1, 2, 5, 16, 31, 32, 33, 40, 255, 256, b])
            r3 = random.getrandbits(32)
            for set_flags in [True, False]:
                flags_in = random.choice([0, C, N | Z | C | V])
                carry = 1 if flags_in & C else 0
                _, value, flags = by_register(set_flags)(
                    a, amount, 0, r3, carry)
                flags_out = flags_in
                if flags:
                    flags_out = (flags_in & ~flags[1]) | flags[0]
                name = kind + ('s' if set_flags else '')
                cases.append(Case(
                    name, f'{name}.w r3, r0, r1',
                    [a, amount, 0, r3], flags_in, 0, value, flags_out))


def bitfield(lsb, width):
    return ((1 << width) - 1) << lsb


def saturate_unsigned(value, bits):
    high = (1 << bits) - 1
    clamped = max(0, min(high, value))
    return clamped, clamped != value


def make_misc():
    def unary(name, fn, count=24):
        emit(name, f'{name} r3, r0',
             lambda a, b, r2, r3, c: (r2, fn(a) & M, None), count)

    unary('clz', lambda a: 32 - a.bit_length())
    unary('rbit', lambda a: int(f'{a:032b}'[::-1], 2))
    unary('rev.w', lambda a: int.from_bytes(a.to_bytes(4, 'little'), 'big'))
    unary('rev16.w', lambda a: ((a & 0x00ff00ff) << 8) | ((a >> 8) & 0x00ff00ff))
    unary('revsh.w', lambda a: signed16(((a & 0xff) << 8) | ((a >> 8) & 0xff)))

    for rotation in [0, 8, 16, 24]:
        def rotated(a, rotation=rotation):
            return ((a >> rotation) | (a << (32 - rotation))) & M
        suffix = f', ror #{rotation}' if rotation else ''
        for name, fn in [
                ('uxtb', lambda v: v & 0xff),
                ('uxth', lambda v: v & 0xffff),
                ('sxtb', lambda v: signed8(v & 0xff)),
                ('sxth', lambda v: signed16(v & 0xffff))]:
            emit(name, f'{name}.w r3, r0{suffix}',
                 lambda a, b, r2, r3, c, fn=fn, rotated=rotated:
                 (r2, fn(rotated(a)) & M, None), 8)
            emit(name.replace('xt', 'xta'),
                 f'{name.replace("xt", "xta")} r3, r1, r0{suffix}',
                 lambda a, b, r2, r3, c, fn=fn, rotated=rotated:
                 (r2, (b + fn(rotated(a))) & M, None), 8)

    for lsb, width in [(0, 1), (0, 32), (4, 8), (31, 1), (7, 17), (16, 16)]:
        mask = bitfield(lsb, width)
        emit('ubfx', f'ubfx r3, r0, #{lsb}, #{width}',
             lambda a, b, r2, r3, c, lsb=lsb, mask=mask:
             (r2, (a & mask) >> lsb, None), 6)
        emit('sbfx', f'sbfx r3, r0, #{lsb}, #{width}',
             lambda a, b, r2, r3, c, lsb=lsb, mask=mask, width=width:
             (r2, sign_extend((a & mask) >> lsb, width), None), 6)
        emit('bfi', f'bfi r3, r0, #{lsb}, #{width}',
             lambda a, b, r2, r3, c, lsb=lsb, mask=mask:
             (r2, (r3 & ~mask) | ((a << lsb) & mask), None), 6)
        emit('bfc', f'bfc r3, #{lsb}, #{width}',
             lambda a, b, r2, r3, c, mask=mask: (r2, r3 & ~mask, None), 3)

    for bits in [0, 1, 7, 8, 16, 31]:
        for kind, amount in [('lsl', 0), ('lsl', 4), ('asr', 3), ('asr', 31)]:
            def usat(a, b, r2, r3, c, bits=bits, kind=kind, amount=amount):
                value = signed(a)
                if kind == 'lsl':
                    value = signed((value << amount) & M)
                else:
                    value >>= amount
                result, saturated = saturate_unsigned(value, bits)
                return r2, result, (Q if saturated else 0, Q) \
                    if saturated else None
            suffix = f', {kind} #{amount}' if amount else ''
            emit('usat', f'usat r3, #{bits}, r0{suffix}', usat, 8)


def signed8(x):
    return x - 0x100 if x & 0x80 else x


def signed16(x):
    return x - 0x10000 if x & 0x8000 else x


def sign_extend(x, width):
    return (x - (1 << width)) & M if x & (1 << (width - 1)) else x


def make_multiply():
    emit('mul', 'mul r3, r0, r1',
         lambda a, b, r2, r3, c: (r2, a * b, None), 40)
    emit('mla', 'mla r3, r0, r1, r2',
         lambda a, b, r2, r3, c: (r2, a * b + r2, None), 40)
    emit('mls', 'mls r3, r0, r1, r2',
         lambda a, b, r2, r3, c: (r2, r2 - a * b, None), 40)

    def long_result(value):
        value &= (1 << 64) - 1
        return value & M, value >> 32

    def umull(a, b, r2, r3, c):
        low, high = long_result(a * b)
        return low, high, None

    def smull(a, b, r2, r3, c):
        low, high = long_result(signed(a) * signed(b))
        return low, high, None

    def umlal(a, b, r2, r3, c):
        low, high = long_result(a * b + ((r3 << 32) | r2))
        return low, high, None

    def smlal(a, b, r2, r3, c):
        low, high = long_result(signed(a) * signed(b) + ((r3 << 32) | r2))
        return low, high, None

    emit('umull', 'umull r2, r3, r0, r1', umull, 40)
    emit('smull', 'smull r2, r3, r0, r1', smull, 40)
    emit('umlal', 'umlal r2, r3, r0, r1', umlal, 40)
    emit('smlal', 'smlal r2, r3, r0, r1', smlal, 40)

    def udiv(a, b, r2, r3, c):
        return r2, (a // b if b else 0), None

    def sdiv(a, b, r2, r3, c):
        if b == 0:
            return r2, 0, None
        quotient = abs(signed(a)) // abs(signed(b))
        if (signed(a) < 0) != (signed(b) < 0):
            quotient = -quotient
        return r2, quotient, None

    emit('udiv', 'udiv r3, r0, r1', udiv, 60)
    emit('sdiv', 'sdiv r3, r0, r1', sdiv, 60)

    def smulbb(a, b, r2, r3, c):
        return r2, signed16(a & 0xffff) * signed16(b & 0xffff), None

    def smlabb(a, b, r2, r3, c):
        product = signed16(a & 0xffff) * signed16(b & 0xffff)
        total = product + signed(r2)
        overflow = total != signed(total & M)
        return r2, total, (Q, Q) if overflow else None

    emit('smulbb', 'smulbb r3, r0, r1', smulbb, 30)
    emit('smlabb', 'smlabb r3, r0, r1, r2', smlabb, 30)


def make_narrow():
    '''The 16 bit encodings, which outside of an IT block always set
    the flags.'''
    def arith(op):
        fn = arithmetic(op)

        def result(a, b, r2, r3, carry):
            value, flags = fn(a, b, carry)
            return r2, value, (flags, NZCV)
        return result

    emit('adds.n', 'adds r3, r0, r1', arith('add'), 30)
    emit('subs.n', 'subs r3, r0, r1', arith('sub'), 30)
    for const in [0, 1, 7]:
        emit('adds.n', f'adds r3, r0, #{const}',
             lambda a, b, r2, r3, c, k=const:
             (r2,) + flagged(add_with_carry(a, k, 0)), 6)
        emit('subs.n', f'subs r3, r0, #{const}',
             lambda a, b, r2, r3, c, k=const:
             (r2,) + flagged(add_with_carry(a, ~k & M, 1)), 6)
    emit('negs.n', 'negs r3, r0',
         lambda a, b, r2, r3, c: (r2,) + flagged(add_with_carry(~a & M, 0, 1)),
         16)

    # These operate upon r3 in place, with r1 as the other operand.
    def in_place(fn, mask):
        def result(a, b, r2, r3, carry):
            value, flags = fn(r3, b, carry)
            return r2, value, (flags, mask)
        return result

    emit('adcs.n', 'adcs r3, r1',
         in_place(lambda x, y, c: add_with_carry(x, y, c), NZCV), 30)
    emit('sbcs.n', 'sbcs r3, r1',
         in_place(lambda x, y, c: add_with_carry(x, ~y & M, c), NZCV), 30)
    for name, fn in [('ands', lambda x, y: x & y),
                     ('orrs', lambda x, y: x | y),
                     ('eors', lambda x, y: x ^ y),
                     ('bics', lambda x, y: x & ~y & M),
                     ('muls', lambda x, y: (x * y) & M)]:
        emit(name + '.n', f'{name} r3, r1',
             in_place(lambda x, y, c, fn=fn: (fn(x, y), nz(fn(x, y))), N | Z),
             20)
    emit('mvns.n', 'mvns r3, r1',
         in_place(lambda x, y, c: (~y & M, nz(~y & M)), N | Z), 16)
    for kind in ['lsl', 'lsr', 'asr', 'ror']:
        def shift(x, y, carry, kind=kind):
            value, carry_out = shift_c(x, kind, y & 0xff, carry)
            return value, nz(value) | (C if carry_out else 0)

        for a, b in operands(30):
            amount = random.choice([0, 1, 4, 31, 32, 33, 255, b])
            r3 = random.getrandbits(32)
            flags_in = random.choice([0, C, N | Z | C | V])
            value, flags = shift(r3, amount, 1 if flags_in & C else 0)
            cases.append(Case(
                kind + 's.n', f'{kind}s r3, r1', [a, amount, 0, r3],
                flags_in, 0, value, (flags_in & V) | flags))
        if kind == 'ror':
            continue
        for amount in [1, 5, 31]:
            def imm(a, b, r2, r3, carry, kind=kind, amount=amount):
                value, carry_out = shift_c(a, kind, amount, carry)
                return (r2, value,
                        (nz(value) | (C if carry_out else 0), N | Z | C))
            emit(kind + 's.n', f'{kind}s r3, r0, #{amount}', imm, 8)

    def compare(fn):
        def result(a, b, r2, r3, carry):
            _, flags = fn(a, b, carry)
            return r2, r3, (flags, NZCV)
        return result

    emit('cmp.n', 'cmp r0, r1', compare(arithmetic('cmp')), 30)
    emit('cmn.n', 'cmn r0, r1', compare(arithmetic('cmn')), 20)
    emit('tst.n', 'tst r0, r1',
         lambda a, b, r2, r3, c: (r2, r3, (nz(a & b), N | Z)), 20)
    for name, fn in [('uxtb', lambda v: v & 0xff),
                     ('uxth', lambda v: v & 0xffff),
                     ('sxtb', lambda v: signed8(v & 0xff)),
                     ('sxth', lambda v: signed16(v & 0xffff))]:
        emit(name + '.n', f'{name} r3, r0',
             lambda a, b, r2, r3, c, fn=fn: (r2, fn(a) & M, None), 8)
    emit('rev.n', 'rev r3, r0',
         lambda a, b, r2, r3, c:
         (r2, int.from_bytes(a.to_bytes(4, 'little'), 'big'), None), 8)
    emit('rev16.n', 'rev16 r3, r0',
         lambda a, b, r2, r3, c:
         (r2, ((a & 0x00ff00ff) << 8) | ((a >> 8) & 0x00ff00ff), None), 8)


def flagged(result_and_flags):
    value, flags = result_and_flags
    return value, (flags, NZCV)


def encode(instructions):
    '''Returns the halfwords of each of a list of instructions.'''
    with tempfile.TemporaryDirectory() as tmp:
        source = tmp + '/enc.s'
        obj = tmp + '/enc.o'
        with open(source, 'w') as f:
            f.write('.syntax unified\n.cpu cortex-m4\n.thumb\n')
            f.write(''.join(x + '\n' for x in instructions))
        subprocess.check_call(['arm-none-eabi-as', '-o', obj, source])
        listing = subprocess.check_output(
            ['arm-none-eabi-objdump', '-d', obj]).decode()
    result = re.findall(
        r'^\s+[0-9a-f]+:\s+([0-9a-f]{4})(?: ([0-9a-f]{4}))?\s', listing, re.M)
    assert len(result) == len(instructions), (len(result), len(instructions))
    return result


def main():
    random.seed(1)
    make_data_processing()
    make_shifts()
    make_misc()
    make_multiply()
    make_narrow()

    assembly = sorted(set(x.asm for x in cases))
    encodings = dict(zip(assembly, encode(assembly)))

    print('# Generated by make_int_vectors.py from the documented')
    print('# behavior of each instruction.')
    print('#')
    print('# name hw1 hw2 flags r0 r1 r2 r3 -> r2 r3 flags')
    print('#')
    print('# hw2 is ---- for a 16 bit instruction.  Flags are NZCVQ, as')
    print('# in APSR[31:27].')
    for c in cases:
        hw1, hw2 = encodings[c.asm]
        inputs = ' '.join(f'{x:08x}' for x in c.inputs)
        print(f'{c.name} {hw1} {hw2 or "----"} {c.flags_in:02x} {inputs} '
              f'-> {c.r2:08x} {c.r3:08x} {c.flags_out:02x}')


if __name__ == '__main__':
    main()
