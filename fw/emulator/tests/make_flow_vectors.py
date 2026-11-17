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

"""Generates flow_vectors.txt, the expected results of short
sequences of load, store, branch and IT block instructions, from an
independent model of their documented behavior in the ARMv7-M
Architecture Reference Manual.

Each sequence is run from CODE with r4 and sp pointing into the
middle of a small buffer at DATA.

Requires arm-none-eabi-as.

  ./make_flow_vectors.py > flow_vectors.txt
"""

import random
import subprocess
import tempfile

M = 0xffffffff

# These must match cpu_test.rs.
CODE = 0x10000000
DATA = 0x20001000
DATA_WORDS = 8
POINTER = DATA + 16

N, Z, C, V = 8, 4, 2, 1

CONDITIONS = {
    'eq': lambda f: bool(f & Z),
    'ne': lambda f: not f & Z,
    'cs': lambda f: bool(f & C),
    'cc': lambda f: not f & C,
    'mi': lambda f: bool(f & N),
    'pl': lambda f: not f & N,
    'vs': lambda f: bool(f & V),
    'vc': lambda f: not f & V,
    'hi': lambda f: bool(f & C) and not f & Z,
    'ls': lambda f: not f & C or bool(f & Z),
    'ge': lambda f: bool(f & N) == bool(f & V),
    'lt': lambda f: bool(f & N) != bool(f & V),
    'gt': lambda f: not f & Z and bool(f & N) == bool(f & V),
    'le': lambda f: bool(f & Z) or bool(f & N) != bool(f & V),
}
OPPOSITE = {'eq': 'ne', 'ne': 'eq', 'cs': 'cc', 'cc': 'cs', 'mi': 'pl',
            'pl': 'mi', 'vs': 'vc', 'vc': 'vs', 'hi': 'ls', 'ls': 'hi',
            'ge': 'lt', 'lt': 'ge', 'gt': 'le', 'le': 'gt'}


class State:
    def __init__(self):
        # r0 to r5, then sp and lr.
        self.r = [random.getrandbits(32) for _ in range(6)]
        self.r[1] = 0
        self.r[4] = POINTER
        self.sp = POINTER
        self.lr = 0
        self.flags = random.choice([0, N, Z, C, V, N | V, Z | C, N | C,
                                    N | Z | C | V, C | V])
        self.mem = bytearray(random.getrandbits(8)
                             for _ in range(4 * DATA_WORDS))
        self.pc = 0

    def copy(self):
        result = State.__new__(State)
        result.r = list(self.r)
        result.sp = self.sp
        result.lr = self.lr
        result.flags = self.flags
        result.mem = bytearray(self.mem)
        result.pc = self.pc
        return result

    def read(self, addr, size, signed=False):
        offset = addr - DATA
        assert 0 <= offset and offset + size <= len(self.mem), hex(addr)
        value = int.from_bytes(self.mem[offset:offset + size], 'little')
        if signed and value & (1 << (8 * size - 1)):
            value -= 1 << (8 * size)
        return value & M

    def write(self, addr, size, value):
        offset = addr - DATA
        assert 0 <= offset and offset + size <= len(self.mem), hex(addr)
        self.mem[offset:offset + size] = \
            (value & ((1 << (8 * size)) - 1)).to_bytes(size, 'little')

    def text(self):
        regs = ' '.join(f'{x & M:08x}' for x in self.r + [self.sp, self.lr])
        mem = ' '.join(f'{self.read(DATA + 4 * i, 4):08x}'
                       for i in range(DATA_WORDS))
        return f'{self.flags:x} {regs} {mem}'


cases = []


def emit(name, asm, steps, model, count=4, setup=None):
    '''asm is a list of lines.  model updates a State in place, and
    returns the resulting offset of the PC from CODE.'''
    for _ in range(count):
        before = State()
        if setup:
            setup(before)
        after = before.copy()
        after.pc = model(after)
        cases.append((name, tuple(asm), steps, before, after))


def make_loads_and_stores():
    def modes(size):
        '''Yields (assembly, index register value, function of the
        state returning (address, written back base)).'''
        offsets = [0, 4, 8, 12 - size + 4 - 4, -4, -8, -16]
        for k in offsets:
            yield (f'[r4, #{k}]', 0,
                   lambda s, k=k: (s.r[4] + k, None))
            if k != 0:
                yield (f'[r4, #{k}]!', 0,
                       lambda s, k=k: (s.r[4] + k, s.r[4] + k))
                yield (f'[r4], #{k}', 0,
                       lambda s, k=k: (s.r[4], s.r[4] + k))
        for index in [0, 4, 8, 0xfffffffc, 0xfffffff0]:
            yield ('[r4, r1]', index,
                   lambda s: ((s.r[4] + s.r[1]) & M, None))
        for index in [0, 1, 2, 0xfffffffe]:
            yield ('[r4, r1, lsl #2]', index,
                   lambda s: ((s.r[4] + (s.r[1] << 2)) & M, None))

    loads = [('ldr', 4, False), ('ldrb', 1, False), ('ldrh', 2, False),
             ('ldrsb', 1, True), ('ldrsh', 2, True)]
    stores = [('str', 4), ('strb', 1), ('strh', 2)]

    for op, size, signed in loads:
        for text, index, where in modes(size):
            def model(s, size=size, signed=signed, where=where):
                addr, writeback = where(s)
                s.r[0] = s.read(addr, size, signed)
                if writeback is not None:
                    s.r[4] = writeback & M
                return None

            def setup(s, index=index):
                s.r[1] = index
            emit(op, [f'{op} r0, {text}'], 1, model, 2, setup)
            if '!' not in text and ']' == text[-1]:
                emit(op + '.w', [f'{op}.w r0, {text}'], 1, model, 1, setup)

    for op, size in stores:
        for text, index, where in modes(size):
            def model(s, size=size, where=where):
                addr, writeback = where(s)
                s.write(addr, size, s.r[0])
                if writeback is not None:
                    s.r[4] = writeback & M
                return None

            def setup(s, index=index):
                s.r[1] = index
            emit(op, [f'{op} r0, {text}'], 1, model, 2, setup)
            if '!' not in text and ']' == text[-1]:
                emit(op + '.w', [f'{op}.w r0, {text}'], 1, model, 1, setup)

    for k in [0, 4, -8, -16]:
        for text, pre, writeback in [(f'[r4, #{k}]', True, False),
                                     (f'[r4, #{k}]!', True, True),
                                     (f'[r4], #{k}', False, True)]:
            if k == 0 and writeback:
                continue

            def ldrd(s, k=k, pre=pre, writeback=writeback):
                addr = s.r[4] + (k if pre else 0)
                s.r[0] = s.read(addr, 4)
                s.r[2] = s.read(addr + 4, 4)
                if writeback:
                    s.r[4] = (s.r[4] + k) & M
                return None

            def strd(s, k=k, pre=pre, writeback=writeback):
                addr = s.r[4] + (k if pre else 0)
                s.write(addr, 4, s.r[0])
                s.write(addr + 4, 4, s.r[2])
                if writeback:
                    s.r[4] = (s.r[4] + k) & M
                return None
            emit('ldrd', [f'ldrd r0, r2, {text}'], 1, ldrd, 3)
            emit('strd', [f'strd r0, r2, {text}'], 1, strd, 3)

    # sp relative.
    for k in [0, 4, 12]:
        def load(s, k=k):
            s.r[0] = s.read(s.sp + k, 4)
            return None

        def store(s, k=k):
            s.write(s.sp + k, 4, s.r[0])
            return None
        emit('ldr.sp', [f'ldr r0, [sp, #{k}]'], 1, load, 2)
        emit('str.sp', [f'str r0, [sp, #{k}]'], 1, store, 2)


def make_multiples():
    registers = [0, 2, 3]

    def ldm(writeback, before):
        def model(s):
            addr = s.r[4] - (12 if before else 0)
            for i, reg in enumerate(registers):
                s.r[reg] = s.read(addr + 4 * i, 4)
            if writeback:
                s.r[4] = (s.r[4] + (-12 if before else 12)) & M
            return None
        return model

    def stm(writeback, before):
        def model(s):
            addr = s.r[4] - (12 if before else 0)
            for i, reg in enumerate(registers):
                s.write(addr + 4 * i, 4, s.r[reg])
            if writeback:
                s.r[4] = (s.r[4] + (-12 if before else 12)) & M
            return None
        return model

    emit('ldmia', ['ldmia r4, {r0, r2, r3}'], 1, ldm(False, False), 6)
    emit('ldmia!', ['ldmia r4!, {r0, r2, r3}'], 1, ldm(True, False), 6)
    # LDMDB is omitted: the simulator does not implement it, and the
    # firmware does not use it.
    emit('stmia', ['stmia.w r4, {r0, r2, r3}'], 1, stm(False, False), 6)
    emit('stmia!', ['stmia r4!, {r0, r2, r3}'], 1, stm(True, False), 6)
    emit('stmdb', ['stmdb r4, {r0, r2, r3}'], 1, stm(False, True), 6)
    emit('stmdb!', ['stmdb r4!, {r0, r2, r3}'], 1, stm(True, True), 6)

    def push(s):
        s.sp -= 12
        for i, reg in enumerate(registers):
            s.write(s.sp + 4 * i, 4, s.r[reg])
        return None

    def pop(s):
        for i, reg in enumerate(registers):
            s.r[reg] = s.read(s.sp + 4 * i, 4)
        s.sp += 12
        return None

    emit('push', ['push {r0, r2, r3}'], 1, push, 6)
    emit('pop', ['pop {r0, r2, r3}'], 1, pop, 6)
    emit('push.w', ['push.w {r0, r2, r3}'], 1, push, 4)
    emit('pop.w', ['pop.w {r0, r2, r3}'], 1, pop, 4)

    # Whatever is done to the registers in between, a pop restores
    # what was pushed.
    emit('push_pop',
         ['push {r0, r2, r3}', 'mov.w r0, #1', 'mov.w r2, #2',
          'mov.w r3, #3', 'pop {r0, r2, r3}'],
         5, lambda s: (push(s), pop(s))[1], 4)


def make_branches():
    for cond, passes in CONDITIONS.items():
        for width in ['.n', '.w']:
            size = 2 if width == '.n' else 4
            asm = [f'b{cond}{width} 1f', 'nop', 'nop', '1:', 'nop']

            def model(s, passes=passes, size=size):
                return size + 4 if passes(s.flags) else size
            emit(f'b{cond}{width}', asm, 1, model, 12)

    def cbz(zero):
        def model(s):
            return 6 if (s.r[0] == 0) == zero else 2
        return model

    def maybe_zero(s):
        if random.random() < 0.5:
            s.r[0] = 0
    emit('cbz', ['cbz r0, 1f', 'nop', 'nop', '1:', 'nop'], 1, cbz(True),
         12, maybe_zero)
    emit('cbnz', ['cbnz r0, 1f', 'nop', 'nop', '1:', 'nop'], 1, cbz(False),
         12, maybe_zero)

    def bl(s):
        s.lr = CODE + 4 + 1
        return 8
    emit('bl', ['bl 1f', 'nop', 'nop', '1:', 'nop'], 1, bl, 3)

    def to_register(link):
        def model(s):
            if link:
                s.lr = CODE + 2 + 1
            return 6
        return model

    def target(s):
        s.r[0] = CODE + 6 + 1
    emit('bx', ['bx r0', 'nop', 'nop', 'nop'], 1, to_register(False), 3,
         target)
    emit('blx', ['blx r0', 'nop', 'nop', 'nop'], 1, to_register(True), 3,
         target)

    # tbb and tbh branch forward by twice the table entry from the
    # end of the instruction.
    for index in [0, 1, 2]:
        def setup(s, index=index):
            s.r[1] = index
        emit('tbb',
             ['tbb [pc, r1]', '.byte 2, 3, 4, 0', 'nop', 'nop', 'nop'],
             1, lambda s, index=index: 4 + 2 * [2, 3, 4][index], 2, setup)
        emit('tbh',
             ['tbh [pc, r1, lsl #1]', '.short 3, 4, 5', 'nop', 'nop', 'nop'],
             1, lambda s, index=index: 4 + 2 * [3, 4, 5][index], 2, setup)

    def literal(s):
        s.r[0] = 0x12345678
        return None
    emit('ldr.literal', ['ldr r0, 1f', 'nop', '1:', '.word 0x12345678'],
         1, literal, 2)
    emit('ldr.literal.w',
         ['ldr.w r0, 1f', '1:', '.word 0x12345678'], 1, literal, 2)


def make_it_blocks():
    for cond, passes in CONDITIONS.items():
        other = OPPOSITE[cond]
        for pattern in ['t', 'te', 'tt', 'tee', 'tet', 'ttt', 'tttt',
                        'tete', 'teee', 'ttee']:
            conditions = [cond if x == 't' else other for x in pattern]
            asm = [f'i{pattern} {cond}']
            # Each instruction in the block is of a different kind,
            # none of which may alter the flags there.
            operations = [
                ('mov{c} r0, #1', lambda s: s.r.__setitem__(0, 1)),
                ('add{c} r2, r2, r3',
                 lambda s: s.r.__setitem__(2, (s.r[2] + s.r[3]) & M)),
                ('ldr{c} r3, [r4, #4]',
                 lambda s: s.r.__setitem__(3, s.read(s.r[4] + 4, 4))),
                ('str{c} r5, [r4, #-8]',
                 lambda s: s.write(s.r[4] - 8, 4, s.r[5])),
            ]
            used = operations[:len(pattern)]
            for (text, _), c in zip(used, conditions):
                asm.append(text.format(c=c))
            asm.append('nop')

            def model(s, passes=passes, pattern=pattern, used=used):
                executes = passes(s.flags)
                for (_, effect), which in zip(used, pattern):
                    if executes == (which == 't'):
                        effect(s)
                return None
            emit(f'i{pattern}', asm, 1 + len(pattern), model, 3)


def assemble(snippets):
    '''Returns the halfwords of each of a list of snippets, each of
    which is assembled as if located at CODE.'''
    lines = ['.syntax unified', '.cpu cortex-m4', '.thumb']
    for i, snippet in enumerate(snippets):
        lines += ['.balign 4', f'case_{i}:'] + list(snippet)
    lines += ['.balign 4', f'case_{len(snippets)}:']
    with tempfile.TemporaryDirectory() as tmp:
        source, obj, binary = tmp + '/a.s', tmp + '/a.o', tmp + '/a.bin'
        with open(source, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        subprocess.check_call(['arm-none-eabi-as', '-o', obj, source])
        subprocess.check_call(
            ['arm-none-eabi-objcopy', '-O', 'binary', '-j', '.text',
             obj, binary])
        symbols = subprocess.check_output(
            ['arm-none-eabi-nm', obj]).decode()
        data = open(binary, 'rb').read()
    offsets = {}
    for line in symbols.splitlines():
        value, _, name = line.split()
        if name.startswith('case_'):
            offsets[int(name[5:])] = int(value, 16)
    result = []
    for i in range(len(snippets)):
        chunk = data[offsets[i]:offsets[i + 1]]
        result.append([int.from_bytes(chunk[j:j + 2], 'little')
                       for j in range(0, len(chunk), 2)])
    return result


def main():
    random.seed(1)
    make_loads_and_stores()
    make_multiples()
    make_branches()
    make_it_blocks()

    snippets = sorted(set(x[1] for x in cases))
    encodings = dict(zip(snippets, assemble(snippets)))

    print('# Generated by make_flow_vectors.py from the documented')
    print('# behavior of each instruction.')
    print('#')
    print('# name steps halfwords : before -> after pc')
    print('#')
    print('# before and after are the NZCV flags, r0 to r5, sp, lr, and')
    print(f'# then the {DATA_WORDS} words at DATA.  pc is the final offset')
    print('# from CODE, or - if execution simply ran through.')
    for name, asm, steps, before, after in cases:
        code = ' '.join(f'{x:04x}' for x in encodings[asm])
        pc = '-' if after.pc is None else f'{after.pc:x}'
        print(f'{name} {steps} {code} : {before.text()} -> '
              f'{after.text()} {pc}')


if __name__ == '__main__':
    main()
