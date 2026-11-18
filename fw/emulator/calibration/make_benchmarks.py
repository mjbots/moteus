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

"""Generates benchmarks.txt: a program which times many short
instruction sequences with the DWT cycle counter, in order to compare
the emulator's timing model against a real STM32G474.

There are too many to fit in memory at once, so the result is a
series of programs.  Each is self contained and position dependent:
it is loaded into CCM RAM at CODE and started at its first byte,
with no need for any firmware.  For each benchmark it stores the
elapsed cycle count as one word at RESULTS, followed by END_MARKER,
and then spins.

Each sequence is bracketed by a pair of NOPs on either side, so that
it does not interact with the loads which read the counter.  The
first benchmark of each program is empty, and is to be subtracted
from the others.

Requires arm-none-eabi-as, -ld and -objcopy.

  ./make_benchmarks.py > benchmarks.txt

With --170mhz, each program first switches to the 170MHz clock the
firmware runs at.
"""

import subprocess
import sys
import tempfile

# These must match calibration_test.rs and run_on_hardware.py.
CODE = 0x10000000
DATA = 0x20000200
RESULTS = 0x20000400
STACK = 0x20002000
END_MARKER = 0xd0e5d0e5
# Scratch data in the RAM the program runs from, beyond its end.
CODE_RAM_DATA = 0x10007000

# Registers the harness reserves.
#   r5: where the next result goes
#   r6: the address of DWT_CYCCNT
#   r7: the counter at the start of the benchmark
#
# Before each benchmark r4 = DATA, r1 = 4, r2 = 8, r3 = 3, r0 = 0, the
# word at DATA holds DATA + 16, and the flags are clear.

benchmarks = []

# Whether to run at 170MHz, as the firmware does, rather than the
# 16MHz the processor starts at.
FAST_CLOCK = False

# Switch to 170MHz from the PLL: HSI16 / 4 * 85 / 2.
CLOCK_SETUP = '''
    @ Enable the power controller, and select boost mode.
    ldr r0, =0x40021058
    ldr r1, [r0]
    orr r1, r1, #0x10000000
    str r1, [r0]
    ldr r1, [r0]
    ldr r0, =0x40007080
    ldr r1, [r0]
    bic r1, r1, #0x100
    str r1, [r0]
    @ Four flash wait states.
    ldr r0, =0x40022000
    ldr r1, [r0]
    bic r1, r1, #0xf
    orr r1, r1, #4
    str r1, [r0]
    ldr r1, [r0]
    @ Configure and start the PLL.
    ldr r0, =0x4002100c
    ldr r1, =0x01005532
    str r1, [r0]
    ldr r0, =0x40021000
    ldr r1, [r0]
    orr r1, r1, #0x01000000
    str r1, [r0]
1:  ldr r1, [r0]
    tst r1, #0x02000000
    beq 1b
    @ Switch to it with the AHB clock halved, then restore that.
    ldr r0, =0x40021008
    ldr r1, [r0]
    bic r1, r1, #0xf0
    orr r1, r1, #0x80
    str r1, [r0]
    orr r1, r1, #3
    str r1, [r0]
2:  ldr r1, [r0]
    and r2, r1, #0xc
    cmp r2, #0xc
    bne 2b
    mov r2, #0x1000
3:  subs r2, r2, #1
    bne 3b
    bic r1, r1, #0xf0
    str r1, [r0]
    mov r2, #0x1000
4:  subs r2, r2, #1
    bne 4b
    b 5f
    .ltorg
5:
'''


def bench(name, lines, setup=(), trailer=()):
    '''Time 'lines', after running 'setup'.  'trailer' is assembled
    after the benchmark, out of the path of execution.'''
    benchmarks.append((name, list(setup), list(lines), list(trailer)))


def repeat(line, count):
    return [line] * count


def make_integer():
    bench('empty', [])
    bench('nop', ['nop'])
    bench('nop_x4', repeat('nop', 4))
    bench('nop.w', ['nop.w'])
    bench('adds', ['adds r0, r1, r2'])
    bench('add.w_x4', repeat('add.w r0, r1, r2', 4))
    bench('mul', ['muls r0, r1, r0'])
    bench('mla', ['mla r0, r1, r2, r3'])
    bench('umull', ['umull r0, r8, r1, r2'])
    bench('lsl_reg', ['lsl.w r0, r1, r2'])
    bench('clz', ['clz r0, r1'])
    bench('ubfx', ['ubfx r0, r1, #1, #2'])
    bench('movw_movt', ['movw r0, #0x1234', 'movt r0, #0x5678'])

    for name, dividend, divisor in [
            ('1_by_1', 1, 1), ('small_by_large', 3, 0x10000),
            ('0x100_by_3', 0x100, 3), ('0x10000_by_3', 0x10000, 3),
            ('0x1000000_by_3', 0x1000000, 3),
            ('max_by_1', 0xffffffff, 1), ('max_by_3', 0xffffffff, 3),
            ('max_by_0x10000', 0xffffffff, 0x10000),
            ('max_by_max', 0xffffffff, 0xffffffff),
            ('by_zero', 5, 0)]:
        setup = [f'ldr r8, ={dividend}', f'ldr r9, ={divisor}']
        bench(f'udiv_{name}', ['udiv r0, r8, r9'], setup)
    bench('sdiv_negative', ['sdiv r0, r8, r9'],
          ['ldr r8, =-1000000', 'ldr r9, =7'])

    bench('mrs', ['mrs r0, apsr'])
    bench('msr', ['msr apsr_nzcvq, r0'])
    bench('cpsid_cpsie', ['cpsid i', 'cpsie i'])
    bench('dmb', ['dmb'])
    bench('isb', ['isb'])


def make_loads_and_stores():
    bench('ldr', ['ldr r0, [r4]'])
    bench('ldr_x2', ['ldr r0, [r4]', 'ldr r3, [r4, #4]'])
    bench('ldr_x3', ['ldr r0, [r4]', 'ldr r3, [r4, #4]', 'ldr r2, [r4, #8]'])
    bench('ldr_x4', ['ldr r0, [r4]', 'ldr r3, [r4, #4]', 'ldr r2, [r4, #8]',
                     'ldr r1, [r4, #12]'])
    bench('ldr.w', ['ldr.w r0, [r4]'])
    bench('ldr.w_x2', ['ldr.w r0, [r4]', 'ldr.w r3, [r4, #4]'])
    # The second address comes from the first load.
    bench('ldr_dependent', ['ldr r0, [r4]', 'ldr r3, [r0]'])
    # The address comes from the preceding arithmetic.
    bench('add_ldr', ['adds r3, r4, #4', 'ldr r0, [r3]'])
    bench('add_nop_ldr', ['adds r3, r4, #4', 'nop', 'ldr r0, [r3]'])
    bench('ldr_reg', ['ldr r0, [r4, r1]'])
    bench('ldr_reg_shift', ['ldr.w r0, [r4, r1, lsl #1]'])
    bench('ldrb', ['ldrb r0, [r4]'])
    bench('ldrsh', ['ldrsh r0, [r4, r1]'])
    bench('ldr_unaligned', ['ldr.w r0, [r4, #1]'])
    bench('ldr_unaligned_half', ['ldr.w r0, [r4, #2]'])
    bench('ldr_pre', ['ldr r0, [r4, #4]!'])
    bench('ldr_post', ['ldr r0, [r4], #4'])
    bench('ldr_literal', ['ldr r0, 1f', 'b 2f', '.balign 4',
                          '1: .word 0x12345678', '2:'])
    bench('b_over_word', ['b 2f', '.balign 4', '1: .word 0x12345678', '2:'])

    bench('str', ['str r0, [r4]'])
    bench('str_x2', ['str r0, [r4]', 'str r1, [r4, #4]'])
    bench('str_x3', ['str r0, [r4]', 'str r1, [r4, #4]', 'str r2, [r4, #8]'])
    bench('str.w', ['str.w r0, [r4]'])
    bench('str_reg', ['str r0, [r4, r1]'])
    bench('str_reg_x2', ['str r0, [r4, r1]', 'str r3, [r4, r2]'])
    bench('strb', ['strb r0, [r4]'])
    bench('ldr_str', ['ldr r0, [r4]', 'str r0, [r4, #4]'])
    bench('ldr_str_reg', ['ldr r0, [r4]', 'str r0, [r4, r1]'])
    bench('str_ldr', ['str r0, [r4]', 'ldr r3, [r4, #4]'])
    bench('str_ldr_same', ['str r1, [r4, #8]', 'ldr r3, [r4, #8]'])
    bench('add_str', ['adds r3, r4, #4', 'str r0, [r3]'])

    bench('ldrd', ['ldrd r0, r3, [r4]'])
    bench('strd', ['strd r0, r3, [r4]'])
    for count in [1, 2, 4, 8]:
        low = ', '.join(f'r{i}' for i in range(count)) if count <= 4 else \
            'r0, r1, r2, r3, r8, r9, r10, r11'
        bench(f'push_pop_{count}', [f'push {{{low}}}', f'pop {{{low}}}'])
        bench(f'push_{count}', [f'push {{{low}}}'])
        # Rebalance the stack outside of the timed region.
        bench(f'_drop_{count}', [f'add sp, #{4 * count}'])
    bench('ldm_3', ['ldmia r4, {r0, r2, r3}'])
    bench('stm_3', ['stmia.w r4, {r0, r2, r3}'])


def make_branches():
    bench('b_taken', ['b 1f', 'nop', '.balign 4', '1:'])
    bench('b_taken_unaligned', ['b 1f', 'nop', 'nop', '.balign 4',
                                'nop', '1:'])
    # The target instruction is 32 bits wide.
    bench('b_to_wide', ['b 1f', 'nop', '.balign 4', '1:', 'nop.w'])
    bench('wide_only', ['nop.w'])
    # And it straddles a word boundary.
    bench('b_to_wide_unaligned', ['b 1f', 'nop', 'nop', '.balign 4',
                                  'nop', '1:', 'nop.w'])
    bench('b.w_taken', ['b.w 1f', 'nop', '.balign 4', '1:'])
    bench('bcc_not_taken', ['cmp r1, r1', 'bne 1f', '1:'])
    bench('cmp_only', ['cmp r1, r1'])
    bench('bcc_taken', ['cmp r1, r1', 'beq 1f', 'nop', '.balign 4', '1:'])
    bench('bcc.w_taken', ['cmp r1, r1', 'beq.w 1f', 'nop', '.balign 4', '1:'])
    bench('cbz_not_taken', ['cbz r1, 1f', '1:'])
    bench('cbz_taken', ['cbz r0, 1f', 'nop', '.balign 4', '1:'])
    bench('bl_bx', ['bl 1f', 'b 2f', '.balign 4', '1: bx lr',
                    '.balign 4', '2:'])
    bench('blx_reg', ['adr r0, 1f', 'adds r0, #1', 'blx r0', 'b 2f',
                      '.balign 4', '1: bx lr', '.balign 4', '2:'])
    bench('adr_only', ['adr r0, 1f', 'adds r0, #1', '.balign 4', '1:'])
    bench('push_pop_pc', ['bl 1f', 'b 2f', '.balign 4',
                          '1: push {r4, lr}', 'pop {r4, pc}',
                          '.balign 4', '2:'])
    bench('tbb', ['tbb [pc, r0]', '1: .byte (2f - 1b) / 2, 0',
                  '.balign 4', '2:'])
    bench('loop_x8', ['movs r0, #8', '1: subs r0, #1', 'bne 1b'])

    # Whether an IT instruction is free may depend on whether it is
    # the first or second half of a word.
    bench('it_second_half', ['cmp r1, r1', 'it eq', 'addeq r0, r1'])
    bench('it_first_half', ['cmp r1, r1', 'nop', 'it eq', 'addeq r0, r1'])
    bench('cmp_nop_add', ['cmp r1, r1', 'nop', 'adds r0, r1'])
    bench('it_skipped', ['cmp r1, r1', 'it ne', 'addne r0, r1'])
    bench('itttt_skipped', ['cmp r1, r1', 'itttt ne', 'addne r0, r1',
                            'addne r0, r1', 'addne r0, r1', 'addne r0, r1'])
    bench('it_skipped_wide', ['cmp r1, r1', 'it ne', 'addne.w r0, r1, r2'])
    bench('it_skipped_load', ['cmp r1, r1', 'it ne', 'ldrne r0, [r4]'])


def make_floating_point():
    setup = ['vldr s0, [r4, #32]', 'vldr s1, [r4, #36]',
             'vldr s2, [r4, #40]', 'vmov.f32 s3, s0', 'vmov.f32 s4, s1',
             'vmov.f32 s5, s2']

    def fp(name, lines):
        bench(name, lines, setup)

    fp('vadd', ['vadd.f32 s3, s0, s1'])
    fp('vmul', ['vmul.f32 s3, s0, s1'])
    # Independent operations.
    fp('vadd_x3', ['vadd.f32 s3, s0, s1', 'vadd.f32 s4, s0, s2',
                   'vadd.f32 s5, s1, s2'])
    # Each consumes the previous result.
    fp('vadd_chain_x3', ['vadd.f32 s3, s0, s1', 'vadd.f32 s3, s3, s2',
                         'vadd.f32 s3, s3, s1'])
    fp('vmul_vadd_chain', ['vmul.f32 s3, s0, s1', 'vadd.f32 s3, s3, s2'])
    fp('vfma', ['vfma.f32 s3, s0, s1'])
    fp('vfma_x2', ['vfma.f32 s3, s0, s1', 'vfma.f32 s4, s0, s2'])
    fp('vfma_chain_x2', ['vfma.f32 s3, s0, s1', 'vfma.f32 s3, s0, s2'])
    fp('vmul_vfma', ['vmul.f32 s4, s0, s1', 'vfma.f32 s3, s4, s2'])
    fp('vdiv', ['vdiv.f32 s3, s0, s1'])
    fp('vdiv_vadd', ['vdiv.f32 s3, s0, s1', 'vadd.f32 s4, s0, s2'])
    fp('vdiv_vadd_chain', ['vdiv.f32 s3, s0, s1', 'vadd.f32 s4, s3, s2'])
    for count in [4, 8, 16]:
        fp(f'vdiv_adds_x{count}_vadd',
           ['vdiv.f32 s3, s0, s1'] + repeat('adds r0, r1', count) +
           ['vadd.f32 s4, s0, s2'])
    fp('vsqrt', ['vsqrt.f32 s3, s0'])
    fp('vsqrt_vadd', ['vsqrt.f32 s3, s0', 'vadd.f32 s4, s0, s2'])
    fp('vabs', ['vabs.f32 s3, s0'])
    fp('vneg', ['vneg.f32 s3, s0'])
    fp('vmov_reg', ['vmov.f32 s3, s0'])
    fp('vmov_imm', ['vmov.f32 s3, #1.0'])
    fp('vmov_to_core', ['vmov r0, s0'])
    fp('vmov_from_core', ['vmov s3, r1'])
    fp('vadd_vmov_to_core', ['vadd.f32 s3, s0, s1', 'vmov r0, s3'])
    fp('vcvt_to_int', ['vcvt.s32.f32 s3, s0'])
    fp('vcvt_from_int', ['vcvt.f32.s32 s3, s0'])
    fp('vcvt_fixed', ['vcvt.s32.f32 s3, s3, #16'])
    fp('vcmp_vmrs', ['vcmpe.f32 s0, s1', 'vmrs APSR_nzcv, fpscr'])
    fp('vcmp_vmrs_it_vmov', ['vcmpe.f32 s0, s1', 'vmrs APSR_nzcv, fpscr',
                             'it gt', 'vmovgt.f32 s3, s1'])
    fp('vldr', ['vldr s3, [r4, #32]'])
    fp('vldr_x2', ['vldr s3, [r4, #32]', 'vldr s4, [r4, #36]'])
    fp('vldr_x3', ['vldr s3, [r4, #32]', 'vldr s4, [r4, #36]',
                   'vldr s5, [r4, #40]'])
    fp('ldr_vldr', ['ldr r0, [r4]', 'vldr s3, [r4, #32]'])
    fp('vstr', ['vstr s0, [r4, #48]'])
    fp('vstr_x2', ['vstr s0, [r4, #48]', 'vstr s1, [r4, #52]'])
    fp('vadd_vstr', ['vadd.f32 s3, s0, s1', 'vstr s3, [r4, #48]'])
    fp('vldr_vadd', ['vldr s3, [r4, #32]', 'vadd.f32 s4, s3, s1'])
    # Compilers save these registers as doubles.
    fp('vpush_vpop_2', ['vpush {d8}', 'vpop {d8}'])
    fp('vpush_vpop_8', ['vpush {d8-d11}', 'vpop {d8-d11}'])


def make_peripherals():
    def periph(name, address, lines):
        bench(name, lines, [f'ldr r8, ={address}'])

    # These are all read-only as far as the benchmark is concerned.
    periph('read_gpioa_idr_ahb2', 0x48000010, ['ldr r0, [r8]'])
    periph('read_gpioa_idr_ahb2_x2', 0x48000010,
           ['ldr r0, [r8]', 'ldr r3, [r8]'])
    periph('read_adc1_isr_ahb2', 0x50000000, ['ldr r0, [r8]'])
    periph('read_tim2_cnt_apb1', 0x40000024, ['ldr r0, [r8]'])
    periph('read_tim2_cnt_apb1_x2', 0x40000024,
           ['ldr r0, [r8]', 'ldr r3, [r8]'])
    periph('read_tim1_cnt_apb2', 0x40012c24, ['ldr r0, [r8]'])
    periph('read_rcc_cr_ahb1', 0x40021000, ['ldr r0, [r8]'])
    periph('read_scb_icsr_ppb', 0xe000ed04, ['ldr r0, [r8]'])
    # The unused scratch register of TIM2: its auto reload value
    # while the timer is stopped.
    periph('write_tim2_arr_apb1', 0x4000002c, ['str r1, [r8]'])
    periph('write_tim2_arr_apb1_x2', 0x4000002c,
           ['str r1, [r8]', 'str r2, [r8]'])
    # Writing zero to the bit set/reset register does nothing.
    periph('write_gpioa_bsrr_ahb2', 0x48000018, ['str r0, [r8]'])
    periph('write_read_gpioa_ahb2', 0x48000018,
           ['str r0, [r8]', 'ldr r3, [r8, #-8]'])


def make_exceptions():
    pend = ['ldr r8, =0xe000ed04', 'mov r9, #0x10000000']
    # PendSV with no floating point context to save.
    bench('pendsv', ['str r9, [r8]', 'isb'],
          ['bl clear_fp_context'] + pend)
    bench('isb_after_store', ['str r0, [r4]', 'isb'])
    # With a floating point context, but a handler which leaves it
    # alone, so that nothing is actually saved.
    bench('pendsv_fp_context', ['str r9, [r8]', 'isb'],
          ['vmov.f32 s0, #1.0'] + pend)
    # And a handler which uses the FPU, forcing the lazy save.
    bench('pendsv_fp_used', ['str r9, [r8]', 'isb'],
          ['vmov.f32 s0, #1.0', 'ldr r0, =fp_handler', 'ldr r1, =pendsv_target',
           'str r0, [r1]'] + pend)
    bench('_restore_handler', [],
          ['ldr r0, =plain_handler', 'ldr r1, =pendsv_target', 'str r0, [r1]',
           'bl clear_fp_context'])


def make_followups():
    '''Benchmarks added to separate effects which the first results
    from hardware left ambiguous.'''
    bench('nop_x16', repeat('nop', 16))
    bench('cpsid', ['cpsid i'])
    bench('_cpsie', ['cpsie i'])
    bench('cpsie', ['cpsie i'])

    # Does using the result of a load cost anything?
    bench('ldr_add_use', ['ldr r0, [r4]', 'adds r3, r0, r1'])
    bench('ldr_nop_add_use', ['ldr r0, [r4]', 'nop', 'adds r3, r0, r1'])
    bench('ldr_cmp_use', ['ldr r0, [r4]', 'cmp r0, #0'])
    bench('ldr_x2_add_use', ['ldr r0, [r4]', 'ldr r3, [r4, #4]',
                             'adds r2, r0, r3'])
    bench('ldr_str_to_loaded', ['ldr r0, [r4]', 'str r1, [r0]'])
    bench('mov_ldr', ['mov r3, r4', 'ldr r0, [r3]'])
    bench('ldrb_add_use', ['ldrb r0, [r4]', 'adds r3, r0, r1'])
    bench('ldr_mul_use', ['ldr r0, [r4]', 'muls r0, r1, r0'])
    bench('ldr_bx', ['ldr r0, [r4]', 'b 1f', '.balign 4', '1:'])
    bench('ldr_x8', ['ldr r0, [r4]', 'ldr r3, [r4, #4]'] * 4)
    bench('ldr_add_x4', ['ldr r0, [r4]', 'adds r2, r1, r1'] * 4)
    bench('str_add_x4', ['str r1, [r4]', 'adds r2, r1, r1'] * 4)
    bench('str_x8', ['str r1, [r4]', 'str r2, [r4, #4]'] * 4)
    bench('str_reg_x3', ['str r0, [r4, r1]', 'str r3, [r4, r2]',
                         'str r3, [r4, r1]'])
    bench('str_reg_nop_str_reg', ['str r0, [r4, r1]', 'nop',
                                  'str r3, [r4, r2]'])
    bench('str_str_reg', ['str r0, [r4]', 'str r3, [r4, r2]'])
    bench('ldr_reg_x2', ['ldr r0, [r4, r1]', 'ldr r3, [r4, r2]'])
    bench('ldrd_add', ['ldrd r0, r3, [r4]', 'adds r2, r0, r3'])
    bench('ldrd_only_x2', ['ldrd r0, r3, [r4]', 'ldrd r2, r1, [r4, #8]'])
    bench('pop_1_only', ['pop {r0}'], ['push {r0}'])
    bench('pop_4_only', ['pop {r0, r1, r2, r3}'], ['push {r0, r1, r2, r3}'])

    # Data in the RAM which code is executing from.
    code = ['ldr r8, =code_ram_data']
    bench('ldr_code_ram', ['ldr.w r0, [r8]'], code)
    bench('ldr_code_ram_x2', ['ldr.w r0, [r8]', 'ldr.w r3, [r8, #4]'], code)
    bench('ldr_code_ram_x4', ['ldr.w r0, [r8]', 'ldr.w r3, [r8, #4]',
                              'ldr.w r2, [r8, #8]', 'ldr.w r1, [r8, #12]'],
          code)
    bench('ldr_code_ram_add_use', ['ldr.w r0, [r8]', 'adds r3, r0, r1'], code)
    bench('ldr_code_ram_dependent', ['ldr.w r0, [r8]', 'ldr r3, [r0]'], code)
    bench('str_code_ram', ['str.w r0, [r8, #16]'], code)
    bench('str_code_ram_x2', ['str.w r0, [r8, #16]', 'str.w r1, [r8, #20]'],
          code)
    bench('ldr_str_code_ram', ['ldr.w r0, [r8]', 'str.w r0, [r8, #16]'], code)
    bench('vldr_code_ram', ['vldr s3, [r8, #24]'], code)
    bench('vldr_code_ram_x2', ['vldr s3, [r8, #24]', 'vldr s4, [r8, #28]'],
          code)
    bench('ldr_sram_ldr_code_ram', ['ldr r0, [r4]', 'ldr.w r3, [r8]'], code)

    # Literals, which are necessarily in that same RAM.
    bench('ldr_pool', ['ldr r0, 1f'], trailer=['1: .word 0x12345678'])
    bench('ldr_pool_x2', ['ldr r0, 1f', 'ldr r3, 2f'],
          trailer=['1: .word 0x12345678', '2: .word 0x23456789'])
    bench('ldr_pool_x4', ['ldr r0, 1f', 'ldr r3, 2f', 'ldr r2, 1f',
                          'ldr r1, 2f'],
          trailer=['1: .word 0x12345678', '2: .word 0x23456789'])
    bench('ldr_pool_add_use', ['ldr r0, 1f', 'adds r3, r0, r1'],
          trailer=['1: .word 0x12345678'])
    bench('ldr.w_pool', ['ldr.w r0, 1f'], trailer=['1: .word 0x12345678'])
    bench('vldr_pool', ['vldr s3, 1f'], trailer=['1: .float 1.5'])
    bench('vldr_pool_x2', ['vldr s3, 1f', 'vldr s4, 2f'],
          trailer=['1: .float 1.5', '2: .float 2.5'])
    bench('vldr_pool_vadd_use', ['vldr s3, 1f', 'vadd.f32 s4, s3, s3'],
          trailer=['1: .float 1.5'])
    bench('ldr_pool_ldr', ['ldr r0, 1f', 'ldr r3, [r4]'],
          trailer=['1: .word 0x12345678'])
    bench('ldr_pool_dependent', ['ldr r0, 1f', 'ldr r3, [r0]'],
          trailer=['1: .word 0x20000200'])

    # Branches whose target is already in a register.
    target = ['adr r8, 1f', 'add r8, r8, #1']
    bench('bx_reg', ['bx r8', 'nop', '.balign 4', '1:'],
          ['adr r8, 1f', 'add r8, r8, #1'])
    bench('blx_reg_preset', ['blx r8', 'b 2f', '.balign 4', '1: bx lr',
                             '.balign 4', '2:'], target)
    bench('mov_pc', ['mov pc, r8', 'nop', '.balign 4', '1:'], target)
    bench('bl_only', ['bl 1f', 'nop', '.balign 4', '1:'])
    bench('cbnz_taken', ['cbnz r1, 1f', 'nop', '.balign 4', '1:'])
    bench('cbz_taken_to_wide', ['cbz r0, 1f', 'nop', '.balign 4', '1:',
                                'nop.w'])
    bench('bcc_taken_backward', ['b 2f', '.balign 4', '1: b 3f',
                                 '.balign 4', '2: cmp r1, r1', 'beq 1b',
                                 '.balign 4', '3:'])
    bench('b_b', ['b 1f', 'nop', '.balign 4', '1: b 2f', 'nop',
                  '.balign 4', '2:'])
    bench('ldr_pc', ['ldr pc, [r4, #20]', 'nop', '.balign 4', '1:'],
          ['adr r8, 1f', 'add r8, r8, #1', 'str r8, [r4, #20]'])
    bench('pop_pc_1', ['bl 1f', 'b 2f', '.balign 4', '1: push {lr}',
                       'pop {pc}', '.balign 4', '2:'])
    bench('push_lr_only', ['push {lr}'])
    bench('_drop_lr', ['add sp, #4'])
    bench('pop_pc_4', ['bl 1f', 'b 2f', '.balign 4',
                       '1: push {r4, r8, r9, lr}', 'pop {r4, r8, r9, pc}',
                       '.balign 4', '2:'])
    bench('tbh', ['tbh [pc, r0, lsl #1]', '1: .short (2f - 1b) / 2, 0',
                  '.balign 4', '2:'])

    bench('it_after_wide', ['cmp.w r1, r1', 'it eq', 'addeq r0, r1'])
    bench('cmp.w_add', ['cmp.w r1, r1', 'adds r0, r1'])
    bench('it_after_wide_unaligned', ['nop', 'cmp.w r1, r1', 'it eq',
                                      'addeq r0, r1'])
    bench('ite_taken', ['cmp r1, r1', 'ite eq', 'addeq r0, r1',
                        'addne r0, r2'])

    for name, dividend, divisor in [
            ('7_by_8', 7, 8), ('8_by_7', 8, 7), ('5_by_5', 5, 5),
            ('15_by_1', 15, 1), ('16_by_1', 16, 1), ('31_by_1', 31, 1),
            ('32_by_1', 32, 1), ('255_by_1', 255, 1), ('256_by_1', 256, 1),
            ('0_by_5', 0, 5), ('0x80000000_by_1', 0x80000000, 1),
            ('0x7fffffff_by_1', 0x7fffffff, 1),
            ('0xffff_by_0xff', 0xffff, 0xff)]:
        bench(f'udiv_{name}', ['udiv r0, r8, r9'],
              [f'ldr r8, ={dividend}', f'ldr r9, ={divisor}'])
    for name, dividend, divisor in [
            ('minus1_by_1', -1, 1), ('min_by_minus1', -0x80000000, -1),
            ('100_by_minus3', 100, -3), ('1000000_by_7', 1000000, 7),
            ('minus7_by_minus8', -7, -8)]:
        bench(f'sdiv_{name}', ['sdiv r0, r8, r9'],
              [f'ldr r8, ={dividend}', f'ldr r9, ={divisor}'])
    bench('udiv_then_use', ['udiv r0, r8, r9', 'adds r0, r0, r1'],
          ['ldr r8, =0x10000', 'ldr r9, =3'])

    setup = ['vldr s0, [r4, #32]', 'vldr s1, [r4, #36]',
             'vldr s2, [r4, #40]', 'vmov.f32 s3, s0', 'vmov.f32 s4, s1',
             'vmov.f32 s5, s2']

    def fp(name, lines):
        bench(name, lines, setup)

    fp('vfma_vadd', ['vfma.f32 s3, s0, s1', 'vadd.f32 s4, s0, s2'])
    fp('vfma_vadd_use', ['vfma.f32 s3, s0, s1', 'vadd.f32 s4, s3, s2'])
    fp('vfma_nop_vadd', ['vfma.f32 s3, s0, s1', 'nop', 'vadd.f32 s4, s0, s2'])
    fp('vfma_nop_x2_vadd', ['vfma.f32 s3, s0, s1', 'nop', 'nop',
                            'vadd.f32 s4, s0, s2'])
    fp('vfma_nop_x3_vadd_use', ['vfma.f32 s3, s0, s1', 'nop', 'nop', 'nop',
                                'vadd.f32 s4, s3, s2'])
    fp('vfma_x3', ['vfma.f32 s3, s0, s1', 'vfma.f32 s4, s0, s2',
                   'vfma.f32 s5, s1, s2'])
    fp('vfma_vstr_use', ['vfma.f32 s3, s0, s1', 'vstr s3, [r4, #48]'])
    fp('vfma_vmov_core_use', ['vfma.f32 s3, s0, s1', 'vmov r0, s3'])
    fp('vadd_vfma_addend', ['vadd.f32 s3, s0, s1', 'vfma.f32 s3, s0, s2'])
    fp('vadd_vfma_operand', ['vadd.f32 s4, s0, s1', 'vfma.f32 s3, s4, s2'])
    fp('vfms', ['vfms.f32 s3, s0, s1'])
    fp('vnmul', ['vnmul.f32 s3, s0, s1'])
    fp('vmul_x2', ['vmul.f32 s3, s0, s1', 'vmul.f32 s4, s0, s2'])
    fp('vmul_chain_x2', ['vmul.f32 s3, s0, s1', 'vmul.f32 s4, s3, s2'])
    fp('vmul_nop_vadd_use', ['vmul.f32 s3, s0, s1', 'nop',
                             'vadd.f32 s4, s3, s2'])
    fp('vadd_adds_vadd_use', ['vadd.f32 s3, s0, s1', 'adds r0, r1',
                              'vadd.f32 s4, s3, s2'])
    fp('vsub_chain_x2', ['vsub.f32 s3, s0, s1', 'vsub.f32 s4, s3, s2'])
    fp('vadd_vcmp_use', ['vadd.f32 s3, s0, s1', 'vcmpe.f32 s3, s2'])
    fp('vadd_vcmp', ['vadd.f32 s3, s0, s1', 'vcmpe.f32 s0, s2'])
    fp('vadd_vneg_use', ['vadd.f32 s3, s0, s1', 'vneg.f32 s4, s3'])
    fp('vadd_vmov_use', ['vadd.f32 s3, s0, s1', 'vmov.f32 s4, s3'])
    fp('vabs_vadd_use', ['vabs.f32 s3, s0', 'vadd.f32 s4, s3, s2'])
    fp('vneg_vadd_use', ['vneg.f32 s3, s0', 'vadd.f32 s4, s3, s2'])
    fp('vmov_vadd_use', ['vmov.f32 s3, s0', 'vadd.f32 s4, s3, s2'])
    fp('vcvt_vadd_use', ['vcvt.f32.s32 s3, s0', 'vadd.f32 s4, s3, s2'])
    fp('vadd_vcvt_use', ['vadd.f32 s3, s0, s1', 'vcvt.s32.f32 s4, s3'])
    fp('vmov_from_core_vadd_use', ['vmov s3, r1', 'vadd.f32 s4, s3, s2'])
    fp('vldr_vmul_use', ['vldr s3, [r4, #32]', 'vmul.f32 s4, s3, s1'])
    fp('vldr_vstr', ['vldr s3, [r4, #32]', 'vstr s3, [r4, #48]'])
    fp('vcmp_vmrs_bcc', ['vcmpe.f32 s0, s1', 'vmrs APSR_nzcv, fpscr',
                         'bmi 1f', 'nop', '.balign 4', '1:'])
    fp('vcmp0_vmrs', ['vcmpe.f32 s0, #0', 'vmrs APSR_nzcv, fpscr'])
    fp('vdiv_nop_x16', ['vdiv.f32 s3, s0, s1'] + repeat('nop', 16))
    fp('vdiv_vstr_other', ['vdiv.f32 s3, s0, s1', 'vstr s0, [r4, #48]'])
    fp('vdiv_vldr_other', ['vdiv.f32 s3, s0, s1', 'vldr s4, [r4, #32]'])
    fp('vdiv_vmov_core_other', ['vdiv.f32 s3, s0, s1', 'vmov r0, s0'])
    fp('vdiv_vmov_reg_other', ['vdiv.f32 s3, s0, s1', 'vmov.f32 s4, s0'])
    fp('vdiv_vcmp_other', ['vdiv.f32 s3, s0, s1', 'vcmpe.f32 s0, s2'])
    fp('vdiv_vmrs', ['vdiv.f32 s3, s0, s1', 'vmrs APSR_nzcv, fpscr'])
    fp('vdiv_adds_x13_vadd',
       ['vdiv.f32 s3, s0, s1'] + repeat('adds r0, r1', 13) +
       ['vadd.f32 s4, s0, s2'])
    fp('vdiv_adds_x12_vadd',
       ['vdiv.f32 s3, s0, s1'] + repeat('adds r0, r1', 12) +
       ['vadd.f32 s4, s0, s2'])
    fp('vsqrt_vadd_use', ['vsqrt.f32 s3, s0', 'vadd.f32 s4, s3, s2'])
    fp('vdiv_x2', ['vdiv.f32 s3, s0, s1', 'vdiv.f32 s4, s0, s2'])
    fp('vdiv_by_one', ['vdiv.f32 s3, s0, s4', 'vadd.f32 s5, s0, s2'],
       )
    fp('vpush_2', ['vpush {d8}'])
    fp('_vdrop_2', ['add sp, #8'])
    fp('vpush_vpop_4', ['vpush {d8-d9}', 'vpop {d8-d9}'])

    def periph(name, address, lines):
        bench(name, lines, [f'ldr r8, ={address}'])

    periph('read_gpioa_idr_ahb2_x4', 0x48000010, repeat('ldr r0, [r8]', 4))
    periph('read_gpioa_idr_nop_x2', 0x48000010,
           ['ldr r0, [r8]', 'nop', 'ldr r3, [r8]'])
    periph('read_gpioa_idr_add_use', 0x48000010,
           ['ldr r0, [r8]', 'adds r3, r0, r1'])
    periph('read_tim2_cnt_apb1_x4', 0x40000024, repeat('ldr r0, [r8]', 4))
    periph('read_tim2_cnt_nop_x4_read', 0x40000024,
           ['ldr r0, [r8]'] + repeat('nop', 4) + ['ldr r3, [r8]'])
    periph('read_tim2_cnt_add_use', 0x40000024,
           ['ldr r0, [r8]', 'adds r3, r0, r1'])
    periph('write_gpioa_bsrr_ahb2_x2', 0x48000018,
           ['str r0, [r8]', 'str r0, [r8]'])
    periph('write_gpioa_bsrr_ahb2_x4', 0x48000018, repeat('str r0, [r8]', 4))
    periph('write_tim2_arr_nop_x4_write', 0x4000002c,
           ['str r1, [r8]'] + repeat('nop', 4) + ['str r2, [r8]'])
    periph('write_tim2_arr_read', 0x4000002c,
           ['str r1, [r8]', 'ldr r0, [r8]'])
    periph('write_tim2_arr_ldr_sram', 0x4000002c,
           ['str r1, [r8]', 'ldr r0, [r4]'])
    periph('read_cordic_csr_ahb1', 0x40020c00, ['ldr r0, [r8]'])
    periph('read_cordic_csr_ahb1_x2', 0x40020c00,
           ['ldr r0, [r8]', 'ldr r3, [r8]'])
    periph('read_adc1_isr_ahb2_x2', 0x50000000,
           ['ldr r0, [r8]', 'ldr r3, [r8]'])
    periph('read_nvic_iser_ppb', 0xe000e100, ['ldr r0, [r8]'])
    periph('read_nvic_iser_ppb_x2', 0xe000e100,
           ['ldr r0, [r8]', 'ldr r3, [r8]'])
    # Clearing the enable of an interrupt which is not enabled.
    periph('write_nvic_icer_ppb', 0xe000e180, ['str r1, [r8]'])
    periph('write_nvic_icer_ppb_x2', 0xe000e180,
           ['str r1, [r8]', 'str r1, [r8]'])
    periph('read_dwt_cyccnt_ppb', 0xe0001004, ['ldr r0, [r8]'])

    # Exceptions whose handler is entered directly from the vector.
    pend = ['ldr r8, =0xe000ed04', 'mov r9, #0x10000000']
    direct = ['ldr r0, =vectors', 'ldr r1, =plain_handler', 'str r1, [r0, #56]']
    bench('pendsv_direct', ['str r9, [r8]', 'isb'],
          ['bl clear_fp_context'] + direct + pend)
    bench('pendsv_direct_nop_x4', ['str r9, [r8]'] + repeat('nop', 4),
          ['bl clear_fp_context'] + direct + pend)
    bench('pendsv_direct_fp_context', ['str r9, [r8]', 'isb'],
          ['vmov.f32 s0, #1.0'] + direct + pend)
    bench('pendsv_direct_fp_used', ['str r9, [r8]', 'isb'],
          ['vmov.f32 s0, #1.0', 'ldr r0, =vectors', 'ldr r1, =fp_handler',
           'str r1, [r0, #56]'] + pend)
    bench('pendsv_direct_nops', ['str r9, [r8]', 'isb'],
          ['bl clear_fp_context', 'ldr r0, =vectors', 'ldr r1, =nop_handler',
           'str r1, [r0, #56]'] + pend)
    bench('pendsv_direct_push', ['str r9, [r8]', 'isb'],
          ['bl clear_fp_context', 'ldr r0, =vectors', 'ldr r1, =push_handler',
           'str r1, [r0, #56]'] + pend)
    bench('_restore_vector', [],
          ['ldr r0, =vectors', 'ldr r1, =pendsv', 'str r1, [r0, #56]',
           'bl clear_fp_context'])


def make_followups_2():
    '''A second round, mostly about data which shares a RAM with the
    code, as literal pools do.'''
    pool = ['1: .word 0x12345678', '2: .word 0x23456789']
    fpool = ['1: .float 1.5', '2: .float 2.5']

    def alternate(a, b, count):
        return [a if i % 2 == 0 else b for i in range(count)]

    for count in [3, 5, 6, 8]:
        bench(f'ldr_pool_x{count}',
              alternate('ldr r0, 1f', 'ldr r3, 2f', count), trailer=pool)
    for count in [2, 3, 4, 6]:
        bench(f'ldr.w_pool_x{count}',
              alternate('ldr.w r0, 1f', 'ldr.w r3, 2f', count), trailer=pool)
    for count in [3, 4, 6]:
        bench(f'vldr_pool_x{count}',
              alternate('vldr s3, 1f', 'vldr s4, 2f', count), trailer=fpool)
    code = ['ldr r8, =code_ram_data']
    for count in [3, 6]:
        bench(f'ldr_code_ram_x{count}',
              alternate('ldr.w r0, [r8]', 'ldr.w r3, [r8, #4]', count), code)
    bench('ldr_code_ram_narrow_x4',
          alternate('ldr r0, [r3]', 'ldr r2, [r3, #4]', 4),
          ['ldr r3, =code_ram_data', 'nop', 'nop'])
    bench('ldr_pool_add_x4', ['ldr r0, 1f', 'adds r2, r1, r1'] * 4,
          trailer=pool)
    bench('ldr_pool_add_x2', ['ldr r0, 1f', 'adds r2, r1, r1'] * 2,
          trailer=pool)
    setup = ['vldr s0, [r4, #32]', 'vldr s1, [r4, #36]']
    bench('vldr_pool_vmul_x3', ['vldr s3, 1f', 'vmul.f32 s4, s0, s1'] * 3,
          setup, trailer=fpool)
    bench('vldr_pool_vmul_use_x3', ['vldr s3, 1f', 'vmul.f32 s4, s3, s1'] * 3,
          setup, trailer=fpool)
    bench('ldr_pool_ldr_x2', ['ldr r0, 1f', 'ldr r3, [r4]'] * 2, trailer=pool)
    bench('ldr_ldr_pool', ['ldr r3, [r4]', 'ldr r0, 1f'], trailer=pool)
    bench('ldr_pool_nop_ldr_pool', ['ldr r0, 1f', 'nop', 'ldr r3, 2f'],
          trailer=pool)
    bench('ldr_pool_nop_x2_ldr_pool',
          ['ldr r0, 1f', 'nop', 'nop', 'ldr r3, 2f'], trailer=pool)
    bench('ldr_pool_str', ['ldr r0, 1f', 'str r0, [r4]'], trailer=pool)
    bench('str_ldr_pool', ['str r1, [r4]', 'ldr r0, 1f'], trailer=pool)
    bench('ldr_pool_b', ['ldr r0, 1f', 'b 3f', 'nop', '.balign 4', '3:'],
          trailer=pool)
    bench('adds_ldr_pool', ['adds r2, r1, r1', 'ldr r0, 1f'], trailer=pool)
    bench('ldr_pool_x2_add_use', ['ldr r0, 1f', 'ldr r3, 2f',
                                  'adds r2, r0, r3'], trailer=pool)
    bench('ldr_pool_unaligned_start', ['nop', 'ldr r0, 1f'], trailer=pool)
    bench('ldr_pool_x2_unaligned_start', ['nop', 'ldr r0, 1f', 'ldr r3, 2f'],
          trailer=pool)

    # What else, besides a NOP, runs during the second cycle of a
    # load?
    bench('ldr_adds_unrelated', ['ldr r0, [r4]', 'adds r2, r1, r1'])
    bench('ldr_cmp_unrelated', ['ldr r0, [r4]', 'cmp r1, r1'])
    bench('ldr_mov_unrelated', ['ldr r0, [r4]', 'mov r8, r8'])
    bench('ldr_movs_imm', ['ldr r0, [r4]', 'movs r2, #1'])
    bench('ldr_nop.w', ['ldr r0, [r4]', 'nop.w'])
    bench('ldr_nop_nop', ['ldr r0, [r4]', 'nop', 'nop'])
    bench('ldr_it_skipped', ['cmp r1, r1', 'ldr r0, [r4]', 'it ne',
                             'addne r2, r1'])
    bench('ldr_bcc_not_taken', ['cmp r1, r1', 'ldr r0, [r4]', 'bne 1f',
                                '1:'])
    bench('ldr_vadd', ['ldr r0, [r4]', 'vadd.f32 s3, s0, s1'], setup)
    bench('vldr_adds', ['vldr s3, [r4, #32]', 'adds r2, r1, r1'], setup)
    bench('ldrd_nop', ['ldrd r0, r3, [r4]', 'nop'])
    bench('pop_nop', ['pop {r0}', 'nop'], ['push {r0}'])
    bench('str_reg_ldr', ['str r0, [r4, r1]', 'ldr r3, [r4]'])
    bench('str_reg_str', ['str r0, [r4, r1]', 'str r3, [r4]'])
    bench('str_reg_adds_x2', ['str r0, [r4, r1]', 'adds r2, r1, r1'] * 2)

    for name, dividend, divisor in [
            ('1_by_2', 1, 2), ('3_by_2', 3, 2), ('2_by_3', 2, 3),
            ('0_by_0', 0, 0)]:
        bench(f'udiv_{name}', ['udiv r0, r8, r9'],
              [f'ldr r8, ={dividend}', f'ldr r9, ={divisor}'])
    for name, dividend, divisor in [
            ('7_by_8', 7, 8), ('minus7_by_8', -7, 8), ('7_by_minus8', 7, -8),
            ('0_by_5', 0, 5), ('5_by_0', 5, 0), ('1_by_1', 1, 1),
            ('8_by_7', 8, 7), ('minus8_by_7', -8, 7),
            ('max_by_1', 0x7fffffff, 1), ('minus256_by_1', -256, 1),
            ('minus1_by_minus1', -1, -1), ('1_by_minus1', 1, -1),
            ('3_by_0x10000', 3, 0x10000),
            ('minus3_by_0x10000', -3, 0x10000)]:
        bench(f'sdiv_{name}', ['sdiv r0, r8, r9'],
              [f'ldr r8, ={dividend}', f'ldr r9, ={divisor}'])


def make_followups_3():
    target = ['adr r8, 1f', 'add r8, r8, #1', 'str r8, [r4, #20]']
    # A branch to a register which was only just loaded.
    bench('ldr_bx_use', ['ldr r0, [r4, #20]', 'bx r0', 'nop', '.balign 4',
                         '1:'], target)
    bench('ldr_nop_bx_use', ['ldr r0, [r4, #20]', 'nop', 'bx r0', 'nop',
                             '.balign 4', '1:'], target)
    bench('mov_bx_use', ['mov r0, r8', 'bx r0', 'nop', '.balign 4', '1:'],
          ['adr r8, 1f', 'add r8, r8, #1'])
    bench('ldr_pool_bx_use', ['ldr r0, 2f', 'bx r0', 'nop', '.balign 4',
                              '1:'], trailer=['2: .word 1b + 1'])
    bench('b_next', ['b 1f', '1:'])
    bench('bl_next', ['bl 1f', '1:'])

    pend = ['ldr r8, =0xe000ed04', 'mov r9, #0x10000000']

    def handler(name):
        return ['bl clear_fp_context', 'ldr r0, =vectors',
                f'ldr r1, ={name}', 'str r1, [r0, #56]'] + pend

    bench('pendsv_direct_pool_load', ['str r9, [r8]', 'isb'],
          handler('pool_load_handler'))
    bench('pendsv_direct_nop_pool_load', ['str r9, [r8]', 'isb'],
          handler('nop_pool_load_handler'))
    bench('pendsv_direct_sram_load', ['str r9, [r8]', 'isb'],
          handler('sram_load_handler'))
    bench('pendsv_direct_bx_reg', ['str r9, [r8]', 'isb'],
          ['ldr r10, =plain_handler_2'] +
          handler('bx_reg_handler'))
    bench('pendsv_direct_adds', ['str r9, [r8]', 'isb'],
          handler('adds_handler'))
    bench('_restore_vector_2', [],
          ['ldr r0, =vectors', 'ldr r1, =pendsv', 'str r1, [r0, #56]',
           'bl clear_fp_context'])


def make_memory_banks():
    '''The same accesses, to each of the different RAMs.'''
    banks = [
        ('sram1', 0x20000300),
        ('sram1_high', 0x20013f00),
        ('sram2', 0x20016000),
        ('ccm_alias', 0x2001c000),
        ('ccm', CODE_RAM_DATA),
    ]
    for name, address in banks:
        setup = [f'ldr r8, ={address}', 'movs r0, #0', 'str r0, [r8]',
                 'str r0, [r8, #4]', 'nop', 'nop', 'nop', 'nop']

        def at(test, lines, setup=setup, name=name):
            bench(f'{test}@{name}', lines, setup)

        at('ldr', ['ldr.w r0, [r8]'])
        at('ldr_x2', ['ldr.w r0, [r8]', 'ldr.w r3, [r8, #4]'])
        at('ldr_x4', ['ldr.w r0, [r8]', 'ldr.w r3, [r8, #4]'] * 2)
        at('ldr_add', ['ldr.w r0, [r8]', 'adds r2, r1, r1'])
        at('ldr_add_x4', ['ldr.w r0, [r8]', 'adds r2, r1, r1'] * 4)
        at('str', ['str.w r1, [r8]'])
        at('str_x2', ['str.w r1, [r8]', 'str.w r2, [r8, #4]'])
        at('str_x4', ['str.w r1, [r8]', 'str.w r2, [r8, #4]'] * 2)
        at('str_add_x4', ['str.w r1, [r8]', 'adds r2, r1, r1'] * 4)
        at('ldr_str', ['ldr.w r0, [r8]', 'str.w r0, [r8, #4]'])
        at('str_ldr', ['str.w r1, [r8]', 'ldr.w r0, [r8, #4]'])
        at('vldr', ['vldr s3, [r8]'])
        at('vldr_vadd', ['vldr s3, [r8]', 'vadd.f32 s4, s3, s3'])
        at('vstr', ['vstr s0, [r8]'])
        at('vldr_vstr', ['vldr s3, [r8]', 'vstr s3, [r8, #4]'])
        at('ldrd', ['ldrd r0, r3, [r8]'])
        at('ldm_2', ['ldmia.w r8, {r0, r3}'])


def make_mixed_banks():
    '''Accesses which alternate between different memories, as real
    code does.'''
    banks = [
        ('sram1', 0x20000300),
        ('sram2', 0x20016000),
        ('ccm', CODE_RAM_DATA),
        # GPIOA_IDR, then its unused upper neighbours.
        ('ahb', 0x48000010),
        # TIM2_CNT.
        ('apb', 0x40000024),
        # SCB_ICSR.
        ('ppb', 0xe000ed04),
    ]
    writable = ['sram1', 'sram2', 'ccm']
    for first, first_address in banks:
        for second, second_address in banks:
            if first == second:
                continue
            setup = [f'ldr r8, ={first_address}', f'ldr r9, ={second_address}',
                     'nop', 'nop', 'nop', 'nop']
            name = f'{first}_{second}'
            bench(f'ldr_ldr@{name}',
                  ['ldr.w r0, [r8]', 'ldr.w r3, [r9]'], setup)
            if second in writable:
                bench(f'ldr_str@{name}',
                      ['ldr.w r0, [r8]', 'str.w r1, [r9]'], setup)
            if first in writable:
                bench(f'str_ldr@{name}',
                      ['str.w r1, [r8]', 'ldr.w r3, [r9]'], setup)
            if first in writable and second in writable:
                bench(f'str_str@{name}',
                      ['str.w r1, [r8]', 'str.w r2, [r9]'], setup)
    # Longer alternations, and with something in between.
    for first, second in [('sram1', 'sram2'), ('sram2', 'ahb'),
                          ('sram1', 'ahb'), ('sram2', 'ccm')]:
        addresses = dict(banks)
        setup = [f'ldr r8, ={addresses[first]}', f'ldr r9, ={addresses[second]}',
                 'nop', 'nop', 'nop', 'nop']
        name = f'{first}_{second}'
        bench(f'ldr_x4_alternating@{name}',
              ['ldr.w r0, [r8]', 'ldr.w r3, [r9]'] * 2, setup)
        bench(f'ldr_add_ldr@{name}',
              ['ldr.w r0, [r8]', 'adds r2, r1, r1', 'ldr.w r3, [r9]'], setup)
        bench(f'ldr_add_x2_ldr@{name}',
              ['ldr.w r0, [r8]', 'adds r2, r1, r1', 'adds r2, r1, r1',
               'ldr.w r3, [r9]'], setup)
    # The stack is in SRAM1, and what is saved there often came from
    # or goes to SRAM2.
    setup = ['ldr r8, =0x20016000', 'nop', 'nop', 'nop', 'nop']
    bench('push_ldr@sram2', ['push {r0, r1}', 'ldr.w r3, [r8]'], setup)
    bench('_drop_mixed', ['add sp, #8'])
    bench('ldr_push@sram2', ['ldr.w r3, [r8]', 'push {r0, r1}'], setup)
    bench('_drop_mixed_2', ['add sp, #8'])
    bench('ldr_sp_ldr@sram2', ['ldr r0, [sp, #0]', 'ldr.w r3, [r8]'], setup)
    bench('ldr_ldr_sp@sram2', ['ldr.w r3, [r8]', 'ldr r0, [sp, #0]'], setup)
    bench('str_sp_ldr@sram2', ['str r0, [sp, #0]', 'ldr.w r3, [r8]'], setup)
    bench('vldr_vldr_sp@sram2', ['vldr s3, [r8]', 'vldr s4, [sp]'], setup)
    bench('ldr_b_ldr@sram2', ['ldr.w r0, [r8]', 'b 1f', '.balign 4',
                              '1: ldr.w r3, [r8, #4]'], setup)
    bench('ldr_x2_b_ldr_x2@sram2',
          ['ldr.w r0, [r8]', 'ldr.w r3, [r8, #4]', 'b 1f', '.balign 4',
           '1: ldr.w r0, [r8]', 'ldr.w r3, [r8, #4]'], setup)


def make_calls():
    '''Function calls and returns as compilers write them, with the
    callee and the return address at each alignment.'''
    sram2 = ['ldr r8, =0x20016000', 'nop', 'nop', 'nop', 'nop']

    def call(name, pad_call, pad_callee, callee, pad_return=()):
        # The call is at an offset of len(pad_call) halfwords, and the
        # callee and the instruction returned to are likewise placed.
        bench(name,
              list(pad_call) + ['bl 1f'] + list(pad_return) + ['b 2f',
               '.balign 4'] + list(pad_callee) + ['1:'] + callee +
              ['.balign 4', 'nop', '2:'], sram2)

    for align, pad in [('aligned', []), ('unaligned', ['nop'])]:
        call(f'call_bx_lr_{align}', [], pad, ['bx lr'])
        call(f'call_push_pop_{align}', [], pad,
             ['push {r4, lr}', 'pop {r4, pc}'])
        call(f'call_push5_pop5_{align}', [], pad,
             ['push {r4, r5, r6, r7, lr}', 'pop {r4, r5, r6, r7, pc}'])
        call(f'call_push.w_pop.w_{align}', [], pad,
             ['push.w {r4, r8, r9, lr}', 'pop.w {r4, r8, r9, pc}'])
        call(f'call_ldr_sram2_{align}', [], pad,
             ['ldr.w r0, [r8]', 'bx lr'])
        call(f'call_ldr_narrow_{align}', [], pad,
             ['ldr r0, [r4]', 'bx lr'])
        call(f'call_adds_{align}', [], pad, ['adds r0, r1, r2', 'bx lr'])
        call(f'call_add.w_{align}', [], pad, ['add.w r0, r1, r2', 'bx lr'])
        call(f'call_vldr_{align}', [], pad, ['vldr s3, [r8]', 'bx lr'])
        call(f'call_push_vpush_{align}', [], pad,
             ['push {r4, lr}', 'vpush {d8}', 'vpop {d8}', 'pop {r4, pc}'])
        # The return lands on an instruction at each alignment.
        call(f'return_to_wide_{align}', pad, [], ['bx lr'], ['nop.w'])
        call(f'return_to_ldr_{align}', pad, [], ['bx lr'],
             ['ldr.w r0, [r8]'])
        call(f'pop_return_to_wide_{align}', pad, [],
             ['push {r4, lr}', 'pop {r4, pc}'], ['nop.w'])
        call(f'pop_return_to_ldr_{align}', pad, [],
             ['push {r4, lr}', 'pop {r4, pc}'], ['ldr.w r0, [r8]'])
        call(f'pop_return_to_narrow_ldr_{align}', pad, [],
             ['push {r4, lr}', 'pop {r4, pc}'], ['ldr r0, [r4]'])

    # A branch whose target is a load, or which follows one.
    for align, pad in [('aligned', []), ('unaligned', ['nop'])]:
        bench(f'b_to_ldr_sram2_{align}',
              ['b 1f', '.balign 4'] + pad + ['1: ldr.w r0, [r8]'], sram2)
        bench(f'b_to_ldr_narrow_{align}',
              ['b 1f', '.balign 4'] + pad + ['1: ldr r0, [r4]'], sram2)
        bench(f'bcc_to_ldr_sram2_{align}',
              ['cmp r1, r1', 'beq 1f', '.balign 4'] + pad +
              ['1: ldr.w r0, [r8]'], sram2)
        bench(f'b_to_str_sram2_{align}',
              ['b 1f', '.balign 4'] + pad + ['1: str.w r1, [r8]'], sram2)
        bench(f'b_to_vadd_{align}',
              ['b 1f', '.balign 4'] + pad + ['1: vadd.f32 s3, s0, s1'], sram2)
        bench(f'b_to_push_{align}',
              ['b 1f', '.balign 4'] + pad + ['1: push {r0, r1}'], sram2)
        bench(f'_drop_b_to_push_{align}', ['add sp, #8'])
    bench('ldr_sram2_bcc_not_taken',
          ['cmp r1, r1', 'ldr.w r0, [r8]', 'bne 1f', '1:'], sram2)
    bench('ldr_sram2_cmp_bcc_taken',
          ['ldr.w r0, [r8]', 'cmp r0, r0', 'beq 1f', 'nop', '.balign 4',
           '1:'], sram2)
    bench('ldr_sram2_cbz_taken',
          ['ldr.w r0, [r8]', 'cbz r0, 1f', 'nop', '.balign 4', '1:'], sram2)
    bench('ldr_sram2_bl_bx',
          ['ldr.w r0, [r8]', 'bl 1f', 'b 2f', '.balign 4', '1: bx lr',
           '.balign 4', '2:'], sram2)
    # Straight line code with 32 bit instructions at each alignment.
    bench('wide_x4_aligned', ['add.w r0, r1, r2'] * 4)
    bench('wide_x4_unaligned', ['nop'] + ['add.w r0, r1, r2'] * 4)
    bench('ldr.w_sram2_x4_unaligned',
          ['nop'] + ['ldr.w r0, [r8]', 'ldr.w r3, [r8, #4]'] * 2, sram2)
    bench('ldr.w_add_x4_unaligned',
          ['nop'] + ['ldr.w r0, [r8]', 'adds r2, r1, r1'] * 4, sram2)
    bench('ldr.w_add.w_x4_sram2',
          ['ldr.w r0, [r8]', 'add.w r2, r1, r1'] * 4, sram2)
    bench('ldr.w_add.w_x4_sram2_unaligned',
          ['nop'] + ['ldr.w r0, [r8]', 'add.w r2, r1, r1'] * 4, sram2)
    bench('vldr_vmul_x4_sram2',
          ['vldr s3, [r8]', 'vmul.f32 s4, s0, s1'] * 4, sram2)
    bench('vldr_vmul_x4_sram2_unaligned',
          ['nop'] + ['vldr s3, [r8]', 'vmul.f32 s4, s0, s1'] * 4, sram2)


def make_long_runs():
    '''Long stretches of code, which unlike the short sequences above
    give the instruction fetch no slack to hide behind.'''
    sram2 = ['ldr r8, =0x20016000', 'vldr s0, [r4, #32]',
             'vldr s1, [r4, #36]', 'nop', 'nop', 'nop', 'nop']
    for count in [8, 16, 32, 64]:
        bench(f'adds_x{count}', ['adds r0, r1, r2'] * count)
        bench(f'add.w_x{count}', ['add.w r0, r1, r2'] * count)
        bench(f'add.w_x{count}_unaligned',
              ['nop'] + ['add.w r0, r1, r2'] * count)
        bench(f'vadd_x{count}', ['vadd.f32 s3, s0, s1'] * count, sram2)
        bench(f'vadd_x{count}_unaligned',
              ['nop'] + ['vadd.f32 s3, s0, s1'] * count, sram2)
    for count in [4, 8, 16]:
        # Typical floating point code: load, compute, store.
        body = ['vldr s3, [r8]', 'vmul.f32 s4, s0, s1',
                'vadd.f32 s5, s0, s1', 'vstr s5, [r8, #4]']
        bench(f'fp_block_x{count}', body * count, sram2)
        bench(f'fp_block_x{count}_unaligned', ['nop'] + body * count, sram2)
        body = ['ldr.w r0, [r8]', 'add.w r2, r1, r1', 'add.w r3, r1, r1',
                'str.w r2, [r8, #4]']
        bench(f'int_block_x{count}', body * count, sram2)
        bench(f'int_block_x{count}_unaligned', ['nop'] + body * count, sram2)
        body = ['ldr r0, [r4]', 'adds r2, r1, r1', 'adds r3, r1, r1',
                'str r2, [r4, #4]']
        bench(f'narrow_block_x{count}', body * count, sram2)
        # A mixture of widths.
        body = ['ldr.w r0, [r8]', 'adds r2, r1, r1', 'vadd.f32 s5, s0, s1',
                'cmp r1, r1', 'vmul.f32 s4, s0, s1', 'str r2, [r4, #4]']
        bench(f'mixed_block_x{count}', body * count, sram2)
        # With a literal from the RAM being executed from in each.
        body = ['vldr s3, 1f', 'vmul.f32 s4, s0, s1', 'vadd.f32 s5, s0, s1',
                'vstr s5, [r8, #4]']
        bench(f'fp_literal_block_x{count}', body * count, sram2,
              trailer=['1: .float 1.5'])
        # With a taken branch in each.
        body = ['vldr s3, [r8]', 'vmul.f32 s4, s0, s1', 'b 7f', 'nop',
                '.balign 4', '7: vadd.f32 s5, s0, s1', 'vstr s5, [r8, #4]']
        bench(f'fp_branch_block_x{count}', body * count, sram2)


def make_fetch():
    '''What it is about long stretches of code which costs more than
    the instructions in them do individually.'''
    sram2 = ['ldr r8, =0x20016000', 'vldr s0, [r4, #32]',
             'vldr s1, [r4, #36]', 'nop', 'nop', 'nop', 'nop']

    def both(name, body, count=12, trailer=()):
        bench(f'{name}_x{count}', body * count, sram2, trailer)
        bench(f'{name}_x{count}_unaligned', ['nop'] + body * count, sram2,
              trailer)

    for name, line in [
            ('run_vldr_sram2', 'vldr s3, [r8]'),
            ('run_vldr', 'vldr s3, [r4]'),
            ('run_vstr_sram2', 'vstr s0, [r8, #4]'),
            ('run_vstr', 'vstr s0, [r4, #4]'),
            ('run_ldr.w_sram2', 'ldr.w r0, [r8]'),
            ('run_ldr.w', 'ldr.w r0, [r4]'),
            ('run_str.w_sram2', 'str.w r1, [r8, #4]'),
            ('run_str.w', 'str.w r1, [r4, #4]'),
            ('run_vmul', 'vmul.f32 s4, s0, s1'),
            ('run_vmov', 'vmov.f32 s4, s0'),
            ('run_vmov_core', 'vmov r0, s0'),
            ('run_vcmp', 'vcmp.f32 s0, s1'),
            ('run_vneg', 'vneg.f32 s4, s0'),
            ('run_mul.w', 'mul.w r0, r1, r2'),
            ('run_ubfx', 'ubfx r0, r1, #1, #2'),
    ]:
        both(name, [line])

    LDR, STR = 'ldr.w r0, [r8]', 'str.w r2, [r8, #4]'
    VLDR, VSTR = 'vldr s3, [r8]', 'vstr s5, [r8, #4]'
    ADD, VADD, VMUL = ('add.w r2, r1, r1', 'vadd.f32 s5, s0, s1',
                       'vmul.f32 s4, s0, s1')
    for name, body in [
            ('blk_vldr', [VLDR, ADD, ADD, STR]),
            ('blk_vstr', [LDR, ADD, ADD, VSTR]),
            ('blk_vldr_vstr', [VLDR, ADD, ADD, VSTR]),
            ('blk_fp_mid', [LDR, VMUL, VADD, STR]),
            ('blk_fp_first', [LDR, VMUL, ADD, STR]),
            ('blk_fp_last', [LDR, ADD, VADD, STR]),
            ('blk_ldr_add3', [LDR, ADD, ADD, ADD]),
            ('blk_add3_str', [ADD, ADD, ADD, STR]),
            ('blk_ldr_str', [LDR, STR]),
            ('blk_ldr_add', [LDR, ADD]),
            ('blk_add_str', [ADD, STR]),
            ('blk_ldr_add7', [LDR] + [ADD] * 7),
            ('blk_vldr_vadd3', [VLDR, VADD, VADD, VADD]),
            ('blk_vadd3_vstr', [VADD, VADD, VADD, VSTR]),
            ('blk_vadd_add', [VADD, ADD]),
            ('blk_vadd2_add', [VADD, VADD, ADD]),
            ('blk_vadd_add2', [VADD, ADD, ADD]),
            ('blk_vadd3_add', [VADD, VADD, VADD, ADD]),
            ('blk_vadd_nop2', [VADD, 'nop', 'nop']),
            ('blk_vadd_nop', [VADD, 'nop']),
            ('blk_add_nop', [ADD, 'nop']),
            ('blk_ldr_nop', [LDR, 'nop']),
            ('blk_vldr_nop', [VLDR, 'nop']),
            ('blk_fp_sram1', ['vldr s3, [r4]', VMUL, VADD,
                              'vstr s5, [r4, #4]']),
            ('blk_int_sram1', ['ldr.w r0, [r4]', ADD, ADD,
                               'str.w r2, [r4, #4]']),
    ]:
        both(name, body)


def make_prefixes():
    '''Every leading part of several sequences which do not start on
    a word boundary, to show exactly which instruction takes longer.'''
    sram2 = ['ldr r8, =0x20016000', 'vldr s0, [r4, #32]',
             'vldr s1, [r4, #36]', 'nop', 'nop', 'nop', 'nop']
    V, A = 'vadd.f32 s5, s0, s1', 'add.w r2, r1, r1'
    L, S = 'ldr.w r0, [r8]', 'str.w r2, [r8, #4]'
    L1, S1 = 'ldr.w r0, [r4]', 'str.w r2, [r4, #4]'
    for name, sequence in [
            ('v', [V] * 14),
            ('va', [V, A] * 7),
            ('av', [A, V] * 7),
            ('vaa', [V, A, A] * 5),
            ('vva', [V, V, A] * 5),
            ('laaa', [L, A, A, A] * 4),
            ('la', [L, A] * 6),
            ('as', [A, S] * 7),
            ('l', [L] * 10),
            ('l1', [L1] * 10),
            ('s1', [S1] * 10),
            ('a4v', [A] * 4 + [V] * 10),
            ('fp', ['vldr s3, [r8]', 'vmul.f32 s4, s0, s1', V,
                    'vstr s0, [r8, #4]'] * 4),
            ('a', [A] * 8),
            ('vn', [V, 'nop', 'nop', V, V, 'nop', 'nop', V, V, V, V, V]),
    ]:
        for count in range(1, len(sequence) + 1):
            bench(f'prefix_{name}_{count}', ['nop'] + sequence[:count], sram2)


def make_prefixes_2():
    '''More of the same: stores of what was just calculated, which
    instructions it is that hold up those after them, and whether a
    branch changes that.'''
    sram2 = ['ldr r8, =0x20016000', 'vldr s0, [r4, #32]',
             'vldr s1, [r4, #36]', 'nop', 'nop', 'nop', 'nop']
    V, A = 'vadd.f32 s5, s0, s1', 'add.w r2, r1, r1'

    def prefixes(name, sequence, lead=()):
        for count in range(1, len(sequence) + 1):
            bench(f'prefix_{name}_{count}', list(lead) + sequence[:count],
                  sram2)

    # Aligned: a store of what was just calculated.
    prefixes('fpblock', ['vldr s3, [r4]', 'vmul.f32 s4, s0, s1', V,
                         'vstr s5, [r4, #4]'] * 3)
    prefixes('v3vstr', [V, V, V, 'vstr s5, [r4, #4]'] * 3)
    prefixes('v3vstr_sram2', [V, V, V, 'vstr s5, [r8, #4]'] * 3)
    prefixes('vstr_vldr', ['vstr s0, [r4, #4]', 'vldr s3, [r4]'] * 4)
    prefixes('ldr_str_sram2', ['ldr.w r0, [r8]', 'str.w r2, [r8, #4]'] * 4)
    prefixes('ldr_str', ['ldr.w r0, [r4]', 'str.w r2, [r4, #4]'] * 4)

    # Which instructions it is that are affected.
    for name, line in [
            ('movw', 'movw r0, #0x1234'),
            ('cmp.w', 'cmp.w r1, #0x10000'),
            ('addw', 'addw r0, r1, #0x123'),
            ('lsl.w', 'lsl.w r0, r1, r2'),
            ('bfi', 'bfi r0, r1, #2, #3'),
            ('sxth.w', 'sxth.w r0, r1'),
            ('nop.w', 'nop.w'),
            ('ldrb.w', 'ldrb.w r0, [r4]'),
            ('strb.w', 'strb.w r1, [r4, #4]'),
            ('ldr_literal', 'ldr.w r0, 9f'),
            ('vldr_literal', 'vldr s3, 9f'),
            ('mla', 'mla r0, r1, r2, r3'),
            ('umull', 'umull r0, r9, r1, r2'),
            ('udiv', 'udiv r0, r2, r1'),
            ('vmov_to_fp', 'vmov s3, r1'),
            ('vmrs', 'vmrs APSR_nzcv, fpscr'),
            ('vcvt', 'vcvt.s32.f32 s3, s0'),
            ('vsqrt', 'vsqrt.f32 s3, s0'),
            ('vfma', 'vfma.f32 s3, s0, s1'),
            ('ldrd', 'ldrd r0, r9, [r4]'),
            ('mrs', 'mrs r0, apsr'),
    ]:
        trailer = ['.balign 4', '9: .word 0x3fc00000']
        bench(f'run_{name}_x8', ['movs r0, #1'] + [line] * 8, sram2, trailer)
        bench(f'run_{name}_x8_unaligned', ['movs r0, #1', 'nop'] + [line] * 8,
              sram2, trailer)

    # Just after a branch, when nothing has been fetched ahead.
    to_unaligned = ['b 1f', 'nop', '.balign 4', 'nop', '1:']
    to_aligned = ['b 1f', 'nop', '.balign 4', '1:']
    prefixes('b_unaligned_v', [V] * 10, to_unaligned)
    prefixes('b_aligned_v', [V] * 10, to_aligned)
    prefixes('b_aligned_nop_v', ['nop'] + [V] * 10, to_aligned)
    prefixes('b_unaligned_a', [A] * 6, to_unaligned)
    prefixes('b_unaligned_av', [A, V] * 5, to_unaligned)
    prefixes('b_unaligned_s', ['str.w r2, [r4, #4]'] * 8, to_unaligned)
    prefixes('b_aligned_l', ['ldr.w r0, [r4]'] * 8, to_aligned)
    prefixes('b_unaligned_l', ['ldr.w r0, [r4]'] * 8, to_unaligned)
    prefixes('b_unaligned_va2', [V, A, A] * 4, to_unaligned)


def make_prefixes_3():
    '''Branches which are not taken.'''
    sram2 = ['ldr r8, =0x20016000', 'vldr s0, [r4, #32]',
             'vldr s1, [r4, #36]', 'nop', 'nop', 'nop', 'nop']
    V, A = 'vadd.f32 s5, s0, s1', 'add.w r2, r1, r1'
    BN, BW, CB = 'beq.n 9f', 'beq.w 9f', 'cbz r1, 9f'
    trailer = ['9: nop']

    def prefixes(name, sequence):
        for count in range(1, len(sequence) + 1):
            bench(f'prefix_{name}_{count}', sequence[:count], sram2, trailer)

    prefixes('nt_cbz_run', [CB] * 6)
    prefixes('nt_n_nop_a', [BN, 'nop'] + [A] * 5)
    prefixes('nt_n_a', [BN] + [A] * 6)
    prefixes('nt_w_a', [BW] + [A] * 5)
    prefixes('nt_w_a_unaligned', ['nop', BW] + [A] * 5)
    prefixes('nt_cmp_n_a2', ['cmp r1, #0', BN, A, A] * 3)
    prefixes('nt_cmp_n_adds2', ['cmp r1, #0', BN, 'adds r2, r1, r1',
                                'adds r2, r1, r1'] * 3)
    prefixes('nt_cmp_nop_w_a2', ['cmp r1, #0', 'nop', BW, A, A] * 3)
    prefixes('nt_cbz_a', [CB] + [A] * 5)
    prefixes('nt_cbz_nop_a', [CB, 'nop'] + [A] * 5)
    prefixes('nt_n_nop_v', [BN, 'nop'] + [V] * 6)
    prefixes('nt_n_v', [BN] + [V] * 7)
    prefixes('nt_a_n_a', [A, A, BN, 'nop', A, A, BN, 'nop', A, A])
    prefixes('nt_n_ldr', [BN, 'ldr r0, [r4]', BN, 'ldr r0, [r4]', BN,
                          'ldr r0, [r4]'])
    # Whether a narrow load or store has the effect on what follows
    # that a wide one does.
    prefixes('l16_a', ['ldr r0, [r4]'] + [A] * 6)
    prefixes('s16_a', ['str r1, [r4, #4]'] + [A] * 6)
    prefixes('v_nop_a', [V, 'nop'] + [A] * 6)
    prefixes('msr', ['nop'] + ['msr apsr_nzcvq, r0'] * 8)
    prefixes('ldr_literal', ['nop'] + ['ldr.w r0, 8f'] * 8)
    benchmarks[-1][3].extend(['.balign 4', '8: .word 0x3fc00000'])
    for b in benchmarks[-8:]:
        if '8:' not in ''.join(b[3]):
            b[3].extend(['.balign 4', '8: .word 0x3fc00000'])


def make_divided_apb():
    '''The firmware runs both peripheral buses at half the speed of
    the processor, which makes an access take a different time
    depending on which cycle it starts in.'''
    SPI1, TIM2 = 0x40013000, 0x40000000
    spi_setup = [
        # Clock SPI1.
        'ldr r0, =0x40021060', 'ldr r1, [r0]', 'orr r1, r1, #0x1000',
        'str r1, [r0]',
        # The receiver is clocked by what comes back from the clock
        # pin, PA5, so that has to be connected.
        'ldr r0, =0x48000020', 'ldr r1, [r0]', 'bic r1, r1, #0x00f00000',
        'orr r1, r1, #0x00500000', 'str r1, [r0]',
        'ldr r0, =0x48000000', 'ldr r1, [r0]', 'bic r1, r1, #0xc00',
        'orr r1, r1, #0x800', 'str r1, [r0]',
        # A master, at 1/8 of the bus clock, with 16 bit words.
        f'ldr r8, =0x{SPI1:08x}', 'movw r0, #0x0314', 'str r0, [r8]',
        'movw r0, #0x0f00', 'str r0, [r8, #4]',
        f'ldr r9, =0x{TIM2:08x}', 'movs r1, #4',
        'nop', 'nop', 'nop', 'nop',
    ]
    # Only once, so that everything after has the same relationship
    # to the bus clocks.
    divide = [
        'ldr r0, =0x40021008', 'ldr r1, [r0]', 'tst r1, #0x2000', 'bne 7f',
        'orr r1, r1, #0x2400', 'str r1, [r0]', '7:',
    ]
    # Two writes in a row leave the processor in step with the bus
    # clock, whatever came before, since the second waits for the
    # first.  What follows is then timed from a known point.  What
    # follows that gives the bridge time to finish, as otherwise
    # reading the cycle counter would have to wait for it.
    W, R = 'str r0, [r8, #4]', 'ldr r2, [r8, #8]'
    ADD = 'adds r3, r1, r1'
    tests = [
        ('read', [R]),
        ('write', [W]),
        ('write_x2', [W, W]),
        ('read_x2', [R, R]),
        ('read_write', [R, W]),
        ('write_read', [W, R]),
        ('read_add_write', [R, ADD, W]),
        ('read_apb1', ['ldr r2, [r9, #0x24]']),
        ('read_apb1_read', ['ldr r2, [r9, #0x24]', R]),
        ('read_sram_read', ['ldr r2, [r4]', R]),
        ('read_read_sram', [R, 'ldr r2, [r4]']),
        ('write_read_sram', [W, 'ldr r2, [r4]']),
        ('write_write_sram', [W, 'str r2, [r4, #4]']),
        ('write_add_read_sram', [W, ADD, 'ldr r2, [r4]']),
        ('write_x3', [W, W, W]),
        ('read_x3', [R, R, R]),
        ('read_test_write', [R, 'lsls r2, r2, #31', 'bmi 7f', '7:', W]),
        ('none', []),
        ('write_add_write', [W, ADD, W]),
        ('write_add2_write', [W, ADD, ADD, W]),
        ('write_add_read', [W, ADD, R]),
        ('write_add2_read', [W, ADD, ADD, R]),
        ('write_add4_read', [W, ADD, ADD, ADD, ADD, R]),
        ('read_dwt', ['ldr r2, [r6]']),
        ('write_read_dwt', [W, 'ldr r2, [r6]']),
    ]

    def probe(spin, delay):
        # Start a transfer, wait, and then look once to see whether
        # it has finished, taking longer if it has.
        return ([
            'str r1, [r8, #12]', 'ldr r2, [r8]', 'orr r2, r2, #0x40',
            'str r2, [r8]', f'movs r2, #{spin}', '6: subs r2, #1', 'bne 6b',
        ] + [ADD] * delay + [
            'ldr r2, [r8, #8]', 'lsls r2, r2, #31', 'bpl 7f',
        ] + [ADD] * 8 + [
            '7:',
            '5: ldr r2, [r8, #8]', 'lsls r2, r2, #31', 'bpl 5b',
            'ldr r2, [r8, #12]', 'ldr r2, [r8]', 'bic r2, r2, #0x40',
            'str r2, [r8]'])

    for suffix, setup, spin in [('', spi_setup, 40),
                                ('_div2', divide + spi_setup, 84)]:
        for delay in range(8):
            for name, lines in tests:
                bench(f'apb_{name}{suffix}_after{delay}',
                      [W, W] + [ADD] * delay + lines + [ADD] * 14, setup)
        for delay in range(0, 36, 1 if suffix == '' else 2):
            bench(f'spi_probe{suffix}_after{delay}', probe(spin, delay),
                  setup)


def make_after_branch():
    '''Loads which come straight after a branch, where the firmware
    showed them taking longer than they do elsewhere.'''
    setup = ['ldr r8, =0x20016000', 'str r8, [r8]', 'mov r3, r8',
             'vldr s0, [r4, #32]', 'vldr s1, [r4, #36]',
             'nop', 'nop', 'nop', 'nop']
    L, L2 = 'ldr.w r0, [r8]', 'ldr.w r2, [r8, #4]'
    N, N2 = 'ldr r0, [r3]', 'ldr r2, [r3, #4]'
    VL, VL2 = 'vldr s3, [r8]', 'vldr s4, [r8, #4]'
    A = 'add.w r2, r1, r1'
    tests = [
        ('ldr', [L]),
        ('ldr_x2', [L, L2]),
        ('ldr_x3', [L, L2, L]),
        ('narrow', [N]),
        ('narrow_x2', [N, N2]),
        ('narrow_x3', [N, N2, N]),
        ('vldr_x2', [VL, VL2]),
        ('ldr_vldr', [L, VL2]),
        ('ldr_add_ldr', [L, A, L2]),
        ('ldr_dependent', [L, 'ldr r2, [r0, #4]']),
        ('ldr_sram1_x2', ['ldr r0, [r4]', 'ldr r2, [r4, #4]']),
        ('ldr_str', [L, 'str.w r2, [r8, #8]']),
        ('str_x2', ['str.w r2, [r8, #8]', 'str.w r2, [r8, #12]']),
        ('add_x3', [A, A, A]),
    ]
    for align, pad in [('aligned', []), ('unaligned', ['nop'])]:
        for name, lines in tests:
            tail = lines + ['adds r2, r1, r1'] * 2
            bench(f'seq_{name}_{align}', pad + tail, setup)
            bench(f'b_then_{name}_{align}',
                  ['b 1f', 'nop', '.balign 4'] + pad + ['1:'] + tail, setup)
            bench(f'bcc_then_{name}_{align}',
                  ['bne 1f', 'nop', '.balign 4'] + pad + ['1:'] + tail, setup)
            bench(f'not_taken_then_{name}_{align}',
                  pad + ['cmp r1, #0', 'beq 9f'] + tail, setup, ['9: nop'])
            bench(f'return_then_{name}_{align}',
                  ['bl 3f'] + pad + tail + ['b 4f', '.balign 4',
                                            '3: bx lr', '.balign 4', '4:'],
                  setup)
            bench(f'call_then_{name}_{align}',
                  ['bl 3f', 'b 4f', '.balign 4'] + pad + ['3:'] + tail +
                  ['bx lr', '.balign 4', '4:'], setup)

    # Typical code, a piece at a time.
    body = ['ldr.w r0, [r8]', 'adds r2, r1, r1', 'vadd.f32 s5, s0, s1',
            'cmp r1, r1', 'vmul.f32 s4, s0, s1', 'str r2, [r4, #4]']
    sequence = body * 2
    for count in range(1, len(sequence) + 1):
        bench(f'prefix_mixed_{count}', sequence[:count], setup)


def make_firmware_patterns():
    '''Sequences taken from the control interrupt, at places where a
    controller was found to take longer over it than the emulator.'''
    setup = ['ldr r8, =0x20016000', 'str r8, [r8]', 'str r8, [r8, #4]',
             'mov r3, r8', 'add.w r10, r8, #256',
             'vldr s0, [r4, #32]', 'vldr s1, [r4, #36]',
             'nop', 'nop', 'nop', 'nop']
    V, A = 'vadd.f32 s5, s0, s1', 'adds r2, r1, r1'
    # s0 is less than s1, so "mi" and "le" are true of this.
    CMP = ['vcmpe.f32 s0, s1', 'vmrs APSR_nzcv, fpscr']
    trailer = ['.balign 4', '8: .word 0x20016000', '9: nop']

    def both(name, lines):
        bench(f'{name}_aligned', lines, setup, trailer)
        bench(f'{name}_unaligned', ['nop'] + lines, setup, trailer)

    for follow_name, follow in [('wide', [V] * 4), ('narrow', [A] * 4)]:
        for width in ['n', 'w']:
            both(f'vcmp_not_taken_{width}_{follow_name}',
                 CMP + [f'bpl.{width} 9f'] + follow)
            both(f'vcmp_taken_{width}_{follow_name}',
                 CMP + [f'bmi.{width} 1f', 'nop', '1:'] + follow)
            both(f'vcmp_taken_{width}_skip2_{follow_name}',
                 CMP + [f'bmi.{width} 1f', 'nop', 'nop', '1:'] + follow)
            both(f'vadd_vcmp_not_taken_{width}_{follow_name}',
                 [V] + CMP + [f'bpl.{width} 9f'] + follow)
            both(f'cmp_not_taken_{width}_{follow_name}',
                 ['cmp r1, #0', f'beq.{width} 9f'] + follow)
            both(f'cmp.w_not_taken_{width}_{follow_name}',
                 ['cmp.w r1, #0', f'beq.{width} 9f'] + follow)
        both(f'vcmp_it_{follow_name}',
             CMP + ['it gt', 'vmovgt.f32 s3, s0'] + follow)
        both(f'vcmp_it_true_{follow_name}',
             CMP + ['it le', 'vmovle.f32 s3, s0'] + follow)
        both(f'vcmp_ite_{follow_name}',
             CMP + ['ite le', 'vmovle.f32 s3, s0', 'vmovgt.f32 s3, s1'] +
             follow)
        both(f'vcmp_x2_{follow_name}',
             CMP + ['bpl.w 9f'] + CMP + ['bpl.w 9f'] + follow)
        both(f'vcmp_x2_n_{follow_name}',
             CMP + ['bpl.n 9f'] + CMP + ['bpl.n 9f'] + follow)
        both(f'vcmp_only_{follow_name}', CMP + follow)

    tail = [A, A]
    L, L2 = 'ldr.w r0, [r8]', 'ldr.w r2, [r8, #4]'
    N, N2 = 'ldr r0, [r3]', 'ldr r2, [r3, #4]'
    for name, lines in [
            # A load through a pointer loaded two instructions before.
            ('ldr_x2_dependent_on_first', [L, L2, 'ldr r1, [r0, #8]']),
            ('ldr_x2_dependent_on_second', [L, L2, 'ldr r1, [r2, #8]']),
            ('ldr_x2_vldr_on_first', [L, L2, 'vldr s3, [r0, #8]']),
            ('narrow_x2_dependent_on_first', [N, N2, 'ldr r1, [r0, #8]']),
            ('ldr_x3_dependent_on_first', [L, L2, 'ldr.w r1, [r8, #8]',
                                           'ldr r1, [r0, #8]']),
            ('ldr_add_dependent', [L, 'adds r2, r1, r1', 'ldr r1, [r0, #8]']),
            ('ldr_chain_x3', [L, 'ldr r0, [r0]', 'ldr r0, [r0]',
                              'ldr r0, [r0]']),
            ('ldr_two_bases', [L, 'ldr.w r2, [r10]', 'ldr.w r1, [r8, #8]',
                               'ldr.w r2, [r10, #4]']),
            ('ldr_literal_vldr_x3', ['ldr r0, 8f', 'vldr s3, [r0]',
                                     'vldr s4, [r0, #4]',
                                     'vldr s5, [r0, #8]']),
            ('ldr_str_sp_x3', ['ldr r0, [r3, #4]', 'str r0, [sp, #4]',
                               'ldr r2, [r3, #8]', 'str r2, [sp, #8]',
                               'ldr r0, [r3, #12]', 'str r0, [sp]']),
            ('str_ldr_sp_x3', ['str r1, [sp, #4]', 'ldr r0, [r3, #4]',
                               'str r1, [sp, #8]', 'ldr r2, [r3, #8]']),
            ('ldr_x4_wide', [L, L2, 'ldr.w r0, [r8, #8]',
                             'ldr.w r2, [r8, #12]']),
            ('ldrh_x2_ldr_x2', ['ldrh.w r0, [r8]', 'ldrh.w r2, [r8, #4]',
                                'ldr.w r0, [r8, #8]', 'ldr.w r2, [r8, #12]']),
            # From GlobalPendSv.
            ('pendsv_velocity', [
                L, 'ldrb r2, [r0, #1]', 'cmp r2, #0', 'beq 9f', L2,
                'vldr s3, [r0, #48]', 'vldr s4, [r2, #4]', 'vldr s5, 8f',
                'vldr s6, 8f', 'vcvt.f32.u32 s4, s4']),
            ('pendsv_filter', [
                'cmp r1, #0', 'beq.w 9f', L, 'ldrsb.w r2, [r8, #8]',
                'vldr s3, [r0, #4]', 'vldr s4, [r3, #48]']),
            ('vldr_x2_vmul_vfma', [
                'vldr s3, [r8, #8]', 'vldr s4, [r8, #12]',
                'vmul.f32 s5, s0, s3', 'vfma.f32 s5, s1, s4']),
            ('not_taken_vldr_x2_vmul_vfma', [
                'cmp r1, #0', 'beq.w 9f',
                'vldr s3, [r8, #8]', 'vldr s4, [r8, #12]',
                'vmul.f32 s5, s0, s3', 'vfma.f32 s5, s1, s4']),
            ('vcmp_not_taken_vldr_x2_vmul_vfma', CMP + [
                'bpl.w 9f',
                'vldr s3, [r8, #8]', 'vldr s4, [r8, #12]',
                'vmul.f32 s5, s0, s3', 'vfma.f32 s5, s1, s4']),
            ('dmb_ldrb_dmb', ['ldr.w r2, [r8]', 'dmb ish',
                              'ldrb.w r3, [r2, #8]', 'dmb ish',
                              'uxtb r3, r3']),
    ]:
        both(name, lines + tail)

    # Every leading part of those which the model got wrong.
    for name, sequence in [
            ('literal_vldr', ['ldr r0, 8f', 'vldr s3, [r0]',
                              'vldr s4, [r0, #4]', 'vldr s5, [r0, #8]',
                              A, A]),
            ('velocity', [
                L, 'ldrb r2, [r0, #1]', 'cmp r2, #0', 'beq 9f', L2,
                'vldr s3, [r0, #48]', 'vldr s4, [r2, #4]', 'vldr s5, 8f',
                'vldr s6, 8f', 'vcvt.f32.u32 s4, s4', A, A]),
            ('ldr_str_sp', ['ldr r0, [r3, #4]', 'str r0, [sp, #4]',
                            'ldr r2, [r3, #8]', 'str r2, [sp, #8]',
                            'ldr r0, [r3, #12]', 'str r0, [sp]', A, A]),
            ('vcmp_vldr', CMP + [
                'bpl.w 9f', 'vldr s3, [r8, #8]', 'vldr s4, [r8, #12]',
                'vmul.f32 s5, s0, s3', 'vfma.f32 s5, s1, s4', A, A]),
            ('filter', ['cmp r1, #0', 'beq.w 9f', L, 'ldrsb.w r2, [r8, #8]',
                        'vldr s3, [r0, #4]', 'vldr s4, [r3, #48]', A, A]),
    ]:
        for count in range(1, len(sequence) + 1):
            both(f'prefix_fw_{name}_{count}', sequence[:count])

    # What it is about a conditional branch which is not taken, and
    # what is around it, which decides whether it costs a cycle.
    W = 'add.w r2, r1, r1'
    before = [
        ('cmp', ['cmp r1, #0']),
        ('cmp.w', ['cmp.w r1, #0']),
        ('nop', ['nop']),
        ('nop.w', ['nop.w']),
        ('vmrs', ['vmrs APSR_nzcv, fpscr']),
        ('cmp_nop', ['cmp r1, #0', 'nop']),
        ('cmp.w_nop.w', ['cmp.w r1, #0', 'nop.w']),
        ('cmp_nop.w', ['cmp r1, #0', 'nop.w']),
        ('cmp.w_nop', ['cmp.w r1, #0', 'nop']),
        ('add.w_x2_cmp.w', [W, W, 'cmp.w r1, #0']),
        ('add.w_x3', [W, W, W]),
    ]
    after = [('add.w', [W] * 4), ('vadd', [V] * 4), ('adds', [A] * 4),
             ('adds_add.w', [A] + [W] * 4)]
    for before_name, lead in before:
        for width in ['n', 'w']:
            for after_name, follow in after:
                both(f'nt_{before_name}_b{width}_{after_name}',
                     lead + [f'beq.{width} 9f'] + follow)

    # How much of what follows has been fetched after each of these,
    # as shown by whether a branch which is not taken then costs a
    # cycle.
    L16, S16 = 'ldr r0, [r4]', 'str r1, [r4, #4]'
    LW = 'ldr.w r0, [r8]'
    BRANCH = ['b 1f', 'nop', '.balign 4', '1:']
    sizes = {A: 2, 'nop': 2, L16: 2, S16: 2, W: 4, V: 4, LW: 4,
             'cmp r1, #0': 2, 'beq 9f': 2, 'vmrs APSR_nzcv, fpscr': 4,
             'nop.w': 4}
    for name, lines in [
            ('none', []),
            ('adds', [A]), ('adds_x2', [A] * 2), ('adds_x3', [A] * 3),
            ('adds_x4', [A] * 4), ('adds_x5', [A] * 5),
            ('nop_x2', ['nop'] * 2), ('nop_x4', ['nop'] * 4),
            ('add.w', [W]), ('add.w_x2', [W] * 2),
            ('adds_x2_add.w', [A, A, W]),
            ('adds_x2_add.w_x4', [A, A] + [W] * 4),
            ('adds_x4_add.w_x4', [A] * 4 + [W] * 4),
            ('ldr', [L16]), ('ldr_adds', [L16, A]),
            ('ldr_adds_x2', [L16, A, A]), ('ldr_adds_x3', [L16, A, A, A]),
            ('ldr_adds_x4', [L16, A, A, A, A]),
            ('ldr_nop_x2', [L16, 'nop', 'nop']),
            ('adds_x4_ldr', [A] * 4 + [L16]),
            ('adds_x4_ldr_adds', [A] * 4 + [L16, A]),
            ('adds_x4_ldr_adds_x2', [A] * 4 + [L16, A, A]),
            ('adds_x3_ldr_add.w', [A] * 3 + [L16, W]),
            ('adds_x3_ldr_add.w_x2', [A] * 3 + [L16, W, W]),
            ('str', [S16]), ('str_adds_x2', [S16, A, A]),
            ('adds_x4_str', [A] * 4 + [S16]),
            ('adds_x4_str_adds', [A] * 4 + [S16, A]),
            ('vadd', [V]), ('vadd_adds_x2', [V, A, A]),
            ('adds_x2_vadd', [A, A, V]), ('adds_x2_vadd_x2', [A, A, V, V]),
            ('adds_x4_vadd', [A] * 4 + [V]),
            ('adds_x4_vadd_x2', [A] * 4 + [V, V]),
            ('adds_x4_vadd_x3', [A] * 4 + [V, V, V]),
            ('adds_x3_vadd', [A] * 3 + [V]),
            ('adds_x3_vadd_x2', [A] * 3 + [V, V]),
            ('adds_x3_vadd_x3', [A] * 3 + [V, V, V]),
            ('ldr.w', [LW]), ('adds_x4_ldr.w', [A] * 4 + [LW]),
            ('adds_x4_ldr.w_add.w', [A] * 4 + [LW, W]),
            ('adds_x4_ldr.w_add.w_x2', [A] * 4 + [LW, W, W]),
            ('adds_x3_ldr.w', [A] * 3 + [LW]),
            ('adds_x3_ldr.w_add.w', [A] * 3 + [LW, W]),
            ('adds_x3_ldr.w_add.w_x2', [A] * 3 + [LW, W, W]),
            ('adds_x4_not_taken', [A] * 4 + ['cmp r1, #0', 'beq 9f']),
            ('adds_x4_not_taken_adds', [A] * 4 + ['cmp r1, #0', 'beq 9f', A]),
            ('adds_x4_not_taken_adds_x2',
             [A] * 4 + ['cmp r1, #0', 'beq 9f', A, A]),
            ('adds_x4_vmrs', [A] * 4 + ['vmrs APSR_nzcv, fpscr']),
            ('branch', BRANCH), ('branch_adds', BRANCH + [A]),
            ('branch_adds_x2', BRANCH + [A, A]),
            ('branch_adds_x3', BRANCH + [A, A, A]),
            ('branch_add.w', BRANCH + [W]),
            ('branch_adds_add.w', BRANCH + [A, W]),
    ]:
        length = sum(sizes.get(x, 0) for x in lines)
        if length % 4 == 0:
            probe = ['beq.w 9f', W, W, W]
        else:
            probe = ['beq.n 9f', W, W, W]
        bench(f'fetched_after_{name}', lines + probe, setup, trailer)
        bench(f'fetched_base_{name}', lines + [W, W, W], setup, trailer)

    # The same after a branch, where it comes out differently.
    UNALIGNED = ['b 1f', 'nop', '.balign 4', 'nop', '1:']
    CONDITIONAL = ['bne 1f', 'nop', '.balign 4', '1:']
    for name, lead, lines in [
            ('branch_adds_x4', BRANCH, [A] * 4),
            ('branch_adds_x5', BRANCH, [A] * 5),
            ('branch_adds_x6', BRANCH, [A] * 6),
            ('branch_adds_x7', BRANCH, [A] * 7),
            ('branch_add.w_adds', BRANCH, [W, A]),
            ('branch_add.w_x2_adds', BRANCH, [W, W, A]),
            ('branch_adds_add.w_x2', BRANCH, [A, W, W]),
            ('branch_adds_x2_add.w', BRANCH, [A, A, W]),
            ('branch_adds_x3_add.w', BRANCH, [A, A, A, W]),
            ('bcc_adds', CONDITIONAL, [A]),
            ('bcc_adds_x2', CONDITIONAL, [A, A]),
            ('bcc_adds_x3', CONDITIONAL, [A, A, A]),
            ('branch_unaligned', UNALIGNED, []),
            ('branch_unaligned_adds', UNALIGNED, [A]),
            ('branch_unaligned_adds_x2', UNALIGNED, [A, A]),
            ('branch_unaligned_adds_x3', UNALIGNED, [A, A, A]),
            ('branch_unaligned_add.w', UNALIGNED, [W]),
            ('add.w_adds', [], [W, A]),
            ('add.w_x2_adds', [], [W, W, A]),
            ('adds_add.w', [], [A, W]),
            ('adds_add.w_x2', [], [A, W, W]),
            ('adds_x3_add.w', [], [A, A, A, W]),
            ('adds_x6', [], [A] * 6), ('adds_x7', [], [A] * 7),
    ]:
        length = sum(sizes.get(x, 0) for x in lines)
        if lead is UNALIGNED:
            length += 2
        if length % 4 == 0:
            probe = ['beq.w 9f', W, W, W]
        else:
            probe = ['beq.n 9f', W, W, W]
        bench(f'fetched_after_{name}', lead + lines + probe, setup, trailer)
        bench(f'fetched_base_{name}', lead + lines + [W, W, W], setup,
              trailer)

    # Floating point loads through a pointer which was just loaded.
    VA, VB, VC = 'vldr s3, [r0]', 'vldr s4, [r0, #4]', 'vldr s5, [r0, #8]'
    for name, sequence in [
            ('pointer_vldr_other_base', [L, VA, 'vldr s4, [r8, #4]',
                                         'vldr s5, [r8, #8]', A, A]),
            ('pointer_add_vldr', [L, A, VA, VB, VC, A, A]),
            ('pointer_ldr_vldr', [L, 'ldr r1, [r0]', VB, VC, A, A]),
            ('vldr_low_base', ['vldr s3, [r3]', 'vldr s4, [r3, #4]',
                               'vldr s5, [r3, #8]', A, A]),
            ('mov_vldr', ['mov r0, r8', VA, VB, VC, A, A]),
            ('pointer_vldr_add_vldr', [L, VA, A, VB, VC, A, A]),
            ('ldr_other_vldr_low_base', [L2, 'vldr s3, [r3]',
                                         'vldr s4, [r3, #4]',
                                         'vldr s5, [r3, #8]', A, A]),
            ('pointer_vldr_vstr', [L, VA, 'vstr s0, [r0, #16]',
                                   'vstr s1, [r0, #20]', A, A]),
            ('pointer_vstr', [L, 'vstr s0, [r0, #16]',
                              'vstr s1, [r0, #20]', A, A]),
            ('vldr_x3_offsets', ['vldr s3, [r8]', 'vldr s4, [r8, #4]',
                                 'vldr s5, [r8, #8]', A, A]),
    ]:
        for count in range(1, len(sequence) + 1):
            both(f'prefix_fw_{name}_{count}', sequence[:count])

    # The CORDIC, as the firmware sets it up and uses it: how long
    # after the argument is written the results can be read.
    cordic = setup + ['ldr r9, =0x40020c00', 'ldr r0, =0x00080050',
                      'str r0, [r9]', 'ldr r0, =0x20000000',
                      'vmov s6, r0'] + ['nop'] * 6
    for delay in range(9):
        bench(f'cordic_after{delay}',
              ['vstr s6, [r9, #4]'] + [A] * delay +
              ['vldr s3, [r9, #8]', 'vldr s4, [r9, #8]', A, A], cordic,
              trailer)
        bench(f'cordic_core_after{delay}',
              ['str r0, [r9, #4]'] + [A] * delay +
              ['ldr r2, [r9, #8]', 'ldr r2, [r9, #8]', A, A], cordic,
              trailer)

    # Loads of literals in the places the firmware has them.
    LIT, VLIT = 'ldr r2, 8f', 'vldr s3, 8f'
    for name, sequence in [
            ('taken_literal', CMP + ['bmi.n 1f', 'nop', '1:', LIT, VLIT,
                                     'vldr s4, 8f', 'vldr s6, 8f', A, A]),
            ('taken_w_literal', CMP + ['bmi.w 1f', 'nop', '1:', LIT, VLIT,
                                       'vldr s4, 8f', A, A]),
            ('not_taken_literal', ['cmp r1, #0', 'beq.w 9f', 'ldr r0, 8f',
                                   'vldr s3, [r0, #8]', 'vldr s4, [r0, #4]',
                                   'vldr s5, [r0, #12]', A, A]),
            ('vldr_literal_vabs', [VLIT, 'vabs.f32 s4, s0', 'cmp r1, #0',
                                   'bne.w 1f', 'nop', '1:'] + CMP + [A, A]),
            ('vldr_x2_literal', ['vldr s3, [r3, #8]', 'vldr s4, [r3, #12]',
                                 'vldr s5, 8f', 'ldrb.w r2, [r3, #16]',
                                 A, A]),
            ('literal_x2_vcvt', [VLIT, 'vldr s4, 8f',
                                 'vcvt.f32.u32 s3, s3', W, A, A]),
            ('add_literal_add', [W, VLIT, W, W, A, A]),
            ('adds_literal_adds', [A, LIT, A, A, A]),
            ('vmul_literal_vmul', ['vmul.f32 s5, s0, s1', VLIT,
                                   'vmul.f32 s6, s0, s3', V, A, A]),
    ]:
        for count in range(1, len(sequence) + 1):
            if sequence[count - 1].endswith(':'):
                continue
            both(f'prefix_fw_{name}_{count}', sequence[:count] +
                 (['1:'] if any('1f' in x for x in sequence[:count])
                  and '1:' not in sequence[:count] else []))

    # Whether which registers they use matters to that.
    for base in ['r0', 'r1', 'r2', 'r3', 'r10', 'r12']:
        base_setup = setup + [f'mov {base}, r8'] + ['nop'] * 8
        for name, lines in [
                ('', [f'vldr s3, [{base}]', f'vldr s4, [{base}, #4]',
                      f'vldr s5, [{base}, #8]']),
                ('_high', [f'vldr s16, [{base}]', f'vldr s17, [{base}, #4]',
                           f'vldr s18, [{base}, #8]']),
                ('_s0', [f'vldr s0, [{base}]', f'vldr s1, [{base}, #4]',
                         f'vldr s2, [{base}, #8]']),
                ('_s6', [f'vldr s6, [{base}]', f'vldr s7, [{base}, #4]',
                         f'vldr s8, [{base}, #8]']),
                ('_offset', [f'vldr s3, [{base}, #16]',
                             f'vldr s4, [{base}, #20]',
                             f'vldr s5, [{base}, #24]']),
                ('_same', [f'vldr s3, [{base}]'] * 3),
                ('_vstr', [f'vstr s0, [{base}, #16]',
                           f'vstr s1, [{base}, #20]',
                           f'vstr s0, [{base}, #24]']),
                ('_ldr', [f'ldr.w r9, [{base}]', f'ldr.w r9, [{base}, #4]',
                          f'ldr.w r9, [{base}, #8]']),
        ]:
            bench(f'vldr_x3_base_{base}{name}', lines + ['nop.w', 'nop.w'],
                  base_setup, trailer)

    # A load through a pointer, and what else then uses that pointer.
    D1, D2, D3 = 'ldr r1, [r0]', 'ldr r2, [r0, #4]', 'ldr r1, [r0, #8]'
    for name, sequence in [
            ('literal_ldr', ['ldr r0, 8f', D1, D2, D3, A, A]),
            ('literal_add_ldr', ['ldr r0, 8f', A, D1, D2, D3, A, A]),
            ('literal_ldr_add_ldr', ['ldr r0, 8f', D1, A, D2, D3, A, A]),
            ('pointer_ldr', [L, D1, D2, D3, A, A]),
            ('pointer_vldr', [L, 'vldr s3, [r0]', 'vldr s4, [r0, #4]',
                              'vldr s5, [r0, #8]', A, A]),
            ('literal_ldr_r3', ['ldr r0, 8f', 'ldr r1, [r3]',
                                'ldr r2, [r3, #4]', 'ldr r1, [r3, #8]',
                                A, A]),
            ('str_sp_ldr', ['str r1, [sp, #4]', L, A, A]),
            ('adds_str_sp_ldr', [A, 'str r1, [sp, #4]', L, A, A]),
            ('str_sp_ldr_narrow', ['str r1, [sp, #4]', 'ldr r0, [r3]',
                                   A, A]),
    ]:
        for count in range(1, len(sequence) + 1):
            both(f'prefix_fw_{name}_{count}', sequence[:count])

    # One exception handler making another pending, as the timer
    # interrupt does with PendSV.
    svc = ['bl clear_fp_context', 'ldr r0, =vectors',
           'ldr r1, =plain_handler', 'str r1, [r0, #56]']
    bench('svc', ['svc 0'], svc + ['ldr r1, =plain_handler',
                                   'str r1, [r0, #44]'])
    bench('svc_pends_pendsv', ['svc 0'],
          svc + ['ldr r1, =pend_handler', 'str r1, [r0, #44]'])
    bench('svc_pends_pendsv_pop', ['svc 0'],
          svc + ['ldr r1, =pend_pop_handler', 'str r1, [r0, #44]'])
    bench('svc_pends_pendsv_pool_load', ['svc 0'],
          ['bl clear_fp_context', 'ldr r0, =vectors',
           'ldr r1, =pool_load_handler', 'str r1, [r0, #56]',
           'ldr r1, =pend_pop_handler', 'str r1, [r0, #44]'])
    bench('_restore_vectors', [],
          ['ldr r0, =vectors', 'ldr r1, =pendsv', 'str r1, [r0, #56]'])


def make_firmware_patterns_2():
    '''More of the control interrupt, as it is in the function which
    runs the current controllers: mostly floating point, with literals
    and with "this" in r0.'''
    setup = ['ldr r8, =0x20016000', 'str r8, [r8]', 'str r8, [r8, #4]',
             'mov r3, r8', 'add.w r10, r8, #256', 'mov r12, r8',
             'vldr s0, [r4, #32]', 'vldr s1, [r4, #36]', 'mov r0, r8',
             'nop', 'nop', 'nop', 'nop', 'nop', 'nop']
    A = 'adds r2, r1, r1'
    # s0 is less than s1.
    CMP = ['vcmpe.f32 s0, s1', 'vmrs APSR_nzcv, fpscr']
    trailer = ['.balign 4', '8: .float 2.0', '9: nop']

    def prefixes(name, sequence):
        for count in range(1, len(sequence) + 1):
            if sequence[count - 1].endswith(':'):
                continue
            lines = sequence[:count]
            # Not part way through getting to where a branch goes.
            if any(any(x.endswith(f' {label}f') for x in lines) and
                   f'{label}:' not in lines for label in '1234'):
                continue
            if lines[-1] == 'nop' or lines[-1].startswith('.balign'):
                continue
            for align, pad in [('aligned', []), ('unaligned', ['nop'])]:
                bench(f'prefix_fw2_{name}_{count}_{align}', pad + lines,
                      setup, trailer)

    prefixes('clamp', [
        'vldr s14, [r0, #60]', 'vldr s12, [r0, #64]', 'vldr s10, 8f',
        'ldrb.w r2, [r0, #44]', 'vsub.f32 s12, s12, s14',
        'vmov.f32 s13, s14', 'vfma.f32 s13, s12, s1', 'addw r3, r0, #128',
        'vcmpe.f32 s13, s10', 'vmrs APSR_nzcv, fpscr', 'it le',
        'vmovle.f32 s13, s10', 'vcmpe.f32 s14, s13',
        'vmrs APSR_nzcv, fpscr', 'it le', 'vmovle.f32 s13, s14',
        'vstr s13, [r3]', 'add.w r3, r0, #132', 'vldr s11, [r3]',
        'vabs.f32 s9, s11', A, A])
    prefixes('limit', [
        'cmp r1, #0', 'beq.w 9f', 'vldr s12, 8f', 'vcmp.f32 s0, s12',
        'vmrs APSR_nzcv, fpscr', 'bhi.w 9f', 'vldr s12, [r0, #76]',
        'vldr s8, [r0, #72]', 'vldr s9, 8f', 'vmul.f32 s11, s0, s12',
        'vmov.f32 s12, s11', 'vfma.f32 s12, s8, s1', 'vcmpe.f32 s0, s1',
        'vmrs APSR_nzcv, fpscr', 'bmi.w 1f', 'nop', '.balign 4', '1:',
        'vldr s11, 8f', 'vcmpe.f32 s1, s0', 'vmrs APSR_nzcv, fpscr',
        'bmi.w 9f', 'vmov.f32 s12, s10', 'b.n 2f', 'nop', '2:',
        'vstr s12, [r3]', 'movs r2, #0', A, A])
    prefixes('gather', [
        'vldr s12, [r3]', 'vldr s13, [r10, #24]', 'ldrb.w r2, [r12]',
        'vldr s11, 8f', 'vldr s10, [r12, #12]', 'ldr.w r2, [r10, #92]',
        'vldr s9, [r10, #88]', 'vmul.f32 s12, s12, s13', 'vmov s13, r1',
        'vmul.f32 s12, s12, s11', 'vcvt.f32.s32 s13, s13, #1', A, A])
    prefixes('threshold', [
        'vldr s2, [r0]', 'cmp r1, #0', 'beq.w 9f', 'vldr s4, [r3, #4]',
        'vldr s9, 8f', 'vabs.f32 s3, s4', 'vcmp.f32 s3, s9',
        'vmrs APSR_nzcv, fpscr', 'bhi.w 9f', 'vmul.f32 s13, s2, s0',
        'vsub.f32 s3, s1, s4', A, A])
    prefixes('nan_check', [
        'cmp r1, #0', 'beq.w 9f', 'vldr s11, [r0, #96]',
        'vcmp.f32 s11, s11', 'vmrs APSR_nzcv, fpscr', 'bvc.w 1f', 'nop',
        '.balign 4', 'nop', '1:', 'vldr s11, [r0, #92]',
        'vcmp.f32 s11, s11', 'vmrs APSR_nzcv, fpscr', 'bvc.n 2f', 'nop',
        '2:', 'movs r3, #0', 'b.n 3f', 'nop', '.balign 4', 'nop', '3:',
        'vldr s11, [r10, #32]', 'vldr s9, [r0, #68]', 'vabs.f32 s10, s11',
        A, A])
    prefixes('call', [
        'bl 3f', 'b 4f', '.balign 4', '3:',
        'stmdb sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}',
        'addw r3, r0, #92', 'sub sp, #20', 'ldr r2, [r3]',
        'str r2, [sp, #4]', 'vldr s7, [sp, #4]', 'add sp, #20',
        'ldmia.w sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}', '.balign 4',
        '4:', 'ldrb.w r3, [r8, #88]', 'cmp r3, #0', A, A])
    prefixes('call_small', [
        'bl 3f', 'b 4f', '.balign 4', '3:', 'push {r4, lr}',
        'ldr.w r4, [r0, #64]', 'ldrb.w ip, [r0, #28]', 'pop {r4, pc}',
        '.balign 4', '4:', 'vldr s15, [r0, #80]', 'ldrb.w r3, [r0, #84]',
        A, A])
    # Table branches, and what they land on.
    W = 'add.w r2, r1, r1'
    for name, tail in [
            ('ldrb_ldr', ['ldrb.w r2, [r4, #8]', 'ldr.w r3, [r4, #12]',
                          A, A]),
            ('ldr_vldr', ['ldr.w r3, [r8]', 'vldr s3, [r3, #8]',
                          'vcmp.f32 s3, s3', A, A]),
            ('add.w', [W, W, A, A]),
            ('adds', [A, A, A]),
    ]:
        for align, pad in [('aligned', []), ('unaligned', ['nop'])]:
            for index in [0, 1]:
                bench(f'tbb_{index}_{name}_{align}',
                      ['movs r3, #%d' % index, 'tbb [pc, r3]',
                       '5: .byte (6f-5b)/2, (7f-5b)/2', '.balign 4'] + pad +
                      ['6:'] + tail + ['b 4f', '.balign 4'] + pad +
                      ['7:'] + tail + ['4:'], setup, trailer)

    # Calls and returns through the stack, a piece at a time.
    PUSH9 = 'stmdb sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}'
    POP9 = 'ldmia.w sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}'
    for name, body in [
            ('push_pop', ['push {r4, lr}', 'pop {r4, pc}']),
            ('push_adds_pop', ['push {r4, lr}', A, 'pop {r4, pc}']),
            ('push_ldr_pop', ['push {r4, lr}', 'ldr.w r4, [r0, #64]',
                              'pop {r4, pc}']),
            ('push_ldr_x2_pop', ['push {r4, lr}', 'ldr.w r4, [r0, #64]',
                                 'ldrb.w ip, [r0, #28]', 'pop {r4, pc}']),
            ('push9_pop9', [PUSH9, POP9]),
            ('push9_addw_pop9', [PUSH9, 'addw r3, r0, #92', POP9]),
            ('push9_sub_add_pop9', [PUSH9, 'sub sp, #20', 'add sp, #20',
                                    POP9]),
            ('push9_adds_x4_pop9', [PUSH9, A, A, A, A, POP9]),
            ('push9_ldr_str_vldr_pop9', [
                PUSH9, 'addw r3, r0, #92', 'sub sp, #20', 'ldr r2, [r3]',
                'str r2, [sp, #4]', 'vldr s7, [sp, #4]', 'add sp, #20',
                POP9]),
            ('bx', ['bx lr']),
            ('adds_bx', [A, 'bx lr']),
    ]:
        for align, pad in [('aligned', []), ('unaligned', ['nop'])]:
            for after_name, after in [('adds', [A, A]),
                                      ('vldr', ['vldr s15, [r3, #80]', A])]:
                bench(f'call_{name}_then_{after_name}_{align}',
                      ['bl 3f', 'b 4f', '.balign 4'] + pad + ['3:'] + body +
                      ['.balign 4', '4:'] + after, setup, trailer)

    # A store to the private peripheral bus, and then 32 bit
    # instructions.
    # An interrupt priority, which nothing here depends on.
    ppb = setup + ['ldr r9, =0xe000e400']
    for name, lines in [
            ('ppb_str_add.w', ['str r1, [r9, #4]', W, W, W]),
            ('ppb_ldr_str_add.w', ['ldr r2, [r9, #4]', 'str r2, [r9, #4]',
                                   W, W, W]),
            ('ppb_str_adds', ['str r1, [r9, #4]', A, A, A]),
            ('ppb_adds_x4_str_add.w', [A, A, A, A, 'str r1, [r9, #4]',
                                       W, W, W]),
            ('ppb_ldr_orr_str_pop_bx', [
                'bl 3f', 'b 4f', '.balign 4', '3:', 'push {r4}',
                'ldr r3, [r9, #4]', 'orr.w r3, r3, #0x100',
                'str r3, [r9, #4]', 'pop {r4}', 'bx lr', '.balign 4',
                '4:', A, A]),
    ]:
        for align, pad in [('aligned', []), ('unaligned', ['nop'])]:
            bench(f'{name}_{align}', pad + lines, ppb, trailer)

    bench('_restore_r12', [], ['mov r12, #0'])


def make_timer_interrupt():
    '''How long it is from a timer reaching the top of its count, as
    the one which starts the control interrupt does, until a handler
    reads the count.  The firmware reports how long the control
    interrupt took in those terms.

    The handler records the count, and what is timed includes a loop
    which runs that many times, so that a difference in the count
    shows up threefold.'''
    timer = [
        'bl clear_fp_context',
        'ldr r9, =0x40000000',
        'movs r0, #0', 'str r0, [r9]', 'str r0, [r9, #0x24]',
        'movs r0, #200', 'str r0, [r9, #0x2c]',
        'movs r0, #1', 'str r0, [r9, #0x0c]',
        'ldr r0, =0xe000e100', 'mov r1, #0x10000000', 'str r1, [r0]',
        'movs r1, #4',
    ]
    divide = [
        'ldr r0, =0x40021008', 'ldr r1, [r0]', 'tst r1, #0x2000', 'bne 7f',
        'orr r1, r1, #0x2400', 'str r1, [r0]', '7:', 'movs r1, #4',
    ]
    for suffix, setup in [('', timer), ('_div2', divide + timer)]:
        for delay in range(4):
            bench(f'timer_interrupt{suffix}_delay{delay}',
                  ['adds r3, r1, r1'] * delay +
                  ['movs r0, #0x61', 'str r0, [r9]'] +
                  ['adds r3, r1, r1'] * 240 +
                  ['ldr r0, [r4, #48]', '5: subs r0, #1', 'bne 5b'],
                  setup + ['nop'] * 4)
    bench('_disable_timer_interrupt', [],
          ['ldr r0, =0xe000e180', 'mov r1, #0x10000000', 'str r1, [r0]'])


def source(chunk):
    lines = f'''
.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.macro SETUP
    ldr r4, =0x{DATA:08x}
    @ Earlier benchmarks may have stored over this.
    add r0, r4, #16
    str r0, [r4]
    movs r0, #0
    movs r1, #4
    movs r2, #8
    movs r3, #3
    msr apsr_nzcvq, r0
.endm

@ Start timing.  This ends on a 4 byte boundary.
.macro BEGIN
    .balign 4
    nop
    ldr r7, [r6]
    nop
    nop
.endm

.macro END
    nop
    nop
    ldr r0, [r6]
    subs r0, r0, r7
    str r0, [r5], #4
.endm

.text
.global start
.thumb_func
start:
    ldr r0, =0x{STACK:08x}
    mov sp, r0
{CLOCK_SETUP if FAST_CLOCK else ''}
    @ Enable the FPU.
    ldr r0, =0xe000ed88
    ldr r1, =0x00f00000
    str r1, [r0]
    dsb
    isb
    @ Enable the cycle counter.
    ldr r0, =0xe000edfc
    ldr r1, [r0]
    orr r1, r1, #0x01000000
    str r1, [r0]
    ldr r0, =0xe0001000
    ldr r1, [r0]
    orr r1, r1, #1
    str r1, [r0]
    @ Take exceptions from our own table.
    ldr r0, =0xe000ed08
    ldr r1, =vectors
    str r1, [r0]
    @ Clock the CORDIC.
    ldr r0, =0x40021048
    ldr r1, [r0]
    orr r1, r1, #8
    str r1, [r0]
    @ Clock GPIOA, ADC12, TIM1 and TIM2.
    ldr r0, =0x4002104c
    ldr r1, [r0]
    orr r1, r1, #0x2000
    orr r1, r1, #0x0001
    str r1, [r0]
    ldr r0, =0x40021058
    ldr r1, [r0]
    orr r1, r1, #1
    str r1, [r0]
    ldr r0, =0x40021060
    ldr r1, [r0]
    orr r1, r1, #0x800
    str r1, [r0]
    @ Initialize the data the benchmarks load.
    ldr r4, =0x{DATA:08x}
    add r0, r4, #16
    str r0, [r4]
    movs r0, #0
    str r0, [r4, #16]
    ldr r0, =0x3fc00000  @ 1.5
    str r0, [r4, #32]
    ldr r0, =0x40200000  @ 2.5
    str r0, [r4, #36]
    ldr r0, =0x40700000  @ 3.75
    str r0, [r4, #40]

    ldr r5, =0x{RESULTS:08x}
    ldr r6, =0xe0001004
    b benchmarks
    .ltorg

@ Leave thread mode with no floating point context.
.thumb_func
clear_fp_context:
    mrs r0, control
    bic r0, r0, #4
    msr control, r0
    isb
    bx lr

.thumb_func
plain_handler:
    bx lr

.thumb_func
fp_handler:
    vmov.f32 s0, #2.0
    bx lr

.thumb_func
nop_handler:
    nop
    nop
    nop
    nop
    bx lr

.thumb_func
pool_load_handler:
    ldr r0, =0x12345678
    bx lr
    .ltorg

.thumb_func
nop_pool_load_handler:
    nop
    ldr r0, =0x12345678
    bx lr
    .ltorg

.thumb_func
sram_load_handler:
    ldr r0, [r4]
    bx lr

.thumb_func
bx_reg_handler:
    bx r10

.balign 4
.thumb_func
plain_handler_2:
    bx lr

.thumb_func
adds_handler:
    adds r0, r1, r2
    bx lr

.thumb_func
push_handler:
    push {{r4, lr}}
    pop {{r4, pc}}

@ Record how far the timer has counted since it raised the interrupt,
@ and stop it.
.thumb_func
tim2_handler:
    ldr r1, [r9, #0x24]
    str r1, [r4, #48]
    movs r1, #0
    str r1, [r9, #0x10]
    str r1, [r9]
    bx lr

.thumb_func
pend_handler:
    ldr r0, =0xe000ed04
    mov r1, #0x10000000
    str r1, [r0]
    bx lr
    .ltorg

.thumb_func
pend_pop_handler:
    push {{r4}}
    ldr r0, =0xe000ed04
    ldr r3, [r0]
    orr r3, r3, #0x10000000
    str r3, [r0]
    pop {{r4}}
    bx lr
    .ltorg

.thumb_func
pendsv:
    ldr r0, =pendsv_target
    ldr r0, [r0]
    bx r0
    .ltorg

.thumb_func
fault:
    b fault

.balign 4
pendsv_target:
    .word plain_handler + 1

@ Data in the same RAM the code executes from.
.balign 4
code_ram_data:
    .word code_ram_data + 16
    .word 0x11111111
    .word 0x22222222
    .word 0x33333333
    .word 0
    .word 0
    .float 1.5
    .float 2.5

@ The processor forms a vector's address by setting bits in that of
@ the table, so the table has to be aligned to its size.
.balign 256
vectors:
    .word 0x{STACK:08x}
    .word start + 1
    .rept 12
    .word fault + 1
    .endr
    .word pendsv + 1
    .word fault + 1
    .rept 28
    .word fault + 1
    .endr
    .word tim2_handler + 1

benchmarks:
'''.split('\n')
    for i, (name, setup, body, trailer) in enumerate(chunk):
        lines.append(f'@ {name}')
        lines.append('    SETUP')
        lines += ['    ' + x for x in setup]
        if name.startswith('_'):
            # Not timed: this only tidies up after the previous one.
            lines += ['    ' + x for x in body]
        else:
            lines.append('    BEGIN')
            lines += ['    ' + x for x in body]
            lines.append('    END')
        # A label which no trailer will use for its own purposes.
        lines.append('    b 99f')
        lines.append('    .ltorg')
        lines.append('    .balign 4')
        lines += ['    ' + x for x in trailer]
        lines.append('    .balign 4')
        lines.append('99:')
    lines += [f'    ldr r0, =0x{END_MARKER:08x}', '    str r0, [r5]',
              'done:', '    b done', '    .ltorg']
    return '\n'.join(lines) + '\n'


def main():
    global FAST_CLOCK
    FAST_CLOCK = '--170mhz' in sys.argv
    make_integer()
    make_loads_and_stores()
    make_branches()
    make_floating_point()
    make_peripherals()
    make_exceptions()
    make_followups()
    make_followups_2()
    make_followups_3()
    make_memory_banks()
    make_mixed_banks()
    make_calls()
    make_long_runs()
    make_fetch()
    make_prefixes()
    make_prefixes_2()
    make_prefixes_3()
    make_divided_apb()
    make_after_branch()
    make_firmware_patterns()
    make_firmware_patterns_2()
    make_timer_interrupt()

    print('# Generated by make_benchmarks.py.')
    print('#')
    print('# This is a series of programs, each introduced by "program".')
    print('# "name N" lines give the names of its results, in order, the')
    print('# first of which is the empty benchmark to subtract from the')
    print('# rest.  "code" lines are the program itself, as hex bytes, to')
    print(f'# load at 0x{CODE:08x} and start from there.')
    for number, chunk in enumerate(chunks()):
        blob = assemble(chunk)
        print(f'program {number}')
        for name, _, _, _ in chunk:
            if not name.startswith('_'):
                print(f'name {name}')
        for i in range(0, len(blob), 32):
            print('code ' + blob[i:i + 32].hex())


# The most that fits below the data kept in the same RAM.
MAX_PROGRAM_SIZE = CODE_RAM_DATA - CODE
CHUNK_SIZE = 90


def chunks():
    '''Split the benchmarks into programs which fit in the RAM they
    run from, each starting with its own empty benchmark.'''
    result = [[]]
    for entry in benchmarks:
        name = entry[0]
        # Something which only tidies up stays with what it follows.
        if len(result[-1]) >= CHUNK_SIZE and not name.startswith('_'):
            empty = (f'empty_{len(result)}', [], [], [])
            result.append([empty])
        result[-1].append(entry)
    return result


def assemble(chunk):
    with tempfile.TemporaryDirectory() as tmp:
        with open(tmp + '/b.s', 'w') as f:
            f.write(source(chunk))
        subprocess.check_call(
            ['arm-none-eabi-as', '-o', tmp + '/b.o', tmp + '/b.s'])
        subprocess.check_call(
            ['arm-none-eabi-ld', f'-Ttext=0x{CODE:08x}', '-e', 'start',
             '-o', tmp + '/b.elf', tmp + '/b.o'])
        subprocess.check_call(
            ['arm-none-eabi-objcopy', '-O', 'binary', '-j', '.text',
             tmp + '/b.elf', tmp + '/b.bin'])
        blob = open(tmp + '/b.bin', 'rb').read()
    assert len(blob) < MAX_PROGRAM_SIZE, 'does not leave room for data'
    return blob


if __name__ == '__main__':
    main()
