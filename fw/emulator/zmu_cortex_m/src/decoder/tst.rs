use crate::core::bits::Bits;
use crate::core::instruction::Imm32Carry;
use crate::core::instruction::Instruction;
use crate::core::instruction::{Reg2ShiftNoSetFlagsParams, RegImmCarryNoSetFlagsParams, SRType};
use crate::core::operation::decode_imm_shift;
use crate::core::operation::thumb_expand_imm_c;

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_TST_reg_t1(opcode: u16) -> Instruction {
    Instruction::TST_reg {
        params: Reg2ShiftNoSetFlagsParams {
            rn: opcode.get_bits(0..3).into(),
            rm: opcode.get_bits(3..6).into(),
            shift_t: SRType::LSL,
            shift_n: 0,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_TST_reg_t2(opcode: u32) -> Instruction {
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm2: u8 = opcode.get_bits(6..8) as u8;
    let type_: u8 = opcode.get_bits(4..6) as u8;
    let (shift_t, shift_n) = decode_imm_shift(type_, (imm3 << 2) + imm2);

    Instruction::TST_reg {
        params: Reg2ShiftNoSetFlagsParams {
            rm: opcode.get_bits(0..4).into(),
            rn: opcode.get_bits(16..20).into(),
            shift_t,
            shift_n,
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
pub fn decode_TST_imm_t1(opcode: u32) -> Instruction {
    let imm8: u8 = opcode.get_bits(0..8) as u8;
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let i: u8 = u8::from(opcode.get_bit(26));

    let params = [i, imm3, imm8];
    let lengths = [1, 3, 8];

    Instruction::TST_imm {
        params: RegImmCarryNoSetFlagsParams {
            rn: opcode.get_bits(16..20).into(),
            imm32: Imm32Carry::Carry {
                imm32_c0: thumb_expand_imm_c(&params, &lengths, false),
                imm32_c1: thumb_expand_imm_c(&params, &lengths, true),
            },
        },
    }
}
