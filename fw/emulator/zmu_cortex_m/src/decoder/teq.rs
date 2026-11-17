use crate::core::bits::Bits;
use crate::core::instruction::Instruction;
use crate::core::operation::decode_imm_shift;
use crate::core::register::Reg;

use crate::core::instruction::{
    Imm32Carry, Reg2ShiftNoSetFlagsParams, RegImmCarryNoSetFlagsParams,
};
use crate::core::operation::thumb_expand_imm_c;

#[allow(non_snake_case)]
pub fn decode_TEQ_reg_t1(opcode: u32) -> Instruction {
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm2: u8 = opcode.get_bits(6..8) as u8;
    let type_: u8 = opcode.get_bits(4..6) as u8;

    let (shift_t, shift_n) = decode_imm_shift(type_, (imm3 << 2) + imm2);
    Instruction::TEQ_reg {
        params: Reg2ShiftNoSetFlagsParams {
            rm: Reg::from(opcode.get_bits(0..4)),
            rn: Reg::from(opcode.get_bits(16..20)),
            shift_t,
            shift_n,
        },
    }
}

#[allow(non_snake_case)]
pub fn decode_TEQ_imm_t1(opcode: u32) -> Instruction {
    let rn: u8 = opcode.get_bits(16..20) as u8;

    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm8: u8 = opcode.get_bits(0..8) as u8;
    let i: u8 = u8::from(opcode.get_bit(26));

    let params = [i, imm3, imm8];
    let lengths = [1, 3, 8];

    Instruction::TEQ_imm {
        params: RegImmCarryNoSetFlagsParams {
            rn: Reg::from(rn),
            imm32: Imm32Carry::Carry {
                imm32_c0: thumb_expand_imm_c(&params, &lengths, false),
                imm32_c1: thumb_expand_imm_c(&params, &lengths, true),
            },
        },
    }
}
