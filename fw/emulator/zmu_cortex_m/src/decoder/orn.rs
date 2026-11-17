use crate::core::bits::Bits;
use crate::core::instruction::{Instruction, Reg3ShiftParams, SetFlags};
use crate::core::operation::decode_imm_shift;

#[allow(non_snake_case)]
pub fn decode_ORN_reg_t1(opcode: u32) -> Instruction {
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm2: u8 = opcode.get_bits(6..8) as u8;
    let type_: u8 = opcode.get_bits(4..6) as u8;

    let (shift_t, shift_n) = decode_imm_shift(type_, (imm3 << 2) + imm2);
    let s = opcode.get_bit(20);

    Instruction::ORN_reg {
        params: Reg3ShiftParams {
            rd: opcode.get_bits(8..12).into(),
            rn: opcode.get_bits(16..20).into(),
            rm: opcode.get_bits(0..4).into(),
            setflags: if s { SetFlags::True } else { SetFlags::False },
            shift_t,
            shift_n,
        },
    }
}

#[allow(non_snake_case)]
pub fn decode_ORN_imm_t1(opcode: u32) -> Instruction {
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let rn: u8 = opcode.get_bits(16..20) as u8;
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm8: u8 = opcode.get_bits(0..8) as u8;
    let i: u8 = u8::from(opcode.get_bit(26));
    let params = [i, imm3, imm8];
    let lengths = [1, 3, 8];
    Instruction::ORN_imm {
        params: crate::core::instruction::Reg2ImmCarryParams {
            rd: crate::core::register::Reg::from(rd),
            rn: crate::core::register::Reg::from(rn),
            imm32: crate::core::instruction::Imm32Carry::Carry {
                imm32_c0: crate::core::operation::thumb_expand_imm_c(&params, &lengths, false),
                imm32_c1: crate::core::operation::thumb_expand_imm_c(&params, &lengths, true),
            },
            setflags: opcode.get_bit(20),
        },
    }
}
