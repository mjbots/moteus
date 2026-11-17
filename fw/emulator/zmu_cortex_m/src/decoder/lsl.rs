use crate::core::bits::Bits;
use crate::core::instruction::Instruction;
use crate::core::instruction::{Reg2ShiftNParams, Reg3Params, SetFlags};
use crate::core::operation::decode_imm_shift;
use crate::core::register::Reg;

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_LSL_reg_t1(opcode: u16) -> Instruction {
    Instruction::LSL_reg {
        params: Reg3Params {
            rd: opcode.get_bits(0..3).into(),
            rn: opcode.get_bits(0..3).into(),
            rm: opcode.get_bits(3..6).into(),
            setflags: SetFlags::NotInITBlock,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_LSL_reg_t2(opcode: u32) -> Instruction {
    Instruction::LSL_reg {
        params: Reg3Params {
            rd: opcode.get_bits(8..12).into(),
            rn: opcode.get_bits(16..20).into(),
            rm: opcode.get_bits(0..4).into(),
            setflags: if opcode.get_bit(20) {
                SetFlags::True
            } else {
                SetFlags::False
            },
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
pub fn decode_LSL_imm_t2(opcode: u32) -> Instruction {
    let rm: u8 = opcode.get_bits(0..4) as u8;
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let s: u8 = u8::from(opcode.get_bit(20));

    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm2: u8 = opcode.get_bits(6..8) as u8;

    let (_, shift_n) = decode_imm_shift(0b_00, (imm3 << 2) + imm2);

    Instruction::LSL_imm {
        params: Reg2ShiftNParams {
            rd: Reg::from(rd),
            rm: Reg::from(rm),
            setflags: if s == 1 {
                SetFlags::True
            } else {
                SetFlags::False
            },
            shift_n,
        },
        thumb32: true,
    }
}
