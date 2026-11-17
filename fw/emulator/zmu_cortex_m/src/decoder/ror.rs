use crate::core::bits::Bits;
use crate::core::instruction::Instruction;
use crate::core::instruction::{Reg2ShiftNParams, Reg3Params, SetFlags};
use crate::core::operation::decode_imm_shift;
use crate::core::register::Reg;

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_ROR_reg_t1(opcode: u16) -> Instruction {
    Instruction::ROR_reg {
        params: Reg3Params {
            rd: Reg::from(opcode.get_bits(0..3) as u8),
            rn: Reg::from(opcode.get_bits(0..3) as u8),
            rm: Reg::from(opcode.get_bits(3..6) as u8),
            setflags: SetFlags::NotInITBlock,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_ROR_imm_t1(opcode: u32) -> Instruction {
    let rm: u8 = opcode.get_bits(0..4) as u8;
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let s = opcode.get_bit(20);

    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm2: u8 = opcode.get_bits(6..8) as u8;

    let (_, shift_n) = decode_imm_shift(0b11, (imm3 << 2) + imm2);

    Instruction::ROR_imm {
        params: Reg2ShiftNParams {
            rd: Reg::from(rd),
            rm: Reg::from(rm),
            setflags: if s { SetFlags::True } else { SetFlags::False },
            shift_n,
        },
    }
}

#[allow(non_snake_case)]
pub fn decode_ROR_reg_t2(opcode: u32) -> Instruction {
    let s = opcode.get_bit(20);
    Instruction::ROR_reg {
        params: Reg3Params {
            rd: Reg::from(opcode.get_bits(8..12) as u8),
            rn: Reg::from(opcode.get_bits(16..20) as u8),
            rm: Reg::from(opcode.get_bits(0..4) as u8),
            setflags: if s { SetFlags::True } else { SetFlags::False },
        },
        thumb32: true,
    }
}
