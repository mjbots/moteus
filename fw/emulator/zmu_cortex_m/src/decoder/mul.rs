use crate::core::bits::Bits;
use crate::core::instruction::Instruction;
use crate::core::instruction::{Reg3Params, SetFlags};
use crate::core::register::Reg;

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_MUL_t1(opcode: u16) -> Instruction {
    Instruction::MUL {
        params: Reg3Params {
            rn: Reg::from(opcode.get_bits(3..6) as u8),
            rd: Reg::from(opcode.get_bits(0..3) as u8),
            rm: Reg::from(opcode.get_bits(0..3) as u8),
            setflags: SetFlags::NotInITBlock,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_MUL_t2(opcode: u32) -> Instruction {
    Instruction::MUL {
        params: Reg3Params {
            rn: Reg::from(opcode.get_bits(16..20) as u8),
            rd: Reg::from(opcode.get_bits(8..12) as u8),
            rm: Reg::from(opcode.get_bits(0..4) as u8),
            setflags: SetFlags::False,
        },
        thumb32: true,
    }
}
