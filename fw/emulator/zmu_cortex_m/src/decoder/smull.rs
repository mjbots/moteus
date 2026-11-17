use crate::core::bits::Bits;
use crate::core::instruction::{Instruction, Reg643232Params};

#[allow(non_snake_case)]
pub fn decode_SMULL_t1(opcode: u32) -> Instruction {
    Instruction::SMULL {
        params: Reg643232Params {
            rm: opcode.get_bits(0..4).into(),
            rdlo: opcode.get_bits(12..16).into(),
            rdhi: opcode.get_bits(8..12).into(),
            rn: opcode.get_bits(16..20).into(),
        },
    }
}
