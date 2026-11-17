use crate::core::bits::Bits;
use crate::core::instruction::{Instruction, Reg2RdRmParams};

#[allow(non_snake_case)]
pub fn decode_RBIT_t1(opcode: u32) -> Instruction {
    Instruction::RBIT {
        params: Reg2RdRmParams {
            rd: opcode.get_bits(8..12).into(),
            rm: opcode.get_bits(0..4).into(),
        },
    }
}
