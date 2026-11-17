use crate::core::bits::Bits;
use crate::core::instruction::{Instruction, Reg2RdRmParams};
use crate::core::register::Reg;

#[allow(non_snake_case)]
pub fn decode_CLZ_t1(opcode: u32) -> Instruction {
    Instruction::CLZ {
        params: Reg2RdRmParams {
            rd: Reg::from(opcode.get_bits(8..12) as u8),
            rm: Reg::from(opcode.get_bits(0..4) as u8),
        },
    }
}
