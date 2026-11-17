use crate::core::bits::Bits;
use crate::core::instruction::{BfxParams, Instruction};

#[allow(non_snake_case)]
pub fn decode_SBFX_t1(opcode: u32) -> Instruction {
    Instruction::SBFX {
        params: BfxParams {
            rd: From::from(opcode.get_bits(8..12) as u8),
            rn: From::from(opcode.get_bits(16..20) as u8),
            lsb: (opcode.get_bits(6..8) + (opcode.get_bits(12..15) << 2)) as usize,
            widthminus1: opcode.get_bits(0..5) as usize,
        },
    }
}
