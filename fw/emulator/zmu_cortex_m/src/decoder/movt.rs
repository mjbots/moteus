use crate::core::instruction::{Instruction, MovtParams};
use crate::core::operation::zero_extend;

use crate::core::bits::Bits;

#[allow(non_snake_case)]
pub fn decode_MOVT_t1(opcode: u32) -> Instruction {
    let imm4: u8 = opcode.get_bits(16..20) as u8;
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm8: u8 = opcode.get_bits(0..8) as u8;
    let i: u8 = u8::from(opcode.get_bit(26));

    let params = [imm4, i, imm3, imm8];
    let lengths = [4, 1, 3, 8];

    Instruction::MOVT {
        params: MovtParams {
            rd: opcode.get_bits(8..12).into(),
            imm16: zero_extend(&params, &lengths) as u16,
        },
    }
}
