use crate::core::bits::Bits;
use crate::core::instruction::{BfcParams, Instruction};
use crate::core::register::Reg;

#[allow(non_snake_case)]
pub fn decode_BFC_t1(opcode: u32) -> Instruction {
    let rd: u8 = opcode.get_bits(8..12) as u8;

    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm2: u8 = opcode.get_bits(6..8) as u8;

    let lsbit = u32::from((imm3 << 2) + imm2);
    let msbit = opcode.get_bits(0..5);

    Instruction::BFC {
        params: BfcParams {
            rd: Reg::from(rd),
            lsbit: lsbit as usize,
            msbit: msbit as usize,
        },
    }
}
