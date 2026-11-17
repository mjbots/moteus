use crate::core::bits::Bits;
use crate::core::instruction::Instruction;

#[allow(non_snake_case)]
pub fn decode_USAT_t1(opcode: u32) -> Instruction {
    let imm3 = opcode.get_bits(12..15) as u8;
    let imm2 = opcode.get_bits(6..8) as u8;
    Instruction::USAT {
        rd: opcode.get_bits(8..12).into(),
        rn: opcode.get_bits(16..20).into(),
        sat_to: opcode.get_bits(0..5) as u8,
        asr: opcode.get_bit(21),
        shift_n: (imm3 << 2) | imm2,
    }
}
