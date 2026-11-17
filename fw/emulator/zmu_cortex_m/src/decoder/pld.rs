use crate::core::bits::Bits;
use crate::core::instruction::Instruction;
use crate::core::instruction::SRType;

#[allow(non_snake_case)]
pub fn decode_PLD_imm_t1(opcode: u32) -> Instruction {
    Instruction::PLD_imm {
        rn: opcode.get_bits(16..20).into(),
        imm32: opcode.get_bits(0..12),
        add: true,
    }
}

#[allow(non_snake_case)]
pub fn decode_PLD_imm_t2(opcode: u32) -> Instruction {
    Instruction::PLD_imm {
        rn: opcode.get_bits(16..20).into(),
        imm32: opcode.get_bits(0..8),
        add: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_PLD_reg_t1(opcode: u32) -> Instruction {
    Instruction::PLD_reg {
        rm: opcode.get_bits(0..4).into(),
        rn: opcode.get_bits(16..20).into(),
        shift_t: SRType::LSL,
        shift_n: opcode.get_bits(4..6) as u8,
    }
}

#[allow(non_snake_case)]
pub fn decode_PLD_lit_t1(opcode: u32) -> Instruction {
    Instruction::PLD_lit {
        imm32: opcode.get_bits(0..12),
        add: opcode.get_bit(23),
    }
}
