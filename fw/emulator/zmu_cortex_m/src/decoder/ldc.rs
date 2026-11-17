use crate::core::{bits::Bits, instruction::Instruction, register::Reg};

#[allow(non_snake_case)]
pub fn decode_LDC_imm_t1(opcode: u32) -> Instruction {
    let reg_rn: u8 = opcode.get_bits(16..20) as u8;
    Instruction::LDC_imm {
        coproc: opcode.get_bits(8..12) as u8,
        imm32: opcode.get_bits(0..8),
        crd: opcode.get_bits(12..16) as u8,
        rn: Reg::from(reg_rn),
    }
}

#[allow(non_snake_case)]
pub fn decode_LDC2_imm_t2(opcode: u32) -> Instruction {
    let reg_rn: u8 = opcode.get_bits(16..20) as u8;
    Instruction::LDC2_imm {
        coproc: opcode.get_bits(8..12) as u8,
        imm32: opcode.get_bits(0..8),
        crd: opcode.get_bits(12..16) as u8,
        rn: Reg::from(reg_rn),
    }
}

#[allow(non_snake_case)]
pub fn decode_LDC_lit_t1(opcode: u32) -> Instruction {
    Instruction::UDF {
        imm32: 0,
        opcode: opcode.into(),
        thumb32: true,
    }
}

#[allow(non_snake_case)]
pub fn decode_LDC2_lit_t2(opcode: u32) -> Instruction {
    Instruction::UDF {
        imm32: 0,
        opcode: opcode.into(),
        thumb32: true,
    }
}
