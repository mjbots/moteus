use crate::core::{bits::Bits, instruction::Instruction, register::Reg};

#[allow(non_snake_case)]
pub fn decode_MCR2_t2(opcode: u32) -> Instruction {
    let reg: u8 = opcode.get_bits(12..16) as u8;
    Instruction::MCR2 {
        rt: Reg::from(reg),
        coproc: opcode.get_bits(8..12) as u8,
        opc1: opcode.get_bits(21..24) as u8,
        opc2: opcode.get_bits(5..8) as u8,
        crn: opcode.get_bits(16..20) as u8,
        crm: opcode.get_bits(0..4) as u8,
    }
}

#[allow(non_snake_case)]
pub fn decode_MCR_t1(opcode: u32) -> Instruction {
    let reg: u8 = opcode.get_bits(12..16) as u8;
    Instruction::MCR {
        rt: Reg::from(reg),
        coproc: opcode.get_bits(8..12) as u8,
        opc1: opcode.get_bits(21..24) as u8,
        opc2: opcode.get_bits(5..8) as u8,
        crn: opcode.get_bits(16..20) as u8,
        crm: opcode.get_bits(0..4) as u8,
    }
}

#[allow(non_snake_case)]
pub fn decode_MCRR2_t2(opcode: u32) -> Instruction {
    Instruction::UDF {
        imm32: 0,
        opcode: opcode.into(),
        thumb32: true,
    }
}

#[allow(non_snake_case)]
pub fn decode_MCRR_t1(opcode: u32) -> Instruction {
    Instruction::UDF {
        imm32: 0,
        opcode: opcode.into(),
        thumb32: true,
    }
}

#[allow(non_snake_case)]
pub fn decode_MRC2_t2(opcode: u32) -> Instruction {
    Instruction::UDF {
        imm32: 0,
        opcode: opcode.into(),
        thumb32: true,
    }
}

#[allow(non_snake_case)]
pub fn decode_MRC_t1(opcode: u32) -> Instruction {
    Instruction::UDF {
        imm32: 0,
        opcode: opcode.into(),
        thumb32: true,
    }
}
