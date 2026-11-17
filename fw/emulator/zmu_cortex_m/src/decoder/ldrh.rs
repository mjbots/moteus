use crate::core::bits::Bits;
use crate::core::instruction::Instruction;
use crate::core::instruction::{Reg2FullParams, Reg3FullParams, SRType};
use crate::core::register::Reg;

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_LDRH_reg_t1(opcode: u16) -> Instruction {
    Instruction::LDRH_reg {
        params: Reg3FullParams {
            rt: Reg::from(opcode.get_bits(0..3) as u8),
            rn: Reg::from(opcode.get_bits(3..6) as u8),
            rm: Reg::from(opcode.get_bits(6..9) as u8),
            shift_t: SRType::LSL,
            shift_n: 0,
            index: true,
            add: true,
            wback: false,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_LDRH_reg_t2(opcode: u32) -> Instruction {
    Instruction::LDRH_reg {
        params: Reg3FullParams {
            rm: Reg::from(opcode.get_bits(0..4) as u8),
            rt: Reg::from(opcode.get_bits(12..16) as u8),
            rn: Reg::from(opcode.get_bits(16..20) as u8),
            shift_t: SRType::LSL,
            shift_n: opcode.get_bits(4..6) as u8,
            index: true,
            add: true,
            wback: false,
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_LDRH_imm_t1(opcode: u16) -> Instruction {
    Instruction::LDRH_imm {
        params: Reg2FullParams {
            rt: Reg::from(opcode.get_bits(0..3) as u8),
            rn: Reg::from(opcode.get_bits(3..6) as u8),
            imm32: u32::from(opcode.get_bits(6..11) as u8) << 1,
            index: true,
            add: true,
            wback: false,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_LDRH_imm_t2(opcode: u32) -> Instruction {
    Instruction::LDRH_imm {
        params: Reg2FullParams {
            rt: From::from(opcode.get_bits(12..16) as u8),
            rn: From::from(opcode.get_bits(16..20) as u8),
            imm32: opcode.get_bits(0..12),
            index: true,
            add: true,
            wback: false,
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
pub fn decode_LDRH_imm_t3(opcode: u32) -> Instruction {
    Instruction::LDRH_imm {
        params: Reg2FullParams {
            rt: From::from(opcode.get_bits(12..16) as u8),
            rn: From::from(opcode.get_bits(16..20) as u8),
            imm32: opcode.get_bits(0..8),
            index: opcode.get_bit(10),
            add: opcode.get_bit(9),
            wback: opcode.get_bit(8),
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
pub fn decode_LDRH_lit_t1(opcode: u32) -> Instruction {
    Instruction::UDF {
        imm32: 0,
        opcode: opcode.into(),
        thumb32: true,
    }
}
