use crate::core::bits::Bits;
use crate::core::instruction::Instruction;
use crate::core::instruction::{Reg2DoubleParams, Reg2FullParams, Reg3FullParams, SRType};
use crate::core::register::Reg;

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_STR_imm_t1(opcode: u16) -> Instruction {
    Instruction::STR_imm {
        params: Reg2FullParams {
            rt: Reg::from(opcode.get_bits(0..3) as u8),
            rn: Reg::from(opcode.get_bits(3..6) as u8),
            imm32: u32::from(opcode.get_bits(6..11)) << 2,
            index: true,
            add: true,
            wback: false,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_STR_imm_t2(opcode: u16) -> Instruction {
    Instruction::STR_imm {
        params: Reg2FullParams {
            rt: From::from(opcode.get_bits(8..11) as u8),
            rn: Reg::SP,
            imm32: u32::from(opcode.get_bits(0..8)) << 2,
            index: true,
            add: true,
            wback: false,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_STR_reg_t1(opcode: u16) -> Instruction {
    Instruction::STR_reg {
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
pub fn decode_STRB_imm_t1(opcode: u16) -> Instruction {
    Instruction::STRB_imm {
        params: Reg2FullParams {
            rt: Reg::from(opcode.get_bits(0..3) as u8),
            rn: Reg::from(opcode.get_bits(3..6) as u8),
            imm32: u32::from(opcode.get_bits(6..11)),
            index: true,
            add: true,
            wback: false,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_STRB_reg_t1(opcode: u16) -> Instruction {
    Instruction::STRB_reg {
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
pub fn decode_STRH_imm_t1(opcode: u16) -> Instruction {
    Instruction::STRH_imm {
        params: Reg2FullParams {
            rt: Reg::from(opcode.get_bits(0..3) as u8),
            rn: Reg::from(opcode.get_bits(3..6) as u8),
            imm32: u32::from(opcode.get_bits(6..11)) << 1,
            index: true,
            add: true,
            wback: false,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_STRH_reg_t1(opcode: u16) -> Instruction {
    Instruction::STRH_reg {
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
#[inline(always)]
pub fn decode_STRH_reg_t2(opcode: u32) -> Instruction {
    Instruction::STRH_reg {
        params: Reg3FullParams {
            rt: Reg::from(opcode.get_bits(12..16) as u8),
            rn: Reg::from(opcode.get_bits(16..20) as u8),
            rm: Reg::from(opcode.get_bits(0..4) as u8),
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
pub fn decode_STRB_imm_t2(opcode: u32) -> Instruction {
    Instruction::STRB_imm {
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
pub fn decode_STRB_imm_t3(opcode: u32) -> Instruction {
    Instruction::STRB_imm {
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
pub fn decode_STRB_reg_t2(opcode: u32) -> Instruction {
    Instruction::STRB_reg {
        params: Reg3FullParams {
            rt: From::from(opcode.get_bits(12..16) as u8),
            rn: From::from(opcode.get_bits(16..20) as u8),
            rm: From::from(opcode.get_bits(0..4) as u8),
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
pub fn decode_STRH_imm_t2(opcode: u32) -> Instruction {
    Instruction::STRH_imm {
        params: Reg2FullParams {
            rt: Reg::from(opcode.get_bits(12..16) as u8),
            rn: Reg::from(opcode.get_bits(16..20) as u8),
            imm32: opcode.get_bits(0..12),
            index: true,
            add: true,
            wback: false,
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
pub fn decode_STRH_imm_t3(opcode: u32) -> Instruction {
    Instruction::STRH_imm {
        params: Reg2FullParams {
            rt: Reg::from(opcode.get_bits(12..16) as u8),
            rn: Reg::from(opcode.get_bits(16..20) as u8),
            imm32: opcode.get_bits(0..8),
            wback: opcode.get_bit(8),
            add: opcode.get_bit(9),
            index: opcode.get_bit(10),
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
pub fn decode_STR_imm_t3(opcode: u32) -> Instruction {
    // ARMv7-M
    Instruction::STR_imm {
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
pub fn decode_STR_imm_t4(opcode: u32) -> Instruction {
    // ARMv7-M
    Instruction::STR_imm {
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
pub fn decode_STR_reg_t2(opcode: u32) -> Instruction {
    Instruction::STR_reg {
        params: Reg3FullParams {
            rt: From::from(opcode.get_bits(12..16) as u8),
            rn: From::from(opcode.get_bits(16..20) as u8),
            rm: From::from(opcode.get_bits(0..4) as u8),
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
pub fn decode_STRD_imm_t1(opcode: u32) -> Instruction {
    Instruction::STRD_imm {
        params: Reg2DoubleParams {
            rt2: From::from(opcode.get_bits(8..12) as u8),
            rt: From::from(opcode.get_bits(12..16) as u8),
            rn: From::from(opcode.get_bits(16..20) as u8),
            imm32: opcode.get_bits(0..8) << 2,
            index: opcode.get_bit(24),
            add: opcode.get_bit(23),
            wback: opcode.get_bit(21),
        },
    }
}
