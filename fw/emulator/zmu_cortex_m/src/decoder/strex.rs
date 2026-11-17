use crate::core::bits::Bits;
use crate::core::instruction::{Instruction, Reg3RdRtRnImm32Params, Reg3RdRtRnParams};

#[allow(non_snake_case)]
pub fn decode_STREXB_t1(opcode: u32) -> Instruction {
    Instruction::STREXB {
        params: Reg3RdRtRnParams {
            rd: From::from(opcode.get_bits(0..4) as u8),
            rt: From::from(opcode.get_bits(12..16) as u8),
            rn: From::from(opcode.get_bits(16..20) as u8),
        },
    }
}

#[allow(non_snake_case)]
pub fn decode_STREXH_t1(opcode: u32) -> Instruction {
    Instruction::STREXH {
        params: Reg3RdRtRnParams {
            rd: From::from(opcode.get_bits(0..4) as u8),
            rt: From::from(opcode.get_bits(12..16) as u8),
            rn: From::from(opcode.get_bits(16..20) as u8),
        },
    }
}

#[allow(non_snake_case)]
pub fn decode_STREX_t1(opcode: u32) -> Instruction {
    Instruction::STREX {
        params: Reg3RdRtRnImm32Params {
            rd: From::from(opcode.get_bits(8..12) as u8),
            rt: From::from(opcode.get_bits(12..16) as u8),
            rn: From::from(opcode.get_bits(16..20) as u8),
            imm32: opcode.get_bits(0..8) << 2,
        },
    }
}
