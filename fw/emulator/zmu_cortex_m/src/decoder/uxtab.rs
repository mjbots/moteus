use crate::core::bits::Bits;
use crate::core::instruction::{Instruction, Reg3UsizeParams};

#[allow(non_snake_case)]
pub fn decode_UXTAB_t1(opcode: u32) -> Instruction {
    Instruction::UXTAB {
        params: Reg3UsizeParams {
            rd: opcode.get_bits(8..12).into(),
            rn: opcode.get_bits(16..20).into(),
            rm: opcode.get_bits(0..4).into(),
            rotation: (opcode.get_bits(4..6) << 3) as usize,
        },
    }
}

fn ext_add_params(opcode: u32) -> Reg3UsizeParams {
    Reg3UsizeParams {
        rd: opcode.get_bits(8..12).into(),
        rn: opcode.get_bits(16..20).into(),
        rm: opcode.get_bits(0..4).into(),
        rotation: (opcode.get_bits(4..6) << 3) as usize,
    }
}

#[allow(non_snake_case)]
pub fn decode_UXTAH_t1(opcode: u32) -> Instruction {
    Instruction::UXTAH { params: ext_add_params(opcode) }
}

#[allow(non_snake_case)]
pub fn decode_SXTAH_t1(opcode: u32) -> Instruction {
    Instruction::SXTAH { params: ext_add_params(opcode) }
}

#[allow(non_snake_case)]
pub fn decode_SXTAB_t1(opcode: u32) -> Instruction {
    Instruction::SXTAB { params: ext_add_params(opcode) }
}
