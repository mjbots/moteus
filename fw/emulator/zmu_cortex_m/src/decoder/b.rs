use crate::core::condition::Condition;
use crate::core::instruction::{CondBranchParams, Instruction};
use crate::core::operation::build_imm_6_11;
use crate::core::operation::build_imm_10_11;
use crate::core::{bits::Bits, operation::sign_extend};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_B_t1_SVC_t1(opcode: u16) -> Instruction {
    let cond = opcode.get_bits(8..12);
    if cond == 0b1111 {
        return Instruction::SVC {
            imm32: u32::from(opcode.get_bits(0..8)),
        };
    }
    if cond == 0b1110 {
        return Instruction::UDF {
            imm32: 0,
            opcode: opcode.into(),
            thumb32: false,
        };
    }

    Instruction::B_t13 {
        params: CondBranchParams {
            cond: Condition::from_u16(cond).unwrap(),
            imm32: sign_extend(u32::from(opcode.get_bits(0..8)) << 1, 8, 32) as i32,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_B_t2(opcode: u16) -> Instruction {
    Instruction::B_t24 {
        imm32: sign_extend(u32::from(opcode.get_bits(0..11)) << 1, 11, 32) as i32,
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_B_t3(opcode: u32) -> Instruction {
    let cond: u16 = opcode.get_bits(22..26) as u16;

    let imm = build_imm_6_11(opcode);

    match Condition::from_u16(cond) {
        Some(c) => Instruction::B_t13 {
            params: CondBranchParams {
                cond: c,
                imm32: imm,
            },
            thumb32: true,
        },
        None => Instruction::B_t24 {
            imm32: imm,
            thumb32: true,
        },
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_B_t4(opcode: u32) -> Instruction {
    let imm = build_imm_10_11(opcode);

    Instruction::B_t24 {
        imm32: imm,
        thumb32: true,
    }
}
