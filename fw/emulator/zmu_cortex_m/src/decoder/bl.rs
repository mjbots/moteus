use crate::core::instruction::Instruction;
use crate::core::operation::build_imm_10_11;

#[inline(always)]
#[allow(non_snake_case)]
pub fn decode_BL_t1(opcode: u32) -> Instruction {
    let imm = build_imm_10_11(opcode);

    Instruction::BL { imm32: imm }
}
