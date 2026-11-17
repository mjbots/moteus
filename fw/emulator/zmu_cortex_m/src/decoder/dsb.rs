use crate::core::instruction::Instruction;

#[allow(non_snake_case)]
pub fn decode_DSB_t1(_opcode: u32) -> Instruction {
    Instruction::DSB
}
