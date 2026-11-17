use crate::core::instruction::Instruction;

#[allow(non_snake_case)]
pub fn decode_YIELD_t1(_opcode: u16) -> Instruction {
    Instruction::YIELD { thumb32: false }
}

#[allow(non_snake_case)]
pub fn decode_YIELD_t2(_opcode: u32) -> Instruction {
    Instruction::YIELD { thumb32: true }
}
