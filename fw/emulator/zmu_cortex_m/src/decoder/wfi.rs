use crate::core::instruction::Instruction;

#[allow(non_snake_case)]
pub fn decode_WFI_t1(_opcode: u16) -> Instruction {
    Instruction::WFI { thumb32: false }
}

#[allow(non_snake_case)]
pub fn decode_WFI_t2(_opcode: u32) -> Instruction {
    Instruction::WFI { thumb32: true }
}
