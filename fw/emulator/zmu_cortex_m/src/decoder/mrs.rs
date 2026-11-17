use crate::core::{
    bits::Bits,
    instruction::{Instruction, MrsParams},
    register::Reg,
};

#[allow(non_snake_case)]
pub fn decode_MRS_t1(opcode: u32) -> Instruction {
    let reg_rd: u8 = opcode.get_bits(8..13) as u8;
    Instruction::MRS {
        params: MrsParams {
            rd: Reg::from(reg_rd),
            sysm: opcode.get_bits(0..8) as u8,
        },
    }
}
