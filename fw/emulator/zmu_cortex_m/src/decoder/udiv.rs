use crate::core::{
    bits::Bits,
    instruction::{Instruction, Reg3NoSetFlagsParams},
    register::Reg,
};

#[allow(non_snake_case)]
pub fn decode_UDIV_t1(opcode: u32) -> Instruction {
    let reg_rm: u8 = opcode.get_bits(0..4) as u8;
    let reg_rd: u8 = opcode.get_bits(8..12) as u8;
    let reg_rn: u8 = opcode.get_bits(16..20) as u8;
    Instruction::UDIV {
        params: Reg3NoSetFlagsParams {
            rm: Reg::from(reg_rm),
            rd: Reg::from(reg_rd),
            rn: Reg::from(reg_rn),
        },
    }
}
