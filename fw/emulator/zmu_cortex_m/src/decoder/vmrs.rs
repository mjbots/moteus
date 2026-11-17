use crate::core::{
    bits::Bits,
    instruction::{Instruction, VMRSTarget},
    register::Reg,
};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VMRS(opcode: u32) -> Instruction {
    let target = opcode.get_bits(12..16) as u8;

    let rt = if target == 15 {
        VMRSTarget::APSRNZCV
    } else {
        VMRSTarget::Register(Reg::from(target))
    };

    Instruction::VMRS { rt }
}
