use crate::core::instruction::{Instruction, MsrParams};
use crate::core::{bits::Bits, register::Reg};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_MSR_reg_t1(opcode: u32) -> Instruction {
    let reg_rn: u8 = opcode.get_bits(16..20) as u8;
    Instruction::MSR_reg {
        params: MsrParams {
            rn: Reg::from(reg_rn),
            sysm: opcode.get_bits(0..8) as u8,
            mask: opcode.get_bits(10..12) as u8,
        },
    }
}
