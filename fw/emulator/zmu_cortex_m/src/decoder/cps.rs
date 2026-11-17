use crate::core::{bits::Bits, instruction::Instruction};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_CPS_t1(opcode: u16) -> Instruction {
    Instruction::CPS {
        im: opcode.get_bit(4),
        #[cfg(not(feature = "armv6m"))]
        affect_fault: opcode.get_bit(0),
        #[cfg(not(feature = "armv6m"))]
        affect_pri: opcode.get_bit(1),
    }
}
