use crate::core::{bits::Bits, instruction::Instruction};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_BLX_t1(command: u16) -> Instruction {
    Instruction::BLX {
        rm: From::from(command.get_bits(3..7)),
    }
}
