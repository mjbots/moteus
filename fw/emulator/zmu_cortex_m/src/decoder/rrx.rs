use crate::core::bits::Bits;
use crate::core::instruction::{Instruction, Reg2Params, SetFlags};

#[allow(non_snake_case)]
pub fn decode_RRX_t1(opcode: u32) -> Instruction {
    let rm: u8 = opcode.get_bits(0..4) as u8;
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let s: u8 = u8::from(opcode.get_bit(20));

    Instruction::RRX {
        params: Reg2Params {
            rd: rd.into(),
            rm: rm.into(),
            setflags: if s == 1 {
                SetFlags::True
            } else {
                SetFlags::False
            },
        },
    }
}
