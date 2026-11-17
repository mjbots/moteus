use crate::core::bits::Bits;
use crate::core::instruction::{Instruction, ParamsRegImm32};
use crate::core::register::Reg;

#[allow(non_snake_case)]
pub fn decode_CBZ_t1(opcode: u16) -> Instruction {
    let nonzero = opcode.get_bit(11);
    let rn = Reg::from(opcode.get_bits(0..3) as u8);
    let imm32 = ((u32::from(opcode.get_bit(9))) << 6) + (u32::from(opcode.get_bits(3..8)) << 1);

    if nonzero {
        Instruction::CBNZ {
            params: ParamsRegImm32 { rn, imm32 },
        }
    } else {
        Instruction::CBZ {
            params: ParamsRegImm32 { rn, imm32 },
        }
    }
}
