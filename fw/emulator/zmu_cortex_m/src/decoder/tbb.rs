use crate::core::bits::Bits;
use crate::core::instruction::{Instruction, Reg2RnRmParams};
use crate::core::register::Reg;

#[allow(non_snake_case)]
pub fn decode_TBB_t1(opcode: u32) -> Instruction {
    let rn = opcode.get_bits(16..20);
    let rm = opcode.get_bits(0..4);

    Instruction::TBB {
        params: Reg2RnRmParams {
            rn: Reg::from(rn),
            rm: Reg::from(rm),
        },
    }
}
