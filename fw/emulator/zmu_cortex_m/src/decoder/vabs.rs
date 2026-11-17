use crate::core::{
    bits::Bits,
    instruction::{Instruction, VMovRegParamsf32, VMovRegParamsf64},
    register::{DoubleReg, SingleReg},
};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VABS_t1(opcode: u32) -> Instruction {
    let sz = opcode.get_bit(8);

    let D = u8::from(opcode.get_bit(22));
    let vd = opcode.get_bits(12..16) as u8;
    let vm = opcode.get_bits(0..4) as u8;
    let M = u8::from(opcode.get_bit(5));

    if sz {
        Instruction::VABS_f64 {
            params: VMovRegParamsf64 {
                dd: DoubleReg::from(D << 4 | vd),
                dm: DoubleReg::from(M << 4 | vm),
            },
        }
    } else {
        Instruction::VABS_f32 {
            params: VMovRegParamsf32 {
                sd: SingleReg::from(vd << 1 | D),
                sm: SingleReg::from(vm << 1 | M),
            },
        }
    }
}
