use crate::core::{
    bits::Bits,
    instruction::{Instruction, VAddSubParamsf32, VAddSubParamsf64},
    register::{DoubleReg, SingleReg},
};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VADD_t1(opcode: u32) -> Instruction {
    let D = u8::from(opcode.get_bit(22));
    let vn = opcode.get_bits(16..20) as u8;
    let vd = opcode.get_bits(12..16) as u8;
    let sz = opcode.get_bit(8);
    let N = u8::from(opcode.get_bit(7));
    let M = u8::from(opcode.get_bit(5));
    let vm = opcode.get_bits(0..4) as u8;

    if sz {
        Instruction::VADD_f64 {
            params: VAddSubParamsf64 {
                dd: DoubleReg::from(D << 4 | vd),
                dn: DoubleReg::from(N << 4 | vn),
                dm: DoubleReg::from(M << 4 | vm),
            },
        }
    } else {
        Instruction::VADD_f32 {
            params: VAddSubParamsf32 {
                sd: SingleReg::from(vd << 1 | D),
                sn: SingleReg::from(vn << 1 | N),
                sm: SingleReg::from(vm << 1 | M),
            },
        }
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VSUB_t1(opcode: u32) -> Instruction {
    let D = u8::from(opcode.get_bit(22));
    let vn = opcode.get_bits(16..20) as u8;
    let vd = opcode.get_bits(12..16) as u8;
    let sz = opcode.get_bit(8);
    let N = u8::from(opcode.get_bit(7));
    let M = u8::from(opcode.get_bit(5));
    let vm = opcode.get_bits(0..4) as u8;

    if sz {
        Instruction::VSUB_f64 {
            params: VAddSubParamsf64 {
                dd: DoubleReg::from(D << 4 | vd),
                dn: DoubleReg::from(N << 4 | vn),
                dm: DoubleReg::from(M << 4 | vm),
            },
        }
    } else {
        Instruction::VSUB_f32 {
            params: VAddSubParamsf32 {
                sd: SingleReg::from(vd << 1 | D),
                sn: SingleReg::from(vn << 1 | N),
                sm: SingleReg::from(vm << 1 | M),
            },
        }
    }
}
