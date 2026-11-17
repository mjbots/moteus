use crate::core::{
    bits::Bits,
    instruction::{Instruction, VCmpParamsf32, VCmpParamsf64},
    register::{DoubleReg, SingleReg},
};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VCMP_t1(opcode: u32) -> Instruction {
    let sz = opcode.get_bit(8);

    let D = u8::from(opcode.get_bit(22));
    let vd = opcode.get_bits(12..16) as u8;
    let vm = opcode.get_bits(0..4) as u8;
    let M = u8::from(opcode.get_bit(5));
    let e = u8::from(opcode.get_bit(7));

    let quiet_nan_exc = e == 1;
    let with_zero = false;

    if sz {
        Instruction::VCMP_f64 {
            params: VCmpParamsf64 {
                dd: DoubleReg::from(D << 4 | vd),
                dm: DoubleReg::from(M << 4 | vm),
                quiet_nan_exc,
                with_zero,
            },
        }
    } else {
        Instruction::VCMP_f32 {
            params: VCmpParamsf32 {
                sd: SingleReg::from(vd << 1 | D),
                sm: SingleReg::from(vm << 1 | M),
                quiet_nan_exc,
                with_zero,
            },
        }
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VCMP_t2(opcode: u32) -> Instruction {
    let sz = opcode.get_bit(8);

    let D = u8::from(opcode.get_bit(22));
    let vd = opcode.get_bits(12..16) as u8;
    let vm = opcode.get_bits(0..4) as u8;
    let M = u8::from(opcode.get_bit(5));
    let e = u8::from(opcode.get_bit(7));

    let quiet_nan_exc = e == 1;
    let with_zero = true;

    if sz {
        Instruction::VCMP_f64 {
            params: VCmpParamsf64 {
                dd: DoubleReg::from(D << 4 | vd),
                dm: DoubleReg::from(M << 4 | vm),
                quiet_nan_exc,
                with_zero,
            },
        }
    } else {
        Instruction::VCMP_f32 {
            params: VCmpParamsf32 {
                sd: SingleReg::from(vd << 1 | D),
                sm: SingleReg::from(vm << 1 | M),
                quiet_nan_exc,
                with_zero,
            },
        }
    }
}
