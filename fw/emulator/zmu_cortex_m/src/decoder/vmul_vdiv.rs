use crate::core::{
    bits::Bits,
    instruction::{Instruction, VAddSubParamsf32, VAddSubParamsf64},
    register::{DoubleReg, SingleReg},
};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VMUL_t1(opcode: u32) -> Instruction {
    let d = u8::from(opcode.get_bit(22));
    let n = opcode.get_bits(16..20) as u8;
    let vd = opcode.get_bits(12..16) as u8;
    let sz = opcode.get_bit(8);
    let n_low = u8::from(opcode.get_bit(7));
    let m_low = u8::from(opcode.get_bit(5));
    let vm = opcode.get_bits(0..4) as u8;

    if sz {
        Instruction::VMUL_f64 {
            params: VAddSubParamsf64 {
                dd: DoubleReg::from(d << 4 | vd),
                dn: DoubleReg::from(n_low << 4 | n),
                dm: DoubleReg::from(m_low << 4 | vm),
            },
        }
    } else {
        Instruction::VMUL_f32 {
            params: VAddSubParamsf32 {
                sd: SingleReg::from(vd << 1 | d),
                sn: SingleReg::from(n << 1 | n_low),
                sm: SingleReg::from(vm << 1 | m_low),
            },
        }
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VNMUL_t1(opcode: u32) -> Instruction {
    let d = u8::from(opcode.get_bit(22));
    let n = opcode.get_bits(16..20) as u8;
    let vd = opcode.get_bits(12..16) as u8;
    let sz = opcode.get_bit(8);
    let n_low = u8::from(opcode.get_bit(7));
    let m_low = u8::from(opcode.get_bit(5));
    let vm = opcode.get_bits(0..4) as u8;

    if sz {
        Instruction::VNMUL_f64 {
            params: VAddSubParamsf64 {
                dd: DoubleReg::from(d << 4 | vd),
                dn: DoubleReg::from(n_low << 4 | n),
                dm: DoubleReg::from(m_low << 4 | vm),
            },
        }
    } else {
        Instruction::VNMUL_f32 {
            params: VAddSubParamsf32 {
                sd: SingleReg::from(vd << 1 | d),
                sn: SingleReg::from(n << 1 | n_low),
                sm: SingleReg::from(vm << 1 | m_low),
            },
        }
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VDIV_t1(opcode: u32) -> Instruction {
    let d = u8::from(opcode.get_bit(22));
    let n = opcode.get_bits(16..20) as u8;
    let vd = opcode.get_bits(12..16) as u8;
    let sz = opcode.get_bit(8);
    let n_low = u8::from(opcode.get_bit(7));
    let m_low = u8::from(opcode.get_bit(5));
    let vm = opcode.get_bits(0..4) as u8;

    if sz {
        Instruction::VDIV_f64 {
            params: VAddSubParamsf64 {
                dd: DoubleReg::from(d << 4 | vd),
                dn: DoubleReg::from(n_low << 4 | n),
                dm: DoubleReg::from(m_low << 4 | vm),
            },
        }
    } else {
        Instruction::VDIV_f32 {
            params: VAddSubParamsf32 {
                sd: SingleReg::from(vd << 1 | d),
                sn: SingleReg::from(n << 1 | n_low),
                sm: SingleReg::from(vm << 1 | m_low),
            },
        }
    }
}
