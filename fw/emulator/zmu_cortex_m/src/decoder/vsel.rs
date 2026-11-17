use crate::core::{
    bits::Bits,
    condition::Condition,
    instruction::{Instruction, VSelParamsf32, VSelParamsf64},
    register::{DoubleReg, SingleReg},
};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VSEL_t1(opcode: u32) -> Instruction {
    let d = u8::from(opcode.get_bit(22));
    let cc: u8 = opcode.get_bits(20..22) as u8;
    let vn = opcode.get_bits(16..20) as u8;
    let vd = opcode.get_bits(12..16) as u8;
    let sz: bool = opcode.get_bit(8);
    let n = u8::from(opcode.get_bit(7));
    let m = u8::from(opcode.get_bit(5));
    let vm = opcode.get_bits(0..4) as u8;

    let cond_code = (cc << 2) + ((((cc & 0b10) >> 1) ^ (cc & 0b01)) << 1);
    let cond = Condition::from_u16(u16::from(cond_code)).expect("must be valid code");

    if sz {
        Instruction::VSEL_f64 {
            params: VSelParamsf64 {
                dd: DoubleReg::from(d << 4 | vd),
                dn: DoubleReg::from(n << 4 | vn),
                dm: DoubleReg::from(m << 4 | vm),
                cond,
            },
        }
    } else {
        Instruction::VSEL_f32 {
            params: VSelParamsf32 {
                sd: SingleReg::from((vd << 1) | d),
                sn: SingleReg::from((vn << 1) | n),
                sm: SingleReg::from((vm << 1) | m),
                cond,
            },
        }
    }
}
