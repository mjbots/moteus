use crate::core::{
    bits::Bits,
    instruction::{
        Instruction, VMovCr2DpParams, VMovCrSpParams, VMovImmParams32, VMovImmParams64,
        VMovRegParamsf32, VMovRegParamsf64,
    },
    register::{DoubleReg, Reg, SingleReg},
};

// Example:
//
// e = 11
// f = 64 - e - 1 = 52
// bits 7654 3210
// ..............
//      0111 1000
//
// sign = 0
// exp = NOT(imm8[6]) : Replicate(imm8[6], e-3) :imm8[5:4]
//     = NOT(1) : Replicate(1, 8) : 11
//     = 0 : 1111 1111 : 11
//     = 0b01111 1111 11
//     = 0x3ff
//
// frac = imm8[3:0] : Zeros(F-4)
//      = imm8[3:0] : Zeros(48)
//      = 1000 : 48 zeroes
//      = 0x8 0000 0000 0000
//
// result = sign : exp : frac
//        = 0 : 0x3ff : 0x8000_000000000
//        = 0x3ff8_000000000000
fn vfpexpand_imm64(imm8: u8) -> u64 {
    let e = 11;
    let f = 64 - e - 1;
    let sign = imm8.get_bit(7);

    // exp is bitwise concatenation of following:
    // NOT(bit 6 of imm8) : replicate 'bit 6' of imm8 "e-3" times) : imm8 bits 5:4

    let bit6_vec = if imm8.get_bit(6) { 0xff } else { 0 };

    let exp: u64 = u64::from(!imm8.get_bit(6)) << (2 + (e - 3))
        | bit6_vec << 2
        | u64::from(imm8.get_bits(4..6));
    // frac is concatenation of: imm8 bits 3:0 : zeroes F-4 times
    let upper = u64::from(imm8.get_bits(0..4));
    let frac: u64 = upper << (f - 4);
    // result is contatenation of sign : exp : frac

    u64::from(sign) << 63 | exp << f | frac
}

fn vfpexpand_imm32(imm8: u8) -> u32 {
    let e = 8;
    let f = 32 - e - 1;
    let sign = imm8.get_bit(7);

    // exp is bitwise concatenation of following:
    // NOT(bit 6 of imm8) : replicate 'bit 6' of imm8 "e-3" times) : imm8 bits 5:4

    let bit6_vec = if imm8.get_bit(6) { 0x1f } else { 0 };

    let exp: u32 = (u32::from(!imm8.get_bit(6))) << (2 + (e - 3))
        | bit6_vec << 2
        | u32::from(imm8.get_bits(4..6));
    // frac is concatenation of: imm8 bits 3:0 : zeroes F-4 times
    let upper = u32::from(imm8.get_bits(0..4));
    let frac: u32 = upper << (f - 4);

    // result is contatenation of sign : exp : frac
    u32::from(sign) << 31 | exp << f | frac
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VMOV_imm(opcode: u32) -> Instruction {
    let sz = opcode.get_bit(8);
    let D = u8::from(opcode.get_bit(22));
    let vd = opcode.get_bits(12..16) as u8;
    let imm4l = opcode.get_bits(0..4) as u8;
    let imm4h = opcode.get_bits(16..20) as u8;
    let imm8 = imm4h << 4 | imm4l;

    if sz {
        Instruction::VMOV_imm_64 {
            params: VMovImmParams64 {
                dd: DoubleReg::from(D << 4 | vd),
                imm64: vfpexpand_imm64(imm8),
            },
        }
    } else {
        Instruction::VMOV_imm_32 {
            params: VMovImmParams32 {
                sd: SingleReg::from(vd << 1 | D),
                imm32: vfpexpand_imm32(imm8),
            },
        }
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VMOV_reg(opcode: u32) -> Instruction {
    let sz = opcode.get_bit(8);

    let vm = opcode.get_bits(0..4) as u8;
    let M = u8::from(opcode.get_bit(5));
    let vd = opcode.get_bits(12..16) as u8;
    let D = u8::from(opcode.get_bit(22));

    let dp_operation = sz;

    let d = if dp_operation {
        D << 4 | vd
    } else {
        vd << 1 | D
    };

    let m = if dp_operation {
        M << 4 | vm
    } else {
        vm << 1 | M
    };

    if dp_operation {
        return Instruction::VMOV_reg_f64 {
            params: VMovRegParamsf64 {
                dm: DoubleReg::from(m),
                dd: DoubleReg::from(d),
            },
        };
    }
    Instruction::VMOV_reg_f32 {
        params: VMovRegParamsf32 {
            sd: SingleReg::from(d),
            sm: SingleReg::from(m),
        },
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VMOV_cr_scalar(_opcode: u32) -> Instruction {
    unimplemented!()
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VMOV_scalar_cr(_opcode: u32) -> Instruction {
    unimplemented!()
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VMOV_cr_sp(opcode: u32) -> Instruction {
    let rt = Reg::from(opcode.get_bits(12..16) as u8);

    // rt 15 and 13 are unpredictable:
    if rt == Reg::PC || rt == Reg::SP {
        return Instruction::UDF {
            opcode: opcode.into(),
            thumb32: true,
            imm32: 0,
        };
    }

    let N = opcode.get_bit(7);
    let vn = (opcode.get_bits(16..20) as u8) << 1;
    let sn = SingleReg::from(vn | u8::from(N));
    Instruction::VMOV_cr_sp {
        params: VMovCrSpParams {
            to_arm_register: opcode.get_bit(20),
            rt,
            sn,
        },
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VMOV_cr2_sp2(_opcode: u32) -> Instruction {
    unimplemented!()
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VMOV_cr2_dp(opcode: u32) -> Instruction {
    let rt = Reg::from(opcode.get_bits(12..16) as u8);
    let rt2 = Reg::from(opcode.get_bits(16..20) as u8);

    // rt&rt2 15 and 13 are unpredictable:
    if rt == Reg::PC || rt == Reg::SP {
        return Instruction::UDF {
            opcode: opcode.into(),
            thumb32: true,
            imm32: 0,
        };
    }

    if rt2 == Reg::PC || rt2 == Reg::SP {
        return Instruction::UDF {
            opcode: opcode.into(),
            thumb32: true,
            imm32: 0,
        };
    }
    let op = opcode.get_bit(20);
    if op && rt == rt2 {
        return Instruction::UDF {
            opcode: opcode.into(),
            thumb32: true,
            imm32: 0,
        };
    }

    let M = opcode.get_bit(5);
    let vm = opcode.get_bits(0..4) as u8;
    let m = ((u8::from(M)) << 4) + vm;

    let dm = DoubleReg::from(m);

    Instruction::VMOV_cr2_dp {
        params: VMovCr2DpParams {
            to_arm_registers: op,
            rt,
            rt2,
            dm,
        },
    }
}
