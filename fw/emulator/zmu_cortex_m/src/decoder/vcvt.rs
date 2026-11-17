use crate::core::{
    bits::Bits,
    instruction::{Instruction, VCVTParams, VCVTParamsF32F64, VCVTParamsF64F32},
    register::{DoubleReg, ExtensionReg, SingleReg},
};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VCVT_t1(opcode: u32) -> Instruction {
    let vm = opcode.get_bits(0..4) as u8;
    let M = u8::from(opcode.get_bit(5));
    let op = opcode.get_bit(7);
    let sz = opcode.get_bit(8);
    let vd = opcode.get_bits(12..16) as u8;
    let opc2 = opcode.get_bits(16..19) as u8;
    let D = u8::from(opcode.get_bit(22));

    let to_integer = opc2.get_bit(2);
    let dp_operation = sz;
    if to_integer {
        let unsigned = !opc2.get_bit(0);
        let round_zero = op;
        let d = vd << 1 | D;
        let m = if dp_operation {
            M << 4 | vm
        } else {
            vm << 1 | M
        };
        Instruction::VCVT {
            params: VCVTParams {
                to_integer,
                dp_operation,
                unsigned,
                round_zero,
                round_nearest: false,
                d: ExtensionReg::Single {
                    reg: SingleReg::from(d),
                },
                m: if dp_operation {
                    ExtensionReg::Double {
                        reg: DoubleReg::from(m),
                    }
                } else {
                    ExtensionReg::Single {
                        reg: SingleReg::from(m),
                    }
                },
            },
        }
    } else {
        let unsigned = !op;
        let round_nearest = false;
        let m = vm << 1 | M;
        let d = if dp_operation {
            D << 4 | vd
        } else {
            vd << 1 | D
        };
        Instruction::VCVT {
            params: VCVTParams {
                to_integer,
                dp_operation,
                unsigned,
                round_zero: false,
                round_nearest,
                d: if dp_operation {
                    ExtensionReg::Double {
                        reg: DoubleReg::from(d),
                    }
                } else {
                    ExtensionReg::Single {
                        reg: SingleReg::from(d),
                    }
                },
                m: ExtensionReg::Single {
                    reg: SingleReg::from(m),
                },
            },
        }
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VCVT_ds_t1(opcode: u32) -> Instruction {
    let vm = opcode.get_bits(0..4) as u8;
    let m = u8::from(opcode.get_bit(5));
    let sz = opcode.get_bit(8);
    let vd = opcode.get_bits(12..16) as u8;
    let d = u8::from(opcode.get_bit(22));

    if sz {
        Instruction::VCVT_f32_f64 {
            params: VCVTParamsF32F64 {
                sd: SingleReg::from((vd << 1) | d),
                dm: DoubleReg::from((m << 4) | vm),
            },
        }
    } else {
        Instruction::VCVT_f64_f32 {
            params: VCVTParamsF64F32 {
                dd: DoubleReg::from((d << 4) | vd),
                sm: SingleReg::from((vm << 1) | m),
            },
        }
    }
}
