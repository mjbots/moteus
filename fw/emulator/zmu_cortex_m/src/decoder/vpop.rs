use enum_set::EnumSet;

use crate::core::bits::Bits;
use crate::core::instruction::{Instruction, VPushPopParams};
use crate::core::register::{DoubleReg, SingleReg};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VPOP_t1(opcode: u32) -> Instruction {
    let imm32 = opcode.get_bits(0..8) << 2;
    let D = u8::from(opcode.get_bit(22));
    let Vd = opcode.get_bits(12..16) as u8;
    let d = D << 4 | Vd;
    let regs = opcode.get_bits(0..8) as u8 / 2;

    if regs == 0 || regs > 16 || d + regs > 32 {
        return Instruction::UDF {
            imm32: 0,
            opcode: opcode.into(),
            thumb32: true,
        };
    }

    #[cfg(feature = "vfp-register-bank-d16")]
    {
        if d + regs > 16 {
            return Instruction::UDF {
                imm32: 0,
                opcode: opcode.into(),
                thumb32: true,
            };
        }
    }

    let mut double_regs = EnumSet::new();
    for i in 0..regs {
        double_regs.insert(DoubleReg::from(d + i));
    }

    Instruction::VPOP {
        params: VPushPopParams {
            single_regs: false,
            single_precision_registers: EnumSet::new(),
            double_precision_registers: double_regs,
            imm32,
        },
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VPOP_t2(opcode: u32) -> Instruction {
    let imm32 = opcode.get_bits(0..8) << 2;
    let D = u8::from(opcode.get_bit(22));
    let Vd = opcode.get_bits(12..16) as u8;
    let d = D | Vd << 4;
    let regs = opcode.get_bits(0..8) as u8;

    if regs == 0 || d + regs > 32 {
        return Instruction::UDF {
            imm32: 0,
            opcode: opcode.into(),
            thumb32: true,
        };
    }

    let mut single_regs = EnumSet::new();
    for i in 0..regs {
        single_regs.insert(SingleReg::from(d + i));
    }

    Instruction::VPOP {
        params: VPushPopParams {
            single_regs: true,
            single_precision_registers: single_regs,
            double_precision_registers: EnumSet::new(),
            imm32,
        },
    }
}
