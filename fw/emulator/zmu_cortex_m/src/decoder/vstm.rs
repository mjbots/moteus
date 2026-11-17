use crate::core::bits::Bits;
use crate::core::instruction::{
    AddressingMode, Instruction, VStoreMultipleParams32, VStoreMultipleParams64,
};
use crate::core::register::{DoubleReg, Reg, SingleReg};
use enum_set::EnumSet;

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VSTM_t1(opcode: u32) -> Instruction {
    let W = opcode.get_bit(21);
    let U = opcode.get_bit(23);
    let imm8 = opcode.get_bits(0..8) as u8;
    let imm32 = u32::from(imm8 << 2);
    let P = opcode.get_bit(24);

    let n = opcode.get_bits(16..20) as u8;
    let rn = Reg::from(n);
    let D = u8::from(opcode.get_bit(22));
    let Vd = opcode.get_bits(12..16) as u8;
    let d = D << 4 | Vd;

    let regs = imm8 / 2;

    if (P == U && W) || n == 15 || regs == 0 || regs > 16 || (d + regs) > 32 {
        return Instruction::UDF {
            imm32: 0,
            opcode: opcode.into(),
            thumb32: true,
        };
    }

    #[cfg(feature = "vfp-register-bank-d16")]
    {
        if (d + regs) > 16 {
            return Instruction::UDF {
                imm32: 0,
                opcode: opcode.into(),
                thumb32: true,
            };
        }
    }

    let mode = if U {
        AddressingMode::IncrementAfter
    } else {
        AddressingMode::DecrementBefore
    };

    let mut double_regs = EnumSet::new();
    for i in 0..regs {
        double_regs.insert(DoubleReg::from(i + d));
    }

    Instruction::VSTM_T1 {
        params: VStoreMultipleParams64 {
            rn,
            write_back: W,
            list: double_regs,
            mode,
            imm32,
        },
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VSTM_t2(opcode: u32) -> Instruction {
    let W = opcode.get_bit(21);
    let imm8 = opcode.get_bits(0..8) as u8;
    let imm32 = u32::from(imm8 << 2);
    let U = opcode.get_bit(23);
    let P = opcode.get_bit(24);

    let n = opcode.get_bits(16..20) as u8;
    let rn = Reg::from(n);

    let D = u8::from(opcode.get_bit(22));
    let Vd = opcode.get_bits(12..16) as u8;
    let d = Vd << 1 | D;

    let regs = imm8;

    if (P == U && W) || n == 15 || regs == 0 || (d + regs) > 32 {
        return Instruction::UDF {
            imm32: 0,
            opcode: opcode.into(),
            thumb32: true,
        };
    }

    let mode = if U {
        AddressingMode::IncrementAfter
    } else {
        AddressingMode::DecrementBefore
    };

    let mut single_regs = EnumSet::new();
    for i in 0..regs {
        single_regs.insert(SingleReg::from(d + i));
    }

    Instruction::VSTM_T2 {
        params: VStoreMultipleParams32 {
            rn,
            write_back: W,
            list: single_regs,
            mode,
            imm32,
        },
    }
}
