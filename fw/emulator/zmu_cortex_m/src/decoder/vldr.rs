use crate::core::bits::Bits;
use crate::core::instruction::{Instruction, VLoadAndStoreParams};
use crate::core::register::{DoubleReg, ExtensionReg, Reg, SingleReg};

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VLDR_t1(opcode: u32) -> Instruction {
    let vd = opcode.get_bits(12..16) as u8;
    let D = u8::from(opcode.get_bit(22));
    let rn = opcode.get_bits(16..20) as u8;
    Instruction::VLDR {
        params: VLoadAndStoreParams {
            dd: ExtensionReg::Double {
                reg: DoubleReg::from(D << 4 | vd),
            },
            rn: Reg::from(rn),
            imm32: opcode.get_bits(0..8) << 2,
            add: opcode.get_bit(23),
        },
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_VLDR_t2(opcode: u32) -> Instruction {
    let vd = opcode.get_bits(12..16) as u8;
    let D = u8::from(opcode.get_bit(22));
    let rn = opcode.get_bits(16..20) as u8;
    Instruction::VLDR {
        params: VLoadAndStoreParams {
            dd: ExtensionReg::Single {
                reg: SingleReg::from(vd << 1 | D),
            },
            rn: Reg::from(rn),
            imm32: opcode.get_bits(0..8) << 2,
            add: opcode.get_bit(23),
        },
    }
}
