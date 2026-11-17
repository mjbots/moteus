use crate::core::bits::Bits;

use crate::core::instruction::{Instruction, LoadAndStoreMultipleParams};
use crate::core::operation::get_reglist;
use crate::core::register::Reg;

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_STM_t1(opcode: u16) -> Instruction {
    let regs = get_reglist(opcode & 0b111_1111);

    Instruction::STM {
        params: LoadAndStoreMultipleParams {
            registers: regs,
            rn: Reg::from(opcode.get_bits(8..11) as u8),
            wback: true,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_STMDB_t1(opcode: u32) -> Instruction {
    let regs = get_reglist((opcode & 0xffff) as u16);

    Instruction::STMDB {
        params: LoadAndStoreMultipleParams {
            registers: regs,
            rn: Reg::from(opcode.get_bits(16..20) as u8),
            wback: opcode.get_bit(21),
        },
    }
}

#[allow(non_snake_case)]
pub fn decode_STM_t2(opcode: u32) -> Instruction {
    let regs = get_reglist((opcode & 0xffff) as u16);

    Instruction::STM {
        params: LoadAndStoreMultipleParams {
            registers: regs,
            rn: Reg::from(opcode.get_bits(16..20) as u8),
            wback: opcode.get_bit(21),
        },
        thumb32: true,
    }
}
