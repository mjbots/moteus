use crate::core::bits::Bits;
use crate::core::instruction::Imm32Carry;
use crate::core::instruction::Instruction;
use crate::core::instruction::{Reg2Params, Reg2ShiftNParams, RegImmCarryParams, SetFlags};
use crate::core::operation::decode_imm_shift;
use crate::core::operation::thumb_expand_imm_c;
use crate::core::operation::zero_extend;
use crate::core::register::Reg;

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_MOV_imm_t1(opcode: u16) -> Instruction {
    Instruction::MOV_imm {
        params: RegImmCarryParams {
            rd: Reg::from(opcode.get_bits(8..11) as u8),
            imm32: Imm32Carry::NoCarry {
                imm32: u32::from(opcode.get_bits(0..8)),
            },
            setflags: SetFlags::NotInITBlock,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_MOV_reg_t1(opcode: u16) -> Instruction {
    Instruction::MOV_reg {
        params: Reg2Params {
            rd: Reg::from(((u8::from(opcode.get_bit(7))) << 3) + opcode.get_bits(0..3) as u8),
            rm: Reg::from(opcode.get_bits(3..7) as u8),
            setflags: SetFlags::False,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_MOV_reg_t3(opcode: u32) -> Instruction {
    Instruction::MOV_reg {
        params: Reg2Params {
            rd: Reg::from(opcode.get_bits(8..12) as u8),
            rm: Reg::from(opcode.get_bits(0..4) as u8),
            setflags: if opcode.get_bit(20) {
                SetFlags::True
            } else {
                SetFlags::False
            },
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_MOV_reg_t2_LSL_imm_t1(opcode: u16) -> Instruction {
    let imm5 = opcode.get_bits(6..11) as u8;

    if imm5 == 0 {
        Instruction::MOV_reg {
            params: Reg2Params {
                rd: Reg::from(opcode.get_bits(0..3) as u8),
                rm: Reg::from(opcode.get_bits(3..6) as u8),
                setflags: SetFlags::NotInITBlock,
            },
            thumb32: false,
        }
    } else {
        let (_, shift_n) = decode_imm_shift(0b00, imm5);
        Instruction::LSL_imm {
            params: Reg2ShiftNParams {
                rd: Reg::from(opcode.get_bits(0..3) as u8),
                rm: Reg::from(opcode.get_bits(3..6) as u8),
                shift_n,
                setflags: SetFlags::NotInITBlock,
            },
            thumb32: false,
        }
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_MOV_imm_t2(opcode: u32) -> Instruction {
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm8: u8 = opcode.get_bits(0..8) as u8;
    let i: u8 = u8::from(opcode.get_bit(26));
    let s = opcode.get_bit(20);

    let params = [i, imm3, imm8];
    let lengths = [1, 3, 8];

    Instruction::MOV_imm {
        params: RegImmCarryParams {
            rd: Reg::from(rd),
            imm32: Imm32Carry::Carry {
                imm32_c0: thumb_expand_imm_c(&params, &lengths, false),
                imm32_c1: thumb_expand_imm_c(&params, &lengths, true),
            },
            setflags: if s { SetFlags::True } else { SetFlags::False },
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_MOV_imm_t3(opcode: u32) -> Instruction {
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let imm4: u8 = opcode.get_bits(16..20) as u8;
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm8: u8 = opcode.get_bits(0..8) as u8;
    let i: u8 = u8::from(opcode.get_bit(26));

    let params = [imm4, i, imm3, imm8];
    let lengths = [4, 1, 3, 8];

    Instruction::MOV_imm {
        params: RegImmCarryParams {
            rd: Reg::from(rd),
            imm32: Imm32Carry::NoCarry {
                imm32: zero_extend(&params, &lengths),
            },
            setflags: SetFlags::False,
        },
        thumb32: true,
    }
}
