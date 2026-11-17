use crate::core::bits::Bits;
use crate::core::instruction::Instruction;
use crate::core::instruction::{Reg2ImmParams, Reg3ShiftParams, SRType, SetFlags};
use crate::core::operation::decode_imm_shift;
use crate::core::operation::thumb_expand_imm;
use crate::core::operation::zero_extend;
use crate::core::register::Reg;

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_SUB_imm_t1(command: u16) -> Instruction {
    Instruction::SUB_imm {
        params: Reg2ImmParams {
            rd: From::from(command.get_bits(0..3)),
            rn: From::from(command.get_bits(3..6)),
            setflags: SetFlags::NotInITBlock,
            imm32: u32::from(command.get_bits(6..9)),
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_SUB_imm_t2(command: u16) -> Instruction {
    Instruction::SUB_imm {
        params: Reg2ImmParams {
            rd: From::from(command.get_bits(8..11)),
            rn: From::from(command.get_bits(8..11)),
            setflags: SetFlags::NotInITBlock,
            imm32: u32::from(command.get_bits(0..8)),
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_SUB_SP_imm_t1(command: u16) -> Instruction {
    Instruction::SUB_imm {
        params: Reg2ImmParams {
            rn: Reg::SP,
            rd: Reg::SP,
            imm32: u32::from(command.get_bits(0..7)) << 2,
            setflags: SetFlags::False,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_SUB_SP_imm_t2(opcode: u32) -> Instruction {
    let i: u8 = u8::from(opcode.get_bit(26));
    let s: u8 = u8::from(opcode.get_bit(20));

    let rd: u8 = opcode.get_bits(8..12) as u8;
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm8: u8 = opcode.get_bits(0..8) as u8;

    let params = [i, imm3, imm8];
    let lengths = [1, 3, 8];

    Instruction::SUB_imm {
        params: Reg2ImmParams {
            rd: Reg::from(rd),
            rn: Reg::SP,
            imm32: thumb_expand_imm(&params, &lengths),
            setflags: if s == 1 {
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
pub fn decode_SUB_SP_imm_t3(opcode: u32) -> Instruction {
    let i: u8 = u8::from(opcode.get_bit(26));

    let rd: u8 = opcode.get_bits(8..12) as u8;
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm8: u8 = opcode.get_bits(0..8) as u8;

    let params = [i, imm3, imm8];
    let lengths = [1, 3, 8];
    Instruction::SUB_imm {
        params: Reg2ImmParams {
            rd: Reg::from(rd),
            rn: Reg::SP,
            imm32: zero_extend(&params, &lengths),
            setflags: SetFlags::False,
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_SUB_reg_t1(command: u16) -> Instruction {
    Instruction::SUB_reg {
        params: Reg3ShiftParams {
            rd: From::from(command.get_bits(0..3)),
            rn: From::from(command.get_bits(3..6)),
            rm: From::from(command.get_bits(6..9)),
            setflags: SetFlags::NotInITBlock,
            shift_t: SRType::LSL,
            shift_n: 0,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_SUB_reg_t2(opcode: u32) -> Instruction {
    let rn: u8 = opcode.get_bits(16..20) as u8;
    let rm: u8 = opcode.get_bits(0..4) as u8;
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let s: u8 = u8::from(opcode.get_bit(20));

    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm2: u8 = opcode.get_bits(6..8) as u8;
    let type_: u8 = opcode.get_bits(4..6) as u8;

    let (shift_t, shift_n) = decode_imm_shift(type_, (imm3 << 2) + imm2);

    Instruction::SUB_reg {
        params: Reg3ShiftParams {
            rd: Reg::from(rd),
            rn: Reg::from(rn),
            rm: Reg::from(rm),
            setflags: if s == 1 {
                SetFlags::True
            } else {
                SetFlags::False
            },
            shift_t,
            shift_n,
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_SUB_imm_t3(opcode: u32) -> Instruction {
    let i: u8 = u8::from(opcode.get_bit(26));
    let s: u8 = u8::from(opcode.get_bit(20));

    let rn: u8 = opcode.get_bits(16..20) as u8;
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm8: u8 = opcode.get_bits(0..8) as u8;

    let params = [i, imm3, imm8];
    let lengths = [1, 3, 8];

    Instruction::SUB_imm {
        params: Reg2ImmParams {
            rd: Reg::from(rd),
            rn: Reg::from(rn),
            imm32: thumb_expand_imm(&params, &lengths),
            setflags: if s == 1 {
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
pub fn decode_SUB_imm_t4(opcode: u32) -> Instruction {
    let i: u8 = u8::from(opcode.get_bit(26));

    let rn: u8 = opcode.get_bits(16..20) as u8;
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm8: u8 = opcode.get_bits(0..8) as u8;

    let params = [i, imm3, imm8];
    let lengths = [1, 3, 8];

    Instruction::SUB_imm {
        params: Reg2ImmParams {
            rd: rd.into(),
            rn: rn.into(),
            imm32: zero_extend(&params, &lengths),
            setflags: SetFlags::False,
        },
        thumb32: true,
    }
}
