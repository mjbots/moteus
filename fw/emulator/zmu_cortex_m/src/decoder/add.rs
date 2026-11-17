use crate::core::bits::Bits;
use crate::core::instruction::Instruction;
use crate::core::instruction::{Reg2ImmParams, Reg2ShiftParams, Reg3ShiftParams, SRType, SetFlags};
use crate::core::operation::decode_imm_shift;
use crate::core::operation::thumb_expand_imm;
use crate::core::operation::zero_extend;
use crate::core::register::Reg;

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_ADD_reg_t1(opcode: u16) -> Instruction {
    Instruction::ADD_reg {
        params: Reg3ShiftParams {
            rd: Reg::from(opcode.get_bits(0..3) as u8),
            rn: Reg::from(opcode.get_bits(3..6) as u8),
            rm: Reg::from(opcode.get_bits(6..9) as u8),
            setflags: SetFlags::NotInITBlock,
            shift_t: SRType::LSL,
            shift_n: 0,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_ADD_reg_t2(opcode: u16) -> Instruction {
    let rdn = Reg::from((u8::from(opcode.get_bit(7)) << 3) + opcode.get_bits(0..3) as u8);

    Instruction::ADD_reg {
        params: Reg3ShiftParams {
            rm: Reg::from(opcode.get_bits(3..7) as u8),
            rd: rdn,
            rn: rdn,
            setflags: SetFlags::False,
            shift_t: SRType::LSL,
            shift_n: 0,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_ADD_reg_sp_t1(opcode: u16) -> Instruction {
    let rdm = Reg::from((u8::from(opcode.get_bit(7)) << 3) + opcode.get_bits(0..3) as u8);

    Instruction::ADD_sp_reg {
        params: Reg2ShiftParams {
            rm: rdm,
            rd: rdm,
            setflags: SetFlags::False,
            shift_t: SRType::LSL,
            shift_n: 0,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_ADD_reg_sp_t2(opcode: u16) -> Instruction {
    Instruction::ADD_sp_reg {
        params: Reg2ShiftParams {
            rm: Reg::from(opcode.get_bits(3..7) as u8),
            rd: Reg::SP,
            setflags: SetFlags::False,
            shift_t: SRType::LSL,
            shift_n: 0,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
pub fn decode_ADD_reg_t3(opcode: u32) -> Instruction {
    let rn: u8 = opcode.get_bits(16..20) as u8;
    let rm: u8 = opcode.get_bits(0..4) as u8;
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let s: u8 = u8::from(opcode.get_bit(20));

    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm2: u8 = opcode.get_bits(6..8) as u8;
    let type_: u8 = opcode.get_bits(4..6) as u8;

    let (shift_t, shift_n) = decode_imm_shift(type_, (imm3 << 2) + imm2);

    Instruction::ADD_reg {
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
pub fn decode_ADD_imm_t1(opcode: u16) -> Instruction {
    Instruction::ADD_imm {
        params: Reg2ImmParams {
            rd: Reg::from(opcode.get_bits(0..3) as u8),
            rn: Reg::from(opcode.get_bits(3..6) as u8),
            imm32: u32::from(opcode.get_bits(6..9)),
            setflags: SetFlags::NotInITBlock,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_ADD_imm_t2(opcode: u16) -> Instruction {
    Instruction::ADD_imm {
        params: Reg2ImmParams {
            rn: Reg::from(opcode.get_bits(8..11) as u8),
            rd: Reg::from(opcode.get_bits(8..11) as u8),
            imm32: u32::from(opcode.get_bits(0..8)),
            setflags: SetFlags::NotInITBlock,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_ADD_SP_imm_t1(opcode: u16) -> Instruction {
    Instruction::ADD_imm {
        params: Reg2ImmParams {
            rd: Reg::from(opcode.get_bits(8..11) as u8),
            rn: Reg::SP,
            imm32: u32::from(opcode.get_bits(0..8)) << 2,
            setflags: SetFlags::False,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_ADD_SP_imm_t2(opcode: u16) -> Instruction {
    Instruction::ADD_imm {
        params: Reg2ImmParams {
            rd: Reg::SP,
            rn: Reg::SP,
            imm32: u32::from(opcode.get_bits(0..7)) << 2,
            setflags: SetFlags::False,
        },
        thumb32: false,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_ADD_imm_t3(opcode: u32) -> Instruction {
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let rn: u8 = opcode.get_bits(16..20) as u8;
    let s = opcode.get_bit(20);

    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm8: u8 = opcode.get_bits(0..8) as u8;
    let i: u8 = u8::from(opcode.get_bit(26));

    let params = [i, imm3, imm8];
    let lengths = [1, 3, 8];

    Instruction::ADD_imm {
        params: Reg2ImmParams {
            rd: Reg::from(rd),
            rn: Reg::from(rn),
            imm32: thumb_expand_imm(&params, &lengths),
            setflags: if s { SetFlags::True } else { SetFlags::False },
        },
        thumb32: true,
    }
}

#[allow(non_snake_case)]
#[inline(always)]
pub fn decode_ADD_imm_t4(opcode: u32) -> Instruction {
    let rd: u8 = opcode.get_bits(8..12) as u8;
    let rn: u8 = opcode.get_bits(16..20) as u8;

    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm8: u8 = opcode.get_bits(0..8) as u8;
    let i: u8 = u8::from(opcode.get_bit(26));

    let params = [i, imm3, imm8];
    let lengths = [1, 3, 8];

    Instruction::ADD_imm {
        params: Reg2ImmParams {
            rd: Reg::from(rd),
            rn: Reg::from(rn),
            imm32: zero_extend(&params, &lengths),
            setflags: SetFlags::False,
        },
        thumb32: true,
    }
}
