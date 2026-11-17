//!
//! Helper operations commonly used for instruction execution
//!
use crate::core::bits::Bits;
use crate::core::condition::Condition;
use crate::core::instruction::SRType;
use crate::core::register::Apsr;
use crate::core::register::PSR;
use crate::core::register::Reg;
use enum_set::EnumSet;

///
/// Convert a bit pattern to set of registers
///
pub fn get_reglist(pattern: u16) -> EnumSet<Reg> {
    let mut regs: EnumSet<Reg> = EnumSet::new();

    if pattern.get_bit(0) {
        regs.insert(Reg::R0);
    }
    if pattern.get_bit(1) {
        regs.insert(Reg::R1);
    }
    if pattern.get_bit(2) {
        regs.insert(Reg::R2);
    }
    if pattern.get_bit(3) {
        regs.insert(Reg::R3);
    }
    if pattern.get_bit(4) {
        regs.insert(Reg::R4);
    }
    if pattern.get_bit(5) {
        regs.insert(Reg::R5);
    }
    if pattern.get_bit(6) {
        regs.insert(Reg::R6);
    }
    if pattern.get_bit(7) {
        regs.insert(Reg::R7);
    }
    if pattern.get_bit(8) {
        regs.insert(Reg::R8);
    }
    if pattern.get_bit(9) {
        regs.insert(Reg::R9);
    }
    if pattern.get_bit(10) {
        regs.insert(Reg::R10);
    }
    if pattern.get_bit(11) {
        regs.insert(Reg::R11);
    }
    if pattern.get_bit(12) {
        regs.insert(Reg::R12);
    }
    if pattern.get_bit(13) {
        regs.insert(Reg::SP);
    }
    if pattern.get_bit(14) {
        regs.insert(Reg::LR);
    }
    if pattern.get_bit(15) {
        regs.insert(Reg::PC);
    }

    regs
}

///
/// Sign extend a u32 value, with given topbit to u64 value
///
pub fn sign_extend(word: u32, topbit: usize, size: usize) -> u64 {
    if word & (1 << topbit) == (1 << topbit) {
        return u64::from(word) | ((1_u64 << (size - topbit)) - 1) << topbit;
    }
    u64::from(word)
}

///
/// Add two numbers and carry
///
/// x + y + carry
///
/// return tuple of (result, carry, overflow)
///
#[inline(always)]
pub fn add_with_carry(x: u32, y: u32, carry_in: bool) -> (u32, bool, bool) {
    let unsigned_sum = u64::from(x) + u64::from(y) + u64::from(carry_in);
    let result = (unsigned_sum & 0xffff_ffff) as u32; // same value as signed_sum<N-1:0>
    let carry_out = unsigned_sum > u64::from(u32::MAX);

    let signed_sum = i64::from(x as i32) + i64::from(y as i32) + i64::from(u8::from(carry_in));
    let overflow = signed_sum > i64::from(i32::MAX) || signed_sum < i64::from(i32::MIN);

    (result, carry_out, overflow)
}

///
/// This function performs the condition test for an instruction, based on:
/// • the two Thumb conditional branch encodings, encodings T1 and T3 of the B instruction
/// • the current values of the xPSR.IT[7:0] bits for other Thumb instructions.
///
#[inline(always)]
pub fn condition_test(condition: Condition, psr: &PSR) -> bool {
    match condition {
        Condition::EQ => psr.get_z(),
        Condition::NE => !psr.get_z(),
        Condition::CS => psr.get_c(),
        Condition::CC => !psr.get_c(),
        Condition::MI => psr.get_n(),
        Condition::PL => !psr.get_n(),

        Condition::VS => psr.get_v(),
        Condition::VC => !psr.get_v(),

        Condition::HI => psr.get_c() && !psr.get_z(),
        Condition::LS => psr.get_z() || !psr.get_c(),

        Condition::GE => psr.get_n() == psr.get_v(),
        Condition::LT => psr.get_n() != psr.get_v(),

        Condition::GT => (psr.get_n() == psr.get_v()) && !psr.get_z(),
        Condition::LE => psr.get_z() || psr.get_n() != psr.get_v(),

        Condition::AL => true,
    }
}

/// Decode immedate shift type
/// input: bits[2], immedate
/// output: (shitft type, immedate to use)
///
pub fn decode_imm_shift(typebits: u8, imm5: u8) -> (SRType, u8) {
    match typebits.get_bits(0..3) {
        0b00 => (SRType::LSL, imm5),
        0b01 => (SRType::LSR, if imm5 == 0 { 32 } else { imm5 }),
        0b10 => (SRType::ASR, if imm5 == 0 { 32 } else { imm5 }),
        0b11 => {
            if imm5 == 0 {
                (SRType::RRX, 1)
            } else {
                (SRType::ROR, imm5)
            }
        }
        _ => todo!("invalid typebits"),
    }
}

fn lsl_c(value: u32, shift: usize) -> (u32, bool) {
    assert!(shift > 0);
    match shift {
        1..=31 => (value << shift, value.get_bit(32 - shift)),
        32 => (0, value.get_bit(0)),
        _ => (0, false),
    }
}

fn lsl(value: u32, shift: usize) -> u32 {
    if shift == 0 {
        value
    } else if shift < 32 {
        value << shift
    } else {
        0
    }
}

fn lsr_c(value: u32, shift: usize) -> (u32, bool) {
    assert!(shift > 0);
    match shift {
        1..=31 => (value >> shift, value.get_bit(shift - 1)),
        32 => (0, value.get_bit(31)),
        _ => (0, false),
    }
}

fn lsr(value: u32, shift: usize) -> u32 {
    if shift == 0 {
        value
    } else if shift < 32 {
        value >> shift
    } else {
        0
    }
}

fn asr_c(value: u32, shift: usize) -> (u32, bool) {
    assert!(shift > 0);
    if let 1..=31 = shift {
        let result = ((value as i32) >> shift) as u32;
        (result, value.get_bit(shift - 1))
    } else {
        let bit31 = value.get_bit(31);
        (if bit31 { u32::MAX } else { 0 }, bit31)
    }
}

fn ror_c(value: u32, shift: usize) -> (u32, bool) {
    assert!(shift > 0);
    let m = shift % 32;
    let result = if m == 0 {
        value
    } else {
        lsr(value, m) | lsl(value, 32 - m)
    };
    let carry_out = result.get_bit(31);
    (result, carry_out)
}

///
/// rotate value right with given amount of bits
///
pub fn ror(value: u32, shift: usize) -> u32 {
    if shift == 0 {
        value
    } else {
        let (result, _) = ror_c(value, shift);
        result
    }
}

fn rrx_c(value: u32, carry_in: bool) -> (u32, bool) {
    let carry_out = value.get_bit(0);
    let result = (value >> 1) + (u32::from(carry_in) << 31);
    (result, carry_out)
}

///
/// Do the one of the different shifting operations, with carry in support
///
/// Parameters:
/// - value is the number on which to apply the shifting on
/// - ```shift_t``` selects the shift type to use
/// - ```amount``` declares number of bits to shift and
/// - ```carry_in``` gives the input carry bit. Carry in is only used for RRX type shifting.
///
/// Returns:
/// - shifted value
/// - carry out
#[inline(always)]
pub fn shift_c(value: u32, shift_t: SRType, amount: usize, carry_in: bool) -> (u32, bool) {
    assert!(!((shift_t == SRType::RRX) && (amount != 1)));
    if amount == 0 {
        (value, carry_in)
    } else {
        match shift_t {
            SRType::LSL => lsl_c(value, amount),
            SRType::LSR => lsr_c(value, amount),
            SRType::ASR => asr_c(value, amount),
            SRType::ROR => ror_c(value, amount),
            SRType::RRX => rrx_c(value, carry_in),
        }
    }
}

///
/// shift value with given shift type, discard carry
///
#[inline(always)]
pub fn shift(value: u32, shift_t: SRType, amount: usize, carry_in: bool) -> u32 {
    let (result, _) = shift_c(value, shift_t, amount, carry_in);

    result
}

///
/// expand immediate value from thumb encoding, discard carry
///
pub fn thumb_expand_imm(params: &[u8], lengths: &[u8]) -> u32 {
    let (result, _) = thumb_expand_imm_c(params, lengths, false);

    result
}

///
/// Expand immediate value from thumb encoding, with carry calculation
///
pub fn thumb_expand_imm_c(params: &[u8], lengths: &[u8], carry_in: bool) -> (u32, bool) {
    let imm12 = zero_extend(params, lengths);

    let (result, carry_out) = if imm12.get_bits(10..12) == 0 {
        let low_word = imm12.get_bits(0..8);
        let imm32 = match imm12.get_bits(8..10) {
            0b00 => low_word,
            0b01 => (low_word << 16) + low_word,
            0b10 => (low_word << 24) + (low_word << 8),
            0b11 => (low_word << 24) + (low_word << 16) + (low_word << 8) + low_word,
            _ => unreachable!(),
        };

        (imm32, carry_in)
    } else {
        let unrotated_value = (1_u32 << 7) + imm12.get_bits(0..7);
        ror_c(unrotated_value, imm12.get_bits(7..12) as usize)
    };

    (result, carry_out)
}

///
/// zero extend n parameters to n bit lengths
///
pub fn zero_extend(params: &[u8], lengths: &[u8]) -> u32 {
    assert_eq!(params.len(), lengths.len());

    let mut result: u32 = 0;
    let mut shift = 0;
    for (param, length) in params.iter().rev().zip(lengths.iter().rev()) {
        result += u32::from(*param) << shift;
        shift += length;
    }

    result
}

///
/// zero extend n parameters to n bit lengths
/// TODO: make generic
///
pub fn zero_extend_u16(params: &[u16], lengths: &[u8]) -> u32 {
    assert_eq!(params.len(), lengths.len());

    let mut result: u32 = 0;
    let mut shift = 0;
    for (param, length) in params.iter().rev().zip(lengths.iter().rev()) {
        result += u32::from(*param) << shift;
        shift += length;
    }

    result
}

///
/// build signed value from immediate 10/11 representation
///
pub fn build_imm_10_11(opcode: u32) -> i32 {
    let t1 = opcode >> 16;
    let t2 = opcode & 0xffff;

    let s = (t1 >> 10) & 1;
    let imm10 = t1 & 0x3ff;

    let j1 = (t2 >> 13) & 1;
    let j2 = (t2 >> 11) & 1;
    let imm11 = t2 & 0x7ff;

    let i1 = (j1 ^ s) ^ 1;
    let i2 = (j2 ^ s) ^ 1;

    sign_extend(
        (imm11 << 1) + (imm10 << 12) + (i2 << 22) + (i1 << 23) + (s << 24),
        24,
        32,
    ) as i32
}

///
/// build signed value from immediate 6/11 representation
///
pub fn build_imm_6_11(opcode: u32) -> i32 {
    let t1 = opcode >> 16;
    let t2 = opcode & 0xffff;

    let imm6 = t1 & 0x3f;

    let s = (t1 >> 10) & 1;

    let j1 = (t2 >> 13) & 1;
    let j2 = (t2 >> 11) & 1;

    let imm11 = t2 & 0x7ff;

    sign_extend(
        (imm11 << 1) + (imm6 << 12) + (j2 << 18) + (j1 << 19) + (s << 20),
        20,
        32,
    ) as i32
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_shift_c() {
        {
            let (result, carry) = shift_c(0xFFFF_FFF8, SRType::ASR, 8, false);
            assert!(result == 0xFFFF_FFFF);
            assert!(carry);
        }
        {
            let (result, carry) = shift_c(0xef, SRType::ASR, 9, false);
            assert!(result == 0);
            assert!(!carry);
        }
        {
            let (result, carry) = shift_c(0xFFF_FFFC0, SRType::ASR, 1, false);
            assert!(result == 0xFFFF_FFE0);
            assert!(!carry);
        }

        {
            let (result, carry) = shift_c(0, SRType::ROR, 0, false);
            assert!(result == 0x0);
            assert!(!carry);
        }
        {
            let (result, carry) = shift_c(2, SRType::ROR, 1, false);
            assert!(result == 0x1);
            assert!(!carry);
        }
        {
            let (result, carry) = shift_c(1, SRType::ROR, 1, false);
            assert!(result == 0x8000_0000);
            assert!(carry);
        }
        {
            let (result, carry) = shift_c(1, SRType::LSL, 32, false);
            assert_eq!(result, 0);
            assert!(carry);
        }
        {
            let (result, carry) = shift_c(0x8000_0000, SRType::LSR, 32, false);
            assert_eq!(result, 0);
            assert!(carry);
        }
        {
            let (result, carry) = shift_c(0x8000_0001, SRType::LSR, 40, false);
            assert_eq!(result, 0);
            assert!(!carry);
        }
        {
            let (result, carry) = shift_c(0x8000_0001, SRType::ASR, 40, false);
            assert_eq!(result, 0xFFFF_FFFF);
            assert!(carry);
        }
        {
            let (result, carry) = shift_c(0x8000_0001, SRType::ROR, 32, false);
            assert_eq!(result, 0x8000_0001);
            assert!(carry);
        }
    }

    #[test]
    fn test_add_with_carry() {
        let (result, carry, overflow) = add_with_carry(0x410, 4, false);
        assert_eq!(result, 0x414);
        assert!(!carry);
        assert!(!overflow);
    }

    #[test]
    fn test_add_with_carry_basic() {
        let (result, carry, overflow) = add_with_carry(0x0, 0xffff_ffff, false);
        assert_eq!(result, 0xffff_ffff);
        assert!(!carry);
        assert!(!overflow);
    }

    #[test]
    fn test_add_with_carry_basic2() {
        let (result, carry, overflow) = add_with_carry(0x0, 0xffff_ffff, true);
        assert_eq!(result, 0);
        assert!(carry);
        assert!(!overflow);
    }

    #[test]
    fn test_add_with_carry_basic3() {
        let (result, carry, overflow) = add_with_carry(0x0, 0, true);
        assert_eq!(result, 1);
        assert!(!carry);
        assert!(!overflow);
    }

    #[test]
    fn test_add_with_carry_basic4() {
        let (result, carry, overflow) = add_with_carry(0xffff_ffff, 0, true);
        assert_eq!(result, 0);
        assert!(carry);
        assert!(!overflow);
    }

    #[test]
    fn test_add_with_carry_basic5() {
        let (result, carry, overflow) = add_with_carry(0xffff_ffff, 0xffff_ffff, true);
        assert_eq!(result, 0xffff_ffff);
        assert!(carry);
        assert!(!overflow);
    }

    #[test]
    fn test_add_with_carry_basic6() {
        let (result, carry, overflow) = add_with_carry(0xffff_ffff, 0xffff_ffff, false);
        assert_eq!(result, 0xffff_fffe);
        assert!(carry);
        assert!(!overflow);
    }

    #[test]
    fn test_add_with_carry_overflow_positive() {
        let (result, carry, overflow) = add_with_carry(0x7fff_ffff, 0x1, false);
        assert_eq!(result, 0x8000_0000);
        assert!(!carry);
        assert!(overflow);
    }

    #[test]
    fn test_add_with_carry_overflow_negative() {
        let (result, carry, overflow) = add_with_carry(0x8000_0000, 0xffff_ffff, false);
        assert_eq!(result, 0x7fff_ffff);
        assert!(carry);
        assert!(overflow);
    }

    #[test]
    fn test_build_imm_6_11() {
        assert_eq!(build_imm_6_11(0xF000_80C4), 0xc4 << 1);
        assert_eq!(build_imm_6_11(0xf57f_ad69), -1326);
    }

    #[test]
    fn test_decode_imm_shift_rrx_alias() {
        assert_eq!(decode_imm_shift(0b11, 0), (SRType::RRX, 1));
        assert_eq!(decode_imm_shift(0b11, 5), (SRType::ROR, 5));
    }
}
