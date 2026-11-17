//!
//! Floating point arithmetic, done with that of the host.
//!
//! The host's arithmetic is IEEE-754, as the processor's is, so that
//! results are exact.  What this does not do is maintain the
//! cumulative exception flags in FPSCR, or honor its flush to zero
//! and default NaN modes, or its rounding mode other than where
//! rounding to an integer.
//!

use crate::{
    Processor,
    bus::Bus,
    core::{
        bits::Bits,
        fault::Fault,
        fpregister::{FPSCRRounding, Fpscr},
        register::{ExtensionRegOperations, SingleReg},
    },
    peripheral::{
        mpu::{AccType, Mpu},
        scb::{
            FPCCR_ASPEN, FPCCR_LSPACT, FPCCR_USER, FPSCR_STATUS_CONTROL_END,
            FPSCR_STATUS_CONTROL_START,
        },
    },
};

/// A floating point format, in terms of the integer which holds a
/// value of it: u32 for single precision, and u64 for double.
pub trait FloatOps {
    /// The integer type.
    type Bits: Copy;

    /// How many bits that has.
    fn n() -> u32;

    fn add(op1: Self::Bits, op2: Self::Bits) -> Self::Bits;
    fn sub(op1: Self::Bits, op2: Self::Bits) -> Self::Bits;
    fn mul(op1: Self::Bits, op2: Self::Bits) -> Self::Bits;
    fn div(op1: Self::Bits, op2: Self::Bits) -> Self::Bits;
    /// addend + op1 * op2, rounded once.
    fn mul_add(addend: Self::Bits, op1: Self::Bits, op2: Self::Bits) -> Self::Bits;
    fn sqrt(op: Self::Bits) -> Self::Bits;
    fn abs(op: Self::Bits) -> Self::Bits;

    /// The value as a double, which holds any value of either format
    /// exactly.
    fn to_f64(op: Self::Bits) -> f64;
    /// The value of this format nearest to a double.
    fn from_f64(value: f64) -> Self::Bits;

    /// The integer as one of 64 bits, zero extended.
    fn to_u64(op: Self::Bits) -> u64;
    /// The low bits of a 64 bit integer.
    fn from_u64(value: u64) -> Self::Bits;
}

impl FloatOps for u32 {
    type Bits = u32;

    fn n() -> u32 {
        32
    }
    fn add(op1: u32, op2: u32) -> u32 {
        (f32::from_bits(op1) + f32::from_bits(op2)).to_bits()
    }
    fn sub(op1: u32, op2: u32) -> u32 {
        (f32::from_bits(op1) - f32::from_bits(op2)).to_bits()
    }
    fn mul(op1: u32, op2: u32) -> u32 {
        (f32::from_bits(op1) * f32::from_bits(op2)).to_bits()
    }
    fn div(op1: u32, op2: u32) -> u32 {
        (f32::from_bits(op1) / f32::from_bits(op2)).to_bits()
    }
    fn mul_add(addend: u32, op1: u32, op2: u32) -> u32 {
        f32::from_bits(op1)
            .mul_add(f32::from_bits(op2), f32::from_bits(addend))
            .to_bits()
    }
    fn sqrt(op: u32) -> u32 {
        f32::from_bits(op).sqrt().to_bits()
    }
    fn abs(op: u32) -> u32 {
        op & 0x7fff_ffff
    }
    fn to_f64(op: u32) -> f64 {
        f64::from(f32::from_bits(op))
    }
    fn from_f64(value: f64) -> u32 {
        (value as f32).to_bits()
    }
    fn to_u64(op: u32) -> u64 {
        u64::from(op)
    }
    fn from_u64(value: u64) -> u32 {
        value as u32
    }
}

impl FloatOps for u64 {
    type Bits = u64;

    fn n() -> u32 {
        64
    }
    fn add(op1: u64, op2: u64) -> u64 {
        (f64::from_bits(op1) + f64::from_bits(op2)).to_bits()
    }
    fn sub(op1: u64, op2: u64) -> u64 {
        (f64::from_bits(op1) - f64::from_bits(op2)).to_bits()
    }
    fn mul(op1: u64, op2: u64) -> u64 {
        (f64::from_bits(op1) * f64::from_bits(op2)).to_bits()
    }
    fn div(op1: u64, op2: u64) -> u64 {
        (f64::from_bits(op1) / f64::from_bits(op2)).to_bits()
    }
    fn mul_add(addend: u64, op1: u64, op2: u64) -> u64 {
        f64::from_bits(op1)
            .mul_add(f64::from_bits(op2), f64::from_bits(addend))
            .to_bits()
    }
    fn sqrt(op: u64) -> u64 {
        f64::from_bits(op).sqrt().to_bits()
    }
    fn abs(op: u64) -> u64 {
        op & 0x7fff_ffff_ffff_ffff
    }
    fn to_f64(op: u64) -> f64 {
        f64::from_bits(op)
    }
    fn from_f64(value: f64) -> u64 {
        value.to_bits()
    }
    fn to_u64(op: u64) -> u64 {
        op
    }
    fn from_u64(value: u64) -> u64 {
        value
    }
}

/// Round to an integral value, in the given direction.
fn round(value: f64, rounding: FPSCRRounding) -> f64 {
    match rounding {
        FPSCRRounding::RoundToNearest => value.round_ties_even(),
        FPSCRRounding::RoundTowardsPlusInfinity => value.ceil(),
        FPSCRRounding::RoundTowardsMinusInfinity => value.floor(),
        FPSCRRounding::RoundTowardsZero => value.trunc(),
    }
}

pub trait FloatingPointChecks {
    fn check_vfp_enabled(&mut self) -> Result<(), Fault>;
    fn execute_fp_check(&mut self) -> Result<(), Fault>;
}

impl Processor {
    fn preserve_fp_state(&mut self) -> Result<(), Fault> {
        let acctype = if !self.fpccr.get_bit(FPCCR_USER) {
            AccType::Normal
        } else {
            AccType::UnPriv
        };

        for i in 0..16 {
            let memaddrdesc =
                self.validate_address(self.fpcar.wrapping_add(i * 4), acctype, true)?;
            let reg = SingleReg::from(i as u8);
            let value = self.get_sr(reg);
            self.write32(memaddrdesc, value)?;
        }
        let memaddrdesc = self.validate_address(self.fpcar.wrapping_add(0x40), acctype, true)?;
        self.write32(memaddrdesc, self.fpscr)?;
        self.fpccr.set_bit(FPCCR_LSPACT, false);
        Ok(())
    }
}

pub trait FloatingPointPublicOperations {
    fn fp_add<T: FloatOps>(
        &mut self,
        op1: T::Bits,
        op2: T::Bits,
        fpscr_controlled: bool,
    ) -> T::Bits;

    fn fp_sub<T: FloatOps>(
        &mut self,
        op1: T::Bits,
        op2: T::Bits,
        fpscr_controlled: bool,
    ) -> T::Bits;

    fn fp_mul_add<T: FloatOps>(
        &mut self,
        addend: T::Bits,
        op1: T::Bits,
        op2: T::Bits,
        fpscr_controlled: bool,
    ) -> T::Bits;

    fn fp_mul<T: FloatOps>(
        &mut self,
        op1: T::Bits,
        op2: T::Bits,
        fpscr_controlled: bool,
    ) -> T::Bits;

    fn fp_div<T: FloatOps>(
        &mut self,
        op1: T::Bits,
        op2: T::Bits,
        fpscr_controlled: bool,
    ) -> T::Bits;

    fn fp_compare<T: FloatOps>(
        &mut self,
        op1: T::Bits,
        op2: T::Bits,
        quiet_nan_exc: bool,
        fpscr_controlled: bool,
    ) -> (bool, bool, bool, bool);

    fn fp_abs<T: FloatOps>(&mut self, op: T::Bits) -> T::Bits;

    fn fp_sqrt<T: FloatOps>(&mut self, op: T::Bits, fpscr_controlled: bool) -> T::Bits;

    fn fp_round_int<T: FloatOps>(
        &mut self,
        op: T::Bits,
        zero_rounding: bool,
        exact: bool,
        fpscr_controlled: bool,
    ) -> T::Bits;

    fn fp_to_fixed<N: FloatOps, M: FloatOps>(
        &mut self,
        op: N::Bits,
        fraction_bits: usize,
        unsigned: bool,
        round_towards_zero: bool,
        fpscr_controlled: bool,
    ) -> M::Bits;

    fn fixed_to_fp<N: FloatOps, M: FloatOps>(
        &mut self,
        op: M::Bits,
        fraction_bits: usize,
        unsigned: bool,
        round_to_nearest: bool,
        fpscr_controlled: bool,
    ) -> N::Bits;
}

impl FloatingPointPublicOperations for Processor {
    fn fp_add<T: FloatOps>(&mut self, op1: T::Bits, op2: T::Bits, _: bool) -> T::Bits {
        T::add(op1, op2)
    }

    fn fp_sub<T: FloatOps>(&mut self, op1: T::Bits, op2: T::Bits, _: bool) -> T::Bits {
        T::sub(op1, op2)
    }

    fn fp_mul_add<T: FloatOps>(
        &mut self,
        addend: T::Bits,
        op1: T::Bits,
        op2: T::Bits,
        _: bool,
    ) -> T::Bits {
        T::mul_add(addend, op1, op2)
    }

    fn fp_mul<T: FloatOps>(&mut self, op1: T::Bits, op2: T::Bits, _: bool) -> T::Bits {
        T::mul(op1, op2)
    }

    fn fp_div<T: FloatOps>(&mut self, op1: T::Bits, op2: T::Bits, _: bool) -> T::Bits {
        T::div(op1, op2)
    }

    /// Returns the N, Z, C and V flags.
    fn fp_compare<T: FloatOps>(
        &mut self,
        op1: T::Bits,
        op2: T::Bits,
        _quiet_nan_exc: bool,
        _: bool,
    ) -> (bool, bool, bool, bool) {
        match T::to_f64(op1).partial_cmp(&T::to_f64(op2)) {
            None => (false, false, true, true),
            Some(std::cmp::Ordering::Equal) => (false, true, true, false),
            Some(std::cmp::Ordering::Less) => (true, false, false, false),
            Some(std::cmp::Ordering::Greater) => (false, false, true, false),
        }
    }

    fn fp_abs<T: FloatOps>(&mut self, op: T::Bits) -> T::Bits {
        T::abs(op)
    }

    fn fp_sqrt<T: FloatOps>(&mut self, op: T::Bits, _: bool) -> T::Bits {
        T::sqrt(op)
    }

    fn fp_round_int<T: FloatOps>(
        &mut self,
        op: T::Bits,
        zero_rounding: bool,
        _exact: bool,
        _: bool,
    ) -> T::Bits {
        let rounding = if zero_rounding {
            FPSCRRounding::RoundTowardsZero
        } else {
            self.fpscr.get_rounding_mode()
        };
        let value = T::to_f64(op);
        // Rounding keeps the sign of what rounds to zero, and leaves
        // an infinity or a NaN as it was.
        T::from_f64(round(value, rounding).copysign(value))
    }

    fn fp_to_fixed<N: FloatOps, M: FloatOps>(
        &mut self,
        op: N::Bits,
        fraction_bits: usize,
        unsigned: bool,
        round_towards_zero: bool,
        _: bool,
    ) -> M::Bits {
        let rounding = if round_towards_zero {
            FPSCRRounding::RoundTowardsZero
        } else {
            self.fpscr.get_rounding_mode()
        };
        // Scaling by a power of two is exact, short of overflowing to
        // an infinity, which then saturates as it should.
        let scaled = round(N::to_f64(op) * 2f64.powi(fraction_bits as i32), rounding);
        // A cast saturates, and takes a NaN to zero, as is required.
        let result = match (unsigned, M::n()) {
            (true, 32) => u64::from(scaled as u32),
            (false, 32) => u64::from(scaled as i32 as u32),
            (true, _) => scaled as u64,
            (false, _) => scaled as i64 as u64,
        };
        M::from_u64(result)
    }

    fn fixed_to_fp<N: FloatOps, M: FloatOps>(
        &mut self,
        op: M::Bits,
        fraction_bits: usize,
        unsigned: bool,
        _round_to_nearest: bool,
        _: bool,
    ) -> N::Bits {
        let bits = M::to_u64(op);
        // A 32 bit integer is exact as a double, so that the only
        // rounding is in going on to the result.  A 64 bit integer
        // is rounded here, and the result is double precision too.
        let value = match (unsigned, M::n()) {
            (true, 32) => f64::from(bits as u32),
            (false, 32) => f64::from(bits as u32 as i32),
            (true, _) => bits as f64,
            (false, _) => bits as i64 as f64,
        };
        N::from_f64(value / 2f64.powi(fraction_bits as i32))
    }
}

impl FloatingPointChecks for Processor {
    fn check_vfp_enabled(&mut self) -> Result<(), Fault> {
        let cp10 = self.cpacr.get_bits(20..22);
        let cp11 = self.cpacr.get_bits(22..24);

        if cp10 != cp11 {
            return Err(Fault::Forced);
        }

        match cp10 {
            0b00 => {
                self.cfsr.set_bit(19, true);
                Err(Fault::Nocp)
            }
            0b01 => {
                if !self.current_mode_is_privileged() {
                    self.cfsr.set_bit(19, true);
                    Err(Fault::Nocp)
                } else {
                    Ok(())
                }
            }
            0b10 => {
                self.cfsr.set_bit(19, true);
                Err(Fault::Nocp)
            }
            0b11 => Ok(()), // full access
            _ => unreachable!(),
        }
    }
    fn execute_fp_check(&mut self) -> Result<(), Fault> {
        self.check_vfp_enabled()?;

        if self.fpccr.get_bit(FPCCR_LSPACT) {
            self.preserve_fp_state()?;
        }

        if self.fpccr.get_bit(FPCCR_ASPEN) && !self.control.fpca {
            let fpdscr_bits = self
                .fpdscr
                .get_bits(FPSCR_STATUS_CONTROL_START..FPSCR_STATUS_CONTROL_END);
            self.fpscr.set_bits(
                FPSCR_STATUS_CONTROL_START..FPSCR_STATUS_CONTROL_END,
                fpdscr_bits,
            );
            self.control.fpca = true;
        }
        Ok(())
    }
}

