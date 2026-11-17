//!
//! Cortex System Tick Simulation
//!

use crate::Processor;
use crate::core::bits::Bits;
use crate::core::exception::Exception;
use crate::core::exception::ExceptionHandling;

///
/// Register API for ```SysTick``` peripheral
///
pub trait SysTick {
    ///
    /// write to `SYST_RVR`, reload value register
    ///
    fn syst_write_rvr(&mut self, value: u32);

    ///
    /// write to current value register
    ///
    fn syst_write_cvr(&mut self, _value: u32);

    ///
    /// write to control and status register
    ///
    fn syst_write_csr(&mut self, value: u32);

    ///
    /// Read control and status register
    ///
    fn syst_read_csr(&mut self) -> u32;

    ///
    /// Read reload value register
    ///
    fn syst_read_rvr(&self) -> u32;

    ///
    /// Read current value register
    ///
    fn syst_read_cvr(&self) -> u32;

    ///
    /// Read calibration register value
    ///
    fn syst_read_calib(&self) -> u32;

    ///
    /// Step systick ```cycles``` clock cycles forward
    ///
    fn syst_step(&mut self, cycles: u32);
}

const SYST_CSR_ENABLE: u32 = 1;
const SYST_CSR_TICKINT: u32 = 1 << 1;
const SYST_CSR_COUNTFLAG: u32 = 1 << 16;

impl SysTick for Processor {
    fn syst_write_rvr(&mut self, value: u32) {
        self.syst_rvr = value & 0x00ff_ffff;
    }

    fn syst_write_cvr(&mut self, _value: u32) {
        self.syst_cvr = 0;

        // writing to CVR always clears countflag
        self.syst_csr &= !SYST_CSR_COUNTFLAG;
    }

    fn syst_write_csr(&mut self, value: u32) {
        self.syst_csr.set_bits(0..3, value.get_bits(0..3));
    }

    fn syst_read_csr(&mut self) -> u32 {
        let res = self.syst_csr;
        self.syst_csr &= !SYST_CSR_COUNTFLAG;
        res
    }

    fn syst_read_rvr(&self) -> u32 {
        self.syst_rvr
    }

    fn syst_read_cvr(&self) -> u32 {
        self.syst_cvr
    }

    fn syst_read_calib(&self) -> u32 {
        0
    }

    #[inline(always)]
    fn syst_step(&mut self, cycles: u32) {
        if (self.syst_csr & SYST_CSR_ENABLE) == 0 {
            return;
        }

        for _ in 0..cycles {
            if self.syst_cvr > 0 {
                self.syst_cvr -= 1;

                if self.syst_cvr == 0 {
                    self.syst_csr |= SYST_CSR_COUNTFLAG;
                    if (self.syst_csr & SYST_CSR_TICKINT) == SYST_CSR_TICKINT {
                        self.set_exception_pending(Exception::SysTick);
                    }
                }
            } else {
                self.syst_cvr = self.syst_rvr & 0x00ff_ffff;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::reset::Reset;

    #[test]
    fn test_nvic_rvr() {
        // Arrange
        let mut processor = Processor::new();

        processor.reset().unwrap();

        // Act
        processor.syst_write_rvr(0xffff_ffff);

        // Assert
        assert_eq!(processor.syst_read_rvr(), 0x00ff_ffff);
    }

    #[test]
    fn test_nvic_cvr() {
        // Arrange
        let mut processor = Processor::new();

        processor.reset().unwrap();

        // Act
        processor.syst_write_cvr(0xffff_ffff);

        // Assert
        assert_eq!(processor.syst_read_cvr(), 0);

        // Act
        processor.syst_write_cvr(0x1);

        // Assert
        assert_eq!(processor.syst_read_cvr(), 0);

        // Act
        processor.syst_write_cvr(42);

        // Assert
        assert_eq!(processor.syst_read_cvr(), 0);
    }

    #[test]
    fn test_nvic_csr() {
        // Arrange
        let mut processor = Processor::new();

        processor.reset().unwrap();

        // Act
        processor.syst_write_csr(0xffff_ffff);

        // Assert
        assert_eq!(processor.syst_read_csr(), 0b111);
    }

    #[test]
    fn test_nvic_reading_csr_clears_countflag() {
        // Arrange
        let mut processor = Processor::new();

        processor.reset().unwrap();

        //Arrange
        processor.syst_write_rvr(1);
        processor.syst_write_cvr(0);
        processor.syst_write_csr(SYST_CSR_ENABLE);

        // Act
        processor.syst_step(2);

        // Assert
        assert_eq!(
            processor.syst_read_csr(),
            SYST_CSR_COUNTFLAG | SYST_CSR_ENABLE
        );
        assert_eq!(processor.syst_read_csr(), SYST_CSR_ENABLE);
    }

    #[test]
    fn test_nvic_writing_cvr_clears_countflag() {
        // Arrange
        let mut processor = Processor::new();

        processor.reset().unwrap();

        //Arrange
        processor.syst_write_rvr(1);
        processor.syst_write_cvr(0);
        processor.syst_write_csr(SYST_CSR_ENABLE);
        processor.syst_step(2);

        // Act
        processor.syst_write_cvr(42);

        // Assert
        assert_eq!(processor.syst_read_csr(), SYST_CSR_ENABLE);
    }

    #[test]
    fn test_nvic_exception_is_set_pending_on_reaching_zero() {
        // Arrange
        let mut processor = Processor::new();

        processor.reset().unwrap();

        //Arrange
        processor.syst_write_rvr(1);
        processor.syst_write_cvr(0);
        processor.syst_write_csr(SYST_CSR_ENABLE | SYST_CSR_TICKINT);

        // Act
        processor.syst_step(2);

        // Assert
        assert_eq!(processor.get_pending_exception(), Some(Exception::SysTick));

        assert_eq!(
            processor.syst_read_csr(),
            SYST_CSR_COUNTFLAG | SYST_CSR_ENABLE | SYST_CSR_TICKINT
        );
    }
}
