//!
//! Processor Bus related operations
//!

use crate::Processor;

use crate::core::fault::Fault;
use crate::memory::map::MapMemory;
use crate::peripheral::dwt::Dwt;
use crate::peripheral::itm::InstrumentationTraceMacrocell;
use crate::peripheral::nvic::NVIC;
use crate::peripheral::scb::SystemControlBlock;
use crate::peripheral::systick::SysTick;

///
/// Trait for reading and writing via a memory bus.
///
pub trait Bus {
    /// Reads a 32 bit value via the bus from the given address.
    ///
    fn read32(&mut self, addr: u32) -> Result<u32, Fault>;

    /// Reads a 16 bit value via the bus from the given address.
    ///
    fn read16(&self, addr: u32) -> Result<u16, Fault>;

    /// Reads a 8 bit value via the bus from the given address.
    ///
    fn read8(&self, addr: u32) -> Result<u8, Fault>;

    /// Writes a 32 bit value to the bus targeting the given address.
    ///
    fn write32(&mut self, addr: u32, value: u32) -> Result<(), Fault>;

    /// Writes a 16 bit value to the bus targeting the given address.
    ///
    fn write16(&mut self, addr: u32, value: u16) -> Result<(), Fault>;

    /// Writes a 8 bit value to the bus targeting the given address.
    ///
    fn write8(&mut self, addr: u32, value: u8) -> Result<(), Fault>;

    /// Checks if given address can be reached via the bus.
    ///
    fn in_range(&self, addr: u32) -> bool;
}

impl Bus for Processor {
    #[inline(always)]
    fn read8(&self, bus_addr: u32) -> Result<u8, Fault> {
        let addr = self.map_address(bus_addr);

        if self.sram.in_range(addr) {
            return self.sram.read8(addr);
        }
        if self.code.in_range(addr) {
            return self.code.read8(addr);
        }
        if let Some(device) = self.device.as_ref()
            && device.in_range(addr)
        {
            return device.read8(addr);
        }

        let result = match addr {
            0xE000_E400..=0xE000_E5EC => {
                self.nvic_read_ipr_u8(((addr - 0xE000_E400) >> 2) as usize)
            }
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED18..=0xE000_ED1B => self.read_shpr1_u8((addr - 0xE000_ED18) as usize),
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED1C..=0xE000_ED1F => self.read_shpr2_u8((addr - 0xE000_ED1C) as usize),
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED20..=0xE000_ED23 => self.read_shpr3_u8((addr - 0xE000_ED20) as usize),
            _ => return Err(Fault::DAccViol),
        };
        Ok(result)
    }

    #[inline(always)]
    fn read16(&self, bus_addr: u32) -> Result<u16, Fault> {
        let addr = self.map_address(bus_addr);

        if self.sram.in_range(addr) {
            return self.sram.read16(addr);
        }
        if self.code.in_range(addr) {
            return self.code.read16(addr);
        }
        if let Some(device) = self.device.as_ref()
            && device.in_range(addr)
        {
            return device.read16(addr);
        }

        match addr {
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED18..=0xE000_ED1B => {
                Ok(self.read_shpr1_u16(((addr - 0xE000_ED18) >> 1) as usize))
            }
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED1C..=0xE000_ED1F => {
                Ok(self.read_shpr2_u16(((addr - 0xE000_ED1C) >> 1) as usize))
            }
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED20..=0xE000_ED23 => {
                Ok(self.read_shpr3_u16(((addr - 0xE000_ED20) >> 1) as usize))
            }
            0xE000_E400..=0xE000_E5EC => {
                Ok(self.nvic_read_ipr_u16(((addr - 0xE000_E400) >> 1) as usize))
            }
            _ => Err(Fault::DAccViol),
        }
    }

    #[inline(always)]
    fn read32(&mut self, bus_addr: u32) -> Result<u32, Fault> {
        let addr = self.map_address(bus_addr);

        if self.sram.in_range(addr) {
            return self.sram.read32(addr);
        }
        if self.code.in_range(addr) {
            return self.code.read32(addr);
        }
        if let Some(device) = self.device.as_mut()
            && device.in_range(addr)
        {
            return device.read32(addr);
        }

        let result = match addr {
            0xE000_0000 => self.read_stim0(),

            0xE000_1004 => self.dwt_cyccnt,

            0xE000_E004 => self.ictr,
            0xE000_E008 => self.actlr,
            0xE000_E010 => self.syst_read_csr(),
            0xE000_E014 => self.syst_read_rvr(),
            0xE000_E018 => self.syst_read_cvr(),
            0xE000_E01C => self.syst_read_calib(),
            0xE000_E100..=0xE000_E13C => self.nvic_read_iser(((addr - 0xE000_E100) >> 5) as usize),
            0xE000_E180..=0xE000_E1BC => self.nvic_read_icer(((addr - 0xE000_E180) >> 5) as usize),
            0xE000_E200..=0xE000_E23C => self.nvic_read_ispr(((addr - 0xE000_E200) >> 5) as usize),
            0xE000_E280..=0xE000_E2BC => self.nvic_read_icpr(((addr - 0xE000_E280) >> 5) as usize),
            0xE000_E300..=0xE000_E33C => self.nvic_read_iabr(((addr - 0xE000_E300) >> 5) as usize),
            0xE000_E400..=0xE000_E5EC => self.nvic_read_ipr(((addr - 0xE000_E400) >> 2) as usize),

            0xE000_ED00 => self.cpuid,
            0xE000_ED04 => self.read_icsr(),
            0xE000_ED08 => self.read_vtor(),
            0xE000_ED0C => self.aircr,
            0xE000_ED10 => self.read_scr(),
            0xE000_ED14 => self.ccr,
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED18 => self.read_shpr1(),
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED1C => self.read_shpr2(),
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED20 => self.read_shpr3(),
            0xE000_ED24 => self.read_shcsr(),
            #[cfg(not(feature = "armv6m"))]
            0xE000_ED28 => self.cfsr,
            #[cfg(not(feature = "armv6m"))]
            0xE000_ED2C => self.hfsr,
            #[cfg(not(feature = "armv6m"))]
            0xE000_ED30 => self.dfsr,
            #[cfg(not(feature = "armv6m"))]
            0xE000_ED34 => self.mmfar,
            #[cfg(not(feature = "armv6m"))]
            0xE000_ED38 => self.bfar,
            #[cfg(not(feature = "armv6m"))]
            0xE000_ED3C => self.afsr,

            #[cfg(feature = "has-fp")]
            0xE000_ED88 => self.cpacr,

            #[cfg(feature = "has-fp")]
            0xE000_EF34 => self.fpccr,
            #[cfg(feature = "has-fp")]
            0xE000_EF38 => self.fpcar,
            #[cfg(feature = "has-fp")]
            0xE000_EF3C => self.fpdscr,

            #[cfg(feature = "has-fp")]
            0xE000_EF40 => self.mvfr0,
            #[cfg(feature = "has-fp")]
            0xE000_EF44 => self.mvfr1,
            #[cfg(feature = "has-fp")]
            0xE000_EF48 => self.mvfr2,

            0xE000_EDFC => self.read_demcr(),

            // DWT
            0xE000_1000 => self.dwt_ctrl,
            _ => return Err(Fault::DAccViol),
        };
        Ok(result)
    }

    #[inline(always)]
    fn write32(&mut self, bus_addr: u32, value: u32) -> Result<(), Fault> {
        let addr = self.map_address(bus_addr);

        if self.sram.in_range(addr) {
            return self.sram.write32(addr, value);
        }
        if self.code.in_range(addr) {
            return self.code.write32(addr, value);
        }
        if let Some(device) = self.device.as_mut()
            && device.in_range(addr)
        {
            return device.write32(addr, value);
        }

        match addr {
            0xE000_0000..=0xE000_007C => {
                self.write_stim_u32(((addr - 0xE000_0000) >> 2) as u8, value);
            }

            0xE000_1000 => self.dwt_write_ctrl(value),
            0xE000_1004 => self.dwt_write_cyccnt(value),

            0xE000_1FB0 => self.itm_write_lar_u32(value),

            0xE000_ED04 => self.write_icsr(value),
            0xE000_ED08 => self.write_vtor(value),
            0xE000_ED10 => self.write_scr(value),
            0xE000_ED24 => self.write_shcsr(value),
            #[cfg(not(feature = "armv6m"))]
            0xE000_ED28 => self.write_cfsr(value),
            #[cfg(not(feature = "armv6m"))]
            0xE000_ED2C => self.write_hfsr(value),
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED18 => self.write_shpr1(value),
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED1C => self.write_shpr2(value),
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED20 => self.write_shpr3(value),
            #[cfg(feature = "has-fp")]
            0xE000_ED88 => self.write_cpacr(value)?,
            #[cfg(feature = "has-fp")]
            0xE000_EF34 => self.write_fpccr(value)?,
            #[cfg(feature = "has-fp")]
            0xE000_EF38 => self.write_fpcar(value)?,
            #[cfg(feature = "has-fp")]
            0xE000_EF3C => self.write_fpdscr(value)?,

            0xE000_EDFC => self.write_demcr(value),

            0xE000_E010 => self.syst_write_csr(value),
            0xE000_E014 => self.syst_write_rvr(value),
            0xE000_E018 => self.syst_write_cvr(value),
            0xE000_E100..=0xE000_E13C => {
                self.nvic_write_iser(((addr - 0xE000_E100) >> 5) as usize, value);
            }
            0xE000_E180..=0xE000_E1BC => {
                self.nvic_write_icer(((addr - 0xE000_E180) >> 5) as usize, value);
            }
            0xE000_E200..=0xE000_E23C => {
                self.nvic_write_ispr(((addr - 0xE000_E200) >> 5) as usize, value);
            }
            0xE000_E280..=0xE000_E2BC => {
                self.nvic_write_icpr(((addr - 0xE000_E280) >> 5) as usize, value);
            }
            0xE000_E400..=0xE000_E5EC => {
                self.nvic_write_ipr(((addr - 0xE000_E400) >> 2) as usize, value);
            }

            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_EF00 => self.write_stir(value),
            _ => return Err(Fault::DAccViol),
        }
        Ok(())
    }

    #[inline(always)]
    fn write16(&mut self, bus_addr: u32, value: u16) -> Result<(), Fault> {
        let addr = self.map_address(bus_addr);

        if self.sram.in_range(addr) {
            return self.sram.write16(addr, value);
        }
        if self.code.in_range(addr) {
            return self.code.write16(addr, value);
        }
        if let Some(device) = self.device.as_mut()
            && device.in_range(addr)
        {
            return device.write16(addr, value);
        }

        match addr {
            0xE000_0000..=0xE000_007C => {
                self.write_stim_u16(((addr - 0xE000_0000) >> 2) as u8, value);
            }
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED18..=0xE000_ED1B => {
                self.write_shpr1_u16(((addr - 0xE000_ED18) >> 1) as usize, value);
            }
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED1C..=0xE000_ED1F => {
                self.write_shpr2_u16(((addr - 0xE000_ED1C) >> 1) as usize, value);
            }
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED20..=0xE000_ED23 => {
                self.write_shpr3_u16(((addr - 0xE000_ED20) >> 1) as usize, value);
            }
            0xE000_E400..=0xE000_E5EC => {
                self.nvic_write_ipr_u16(((addr - 0xE000_E400) >> 1) as usize, value);
            }
            _ => return Err(Fault::DAccViol),
        }
        Ok(())
    }

    #[inline(always)]
    fn write8(&mut self, bus_addr: u32, value: u8) -> Result<(), Fault> {
        let addr = self.map_address(bus_addr);

        if self.sram.in_range(addr) {
            return self.sram.write8(addr, value);
        }
        if self.code.in_range(addr) {
            return self.code.write8(addr, value);
        }
        if let Some(device) = self.device.as_mut()
            && device.in_range(addr)
        {
            return device.write8(addr, value);
        }

        match addr {
            0xE000_0000..=0xE000_007C => {
                self.write_stim_u8(((addr - 0xE000_0000) >> 2) as u8, value);
            }
            0xE000_E400..=0xE000_E5EC => {
                self.nvic_write_ipr_u8((addr - 0xE000_E400) as usize, value);
            }
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED18..=0xE000_ED1B => self.write_shpr1_u8((addr - 0xE000_ED18) as usize, value),
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED1C..=0xE000_ED1F => self.write_shpr2_u8((addr - 0xE000_ED1C) as usize, value),
            #[cfg(any(feature = "armv7m", feature = "armv7em"))]
            0xE000_ED20..=0xE000_ED23 => self.write_shpr3_u8((addr - 0xE000_ED20) as usize, value),
            _ => return Err(Fault::DAccViol),
        }
        Ok(())
    }

    #[allow(unused)]
    fn in_range(&self, addr: u32) -> bool {
        self.code.in_range(addr)
            || self.sram.in_range(addr)
            || self
                .device
                .as_ref()
                .is_some_and(|device| device.in_range(addr))
    }
}

#[cfg(test)]
mod tests {
    use super::Bus;
    use crate::Processor;
    #[cfg(feature = "armv6m")]
    use crate::core::exception::{Exception, ExceptionHandling};
    use crate::core::fault::Fault;
    #[cfg(feature = "has-fp")]
    use crate::peripheral::scb::{FPCCR_ASPEN, FPCCR_LSPEN};

    #[cfg(feature = "armv6m")]
    const SHCSR_SVCALLPENDED: u32 = 1 << 15;

    #[test]
    #[cfg(feature = "armv6m")]
    fn test_armv6m_rejects_v7_fault_status_registers() {
        let mut processor = Processor::new();

        for address in [
            0xE000_ED28,
            0xE000_ED2C,
            0xE000_ED30,
            0xE000_ED34,
            0xE000_ED38,
            0xE000_ED3C,
        ] {
            assert_eq!(processor.read32(address), Err(Fault::DAccViol));
            assert_eq!(processor.write32(address, 0), Err(Fault::DAccViol));
        }
    }

    #[test]
    #[cfg(feature = "armv6m")]
    fn test_armv6m_exposes_shcsr() {
        let mut processor = Processor::new();

        processor.shcsr = 0x0000_0008;

        assert_eq!(processor.read32(0xE000_ED24), Ok(0x0000_0008));

        processor.write32(0xE000_ED24, u32::MAX).unwrap();

        assert!(processor.exception_pending(Exception::SVCall));
        assert_eq!(processor.read32(0xE000_ED24), Ok(0x0000_8008));
    }

    #[test]
    #[cfg(feature = "armv6m")]
    fn test_armv6m_shcsr_reflects_svcall_pending_state() {
        let mut processor = Processor::new();

        processor.set_exception_pending(Exception::SVCall);

        assert_eq!(processor.read32(0xE000_ED24), Ok(SHCSR_SVCALLPENDED));

        processor.clear_pending_exception(Exception::SVCall);

        assert_eq!(processor.read32(0xE000_ED24), Ok(0));
    }

    #[test]
    #[cfg(feature = "armv6m")]
    fn test_armv6m_shcsr_write_sets_and_clears_svcall_pending() {
        let mut processor = Processor::new();

        processor.write32(0xE000_ED24, SHCSR_SVCALLPENDED).unwrap();

        assert!(processor.exception_pending(Exception::SVCall));
        assert_eq!(processor.read32(0xE000_ED24), Ok(SHCSR_SVCALLPENDED));

        processor.write32(0xE000_ED24, 0).unwrap();

        assert!(!processor.exception_pending(Exception::SVCall));
        assert_eq!(processor.read32(0xE000_ED24), Ok(0));
    }

    #[test]
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn test_armv7_profiles_expose_fault_status_registers() {
        let mut processor = Processor::new();

        processor.shcsr = 0x1234_0008;
        processor.cfsr = 0x0103_9187;
        processor.hfsr = 0x4000_0002;
        processor.dfsr = 0x0000_001f;
        processor.mmfar = 0x2000_1234;
        processor.bfar = 0x4000_5678;
        processor.afsr = 0x89ab_cdef;

        assert_eq!(processor.read32(0xE000_ED24), Ok(0x1234_0000));
        assert_eq!(processor.read32(0xE000_ED28), Ok(0x0103_9187));
        assert_eq!(processor.read32(0xE000_ED2C), Ok(0x4000_0002));
        assert_eq!(processor.read32(0xE000_ED30), Ok(0x0000_001f));
        assert_eq!(processor.read32(0xE000_ED34), Ok(0x2000_1234));
        assert_eq!(processor.read32(0xE000_ED38), Ok(0x4000_5678));
        assert_eq!(processor.read32(0xE000_ED3C), Ok(0x89ab_cdef));
    }

    #[test]
    #[cfg(all(any(feature = "armv7m", feature = "armv7em"), not(feature = "has-fp")))]
    fn test_non_vfp_profiles_reject_fp_system_registers() {
        let mut processor = Processor::new();

        for address in [
            0xE000_ED88,
            0xE000_EF34,
            0xE000_EF38,
            0xE000_EF3C,
            0xE000_EF40,
            0xE000_EF44,
            0xE000_EF48,
        ] {
            assert_eq!(processor.read32(address), Err(Fault::DAccViol));
        }
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_vfp_profile_exposes_fp_system_registers() {
        #[cfg(feature = "fpv5-d16")]
        const EXPECTED_MVFR0: u32 = 0x1011_0221;
        #[cfg(any(feature = "fpv4-sp-d16", feature = "fpv5-sp-d16"))]
        const EXPECTED_MVFR0: u32 = 0x1011_0021;

        #[cfg(feature = "fpv5-d16")]
        const EXPECTED_MVFR1: u32 = 0x1200_0011;
        #[cfg(any(feature = "fpv4-sp-d16", feature = "fpv5-sp-d16"))]
        const EXPECTED_MVFR1: u32 = 0x1100_0011;

        #[cfg(any(feature = "fpv5-d16", feature = "fpv5-sp-d16"))]
        const EXPECTED_MVFR2: u32 = 0x0000_0040;
        #[cfg(feature = "fpv4-sp-d16")]
        const EXPECTED_MVFR2: u32 = 0x0000_0000;

        let mut processor = Processor::new();

        processor.cpacr = 0x00f0_0000;
        processor.fpccr = 0xc000_0039;
        processor.fpcar = 0x2000_0100;
        processor.fpdscr = 0x00ab_0000;

        assert_eq!(processor.read32(0xE000_ED88), Ok(0x00f0_0000));
        assert_eq!(processor.read32(0xE000_EF34), Ok(0xc000_0039));
        assert_eq!(processor.read32(0xE000_EF38), Ok(0x2000_0100));
        assert_eq!(processor.read32(0xE000_EF3C), Ok(0x00ab_0000));
        assert_eq!(processor.read32(0xE000_EF40), Ok(EXPECTED_MVFR0));
        assert_eq!(processor.read32(0xE000_EF44), Ok(EXPECTED_MVFR1));
        assert_eq!(processor.read32(0xE000_EF48), Ok(EXPECTED_MVFR2));
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_vfp_profile_writes_fp_system_registers() {
        let mut processor = Processor::new();
        let initial_cpacr = processor.cpacr;
        let initial_fpccr = processor.fpccr;
        let writable_cpacr_bits = 0x00f0_0000;
        let writable_fpccr_bits = (1 << FPCCR_ASPEN) | (1 << FPCCR_LSPEN);

        assert_eq!(processor.write32(0xE000_EF34, 0xffff_ffff), Ok(()));
        assert_eq!(processor.write32(0xE000_ED88, 0xffff_ffff), Ok(()));
        assert_eq!(processor.write32(0xE000_EF38, 0x2000_0107), Ok(()));
        assert_eq!(processor.write32(0xE000_EF3C, 0xffff_ffff), Ok(()));

        assert_eq!(processor.cpacr & writable_cpacr_bits, writable_cpacr_bits);
        assert_eq!(
            processor.cpacr & !writable_cpacr_bits,
            initial_cpacr & !writable_cpacr_bits
        );
        assert_eq!(processor.fpccr & writable_fpccr_bits, writable_fpccr_bits);
        assert_eq!(
            processor.fpccr & !writable_fpccr_bits,
            initial_fpccr & !writable_fpccr_bits
        );
        assert_eq!(processor.fpcar, 0x2000_0100);
        assert_eq!(processor.fpdscr, 0x07c0_0000);
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_unprivileged_thread_mode_cannot_write_fp_system_registers() {
        let mut processor = Processor::new();
        processor.control.n_priv = true;
        let initial_cpacr = processor.cpacr;
        let initial_fpccr = processor.fpccr;
        let initial_fpcar = processor.fpcar;
        let initial_fpdscr = processor.fpdscr;

        assert_eq!(
            processor.write32(0xE000_ED88, 0x00f0_0000),
            Err(Fault::DAccViol)
        );
        assert_eq!(
            processor.write32(0xE000_EF34, 0xc000_0000),
            Err(Fault::DAccViol)
        );
        assert_eq!(
            processor.write32(0xE000_EF38, 0x2000_0100),
            Err(Fault::DAccViol)
        );
        assert_eq!(
            processor.write32(0xE000_EF3C, 0x07c0_0000),
            Err(Fault::DAccViol)
        );

        assert_eq!(processor.cpacr, initial_cpacr);
        assert_eq!(processor.fpccr, initial_fpccr);
        assert_eq!(processor.fpcar, initial_fpcar);
        assert_eq!(processor.fpdscr, initial_fpdscr);
    }
}
