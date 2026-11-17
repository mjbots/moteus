//!
//! Cortex System Control Block Simulation
//!

use crate::Processor;
use crate::core::bits::Bits;
use crate::core::exception::Exception;
use crate::core::exception::ExceptionHandling;
use crate::core::fault::{Fault, FaultStatusContext};
#[cfg(feature = "has-fp")]
use crate::{FP_MVFR0_RESET, FP_MVFR1_RESET, FP_MVFR2_RESET};

use crate::core::register::Ipsr;

// Configuration Control Register bit positions
#[allow(dead_code)]
const CCR_NONBASETHRDENA: usize = 0;
#[allow(dead_code)]
const CCR_USERSETMPEND: usize = 1;
#[allow(dead_code)]
const CCR_UNALIGN_TRP: usize = 3;
#[allow(dead_code)]
const CCR_DIV_0_TRP: usize = 4;
#[allow(dead_code)]
const CCR_BFHFNMIGN: usize = 8;
pub(crate) const CCR_STKALIGN: usize = 9;
#[allow(dead_code)]
const CCR_DC: usize = 16;

// System Handler Control and State Register bit positions
const SHCSR_MEMFAULTACT: u32 = 1 << 0;
const SHCSR_BUSFAULTACT: u32 = 1 << 1;
const SHCSR_USGFAULTACT: u32 = 1 << 3;
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
const SHCSR_SVCALLACT: u32 = 1 << 7;
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
const SHCSR_MONITORACT: u32 = 1 << 8;
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
const SHCSR_PENDSVACT: u32 = 1 << 10;
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
const SHCSR_SYSTICKACT: u32 = 1 << 11;
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
const SHCSR_USGFAULTPENDED: u32 = 1 << 12;
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
const SHCSR_MEMFAULTPENDED: u32 = 1 << 13;
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
const SHCSR_BUSFAULTPENDED: u32 = 1 << 14;
const SHCSR_SVCALLPENDED: u32 = 1 << 15;
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
pub(crate) const SHCSR_MEMFAULTENA: u32 = 1 << 16;
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
pub(crate) const SHCSR_BUSFAULTENA: u32 = 1 << 17;
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
const SHCSR_USGFAULTENA: u32 = 1 << 18;

// Debug Exception and Monitor Control Register bit positions
#[cfg(feature = "has-fp")]
pub(crate) const DEMCR_MON_EN: usize = 16;

// Floating-Point Context Control Register bit positions
#[cfg(feature = "has-fp")]
pub(crate) const FPCCR_LSPACT: usize = 0;
#[cfg(feature = "has-fp")]
pub(crate) const FPCCR_USER: usize = 1;
#[cfg(feature = "has-fp")]
pub(crate) const FPCCR_THREAD: usize = 3;
#[cfg(feature = "has-fp")]
pub(crate) const FPCCR_HFRDY: usize = 4;
#[cfg(feature = "has-fp")]
pub(crate) const FPCCR_MMRDY: usize = 5;
#[cfg(feature = "has-fp")]
pub(crate) const FPCCR_BFRDY: usize = 6;
#[cfg(feature = "has-fp")]
pub(crate) const FPCCR_MONRDY: usize = 8;
#[cfg(feature = "has-fp")]
pub(crate) const FPCCR_LSPEN: usize = 30;
#[cfg(feature = "has-fp")]
pub(crate) const FPCCR_ASPEN: usize = 31;

// Shared FPSCR/FPDSCR status-control field range
#[cfg(feature = "has-fp")]
pub(crate) const FPSCR_STATUS_CONTROL_START: usize = 22;
#[cfg(feature = "has-fp")]
pub(crate) const FPSCR_STATUS_CONTROL_END: usize = 27;
#[cfg(feature = "has-fp")]
const CPACR_CP10_CP11_MASK: u32 = 0x00f0_0000;
#[cfg(feature = "has-fp")]
const FPCCR_WRITABLE_MASK: u32 = (1 << FPCCR_LSPEN) | (1 << FPCCR_ASPEN);
#[cfg(feature = "has-fp")]
const FPDSCR_WRITABLE_MASK: u32 = 0x07c0_0000;

#[cfg(any(feature = "armv7m", feature = "armv7em"))]
const SHCSR_ENABLE_MASK: u32 = SHCSR_MEMFAULTENA | SHCSR_BUSFAULTENA | SHCSR_USGFAULTENA;
#[cfg(feature = "armv6m")]
const SHCSR_ENABLE_MASK: u32 = 0;
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
const SHCSR_STATUS_MASK: u32 = SHCSR_USGFAULTPENDED
    | SHCSR_MEMFAULTPENDED
    | SHCSR_BUSFAULTPENDED
    | SHCSR_SVCALLPENDED
    | SHCSR_SYSTICKACT
    | SHCSR_PENDSVACT
    | SHCSR_MONITORACT
    | SHCSR_SVCALLACT
    | SHCSR_USGFAULTACT
    | SHCSR_BUSFAULTACT
    | SHCSR_MEMFAULTACT;
#[cfg(feature = "armv6m")]
const SHCSR_STATUS_MASK: u32 = SHCSR_SVCALLPENDED;

// Configurable Fault Status Register bit positions
const CFSR_IACCVIOL: u32 = 1 << 0;
const CFSR_DACCVIOL: u32 = 1 << 1;
const CFSR_MSTKERR: u32 = 1 << 4;
const CFSR_MMARVALID: u32 = 1 << 7;
const CFSR_IBUSERR: u32 = 1 << 8;
const CFSR_PRECISERR: u32 = 1 << 9;
const CFSR_UNSTKERR: u32 = 1 << 11;
const CFSR_STKERR: u32 = 1 << 12;
const CFSR_BFARVALID: u32 = 1 << 15;
const CFSR_UNDEFINSTR: u32 = 1 << 16;
const CFSR_INVSTATE: u32 = 1 << 17;
const CFSR_INVPC: u32 = 1 << 18;

// HardFault Status Register bit positions
const HFSR_VECTTBL: u32 = 1 << 1;
const HFSR_FORCED: u32 = 1 << 30;
const HFSR_WRITE_ONE_TO_CLEAR_MASK: u32 = (1 << 1) | HFSR_FORCED | (1 << 31);

impl Processor {
    pub(crate) fn read_shcsr(&self) -> u32 {
        (self.shcsr & !SHCSR_STATUS_MASK) | self.shcsr_status_bits()
    }

    fn shcsr_status_bits(&self) -> u32 {
        #[cfg(feature = "armv6m")]
        {
            if self.exception_pending(Exception::SVCall) {
                SHCSR_SVCALLPENDED
            } else {
                0
            }
        }

        #[cfg(any(feature = "armv7m", feature = "armv7em"))]
        {
            let mut bits = 0;

            if self.exception_pending(Exception::UsageFault) {
                bits |= SHCSR_USGFAULTPENDED;
            }
            if self.exception_pending(Exception::MemoryManagementFault) {
                bits |= SHCSR_MEMFAULTPENDED;
            }
            if self.exception_pending(Exception::BusFault) {
                bits |= SHCSR_BUSFAULTPENDED;
            }
            if self.exception_pending(Exception::SVCall) {
                bits |= SHCSR_SVCALLPENDED;
            }

            if self.exception_active(Exception::SysTick) {
                bits |= SHCSR_SYSTICKACT;
            }
            if self.exception_active(Exception::PendSV) {
                bits |= SHCSR_PENDSVACT;
            }
            if self.exception_active(Exception::DebugMonitor) {
                bits |= SHCSR_MONITORACT;
            }
            if self.exception_active(Exception::SVCall) {
                bits |= SHCSR_SVCALLACT;
            }
            if self.exception_active(Exception::UsageFault) {
                bits |= SHCSR_USGFAULTACT;
            }
            if self.exception_active(Exception::BusFault) {
                bits |= SHCSR_BUSFAULTACT;
            }
            if self.exception_active(Exception::MemoryManagementFault) {
                bits |= SHCSR_MEMFAULTACT;
            }

            bits
        }
    }

    #[cfg(not(feature = "armv6m"))]
    pub(crate) fn configurable_fault_enabled(&self, exception: Exception) -> bool {
        let enable_bit = match exception {
            Exception::MemoryManagementFault => SHCSR_MEMFAULTENA,
            Exception::BusFault => SHCSR_BUSFAULTENA,
            Exception::UsageFault => SHCSR_USGFAULTENA,
            _ => return true,
        };

        (self.shcsr & enable_bit) != 0
    }

    #[cfg(feature = "has-fp")]
    pub(crate) fn reset_fp_system_state(&mut self) {
        self.cpacr = 0;
        self.fpccr = (1 << FPCCR_ASPEN) | (1 << FPCCR_LSPEN);
        self.fpcar = 0;
        self.fpdscr = 0;
        self.mvfr0 = FP_MVFR0_RESET;
        self.mvfr1 = FP_MVFR1_RESET;
        self.mvfr2 = FP_MVFR2_RESET;
    }

    #[cfg(not(feature = "has-fp"))]
    pub(crate) fn reset_fp_system_state(&mut self) {
        let _ = self;
    }

    pub(crate) fn reset_scb_fault_state(&mut self) {
        self.shcsr = 0;
        self.cfsr = 0;
        self.dfsr = 0;
        self.hfsr = 0;
        self.demcr = 0;
        self.mmfar = 0;
        self.bfar = 0;
        self.afsr = 0;
    }

    pub(crate) fn set_shcsr_exception_active(&mut self, exception: Exception, active: bool) {
        let bit = match exception {
            Exception::MemoryManagementFault => SHCSR_MEMFAULTACT,
            Exception::BusFault => SHCSR_BUSFAULTACT,
            Exception::UsageFault => SHCSR_USGFAULTACT,
            _ => return,
        };

        if active {
            self.shcsr |= bit;
        } else {
            self.shcsr &= !bit;
        }
    }

    fn latch_mmfar(&mut self, address: u32) {
        self.mmfar = address;
        self.cfsr |= CFSR_MMARVALID;
    }

    fn latch_bfar(&mut self, address: u32) {
        self.bfar = address;
        self.cfsr |= CFSR_BFARVALID;
    }

    pub(crate) fn set_hfsr_forced(&mut self) {
        self.hfsr |= HFSR_FORCED;
    }

    #[cfg(feature = "has-fp")]
    pub(crate) fn write_cpacr(&mut self, value: u32) -> Result<(), Fault> {
        if !self.current_mode_is_privileged() {
            return Err(Fault::DAccViol);
        }

        self.cpacr = value & CPACR_CP10_CP11_MASK;
        Ok(())
    }

    #[cfg(feature = "has-fp")]
    pub(crate) fn write_fpccr(&mut self, value: u32) -> Result<(), Fault> {
        if !self.current_mode_is_privileged() {
            return Err(Fault::DAccViol);
        }

        self.fpccr = (self.fpccr & !FPCCR_WRITABLE_MASK) | (value & FPCCR_WRITABLE_MASK);
        Ok(())
    }

    #[cfg(feature = "has-fp")]
    pub(crate) fn write_fpcar(&mut self, value: u32) -> Result<(), Fault> {
        if !self.current_mode_is_privileged() {
            return Err(Fault::DAccViol);
        }

        self.fpcar = value & !0x7;
        Ok(())
    }

    #[cfg(feature = "has-fp")]
    pub(crate) fn write_fpdscr(&mut self, value: u32) -> Result<(), Fault> {
        if !self.current_mode_is_privileged() {
            return Err(Fault::DAccViol);
        }

        self.fpdscr = value & FPDSCR_WRITABLE_MASK;
        Ok(())
    }

    pub(crate) fn record_fault_status(&mut self, fault: Fault, status: FaultStatusContext) {
        match fault {
            Fault::IAccViol => {
                self.cfsr |= CFSR_IACCVIOL;
                if let Some(address) = status.fault_address {
                    self.latch_mmfar(address);
                }
            }
            Fault::DAccViol => {
                self.cfsr |= CFSR_DACCVIOL;
                if let Some(address) = status.fault_address {
                    self.latch_mmfar(address);
                }
            }
            Fault::Msunskerr => self.cfsr |= CFSR_UNSTKERR,
            Fault::Mstkerr => self.cfsr |= CFSR_MSTKERR,
            Fault::IBusErr => {
                self.cfsr |= CFSR_IBUSERR;
                if let Some(address) = status.fault_address {
                    self.latch_bfar(address);
                }
            }
            Fault::Preciserr => {
                self.cfsr |= CFSR_PRECISERR;
                if let Some(address) = status.fault_address {
                    self.latch_bfar(address);
                }
            }
            Fault::Stkerr => self.cfsr |= CFSR_STKERR,
            Fault::UndefInstr => self.cfsr |= CFSR_UNDEFINSTR,
            Fault::Invstate => self.cfsr |= CFSR_INVSTATE,
            Fault::InvPc => self.cfsr |= CFSR_INVPC,
            Fault::Forced => self.set_hfsr_forced(),
            Fault::VectorTable => self.hfsr |= HFSR_VECTTBL,
            _ => {}
        }
    }

    fn write_shcsr_pending_bit(&mut self, value: u32, bit: usize, exception: Exception) {
        if value.get_bit(bit) {
            self.set_exception_pending(exception);
        } else {
            self.clear_pending_exception(exception);
        }
    }
}

///
/// Register based API to SCB
///
pub trait SystemControlBlock {
    ///
    /// Read Interrupt Control and State Register
    ///
    fn read_icsr(&self) -> u32;

    ///
    /// Write Interrupt Control and State Register
    ///
    fn write_icsr(&mut self, value: u32);

    ///
    /// Write Vector Table Offset
    ///
    fn write_vtor(&mut self, value: u32);

    ///
    /// Write System Handler Priority Register 1
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr1(&mut self, value: u32);

    ///
    /// Write System Handler Priority Register 2
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr2(&mut self, value: u32);

    ///
    /// Write System Handler Priority Register 3
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr3(&mut self, value: u32);

    ///
    /// Write System Handler Priority Register 1, 8-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr1_u8(&mut self, offset: usize, value: u8);

    ///
    /// Write System Handler Priority Register 2, 8-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr2_u8(&mut self, offset: usize, value: u8);

    ///
    /// Write System Handler Priority Register 3, 8-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr3_u8(&mut self, offset: usize, value: u8);

    ///
    /// Write System Handler Priority Register 1, 16-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr1_u16(&mut self, offset: usize, value: u16);

    ///
    /// Write System Handler Priority Register 2, 16-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr2_u16(&mut self, offset: usize, value: u16);

    ///
    /// Write System Handler Priority Register 3, 16-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr3_u16(&mut self, offset: usize, value: u16);

    ///
    /// Read System Handler Priority Register 1
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr1(&self) -> u32;

    ///
    /// Read System Handler Priority Register 2
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr2(&self) -> u32;

    ///
    /// Read System Handler Priority Register 3
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr3(&self) -> u32;

    ///
    /// Read System Handler Priority Register 1, 8-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr1_u8(&self, offset: usize) -> u8;

    ///
    /// Read System Handler Priority Register 2, 8-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr2_u8(&self, offset: usize) -> u8;

    ///
    /// Read System Handler Priority Register 3, 8-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr3_u8(&self, offset: usize) -> u8;

    ///
    /// Read System Handler Priority Register 1, 16-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr1_u16(&self, offset: usize) -> u16;

    ///
    /// Read System Handler Priority Register 2, 16-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr2_u16(&self, offset: usize) -> u16;

    ///
    /// Read System Handler Priority Register 3, 16-bit access
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr3_u16(&self, offset: usize) -> u16;

    ///
    /// Write System Control Register
    ///
    fn write_scr(&mut self, value: u32);

    /// Write System Handler Control and State Register.
    fn write_shcsr(&mut self, value: u32);

    /// Read System Handler Control and State Register.
    fn read_shcsr(&self) -> u32;

    /// Write Configurable Fault Status Register.
    fn write_cfsr(&mut self, value: u32);

    /// Write `HardFault Status Register`.
    fn write_hfsr(&mut self, value: u32);

    ///
    /// Write Debug Exception and Monitor Control Register
    ///
    fn write_demcr(&mut self, value: u32);

    ///
    /// Read Debug Exception and Monitor Control Register
    ///
    fn read_demcr(&self) -> u32;

    ///
    /// Read Vector Table Offset
    ///
    fn read_vtor(&self) -> u32;

    ///
    /// Read System Control Register
    ///
    fn read_scr(&self) -> u32;

    ///
    /// Write "Software Triggered Interrupt Register"
    ///
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_stir(&mut self, value: u32);
}

impl SystemControlBlock for Processor {
    fn read_icsr(&self) -> u32 {
        let mut value: u32 = 0;

        value.set_bits(0..9, self.psr.get_isr_number() as u32);

        if let Some(exception) = self.get_pending_exception() {
            value.set_bits(12..21, usize::from(exception) as u32);
        }

        value
    }

    fn write_icsr(&mut self, value: u32) {
        if value.get_bit(31) {
            self.set_exception_pending(Exception::NMI);
        }
        if value.get_bit(28) {
            self.set_exception_pending(Exception::PendSV);
        } else if value.get_bit(27) {
            self.clear_pending_exception(Exception::PendSV);
        }
        if value.get_bit(26) {
            self.set_exception_pending(Exception::SysTick);
        } else if value.get_bit(25) {
            self.clear_pending_exception(Exception::SysTick);
        }
    }

    fn write_vtor(&mut self, value: u32) {
        self.vtor = value;
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr1(&mut self, value: u32) {
        self.write_shpr1_u8(0, value.get_bits(0..8) as u8);
        self.write_shpr1_u8(1, value.get_bits(8..16) as u8);
        self.write_shpr1_u8(2, value.get_bits(16..24) as u8);
        self.write_shpr1_u8(3, value.get_bits(24..32) as u8);
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr2(&mut self, value: u32) {
        self.write_shpr2_u8(0, value.get_bits(0..8) as u8);
        self.write_shpr2_u8(1, value.get_bits(8..16) as u8);
        self.write_shpr2_u8(2, value.get_bits(16..24) as u8);
        self.write_shpr2_u8(3, value.get_bits(24..32) as u8);
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr3(&mut self, value: u32) {
        self.write_shpr3_u8(0, value.get_bits(0..8) as u8);
        self.write_shpr3_u8(1, value.get_bits(8..16) as u8);
        self.write_shpr3_u8(2, value.get_bits(16..24) as u8);
        self.write_shpr3_u8(3, value.get_bits(24..32) as u8);
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr1_u16(&mut self, offset: usize, value: u16) {
        match offset {
            0 | 1 => {
                let offset_base = offset * 2;
                self.write_shpr1_u8(offset_base, value.get_bits(0..8) as u8);
                self.write_shpr1_u8(offset_base + 1, value.get_bits(8..16) as u8);
            }
            _ => (),
        }
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr2_u16(&mut self, offset: usize, value: u16) {
        match offset {
            0 | 1 => {
                let offset_base = offset * 2;
                self.write_shpr2_u8(offset_base, value.get_bits(0..8) as u8);
                self.write_shpr2_u8(offset_base + 1, value.get_bits(8..16) as u8);
            }
            _ => (),
        }
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr3_u16(&mut self, offset: usize, value: u16) {
        match offset {
            0 | 1 => {
                let offset_base = offset * 2;
                self.write_shpr3_u8(offset_base, value.get_bits(0..8) as u8);
                self.write_shpr3_u8(offset_base + 1, value.get_bits(8..16) as u8);
            }
            _ => (),
        }
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr1_u8(&mut self, offset: usize, value: u8) {
        match offset {
            0 => self.set_exception_priority(Exception::MemoryManagementFault, value),
            1 => self.set_exception_priority(Exception::BusFault, value),
            2 => self.set_exception_priority(Exception::UsageFault, value),
            _ => (),
        }
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr2_u8(&mut self, offset: usize, value: u8) {
        if 3 == offset {
            self.set_exception_priority(Exception::SVCall, value);
        }
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_shpr3_u8(&mut self, offset: usize, value: u8) {
        match offset {
            0 => self.set_exception_priority(Exception::DebugMonitor, value),
            2 => self.set_exception_priority(Exception::PendSV, value),
            3 => self.set_exception_priority(Exception::SysTick, value),
            _ => (),
        }
    }

    fn write_scr(&mut self, value: u32) {
        self.scr = value;
    }

    fn write_shcsr(&mut self, value: u32) {
        self.shcsr = (self.shcsr & !SHCSR_ENABLE_MASK) | (value & SHCSR_ENABLE_MASK);

        #[cfg(any(feature = "armv7m", feature = "armv7em"))]
        {
            self.write_shcsr_pending_bit(value, 12, Exception::UsageFault);
            self.write_shcsr_pending_bit(value, 13, Exception::MemoryManagementFault);
            self.write_shcsr_pending_bit(value, 14, Exception::BusFault);
        }

        self.write_shcsr_pending_bit(value, 15, Exception::SVCall);
    }

    fn read_shcsr(&self) -> u32 {
        Processor::read_shcsr(self)
    }

    fn write_cfsr(&mut self, value: u32) {
        self.cfsr &= !value;
    }

    fn write_hfsr(&mut self, value: u32) {
        self.hfsr &= !(value & HFSR_WRITE_ONE_TO_CLEAR_MASK);
    }

    fn write_demcr(&mut self, value: u32) {
        self.demcr = value;
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr1(&self) -> u32 {
        (u32::from(self.read_shpr1_u8(3)) << 24)
            + (u32::from(self.read_shpr1_u8(2)) << 16)
            + (u32::from(self.read_shpr1_u8(1)) << 8)
            + u32::from(self.read_shpr1_u8(0))
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr2(&self) -> u32 {
        (u32::from(self.read_shpr2_u8(3)) << 24)
            + (u32::from(self.read_shpr2_u8(2)) << 16)
            + (u32::from(self.read_shpr2_u8(1)) << 8)
            + u32::from(self.read_shpr2_u8(0))
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr3(&self) -> u32 {
        (u32::from(self.read_shpr3_u8(3)) << 24)
            + (u32::from(self.read_shpr3_u8(2)) << 16)
            + (u32::from(self.read_shpr3_u8(1)) << 8)
            + u32::from(self.read_shpr3_u8(0))
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr1_u8(&self, offset: usize) -> u8 {
        match offset {
            0 => self.get_exception_priority(Exception::MemoryManagementFault) as u8,
            1 => self.get_exception_priority(Exception::BusFault) as u8,
            2 => self.get_exception_priority(Exception::UsageFault) as u8,
            _ => 0,
        }
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr2_u8(&self, offset: usize) -> u8 {
        match offset {
            3 => self.get_exception_priority(Exception::SVCall) as u8,
            _ => 0,
        }
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr3_u8(&self, offset: usize) -> u8 {
        match offset {
            0 => self.get_exception_priority(Exception::DebugMonitor) as u8,
            2 => self.get_exception_priority(Exception::PendSV) as u8,
            3 => self.get_exception_priority(Exception::SysTick) as u8,
            _ => 0,
        }
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr1_u16(&self, offset: usize) -> u16 {
        match offset {
            0 | 1 => {
                (u16::from(self.read_shpr1_u8((offset * 2) + 1)) << 8)
                    + u16::from(self.read_shpr1_u8(offset * 2))
            }
            _ => 0,
        }
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr2_u16(&self, offset: usize) -> u16 {
        match offset {
            0 | 1 => {
                (u16::from(self.read_shpr2_u8((offset * 2) + 1)) << 8)
                    + u16::from(self.read_shpr2_u8(offset * 2))
            }
            _ => 0,
        }
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn read_shpr3_u16(&self, offset: usize) -> u16 {
        match offset {
            0 | 1 => {
                (u16::from(self.read_shpr3_u8((offset * 2) + 1)) << 8)
                    + u16::from(self.read_shpr3_u8(offset * 2))
            }
            _ => 0,
        }
    }

    fn read_scr(&self) -> u32 {
        0
    }
    fn read_vtor(&self) -> u32 {
        self.vtor
    }

    fn read_demcr(&self) -> u32 {
        self.demcr
    }

    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn write_stir(&mut self, value: u32) {
        self.set_exception_pending(Exception::Interrupt {
            n: value.get_bits(0..9) as usize,
        });
    }
}

#[cfg(test)]
#[cfg(any(feature = "armv7m", feature = "armv7em"))]
mod tests {
    use super::*;
    use crate::bus::Bus;
    use crate::core::exception::Exception;
    use crate::core::exception::ExceptionHandling;
    use crate::core::fault::FaultTrapMode;
    use crate::core::register::{BaseReg, Epsr, Reg};
    use crate::core::reset::Reset;
    use crate::executor::Executor;

    const SHCSR_MEMFAULTENA: u32 = 1 << 16;
    const SHCSR_BUSFAULTENA: u32 = 1 << 17;
    const SHCSR_USGFAULTENA: u32 = 1 << 18;
    const SHCSR_SVCALLPENDED: u32 = 1 << 15;
    const SHCSR_BUSFAULTPENDED: u32 = 1 << 14;
    const SHCSR_MEMFAULTPENDED: u32 = 1 << 13;
    const SHCSR_USGFAULTPENDED: u32 = 1 << 12;
    const SHCSR_SYSTICKACT: u32 = 1 << 11;
    const SHCSR_PENDSVACT: u32 = 1 << 10;
    const SHCSR_MONITORACT: u32 = 1 << 8;
    const SHCSR_SVCALLACT: u32 = 1 << 7;
    const SHCSR_PRESERVED_RAW_BITS: u32 = 1 << 2;

    const HFSR_VECTTBL: u32 = 1 << 1;
    const HFSR_FORCED: u32 = 1 << 30;
    const HFSR_DEBUGEVT: u32 = 1 << 31;

    fn reset_test_image() -> Box<[u8]> {
        vec![
            0x00, 0x01, 0x00, 0x20, // initial SP = 0x2000_0100
            0x09, 0x00, 0x00, 0x00, // reset vector = 0x0000_0009
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ]
        .into_boxed_slice()
    }

    #[test]
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn test_shpr_read_write_32() {
        // Arrange
        let mut processor = Processor::new();

        // Act
        processor.write_shpr1(0xffee_ccbb);
        processor.write_shpr2(0xaa99_8877);
        processor.write_shpr3(0x6655_4433);

        // Assert
        assert_eq!(
            processor.get_exception_priority(Exception::UsageFault),
            0xee
        );
        assert_eq!(processor.get_exception_priority(Exception::BusFault), 0xcc);
        assert_eq!(
            processor.get_exception_priority(Exception::MemoryManagementFault),
            0xbb
        );
        assert_eq!(processor.read_shpr1(), 0x00ee_ccbb);

        assert_eq!(processor.get_exception_priority(Exception::SVCall), 0xaa);

        assert_eq!(processor.read_shpr2(), 0xaa00_0000);

        assert_eq!(processor.get_exception_priority(Exception::SysTick), 0x66);
        assert_eq!(processor.get_exception_priority(Exception::PendSV), 0x55);
        assert_eq!(
            processor.get_exception_priority(Exception::DebugMonitor),
            0x33
        );

        assert_eq!(processor.read_shpr3(), 0x6655_0033);
    }

    #[test]
    #[cfg(any(feature = "armv7m", feature = "armv7em"))]
    fn test_shpr_read_write_16() {
        // Arrange
        let mut processor = Processor::new();

        // Act
        processor.write_shpr1_u16(0, 0xccbb);
        processor.write_shpr1_u16(1, 0xffee);

        processor.write_shpr2_u16(0, 0x8877);
        processor.write_shpr2_u16(1, 0xaa99);

        processor.write_shpr3_u16(0, 0x4433);
        processor.write_shpr3_u16(1, 0x6655);

        // Assert
        assert_eq!(
            processor.get_exception_priority(Exception::UsageFault),
            0xee
        );
        assert_eq!(processor.get_exception_priority(Exception::BusFault), 0xcc);
        assert_eq!(
            processor.get_exception_priority(Exception::MemoryManagementFault),
            0xbb
        );
        assert_eq!(processor.read_shpr1(), 0x00ee_ccbb);

        assert_eq!(processor.read_shpr1_u16(0), 0xccbb);
        assert_eq!(processor.read_shpr1_u16(1), 0x00ee);

        assert_eq!(processor.get_exception_priority(Exception::SVCall), 0xaa);

        assert_eq!(processor.read_shpr2(), 0xaa00_0000);
        assert_eq!(processor.read_shpr2_u16(0), 0x0000);
        assert_eq!(processor.read_shpr2_u16(1), 0xaa00);

        assert_eq!(processor.get_exception_priority(Exception::SysTick), 0x66);
        assert_eq!(processor.get_exception_priority(Exception::PendSV), 0x55);
        assert_eq!(
            processor.get_exception_priority(Exception::DebugMonitor),
            0x33
        );

        assert_eq!(processor.read_shpr3(), 0x6655_0033);
        assert_eq!(processor.read_shpr3_u16(0), 0x0033);
        assert_eq!(processor.read_shpr3_u16(1), 0x6655);
    }

    #[test]
    fn test_shcsr_enable_bits_are_writable_via_bus32() {
        let mut processor = Processor::new();

        processor.shcsr = SHCSR_PRESERVED_RAW_BITS;

        processor
            .write32(
                0xE000_ED24,
                SHCSR_MEMFAULTENA | SHCSR_BUSFAULTENA | SHCSR_USGFAULTENA,
            )
            .unwrap();

        assert_eq!(
            processor.read32(0xE000_ED24).unwrap(),
            SHCSR_PRESERVED_RAW_BITS | SHCSR_MEMFAULTENA | SHCSR_BUSFAULTENA | SHCSR_USGFAULTENA
        );
    }

    #[test]
    fn test_shcsr_active_fault_bits_reflect_live_exception_state() {
        for (exception, expected_bit) in [
            (Exception::MemoryManagementFault, SHCSR_MEMFAULTACT),
            (Exception::BusFault, SHCSR_BUSFAULTACT),
            (Exception::UsageFault, SHCSR_USGFAULTACT),
        ] {
            let mut processor = Processor::new();

            processor.set_msp(0x2000_0100);
            processor.set_pc(0x1004);
            processor.exception_entry(exception, 0x1004).unwrap();

            assert_eq!(
                processor.read32(0xE000_ED24).unwrap() & expected_bit,
                expected_bit
            );
        }
    }

    #[test]
    fn test_shcsr_pending_bits_reflect_live_exception_state() {
        for (exception, expected_bit) in [
            (Exception::UsageFault, SHCSR_USGFAULTPENDED),
            (Exception::MemoryManagementFault, SHCSR_MEMFAULTPENDED),
            (Exception::BusFault, SHCSR_BUSFAULTPENDED),
            (Exception::SVCall, SHCSR_SVCALLPENDED),
        ] {
            let mut processor = Processor::new();

            processor.set_exception_pending(exception);

            assert_eq!(
                processor.read32(0xE000_ED24).unwrap() & expected_bit,
                expected_bit
            );

            processor.clear_pending_exception(exception);

            assert_eq!(processor.read32(0xE000_ED24).unwrap() & expected_bit, 0);
        }
    }

    #[test]
    fn test_shcsr_write_sets_and_clears_pending_bits() {
        for (exception, pending_bit) in [
            (Exception::UsageFault, SHCSR_USGFAULTPENDED),
            (Exception::MemoryManagementFault, SHCSR_MEMFAULTPENDED),
            (Exception::BusFault, SHCSR_BUSFAULTPENDED),
            (Exception::SVCall, SHCSR_SVCALLPENDED),
        ] {
            let mut processor = Processor::new();

            processor.write32(0xE000_ED24, pending_bit).unwrap();

            assert!(processor.exception_pending(exception));
            assert_eq!(
                processor.read32(0xE000_ED24).unwrap() & pending_bit,
                pending_bit
            );

            processor.write32(0xE000_ED24, 0).unwrap();

            assert!(!processor.exception_pending(exception));
            assert_eq!(processor.read32(0xE000_ED24).unwrap() & pending_bit, 0);
        }
    }

    #[test]
    fn test_shcsr_active_system_handler_bits_reflect_live_exception_state() {
        for (exception, expected_bit) in [
            (Exception::SysTick, SHCSR_SYSTICKACT),
            (Exception::PendSV, SHCSR_PENDSVACT),
            (Exception::DebugMonitor, SHCSR_MONITORACT),
            (Exception::SVCall, SHCSR_SVCALLACT),
        ] {
            let mut processor = Processor::new();

            let mut image = vec![0; 0x100].into_boxed_slice();
            let vector_offset = usize::from(exception) * 4;
            image[vector_offset..vector_offset + 4].copy_from_slice(&0x0000_0041_u32.to_le_bytes());

            processor.flash_memory(image.len(), &image);
            processor.set_msp(0x2000_0100);
            processor.set_pc(0x1004);
            processor.psr.set_t(true);

            processor.exception_entry(exception, 0x1004).unwrap();

            assert_eq!(
                processor.read32(0xE000_ED24).unwrap() & expected_bit,
                expected_bit
            );

            processor.exception_return(0xFFFF_FFF9).unwrap();

            assert_eq!(processor.read32(0xE000_ED24).unwrap() & expected_bit, 0);
        }
    }

    #[test]
    fn test_shcsr_active_fault_bits_clear_on_exception_return() {
        let mut processor = Processor::new();

        processor.set_msp(0x2000_0100);
        processor.set_pc(0x1000);
        processor.psr.set_t(true);

        processor
            .exception_entry(Exception::BusFault, 0x1000)
            .unwrap();
        assert_eq!(
            processor.read32(0xE000_ED24).unwrap() & SHCSR_BUSFAULTACT,
            SHCSR_BUSFAULTACT
        );

        processor.exception_return(0xFFFF_FFF9).unwrap();

        assert_eq!(
            processor.read32(0xE000_ED24).unwrap() & SHCSR_BUSFAULTACT,
            0
        );
    }

    #[test]
    fn test_shcsr_usagefault_active_bit_is_visible_to_handler_code() {
        let mut image = vec![0; 0x100].into_boxed_slice();

        image[0..4].copy_from_slice(&0x2000_0100_u32.to_le_bytes());
        image[4..8].copy_from_slice(&0x0000_0041_u32.to_le_bytes());
        image[24..28].copy_from_slice(&0x0000_0081_u32.to_le_bytes());

        image[0x40..0x42].copy_from_slice(&0xde00_u16.to_le_bytes());

        image[0x80..0x82].copy_from_slice(&0x4801_u16.to_le_bytes());
        image[0x82..0x84].copy_from_slice(&0x6801_u16.to_le_bytes());
        image[0x84..0x86].copy_from_slice(&0xbf00_u16.to_le_bytes());
        image[0x88..0x8c].copy_from_slice(&0xE000_ED24_u32.to_le_bytes());

        let mut processor = Processor::new();
        processor.fault_trap_mode(FaultTrapMode::none());
        processor.flash_memory(image.len(), &image);
        processor.reset().unwrap();

        processor.write32(0xE000_ED24, SHCSR_USGFAULTENA).unwrap();

        processor.step();

        assert!(processor.exception_active(Exception::UsageFault));
        assert_eq!(
            processor.read32(0xE000_ED24).unwrap() & SHCSR_USGFAULTACT,
            SHCSR_USGFAULTACT
        );

        processor.step();
        processor.step();

        assert_eq!(processor.get_r(Reg::R0), 0xE000_ED24);
        assert_eq!(
            processor.get_r(Reg::R1) & SHCSR_USGFAULTACT,
            SHCSR_USGFAULTACT,
            "handler-side SHCSR read should observe UsageFault active bit"
        );
    }

    #[test]
    fn test_shcsr_memmanage_active_bit_is_visible_to_handler_code() {
        let mut image = vec![0; 0x100].into_boxed_slice();

        image[0..4].copy_from_slice(&0x2000_0100_u32.to_le_bytes());
        image[4..8].copy_from_slice(&0x0000_0041_u32.to_le_bytes());
        image[16..20].copy_from_slice(&0x0000_0081_u32.to_le_bytes());

        image[0x40..0x42].copy_from_slice(&0x4800_u16.to_le_bytes());
        image[0x42..0x44].copy_from_slice(&0x6801_u16.to_le_bytes());
        image[0x44..0x48].copy_from_slice(&0x6000_0000_u32.to_le_bytes());

        image[0x80..0x82].copy_from_slice(&0x4801_u16.to_le_bytes());
        image[0x82..0x84].copy_from_slice(&0x6801_u16.to_le_bytes());
        image[0x84..0x86].copy_from_slice(&0xbf00_u16.to_le_bytes());
        image[0x88..0x8c].copy_from_slice(&0xE000_ED24_u32.to_le_bytes());

        let mut processor = Processor::new();
        processor.fault_trap_mode(FaultTrapMode::none());
        processor.flash_memory(image.len(), &image);
        processor.reset().unwrap();

        processor.write32(0xE000_ED24, SHCSR_MEMFAULTENA).unwrap();

        processor.step();
        processor.step();

        assert!(processor.exception_active(Exception::MemoryManagementFault));
        assert_eq!(
            processor.read32(0xE000_ED24).unwrap() & SHCSR_MEMFAULTACT,
            SHCSR_MEMFAULTACT
        );

        processor.step();
        processor.step();

        assert_eq!(processor.get_r(Reg::R0), 0xE000_ED24);
        assert_eq!(
            processor.get_r(Reg::R1) & SHCSR_MEMFAULTACT,
            SHCSR_MEMFAULTACT,
            "handler-side SHCSR read should observe MemManage active bit"
        );
    }

    #[test]
    fn test_cfsr_write_one_to_clear_via_bus32() {
        let mut processor = Processor::new();

        processor.cfsr = 0x0103_0005;

        processor.write32(0xE000_ED28, 0x0001_0001).unwrap();

        assert_eq!(processor.read32(0xE000_ED28).unwrap(), 0x0102_0004);
    }

    #[test]
    fn test_hfsr_write_one_to_clear_via_bus32() {
        let mut processor = Processor::new();

        processor.hfsr = HFSR_VECTTBL | HFSR_FORCED | HFSR_DEBUGEVT;

        processor
            .write32(0xE000_ED2C, HFSR_VECTTBL | HFSR_DEBUGEVT)
            .unwrap();

        assert_eq!(processor.read32(0xE000_ED2C).unwrap(), HFSR_FORCED);
    }

    #[test]
    fn test_reset_restores_scb_fault_status_boot_defaults() {
        let image = reset_test_image();

        let mut processor = Processor::new();
        processor.flash_memory(image.len(), &image);

        processor.shcsr = SHCSR_MEMFAULTENA | SHCSR_BUSFAULTENA | SHCSR_USGFAULTENA | 0x0b;
        processor.cfsr = 0x0103_9187;
        processor.dfsr = 0x1f;
        processor.hfsr = HFSR_VECTTBL | HFSR_FORCED | HFSR_DEBUGEVT;
        processor.mmfar = 0x2000_1234;
        processor.bfar = 0x4000_5678;
        processor.afsr = 0x89ab_cdef;

        processor.reset().unwrap();

        assert_eq!(processor.read32(0xE000_ED24).unwrap(), 0);
        assert_eq!(processor.read32(0xE000_ED28).unwrap(), 0);
        assert_eq!(processor.read32(0xE000_ED2C).unwrap(), 0);
        assert_eq!(processor.read32(0xE000_ED30).unwrap(), 0);
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_reset_restores_fp_system_register_defaults() {
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

        let image = reset_test_image();

        let mut processor = Processor::new();
        processor.flash_memory(image.len(), &image);

        processor.cpacr = 0x00f0_0000;
        processor.fpccr = 0xc000_0039;
        processor.fpcar = 0x2000_0100;
        processor.fpdscr = 0x00ab_0000;
        processor.mvfr0 = 0x1011_0021;
        processor.mvfr1 = 0x1100_0011;
        processor.mvfr2 = 0x0000_0040;

        processor.reset().unwrap();

        assert_eq!(processor.cpacr & 0x00f0_0000, 0);
        assert_eq!(
            processor.fpccr & ((1 << FPCCR_ASPEN) | (1 << FPCCR_LSPEN)),
            0xc000_0000
        );
        assert!(!processor.fpccr.get_bit(FPCCR_LSPACT));
        assert_eq!(processor.fpdscr, 0);
        assert_eq!(processor.mvfr0, EXPECTED_MVFR0);
        assert_eq!(processor.mvfr1, EXPECTED_MVFR1);
        assert_eq!(processor.mvfr2, EXPECTED_MVFR2);
    }

    #[test]
    fn test_demcr_write_read_via_bus32() {
        let mut processor = Processor::new();

        processor.write32(0xE000_EDFC, 0x1234_0000).unwrap();

        assert_eq!(processor.read32(0xE000_EDFC).unwrap(), 0x1234_0000);
    }
}
