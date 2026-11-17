//!
//! Functionality for representing Cortex Exceptions.
//!
//!

use crate::Processor;
use crate::ProcessorMode;
use crate::bus::Bus;
use crate::core::bits::Bits;
use crate::core::fault::Fault;
#[cfg(feature = "has-fp")]
use crate::core::register::ExtensionRegOperations;
#[cfg(feature = "has-fp")]
use crate::core::register::SingleReg;
use crate::core::register::{BaseReg, Ipsr, Reg};
use crate::core::reset::Reset;
#[cfg(feature = "has-fp")]
use crate::executor::FloatingPointChecks;
use crate::peripheral::nvic::NVIC;
use crate::peripheral::scb::CCR_STKALIGN;
#[cfg(feature = "has-fp")]
use crate::peripheral::scb::{
    DEMCR_MON_EN, FPCCR_BFRDY, FPCCR_HFRDY, FPCCR_LSPACT, FPCCR_LSPEN, FPCCR_MMRDY, FPCCR_MONRDY,
    FPCCR_THREAD, FPCCR_USER, SystemControlBlock,
};

#[derive(Debug, Eq, Ord, PartialEq, PartialOrd, Copy, Clone)]
///
/// Status information for an exception
///
pub struct ExceptionState {
    priority: i16,
    pending: bool,
    active: bool,
    exception_number: usize,
}

impl ExceptionState {
    ///
    /// Create a state information for specific exception with given priority
    ///
    pub fn new(exception: Exception, priority: i16) -> Self {
        Self {
            exception_number: usize::from(exception),
            priority,
            pending: false,
            active: false,
        }
    }
}

impl Processor {
    /// Implements the ARMv7-M `ExecutionPriority()` pseudocode.
    ///
    /// Returns the effective execution priority after applying active
    /// exceptions, BASEPRI / PRIMASK / FAULTMASK, and PRIGROUP.
    ///
    /// This differs from `get_exception_priority()`, which returns a stored
    /// SHPRx/IPR priority for one exception.
    fn calculate_execution_priority_helper(&self, include_primask: bool) -> i16 {
        let mut highestpri: i16 = 256;
        let mut boostedpri: i16 = 256;
        let subgroupshift = self.aircr.get_bits(8..11);
        let mut groupvalue = 2 << subgroupshift;

        for (_, exp) in self.exceptions.iter().filter(|&(_, e)| e.active) {
            if exp.priority < highestpri {
                highestpri = exp.priority;

                if exp.exception_number == Exception::NMI.into() {
                    groupvalue = -2;
                }

                if exp.exception_number == Exception::HardFault.into() {
                    groupvalue = -1;
                }

                let subgroupvalue = highestpri % groupvalue;
                highestpri -= subgroupvalue;
            }
        }
        #[cfg(not(feature = "armv6m"))]
        if self.basepri != 0 {
            boostedpri = i16::from(self.basepri);
            let subgroupvalue = boostedpri % groupvalue;
            boostedpri -= subgroupvalue;
        }

        if include_primask && self.primask {
            boostedpri = 0;
        }

        #[cfg(not(feature = "armv6m"))]
        {
            if self.faultmask {
                boostedpri = -1;
            }
        }

        if boostedpri < highestpri {
            boostedpri
        } else {
            highestpri
        }
    }

    ///
    /// Check if WFI wakeup condition is met (ignoring PRIMASK)
    ///
    pub fn has_wakeup_condition(&self) -> bool {
        // We specifically ignore PRIMASK checks here as WFI should wake up
        // if an interrupt is pending which would be taken if PRIMASK was clear.
        let wakeup_priority = self.calculate_execution_priority_helper(false);

        self.exceptions
            .values()
            .any(|e| e.pending && e.priority < wakeup_priority)
    }

    #[cfg(all(test, not(feature = "armv6m")))]
    pub(crate) fn test_set_active_exception(&mut self, exception: Exception) {
        self.psr.set_isr_number(exception.into());
        self.exceptions
            .get_mut(&usize::from(exception))
            .unwrap()
            .active = true;
        self.mode = ProcessorMode::HandlerMode;
    }
}

///
/// Trait for interacting with exceptions
///
pub trait ExceptionHandling {
    ///
    /// Get the current processor execution priority (EP).
    ///
    /// This is the ARM pseudocode `ExecutionPriority()`, not a single
    /// exception's stored SHPR/IPR priority.
    ///
    fn get_execution_priority(&self) -> i16;

    ///
    /// Set exception pending if it is not already pending
    ///
    fn set_exception_pending(&mut self, exception: Exception);

    ///
    /// Get the currently highest priority pending exception
    ///
    fn get_pending_exception(&self) -> Option<Exception>;

    ///
    /// Clear the pending status of an exception
    ///
    fn clear_pending_exception(&mut self, exception: Exception);

    ///
    /// Check if given exception is currently pending.
    ///
    fn exception_pending(&self, exception: Exception) -> bool;

    ///
    /// Enter an exception.
    ///
    /// Return adress is the address to which the execution should return when this exception is returned from.
    ///
    fn exception_entry(&mut self, exception: Exception, return_address: u32) -> Result<(), Fault>;

    ///
    /// Return from an exception.
    ///
    /// `exc_return` determines the mode to which to return to.
    ///
    /// Exception return happens when processor is in `HandlerMode` and `exc_return` value is loaded to PC using
    /// LDM, POP, LDR, or BX instructions
    ///
    fn exception_return(&mut self, exc_return: u32) -> Result<(), Fault>;

    ///
    /// Check if given exception is currently active
    ///
    fn exception_active(&self, exception: Exception) -> bool;

    ///
    /// Set priority of an exception. Smaller priority number has higher urgency.
    ///
    fn set_exception_priority(&mut self, exception: Exception, priority: u8);

    ///
    /// Get the configured priority of an exception.
    ///
    /// For configurable exceptions this corresponds to the stored SHPRx/IPR
    /// priority, for example BusFault -> `SHPR1.PRI_5`.
    ///
    fn get_exception_priority(&self, exception: Exception) -> i16;

    ///
    /// Clear exceptions to reset state
    ///
    fn exceptions_reset(&mut self);

    ///
    /// Check if any exceptions have happened.
    ///
    fn check_exceptions(&mut self);
}

trait ExceptionHandlingHelpers {
    fn exception_taken(&mut self, exception: Exception) -> Result<(), Fault>;
    fn deactivate(&mut self, returning_exception_number: usize);
    fn invalid_exception_return_fault(&mut self, exc_return: u32) -> Result<(), Fault>;
    fn invalid_exception_return(
        &mut self,
        returning_exception_number: usize,
        exc_return: u32,
    ) -> Result<(), Fault>;
    fn return_address(&self, exception_type: Exception, return_address: u32) -> u32;
    fn push_stack(&mut self, exception_type: Exception, return_address: u32) -> Result<(), Fault>;
    #[cfg(feature = "has-fp")]
    fn update_fpccr(&mut self, frameptr: u32) -> Result<(), Fault>;
    fn pop_stack(&mut self, frameptr: u32, exc_return: u32) -> Result<(), Fault>;
    fn exception_active_bit_count(&self) -> usize;
}

#[derive(PartialEq, Debug, Copy, Clone)]
///
/// List of supported Exceptions
///
/// Interrupts are controlled by NVIC, but still are generally handled like other exceptions (being sorted by priority)
///
pub enum Exception {
    /// Special exception to reset the processor
    Reset,
    /// Highest priority exception (for except the reset) that cannot be ever masked away.
    /// Can be triggered by a peripheral or triggered by software.
    NMI,
    /// Denotes a error during exception processing, or because an exception cannot be
    /// handled by any other exception handling mechanism.
    HardFault,
    /// Memory protection related fault
    MemoryManagementFault,
    /// Memory related fault (bus access error either for instructions or data)
    BusFault,
    /// Instruction execution faults for multiple underlying reasons. Example: undefined instructions.
    UsageFault,
    /// Reserved for future
    Reserved4,
    /// Reserved for future
    Reserved5,
    /// Reserved for future
    Reserved6,
    /// Debugging related exceptions
    DebugMonitor,
    /// Supervisor call exception, used typically for OS supervisor API handling.
    /// SVC instruction triggers `SVCall` exception.
    SVCall,
    /// Reserved for future
    Reserved8,
    /// Reserved for future
    Reserved9,
    /// Request for system level service, used typically for context switching in OS.
    PendSV,
    /// System timer exception, used for generating timer ticks.
    SysTick,
    /// Exception from a peripheral or software triggered interrupt
    Interrupt {
        /// Interrupt number, 0..
        n: usize,
    },
}

impl ExceptionHandlingHelpers for Processor {
    fn exception_taken(&mut self, exception: Exception) -> Result<(), Fault> {
        let vtor = self.vtor;
        let offset: u32 = usize::from(exception) as u32 * 4;
        let start = self.read32(vtor + offset).map_err(Fault::on_vector_read)?;
        self.blx_write_pc(start);

        self.mode = ProcessorMode::HandlerMode;
        self.psr.set_isr_number(exception.into());
        #[cfg(feature = "has-fp")]
        {
            self.control.fpca = false;
        }
        self.control.sp_sel = false;
        self.exceptions.get_mut(&exception.into()).unwrap().active = true;
        self.set_shcsr_exception_active(exception, true);
        self.execution_priority = self.get_execution_priority();

        // ClearExclusiveLocal(;)
        // SetEventRegister();
        // InstructionSynchronizationBarrier();
        Ok(())
    }

    fn deactivate(&mut self, returning_exception_number: usize) {
        let exception = Exception::from(returning_exception_number);

        self.exceptions
            .get_mut(&returning_exception_number)
            .unwrap()
            .active = false;
        self.set_shcsr_exception_active(exception, false);

        #[cfg(not(feature = "armv6m"))]
        {
            if self.psr.get_isr_number() != 0b10 {
                self.faultmask = false;
            }
        }
        self.execution_priority = self.get_execution_priority();
    }

    fn invalid_exception_return_fault(&mut self, exc_return: u32) -> Result<(), Fault> {
        self.set_r(Reg::LR, (0b1111 << 28) + exc_return);
        Err(Fault::InvPc)
    }

    fn invalid_exception_return(
        &mut self,
        returning_exception_number: usize,
        exc_return: u32,
    ) -> Result<(), Fault> {
        self.deactivate(returning_exception_number);
        self.invalid_exception_return_fault(exc_return)
    }

    fn exception_active_bit_count(&self) -> usize {
        self.exceptions
            .iter()
            .filter(|&(_, exp)| exp.active)
            .fold(0, |acc, _| acc + 1)
    }
    fn return_address(&self, exception_type: Exception, return_address: u32) -> u32 {
        match exception_type {
            Exception::NMI
            | Exception::HardFault
            | Exception::MemoryManagementFault
            | Exception::BusFault
            | Exception::SVCall
            | Exception::DebugMonitor
            | Exception::PendSV
            | Exception::SysTick
            | Exception::Interrupt { .. } => return_address,
            Exception::UsageFault => return_address - 4,
            _ => unreachable!("return address requested for unsupported exception type"),
        }
    }

    fn push_stack(&mut self, exception_type: Exception, return_address: u32) -> Result<(), Fault> {
        #[cfg(feature = "has-fp")]
        let (forcealign, frame_size): (bool, u32) = if self.control.fpca {
            (true, 0x68)
        } else {
            (self.ccr.get_bit(CCR_STKALIGN), 0x20)
        };

        #[cfg(not(feature = "has-fp"))]
        let (forcealign, frame_size): (bool, u32) = (self.ccr.get_bit(CCR_STKALIGN), 0x20);

        // forces 8 byte alignment on the stack
        let spmask = (u32::from(forcealign) << 2) ^ 0xFFFF_FFFF;

        let (frameptr, frameptralign) =
            if self.control.sp_sel && self.mode == ProcessorMode::ThreadMode {
                let align = u32::from(self.psp.get_bit(2) & forcealign);
                self.set_psp((self.psp.wrapping_sub(frame_size)) & spmask);
                (self.psp, align)
            } else {
                let align = u32::from(self.msp.get_bit(2));
                self.set_msp((self.msp.wrapping_sub(frame_size)) & spmask);
                (self.msp, align)
            };

        let r0 = self.get_r(Reg::R0);
        let r1 = self.get_r(Reg::R1);
        let r2 = self.get_r(Reg::R2);
        let r3 = self.get_r(Reg::R3);
        let r12 = self.get_r(Reg::R12);
        let lr = self.get_r(Reg::LR);

        let ret_addr = self.return_address(exception_type, return_address);

        self.write32(frameptr, r0)?;
        self.write32(frameptr.wrapping_add(0x4), r1)?;
        self.write32(frameptr.wrapping_add(0x8), r2)?;
        self.write32(frameptr.wrapping_add(0xc), r3)?;
        self.write32(frameptr.wrapping_add(0x10), r12)?;
        self.write32(frameptr.wrapping_add(0x14), lr)?;
        self.write32(frameptr.wrapping_add(0x18), ret_addr)?;
        let xpsr =
            (self.psr.value & 0b1111_1111_1111_1111_1111_1101_1111_1111) | frameptralign << 9;
        self.write32(frameptr.wrapping_add(0x1c), xpsr)?;

        #[cfg(feature = "has-fp")]
        if self.control.fpca {
            if !self.fpccr.get_bit(FPCCR_LSPEN) {
                self.check_vfp_enabled()?;
                for i in 0..16 {
                    let reg = SingleReg::from(i as u8);
                    let value = self.get_sr(reg);
                    self.write32(frameptr.wrapping_add(0x20 + i * 4), value)?;
                }
                // write FPSCR:
                self.write32(frameptr.wrapping_add(0x60), self.fpscr)?;
            } else {
                self.update_fpccr(frameptr)?;
            }
        }

        #[cfg(feature = "has-fp")]
        {
            let fpca = u32::from(!self.control.fpca) << 4;
            if self.mode == ProcessorMode::HandlerMode {
                self.lr = 0b1111_1111_1111_1111_1111_1111_1110_0001 + fpca;
            } else {
                let thread = 1 << 3;
                let spsel = u32::from(self.control.sp_sel) << 2;
                self.lr = 0b1111_1111_1111_1111_1111_1111_1110_0001 + fpca + thread + spsel;
            }
        }
        #[cfg(not(feature = "has-fp"))]
        {
            if self.mode == ProcessorMode::HandlerMode {
                self.lr = 0xFFFF_FFF1;
            } else if self.control.sp_sel {
                self.lr = 0xFFFF_FFFD;
            } else {
                self.lr = 0xFFFF_FFF9;
            }
        }
        Ok(())
    }

    #[cfg(feature = "has-fp")]
    fn update_fpccr(&mut self, frameptr: u32) -> Result<(), Fault> {
        if !(self.control.fpca && self.fpccr.get_bit(FPCCR_LSPEN)) {
            return Ok(());
        }

        let addr = frameptr.wrapping_add(0x20);
        self.fpcar = addr & !0x7;
        self.fpccr.set_bit(FPCCR_LSPACT, true);

        if self.current_mode_is_privileged() {
            self.fpccr.set_bit(FPCCR_USER, false);
        } else {
            self.fpccr.set_bit(FPCCR_USER, true);
        }

        if self.mode == ProcessorMode::ThreadMode {
            self.fpccr.set_bit(FPCCR_THREAD, true);
        } else {
            self.fpccr.set_bit(FPCCR_THREAD, false);
        }

        if self.execution_priority > -1 {
            self.fpccr.set_bit(FPCCR_HFRDY, true);
        } else {
            self.fpccr.set_bit(FPCCR_HFRDY, false);
        }

        if self.configurable_fault_enabled(Exception::BusFault)
            && self.execution_priority > self.get_exception_priority(Exception::BusFault)
        {
            self.fpccr.set_bit(FPCCR_BFRDY, true);
        } else {
            self.fpccr.set_bit(FPCCR_BFRDY, false);
        }

        if self.configurable_fault_enabled(Exception::MemoryManagementFault)
            && self.execution_priority
                > self.get_exception_priority(Exception::MemoryManagementFault)
        {
            self.fpccr.set_bit(FPCCR_MMRDY, true);
        } else {
            self.fpccr.set_bit(FPCCR_MMRDY, false);
        }

        if self.read_demcr().get_bit(DEMCR_MON_EN)
            && self.execution_priority > self.get_exception_priority(Exception::DebugMonitor)
        {
            self.fpccr.set_bit(FPCCR_MONRDY, true);
        } else {
            self.fpccr.set_bit(FPCCR_MONRDY, false);
        }

        Ok(())
    }

    fn pop_stack(&mut self, frameptr: u32, exc_return: u32) -> Result<(), Fault> {
        #[cfg(feature = "has-fp")]
        let (frame_size, forcealign): (u32, bool) = if !exc_return.get_bit(4) {
            (0x68, true)
        } else {
            (0x20, self.ccr.get_bit(CCR_STKALIGN))
        };

        #[cfg(not(feature = "has-fp"))]
        let (frame_size, forcealign): (u32, bool) = (0x20, self.ccr.get_bit(CCR_STKALIGN));

        let r0 = self.read32(frameptr)?;
        let r1 = self.read32(frameptr.wrapping_add(0x4))?;
        let r2 = self.read32(frameptr.wrapping_add(0x8))?;
        let r3 = self.read32(frameptr.wrapping_add(0xc))?;
        let r12 = self.read32(frameptr.wrapping_add(0x10))?;
        let lr = self.read32(frameptr.wrapping_add(0x14))?;
        let pc = self.read32(frameptr.wrapping_add(0x18))?;
        let psr = self.read32(frameptr.wrapping_add(0x1c))?;

        #[cfg(feature = "has-fp")]
        {
            if !exc_return.get_bit(4) {
                if self.fpccr.get_bit(FPCCR_LSPACT) {
                    self.fpccr.set_bit(FPCCR_LSPACT, false);
                } else {
                    self.check_vfp_enabled()?;
                    for i in 0..16 {
                        let value = self.read32(frameptr.wrapping_add(0x20 + i * 4))?;
                        let reg = SingleReg::from(i as u8);
                        self.set_sr(reg, value);
                    }
                    self.fpscr = self.read32(frameptr.wrapping_add(0x60))?;
                }
            }
            self.control.fpca = !exc_return.get_bit(4);
        }

        let stacked_exception_number = psr.get_bits(0..9) as usize;

        if !psr.get_bit(24) {
            return Err(Fault::Invstate);
        }

        match exc_return.get_bits(0..4) {
            0b0001 => {
                if stacked_exception_number == 0 {
                    return self.invalid_exception_return_fault(exc_return);
                }
            }
            0b1001 | 0b1101 => {
                if stacked_exception_number != 0 {
                    return self.invalid_exception_return_fault(exc_return);
                }
            }
            _ => {
                return self.invalid_exception_return_fault(exc_return);
            }
        }

        self.set_r(Reg::R0, r0);
        self.set_r(Reg::R1, r1);
        self.set_r(Reg::R2, r2);
        self.set_r(Reg::R3, r3);
        self.set_r(Reg::R12, r12);
        self.set_r(Reg::LR, lr);

        self.branch_write_pc(pc);

        let spmask = u32::from(psr.get_bit(9) && forcealign) << 2;
        match exc_return.get_bits(0..4) {
            0b0001 | 0b1001 => {
                let msp = self.get_msp();
                self.set_msp((msp.wrapping_add(frame_size)) | spmask);
            }
            0b1101 => {
                let psp = self.get_psp();
                self.set_psp((psp.wrapping_add(frame_size)) | spmask);
            }
            _ => {
                return self.invalid_exception_return_fault(exc_return);
            }
        }
        self.psr.value.set_bits(27..32, psr.get_bits(27..32));
        self.psr.value.set_bits(0..9, psr.get_bits(0..9));
        self.psr.value.set_bits(10..16, psr.get_bits(10..16));
        self.psr.value.set_bits(24..27, psr.get_bits(24..27));
        // GE[3:0] bits (APSR bits 19:16) are DSP-extension-only architectural state.
        // Restore them from the stacked xPSR only when the DSP extension is present.
        #[cfg(feature = "has-dsp-ext")]
        self.psr.value.set_bits(16..20, psr.get_bits(16..20));
        Ok(())
    }
}

impl ExceptionHandling for Processor {
    fn exceptions_reset(&mut self) {
        self.set_shcsr_exception_active(Exception::MemoryManagementFault, false);
        self.set_shcsr_exception_active(Exception::BusFault, false);
        self.set_shcsr_exception_active(Exception::UsageFault, false);

        for exception in self.exceptions.values_mut() {
            exception.pending = false;
            exception.active = false;

            if exception.exception_number > Exception::HardFault.into() {
                exception.priority = 0;
            }
        }
    }
    fn exception_active(&self, exception: Exception) -> bool {
        self.exceptions[&usize::from(exception)].active
    }

    fn exception_pending(&self, exception: Exception) -> bool {
        self.exceptions[&usize::from(exception)].pending
    }

    fn set_exception_priority(&mut self, exception: Exception, priority: u8) {
        self.exceptions.get_mut(&exception.into()).unwrap().priority = i16::from(priority);
    }

    fn get_exception_priority(&self, exception: Exception) -> i16 {
        self.exceptions[&exception.into()].priority
    }

    fn get_execution_priority(&self) -> i16 {
        self.calculate_execution_priority_helper(true)
    }

    fn set_exception_pending(&mut self, exception: Exception) {
        let exp = self.exceptions.get_mut(&exception.into()).unwrap();

        if !exp.pending {
            exp.pending = true;
            self.pending_exception_count += 1;
        }
    }

    fn get_pending_exception(&self) -> Option<Exception> {
        if self.pending_exception_count > 0 {
            let mut selected: Option<ExceptionState> = None;

            for exception in self.exceptions.values() {
                if !exception.pending || exception.priority >= self.execution_priority {
                    continue;
                }

                let replace = match selected {
                    None => true,
                    Some(current) => {
                        exception.priority < current.priority
                            || (exception.priority == current.priority
                                && exception.exception_number < current.exception_number)
                    }
                };

                if replace {
                    selected = Some(*exception);
                }
            }

            return selected.map(|exception| exception.exception_number.into());
        }
        None
    }

    fn clear_pending_exception(&mut self, exception: Exception) {
        let exp = self.exceptions.get_mut(&exception.into()).unwrap();
        if exp.pending {
            exp.pending = false;
            self.pending_exception_count -= 1;
        }
    }

    fn exception_entry(&mut self, exception: Exception, return_address: u32) -> Result<(), Fault> {
        if exception == Exception::Reset {
            self.reset()
        } else {
            if let Exception::Interrupt { n } = exception {
                self.nvic_unpend_interrupt(n);
            }
            self.push_stack(exception, return_address)
                .map_err(Fault::on_exception_entry_stack)?;
            self.exception_taken(exception)
        }
    }

    fn exception_return(&mut self, exc_return: u32) -> Result<(), Fault> {
        assert!(self.mode == ProcessorMode::HandlerMode);

        let returning_exception_number = self.psr.get_isr_number();
        let nested_activation = self.exception_active_bit_count();

        if self.exceptions[&returning_exception_number].active {
            let frameptr;
            match exc_return.get_bits(0..4) {
                0b0001 => {
                    // return to handler
                    if nested_activation == 1 {
                        return self
                            .invalid_exception_return(returning_exception_number, exc_return);
                    }
                    frameptr = self.get_msp();
                    self.mode = ProcessorMode::HandlerMode;
                    self.control.sp_sel = false;
                }
                0b1001 => {
                    // returning to thread using main stack
                    if nested_activation != 1
                    /*&& !self.ccr.nonbasethreadena*/
                    {
                        return self
                            .invalid_exception_return(returning_exception_number, exc_return);
                    } else {
                        frameptr = self.get_msp();
                        self.mode = ProcessorMode::ThreadMode;
                        self.control.sp_sel = false;
                    }
                }
                0b1101 => {
                    // returning to thread using process stack
                    if nested_activation != 1
                    /*&& !self.ccr.nonbasethreadena*/
                    {
                        return self
                            .invalid_exception_return(returning_exception_number, exc_return);
                    } else {
                        frameptr = self.get_psp();
                        self.mode = ProcessorMode::ThreadMode;
                        self.control.sp_sel = true;
                    }
                }
                _ => {
                    return self.invalid_exception_return(returning_exception_number, exc_return);
                }
            }

            self.deactivate(returning_exception_number);
            self.pop_stack(frameptr, exc_return)?;

            if self.mode == ProcessorMode::ThreadMode
                && nested_activation == 1 // deactivate() reduced one
                && self.scr.get_bit(1)
            {
                self.sleeping = true;
            }

            Ok(())
        } else {
            self.invalid_exception_return(returning_exception_number, exc_return)
        }
    }

    #[inline(always)]
    fn check_exceptions(&mut self) {
        if let Some(exception) = self.get_pending_exception() {
            self.sleeping = false;
            self.clear_pending_exception(exception);
            let pc = self.get_pc();
            if let Err(fault) = self.exception_entry(exception, pc) {
                let active_exception = match self.psr.get_isr_number() {
                    0 => None,
                    n => Some(Exception::from(n)),
                };
                self.record_fault_status(fault, crate::core::fault::FaultStatusContext::default());
                self.set_hfsr_forced();
                let trap_reason = if exception == Exception::HardFault
                    || matches!(
                        active_exception,
                        Some(Exception::HardFault | Exception::NMI)
                    ) {
                    crate::core::fault::FaultTrapReason::Lockup
                } else {
                    crate::core::fault::FaultTrapReason::Fault
                };
                self.pending_fault_trap = Some(crate::core::fault::FaultContext {
                    trap_reason,
                    fault,
                    exception: Exception::HardFault,
                    pc,
                    active_exception,
                });
            }
        } else if self.sleeping && self.has_wakeup_condition() {
            self.sleeping = false;
        }
    }
}

impl From<Exception> for usize {
    fn from(value: Exception) -> Self {
        match value {
            Exception::Reset => 1,
            Exception::NMI => 2,
            Exception::HardFault => 3,
            Exception::MemoryManagementFault => 4,
            Exception::BusFault => 5,
            Exception::UsageFault => 6,
            Exception::Reserved4 => 7,
            Exception::Reserved5 => 8,
            Exception::Reserved6 => 9,
            Exception::DebugMonitor => 10,
            Exception::SVCall => 11,
            Exception::Reserved8 => 12,
            Exception::Reserved9 => 13,
            Exception::PendSV => 14,
            Exception::SysTick => 15,
            Exception::Interrupt { n } => 16 + n,
        }
    }
}

impl From<usize> for Exception {
    fn from(value: usize) -> Self {
        match value {
            1 => Self::Reset,
            2 => Self::NMI,
            3 => Self::HardFault,
            4 => Self::MemoryManagementFault,
            5 => Self::BusFault,
            6 => Self::UsageFault,
            7 => Self::Reserved4,
            8 => Self::Reserved5,
            9 => Self::Reserved6,
            10 => Self::DebugMonitor,
            11 => Self::SVCall,
            12 => Self::Reserved8,
            13 => Self::Reserved9,
            14 => Self::PendSV,
            15 => Self::SysTick,
            _ => Self::Interrupt { n: value - 16 },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::Bus;
    use crate::core::exception::Exception;
    use crate::core::exception::ExceptionHandling;
    #[cfg(not(feature = "armv6m"))]
    use crate::core::fault::Fault;
    #[cfg(not(feature = "armv6m"))]
    use crate::core::instruction::Instruction;
    #[cfg(feature = "has-dsp-ext")]
    use crate::core::register::Apsr;
    #[cfg(feature = "has-fp")]
    use crate::core::register::{ExtensionRegOperations, SingleReg};
    #[cfg(not(feature = "armv6m"))]
    use crate::executor::Executor;
    use crate::peripheral::nvic::NVIC;
    #[cfg(feature = "has-fp")]
    use crate::peripheral::scb::{
        DEMCR_MON_EN, FPCCR_ASPEN, FPCCR_BFRDY, FPCCR_HFRDY, FPCCR_LSPACT, FPCCR_LSPEN,
        FPCCR_MMRDY, FPCCR_MONRDY, FPCCR_THREAD, FPCCR_USER, SHCSR_BUSFAULTENA, SHCSR_MEMFAULTENA,
    };

    #[cfg(not(feature = "armv6m"))]
    const CFSR_MSTKERR: u32 = 1 << 4;
    #[cfg(not(feature = "armv6m"))]
    const HFSR_FORCED: u32 = 1 << 30;
    #[cfg(all(not(feature = "armv6m"), feature = "has-fp"))]
    const EXC_RETURN_HANDLER: u32 = 0x11;
    #[cfg(all(not(feature = "armv6m"), not(feature = "has-fp")))]
    const EXC_RETURN_HANDLER: u32 = 0x1;
    #[cfg(all(not(feature = "armv6m"), feature = "has-fp"))]
    const EXC_RETURN_THREAD_MSP: u32 = 0x19;
    #[cfg(all(not(feature = "armv6m"), not(feature = "has-fp")))]
    const EXC_RETURN_THREAD_MSP: u32 = 0x9;

    #[cfg(not(feature = "armv6m"))]
    fn set_active_exception(processor: &mut Processor, exception: Exception) {
        processor.test_set_active_exception(exception);
    }

    #[cfg(feature = "has-fp")]
    fn fp_test_processor() -> Processor {
        let mut core = Processor::new();
        core.cpacr = 0x00f0_0000;
        core
    }

    #[cfg(feature = "has-fp")]
    fn seed_low_fp_registers(core: &mut Processor, base: u32) {
        for i in 0..16 {
            core.set_sr(SingleReg::from(i as u8), base + i);
        }
    }

    #[cfg(feature = "has-fp")]
    fn configure_update_fpccr_ready_context(core: &mut Processor) {
        core.shcsr = SHCSR_BUSFAULTENA | SHCSR_MEMFAULTENA;
        core.demcr = 1 << DEMCR_MON_EN;
        core.set_exception_priority(Exception::BusFault, 5);
        core.set_exception_priority(Exception::MemoryManagementFault, 4);
        core.set_exception_priority(Exception::DebugMonitor, 3);
    }

    #[cfg(feature = "has-fp")]
    fn write_basic_exception_frame(core: &mut Processor, frameptr: u32) {
        core.write32(frameptr, 1).unwrap();
        core.write32(frameptr.wrapping_add(0x4), 2).unwrap();
        core.write32(frameptr.wrapping_add(0x8), 3).unwrap();
        core.write32(frameptr.wrapping_add(0xc), 4).unwrap();
        core.write32(frameptr.wrapping_add(0x10), 12).unwrap();
        core.write32(frameptr.wrapping_add(0x14), 0x0800_00f1)
            .unwrap();
        core.write32(frameptr.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        core.write32(frameptr.wrapping_add(0x1c), 1 << 24).unwrap();
    }

    #[test]
    fn test_push_stack() {
        const STACK_START: u32 = 0x2000_0100;
        let mut core = Processor::new();

        // arrange
        let lr = {
            //    if self.control.sp_sel && self.mode == ProcessorMode::ThreadMode {
            core.control.sp_sel = false;
            //core.mode = ProcessorMode::ThreadMode;
            core.set_r(Reg::R0, 42);
            core.set_r(Reg::R1, 43);
            core.set_r(Reg::R2, 44);
            core.set_r(Reg::R3, 45);
            core.set_r(Reg::R12, 46);
            core.set_r(Reg::LR, 47);
            core.set_psp(0);
            core.set_msp(STACK_START);
            core.psr.value = 0xffff_ffff;

            // act
            core.push_stack(Exception::HardFault, 99).unwrap();

            assert_eq!(core.msp, STACK_START - 32);
            core.get_r(Reg::LR)
        };

        // values pushed on to stack
        assert_eq!(core.read32(STACK_START - 0x20).unwrap(), 42);
        assert_eq!(core.read32(STACK_START - 0x20 + 4).unwrap(), 43);
        assert_eq!(core.read32(STACK_START - 0x20 + 8).unwrap(), 44);
        assert_eq!(core.read32(STACK_START - 0x20 + 12).unwrap(), 45);
        assert_eq!(core.read32(STACK_START - 0x20 + 16).unwrap(), 46);
        assert_eq!(core.read32(STACK_START - 0x20 + 20).unwrap(), 47);
        assert_eq!(core.read32(STACK_START - 0x20 + 24).unwrap(), 99);
        assert_eq!(
            core.read32(STACK_START - 0x20 + 28).unwrap(),
            0b1111_1111_1111_1111_1111_1101_1111_1111
        );
        assert_eq!(lr, 0xffff_fff9);
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_push_stack_with_active_fp_context_uses_extended_frame_and_fp_exc_return() {
        const STACK_START: u32 = 0x2000_0100;
        let mut core = fp_test_processor();

        core.control.fpca = true;
        core.fpccr = 0;
        core.control.sp_sel = true;
        core.mode = ProcessorMode::ThreadMode;
        core.set_r(Reg::R0, 42);
        core.set_r(Reg::R1, 43);
        core.set_r(Reg::R2, 44);
        core.set_r(Reg::R3, 45);
        core.set_r(Reg::R12, 46);
        core.set_r(Reg::LR, 47);
        core.set_psp(STACK_START);
        core.set_msp(0);
        core.fpscr = 0xabcd_1234;
        core.psr.value = 0xffff_ffff;

        seed_low_fp_registers(&mut core, 0x1111_0000);

        core.push_stack(Exception::HardFault, 99).unwrap();

        assert_eq!(core.psp, STACK_START - 0x68);
        assert_eq!(core.read32(STACK_START - 0x68).unwrap(), 42);
        assert_eq!(core.read32(STACK_START - 0x68 + 4).unwrap(), 43);
        assert_eq!(core.read32(STACK_START - 0x68 + 8).unwrap(), 44);
        assert_eq!(core.read32(STACK_START - 0x68 + 12).unwrap(), 45);
        assert_eq!(core.read32(STACK_START - 0x68 + 16).unwrap(), 46);
        assert_eq!(core.read32(STACK_START - 0x68 + 20).unwrap(), 47);
        assert_eq!(core.read32(STACK_START - 0x68 + 24).unwrap(), 99);
        assert_eq!(
            core.read32(STACK_START - 0x68 + 28).unwrap(),
            0b1111_1111_1111_1111_1111_1101_1111_1111
        );

        for i in 0..16 {
            assert_eq!(
                core.read32(STACK_START - 0x68 + 0x20 + (i * 4)).unwrap(),
                0x1111_0000 + i
            );
        }

        assert_eq!(core.read32(STACK_START - 0x68 + 0x60).unwrap(), 0xabcd_1234);
        assert_eq!(core.get_r(Reg::LR), 0xffff_ffed);
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_push_stack_with_lazy_fp_state_sets_lspact_and_fpcar() {
        const STACK_START: u32 = 0x2000_0100;
        let mut core = fp_test_processor();

        core.control.fpca = true;
        core.mode = ProcessorMode::ThreadMode;
        core.set_msp(STACK_START);
        core.fpccr = 1 << FPCCR_LSPEN;

        core.push_stack(Exception::HardFault, 99).unwrap();

        assert_eq!(core.msp, STACK_START - 0x68);
        assert!(core.fpccr.get_bit(FPCCR_LSPACT));
        assert_eq!(core.fpcar, STACK_START - 0x68 + 0x20);
        assert_eq!(core.get_r(Reg::LR), 0xffff_ffe9);
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_update_fpccr_sets_ready_bits_from_fault_enables_and_demcr() {
        let mut core = fp_test_processor();

        core.control.fpca = true;
        core.mode = ProcessorMode::ThreadMode;
        core.execution_priority = 10;
        core.control.n_priv = true;
        core.fpccr = (1 << FPCCR_ASPEN) | (1 << FPCCR_LSPEN);
        configure_update_fpccr_ready_context(&mut core);

        core.update_fpccr(0x2000_0080).unwrap();

        assert_eq!(core.fpcar, 0x2000_00a0);
        assert!(core.fpccr.get_bit(FPCCR_LSPACT));
        assert!(core.fpccr.get_bit(FPCCR_USER));
        assert!(core.fpccr.get_bit(FPCCR_THREAD));
        assert!(core.fpccr.get_bit(FPCCR_HFRDY));
        assert!(core.fpccr.get_bit(FPCCR_BFRDY));
        assert!(core.fpccr.get_bit(FPCCR_MMRDY));
        assert!(core.fpccr.get_bit(FPCCR_MONRDY));
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_update_fpccr_clears_ready_bits_when_fault_handlers_not_ready() {
        let mut core = fp_test_processor();

        core.control.fpca = true;
        core.mode = ProcessorMode::HandlerMode;
        core.execution_priority = -1;
        core.control.n_priv = false;
        core.fpccr = (1 << FPCCR_ASPEN) | (1 << FPCCR_LSPEN);
        configure_update_fpccr_ready_context(&mut core);

        core.update_fpccr(0x2000_0080).unwrap();

        assert!(!core.fpccr.get_bit(FPCCR_USER));
        assert!(!core.fpccr.get_bit(FPCCR_THREAD));
        assert!(!core.fpccr.get_bit(FPCCR_HFRDY));
        assert!(!core.fpccr.get_bit(FPCCR_BFRDY));
        assert!(!core.fpccr.get_bit(FPCCR_MMRDY));
        assert!(!core.fpccr.get_bit(FPCCR_MONRDY));
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_update_fpccr_leaves_state_unmodified_when_fpca_clear() {
        let mut core = fp_test_processor();

        core.control.fpca = false;
        core.mode = ProcessorMode::ThreadMode;
        core.execution_priority = 10;
        core.control.n_priv = true;
        core.fpccr = 1 << FPCCR_LSPEN;
        core.fpcar = 0x2000_0120;
        configure_update_fpccr_ready_context(&mut core);

        core.update_fpccr(0x2000_0080).unwrap();

        assert_eq!(core.fpccr, 1 << FPCCR_LSPEN);
        assert_eq!(core.fpcar, 0x2000_0120);
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_update_fpccr_leaves_state_unmodified_when_lspen_clear() {
        let mut core = fp_test_processor();

        core.control.fpca = true;
        core.mode = ProcessorMode::ThreadMode;
        core.execution_priority = 10;
        core.control.n_priv = true;
        core.fpccr = 0;
        core.fpcar = 0x2000_0140;
        configure_update_fpccr_ready_context(&mut core);

        core.update_fpccr(0x2000_0080).unwrap();

        assert_eq!(core.fpccr, 0);
        assert_eq!(core.fpcar, 0x2000_0140);
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_pop_stack_with_extended_fp_frame_restores_fp_state() {
        const FRAMEPTR: u32 = 0x2000_0180;
        let mut core = fp_test_processor();

        core.set_msp(FRAMEPTR);
        write_basic_exception_frame(&mut core, FRAMEPTR);

        for i in 0..16 {
            core.write32(FRAMEPTR.wrapping_add(0x20 + i * 4), 0x2222_0000 + i)
                .unwrap();
        }
        core.write32(FRAMEPTR.wrapping_add(0x60), 0xabcd_1234)
            .unwrap();

        core.pop_stack(FRAMEPTR, 0xffff_ffe9).unwrap();

        for i in 0..16 {
            assert_eq!(core.get_sr(SingleReg::from(i as u8)), 0x2222_0000 + i);
        }
        assert_eq!(core.fpscr, 0xabcd_1234);
        assert!(core.control.fpca);
        assert_eq!(core.get_msp(), FRAMEPTR + 0x68);
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_pop_stack_with_lazy_fp_state_only_clears_lspact() {
        const FRAMEPTR: u32 = 0x2000_0200;
        let mut core = fp_test_processor();

        core.fpccr = 1 << FPCCR_LSPACT;
        core.fpscr = 0x1111_2222;
        core.set_msp(FRAMEPTR);
        core.write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x1c), 1 << 24).unwrap();

        core.pop_stack(FRAMEPTR, 0xffff_ffe9).unwrap();

        assert!(!core.fpccr.get_bit(FPCCR_LSPACT));
        assert_eq!(core.fpscr, 0x1111_2222);
        assert!(core.control.fpca);
        assert_eq!(core.get_msp(), FRAMEPTR + 0x68);
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_pop_stack_with_extended_fp_frame_updates_psp_for_thread_return() {
        const FRAMEPTR: u32 = 0x2000_0280;
        let mut core = fp_test_processor();

        core.fpccr = 1 << FPCCR_LSPACT;
        core.set_psp(FRAMEPTR);
        core.write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x1c), 1 << 24).unwrap();

        core.pop_stack(FRAMEPTR, 0xffff_ffed).unwrap();

        assert_eq!(core.get_psp(), FRAMEPTR + 0x68);
        assert!(core.control.fpca);
    }

    #[test]
    fn test_exception_taken() {
        // Arrange
        let mut core = Processor::new();

        core.control.sp_sel = true;
        core.mode = ProcessorMode::ThreadMode;
        core.psr.value = 0xffff_ffff;

        // Act
        core.exception_taken(Exception::BusFault).unwrap();

        // Assert
        assert!(!core.control.sp_sel);
        assert_eq!(core.mode, ProcessorMode::HandlerMode);
        assert_eq!(core.psr.get_isr_number(), Exception::BusFault.into());
        assert!(core.exception_active(Exception::BusFault));
    }

    #[test]
    fn test_get_execution_priority() {
        let mut p = Processor::new();

        p.reset().unwrap();
        p.msp = 0x2000_0400;
        p.set_exception_pending(Exception::HardFault);
        p.check_exceptions();

        assert_eq!(p.get_execution_priority(), -1);

        p.reset().unwrap();
        p.msp = 0x2000_0400;
        p.set_exception_pending(Exception::NMI);
        p.check_exceptions();
        assert_eq!(p.get_execution_priority(), -2);
    }

    #[test]
    fn test_exception_priority() {
        // Arrange
        let mut processor = Processor::new();

        processor.reset().unwrap();

        // Act
        processor.set_exception_pending(Exception::Reset);
        processor.set_exception_pending(Exception::NMI);
        processor.set_exception_pending(Exception::HardFault);

        // Assert
        assert_eq!(processor.get_pending_exception(), Some(Exception::Reset));
    }

    #[test]
    fn test_exception_priority_same_priority_setting() {
        // Arrange
        let mut processor = Processor::new();

        processor.reset().unwrap();

        // Act
        processor.set_exception_pending(Exception::MemoryManagementFault);
        processor.set_exception_pending(Exception::UsageFault);
        processor.set_exception_pending(Exception::BusFault);

        // Assert (exception number should define the priority for same priority setting)
        assert_eq!(
            processor.get_pending_exception(),
            Some(Exception::MemoryManagementFault)
        );
    }

    #[cfg(not(feature = "armv6m"))]
    #[test]
    fn test_faultmask_priority() {
        // Arrange
        let mut processor = Processor::new();

        processor.reset().unwrap();

        // Act
        processor.set_exception_pending(Exception::HardFault);

        processor.execute(
            &Instruction::CPS {
                im: true,
                affect_pri: false,
                affect_fault: true,
            },
            2,
        );

        // Assert
        assert_eq!(processor.get_pending_exception(), None);
    }

    #[test]
    fn test_exception_entry_clears_nvic() {
        // Arrange
        let mut processor = Processor::new();

        // Arrange
        processor.reset().unwrap();
        processor.nvic_write_iser(0, 1);
        processor.nvic_write_ispr(0, 1);
        assert_eq!(processor.nvic_read_ispr(0), 1);

        // Act
        let _ = processor.exception_entry(Exception::Interrupt { n: 0 }, 0);

        // Assert
        assert_eq!(processor.nvic_read_ispr(0), 0);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_check_exceptions_escalates_entry_stack_fault_to_hardfault() {
        let mut processor = Processor::new();

        processor.reset().unwrap();
        processor.set_msp(0);
        processor.set_exception_pending(Exception::SysTick);

        processor.check_exceptions();

        let trap = processor
            .take_pending_fault_trap()
            .expect("fault trap expected");
        assert_eq!(trap.fault, Fault::Mstkerr);
        assert_eq!(trap.exception, Exception::HardFault);
        assert_eq!(processor.cfsr, CFSR_MSTKERR);
        assert_eq!(processor.hfsr, HFSR_FORCED);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_exception_return_rejects_invalid_exc_return_encoding() {
        let mut processor = Processor::new();

        set_active_exception(&mut processor, Exception::SysTick);

        let result = processor.exception_return(0x0);

        assert_eq!(result, Err(Fault::InvPc));
        assert_eq!(processor.get_r(Reg::LR), 0xF000_0000);
        assert!(!processor.exception_active(Exception::SysTick));
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_exception_return_rejects_handler_return_without_nested_activation() {
        let mut processor = Processor::new();

        set_active_exception(&mut processor, Exception::SysTick);

        let result = processor.exception_return(EXC_RETURN_HANDLER);

        assert_eq!(result, Err(Fault::InvPc));
        assert_eq!(processor.get_r(Reg::LR), 0xF000_0000 + EXC_RETURN_HANDLER);
        assert!(!processor.exception_active(Exception::SysTick));
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_exception_return_rejects_thread_return_with_other_active_exception() {
        const FRAMEPTR: u32 = 0x2000_00e0;

        let mut processor = Processor::new();
        processor.set_msp(FRAMEPTR);
        processor
            .write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        processor
            .write32(FRAMEPTR.wrapping_add(0x1c), 1 << 24)
            .unwrap();

        set_active_exception(&mut processor, Exception::HardFault);
        processor
            .exceptions
            .get_mut(&usize::from(Exception::SysTick))
            .unwrap()
            .active = true;

        let result = processor.exception_return(EXC_RETURN_THREAD_MSP);

        assert_eq!(result, Err(Fault::InvPc));
        assert_eq!(
            processor.get_r(Reg::LR),
            0xF000_0000 + EXC_RETURN_THREAD_MSP
        );
        assert!(!processor.exception_active(Exception::HardFault));
        assert!(processor.exception_active(Exception::SysTick));
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_exception_return_rejects_handler_return_with_zero_stacked_ipsr() {
        const FRAMEPTR: u32 = 0x2000_00e0;

        let mut processor = Processor::new();
        processor.set_msp(FRAMEPTR);
        processor
            .write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        processor
            .write32(FRAMEPTR.wrapping_add(0x1c), 1 << 24)
            .unwrap();

        set_active_exception(&mut processor, Exception::HardFault);
        processor
            .exceptions
            .get_mut(&usize::from(Exception::SysTick))
            .unwrap()
            .active = true;

        let result = processor.exception_return(EXC_RETURN_HANDLER);

        assert_eq!(result, Err(Fault::InvPc));
        assert_eq!(processor.get_r(Reg::LR), 0xF000_0000 + EXC_RETURN_HANDLER);
        assert!(!processor.exception_active(Exception::HardFault));
        assert!(processor.exception_active(Exception::SysTick));
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_exception_return_rejects_thread_return_with_nonzero_stacked_ipsr() {
        const FRAMEPTR: u32 = 0x2000_00e0;

        let mut processor = Processor::new();
        processor.set_msp(FRAMEPTR);
        processor
            .write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        processor
            .write32(
                FRAMEPTR.wrapping_add(0x1c),
                (1 << 24) | usize::from(Exception::HardFault) as u32,
            )
            .unwrap();

        set_active_exception(&mut processor, Exception::SysTick);

        let result = processor.exception_return(EXC_RETURN_THREAD_MSP);

        assert_eq!(result, Err(Fault::InvPc));
        assert_eq!(
            processor.get_r(Reg::LR),
            0xF000_0000 + EXC_RETURN_THREAD_MSP
        );
        assert!(!processor.exception_active(Exception::SysTick));
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_exception_return_rejects_stacked_frame_with_thumb_bit_clear() {
        const FRAMEPTR: u32 = 0x2000_00e0;

        let mut processor = Processor::new();
        processor.set_msp(FRAMEPTR);
        processor.set_pc(0x0800_00aa);
        processor.set_r(Reg::R0, 0x1111_1111);
        processor.set_r(Reg::R1, 0x2222_2222);
        processor.set_r(Reg::R2, 0x3333_3333);
        processor.set_r(Reg::R3, 0x4444_4444);
        processor.set_r(Reg::R12, 0x5555_5555);
        processor.set_r(Reg::LR, 0x6666_6666);
        processor.write32(FRAMEPTR, 0xaaaa_0000).unwrap();
        processor
            .write32(FRAMEPTR.wrapping_add(0x4), 0xbbbb_0001)
            .unwrap();
        processor
            .write32(FRAMEPTR.wrapping_add(0x8), 0xcccc_0002)
            .unwrap();
        processor
            .write32(FRAMEPTR.wrapping_add(0xc), 0xdddd_0003)
            .unwrap();
        processor
            .write32(FRAMEPTR.wrapping_add(0x10), 0xeeee_0004)
            .unwrap();
        processor
            .write32(FRAMEPTR.wrapping_add(0x14), 0xffff_0005)
            .unwrap();
        processor
            .write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        processor.write32(FRAMEPTR.wrapping_add(0x1c), 0).unwrap();

        set_active_exception(&mut processor, Exception::SysTick);

        let result = processor.exception_return(EXC_RETURN_THREAD_MSP);

        assert_eq!(result, Err(Fault::Invstate));
        assert_eq!(processor.get_pc(), 0x0800_00aa);
        assert_eq!(processor.get_r(Reg::R0), 0x1111_1111);
        assert_eq!(processor.get_r(Reg::R1), 0x2222_2222);
        assert_eq!(processor.get_r(Reg::R2), 0x3333_3333);
        assert_eq!(processor.get_r(Reg::R3), 0x4444_4444);
        assert_eq!(processor.get_r(Reg::R12), 0x5555_5555);
        assert_eq!(processor.get_r(Reg::LR), 0x6666_6666);
    }

    #[test]
    fn test_wfi_wakeup_primask_masked_interrupt() {
        // Arrange
        let mut processor = Processor::new();
        processor.reset().unwrap();

        // Enable PRIMASK
        processor.primask = true;

        // Set pending interrupt
        processor.set_exception_pending(Exception::Interrupt { n: 1 });
        processor.set_exception_priority(Exception::Interrupt { n: 1 }, 10);

        // Act & Assert
        // Should return true because WFI ignores PRIMASK
        assert!(processor.has_wakeup_condition());
    }

    #[test]
    fn test_wfi_wakeup_no_interrupt() {
        // Arrange
        let mut processor = Processor::new();
        processor.reset().unwrap();

        // Act & Assert
        assert!(!processor.has_wakeup_condition());
    }

    #[test]
    fn test_wfi_wakeup_low_priority_interrupt() {
        // Arrange
        let mut processor = Processor::new();
        processor.reset().unwrap();

        // Simulate being in a high priority handler
        processor.set_exception_priority(Exception::SysTick, 0); // High urgency
        processor.exception_taken(Exception::SysTick).unwrap();

        // Set pending interrupt with lower urgency
        processor.set_exception_priority(Exception::Interrupt { n: 1 }, 10); // Low urgency
        processor.set_exception_pending(Exception::Interrupt { n: 1 });

        // Act & Assert
        // Should return false because pending interrupt priority (10) is not < current execution priority (0)
        assert!(!processor.has_wakeup_condition());
    }

    #[cfg(not(feature = "armv6m"))]
    #[test]
    fn test_wfi_wakeup_faultmask_masked_interrupt() {
        // Arrange
        let mut processor = Processor::new();
        processor.reset().unwrap();

        // Enable FAULTMASK
        processor.faultmask = true;

        // Set pending interrupt
        processor.set_exception_pending(Exception::Interrupt { n: 1 });
        processor.set_exception_priority(Exception::Interrupt { n: 1 }, 10);

        // Act & Assert
        // Should return false because FAULTMASK blocks normal interrupts
        assert!(!processor.has_wakeup_condition());
    }

    // pop_stack GE gating: with DSP extension GE bits must be restored from the stacked xPSR.
    #[test]
    #[cfg(feature = "has-dsp-ext")]
    fn test_pop_stack_restores_ge_bits_with_dsp_ext() {
        const FRAMEPTR: u32 = 0x2000_0100;
        let mut core = Processor::new();
        core.set_msp(FRAMEPTR);

        // Build a minimal exception frame (basic, no FP).
        // offset 0x00..0x18: r0..r3, r12, lr, pc (Thumb address, bit 0 = 1)
        core.write32(FRAMEPTR, 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x4), 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x8), 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0xc), 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x10), 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x14), 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        // Stacked xPSR: Thumb bit (bit 24) set; ISR = 0 (thread return); GE1 + GE3 set.
        // GE[3:0] = 0b1010 = bits [19:16] → 0x000A_0000
        let stacked_xpsr: u32 = (1 << 24) | 0x000A_0000;
        core.write32(FRAMEPTR.wrapping_add(0x1c), stacked_xpsr)
            .unwrap();

        // Clear GE bits in live PSR so we can verify they come from the stacked value.
        core.psr.set_ge0(false);
        core.psr.set_ge1(false);
        core.psr.set_ge2(false);
        core.psr.set_ge3(false);

        // EXC_RETURN = 0xFFFF_FFF9: return-to-thread, MSP, no FP frame.
        core.pop_stack(FRAMEPTR, 0xFFFF_FFF9).unwrap();

        assert!(!core.psr.get_ge0(), "GE0 must remain clear");
        assert!(core.psr.get_ge1(), "GE1 must be restored from stacked xPSR");
        assert!(!core.psr.get_ge2(), "GE2 must remain clear");
        assert!(core.psr.get_ge3(), "GE3 must be restored from stacked xPSR");
    }

    // pop_stack GE gating: without DSP extension GE bits in the stacked xPSR must be ignored.
    #[test]
    #[cfg(not(feature = "has-dsp-ext"))]
    fn test_pop_stack_ignores_ge_bits_without_dsp_ext() {
        const FRAMEPTR: u32 = 0x2000_0200;
        let mut core = Processor::new();
        core.set_msp(FRAMEPTR);

        core.write32(FRAMEPTR, 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x4), 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x8), 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0xc), 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x10), 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x14), 0).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        // Stacked xPSR: Thumb bit set + all GE bits set.
        let stacked_xpsr: u32 = (1 << 24) | 0x000F_0000;
        core.write32(FRAMEPTR.wrapping_add(0x1c), stacked_xpsr)
            .unwrap();

        // GE bits in live PSR are already zero (internal storage may still hold them).
        // Capture state before pop.
        let pre_psr_ge_bits = core.psr.value & 0x000F_0000;

        core.pop_stack(FRAMEPTR, 0xFFFF_FFF9).unwrap();

        // Without DSP extension the GE bits in the live PSR must not change.
        let post_psr_ge_bits = core.psr.value & 0x000F_0000;
        assert_eq!(
            pre_psr_ge_bits, post_psr_ge_bits,
            "GE bits must not be modified by pop_stack without DSP extension"
        );
    }
}
