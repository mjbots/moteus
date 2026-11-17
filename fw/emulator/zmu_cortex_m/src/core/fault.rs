//!
//! A Trait for representing a Cortex-M fault
//!
//!
//!

use crate::core::exception::Exception;
use std::fmt;

#[derive(thiserror::Error, Eq, PartialEq, Debug, Copy, Clone)]
///
/// Fault types
/// See Armv7-M Architecture Reference Manual, DDI0403E.e, B1.5.14 List of Armv7-M faults
///
pub enum Fault {
    ///
    /// Bus error on vector table read
    ///
    #[error("Bus error on vector table read")]
    VectorTable,
    ///
    /// Fault or supervisor call escalation to hard fault.
    ///
    #[error("Fault or supervisor call escalation to hard fault")]
    Forced,
    ///
    /// Hardfault on BKPT escalation
    ///
    #[error("Hardfault on BKPT escalation")]
    DebugEvt,
    ///
    /// Bus fault on exception entry stack operations
    ///
    #[error("Bus fault on exception entry stack operations")]
    Stkerr,
    ///
    /// Memmanage fault on exception entry with stack operations
    ///
    #[error("Memmanage fault on exception entry with stack operations")]
    Mstkerr,
    ///
    /// Bus fault on exception return stack operations
    ///
    #[error("Bus fault on exception return stack operations")]
    Msunskerr,
    ///
    /// Memmanage fault on data access
    ///
    #[error("Memmanage fault on data access")]
    DAccViol,
    ///
    /// Memmanage fault on instruction access
    ///
    #[error("Memmanage fault on instruction access")]
    IAccViol,
    ///
    /// Busfault on instruction fetch, precise access
    ///
    #[error("Busfault on instruction fetch, precise access")]
    IBusErr,
    ///
    /// Busfault on data access, precise
    ///
    #[error("Busfault on data access, precise")]
    Preciserr,
    ///
    /// Busfault on data access, imprecise
    ///
    #[error("Busfault on data access, imprecise")]
    Impreciseerr,
    ///
    /// Usage fault, no coprocessor
    ///
    #[error("Usage fault, no coprocessor")]
    Nocp,
    ///
    /// Unknown instruction was tried to be executed.
    ///
    #[error("Unknown instruction was tried to be executed.")]
    UndefInstr,
    ///
    /// Usage fault attempt to execute an instruction that is not permitted in the current state.
    #[error(
        "Usage fault attempt to execute an instruction that is not permitted in the current state."
    )]
    Invstate,
    ///
    /// Usage fault on exception return integrity check
    ///
    #[error("Usage fault on exception return integrity check")]
    InvPc,
    ///
    /// Usage fault illegal unaligned memory load or store
    ///
    #[error("Usage fault illegal unaligned memory load or store")]
    Unaligned,
    ///
    /// Divide number by zero
    ///
    #[error("Divide number by zero")]
    DivByZero,
    ///
    /// Memmanage fault, delayed fp processing
    ///
    #[error("Memmanage fault, delayed fp processing")]
    MlspErr,
    ///
    /// Busfault, delayed fp preservation
    ///
    #[error("Busfault, delayed fp preservation")]
    LspErr,
}

#[derive(PartialEq, Eq, Debug, Copy, Clone)]
/// Why execution stopped because of fault handling.
pub enum FaultTrapReason {
    /// A configured fault trap fired.
    Fault,
    /// The core reached lockup state and execution must stop.
    Lockup,
}

#[derive(PartialEq, Eq, Debug, Copy, Clone)]
/// Fault trap configuration.
pub struct FaultTrapMode {
    trap_hardfault: bool,
    trap_memmanage: bool,
    trap_busfault: bool,
    trap_usagefault: bool,
}

#[derive(PartialEq, Eq, Debug, Copy, Clone, Default)]
/// Additional metadata used when latching SCB fault status registers.
pub struct FaultStatusContext {
    /// Faulting access address when the architecture defines it as valid.
    pub fault_address: Option<u32>,
}

impl FaultStatusContext {
    /// Create status context with a faulting access address.
    pub const fn with_fault_address(fault_address: u32) -> Self {
        Self {
            fault_address: Some(fault_address),
        }
    }
}

#[derive(PartialEq, Debug, Copy, Clone)]
/// Information about a trapped fault.
pub struct FaultContext {
    /// Why the trap fired.
    pub trap_reason: FaultTrapReason,
    /// The underlying fault.
    pub fault: Fault,
    /// The architecturally mapped exception.
    pub exception: Exception,
    /// The program counter of the faulting instruction or fetch.
    pub pc: u32,
    /// The active exception at the moment the fault was raised.
    pub active_exception: Option<Exception>,
}

impl fmt::Display for FaultContext {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let prefix = match self.trap_reason {
            FaultTrapReason::Fault => "fault trap",
            FaultTrapReason::Lockup => "lockup trap",
        };

        match self.active_exception {
            Some(active_exception) => write!(
                f,
                "{prefix}: fault={:?}, exception={:?}, pc=0x{:08x}, active_exception={active_exception:?}",
                self.fault, self.exception, self.pc
            ),
            None => write!(
                f,
                "{prefix}: fault={:?}, exception={:?}, pc=0x{:08x}, active_exception=ThreadMode",
                self.fault, self.exception, self.pc
            ),
        }
    }
}

impl FaultTrapMode {
    /// Trap no configurable faults.
    pub const fn none() -> Self {
        Self {
            trap_hardfault: false,
            trap_memmanage: false,
            trap_busfault: false,
            trap_usagefault: false,
        }
    }

    /// Trap `HardFault` only.
    pub const fn hardfault() -> Self {
        Self {
            trap_hardfault: true,
            ..Self::none()
        }
    }

    /// Trap all architecturally visible fault exceptions.
    pub const fn all() -> Self {
        #[cfg(feature = "armv6m")]
        {
            Self::hardfault()
        }

        #[cfg(not(feature = "armv6m"))]
        Self {
            trap_hardfault: true,
            trap_memmanage: true,
            trap_busfault: true,
            trap_usagefault: true,
        }
    }

    /// Enable or disable trapping for a mapped exception.
    pub fn set_trap(&mut self, exception: Exception, enabled: bool) {
        match exception {
            Exception::HardFault => self.trap_hardfault = enabled,
            Exception::MemoryManagementFault => self.trap_memmanage = enabled,
            Exception::BusFault => self.trap_busfault = enabled,
            Exception::UsageFault => self.trap_usagefault = enabled,
            _ => {}
        }
    }

    /// Determine whether a fault mapped to `exception` should stop execution.
    pub fn should_trap(self, exception: Exception) -> bool {
        match self {
            Self {
                trap_hardfault,
                trap_memmanage,
                trap_busfault,
                trap_usagefault,
            } => match exception {
                Exception::HardFault => trap_hardfault,
                Exception::MemoryManagementFault => trap_memmanage,
                Exception::BusFault => trap_busfault,
                Exception::UsageFault => trap_usagefault,
                _ => false,
            },
        }
    }
}

impl Default for FaultTrapMode {
    fn default() -> Self {
        Self::hardfault()
    }
}

impl Fault {
    /// Map a generic memory-access fault into the corresponding instruction-fetch fault.
    #[must_use]
    pub fn on_instruction_fetch(self) -> Self {
        match self {
            Self::DAccViol | Self::IAccViol | Self::Mstkerr | Self::MlspErr => Self::IAccViol,
            Self::Preciserr
            | Self::Impreciseerr
            | Self::Stkerr
            | Self::Msunskerr
            | Self::LspErr
            | Self::VectorTable => Self::IBusErr,
            other => other,
        }
    }

    /// Map a vector-table read failure to the architectural vector-table fault.
    #[must_use]
    pub fn on_vector_read(self) -> Self {
        match self {
            Self::VectorTable => Self::VectorTable,
            _ => Self::VectorTable,
        }
    }

    /// Map exception-entry stack access failures to stacking faults.
    #[must_use]
    pub fn on_exception_entry_stack(self) -> Self {
        match self {
            Self::DAccViol | Self::IAccViol | Self::MlspErr | Self::Mstkerr => Self::Mstkerr,
            Self::Preciserr
            | Self::Impreciseerr
            | Self::IBusErr
            | Self::Stkerr
            | Self::Msunskerr
            | Self::LspErr
            | Self::VectorTable => Self::Stkerr,
            other => other,
        }
    }

    /// Map a fault to the architecturally visible exception.
    pub fn exception(self) -> Exception {
        #[cfg(feature = "armv6m")]
        {
            Exception::HardFault
        }

        #[cfg(not(feature = "armv6m"))]
        {
            match self {
                Self::Forced | Self::DebugEvt | Self::VectorTable => Exception::HardFault,
                Self::DAccViol | Self::IAccViol | Self::Mstkerr | Self::MlspErr => {
                    Exception::MemoryManagementFault
                }
                Self::Stkerr
                | Self::Msunskerr
                | Self::IBusErr
                | Self::Preciserr
                | Self::Impreciseerr
                | Self::LspErr => Exception::BusFault,
                Self::Nocp
                | Self::UndefInstr
                | Self::Invstate
                | Self::InvPc
                | Self::Unaligned
                | Self::DivByZero => Exception::UsageFault,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(feature = "armv6m")]
    fn test_fault_trap_mode_all_matches_hardfault_on_armv6m() {
        let mode = FaultTrapMode::all();

        assert!(mode.should_trap(Exception::HardFault));
        assert!(!mode.should_trap(Exception::MemoryManagementFault));
        assert!(!mode.should_trap(Exception::BusFault));
        assert!(!mode.should_trap(Exception::UsageFault));
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_fault_trap_mode_all_enables_all_fault_classes() {
        let mode = FaultTrapMode::all();

        assert!(mode.should_trap(Exception::HardFault));
        assert!(mode.should_trap(Exception::MemoryManagementFault));
        assert!(mode.should_trap(Exception::BusFault));
        assert!(mode.should_trap(Exception::UsageFault));
    }

    #[test]
    fn test_exception_entry_stack_mapping() {
        assert_eq!(Fault::DAccViol.on_exception_entry_stack(), Fault::Mstkerr);
        assert_eq!(Fault::IAccViol.on_exception_entry_stack(), Fault::Mstkerr);
        assert_eq!(Fault::Preciserr.on_exception_entry_stack(), Fault::Stkerr);
        assert_eq!(Fault::IBusErr.on_exception_entry_stack(), Fault::Stkerr);
    }
}
