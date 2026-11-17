//!
//! Processor simulator for ARM Cortex-M processors.
//!
//!
#![deny(missing_docs)]
#![doc(test(attr(allow(unused_variables), deny(warnings))))]
#![deny(clippy::all)]
#![deny(clippy::pedantic)]
#![allow(clippy::inline_always)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
#![allow(clippy::cast_possible_wrap)]
#![allow(clippy::match_same_arms)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::new_without_default)]
#![allow(clippy::similar_names)]
#![allow(clippy::must_use_candidate)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::missing_panics_doc)]
#![allow(clippy::upper_case_acronyms)]
#![allow(clippy::too_many_lines)]
#![allow(clippy::redundant_else)]
#![allow(clippy::empty_docs)]
#![allow(clippy::struct_excessive_bools)]
#![allow(clippy::many_single_char_names)]
#![allow(clippy::too_many_arguments)]

extern crate byteorder;
extern crate enum_set;

pub mod bus;
pub mod core;
pub mod decoder;
pub mod executor;
pub mod gdb;
pub mod memory;
pub mod peripheral;
pub mod semihosting;
pub mod system;

use crate::core::instruction::instruction_size;

use crate::core::exception::Exception;
use crate::core::fault::{Fault, FaultContext, FaultStatusContext, FaultTrapMode};
use crate::core::fetch::Fetch;
use crate::core::instruction::Instruction;
use crate::core::register::{Apsr, BaseReg, Control, PSR, Reg};

use crate::memory::flash::FlashMemory;
use crate::memory::map::MemoryMapConfig;
use crate::memory::ram::RAM;
use crate::semihosting::SemihostingCommand;
use crate::semihosting::SemihostingResponse;

use crate::core::exception::ExceptionState;
use decoder::Decoder;
use std::collections::HashMap;
use std::fmt;
use std::io;

#[derive(PartialEq, Debug, Copy, Clone)]
/// Main execution mode of the processor
pub enum ProcessorMode {
    /// Thread mode
    ThreadMode,
    /// Handler mode
    HandlerMode,
}

type SemihostingCall = Option<Box<dyn FnMut(&SemihostingCommand) -> SemihostingResponse>>;

/// External device/peripheral bus attachment owned by the caller.
pub type DeviceBus = Box<dyn crate::bus::Bus + 'static>;

#[derive(PartialEq, Debug, Copy, Clone)]
enum CachedInstruction {
    Decoded {
        instruction: Instruction,
        instruction_size: usize,
    },
    FetchFault {
        fault: Fault,
        status: FaultStatusContext,
    },
}
///
/// Representation of all Processor related data
///
#[allow(missing_docs)]
pub struct Processor {
    /// 13 of 32-bit general purpose registers.
    pub r0_12: [u32; 13],

    /// 32 of 32-bit floating pointer registers
    pub fp_regs: [u32; 32],

    /// MSP, virtual reg r[13]
    pub msp: u32,
    /// PSP, virtual reg r[13]
    pub psp: u32,
    pub lr: u32,
    pc: u32,

    /// Total number of processor clock cycles run
    pub cycle_count: u64,
    pub instruction_count: u64,

    /// Processor state register, status flags.
    pub psr: PSR,

    ///
    /// interrupt primary mask, a 1 bit mask register for
    /// global interrupt masking
    ///
    primask: bool,

    ///
    /// interrupt fault mask, a 1 bit mask register for
    /// global interrupt masking
    ///
    #[cfg(not(feature = "armv6m"))]
    faultmask: bool,

    ///
    /// basepri for selection of executed interrupt priorities
    ///
    #[cfg(not(feature = "armv6m"))]
    basepri: u8,

    ///
    /// Control bits: currently used stack and execution privilege if core.mode == `ThreadMode`
    ///
    control: Control,

    ///
    /// Processor mode: either handler or thread mode.
    ///
    mode: ProcessorMode,

    ///
    /// processor simulation state
    ///
    /// bit 1 : 1= processor sleeping, 0 : processor awake
    pub running: bool,
    pub sleeping: bool,
    pub exit_code: u32,

    ///
    /// lookup table for exceptions and their states
    ///
    pub exceptions: HashMap<usize, ExceptionState>,

    ///
    /// number of exceptions currently pending, used for optimization purposes
    ///
    pub pending_exception_count: u32,

    ///
    /// cached current execution priority, used for optimization
    /// of exception activation rule resolving
    ///
    pub execution_priority: i16,

    itstate: u8,

    ///
    /// flash memory data
    ///
    pub code: FlashMemory,
    ///
    /// ram data
    ///
    pub sram: RAM,

    pub cpuid: u32,
    pub icsr: u32,
    pub vtor: u32,
    pub aircr: u32,
    pub scr: u32,
    pub ccr: u32,
    pub shcsr: u32,
    pub cfsr: u32,
    pub hfsr: u32,
    pub dfsr: u32,
    pub demcr: u32,
    pub mmfar: u32,
    pub bfar: u32,
    pub afsr: u32,
    #[cfg(feature = "has-fp")]
    pub cpacr: u32,

    #[cfg(feature = "has-fp")]
    pub fpccr: u32,
    #[cfg(feature = "has-fp")]
    pub fpcar: u32,
    #[cfg(feature = "has-fp")]
    pub fpdscr: u32,
    pub fpscr: u32,

    #[cfg(feature = "has-fp")]
    pub mvfr0: u32,
    #[cfg(feature = "has-fp")]
    pub mvfr1: u32,
    #[cfg(feature = "has-fp")]
    pub mvfr2: u32,

    pub ictr: u32,
    pub actlr: u32,

    pub nvic_interrupt_enabled: [u32; 16],
    pub nvic_interrupt_pending: [u32; 16],

    pub dwt_ctrl: u32,
    pub dwt_cyccnt: u32,

    pub syst_rvr: u32,
    pub syst_cvr: u32,
    pub syst_csr: u32,

    ///
    /// file handle to which to write ITM data
    ///
    pub itm_file: Option<Box<dyn io::Write + 'static>>,

    ///
    /// semihosting plug
    ///
    semihost_func: SemihostingCall,

    instruction_cache: Vec<CachedInstruction>,

    fault_trap_mode: FaultTrapMode,
    pending_fault_trap: Option<FaultContext>,
    pending_fault_status: Option<FaultStatusContext>,

    pub last_pc: u32,

    mem_map: Option<MemoryMapConfig>,

    device: Option<DeviceBus>,
}

#[cfg(feature = "has-fp")]
#[cfg(feature = "fpv5-d16")]
pub(crate) const FP_MVFR0_RESET: u32 = 0x1011_0221;
#[cfg(feature = "has-fp")]
#[cfg(any(feature = "fpv4-sp-d16", feature = "fpv5-sp-d16"))]
pub(crate) const FP_MVFR0_RESET: u32 = 0x1011_0021;

#[cfg(feature = "has-fp")]
#[cfg(feature = "fpv5-d16")]
pub(crate) const FP_MVFR1_RESET: u32 = 0x1200_0011;
#[cfg(feature = "has-fp")]
#[cfg(any(feature = "fpv4-sp-d16", feature = "fpv5-sp-d16"))]
pub(crate) const FP_MVFR1_RESET: u32 = 0x1100_0011;

#[cfg(feature = "has-fp")]
#[cfg(any(feature = "fpv5-d16", feature = "fpv5-sp-d16"))]
pub(crate) const FP_MVFR2_RESET: u32 = 0x0000_0040;
#[cfg(feature = "has-fp")]
#[cfg(feature = "fpv4-sp-d16")]
pub(crate) const FP_MVFR2_RESET: u32 = 0x0000_0000;

fn make_default_exception_priorities() -> HashMap<usize, ExceptionState> {
    let mut priorities = HashMap::new();

    priorities.insert(
        Exception::Reset.into(),
        ExceptionState::new(Exception::Reset, -3),
    );
    priorities.insert(
        Exception::NMI.into(),
        ExceptionState::new(Exception::NMI, -2),
    );
    priorities.insert(
        Exception::HardFault.into(),
        ExceptionState::new(Exception::HardFault, -1),
    );

    priorities.insert(
        Exception::MemoryManagementFault.into(),
        ExceptionState::new(Exception::MemoryManagementFault, 0),
    );

    priorities.insert(
        Exception::BusFault.into(),
        ExceptionState::new(Exception::BusFault, 0),
    );

    priorities.insert(
        Exception::UsageFault.into(),
        ExceptionState::new(Exception::UsageFault, 0),
    );

    priorities.insert(
        Exception::DebugMonitor.into(),
        ExceptionState::new(Exception::DebugMonitor, 0),
    );

    priorities.insert(
        Exception::SVCall.into(),
        ExceptionState::new(Exception::SVCall, 0),
    );

    priorities.insert(
        Exception::PendSV.into(),
        ExceptionState::new(Exception::PendSV, 0),
    );

    priorities.insert(
        Exception::SysTick.into(),
        ExceptionState::new(Exception::SysTick, 0),
    );

    for irqn in 0..32 {
        let irq = Exception::Interrupt { n: irqn };
        priorities.insert(irq.into(), ExceptionState::new(irq, 0));
    }

    priorities
}

type SemihostingStaticCall =
    Option<Box<dyn FnMut(&SemihostingCommand) -> SemihostingResponse + 'static>>;

impl Processor {
    ///
    /// Create processor with default data
    ///
    pub fn new() -> Self {
        Self {
            mode: ProcessorMode::ThreadMode,
            vtor: 0,
            psr: PSR { value: 0 },
            primask: false,
            #[cfg(not(feature = "armv6m"))]
            faultmask: false,
            #[cfg(not(feature = "armv6m"))]
            basepri: 0,
            control: Control {
                n_priv: false,
                sp_sel: false,
                #[cfg(feature = "has-fp")]
                fpca: false,
            },
            r0_12: [0; 13],
            fp_regs: [0; 32],
            pc: 0,
            msp: 0,
            psp: 0,
            lr: 0,
            // TODO make flash size configurable
            code: FlashMemory::new(65536, &vec![0; 65536].into_boxed_slice()),
            // TODO make RAM size configurable
            sram: RAM::new_with_fill(0x2000_0000, 128 * 1024, 0xcd),
            itm_file: None,
            sleeping: false,
            running: true,
            exit_code: 0,
            cycle_count: 0,
            instruction_count: 0,
            exceptions: make_default_exception_priorities(),
            execution_priority: 0,
            pending_exception_count: 0,
            itstate: 0,
            semihost_func: None,
            cpuid: 0,
            icsr: 0,
            aircr: 0,
            scr: 0,
            ccr: 0,
            shcsr: 0,
            cfsr: 0,
            dfsr: 0,
            hfsr: 0,
            demcr: 0,
            mmfar: 0,
            bfar: 0,
            afsr: 0,
            #[cfg(feature = "has-fp")]
            cpacr: 0,

            #[cfg(feature = "has-fp")]
            fpccr: 0,
            #[cfg(feature = "has-fp")]
            fpcar: 0,
            #[cfg(feature = "has-fp")]
            fpdscr: 0,
            fpscr: 0,
            #[cfg(feature = "has-fp")]
            mvfr0: FP_MVFR0_RESET,
            #[cfg(feature = "has-fp")]
            mvfr1: FP_MVFR1_RESET,
            #[cfg(feature = "has-fp")]
            mvfr2: FP_MVFR2_RESET,

            ictr: 0,
            actlr: 0,

            dwt_ctrl: 0x4000_0000,
            dwt_cyccnt: 0,

            nvic_interrupt_enabled: [0; 16],
            nvic_interrupt_pending: [0; 16],
            syst_rvr: 0,
            syst_cvr: 0,
            syst_csr: 0,
            instruction_cache: Vec::new(),
            fault_trap_mode: FaultTrapMode::hardfault(),
            pending_fault_trap: None,
            pending_fault_status: None,
            last_pc: 0,
            mem_map: None,
            device: None,
        }
    }

    /// Configure flash memory
    pub fn flash_memory<'a>(&'a mut self, flash_size: usize, code: &[u8]) -> &'a mut Self {
        self.code = FlashMemory::new(flash_size, code);
        self
    }

    /// Configure memory mapping
    pub fn memory_map(&mut self, map: Option<MemoryMapConfig>) -> &mut Self {
        self.mem_map = map;
        self
    }

    /// Attach or replace the external device/peripheral bus implementation.
    pub fn device(&mut self, device: Option<DeviceBus>) -> &mut Self {
        self.device = device;
        self
    }

    /// Configure itm output file
    pub fn itm<'a>(&'a mut self, file: Option<Box<dyn io::Write + 'static>>) -> &'a mut Self {
        self.itm_file = file;
        self
    }

    /// Configure semihosting
    pub fn semihost(&mut self, func: SemihostingStaticCall) -> &mut Self {
        self.semihost_func = func;
        self
    }

    /// Configure fault-trap behavior.
    pub fn fault_trap_mode(&mut self, mode: FaultTrapMode) -> &mut Self {
        self.fault_trap_mode = mode;
        self
    }

    /// Return the active fault-trap mode.
    pub fn get_fault_trap_mode(&self) -> FaultTrapMode {
        self.fault_trap_mode
    }

    /// Take the currently pending fault trap, if any.
    pub fn take_pending_fault_trap(&mut self) -> Option<FaultContext> {
        self.pending_fault_trap.take()
    }

    pub(crate) fn fault_with_status(&mut self, fault: Fault, status: FaultStatusContext) -> Fault {
        self.pending_fault_status = Some(status);
        fault
    }

    pub(crate) fn take_pending_fault_status(&mut self) -> FaultStatusContext {
        self.pending_fault_status.take().unwrap_or_default()
    }

    ///
    /// Pre cache (decode) instructions to speed up simulation
    ///
    pub fn cache_instructions(&mut self) {
        // pre-cache the decoded instructions
        {
            let mut pc = 0;

            while pc < (self.code.len() as u32) {
                match self.fetch(pc) {
                    Ok(thumb) => {
                        let instruction = self.decode(thumb);
                        self.instruction_cache.push(CachedInstruction::Decoded {
                            instruction,
                            instruction_size: instruction_size(&instruction),
                        });
                    }
                    Err(fault) => {
                        let status = self.take_pending_fault_status();
                        self.instruction_cache
                            .push(CachedInstruction::FetchFault { fault, status });
                    }
                }
                pc += 2;
            }
        }
    }

    ///
    /// `BigEndian()` returns true if the processor is big endian
    /// TODO: check architectures where this is supported
    pub fn big_endian(&self) -> bool {
        // TODO: should be AIRCR.ENDIANNESS==1
        false
    }

    /// Get the IT state register value
    pub fn get_itstate(&self) -> u8 {
        self.itstate
    }

    /// Check if currently running in privileged mode
    pub fn current_mode_is_privileged(&self) -> bool {
        self.mode == ProcessorMode::HandlerMode || !self.control.n_priv
    }
}

impl fmt::Display for Processor {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "PC:{:08X} {}{}{}{}{} R0:{:08X} R1:{:08X} R2:{:08X} R3:{:08X} R4:{:08X} R5:{:08X} \
                  R6:{:08X} R7:{:08X} R8:{:08X} R9:{:08X} R10:{:08X} R11:{:08X} R12:{:08X} SP:{:08X} LR:{:08X}",
            self.get_r(Reg::PC),
            if self.psr.get_z() { 'Z' } else { 'z' },
            if self.psr.get_n() { 'N' } else { 'n' },
            if self.psr.get_c() { 'C' } else { 'c' },
            if self.psr.get_v() { 'V' } else { 'v' },
            if self.psr.get_q() { 'Q' } else { 'q' },
            self.get_r(Reg::R0),
            self.get_r(Reg::R1),
            self.get_r(Reg::R2),
            self.get_r(Reg::R3),
            self.get_r(Reg::R4),
            self.get_r(Reg::R5),
            self.get_r(Reg::R6),
            self.get_r(Reg::R7),
            self.get_r(Reg::R8),
            self.get_r(Reg::R9),
            self.get_r(Reg::R10),
            self.get_r(Reg::R11),
            self.get_r(Reg::R12),
            self.get_r(Reg::SP),
            self.get_r(Reg::LR)
        )
    }
}
