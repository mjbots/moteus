//!
//! Cortex system simulation framework
//!

use crate::DeviceBus;
use crate::MemoryMapConfig;
use crate::Processor;
use crate::core::fault::{Fault, FaultContext, FaultTrapMode, FaultTrapReason};
use crate::core::register::{BaseReg, Ipsr, Reg};
use crate::core::reset::Reset;
use crate::executor::Executor;
use crate::semihosting::SemihostingCommand;
use crate::semihosting::SemihostingResponse;
use std::io;
use std::time::Duration;
use std::time::Instant;

///
/// Various reasons for simulation to stop before completing fully
///
#[derive(thiserror::Error, Debug)]
pub enum SimulationError {
    ///
    /// A fault was triggered and escalated to stop the simulation
    ///
    #[error("{context}")]
    FaultTrap {
        /// Detailed trap context.
        context: FaultContext,
    },
}

///
/// Statistical information on the simulation run.
///
///
pub struct SimulationStatistics {
    ///
    /// Total number of instructions executed (taken, or not taken).
    ///
    pub instruction_count: u64,

    ///
    /// Number of system clock cycles simulated.
    ///
    pub cycle_count: u64,

    ///
    /// Wallclock time spent for the simulation
    ///
    pub duration: Duration,

    ///
    /// exit code from process, if any
    ///
    pub exit_code: u32,
}

impl SimulationError {
    pub(crate) fn from_fault(processor: &Processor, fault: Fault) -> Self {
        let active_exception = match processor.psr.get_isr_number() {
            0 => None,
            n => Some(crate::core::exception::Exception::from(n)),
        };

        Self::FaultTrap {
            context: FaultContext {
                trap_reason: FaultTrapReason::Fault,
                fault,
                exception: fault.exception(),
                pc: processor.get_r(Reg::PC),
                active_exception,
            },
        }
    }
}

///
/// Run simulation until processing gets terminated
///
pub fn simulate(
    code: &[u8],
    device: Option<DeviceBus>,
    semihost_func: Box<dyn FnMut(&SemihostingCommand) -> SemihostingResponse + 'static>,
    itm_file: Option<Box<dyn io::Write + 'static>>,
    map: Option<MemoryMapConfig>,
    flash_size: usize,
    fault_trap_mode: FaultTrapMode,
) -> Result<SimulationStatistics, SimulationError> {
    let mut processor = Processor::new();

    processor.device(device);
    processor.itm(itm_file);
    processor.semihost(Some(semihost_func));
    processor.memory_map(map);
    processor.fault_trap_mode(fault_trap_mode);
    processor.flash_memory(flash_size, code);
    //processor.ram_memory(ram_size);

    processor.cache_instructions();

    let start = Instant::now();
    processor
        .reset()
        .map_err(|fault| SimulationError::from_fault(&processor, fault))?;
    processor.running = true;

    while processor.running {
        while !processor.sleeping && processor.running {
            //running, !sleeping
            processor.step();
            if let Some(context) = processor.take_pending_fault_trap() {
                return Err(SimulationError::FaultTrap { context });
            }
        }

        while processor.sleeping && processor.running {
            //running, sleeping
            processor.step_sleep();
            if let Some(context) = processor.take_pending_fault_trap() {
                return Err(SimulationError::FaultTrap { context });
            }
        }
    }
    let end = Instant::now();

    Ok(SimulationStatistics {
        instruction_count: processor.instruction_count,
        cycle_count: processor.cycle_count,
        duration: end.duration_since(start),
        exit_code: processor.exit_code,
    })
}

///
/// Run System simulation with tracing support
///
pub fn simulate_trace<F>(
    code: &[u8],
    device: Option<DeviceBus>,
    mut trace_func: F,
    semihost_func: Box<dyn FnMut(&SemihostingCommand) -> SemihostingResponse + 'static>,
    itm_file: Option<Box<dyn io::Write + 'static>>,
    map: Option<MemoryMapConfig>,
    flash_size: usize,
    fault_trap_mode: FaultTrapMode,
) -> Result<SimulationStatistics, SimulationError>
where
    F: FnMut(&Processor),
{
    let mut processor = Processor::new();
    processor.device(device);
    processor.itm(itm_file);
    processor.semihost(Some(semihost_func));
    processor.memory_map(map);
    processor.fault_trap_mode(fault_trap_mode);
    processor.flash_memory(flash_size, code);
    processor.cache_instructions();

    let start = Instant::now();

    processor
        .reset()
        .map_err(|fault| SimulationError::from_fault(&processor, fault))?;
    processor.running = true;

    while processor.running {
        while !processor.sleeping && processor.running {
            //running, !sleeping
            processor.last_pc = processor.get_pc();
            processor.step();
            trace_func(&processor);
            if let Some(context) = processor.take_pending_fault_trap() {
                return Err(SimulationError::FaultTrap { context });
            }
        }
        processor.last_pc = processor.get_pc();
        while processor.sleeping && processor.running {
            //running, sleeping
            processor.step_sleep();
            if let Some(context) = processor.take_pending_fault_trap() {
                return Err(SimulationError::FaultTrap { context });
            }
        }
    }

    let end = Instant::now();

    Ok(SimulationStatistics {
        instruction_count: processor.instruction_count,
        cycle_count: processor.cycle_count,
        duration: end.duration_since(start),
        exit_code: processor.exit_code,
    })
}
