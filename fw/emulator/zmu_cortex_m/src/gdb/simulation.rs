//!
//! Cortex System simulation
//!

use crate::DeviceBus;
use crate::MemoryMapConfig;
use crate::Processor;
use crate::core::fault::{FaultContext, FaultTrapMode};
use crate::core::register::BaseReg;
use crate::core::reset::Reset;
use crate::executor::Executor;
use crate::semihosting::SemihostingCommand;
use crate::semihosting::SemihostingResponse;

///
/// Cortex System simulation framework
///
pub struct Simulation {
    /// processor state
    pub processor: Processor,
    /// Simulation Execution mode
    pub exec_mode: SimulationExecMode,
    /// Watchpoints
    pub watchpoints: Vec<u32>,
    /// Breakpoints
    pub breakpoints: Vec<u32>,
}

///
/// A simulation event.
///
pub enum SimulationEvent {
    /// Step done
    DoneStep,
    /// Processor is halted
    #[allow(dead_code)]
    Halted,
    /// A breakpoint was hit
    Break,
    /// A write watchpoint was hit
    #[allow(dead_code)]
    WatchWrite(u32),
    /// A read watchpoint was hit
    WatchRead(u32),
    /// Execution stopped because a fault trap was hit.
    FaultTrap(FaultContext),
    /// Simulation is finalized
    Finalized(u32),
}

///
/// Mode of execution for the simulation
///
pub enum SimulationExecMode {
    /// Step by Step execution (Single Step)
    Step,
    /// Continuous execution
    Continue,
    /// Range Step execution (from start to end)
    RangeStep(u32, u32),
}

impl Simulation {
    ///
    /// Prepare a new simulation instance
    ///
    pub fn new(
        code: &[u8],
        device: Option<DeviceBus>,
        semihost_func: Box<dyn FnMut(&SemihostingCommand) -> SemihostingResponse + 'static>,
        map: Option<MemoryMapConfig>,
        flash_size: usize,
        fault_trap_mode: FaultTrapMode,
    ) -> Result<Simulation, crate::system::simulation::SimulationError> {
        let mut processor = Processor::new();
        processor.device(device);
        processor.semihost(Some(semihost_func));
        processor.memory_map(map);
        processor.fault_trap_mode(fault_trap_mode);
        processor.flash_memory(flash_size, code);
        processor.cache_instructions();
        processor.running = true;
        match processor.reset() {
            Ok(()) => {}
            Err(fault) => {
                return Err(crate::system::simulation::SimulationError::from_fault(
                    &processor, fault,
                ));
            }
        }

        Ok(Simulation {
            processor,
            exec_mode: SimulationExecMode::Step,
            watchpoints: Vec::new(),
            breakpoints: Vec::new(),
        })
    }

    /// Run the simulation. This function will block until the simulation is complete.
    pub fn run(&mut self, mut poll_incoming_data: impl FnMut() -> bool) -> SimulationRunEvent {
        match self.exec_mode {
            SimulationExecMode::Step => SimulationRunEvent::Event(self.step()),
            SimulationExecMode::Continue => {
                let mut cycles = 0;
                loop {
                    // The poll_incoming_data function checks if there is any data available
                    // to be read from the connection to the GDB server.
                    // Currently, each call to poll_incoming_data blocks until either data
                    // is available or a timeout occurs. This provides a simple mechanism
                    // to detect if the GDB client has sent any commands to the server.
                    // However, this approach is inefficient because the simulation is
                    // effectively stalled until either the GDB client sends data or the
                    // timeout expires.
                    // To reduce the performance impact, we only poll for incoming data
                    // every 1024 cycles.
                    if cycles % 1024 == 0 && poll_incoming_data() {
                        return SimulationRunEvent::IncomingData;
                    }
                    cycles += 1;
                    let evt = self.step();
                    match evt {
                        SimulationEvent::DoneStep => {}
                        _ => return SimulationRunEvent::Event(evt),
                    }
                }
            }
            SimulationExecMode::RangeStep(start, end) => {
                let mut cycles = 0;
                loop {
                    if cycles % 1024 == 0 && poll_incoming_data() {
                        return SimulationRunEvent::IncomingData;
                    }
                    cycles += 1;

                    let evt = self.step();

                    match evt {
                        SimulationEvent::DoneStep => {
                            if !(start..end).contains(&self.processor.get_pc()) {
                                return SimulationRunEvent::Event(evt);
                            }
                        }
                        _ => return SimulationRunEvent::Event(evt),
                    }
                }
            }
        }
    }

    ///
    /// Reset the simulation
    ///
    pub fn reset(&mut self) -> Result<(), crate::system::simulation::SimulationError> {
        match self.processor.reset() {
            Ok(()) => Ok(()),
            Err(fault) => Err(crate::system::simulation::SimulationError::from_fault(
                &self.processor,
                fault,
            )),
        }
    }

    ///
    /// Single step the simulation
    ///
    pub fn step(&mut self) -> SimulationEvent {
        if self.processor.running {
            self.processor.step();
        } else if self.processor.sleeping {
            self.processor.step_sleep();
        }
        if let Some(context) = self.processor.take_pending_fault_trap() {
            return SimulationEvent::FaultTrap(context);
        }
        if self.breakpoints.contains(&self.processor.get_pc()) {
            return SimulationEvent::Break;
        }
        if self.watchpoints.contains(&self.processor.get_pc()) {
            return SimulationEvent::WatchRead(self.processor.get_pc());
        }
        if !self.processor.running && !self.processor.sleeping {
            SimulationEvent::Finalized(self.processor.exit_code)
        } else {
            SimulationEvent::DoneStep
        }
    }
}

///
/// A simulation run event.
///
pub enum SimulationRunEvent {
    /// Incoming data from the GDB server
    IncomingData,
    /// A simulation event
    Event(SimulationEvent),
}
