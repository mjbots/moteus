//!
//! GDB Server implementation
//!
//!

use gdbstub::common::Signal;
use gdbstub::conn::Connection;
use gdbstub::conn::ConnectionExt;
use gdbstub::stub::DisconnectReason;
use gdbstub::stub::GdbStub;
use gdbstub::stub::SingleThreadStopReason;
use gdbstub::stub::run_blocking;
use gdbstub::target::Target;

use crate::DeviceBus;
use crate::MemoryMapConfig;
use crate::core::fault::{FaultContext, FaultTrapMode};
use crate::gdb::conn;
use crate::gdb::simulation::SimulationEvent;
use crate::gdb::simulation::SimulationRunEvent;
use conn::TcpConnection;

use crate::gdb::target::ZmuTarget;

use crate::semihosting::SemihostingCommand;
use crate::semihosting::SemihostingResponse;

///
/// The gdb Server
///
pub struct GdbServer {
    target: ZmuTarget,
}

#[derive(thiserror::Error, Debug)]
/// Errors that can occur in the GDB server
pub enum GdbServerError {
    /// Error related to the connection
    #[error("Connection error")]
    ConnectionError(String),
    /// Error related to the target
    #[error("Target error")]
    TargetError,
    /// A fault trap stopped the target.
    #[error("{0}")]
    FaultTrap(FaultContext),
}

impl GdbServer {
    /// Create a new GDB server.
    ///
    /// # Arguments
    ///
    /// * `code` - The binary code to run in the emulator
    /// * `semihost_func` - A function that will be called when a semihosting command is issued
    /// * `map` - The memory map configuration
    /// * `flash_size` - The size of the flash memory
    pub fn new(
        code: &[u8],
        device: Option<DeviceBus>,
        semihost_func: Box<dyn FnMut(&SemihostingCommand) -> SemihostingResponse + 'static>,
        map: Option<MemoryMapConfig>,
        flash_size: usize,
        fault_trap_mode: FaultTrapMode,
    ) -> Result<GdbServer, GdbServerError> {
        let target = ZmuTarget::new(
            code,
            device,
            semihost_func,
            map,
            flash_size,
            fault_trap_mode,
        )
        .map_err(|err| match err {
            crate::system::simulation::SimulationError::FaultTrap { context } => {
                GdbServerError::FaultTrap(context)
            }
        })?;

        Ok(GdbServer { target })
    }

    /// Start the GDB Server. This function will block until the GDB client disconnects.
    /// or program execution is complete..
    pub fn start(&mut self) -> Result<u32, GdbServerError> {
        println!("GDB Server listening on port 9001");
        let mut exit_code = 0;
        let conn = match conn::TcpConnection::new_localhost(9001) {
            Ok(conn) => conn,
            Err(e) => return Err(GdbServerError::ConnectionError(e.to_string())),
        };

        let gdb = GdbStub::new(conn);

        match gdb.run_blocking::<EventLoop>(&mut self.target) {
            Ok(disconnect_reason) => match disconnect_reason {
                DisconnectReason::Disconnect => {
                    println!("GDB client has disconnected. Running to completion...");
                    loop {
                        match self.target.step() {
                            SimulationEvent::Halted => break,
                            SimulationEvent::FaultTrap(context) => {
                                return Err(GdbServerError::FaultTrap(context));
                            }
                            SimulationEvent::Finalized(code) => {
                                exit_code = code;
                                break;
                            }
                            _ => {}
                        }
                    }
                }
                DisconnectReason::TargetExited(code) => {
                    println!("\nTarget exited with code {code}!");
                }
                DisconnectReason::TargetTerminated(sig) => {
                    println!("\nTarget terminated with signal {sig}!");
                }
                DisconnectReason::Kill => println!("\nGDB sent a kill command!"),
            },
            Err(e) => {
                if e.is_target_error() {
                    println!(
                        "target encountered a fatal error: {}",
                        e.into_target_error().unwrap()
                    );
                } else if e.is_connection_error() {
                    let (e, kind) = e.into_connection_error().unwrap();
                    println!("connection error: {kind:?} - {e}",);
                } else {
                    println!("gdbstub encountered a fatal error: {e}");
                }
            }
        }
        Ok(exit_code)
    }
}

enum EventLoop {}

impl run_blocking::BlockingEventLoop for EventLoop {
    type Target = ZmuTarget;
    type Connection = TcpConnection;
    type StopReason = SingleThreadStopReason<u32>;

    #[allow(clippy::type_complexity)]
    fn wait_for_stop_reason(
        target: &mut ZmuTarget,
        conn: &mut Self::Connection,
    ) -> Result<
        run_blocking::Event<SingleThreadStopReason<u32>>,
        run_blocking::WaitForStopReasonError<
            <Self::Target as Target>::Error,
            <Self::Connection as Connection>::Error,
        >,
    > {
        let poll_incoming_data = || {
            // gdbstub takes ownership of the underlying connection, so the `borrow_conn`
            // method is used to borrow the underlying connection back from the stub to
            // check for incoming data.
            conn.peek().map(|b| b.is_some()).unwrap_or(true)
        };

        match target.run(poll_incoming_data) {
            SimulationRunEvent::IncomingData => {
                let byte = conn
                    .read()
                    .map_err(run_blocking::WaitForStopReasonError::Connection)?;
                Ok(run_blocking::Event::IncomingData(byte))
            }
            SimulationRunEvent::Event(event) => {
                use gdbstub::target::ext::breakpoints::WatchKind;

                // translate emulator stop reason into GDB stop reason
                let stop_reason = match event {
                    SimulationEvent::DoneStep => SingleThreadStopReason::DoneStep,
                    SimulationEvent::Halted => SingleThreadStopReason::Terminated(Signal::SIGSTOP),
                    SimulationEvent::Break => SingleThreadStopReason::SwBreak(()),
                    SimulationEvent::WatchWrite(addr) => SingleThreadStopReason::Watch {
                        tid: (),
                        kind: WatchKind::Write,
                        addr,
                    },
                    SimulationEvent::WatchRead(addr) => SingleThreadStopReason::Watch {
                        tid: (),
                        kind: WatchKind::Read,
                        addr,
                    },
                    SimulationEvent::FaultTrap(_) => {
                        SingleThreadStopReason::Signal(Signal::SIGTRAP)
                    }
                    SimulationEvent::Finalized(exit_code) => {
                        SingleThreadStopReason::Exited(exit_code as u8)
                    }
                };

                Ok(run_blocking::Event::TargetStopped(stop_reason))
            }
        }
    }

    fn on_interrupt(
        _target: &mut ZmuTarget,
    ) -> Result<Option<SingleThreadStopReason<u32>>, <ZmuTarget as Target>::Error> {
        // Because this emulator runs as part of the GDB stub loop, there isn't any
        // special action that needs to be taken to interrupt the underlying target. It
        // is implicitly paused whenever the stub isn't within the
        // `wait_for_stop_reason` callback.
        Ok(Some(SingleThreadStopReason::Signal(Signal::SIGINT)))
    }
}
