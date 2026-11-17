//!
//! Cortex Semihosting simulation
//!

use crate::Processor;
use crate::bus::Bus;
use crate::core::fault::Fault;
use crate::core::register::BaseReg;
use crate::core::register::Reg;

#[derive(PartialEq, Debug, Copy, Clone)]
#[allow(missing_docs)]
pub enum SysExceptionReason {
    ADPStoppedBranchThroughZero,
    ADPStoppedUndefinedInstr,
    ADPStoppedSoftwareInterrupt,
    ADPStoppedPrefetchAbort,
    ADPStoppedDataAbort,
    ADPStoppedAddressException,
    ADPStoppedIRQ,
    ADPStoppedFIQ,
    ADPStoppedBreakPoint,
    ADPStoppedWatchPoint,
    ADPStoppedStepComplete,
    ADPStoppedRunTimeErrorUnknown,
    ADPStoppedInternalError,
    ADPStoppedUserInterruption,
    ADPStoppedApplicationExit,
    ADPStoppedStackOverflow,
    ADPStoppedDivisionByZero,
    ADPStoppedOSSpecific,
    ADPStopped,
}

impl SysExceptionReason {
    ///
    /// Convert reason code to reason enum value
    ///
    pub fn from_u32(reason: u32) -> Self {
        match reason {
            0x20000 => Self::ADPStoppedBranchThroughZero,
            0x20001 => Self::ADPStoppedUndefinedInstr,
            0x20002 => Self::ADPStoppedSoftwareInterrupt,
            0x20003 => Self::ADPStoppedPrefetchAbort,
            0x20004 => Self::ADPStoppedDataAbort,
            0x20005 => Self::ADPStoppedAddressException,
            0x20006 => Self::ADPStoppedIRQ,
            0x20007 => Self::ADPStoppedFIQ,
            0x20020 => Self::ADPStoppedBreakPoint,
            0x20021 => Self::ADPStoppedWatchPoint,
            0x20022 => Self::ADPStoppedStepComplete,
            0x20023 => Self::ADPStoppedRunTimeErrorUnknown,
            0x20024 => Self::ADPStoppedInternalError,
            0x20025 => Self::ADPStoppedUserInterruption,
            0x20026 => Self::ADPStoppedApplicationExit,
            0x20027 => Self::ADPStoppedStackOverflow,
            0x20028 => Self::ADPStoppedDivisionByZero,
            0x20029 => Self::ADPStoppedOSSpecific,
            _ => Self::ADPStopped,
        }
    }
}

#[derive(PartialEq, Debug)]
///
/// Semihosting commands
///
pub enum SemihostingCommand {
    ///
    /// Open a file handle
    ///
    SysOpen {
        /// name of the handle
        name: String,
        /// opening mode
        mode: u32,
    },
    ///
    /// Close a file handle
    ///
    SysClose {
        /// handle to close
        handle: u32,
    },
    ///
    /// Seek a file handle
    ///
    SysSeek {
        /// handle to seek
        handle: u32,
        /// position to seek
        position: u32,
    },
    ///
    /// Get a length of file by handle
    ///
    SysFlen {
        /// handle for which the length is calculated
        handle: u32,
    },
    ///
    /// Check if file handle is interactive tty console
    ///
    SysIstty {
        /// handle for which the property is checked for
        handle: u32,
    },
    ///
    /// Write single char to the debug console
    ///
    SysWriteC {
        /// char to write
        data: u8,
    },
    ///
    /// Write data to open file handle
    ///
    SysWrite {
        ///
        /// Handle to which the data is written to
        ///
        handle: u32,
        ///
        /// data to be writtemn
        ///
        data: Vec<u8>,
    },
    ///
    /// Read data from open file handle
    ///
    SysRead {
        ///
        /// handle from which to read the data
        ///
        handle: u32,
        ///
        /// location in memory to which the data is read
        ///
        memoryptr: u32,
        ///
        /// length of read in bytes
        ///
        len: u32,
    },
    ///
    /// Trigger an exception
    ///
    SysException {
        /// reason code for the exception
        reason: SysExceptionReason,
    },
    ///
    /// perform system exit
    ///
    SysExitExtended {
        /// reason code for the exit
        reason: SysExceptionReason,
        /// subcode of the exit, dependant of the reason
        subcode: u32,
    },
    ///
    /// Get the value of sysclock
    ///
    SysClock,
    ///
    /// Get the value of errno
    ///
    SysErrno,
}

#[derive(PartialEq, Debug, Clone)]
///
/// Responses for the semihosting commands
///
pub enum SemihostingResponse {
    /// open command response
    SysOpen {
        /// result Ok(handle), error code
        result: Result<u32, i32>,
    },
    /// close command response
    SysClose {
        /// result
        success: bool,
    },
    /// flen command response
    SysFlen {
        /// result Ok(file length), Err = errorcode
        result: Result<u32, i32>,
    },
    /// istty command response
    SysIstty {
        /// result Ok(istty), Err = errorcode
        result: Result<u32, i32>,
    },
    /// seek command response
    SysSeek {
        /// result
        success: bool,
    },
    /// syswrite command response
    SysWrite {
        /// result Ok = bytes written, Err = error code
        result: Result<u32, i32>,
    },
    /// sysread command response
    SysRead {
        /// result Ok = data, Err = error code
        result: Result<(u32, Vec<u8>, u32), i32>,
    },
    /// sysexception command response
    SysException {
        /// result
        success: bool,
        /// system is stopping
        stop: bool,
    },
    /// sysexitextended command response
    SysExitExtended {
        /// result
        success: bool,
        /// system is stopping
        stop: bool,
        /// subcode of the exit, dependant of the reason
        exit_code: Option<u32>,
    },
    /// sysclock command response
    SysClock {
        /// result Ok = value, Err = error code
        result: Result<u32, i32>,
    },
    /// syserrno command response
    SysErrno {
        /// result
        result: u32,
    },
}

const SYS_OPEN: u32 = 0x01;
const SYS_CLOSE: u32 = 0x02;
const SYS_WRITEC: u32 = 0x03;
const SYS_WRITE: u32 = 0x05;
const SYS_READ: u32 = 0x06;
const SYS_ISTTY: u32 = 0x09;
const SYS_SEEK: u32 = 0x0a;
const SYS_FLEN: u32 = 0x0c;
const SYS_CLOCK: u32 = 0x10;
const SYS_ERRNO: u32 = 0x13;
const SYS_EXIT: u32 = 0x18;
const SYS_EXIT_EXTENDED: u32 = 0x20;

///
/// Decode semihosting command based on register values
///
pub fn decode_semihostcmd(
    r0: u32,
    r1: u32,
    processor: &mut Processor,
) -> Result<SemihostingCommand, Fault> {
    let result = match r0 {
        SYS_OPEN => {
            let argument_block = r1;

            let mut string_ptr = processor.read32(argument_block)?;
            let mode = processor.read32(argument_block + 4)?;
            let mut filename_len = processor.read32(argument_block + 8)?;
            let mut string_bytes: Vec<u8> = Vec::new();

            while filename_len > 0 {
                string_bytes.push(processor.read8(string_ptr)?);
                string_ptr += 1;
                filename_len -= 1;
            }

            SemihostingCommand::SysOpen {
                name: String::from_utf8(string_bytes).unwrap(),
                mode,
            }
        }
        SYS_CLOSE => {
            let params_ptr = r1;
            let handle = processor.read32(params_ptr)?;
            SemihostingCommand::SysClose { handle }
        }
        SYS_WRITEC => {
            let params_ptr = r1;
            let ch = processor.read8(params_ptr)?;
            SemihostingCommand::SysWriteC { data: ch }
        }
        SYS_WRITE => {
            let params_ptr = r1;
            let handle = processor.read32(params_ptr)?;
            let mut memoryptr = processor.read32(params_ptr + 4)?;
            let mut len = processor.read32(params_ptr + 8)?;

            let mut data: Vec<u8> = Vec::new();

            // :tt console output
            while len > 0 {
                data.push(processor.read8(memoryptr)?);
                memoryptr += 1;
                len -= 1;
            }
            SemihostingCommand::SysWrite { handle, data }
        }
        SYS_READ => {
            let params_ptr = r1;
            let handle = processor.read32(params_ptr)?;
            let memoryptr = processor.read32(params_ptr + 4)?;
            let len = processor.read32(params_ptr + 8)?;

            SemihostingCommand::SysRead {
                handle,
                memoryptr,
                len,
            }
        }
        SYS_FLEN => {
            let params_ptr = r1;
            let handle = processor.read32(params_ptr)?;

            SemihostingCommand::SysFlen { handle }
        }
        SYS_ISTTY => {
            let params_ptr = r1;
            let handle = processor.read32(params_ptr)?;

            SemihostingCommand::SysIstty { handle }
        }
        SYS_SEEK => {
            let params_ptr = r1;
            let handle = processor.read32(params_ptr)?;
            let position = processor.read32(params_ptr + 4)?;

            SemihostingCommand::SysSeek { handle, position }
        }
        SYS_CLOCK => SemihostingCommand::SysClock,
        SYS_ERRNO => SemihostingCommand::SysErrno,
        SYS_EXIT_EXTENDED => {
            let params_ptr = r1;

            // see: https://github.com/ARM-software/abi-aa/blob/main/semihosting/semihosting.rst#sys-exit-extended-0x20

            // read field 1 and field 2
            let reason = SysExceptionReason::from_u32(processor.read32(params_ptr)?);
            let subcode = processor.read32(params_ptr + 4)?;

            SemihostingCommand::SysExitExtended { reason, subcode }
        }
        SYS_EXIT => SemihostingCommand::SysException {
            reason: SysExceptionReason::from_u32(r1),
        },
        _ => {
            todo!("unknown semihosting command {}", r0);
        }
    };
    Ok(result)
}

#[allow(unused)]
///
/// Handle semihosting response received from semihosting implementation
///
pub fn semihost_return(processor: &mut Processor, response: &SemihostingResponse) {
    match *response {
        SemihostingResponse::SysOpen { result } => match result {
            Ok(handle) => processor.set_r(Reg::R0, handle),
            Err(error_code) => processor.set_r(Reg::R0, error_code as u32),
        },
        SemihostingResponse::SysFlen { result } => match result {
            Ok(size) => processor.set_r(Reg::R0, size),
            Err(error_code) => processor.set_r(Reg::R0, error_code as u32),
        },
        SemihostingResponse::SysIstty { result } => match result {
            Ok(response) => processor.set_r(Reg::R0, response),
            Err(error_code) => processor.set_r(Reg::R0, error_code as u32),
        },
        SemihostingResponse::SysException { success, stop } => {
            if success && stop {
                processor.running = false;
            }
        }
        SemihostingResponse::SysExitExtended {
            success,
            stop,
            exit_code,
        } => {
            if success && stop {
                processor.running = false;
                if let Some(exit_code) = exit_code {
                    processor.exit_code = exit_code;
                }
            }
        }
        SemihostingResponse::SysClose { success } | SemihostingResponse::SysSeek { success } => {
            if success {
                processor.set_r(Reg::R0, 0);
            } else {
                processor.set_r(Reg::R0, (-1_i32) as u32);
            }
        }
        SemihostingResponse::SysWrite { result } => match result {
            Ok(_) => processor.set_r(Reg::R0, 0),
            Err(unwritten_bytes) => processor.set_r(Reg::R0, unwritten_bytes as u32),
        },
        SemihostingResponse::SysRead { ref result } => match result {
            Ok((memoryptr, data, diff)) => {
                let mut addr = *memoryptr;
                for x in data {
                    processor.write8(addr, *x);
                    addr += 1;
                }
                processor.set_r(Reg::R0, *diff);
            }
            Err(error_code) => processor.set_r(Reg::R0, *error_code as u32),
        },
        SemihostingResponse::SysClock { result } => match result {
            Ok(centiseconds) => processor.set_r(Reg::R0, centiseconds),
            Err(error_code) => processor.set_r(Reg::R0, error_code as u32),
        },
        SemihostingResponse::SysErrno { result } => {
            processor.set_r(Reg::R0, result);
        }
    }
}
