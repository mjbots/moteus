//!
//! Thumb-2 Representation
//!
use std::fmt;

#[derive(PartialEq, Debug, Copy, Clone)]
///
/// Either 32 bit or 16 bit thumb coded instruction
pub enum ThumbCode {
    /// 32 bit thumb coded instruction
    Thumb32 {
        /// 32 bit opcode value
        opcode: u32,
    },
    /// 16 bit thumb coded instruction
    Thumb16 {
        /// 16 bit opcode value
        opcode: u16,
    },
    /// value does not map to thumb32 or thumb16
    Undefined,
}

impl From<u16> for ThumbCode {
    fn from(value: u16) -> Self {
        Self::Thumb16 { opcode: value }
    }
}

impl From<u32> for ThumbCode {
    fn from(value: u32) -> Self {
        Self::Thumb32 { opcode: value }
    }
}

impl fmt::Display for ThumbCode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Thumb16 { opcode } => write!(f, "0x{opcode:x}"),
            Self::Thumb32 { opcode } => write!(f, "0x{opcode:x}"),
            Self::Undefined => write!(f, "Undefined"),
        }
    }
}
