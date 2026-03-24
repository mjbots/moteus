// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Resolution types for register values.

/// The resolution (data type) used when encoding or decoding a register value.
///
/// Each resolution type determines:
/// - The number of bytes used on the wire
/// - The scaling applied to convert between wire format and physical units
/// - The range of representable values
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(i8)]
pub enum Resolution {
    /// 8-bit signed integer (1 byte)
    Int8 = 0,
    /// 16-bit signed integer (2 bytes)
    Int16 = 1,
    /// 32-bit signed integer (4 bytes)
    Int32 = 2,
    /// 32-bit IEEE 754 float (4 bytes)
    Float = 3,
    /// This register should be ignored/not transmitted
    #[default]
    Ignore = 4,
}

impl core::fmt::Display for Resolution {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Resolution::Int8 => write!(f, "int8"),
            Resolution::Int16 => write!(f, "int16"),
            Resolution::Int32 => write!(f, "int32"),
            Resolution::Float => write!(f, "float"),
            Resolution::Ignore => write!(f, "ignore"),
        }
    }
}

impl Resolution {
    /// Returns the size in bytes for this resolution type.
    ///
    /// Returns 0 for `Ignore`.
    #[inline]
    pub const fn size(self) -> usize {
        match self {
            Resolution::Int8 => 1,
            Resolution::Int16 => 2,
            Resolution::Int32 => 4,
            Resolution::Float => 4,
            Resolution::Ignore => 0,
        }
    }

    /// Returns the multiplex protocol type code for this resolution.
    ///
    /// This is used in the wire protocol to identify the data type.
    #[inline]
    pub const fn type_code(self) -> u8 {
        match self {
            Resolution::Int8 => 0x00,
            Resolution::Int16 => 0x04,
            Resolution::Int32 => 0x08,
            Resolution::Float => 0x0c,
            Resolution::Ignore => 0x00, // Should not be used
        }
    }

    /// Creates a Resolution from a type code.
    ///
    /// Returns `None` if the type code is invalid.
    #[inline]
    pub const fn from_type_code(code: u8) -> Option<Resolution> {
        match code & 0x0c {
            0x00 => Some(Resolution::Int8),
            0x04 => Some(Resolution::Int16),
            0x08 => Some(Resolution::Int32),
            0x0c => Some(Resolution::Float),
            _ => None,
        }
    }

    /// Returns the minimum representable value for integer types.
    ///
    /// This value is used to represent NaN in the protocol.
    #[inline]
    pub const fn nan_value(self) -> i32 {
        match self {
            Resolution::Int8 => -128,
            Resolution::Int16 => -32768,
            Resolution::Int32 => i32::MIN,
            Resolution::Float => 0, // NaN represented differently
            Resolution::Ignore => 0,
        }
    }

    /// Returns the maximum representable value for integer types.
    #[inline]
    pub const fn max_value(self) -> i32 {
        match self {
            Resolution::Int8 => 127,
            Resolution::Int16 => 32767,
            Resolution::Int32 => i32::MAX,
            Resolution::Float => 0, // Not applicable
            Resolution::Ignore => 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_resolution_size() {
        assert_eq!(Resolution::Int8.size(), 1);
        assert_eq!(Resolution::Int16.size(), 2);
        assert_eq!(Resolution::Int32.size(), 4);
        assert_eq!(Resolution::Float.size(), 4);
        assert_eq!(Resolution::Ignore.size(), 0);
    }

    #[test]
    fn test_type_code_roundtrip() {
        for res in [
            Resolution::Int8,
            Resolution::Int16,
            Resolution::Int32,
            Resolution::Float,
        ] {
            let code = res.type_code();
            assert_eq!(Resolution::from_type_code(code), Some(res));
        }
    }
}
