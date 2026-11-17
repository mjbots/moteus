//!
//! Instruction conditionals
//!

use std::fmt;

///
/// Condition variants used for conditional execution
///
#[derive(PartialEq, Debug, Copy, Clone)]
pub enum Condition {
    /// Equal
    EQ,
    /// Not Equal
    NE,
    /// Carry Set
    CS,
    /// Carry clear
    CC,
    /// Minus, negative
    MI,
    /// Plus, positive or zero
    PL,
    /// Overflow
    VS,
    /// No overflow
    VC,
    /// Unsigned higher
    HI,
    /// Unsigned lower or same
    LS,
    /// Signed greater than or equal
    GE,
    /// Signed less than
    LT,
    /// Signed greater than
    GT,
    /// Signed less than or equal
    LE,
    /// None or (AL = optional mnemonic extension for always)
    AL,
}

impl Condition {
    ///
    /// Condition encoding as bitvalue
    ///
    pub fn value(self) -> usize {
        match self {
            Self::EQ => 0b0000,
            Self::NE => 0b0001,
            Self::CS => 0b0010,
            Self::CC => 0b0011,
            Self::MI => 0b0100,
            Self::PL => 0b0101,
            Self::VS => 0b0110,
            Self::VC => 0b0111,
            Self::HI => 0b1000,
            Self::LS => 0b1001,
            Self::GE => 0b1010,
            Self::LT => 0b1011,
            Self::GT => 0b1100,
            Self::LE => 0b1101,
            Self::AL => 0b1110,
        }
    }

    ///
    /// bitvalue conversion to Condition
    ///
    pub fn from_u16(n: u16) -> Option<Self> {
        match n {
            0 => Some(Self::EQ),
            1 => Some(Self::NE),
            2 => Some(Self::CS),
            3 => Some(Self::CC),
            4 => Some(Self::MI),
            5 => Some(Self::PL),
            6 => Some(Self::VS),
            7 => Some(Self::VC),
            8 => Some(Self::HI),
            9 => Some(Self::LS),
            10 => Some(Self::GE),
            11 => Some(Self::LT),
            12 => Some(Self::GT),
            13 => Some(Self::LE),
            14 => Some(Self::AL),
            _ => None,
        }
    }
}

impl fmt::Display for Condition {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::EQ => write!(f, "eq"),
            Self::NE => write!(f, "ne"),
            Self::CS => write!(f, "cs"),
            Self::CC => write!(f, "cc"),
            Self::MI => write!(f, "mi"),
            Self::PL => write!(f, "pl"),
            Self::VS => write!(f, "vs"),
            Self::VC => write!(f, "vc"),
            Self::HI => write!(f, "hi"),
            Self::LS => write!(f, "ls"),
            Self::GE => write!(f, "ge"),
            Self::LT => write!(f, "lt"),
            Self::GT => write!(f, "gt"),
            Self::LE => write!(f, "le"),
            Self::AL => write!(f, ""),
        }
    }
}

/// Decode IT state byte into condition and mask string
/// Returns None if itstate is 0 (not in IT block)
pub fn decode_itstate(itstate: u8) -> Option<String> {
    if itstate == 0 {
        None
    } else {
        let cond = (itstate >> 4) & 0xF;
        let mask = itstate & 0xF;
        Condition::from_u16(u16::from(cond))
            .map(|c| format!("{}:{:x}", format!("{c}").to_uppercase(), mask))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decode_itstate_zero() {
        assert_eq!(decode_itstate(0x00), None);
    }

    #[test]
    fn test_decode_itstate_eq() {
        // EQ condition (0) with mask 4
        assert_eq!(decode_itstate(0x04), Some("EQ:4".to_string()));
    }

    #[test]
    fn test_decode_itstate_ne() {
        // NE condition (1) with mask 8
        assert_eq!(decode_itstate(0x18), Some("NE:8".to_string()));
    }

    #[test]
    fn test_decode_itstate_ge() {
        // GE condition (0xA) with mask 2
        assert_eq!(decode_itstate(0xa2), Some("GE:2".to_string()));
        // GE condition with mask 4
        assert_eq!(decode_itstate(0xa4), Some("GE:4".to_string()));
        // GE condition with mask 8
        assert_eq!(decode_itstate(0xa8), Some("GE:8".to_string()));
    }

    #[test]
    fn test_decode_itstate_lt() {
        // LT condition (0xB) with mask 8
        assert_eq!(decode_itstate(0xb8), Some("LT:8".to_string()));
    }

    #[test]
    fn test_decode_itstate_all_conditions() {
        assert_eq!(decode_itstate(0x08), Some("EQ:8".to_string()));
        assert_eq!(decode_itstate(0x18), Some("NE:8".to_string()));
        assert_eq!(decode_itstate(0x28), Some("CS:8".to_string()));
        assert_eq!(decode_itstate(0x38), Some("CC:8".to_string()));
        assert_eq!(decode_itstate(0x48), Some("MI:8".to_string()));
        assert_eq!(decode_itstate(0x58), Some("PL:8".to_string()));
        assert_eq!(decode_itstate(0x68), Some("VS:8".to_string()));
        assert_eq!(decode_itstate(0x78), Some("VC:8".to_string()));
        assert_eq!(decode_itstate(0x88), Some("HI:8".to_string()));
        assert_eq!(decode_itstate(0x98), Some("LS:8".to_string()));
        assert_eq!(decode_itstate(0xa8), Some("GE:8".to_string()));
        assert_eq!(decode_itstate(0xb8), Some("LT:8".to_string()));
        assert_eq!(decode_itstate(0xc8), Some("GT:8".to_string()));
        assert_eq!(decode_itstate(0xd8), Some("LE:8".to_string()));
        assert_eq!(decode_itstate(0xe8), Some(":8".to_string())); // AL displays as empty
    }

    #[test]
    fn test_decode_itstate_invalid_condition() {
        // Invalid condition code (0xF) should return None
        assert_eq!(decode_itstate(0xf8), None);
    }

    #[test]
    fn test_decode_itstate_all_masks() {
        // Test all possible mask values with GE condition
        assert_eq!(decode_itstate(0xa1), Some("GE:1".to_string()));
        assert_eq!(decode_itstate(0xa2), Some("GE:2".to_string()));
        assert_eq!(decode_itstate(0xa3), Some("GE:3".to_string()));
        assert_eq!(decode_itstate(0xa4), Some("GE:4".to_string()));
        assert_eq!(decode_itstate(0xa5), Some("GE:5".to_string()));
        assert_eq!(decode_itstate(0xa6), Some("GE:6".to_string()));
        assert_eq!(decode_itstate(0xa7), Some("GE:7".to_string()));
        assert_eq!(decode_itstate(0xa8), Some("GE:8".to_string()));
        assert_eq!(decode_itstate(0xa9), Some("GE:9".to_string()));
        assert_eq!(decode_itstate(0xaa), Some("GE:a".to_string()));
        assert_eq!(decode_itstate(0xab), Some("GE:b".to_string()));
        assert_eq!(decode_itstate(0xac), Some("GE:c".to_string()));
        assert_eq!(decode_itstate(0xad), Some("GE:d".to_string()));
        assert_eq!(decode_itstate(0xae), Some("GE:e".to_string()));
        assert_eq!(decode_itstate(0xaf), Some("GE:f".to_string()));
    }
}
