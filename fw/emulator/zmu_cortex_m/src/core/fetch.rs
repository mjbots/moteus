//!
//! Fetching instructions for execution
//!
//!
use crate::bus::Bus;
use crate::core::fault::{Fault, FaultStatusContext};
use crate::core::thumb::ThumbCode;

use crate::{Processor, decoder::is_thumb32};

impl Processor {
    /// Read instruction halfwords without mutating fault-status side channels.
    pub fn fetch_code(&self, pc: u32) -> Result<ThumbCode, Fault> {
        let hw = self.read16(pc).map_err(Fault::on_instruction_fetch)?;

        if is_thumb32(hw) {
            let hw2 = self.read16(pc + 2).map_err(Fault::on_instruction_fetch)?;
            Ok(ThumbCode::Thumb32 {
                opcode: (u32::from(hw) << 16) + u32::from(hw2),
            })
        } else {
            Ok(ThumbCode::Thumb16 { opcode: hw })
        }
    }
}

///
/// Fetching instructions
pub trait Fetch {
    /// Fetch instruction from current PC (Program Counter) position,
    /// decoding the possible thumb32 variant
    fn fetch(&mut self, pc: u32) -> Result<ThumbCode, Fault>;
}

impl Fetch for Processor {
    // Fetch next Thumb2-coded instruction from current
    // PC location. Depending on instruction type, fetches
    // one or two half-words.
    fn fetch(&mut self, pc: u32) -> Result<ThumbCode, Fault> {
        self.fetch_code(pc).map_err(|fault| {
            self.fault_with_status(fault, FaultStatusContext::with_fault_address(pc))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fetch_maps_unmapped_read_to_instruction_access_fault() {
        let mut processor = Processor::new();

        assert_eq!(processor.fetch(0x6000_0000), Err(Fault::IAccViol));
    }
}
