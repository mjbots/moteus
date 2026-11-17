//!
//! Cortex Memory Protection Unit simulation
//!

use crate::Processor;
use crate::core::fault::Fault;

/// Memory access type for MPU validation.
#[derive(Copy, Clone)]
pub enum AccType {
    /// Privileged data access.
    Normal,
    /// Unprivileged data access.
    UnPriv,
}

/// Register API for MPU address validation.
pub trait Mpu {
    ///
    /// Validate memory access at `address`.
    ///
    fn validate_address(
        &mut self,
        address: u32,
        acctype: AccType,
        write: bool,
    ) -> Result<u32, Fault>;
}

impl Mpu for Processor {
    fn validate_address(
        &mut self,
        address: u32,
        _acctype: AccType,
        _write: bool,
    ) -> Result<u32, Fault> {
        // TODO MPU check
        Ok(address)
    }
}

#[cfg(test)]
mod tests {
    use super::{AccType, Mpu};
    use crate::Processor;

    #[test]
    fn test_validate_address_returns_input_for_privileged_and_unprivileged_access() {
        let mut processor = Processor::new();

        assert_eq!(
            processor
                .validate_address(0x2000_0100, AccType::Normal, false)
                .unwrap(),
            0x2000_0100
        );
        assert_eq!(
            processor
                .validate_address(0x2000_0200, AccType::UnPriv, true)
                .unwrap(),
            0x2000_0200
        );
    }
}
