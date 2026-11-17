//!
//! Cortex Debug and Trace unit simulation
//!

use crate::Processor;
use crate::core::bits::Bits;

/// Register API to Debug and Trace peripheral
pub trait Dwt {
    ///
    /// write ctrl register value
    ///
    fn dwt_write_ctrl(&mut self, value: u32);

    ///
    /// write cycle counter value
    ///
    fn dwt_write_cyccnt(&mut self, value: u32);

    ///
    /// Clock dwt block ```cycles```.
    ///
    ///
    fn dwt_tick(&mut self, cycles: u32);
}

const DWT_CTRL_CYCCNTENA: u32 = 1;

impl Dwt for Processor {
    fn dwt_write_ctrl(&mut self, value: u32) {
        self.dwt_ctrl.set_bits(16..23, value.get_bits(16..23));
        self.dwt_ctrl.set_bits(0..13, value.get_bits(0..13));
    }

    fn dwt_write_cyccnt(&mut self, value: u32) {
        self.dwt_cyccnt = value;
    }

    #[inline(always)]
    fn dwt_tick(&mut self, cycles: u32) {
        self.dwt_cyccnt = self
            .dwt_cyccnt
            .wrapping_add(cycles * (self.dwt_ctrl & DWT_CTRL_CYCCNTENA));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::reset::Reset;

    #[test]
    fn test_dwt_tick() {
        // Arrange

        let mut processor = Processor::new();

        // Arrange
        processor.reset().unwrap();
        processor.dwt_write_cyccnt(0);

        // Act
        processor.dwt_write_ctrl(DWT_CTRL_CYCCNTENA);
        processor.dwt_tick(42);

        // Act
        assert_eq!(processor.dwt_cyccnt, 42);
    }
}
