//!
//! Cortex Nested Vectored Interrupt Controller simulation
//!

use crate::Processor;
use crate::core::bits::Bits;
use crate::core::exception::Exception;
use crate::core::exception::ExceptionHandling;
use Exception::Interrupt;

///
/// Register API for NVIC
///
pub trait NVIC {
    ///
    /// Write Interrupt Set Enable
    ///
    fn nvic_write_iser(&mut self, index: usize, value: u32);

    ///
    /// Read Interrupt Set Enable
    ///
    fn nvic_read_iser(&self, index: usize) -> u32;

    ///
    /// Write Interrupt Clear Enable
    ///
    fn nvic_write_icer(&mut self, index: usize, value: u32);

    ///
    /// Read Interrupt Clear Enable
    ///
    fn nvic_read_icer(&self, index: usize) -> u32;

    ///
    /// Write interrupt Set Pending
    ///
    fn nvic_write_ispr(&mut self, index: usize, value: u32);

    ///
    /// Read interrupt Set Pending
    ///
    fn nvic_read_ispr(&self, index: usize) -> u32;

    ///
    /// Write interrupt Clear Pending
    ///
    fn nvic_write_icpr(&mut self, index: usize, value: u32);

    ///
    /// Read interrupt Clear Pending
    ///
    fn nvic_read_icpr(&self, index: usize) -> u32;

    ///
    /// Read Interrupt Active Bit Register.
    /// ```index``` is the 32 bit set of irqs to list. Value 0 means irqs 0..=31.
    ///
    fn nvic_read_iabr(&self, index: usize) -> u32;

    ///
    /// 32bit write to interrupt priority register
    ///
    fn nvic_write_ipr(&mut self, index: usize, value: u32);

    ///
    /// 32bit read from interrupt priority register
    ///
    fn nvic_read_ipr(&self, index: usize) -> u32;

    ///
    /// 8bit write to interrupt priority register
    ///
    fn nvic_write_ipr_u8(&mut self, index: usize, value: u8);

    ///
    /// 8bit read from interrupt priority register
    ///
    fn nvic_read_ipr_u8(&self, index: usize) -> u8;

    ///
    /// 16 bit write to interrupt priority register
    ///
    fn nvic_write_ipr_u16(&mut self, index: usize, value: u16);

    ///
    /// 16 bit read from interrupt priority register
    ///
    fn nvic_read_ipr_u16(&self, index: usize) -> u16;

    ///
    /// Mark interrupt no longer pending in NVIC point of view.
    ///
    fn nvic_unpend_interrupt(&mut self, irqn: usize);
}

trait NVICHelper {
    fn nvic_set_pending_exceptions(&mut self, index: usize);
    fn nvic_clear_unpended_exceptions(&mut self, index: usize);
}

fn set_bits_array(array: &mut [u32; 16], index: usize, value: u32) {
    if index == 15 {
        array[index] |= value & 0xffff;
    } else {
        array[index] |= value;
    }
}

fn clear_bits_array(array: &mut [u32; 16], index: usize, value: u32) {
    if index == 15 {
        array[index] &= (value & 0xffff) ^ 0xFFFF_FFFF;
    } else {
        array[index] &= value ^ 0xFFFF_FFFF;
    }
}

impl NVICHelper for Processor {
    fn nvic_set_pending_exceptions(&mut self, index: usize) {
        let mut active = self.nvic_interrupt_pending[index] & self.nvic_interrupt_enabled[index];
        let mut irqn = index * 4;
        while active != 0 {
            if active & 1 != 0 {
                self.set_exception_pending(Interrupt { n: irqn });
            }
            active >>= 1;
            irqn += 1;
        }
    }

    fn nvic_clear_unpended_exceptions(&mut self, index: usize) {
        let mut active = self.nvic_interrupt_pending[index] & self.nvic_interrupt_enabled[index];
        for irqn in (index * 4)..(index * 4) + 32 {
            if active & 1 == 0 {
                self.clear_pending_exception(Interrupt { n: irqn });
            }
            active >>= 1;
        }
    }
}

impl NVIC for Processor {
    fn nvic_write_iser(&mut self, index: usize, value: u32) {
        set_bits_array(&mut self.nvic_interrupt_enabled, index, value);
        self.nvic_set_pending_exceptions(index);
    }

    fn nvic_read_iser(&self, index: usize) -> u32 {
        self.nvic_interrupt_enabled[index]
    }

    fn nvic_write_icer(&mut self, index: usize, value: u32) {
        clear_bits_array(&mut self.nvic_interrupt_enabled, index, value);
        self.nvic_clear_unpended_exceptions(index);
    }

    fn nvic_unpend_interrupt(&mut self, irqn: usize) {
        let index = irqn / 32;
        let bit = irqn % 32;
        clear_bits_array(&mut self.nvic_interrupt_pending, index, 1 << bit);
    }

    fn nvic_read_icer(&self, index: usize) -> u32 {
        self.nvic_interrupt_enabled[index] ^ 0xFFFF_FFFF
    }

    fn nvic_write_ispr(&mut self, index: usize, value: u32) {
        set_bits_array(&mut self.nvic_interrupt_pending, index, value);
        self.nvic_set_pending_exceptions(index);
    }

    fn nvic_read_ispr(&self, index: usize) -> u32 {
        self.nvic_interrupt_pending[index]
    }

    fn nvic_write_icpr(&mut self, index: usize, value: u32) {
        clear_bits_array(&mut self.nvic_interrupt_pending, index, value);
        self.nvic_clear_unpended_exceptions(index);
    }

    fn nvic_read_icpr(&self, index: usize) -> u32 {
        self.nvic_interrupt_pending[index] ^ 0xFFFF_FFFF
    }

    fn nvic_read_iabr(&self, index: usize) -> u32 {
        let first_irqn = index * 32;
        let mut active = 0;
        let mut mask = 1;
        for irqn in first_irqn..first_irqn + 32 {
            if self.exception_active(Interrupt { n: irqn }) {
                active |= mask;
            }
            mask <<= 1;
        }
        active
    }

    fn nvic_write_ipr(&mut self, index: usize, value: u32) {
        self.nvic_write_ipr_u8(index * 4, value.get_bits(0..8) as u8);
        self.nvic_write_ipr_u8((index * 4) + 1, value.get_bits(8..16) as u8);
        self.nvic_write_ipr_u8((index * 4) + 2, value.get_bits(16..24) as u8);
        self.nvic_write_ipr_u8((index * 4) + 3, value.get_bits(24..32) as u8);
    }

    fn nvic_write_ipr_u16(&mut self, index: usize, value: u16) {
        self.nvic_write_ipr_u8(index * 2, value.get_bits(0..8) as u8);
        self.nvic_write_ipr_u8((index * 2) + 1, value.get_bits(8..16) as u8);
    }

    fn nvic_read_ipr(&self, index: usize) -> u32 {
        u32::from(self.nvic_read_ipr_u8(index * 4))
            + (u32::from(self.nvic_read_ipr_u8((index * 4) + 1)) << 8)
            + (u32::from(self.nvic_read_ipr_u8((index * 4) + 2)) << 16)
            + (u32::from(self.nvic_read_ipr_u8((index * 4) + 3)) << 24)
    }

    fn nvic_read_ipr_u16(&self, index: usize) -> u16 {
        u16::from(self.nvic_read_ipr_u8(index * 2))
            + (u16::from(self.nvic_read_ipr_u8((index * 2) + 1)) << 8)
    }

    fn nvic_read_ipr_u8(&self, index: usize) -> u8 {
        let priority = self.get_exception_priority(Interrupt { n: index });
        assert!((0..256).contains(&priority));
        priority as u8
    }

    fn nvic_write_ipr_u8(&mut self, index: usize, value: u8) {
        self.set_exception_priority(Interrupt { n: index }, value);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::exception::ExceptionHandling;
    use crate::core::instruction::Instruction;
    use crate::core::reset::Reset;
    use crate::executor::Executor;

    #[test]
    fn test_nvic_iser_icer() {
        // Arrange
        let mut processor = Processor::new();

        assert_eq!(processor.nvic_read_iser(0), 0);
        assert_eq!(processor.nvic_read_icer(0), 0xffff_ffff);

        // Act
        processor.nvic_write_iser(0, 0xffff_ffff);

        // Assert
        assert_eq!(processor.nvic_read_iser(0), 0xffff_ffff);
        assert_eq!(processor.nvic_read_icer(0), 0);

        // Act
        processor.nvic_write_iser(0, 0);

        // Assert
        assert_eq!(processor.nvic_read_iser(0), 0xffff_ffff);
        assert_eq!(processor.nvic_read_icer(0), 0);

        // Act
        processor.nvic_write_icer(0, 0);

        // Assert
        assert_eq!(processor.nvic_read_iser(0), 0xffff_ffff);
        assert_eq!(processor.nvic_read_icer(0), 0);

        // Act
        processor.nvic_write_icer(0, 0xffff_ffff);

        // Assert
        assert_eq!(processor.nvic_read_iser(0), 0);
        assert_eq!(processor.nvic_read_icer(0), 0xffff_ffff);
    }

    #[test]
    fn test_nvic_ispr_icpr() {
        // Arrange
        let mut processor = Processor::new();

        assert_eq!(processor.nvic_read_ispr(0), 0);
        assert_eq!(processor.nvic_read_icpr(0), 0xffff_ffff);

        // Act
        processor.nvic_write_ispr(0, 0xffff_ffff);

        // Assert
        assert_eq!(processor.nvic_read_ispr(0), 0xffff_ffff);
        assert_eq!(processor.nvic_read_icpr(0), 0);

        // Act
        processor.nvic_write_ispr(0, 0);

        // Assert
        assert_eq!(processor.nvic_read_ispr(0), 0xffff_ffff);
        assert_eq!(processor.nvic_read_icpr(0), 0);

        // Act
        processor.nvic_write_icpr(0, 0);

        // Assert
        assert_eq!(processor.nvic_read_ispr(0), 0xffff_ffff);
        assert_eq!(processor.nvic_read_icpr(0), 0);

        // Act
        processor.nvic_write_icpr(0, 0xffff_ffff);

        // Assert
        assert_eq!(processor.nvic_read_ispr(0), 0);
        assert_eq!(processor.nvic_read_icpr(0), 0xffff_ffff);
    }

    #[test]
    fn test_nvic_iabr() {
        // Arrange

        let mut data = vec![0; 65536].into_boxed_slice();
        data[3] = 0x20; // stack pointer
        data[0] = 0xff;

        let mut processor = Processor::new();

        processor.flash_memory(65536, &data);

        processor.reset().unwrap();

        // Act
        processor.nvic_write_ispr(0, 1);
        processor.nvic_write_iser(0, 1);

        processor.execute(&Instruction::NOP { thumb32: false }, 2);
        processor.check_exceptions();

        // Assert
        assert_eq!(processor.nvic_read_iabr(0), 1);
    }

    #[test]
    fn test_nvic_ipr() {
        // Arrange

        let mut processor = Processor::new();

        processor.reset().unwrap();

        // Act
        processor.nvic_write_ipr(0, 0xaabb_ccdd);

        // Assert
        assert_eq!(
            processor.get_exception_priority(Exception::Interrupt { n: 0 }),
            0xdd
        );
        assert_eq!(
            processor.get_exception_priority(Exception::Interrupt { n: 1 }),
            0xcc
        );
        assert_eq!(
            processor.get_exception_priority(Exception::Interrupt { n: 2 }),
            0xbb
        );
        assert_eq!(
            processor.get_exception_priority(Exception::Interrupt { n: 3 }),
            0xaa
        );
        assert_eq!(processor.nvic_read_ipr(0), 0xaabb_ccdd);

        // Act
        processor.nvic_write_ipr(1, 0x1234_5678);

        // Assert
        assert_eq!(
            processor.get_exception_priority(Exception::Interrupt { n: 4 }),
            0x78
        );
        assert_eq!(
            processor.get_exception_priority(Exception::Interrupt { n: 5 }),
            0x56
        );
        assert_eq!(
            processor.get_exception_priority(Exception::Interrupt { n: 6 }),
            0x34
        );
        assert_eq!(
            processor.get_exception_priority(Exception::Interrupt { n: 7 }),
            0x12
        );
        assert_eq!(processor.nvic_read_ipr(0), 0xaabb_ccdd);
        assert_eq!(processor.nvic_read_ipr(1), 0x1234_5678);
    }

    #[test]
    fn test_nvic_ipr_u8() {
        // Arrange

        let mut processor = Processor::new();

        processor.reset().unwrap();

        // Act

        for n in 0..32 {
            processor.nvic_write_ipr_u8(n, n as u8);
        }

        // Assert
        for n in 0..32 {
            assert_eq!(
                processor.get_exception_priority(Exception::Interrupt { n }),
                n as i16
            );
            assert_eq!(processor.nvic_read_ipr_u8(n), n as u8);
        }
    }

    #[test]
    fn test_nvic_ipr_u16() {
        // Arrange

        let mut processor = Processor::new();

        processor.reset().unwrap();

        // Act

        for n in 0..16 {
            let value = ((((n * 2) + 1) as u16) << 8) + ((n * 2) as u16);
            processor.nvic_write_ipr_u16(n, value);
        }

        // Assert
        for n in 0..32 {
            assert_eq!(
                processor.get_exception_priority(Exception::Interrupt { n }),
                n as i16
            );
        }

        for n in 0..16 {
            let value = ((((n * 2) + 1) as u16) << 8) + ((n * 2) as u16);
            assert_eq!(processor.nvic_read_ipr_u16(n), value);
        }
    }
}
