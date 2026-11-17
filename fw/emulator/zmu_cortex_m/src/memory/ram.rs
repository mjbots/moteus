//!
//! RAM simulation
//!
//!

use crate::bus::Bus;
use crate::core::fault::Fault;

#[derive(Debug)]
/// RAM memory with configurable start address
pub struct RAM {
    start_address: u32,
    data: Box<[u8]>,
}

impl RAM {
    /// Create RAM memory data struct with configurable start address and size
    pub fn new(start_address: u32, size: usize) -> Self {
        let data = vec![0_u8; size].into_boxed_slice();

        Self {
            start_address,
            data,
        }
    }
    ///
    pub fn new_with_fill(start_address: u32, size: usize, fill: u8) -> Self {
        let data = vec![fill; size].into_boxed_slice();

        Self {
            start_address,
            data,
        }
    }
}

impl Bus for RAM {
    #[inline(always)]
    fn read8(&self, addr: u32) -> Result<u8, Fault> {
        let a = addr - self.start_address;
        Ok(self.data[a as usize])
    }

    #[inline(always)]
    fn read16(&self, addr: u32) -> Result<u16, Fault> {
        let a = (addr - self.start_address) as usize;

        Ok(u16::from_le_bytes([self.data[a], self.data[a + 1]]))
    }

    #[inline(always)]
    fn read32(&mut self, addr: u32) -> Result<u32, Fault> {
        let a = (addr - self.start_address) as usize;
        Ok(u32::from_le_bytes([
            self.data[a],
            self.data[a + 1],
            self.data[a + 2],
            self.data[a + 3],
        ]))
    }

    #[inline(always)]
    fn write8(&mut self, addr: u32, value: u8) -> Result<(), Fault> {
        let a = addr - self.start_address;
        self.data[a as usize] = value;
        Ok(())
    }

    #[inline(always)]
    fn write16(&mut self, addr: u32, value: u16) -> Result<(), Fault> {
        let a = (addr - self.start_address) as usize;

        self.data[a..a + 2].copy_from_slice(&value.to_le_bytes());
        Ok(())
    }

    #[inline(always)]
    fn write32(&mut self, addr: u32, value: u32) -> Result<(), Fault> {
        let a = (addr - self.start_address) as usize;
        self.data[a..a + 4].copy_from_slice(&value.to_le_bytes());
        Ok(())
    }

    #[inline(always)]
    fn in_range(&self, addr: u32) -> bool {
        addr.wrapping_sub(self.start_address) < self.data.len() as u32
    }
}

#[test]
fn test_new() {
    // should be able to make new instance of memory
    let _mem = RAM::new(0, 1024);
}

#[test]
fn test_in_range() {
    {
        /* no offset */
        let mem = RAM::new(0, 1024);
        assert!(mem.in_range(0));
        assert!(mem.in_range(1023));
        assert!(!mem.in_range(1024));
        assert!(!mem.in_range(0xFFFF_FFFF));
    }

    {
        /* offset of 0x8000_0000 */
        let mem = RAM::new(0x8000_0000, 1024);
        assert!(mem.in_range(0x8000_0000));
        assert!(mem.in_range(0x8000_0001));
        assert!(!mem.in_range(0x8000_0000 + 1024));
        assert!(!mem.in_range(0x8000_0000 + 0xffff));
    }
}

#[test]
fn test_write_read() {
    {
        let mut mem = RAM::new(0, 1024);
        mem.write32(0, 0xAABB_CCDD).unwrap();
        assert_eq!(mem.read32(0).unwrap(), 0xAABB_CCDD);
        mem.write32(1020, 0xAABB_CCDD).unwrap();
        assert_eq!(mem.read32(1020).unwrap(), 0xAABB_CCDD);
    }

    {
        let mut mem = RAM::new(0, 1024);
        mem.write16(0, 0xAABB).unwrap();
        assert_eq!(mem.read16(0).unwrap(), 0xAABB);
        mem.write16(1022, 0xCCDD).unwrap();
        assert_eq!(mem.read16(1022).unwrap(), 0xCCDD);
    }

    {
        let mut mem = RAM::new(0, 1024);
        mem.write8(0, 0xAA).unwrap();
        assert_eq!(mem.read8(0).unwrap(), 0xAA);
        mem.write8(1022, 0xCC).unwrap();
        assert_eq!(mem.read8(1022).unwrap(), 0xCC);
    }
}
