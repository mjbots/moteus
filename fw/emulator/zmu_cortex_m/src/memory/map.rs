//!
//! Memory mapping configuration and traits.
//!

use crate::Processor;
///
/// Mapping of memory from one range to another
///
pub trait MapMemory {
    ///
    /// Resolves the mapped address for given address
    /// Might return same address in case range not mapped.
    ///
    fn map_address(&self, address: u32) -> u32;
}

///
/// Mapping of memory range to another range
pub struct MemoryMapConfig {
    /// source of mapping
    source_start: u32,
    /// source of mapping (end)
    source_end: u32,
    /// target of mapping
    target_start: u32,
}

impl MemoryMapConfig {
    /// construct mapping
    pub fn new(from: u32, to: u32, len: usize) -> Self {
        Self {
            source_start: from,
            source_end: from + len as u32,
            target_start: to,
        }
    }

    /// check if address is affected by mapping
    pub fn contains(&self, address: u32) -> bool {
        address >= self.source_start && address < self.source_end
    }
}

impl MapMemory for MemoryMapConfig {
    fn map_address(&self, address: u32) -> u32 {
        if self.contains(address) {
            address - self.source_start + self.target_start
        } else {
            address
        }
    }
}

impl MapMemory for Processor {
    #[inline(always)]
    fn map_address(&self, address: u32) -> u32 {
        if let Some(map) = &self.mem_map {
            map.map_address(address)
        } else {
            address
        }
    }
}
