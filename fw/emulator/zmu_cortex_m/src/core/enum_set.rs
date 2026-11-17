//!
//! A set of the members of an enum, held as a bit mask.
//!
//! This stands in for the "enum-set" crate, of which only this much
//! was used.
//!

use std::fmt;
use std::marker::PhantomData;

/// An enum whose members convert to and from small integers.
pub trait CLike {
    /// The member as an integer, which must be less than 64.
    fn to_u32(&self) -> u32;

    /// The member for an integer.
    ///
    /// # Safety
    ///
    /// The integer must be one which `to_u32` returns for some member.
    unsafe fn from_u32(v: u32) -> Self;
}

/// A set of the members of `E`.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct EnumSet<E> {
    bits: u64,
    marker: PhantomData<E>,
}

impl<E: CLike> EnumSet<E> {
    /// An empty set.
    pub fn new() -> Self {
        Self {
            bits: 0,
            marker: PhantomData,
        }
    }

    /// The number of members in the set.
    pub fn len(&self) -> usize {
        self.bits.count_ones() as usize
    }

    /// Whether the set has no members.
    pub fn is_empty(&self) -> bool {
        self.bits == 0
    }

    /// Whether `e` is in the set.
    pub fn contains(&self, e: &E) -> bool {
        self.bits & (1 << e.to_u32()) != 0
    }

    /// Add `e` to the set, returning whether it was absent before.
    pub fn insert(&mut self, e: E) -> bool {
        let absent = !self.contains(&e);
        self.bits |= 1 << e.to_u32();
        absent
    }

    /// The members of the set, in ascending order.
    pub fn iter(&self) -> Iter<E> {
        Iter {
            bits: self.bits,
            marker: PhantomData,
        }
    }
}

impl<E: CLike> Default for EnumSet<E> {
    fn default() -> Self {
        Self::new()
    }
}

impl<E: CLike + fmt::Debug> fmt::Debug for EnumSet<E> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_set().entries(self.iter()).finish()
    }
}

impl<E: CLike> IntoIterator for &EnumSet<E> {
    type Item = E;
    type IntoIter = Iter<E>;

    fn into_iter(self) -> Iter<E> {
        self.iter()
    }
}

/// An iterator over the members of an `EnumSet`.
pub struct Iter<E> {
    bits: u64,
    marker: PhantomData<E>,
}

impl<E: CLike> Iterator for Iter<E> {
    type Item = E;

    fn next(&mut self) -> Option<E> {
        if self.bits == 0 {
            return None;
        }
        let index = self.bits.trailing_zeros();
        self.bits &= self.bits - 1;
        // Only what `to_u32` returned was ever put in.
        Some(unsafe { E::from_u32(index) })
    }
}
