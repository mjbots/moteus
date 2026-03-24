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

//! CAN-FD frame types and moteus arbitration ID helpers.

/// Toggle state for CAN-FD frame options.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Toggle {
    /// Use the default behavior
    #[default]
    Default,
    /// Force the option off
    ForceOff,
    /// Force the option on
    ForceOn,
}

/// A CAN-FD frame.
///
/// This structure represents a CAN-FD frame with only standard CAN fields:
/// arbitration ID, payload data, and CAN-FD options. Moteus-specific routing
/// (destination, source, can_prefix) is handled by the higher-level `Command`
/// type in the `moteus` crate.
#[derive(Clone)]
pub struct CanFdFrame {
    /// Channel index identifying which transport device this frame was
    /// received from or should be sent to.
    pub channel: Option<usize>,
    /// CAN arbitration ID
    pub arbitration_id: u32,
    /// Frame payload data (up to 64 bytes for CAN-FD)
    pub data: [u8; 64],
    /// Actual size of data in the frame
    pub size: u8,

    /// Bit rate switch (BRS) toggle
    pub brs: Toggle,
    /// FD frame format toggle
    pub fdcan_frame: Toggle,
}

impl Default for CanFdFrame {
    fn default() -> Self {
        Self::new()
    }
}

impl CanFdFrame {
    /// Creates a new empty CAN-FD frame.
    pub const fn new() -> Self {
        CanFdFrame {
            channel: None,
            arbitration_id: 0,
            data: [0u8; 64],
            size: 0,
            brs: Toggle::Default,
            fdcan_frame: Toggle::Default,
        }
    }

    /// Returns the payload data as a slice.
    pub fn payload(&self) -> &[u8] {
        &self.data[..self.size as usize]
    }

    /// Returns the payload data as a mutable slice.
    pub fn payload_mut(&mut self) -> &mut [u8] {
        &mut self.data[..self.size as usize]
    }

    /// Clears the frame data, keeping metadata.
    pub fn clear_data(&mut self) {
        self.data = [0u8; 64];
        self.size = 0;
    }

    /// Returns the remaining capacity in bytes.
    pub fn remaining_capacity(&self) -> usize {
        64 - self.size as usize
    }

    /// Returns true if the frame is empty (no data).
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    /// Returns true if BRS (bit rate switching) should be enabled.
    pub fn brs_enabled(&self) -> bool {
        matches!(self.brs, Toggle::ForceOn | Toggle::Default)
    }

    /// Returns true if FD CAN frame format should be used.
    pub fn fdcan_enabled(&self) -> bool {
        matches!(self.fdcan_frame, Toggle::ForceOn | Toggle::Default)
    }

    /// Sets BRS toggle.
    pub fn set_brs(&mut self, enabled: bool) {
        self.brs = if enabled {
            Toggle::ForceOn
        } else {
            Toggle::ForceOff
        };
    }

    /// Sets FD CAN frame format toggle.
    pub fn set_fdcan(&mut self, enabled: bool) {
        self.fdcan_frame = if enabled {
            Toggle::ForceOn
        } else {
            Toggle::ForceOff
        };
    }

    /// Rounds up a data length to the next valid CAN-FD DLC size.
    pub fn round_up_dlc(size: usize) -> usize {
        match size {
            0..=8 => size,
            9..=12 => 12,
            13..=16 => 16,
            17..=20 => 20,
            21..=24 => 24,
            25..=32 => 32,
            33..=48 => 48,
            _ => 64,
        }
    }

    /// Pads the frame data with `0x50` to the next valid CAN-FD DLC size.
    pub fn pad_to_dlc(&mut self) {
        let on_wire_size = Self::round_up_dlc(self.size as usize);
        for i in self.size as usize..on_wire_size {
            self.data[i] = 0x50;
        }
        self.size = on_wire_size as u8;
    }
}

impl core::fmt::Debug for CanFdFrame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let (source, destination, _) = parse_arbitration_id(self.arbitration_id);
        f.debug_struct("CanFdFrame")
            .field("channel", &self.channel)
            .field(
                "arbitration_id",
                &format_args!("0x{:08x}", self.arbitration_id),
            )
            .field("size", &self.size)
            .field("data", &format_args!("{:02x?}", self.payload()))
            .field("destination", &destination)
            .field("source", &source)
            .finish()
    }
}

/// Computes a moteus CAN arbitration ID from routing fields.
///
/// The moteus CAN protocol encodes routing information in the arbitration ID:
/// - bits 28:16 = can_prefix (13 bits)
/// - bit 15     = reply expected flag
/// - bits 14:8  = source (7 bits)
/// - bits 7:0   = destination (8 bits)
pub fn calculate_arbitration_id(
    source: i8,
    destination: i8,
    can_prefix: u16,
    reply_required: bool,
) -> u32 {
    let prefix = (can_prefix as u32) << 16;
    let src = ((source as u8) as u32) << 8;
    let dest = (destination as u8) as u32;
    let reply = if reply_required { 0x8000 } else { 0 };
    prefix | src | dest | reply
}

/// Extracts routing fields from a moteus CAN arbitration ID.
///
/// Returns (source, destination, can_prefix).
/// This is the inverse of [`calculate_arbitration_id`].
pub fn parse_arbitration_id(arb_id: u32) -> (i8, i8, u16) {
    let source = ((arb_id >> 8) & 0x7F) as i8;
    let destination = (arb_id & 0xFF) as i8;
    let can_prefix = ((arb_id >> 16) & 0x1FFF) as u16;
    (source, destination, can_prefix)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_frame() {
        let frame = CanFdFrame::new();
        assert_eq!(frame.size, 0);
        assert_eq!(frame.arbitration_id, 0);
        assert!(frame.channel.is_none());
    }

    #[test]
    fn test_payload() {
        let mut frame = CanFdFrame::new();
        frame.data[0] = 0x01;
        frame.data[1] = 0x02;
        frame.size = 2;

        assert_eq!(frame.payload(), &[0x01, 0x02]);
    }

    #[test]
    fn test_calculate_arbitration_id() {
        // source=0, dest=1, no prefix, reply required
        let arb_id = calculate_arbitration_id(0, 1, 0, true);
        assert_eq!(arb_id, 0x8001);

        // source=0, dest=1, no prefix, no reply
        let arb_id = calculate_arbitration_id(0, 1, 0, false);
        assert_eq!(arb_id, 0x0001);

        // source=5, dest=3, prefix=0x10, reply required
        let arb_id = calculate_arbitration_id(5, 3, 0x10, true);
        assert_eq!(arb_id, 0x00_10_85_03);
    }

    #[test]
    fn test_parse_arbitration_id() {
        let (source, dest, prefix) = parse_arbitration_id(0x00_10_85_03);
        assert_eq!(source, 5);
        assert_eq!(dest, 3);
        assert_eq!(prefix, 0x10);
    }

    #[test]
    fn test_arbitration_id_roundtrip() {
        let arb_id = calculate_arbitration_id(7, 42, 0x1A, true);
        let (source, dest, prefix) = parse_arbitration_id(arb_id);
        assert_eq!(source, 7);
        assert_eq!(dest, 42);
        assert_eq!(prefix, 0x1A);
    }
}
