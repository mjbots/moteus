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

//! Moteus Command type — a routed protocol message.
//!
//! A `Command` pairs moteus routing information (destination, source,
//! can_prefix) with serialized multiplex protocol data. It is produced by
//! [`Controller::make_*()`](crate::Controller) methods and converted to a
//! wire-level [`CanFdFrame`] via [`into_frame()`](Command::into_frame).

use crate::device_address::DeviceAddress;
use crate::transport::transaction::FrameFilter;
use moteus_protocol::{calculate_arbitration_id, CanFdFrame, SERVER_TO_CLIENT};

/// A moteus Command representing a routed protocol message.
///
/// This is the Rust equivalent of Python's `command.Command`. It combines
/// moteus routing information with serialized multiplex protocol data, and
/// can be converted to a wire-level `CanFdFrame` via `into_frame()`.
///
/// # Example
///
/// ```rust
/// use moteus::{Controller, Command};
///
/// let controller = Controller::new(1);
/// let cmd = controller.make_stop(false);
/// let frame = cmd.into_frame();
/// // frame is now a CanFdFrame ready for transport
/// ```
#[non_exhaustive]
#[derive(Clone)]
pub struct Command {
    /// Destination device ID
    pub destination: i8,
    /// Source device ID
    pub source: i8,
    /// 13-bit CAN prefix
    pub can_prefix: u16,

    /// Whether this command expects a reply
    pub reply_required: bool,
    /// Expected size of the reply frame
    pub expected_reply_size: u8,

    /// If true, `arbitration_id` is used directly instead of being
    /// calculated from destination/source/can_prefix.
    pub raw: bool,
    /// Direct arbitration ID (only used when `raw` is true)
    pub arbitration_id: u32,

    /// Optional content-level reply filter for response matching.
    ///
    /// When set, this filter is combined with routing-level checks
    /// (prefix, source, destination) in [`crate::transport::transaction::Request::from_command()`] to
    /// build a compound filter matching Python's `_make_response_filter`.
    pub reply_filter: Option<FrameFilter>,

    /// Optional channel for multi-channel transport routing
    pub channel: Option<usize>,

    /// Optional device address for routing table lookup.
    ///
    /// When set, the transport uses this to look up the exact device
    /// (e.g. via UUID) rather than routing to all devices sharing the
    /// same CAN ID.
    pub address: Option<DeviceAddress>,

    /// Internal frame used as a data buffer for serialization.
    /// Protocol command types serialize into this frame's data/size fields.
    frame: CanFdFrame,
}

impl Command {
    /// Creates a new Command for the given routing parameters.
    pub fn new(destination: i8, source: i8, can_prefix: u16) -> Self {
        Command {
            destination,
            source,
            can_prefix,
            reply_required: false,
            expected_reply_size: 0,
            raw: false,
            arbitration_id: 0,
            reply_filter: None,
            channel: None,
            address: None,
            frame: CanFdFrame::new(),
        }
    }

    /// Sets `reply_required` (builder pattern).
    #[must_use]
    pub fn reply_required(mut self, required: bool) -> Self {
        self.reply_required = required;
        self
    }

    /// Returns a mutable reference to the internal frame for serialization.
    ///
    /// Protocol command types (`PositionCommand`, `StopCommand`, etc.) and
    /// `QueryFormat` serialize their data into this frame.
    pub fn frame_mut(&mut self) -> &mut CanFdFrame {
        &mut self.frame
    }

    /// Returns the serialized payload data.
    pub fn data(&self) -> &[u8] {
        self.frame.payload()
    }

    /// Returns the serialized data size.
    pub fn data_size(&self) -> u8 {
        self.frame.size
    }

    /// Returns a filter matching query reply frames.
    ///
    /// Matches multiplex reply subframes (`data[0] & 0xf0 == 0x20`) and
    /// write-error subframes (`data[0] == 0x31`), matching Python's
    /// `expect_reply` filter.
    pub fn query_reply_filter() -> FrameFilter {
        FrameFilter::custom(|f| {
            if f.size < 1 {
                return false;
            }
            f.data[0] & 0xf0 == 0x20 || f.data[0] == 0x31
        })
    }

    /// Returns a filter matching diagnostic stream response frames.
    ///
    /// Matches `SERVER_TO_CLIENT` (`data[0] == 0x41`), matching Python's
    /// `expect_diagnostic_response` filter.
    pub fn diagnostic_reply_filter() -> FrameFilter {
        FrameFilter::custom(|f| {
            if f.size < 3 {
                return false;
            }
            f.data[0] == SERVER_TO_CLIENT
        })
    }

    /// Converts this Command into a wire-level `CanFdFrame`.
    ///
    /// Computes the CAN arbitration ID from routing fields and produces
    /// a frame ready for transmission via a transport.
    pub fn into_frame(self) -> CanFdFrame {
        let mut frame = self.frame;

        if self.raw {
            frame.arbitration_id = self.arbitration_id;
        } else {
            frame.arbitration_id = calculate_arbitration_id(
                self.source,
                self.destination,
                self.can_prefix,
                self.reply_required,
            );
        }

        frame.channel = self.channel;

        frame
    }
}

impl std::fmt::Debug for Command {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Command")
            .field("destination", &self.destination)
            .field("source", &self.source)
            .field("can_prefix", &self.can_prefix)
            .field("reply_required", &self.reply_required)
            .field("data_size", &self.frame.size)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_new() {
        let cmd = Command::new(1, 0, 0);
        assert_eq!(cmd.destination, 1);
        assert_eq!(cmd.source, 0);
        assert_eq!(cmd.can_prefix, 0);
        assert!(!cmd.reply_required);
        assert_eq!(cmd.expected_reply_size, 0);
        assert!(!cmd.raw);
        assert!(cmd.channel.is_none());
        assert_eq!(cmd.data_size(), 0);
    }

    #[test]
    fn test_command_reply_required_builder() {
        let cmd = Command::new(1, 0, 0).reply_required(true);
        assert!(cmd.reply_required);
    }

    #[test]
    fn test_command_into_frame() {
        let cmd = Command::new(1, 0, 0).reply_required(true);
        let frame = cmd.into_frame();
        // arbitration_id should be: 0x8000 | (0 << 8) | 1 = 0x8001
        assert_eq!(frame.arbitration_id, 0x8001);
    }

    #[test]
    fn test_command_into_frame_no_reply() {
        let cmd = Command::new(1, 0, 0).reply_required(false);
        let frame = cmd.into_frame();
        // No reply bit: (0 << 8) | 1 = 0x0001
        assert_eq!(frame.arbitration_id, 0x0001);
    }

    #[test]
    fn test_command_into_frame_raw() {
        let mut cmd = Command::new(1, 0, 0);
        cmd.raw = true;
        cmd.arbitration_id = 0xDEAD;
        let frame = cmd.into_frame();
        assert_eq!(frame.arbitration_id, 0xDEAD);
    }

    #[test]
    fn test_command_into_frame_with_prefix() {
        let cmd = Command::new(3, 5, 0x10).reply_required(true);
        let frame = cmd.into_frame();
        assert_eq!(frame.arbitration_id, 0x00_10_85_03);
    }

    #[test]
    fn test_command_into_frame_channel_propagation() {
        let mut cmd = Command::new(1, 0, 0);
        cmd.channel = Some(2);
        let frame = cmd.into_frame();
        assert_eq!(frame.channel, Some(2));
    }

    #[test]
    fn test_command_into_frame_channel_none() {
        let cmd = Command::new(1, 0, 0);
        let frame = cmd.into_frame();
        assert!(frame.channel.is_none());
    }

    #[test]
    fn test_command_frame_mut_serialization() {
        let mut cmd = Command::new(1, 0, 0).reply_required(true);
        // Simulate writing data into the frame
        let frame = cmd.frame_mut();
        frame.data[0] = 0x01;
        frame.data[1] = 0x00;
        frame.data[2] = 0x0A;
        frame.size = 3;

        assert_eq!(cmd.data(), &[0x01, 0x00, 0x0A]);

        let wire_frame = cmd.into_frame();
        assert_eq!(wire_frame.payload(), &[0x01, 0x00, 0x0A]);
        assert_eq!(wire_frame.arbitration_id, 0x8001);
    }
}
