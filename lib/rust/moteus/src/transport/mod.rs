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

//! Transport layer for communicating with moteus controllers.
//!
//! This module provides transports for CAN-FD communication:
//! - [`fdcanusb`]: USB-to-CAN adapter using serial CDC protocol
//! - [`device`]: TransportDevice trait for hardware abstraction
//! - [`transaction`]: Request/response handling

pub mod async_transport;
pub mod device;
pub mod fdcanusb;
pub mod socketcan;
pub(crate) mod socketcan_common;
pub mod transaction;

use crate::error::Result;
use moteus_protocol::CanFdFrame;

// Re-export transaction types
pub use transaction::{dispatch_frame, FrameFilter, Request, ResponseCollector};

// Re-export device types
pub use device::{TransportDevice, TransportDeviceInfo};

/// Trait for transport operations.
///
/// This trait defines the common interface for transport implementations.
pub trait TransportOps {
    /// Executes a cycle: sends frames and collects responses.
    fn cycle(&mut self, requests: &mut [Request]) -> Result<()>;

    /// Sends a frame without waiting for a response.
    fn write(&mut self, frame: &CanFdFrame) -> Result<()>;

    /// Receives an unsolicited frame, if available.
    fn read(&mut self, channel: Option<usize>) -> Result<Option<CanFdFrame>>;

    /// Flushes any pending unsolicited frames.
    fn flush_read(&mut self, channel: Option<usize>) -> Result<()>;

    /// Sets the communication timeout in milliseconds.
    fn set_timeout(&mut self, timeout_ms: u32);

    /// Returns the current timeout in milliseconds.
    fn timeout(&self) -> u32;
}

/// A null transport for testing that doesn't communicate with hardware.
pub struct NullTransport {
    timeout_ms: u32,
}

impl NullTransport {
    /// Create a new null transport.
    pub fn new() -> Self {
        Self { timeout_ms: 100 }
    }
}

impl Default for NullTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl TransportOps for NullTransport {
    fn cycle(&mut self, _requests: &mut [Request]) -> Result<()> {
        Ok(())
    }

    fn write(&mut self, _frame: &CanFdFrame) -> Result<()> {
        Ok(())
    }

    fn read(&mut self, _channel: Option<usize>) -> Result<Option<CanFdFrame>> {
        Ok(None)
    }

    fn flush_read(&mut self, _channel: Option<usize>) -> Result<()> {
        Ok(())
    }

    fn set_timeout(&mut self, timeout_ms: u32) {
        self.timeout_ms = timeout_ms;
    }

    fn timeout(&self) -> u32 {
        self.timeout_ms
    }
}
