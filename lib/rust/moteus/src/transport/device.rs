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

//! Transport device abstraction for CAN-FD hardware.
//!
//! This module defines the `TransportDevice` trait that abstracts over
//! individual CAN-FD hardware interfaces like fdcanusb and socketcan.

use crate::error::Result;
use crate::transport::transaction::Request;
use moteus_protocol::CanFdFrame;

#[cfg(feature = "tokio")]
use crate::transport::async_transport::BoxFuture;

/// Information about a transport device.
#[derive(Debug, Clone)]
pub struct TransportDeviceInfo {
    /// Unique identifier for this device within the router.
    pub id: usize,
    /// Human-readable name for the device.
    pub name: String,
    /// USB serial number or other unique identifier, if available.
    pub serial_number: Option<String>,
    /// Human-readable detail shown in parentheses when displaying.
    /// e.g. `"sn='ABC123'"` or `"'can0'"`.
    pub detail: Option<String>,
    /// Index of the parent device in the Transport's device list.
    ///
    /// When `Some(p)`, this device is a child of device `p`. The Transport
    /// will route requests through the parent device and only call
    /// `read()`/`flush()` on parent devices.
    pub parent_index: Option<usize>,
    /// Whether it's safe to send broadcast frames on this device with
    /// no hardware attached.  Cached at construction time so the async
    /// transport can read it without locking the device mutex.
    pub empty_bus_tx_safe: bool,
}

impl TransportDeviceInfo {
    /// Create a new TransportDeviceInfo.
    pub fn new(id: usize, name: impl Into<String>) -> Self {
        Self {
            id,
            name: name.into(),
            serial_number: None,
            detail: None,
            parent_index: None,
            empty_bus_tx_safe: true,
        }
    }

    /// Set the serial number.
    pub fn with_serial(mut self, serial: impl Into<String>) -> Self {
        self.serial_number = Some(serial.into());
        self
    }

    /// Set the display detail shown in parentheses.
    pub fn with_detail(mut self, detail: impl Into<String>) -> Self {
        self.detail = Some(detail.into());
        self
    }
}

impl std::fmt::Display for TransportDeviceInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.detail {
            Some(detail) => write!(f, "{}({})", self.name, detail),
            None => write!(f, "{}()", self.name),
        }
    }
}

/// A blocking transport device for CAN-FD communication.
///
/// This trait represents a single hardware interface (e.g., one fdcanusb
/// or one socketcan interface). The `Transport` can manage multiple
/// devices to support multi-bus configurations.
///
/// # Transaction Model
///
/// Devices use a transaction model where requests contain frames to
/// send and collectors for responses.
pub trait TransportDevice: Send {
    /// Executes a transaction: sends frames and collects responses.
    ///
    /// This is the primary method for communicating over the CAN bus.
    /// Each request specifies:
    /// - An optional frame to send
    /// - A filter for matching response frames
    /// - Expected number of replies
    /// - A collector where responses are stored
    fn transaction(&mut self, requests: &mut [Request]) -> Result<()>;

    /// Sends a frame without waiting for a response.
    fn write(&mut self, frame: &CanFdFrame) -> Result<()>;

    /// Receives an unsolicited frame, if available.
    fn read(&mut self) -> Result<Option<CanFdFrame>>;

    /// Flushes any pending frames from the receive buffer.
    fn flush(&mut self) -> Result<()>;

    /// Whether it's safe to send broadcast frames on this device with
    /// no hardware attached.
    fn empty_bus_tx_safe(&self) -> bool {
        true
    }

    /// Returns information about this device.
    fn info(&self) -> &TransportDeviceInfo;

    /// Sets the communication timeout in milliseconds.
    fn set_timeout(&mut self, timeout_ms: u32);

    /// Returns the current timeout in milliseconds.
    fn timeout(&self) -> u32;
}

/// An async transport device for CAN-FD communication.
///
/// This is the async version of `TransportDevice`.
///
/// # Cancel safety
///
/// Implementations must tolerate having any future dropped mid-operation.
/// The [`recover`](Self::recover) method is called automatically before
/// each transaction to restore the device to a known-good state after a
/// possible cancellation.
#[cfg(feature = "tokio")]
pub trait AsyncTransportDevice: Send {
    /// Restores the device to a known-good state after a possible
    /// cancellation of a previous operation.
    ///
    /// Called automatically by the transport before each transaction.
    /// Implementations should use an internal `needs_recovery` flag to
    /// skip I/O when the device is already clean, making this a no-op
    /// on the normal (non-cancelled) path.
    ///
    /// # Cancel safety
    ///
    /// If this method itself is cancelled before completing, the device
    /// must remain in a state where the next `recover()` call will
    /// retry the recovery.
    fn recover(&mut self) -> BoxFuture<'_, Result<()>> {
        Box::pin(async { Ok(()) })
    }

    /// Executes a transaction: sends frames and collects responses asynchronously.
    fn transaction<'a>(&'a mut self, requests: &'a mut [Request]) -> BoxFuture<'a, Result<()>>;

    /// Sends a frame without waiting for a response.
    fn write<'a>(&'a mut self, frame: &'a CanFdFrame) -> BoxFuture<'a, Result<()>>;

    /// Receives an unsolicited frame, if available.
    fn read(&mut self) -> BoxFuture<'_, Result<Option<CanFdFrame>>>;

    /// Flushes any pending frames from the receive buffer.
    fn flush(&mut self) -> BoxFuture<'_, Result<()>>;

    /// Whether it's safe to send broadcast frames on this device.
    fn empty_bus_tx_safe(&self) -> bool {
        true
    }

    /// Returns information about this device.
    fn info(&self) -> &TransportDeviceInfo;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transport_device_info() {
        let info = TransportDeviceInfo::new(0, "test")
            .with_serial("ABC123");
        assert_eq!(info.id, 0);
        assert_eq!(info.name, "test");
        assert_eq!(info.serial_number, Some("ABC123".to_string()));
    }

}
