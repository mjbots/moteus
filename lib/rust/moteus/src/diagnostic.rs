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

//! Diagnostic stream protocol for moteus controllers.
//!
//! The diagnostic stream provides a text-based interface for configuration,
//! debugging, and telemetry access. This module provides both blocking and
//! async implementations.
//!
//! # Example (Blocking)
//!
//! ```no_run
//! use moteus::{BlockingController, DiagnosticStream};
//!
//! fn main() -> Result<(), moteus::Error> {
//!     let mut ctrl = BlockingController::new(1);
//!     let mut stream = DiagnosticStream::new(&mut ctrl);
//!
//!     // Stop any telemetry spew and flush pending data
//!     stream.write_message(b"tel stop")?;
//!     stream.flush_read()?;
//!
//!     // Read a configuration value (single-line response)
//!     let value = stream.command_oneline(b"conf get servo.pid_position.kp")?;
//!     println!("kp = {}", String::from_utf8_lossy(&value));
//!
//!     // Set a configuration value (waits for "OK")
//!     stream.command(b"conf set servo.pid_position.kp 4.0")?;
//!     Ok(())
//! }
//! ```
//!
//! # Example (Async)
//!
//! ```ignore
//! use moteus::{AsyncController, AsyncDiagnosticStream};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), moteus::Error> {
//!     let mut ctrl = AsyncController::new(1);
//!     let mut stream = AsyncDiagnosticStream::new(&mut ctrl);
//!
//!     stream.write_message(b"tel stop").await?;
//!     stream.flush_read().await?;
//!
//!     let value = stream.command_oneline(b"conf get servo.pid_position.kp").await?;
//!     Ok(())
//! }
//! ```

use crate::command_types::Command;
use crate::error::{Error, Result};
use crate::transport::transaction::{FrameFilter, Request};
use moteus_protocol::{multiplex, CanFdFrame};

/// Default diagnostic channel.
pub const DEFAULT_CHANNEL: u8 = 1;

/// Maximum data size per diagnostic write frame.
/// CAN-FD frames are 64 bytes max, minus 3 bytes for header (cmd, channel, len).
pub const MAX_DIAGNOSTIC_WRITE: usize = 61;

/// Maximum data size per diagnostic read request.
pub const MAX_DIAGNOSTIC_READ: usize = 61;

/// Result of parsing a diagnostic response.
#[derive(Debug, Clone)]
pub struct DiagnosticResponse {
    /// The CAN ID of the responding device.
    pub id: u8,
    /// The data received (may be empty).
    pub data: Vec<u8>,
}

/// Creates a frame to write data to the diagnostic stream.
///
/// # Arguments
/// * `dest_id` - Destination CAN ID
/// * `source_id` - Source CAN ID
/// * `channel` - Diagnostic channel (usually 1)
/// * `data` - Data to write (max 61 bytes)
pub fn make_diagnostic_write_frame(
    dest_id: u8,
    source_id: u8,
    channel: u8,
    data: &[u8],
) -> CanFdFrame {
    assert!(data.len() <= MAX_DIAGNOSTIC_WRITE);

    let mut frame = CanFdFrame::new();
    frame.arbitration_id =
        moteus_protocol::calculate_arbitration_id(source_id as i8, dest_id as i8, 0, false);

    // Write header: CLIENT_TO_SERVER, channel, length
    frame.data[0] = multiplex::CLIENT_TO_SERVER;
    frame.data[1] = channel;
    frame.data[2] = data.len() as u8;

    // Write data
    frame.data[3..3 + data.len()].copy_from_slice(data);
    frame.size = (3 + data.len()) as u8;

    frame
}

/// Creates a frame to poll for diagnostic data.
///
/// # Arguments
/// * `dest_id` - Destination CAN ID
/// * `source_id` - Source CAN ID
/// * `channel` - Diagnostic channel (usually 1)
/// * `max_length` - Maximum bytes to read
pub fn make_diagnostic_read_frame(
    dest_id: u8,
    source_id: u8,
    channel: u8,
    max_length: u8,
) -> CanFdFrame {
    let mut frame = CanFdFrame::new();
    frame.arbitration_id =
        moteus_protocol::calculate_arbitration_id(source_id as i8, dest_id as i8, 0, true);

    // Write header: CLIENT_POLL_SERVER, channel, max_length
    frame.data[0] = multiplex::CLIENT_POLL_SERVER;
    frame.data[1] = channel;
    frame.data[2] = max_length;
    frame.size = 3;

    frame
}

/// Parses a diagnostic response frame.
///
/// Returns the data if the response is valid for the given channel,
/// or None if the frame is not a diagnostic response.
pub fn parse_diagnostic_response(frame: &CanFdFrame, channel: u8) -> Option<DiagnosticResponse> {
    let data = &frame.data[..frame.size as usize];

    if data.len() < 3 {
        return None;
    }

    // Check for SERVER_TO_CLIENT response
    if data[0] != multiplex::SERVER_TO_CLIENT {
        return None;
    }

    // Check channel
    if data[1] != channel {
        return None;
    }

    // Read varuint length (simplified - assumes single byte for now)
    let data_len = data[2] as usize;
    let data_start = 3;

    if data_len > data.len() - data_start {
        return None;
    }

    let id = ((frame.arbitration_id >> 8) & 0x7F) as u8;

    Some(DiagnosticResponse {
        id,
        data: data[data_start..data_start + data_len].to_vec(),
    })
}

// ============================================================================
// Blocking DiagnosticStream
// ============================================================================

use crate::blocking_controller::BlockingController;

/// A blocking diagnostic stream for a moteus controller.
///
/// This provides a text-based interface for configuration and debugging.
/// The diagnostic protocol is line-based and uses commands like:
/// - `tel stop` - Stop telemetry spew
/// - `conf get <param>` - Read configuration value
/// - `conf set <param> <value>` - Set configuration value
///
/// # Example
///
/// ```no_run
/// use moteus::{BlockingController, DiagnosticStream};
///
/// fn main() -> Result<(), moteus::Error> {
///     let mut ctrl = BlockingController::new(1);
///     let mut stream = DiagnosticStream::new(&mut ctrl);
///
///     // Always stop telemetry and flush before using diagnostic commands
///     stream.write_message(b"tel stop")?;
///     stream.flush_read()?;
///
///     // Read a config value (single-line response)
///     let kp = stream.command_oneline(b"conf get servo.pid_position.kp")?;
///     println!("kp = {}", String::from_utf8_lossy(&kp));
///     Ok(())
/// }
/// ```
pub struct DiagnosticStream<'a> {
    controller: &'a mut BlockingController,
    channel: u8,
    read_buffer: Vec<u8>,
}

impl<'a> std::fmt::Debug for DiagnosticStream<'a> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DiagnosticStream")
            .field("device_id", &self.controller.controller.id)
            .field("channel", &self.channel)
            .field("read_buffer_len", &self.read_buffer.len())
            .finish()
    }
}

impl<'a> DiagnosticStream<'a> {
    /// Creates a new diagnostic stream for a controller.
    pub fn new(controller: &'a mut BlockingController) -> Self {
        Self::with_channel(controller, DEFAULT_CHANNEL)
    }

    /// Creates a new diagnostic stream with a specific channel.
    pub fn with_channel(controller: &'a mut BlockingController, channel: u8) -> Self {
        Self {
            controller,
            channel,
            read_buffer: Vec::new(),
        }
    }

    /// Returns the controller's CAN ID.
    fn id(&self) -> u8 {
        self.controller.controller.id
    }

    /// Returns the controller's source ID.
    fn source_id(&self) -> u8 {
        self.controller.controller.source_id
    }

    /// Writes raw data to the diagnostic stream.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub fn write(&mut self, data: &[u8]) -> Result<()> {
        // Split into chunks if necessary
        for chunk in data.chunks(MAX_DIAGNOSTIC_WRITE) {
            let frame =
                make_diagnostic_write_frame(self.id(), self.source_id(), self.channel, chunk);
            // Write frames don't expect a reply
            let mut requests = [Request::new(frame).with_expected_replies(0)];
            self.controller.transport.cycle(&mut requests)?;
        }
        Ok(())
    }

    /// Writes a message (with newline) to the diagnostic stream.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub fn write_message(&mut self, data: &[u8]) -> Result<()> {
        let mut msg = data.to_vec();
        msg.push(b'\n');
        self.write(&msg)
    }

    /// Reads data from the diagnostic stream.
    ///
    /// Returns up to `max_bytes` of available data.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub fn read(&mut self, max_bytes: usize) -> Result<Vec<u8>> {
        let read_size = std::cmp::min(max_bytes, MAX_DIAGNOSTIC_READ) as u8;
        let frame =
            make_diagnostic_read_frame(self.id(), self.source_id(), self.channel, read_size);

        let id = self.id();
        let mut requests = [Request::new(frame)
            .with_filter(FrameFilter::custom(move |f| {
                // Check source matches device ID
                let frame_source = ((f.arbitration_id >> 8) & 0x7F) as u8;
                if frame_source != id {
                    return false;
                }
                // Check diagnostic content
                Command::diagnostic_reply_filter().matches(f)
            }))
            .with_expected_replies(1)];
        self.controller.transport.cycle(&mut requests)?;

        let mut result = Vec::new();
        for response in requests[0].responses.take() {
            if let Some(diag) = parse_diagnostic_response(&response, self.channel) {
                result.extend(diag.data);
            }
        }

        Ok(result)
    }

    /// Flushes any pending read data.
    ///
    /// This should be called after `tel stop` to clear any buffered telemetry.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub fn flush_read(&mut self) -> Result<()> {
        self.read_buffer.clear();

        let start = std::time::Instant::now();
        let timeout = std::time::Duration::from_millis(200);

        while start.elapsed() < timeout {
            let data = self.read(MAX_DIAGNOSTIC_READ)?;
            if data.is_empty() {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
        }

        self.read_buffer.clear();
        Ok(())
    }

    /// Reads a single line from the diagnostic stream.
    ///
    /// Lines are terminated by '\n' or '\r'. Empty lines are skipped.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub fn readline(&mut self) -> Result<Vec<u8>> {
        loop {
            // Check for newline in buffer
            if let Some(pos) = self
                .read_buffer
                .iter()
                .position(|&b| b == b'\n' || b == b'\r')
            {
                let line: Vec<u8> = self.read_buffer.drain(..=pos).collect();
                // Strip trailing newline/carriage return
                let line: Vec<u8> = line
                    .into_iter()
                    .filter(|&b| b != b'\n' && b != b'\r')
                    .collect();

                // Skip empty lines
                if !line.is_empty() {
                    return Ok(line);
                }
                continue;
            }

            // Read more data
            let data = self.read(MAX_DIAGNOSTIC_READ)?;
            if data.is_empty() {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            self.read_buffer.extend(data);
        }
    }

    /// Reads lines until "OK" or "ERR" is received.
    fn read_until_ok(&mut self) -> Result<Vec<u8>> {
        let mut result = Vec::new();

        loop {
            let line = self.readline()?;

            if line.starts_with(b"OK") {
                return Ok(result);
            }

            if line.starts_with(b"ERR") {
                return Err(Error::Protocol(String::from_utf8_lossy(&line).to_string()));
            }

            result.extend(&line);
            result.push(b'\n');
        }
    }

    /// Sends a command and reads the response until "OK".
    ///
    /// Returns all lines received before the "OK" terminator.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not
    /// respond. Returns `Error::Protocol` if the device replies with "ERR".
    pub fn command(&mut self, data: &[u8]) -> Result<Vec<u8>> {
        self.write_message(data)?;
        self.read_until_ok()
    }

    /// Sends a command and reads a single line of response.
    ///
    /// This is useful for commands like `conf get` which return a
    /// single value without a trailing "OK".
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub fn command_oneline(&mut self, data: &[u8]) -> Result<Vec<u8>> {
        self.write_message(data)?;
        self.readline()
    }
}

// ============================================================================
// Async DiagnosticStream
// ============================================================================

#[cfg(feature = "tokio")]
use crate::async_controller::AsyncController;

/// An async diagnostic stream for a moteus controller.
///
/// This is the async version of `DiagnosticStream`.
///
/// # Cancel safety
///
/// Transport-level operations are cancel safe. However, multi-step
/// diagnostic sequences are not transactional — if `command()` is
/// cancelled between write and read, the response is lost. Call
/// `flush_read()` to resync after cancellation.
///
/// # Example
///
/// ```no_run
/// use moteus::{AsyncController, AsyncDiagnosticStream};
///
/// let mut ctrl = AsyncController::new(1);
/// let mut stream = AsyncDiagnosticStream::new(&mut ctrl);
///
/// stream.write_message(b"tel stop").await?;
/// stream.flush_read().await?;
///
/// let kp = stream.command_oneline(b"conf get servo.pid_position.kp").await?;
/// ```
#[cfg(feature = "tokio")]
pub struct AsyncDiagnosticStream<'a> {
    controller: &'a mut AsyncController,
    channel: u8,
    read_buffer: Vec<u8>,
}

#[cfg(feature = "tokio")]
impl<'a> std::fmt::Debug for AsyncDiagnosticStream<'a> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AsyncDiagnosticStream")
            .field("device_id", &self.controller.controller.id)
            .field("channel", &self.channel)
            .field("read_buffer_len", &self.read_buffer.len())
            .finish()
    }
}

#[cfg(feature = "tokio")]
impl<'a> AsyncDiagnosticStream<'a> {
    /// Creates a new async diagnostic stream for a controller.
    pub fn new(controller: &'a mut AsyncController) -> Self {
        Self::with_channel(controller, DEFAULT_CHANNEL)
    }

    /// Creates a new async diagnostic stream with a specific channel.
    pub fn with_channel(controller: &'a mut AsyncController, channel: u8) -> Self {
        Self {
            controller,
            channel,
            read_buffer: Vec::new(),
        }
    }

    /// Returns the controller's CAN ID.
    fn id(&self) -> u8 {
        self.controller.controller.id
    }

    /// Returns the controller's source ID.
    fn source_id(&self) -> u8 {
        self.controller.controller.source_id
    }

    /// Writes raw data to the diagnostic stream.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub async fn write(&mut self, data: &[u8]) -> Result<()> {
        for chunk in data.chunks(MAX_DIAGNOSTIC_WRITE) {
            let frame =
                make_diagnostic_write_frame(self.id(), self.source_id(), self.channel, chunk);
            // Write frames don't expect a reply
            let mut requests = [Request::new(frame).with_expected_replies(0)];
            self.controller.transport.cycle(&mut requests).await?;
        }
        Ok(())
    }

    /// Writes a message (with newline) to the diagnostic stream.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub async fn write_message(&mut self, data: &[u8]) -> Result<()> {
        let mut msg = data.to_vec();
        msg.push(b'\n');
        self.write(&msg).await
    }

    /// Reads data from the diagnostic stream.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub async fn read(&mut self, max_bytes: usize) -> Result<Vec<u8>> {
        let read_size = std::cmp::min(max_bytes, MAX_DIAGNOSTIC_READ) as u8;
        let frame =
            make_diagnostic_read_frame(self.id(), self.source_id(), self.channel, read_size);

        let id = self.id();
        let mut requests = [Request::new(frame)
            .with_filter(FrameFilter::custom(move |f| {
                // Check source matches device ID
                let frame_source = ((f.arbitration_id >> 8) & 0x7F) as u8;
                if frame_source != id {
                    return false;
                }
                // Check diagnostic content
                Command::diagnostic_reply_filter().matches(f)
            }))
            .with_expected_replies(1)];
        self.controller.transport.cycle(&mut requests).await?;

        let mut result = Vec::new();
        for response in requests[0].responses.take() {
            if let Some(diag) = parse_diagnostic_response(&response, self.channel) {
                result.extend(diag.data);
            }
        }

        Ok(result)
    }

    /// Flushes any pending read data.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub async fn flush_read(&mut self) -> Result<()> {
        self.read_buffer.clear();

        let start = std::time::Instant::now();
        let timeout = std::time::Duration::from_millis(200);

        while start.elapsed() < timeout {
            let data = self.read(MAX_DIAGNOSTIC_READ).await?;
            if data.is_empty() {
                tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            }
        }

        self.read_buffer.clear();
        Ok(())
    }

    /// Reads a single line from the diagnostic stream.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub async fn readline(&mut self) -> Result<Vec<u8>> {
        loop {
            // Check for newline in buffer
            if let Some(pos) = self
                .read_buffer
                .iter()
                .position(|&b| b == b'\n' || b == b'\r')
            {
                let line: Vec<u8> = self.read_buffer.drain(..=pos).collect();
                let line: Vec<u8> = line
                    .into_iter()
                    .filter(|&b| b != b'\n' && b != b'\r')
                    .collect();

                if !line.is_empty() {
                    return Ok(line);
                }
                continue;
            }

            // Read more data
            let data = self.read(MAX_DIAGNOSTIC_READ).await?;
            if data.is_empty() {
                tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            }
            self.read_buffer.extend(data);
        }
    }

    /// Reads lines until "OK" or "ERR" is received.
    async fn read_until_ok(&mut self) -> Result<Vec<u8>> {
        let mut result = Vec::new();

        loop {
            let line = self.readline().await?;

            if line.starts_with(b"OK") {
                return Ok(result);
            }

            if line.starts_with(b"ERR") {
                return Err(Error::Protocol(String::from_utf8_lossy(&line).to_string()));
            }

            result.extend(&line);
            result.push(b'\n');
        }
    }

    /// Sends a command and reads the response until "OK".
    ///
    /// Returns all lines received before the "OK" terminator.
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not
    /// respond. Returns `Error::Protocol` if the device replies with "ERR".
    pub async fn command(&mut self, data: &[u8]) -> Result<Vec<u8>> {
        self.write_message(data).await?;
        self.read_until_ok().await
    }

    /// Sends a command and reads a single line of response.
    ///
    /// This is useful for commands like `conf get` which return a
    /// single value without a trailing "OK".
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails or the device does not respond.
    pub async fn command_oneline(&mut self, data: &[u8]) -> Result<Vec<u8>> {
        self.write_message(data).await?;
        self.readline().await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_make_diagnostic_write_frame() {
        let frame = make_diagnostic_write_frame(1, 0, 1, b"hello");

        assert_eq!(
            frame.arbitration_id,
            moteus_protocol::calculate_arbitration_id(0, 1, 0, false)
        );
        assert_eq!(frame.data[0], multiplex::CLIENT_TO_SERVER);
        assert_eq!(frame.data[1], 1); // channel
        assert_eq!(frame.data[2], 5); // length
        assert_eq!(&frame.data[3..8], b"hello");
        assert_eq!(frame.size, 8);
    }

    #[test]
    fn test_make_diagnostic_read_frame() {
        let frame = make_diagnostic_read_frame(1, 0, 1, 48);

        assert_eq!(
            frame.arbitration_id,
            moteus_protocol::calculate_arbitration_id(0, 1, 0, true)
        );
        assert_eq!(frame.data[0], multiplex::CLIENT_POLL_SERVER);
        assert_eq!(frame.data[1], 1); // channel
        assert_eq!(frame.data[2], 48); // max_length
        assert_eq!(frame.size, 3);
    }

    #[test]
    fn test_parse_diagnostic_response() {
        let mut frame = CanFdFrame::new();
        frame.arbitration_id = 0x8100; // Source ID 1
        frame.data[0] = multiplex::SERVER_TO_CLIENT;
        frame.data[1] = 1; // channel
        frame.data[2] = 5; // length
        frame.data[3..8].copy_from_slice(b"hello");
        frame.size = 8;

        let result = parse_diagnostic_response(&frame, 1).unwrap();
        assert_eq!(result.id, 1);
        assert_eq!(result.data, b"hello");
    }

    #[test]
    fn test_parse_diagnostic_response_wrong_channel() {
        let mut frame = CanFdFrame::new();
        frame.data[0] = multiplex::SERVER_TO_CLIENT;
        frame.data[1] = 2; // different channel
        frame.data[2] = 5;
        frame.size = 8;

        let result = parse_diagnostic_response(&frame, 1);
        assert!(result.is_none());
    }
}
