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

//! Async FdCanUSB transport using tokio.
//!
//! This module provides an async implementation of the FdCanUSB transport
//! using tokio for non-blocking I/O. It reuses the protocol encoder/decoder
//! from the blocking implementation.
//!
//! # Example
//!
//! ```ignore
//! use moteus::transport::async_fdcanusb::AsyncFdcanusb;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), moteus::Error> {
//!     let mut transport = AsyncFdcanusb::open("/dev/ttyACM0").await?;
//!     let responses = transport.transaction(&frames).await?;
//!     Ok(())
//! }
//! ```

use crate::error::{Error, Result};
use crate::transport::async_transport::BoxFuture;
use crate::transport::device::{AsyncTransportDevice, TransportDeviceInfo};
use crate::transport::fdcanusb::FdcanusbProtocol;
use crate::transport::transaction::{dispatch_frame, Request};
use moteus_protocol::CanFdFrame;

use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader, ReadHalf, WriteHalf};
use tokio_serial::SerialStream;

/// Async FdCanUSB transport using tokio-serial.
///
/// This provides an async interface for communicating with moteus
/// controllers through an fdcanusb device.
pub struct AsyncFdcanusb {
    reader: BufReader<ReadHalf<SerialStream>>,
    writer: WriteHalf<SerialStream>,
    timeout_ms: u32,
    disable_brs: bool,
    line_buffer: String,
    pending_frames: Vec<CanFdFrame>,
    pub(crate) info: TransportDeviceInfo,
    needs_recovery: bool,
}

impl AsyncFdcanusb {
    /// Opens an async FdCanUSB transport at the specified path.
    ///
    /// # Arguments
    /// * `path` - Device path (e.g., "/dev/ttyACM0")
    pub async fn open(path: &str) -> Result<Self> {
        Self::open_with_options(path, 100, false).await
    }

    /// Opens an async FdCanUSB transport with BRS option.
    ///
    /// # Arguments
    /// * `path` - Device path (e.g., "/dev/ttyACM0")
    /// * `disable_brs` - Whether to disable bit rate switching
    pub async fn open_with_brs(path: &str, disable_brs: bool) -> Result<Self> {
        Self::open_with_options(path, 100, disable_brs).await
    }

    /// Opens an async FdCanUSB transport with full options.
    ///
    /// # Arguments
    /// * `path` - Device path (e.g., "/dev/ttyACM0")
    /// * `timeout_ms` - Communication timeout in milliseconds
    /// * `disable_brs` - Whether to disable bit rate switching
    pub async fn open_with_options(path: &str, timeout_ms: u32, disable_brs: bool) -> Result<Self> {
        let builder = tokio_serial::new(path, 9600);
        let port = SerialStream::open(&builder).map_err(|e| Error::Io(e.into()))?;

        let (reader, writer) = tokio::io::split(port);
        let reader = BufReader::new(reader);

        Ok(Self {
            reader,
            writer,
            timeout_ms,
            disable_brs,
            line_buffer: String::new(),
            pending_frames: Vec::new(),
            info: TransportDeviceInfo::new(0, "AsyncFdcanusb"),
            needs_recovery: false,
        })
    }

    /// Enables or disables bit rate switching.
    pub fn set_disable_brs(&mut self, disable: bool) {
        self.disable_brs = disable;
    }

    /// Writes a frame without waiting for OK response.
    async fn write_frame(&mut self, frame: &CanFdFrame) -> Result<()> {
        let cmd = FdcanusbProtocol::encode_frame(frame, self.disable_brs);
        self.writer.write_all(cmd.as_bytes()).await?;
        Ok(())
    }

    /// Sends a single frame and waits for OK response.
    async fn send_frame(&mut self, frame: &CanFdFrame) -> Result<()> {
        self.write_frame(frame).await?;
        self.writer.flush().await?;
        self.wait_for_ok().await
    }

    /// Waits for an OK response with timeout.
    /// Any received frames (rcv lines) are buffered for later retrieval.
    async fn wait_for_ok(&mut self) -> Result<()> {
        let timeout = std::time::Duration::from_millis(self.timeout_ms as u64);

        loop {
            self.line_buffer.clear();

            let read_result = tokio::time::timeout(
                timeout,
                self.reader.read_line(&mut self.line_buffer),
            )
            .await;

            match read_result {
                Ok(Ok(0)) => {
                    // EOF - shouldn't happen for serial
                    return Err(Error::Io(std::io::Error::new(
                        std::io::ErrorKind::UnexpectedEof,
                        "Serial port closed",
                    )));
                }
                Ok(Ok(_)) => {
                    let line = self.line_buffer.trim();
                    if FdcanusbProtocol::is_ok_response(line) {
                        return Ok(());
                    }
                    if FdcanusbProtocol::is_error_response(line) {
                        return Err(Error::Protocol(format!("fdcanusb error: {}", line)));
                    }
                    // Save any received frames for later retrieval
                    if let Some(frame) = FdcanusbProtocol::parse_frame(&self.line_buffer) {
                        self.pending_frames.push(frame);
                    }
                }
                Ok(Err(e)) => return Err(Error::Io(e)),
                Err(_) => return Err(Error::Timeout),
            }
        }
    }

    /// Receives frames with timeout.
    async fn receive_frames(&mut self, expected_count: usize) -> Result<Vec<CanFdFrame>> {
        // First, drain any frames that were buffered during wait_for_ok
        let mut frames: Vec<CanFdFrame> = self.pending_frames.drain(..).collect();

        if frames.len() >= expected_count {
            return Ok(frames);
        }

        let timeout = std::time::Duration::from_millis(self.timeout_ms as u64);
        let deadline = tokio::time::Instant::now() + timeout;

        while frames.len() < expected_count {
            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            if remaining.is_zero() {
                break;
            }

            self.line_buffer.clear();

            let read_result = tokio::time::timeout(
                remaining,
                self.reader.read_line(&mut self.line_buffer),
            )
            .await;

            match read_result {
                Ok(Ok(0)) => break,
                Ok(Ok(_)) => {
                    if let Some(frame) = FdcanusbProtocol::parse_frame(&self.line_buffer) {
                        frames.push(frame);
                    }
                }
                Ok(Err(_)) | Err(_) => break,
            }
        }

        Ok(frames)
    }

    /// Execute a cycle sending frames and collecting responses.
    async fn execute_cycle(&mut self, requests: &mut [Request]) -> Result<()> {
        debug_assert!(
            requests.iter().all(|r| r.child_device.is_none()),
            "AsyncFdcanusb does not support child devices"
        );

        self.needs_recovery = true;

        // Pipeline: write all frames first
        let mut frames_sent = 0usize;
        for req in requests.iter() {
            if let Some(frame) = &req.frame {
                self.write_frame(frame).await?;
                frames_sent += 1;
            }
        }

        // Single flush for all frames
        if frames_sent > 0 {
            self.writer.flush().await?;
        }

        // Wait for all OKs
        for _ in 0..frames_sent {
            self.wait_for_ok().await?;
        }

        // Calculate expected replies
        let expected: usize = Request::total_expected_replies(requests);

        // Receive responses and dispatch to matching requests
        if expected > 0 {
            let responses = self.receive_frames(expected).await?;
            for frame in responses {
                dispatch_frame(&frame, requests);
            }
        }

        self.needs_recovery = false;
        Ok(())
    }
}

impl AsyncTransportDevice for AsyncFdcanusb {
    fn recover(&mut self) -> BoxFuture<'_, Result<()>> {
        Box::pin(async move {
            if !self.needs_recovery {
                return Ok(());
            }
            // Discard stale pending frames
            self.pending_frames.clear();

            // Send bare newline to terminate any partial command from
            // a cancelled write_all
            let _ = self.writer.write_all(b"\n").await;
            let _ = self.writer.flush().await;

            // Drain stale responses with short timeout
            let drain_timeout = std::time::Duration::from_millis(20);
            let deadline = tokio::time::Instant::now() + drain_timeout;
            while tokio::time::Instant::now() < deadline {
                let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
                if remaining.is_zero() {
                    break;
                }
                self.line_buffer.clear();
                match tokio::time::timeout(
                    remaining,
                    self.reader.read_line(&mut self.line_buffer),
                )
                .await
                {
                    Ok(Ok(0)) | Ok(Err(_)) | Err(_) => break,
                    Ok(Ok(_)) => continue,
                }
            }

            // Only clear after recovery completes, so cancellation
            // during recovery retries next time
            self.needs_recovery = false;
            Ok(())
        })
    }

    fn transaction<'a>(&'a mut self, requests: &'a mut [Request]) -> BoxFuture<'a, Result<()>> {
        Box::pin(self.execute_cycle(requests))
    }

    fn write<'a>(&'a mut self, frame: &'a CanFdFrame) -> BoxFuture<'a, Result<()>> {
        Box::pin(self.send_frame(frame))
    }

    fn read(&mut self) -> BoxFuture<'_, Result<Option<CanFdFrame>>> {
        Box::pin(async move {
            // Drain pending frames first
            if let Some(frame) = self.pending_frames.pop() {
                return Ok(Some(frame));
            }

            // Wait indefinitely for a frame — caller wraps with tokio::time::timeout()
            loop {
                self.line_buffer.clear();

                let n = self.reader.read_line(&mut self.line_buffer).await?;
                if n == 0 {
                    return Err(Error::Io(std::io::Error::new(
                        std::io::ErrorKind::UnexpectedEof,
                        "Serial port closed",
                    )));
                }

                if let Some(frame) = FdcanusbProtocol::parse_frame(&self.line_buffer) {
                    return Ok(Some(frame));
                }
            }
        })
    }

    fn flush(&mut self) -> BoxFuture<'_, Result<()>> {
        Box::pin(async move {
            // Clear pending frames
            self.pending_frames.clear();

            // Drain any incoming data for a short period
            let flush_timeout = std::time::Duration::from_millis(50);
            let deadline = tokio::time::Instant::now() + flush_timeout;

            while tokio::time::Instant::now() < deadline {
                let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
                if remaining.is_zero() {
                    break;
                }

                self.line_buffer.clear();

                let read_result = tokio::time::timeout(
                    remaining,
                    self.reader.read_line(&mut self.line_buffer),
                )
                .await;

                match read_result {
                    Ok(Ok(0)) | Ok(Err(_)) | Err(_) => break,
                    Ok(Ok(_)) => continue, // Discard
                }
            }

            Ok(())
        })
    }

    fn info(&self) -> &TransportDeviceInfo {
        &self.info
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_protocol_reuse() {
        // Verify we can use the blocking protocol encoder
        let mut frame = CanFdFrame::new();
        frame.arbitration_id = 0x8001;
        frame.data[0..3].copy_from_slice(&[0x01, 0x00, 0x0A]);
        frame.size = 3;
        frame.set_brs(true);
        frame.set_fdcan(true);

        let encoded = FdcanusbProtocol::encode_frame(&frame, false);
        assert!(encoded.starts_with("can send 8001"));
    }
}
