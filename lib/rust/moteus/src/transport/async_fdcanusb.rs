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

//! Async fdcanusb transport using tokio.
//!
//! This module provides an async implementation of the fdcanusb transport
//! using tokio for non-blocking I/O. It reuses the protocol encoder/decoder
//! from the blocking implementation.
//!
//! Like the blocking transport, it also supports a moteus connected
//! directly over a TTL UART (see `docs/integration/uart.md`):
//! non-pipelined operation with per-frame retries and CRC-8 checksums.
//!
//! # Example
//!
//! ```no_run
//! use moteus::transport::async_fdcanusb::AsyncFdcanusbDevice;
//! use moteus::transport::device::AsyncTransportDevice;
//! use moteus::transport::Request;
//! use moteus::CanFdFrame;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), moteus::Error> {
//!     let mut device = AsyncFdcanusbDevice::open("/dev/ttyACM0").await?;
//!     let mut requests = [Request::new(CanFdFrame::new())];
//!     device.transaction(&mut requests).await?;
//!     let responses = requests[0].responses.take();
//!     Ok(())
//! }
//! ```

use crate::error::{Error, Result};
use crate::transport::async_transport::BoxFuture;
use crate::transport::device::{AsyncTransportDevice, TransportDeviceInfo};
use crate::transport::fdcanusb::{is_retryable_error, FdcanusbOptions, FdcanusbProtocol};
use crate::transport::transaction::{dispatch_frame, Request};
use moteus_protocol::fdcanusb as codec;
use moteus_protocol::CanFdFrame;

use tokio::io::{
    AsyncBufReadExt, AsyncRead, AsyncWrite, AsyncWriteExt, BufReader, ReadHalf, WriteHalf,
};
use tokio_serial::SerialStream;

/// Async fdcanusb transport using tokio-serial.
///
/// This provides an async interface for communicating with moteus
/// controllers through an fdcanusb device, or directly over a UART.
///
/// The stream type is generic to allow testing with in-memory streams
/// (e.g. `tokio::io::duplex`); normal use opens a [`SerialStream`] via
/// [`AsyncFdcanusbDevice::open`].
pub struct AsyncFdcanusbDevice<S = SerialStream> {
    reader: BufReader<ReadHalf<S>>,
    writer: WriteHalf<S>,
    timeout: std::time::Duration,
    disable_brs: bool,
    /// When true, non-pipelined operation with per-frame retry is
    /// used (a moteus connected directly over UART).
    uart_mode: bool,
    /// When true, CRC-8 checksums are appended to sent lines and
    /// required on received lines.
    checksum_active: bool,
    max_retries: u32,
    line_buffer: String,
    pending_frames: Vec<CanFdFrame>,
    pub(crate) info: TransportDeviceInfo,
    /// Set when a previous operation was cancelled mid-flight.
    needs_recovery: bool,
    /// Set when a previous operation may have left late responses in
    /// flight (a timeout, retry, or reply shortfall); the next
    /// operation flushes them first so they are not misattributed.
    needs_flush: bool,
}

impl<S> std::fmt::Debug for AsyncFdcanusbDevice<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AsyncFdcanusbDevice")
            .field("info", &self.info)
            .field("timeout", &self.timeout)
            .field("disable_brs", &self.disable_brs)
            .field("uart_mode", &self.uart_mode)
            .field("checksum_active", &self.checksum_active)
            .field("pending_frames", &self.pending_frames.len())
            .field("needs_recovery", &self.needs_recovery)
            .field("needs_flush", &self.needs_flush)
            .finish()
    }
}

impl AsyncFdcanusbDevice {
    /// Opens an async fdcanusb transport at the specified path.
    ///
    /// As with the blocking
    /// [`FdcanusbDevice::new`](crate::transport::fdcanusb::FdcanusbDevice::new),
    /// devices that cannot be conclusively identified as fdcanusbs by
    /// USB VID/PID are treated as moteus UART connections.
    ///
    /// # Arguments
    /// * `path` - Device path (e.g., "/dev/ttyACM0")
    pub async fn open(path: &str) -> Result<Self> {
        Self::open_with(path, &FdcanusbOptions::new()).await
    }

    /// Opens an async fdcanusb transport with BRS option.
    ///
    /// # Arguments
    /// * `path` - Device path (e.g., "/dev/ttyACM0")
    /// * `disable_brs` - Whether to disable bit rate switching
    pub async fn open_with_brs(path: &str, disable_brs: bool) -> Result<Self> {
        Self::open_with(path, &FdcanusbOptions::new().disable_brs(disable_brs)).await
    }

    /// Opens an async fdcanusb transport with timeout and BRS options.
    ///
    /// # Arguments
    /// * `path` - Device path (e.g., "/dev/ttyACM0")
    /// * `timeout` - Communication timeout
    /// * `disable_brs` - Whether to disable bit rate switching
    pub async fn open_with_options(
        path: &str,
        timeout: std::time::Duration,
        disable_brs: bool,
    ) -> Result<Self> {
        Self::open_with(
            path,
            &FdcanusbOptions::new()
                .timeout(timeout)
                .disable_brs(disable_brs),
        )
        .await
    }

    /// Opens an async fdcanusb or moteus UART device with full options.
    pub async fn open_with(path: &str, options: &FdcanusbOptions) -> Result<Self> {
        let builder = tokio_serial::new(path, options.baudrate);
        let mut port = SerialStream::open(&builder).map_err(|e| Error::Io(e.into()))?;

        // Some platforms (notably Windows) do not deliver data from a CDC
        // ACM device until DTR is asserted.  Failure is non-fatal.
        {
            use tokio_serial::SerialPort;
            let _ = port.write_data_terminal_ready(true);
        }

        let mut device = Self::from_stream_with(port, options);

        // Auto-detect UART connections unless the caller decided.
        if options.uart_mode.is_none() && !crate::transport::discovery::is_fdcanusb_path(path) {
            device.uart_mode = true;
            device.checksum_active = true;
        }

        Ok(device)
    }
}

impl<S: AsyncRead + AsyncWrite + Send> AsyncFdcanusbDevice<S> {
    /// Creates an async fdcanusb transport from a pre-opened stream
    /// with full options.
    ///
    /// This is primarily useful for testing with in-memory streams.
    /// No auto-detection is performed: `uart_mode: None` is treated as
    /// false.
    pub fn from_stream_with(stream: S, options: &FdcanusbOptions) -> Self {
        let (reader, writer) = tokio::io::split(stream);
        let reader = BufReader::new(reader);

        Self {
            reader,
            writer,
            timeout: options.timeout,
            disable_brs: options.disable_brs,
            uart_mode: options.uart_mode.unwrap_or(false),
            checksum_active: options.checksum_enabled,
            max_retries: options.max_retries,
            line_buffer: String::new(),
            pending_frames: Vec::new(),
            info: TransportDeviceInfo::new(0, "AsyncFdcanusb"),
            needs_recovery: false,
            needs_flush: false,
        }
    }

    /// Enables or disables bit rate switching.
    pub fn set_disable_brs(&mut self, disable: bool) {
        self.disable_brs = disable;
    }

    /// Returns true if this device is operating in UART mode.
    pub fn uart_mode(&self) -> bool {
        self.uart_mode
    }

    /// Returns true if CRC-8 checksums are currently active.
    pub fn checksum_active(&self) -> bool {
        self.checksum_active
    }

    /// Writes a frame without waiting for OK response.
    async fn write_frame(&mut self, frame: &CanFdFrame) -> Result<()> {
        let cmd = FdcanusbProtocol::encode_frame_with_options(
            frame,
            self.disable_brs,
            self.checksum_active,
        );
        self.writer.write_all(cmd.as_bytes()).await?;
        Ok(())
    }

    /// Sends a single frame and waits for OK response, applying UART
    /// retry semantics when enabled.
    async fn send_frame(&mut self, frame: &CanFdFrame) -> Result<()> {
        let mut timeout = self.timeout;
        let mut attempt = 0;

        loop {
            self.write_frame(frame).await?;
            self.writer.flush().await?;

            match self.wait_for_ok(timeout).await {
                Ok(()) => {
                    if attempt > 0 {
                        // A retried command may still produce late
                        // responses from the earlier attempts.
                        self.needs_flush = true;
                    }
                    return Ok(());
                }
                Err(e)
                    if self.uart_mode && attempt < self.max_retries && is_retryable_error(&e) =>
                {
                    attempt += 1;
                    // Exponential backoff: 1.5x per attempt.
                    timeout += timeout / 2;
                }
                Err(e) => {
                    self.needs_flush = true;
                    return Err(e);
                }
            }
        }
    }

    /// Reads one complete line (terminated by '\n') into `line_buffer`,
    /// waiting until `deadline`.
    ///
    /// Returns `Ok(true)` when a complete line is available and
    /// `Ok(false)` when the deadline expires first.
    ///
    /// This is built on `fill_buf`, which is cancellation safe, rather
    /// than `read_line`, which is not: a deadline expiring (or the
    /// whole future being dropped) mid-line retains the partial data
    /// in `line_buffer` instead of losing it and mis-framing the
    /// stream.
    ///
    /// `line_buffer` never holds more than one line: only bytes up to
    /// and including the first '\n' are consumed from the underlying
    /// reader, so any following lines remain buffered there.  The
    /// caller clears `line_buffer` once it has processed the line,
    /// which therefore discards nothing.
    async fn read_line_deadline(&mut self, deadline: tokio::time::Instant) -> Result<bool> {
        loop {
            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            if remaining.is_zero() {
                return Ok(false);
            }

            let available = match tokio::time::timeout(remaining, self.reader.fill_buf()).await {
                Err(_) => return Ok(false), // Deadline expired
                Ok(Err(e)) => return Err(Error::Io(e)),
                Ok(Ok([])) => {
                    return Err(Error::Io(std::io::Error::new(
                        std::io::ErrorKind::UnexpectedEof,
                        "Serial port closed",
                    )));
                }
                Ok(Ok(available)) => available,
            };

            let (take, complete) = match available.iter().position(|&b| b == b'\n') {
                Some(pos) => (pos + 1, true),
                None => (available.len(), false),
            };
            self.line_buffer
                .push_str(&String::from_utf8_lossy(&available[..take]));
            self.reader.consume(take);

            if complete {
                return Ok(true);
            }
        }
    }

    /// Applies the checksum policy to the complete line in
    /// `line_buffer`, stripping a valid checksum in place.
    ///
    /// Returns false if the line must be discarded: its checksum is
    /// invalid, or checksum mode is active and it has none.
    fn apply_checksum_policy(&mut self) -> bool {
        let (content_len, had_checksum, valid) = {
            let trimmed = self.line_buffer.trim_end();
            let (content, had_checksum, valid) = codec::strip_checksum(trimmed.as_bytes());
            (content.len(), had_checksum, valid)
        };

        if had_checksum {
            if !valid {
                return false;
            }
            // The checksum and trailing whitespace are a suffix, so
            // the content is a prefix of the buffer.
            self.line_buffer.truncate(content_len);
            true
        } else {
            !self.checksum_active
        }
    }

    /// Waits for an OK response with timeout.
    /// Any received frames (rcv lines) are buffered for later retrieval.
    async fn wait_for_ok(&mut self, timeout: std::time::Duration) -> Result<()> {
        let deadline = tokio::time::Instant::now() + timeout;

        loop {
            if !self.read_line_deadline(deadline).await? {
                return Err(Error::Timeout);
            }

            if !self.apply_checksum_policy() {
                self.line_buffer.clear();
                continue;
            }

            let line = self.line_buffer.trim();
            if FdcanusbProtocol::is_ok_response(line) {
                self.line_buffer.clear();
                return Ok(());
            }
            if FdcanusbProtocol::is_error_response(line) {
                // A checksum error means the device requires
                // checksums; enable them so any retry succeeds.
                if FdcanusbProtocol::is_checksum_error(line) {
                    self.checksum_active = true;
                }
                let error = Error::Device {
                    message: line.to_string(),
                    retryable: FdcanusbProtocol::is_retryable_error_response(line),
                };
                self.line_buffer.clear();
                return Err(error);
            }
            // Save any received frames for later retrieval
            if let Some(frame) = FdcanusbProtocol::parse_frame(&self.line_buffer) {
                self.pending_frames.push(frame);
            }
            self.line_buffer.clear();
        }
    }

    /// Receives frames with timeout.
    async fn receive_frames(&mut self, expected_count: usize) -> Result<Vec<CanFdFrame>> {
        // First, drain any frames that were buffered during wait_for_ok
        let mut frames: Vec<CanFdFrame> = self.pending_frames.drain(..).collect();

        if frames.len() >= expected_count {
            return Ok(frames);
        }

        let deadline = tokio::time::Instant::now() + self.timeout;

        while frames.len() < expected_count {
            if !self.read_line_deadline(deadline).await? {
                // Timeout - return what we have, but flush before the
                // next transaction in case the replies arrive late.
                self.needs_flush = true;
                break;
            }

            if !self.apply_checksum_policy() {
                self.line_buffer.clear();
                continue;
            }

            if let Some(frame) = FdcanusbProtocol::parse_frame(&self.line_buffer) {
                frames.push(frame);
            }
            self.line_buffer.clear();
        }

        Ok(frames)
    }

    /// Waits for the replies to request `target`, dispatching every
    /// received frame to whichever request it matches.
    ///
    /// Only frames matching `target`'s filter count toward its
    /// expected reply count; a late reply belonging to a different
    /// request is routed there instead of being miscounted.
    async fn receive_replies_uart(
        &mut self,
        requests: &mut [Request],
        target: usize,
    ) -> Result<()> {
        // Dispatch any frames buffered during wait_for_ok first.
        for frame in std::mem::take(&mut self.pending_frames) {
            dispatch_frame(&frame, requests);
        }

        let expected = requests[target].expected_reply_count as usize;
        let deadline = tokio::time::Instant::now() + self.timeout;

        while requests[target].responses.len() < expected {
            if !self.read_line_deadline(deadline).await? {
                // Timeout - tolerate the shortfall, but flush before
                // the next transaction in case the replies arrive
                // late.
                self.needs_flush = true;
                break;
            }

            if !self.apply_checksum_policy() {
                self.line_buffer.clear();
                continue;
            }

            if let Some(frame) = FdcanusbProtocol::parse_frame(&self.line_buffer) {
                dispatch_frame(&frame, requests);
            }
            self.line_buffer.clear();
        }

        Ok(())
    }

    /// Execute a pipelined cycle sending frames and collecting responses.
    async fn execute_cycle_pipelined(&mut self, requests: &mut [Request]) -> Result<()> {
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
            self.wait_for_ok(self.timeout).await?;
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

        Ok(())
    }

    /// Execute a UART-mode cycle: one frame at a time, each
    /// acknowledged (with retries) before its replies are awaited.
    ///
    /// A UART link is lossy, so pipelining is avoided and reply
    /// timeouts are tolerated — the caller receives whatever responses
    /// arrived.
    async fn execute_cycle_uart(&mut self, requests: &mut [Request]) -> Result<()> {
        for i in 0..requests.len() {
            let Some(frame) = requests[i].frame.clone() else {
                continue;
            };

            self.send_frame(&frame).await?;

            if requests[i].expected_reply_count > 0 {
                self.receive_replies_uart(requests, i).await?;
            }
        }

        Ok(())
    }

    /// Execute a cycle sending frames and collecting responses.
    async fn execute_cycle(&mut self, requests: &mut [Request]) -> Result<()> {
        debug_assert!(
            requests.iter().all(|r| r.child_device.is_none()),
            "AsyncFdcanusbDevice does not support child devices"
        );

        // Discard anything left over from a cancelled or failed
        // previous operation.
        self.recover_impl().await?;

        self.needs_recovery = true;

        let result = if self.uart_mode {
            self.execute_cycle_uart(requests).await
        } else {
            self.execute_cycle_pipelined(requests).await
        };

        match &result {
            Ok(()) => self.needs_recovery = false,
            Err(_) => self.needs_flush = true,
        }
        result
    }

    /// Restores the device to a known-good state after a possible
    /// cancellation or an operation that may have left late responses
    /// in flight.
    async fn recover_impl(&mut self) -> Result<()> {
        if !self.needs_recovery && !self.needs_flush {
            return Ok(());
        }
        // Discard stale pending frames
        self.pending_frames.clear();

        if self.needs_recovery {
            // Send bare newline to terminate any partial command from
            // a cancelled write_all
            let _ = self.writer.write_all(b"\n").await;
            let _ = self.writer.flush().await;
        }

        // Drain stale responses with short timeout
        let drain_timeout = std::time::Duration::from_millis(20);
        let deadline = tokio::time::Instant::now() + drain_timeout;
        while let Ok(true) = self.read_line_deadline(deadline).await {
            self.line_buffer.clear();
        }

        // Discard any partial line left by the deadline expiring.
        self.line_buffer.clear();

        // Only clear after recovery completes, so cancellation during
        // recovery retries next time
        self.needs_recovery = false;
        self.needs_flush = false;
        Ok(())
    }
}

impl<S: AsyncRead + AsyncWrite + Send> AsyncTransportDevice for AsyncFdcanusbDevice<S> {
    fn recover(&mut self) -> BoxFuture<'_, Result<()>> {
        Box::pin(self.recover_impl())
    }

    fn transaction<'a>(&'a mut self, requests: &'a mut [Request]) -> BoxFuture<'a, Result<()>> {
        Box::pin(self.execute_cycle(requests))
    }

    fn write<'a>(&'a mut self, frame: &'a CanFdFrame) -> BoxFuture<'a, Result<()>> {
        Box::pin(async move {
            self.recover_impl().await?;
            self.send_frame(frame).await
        })
    }

    fn read(&mut self) -> BoxFuture<'_, Result<Option<CanFdFrame>>> {
        Box::pin(async move {
            // Drain pending frames first
            if let Some(frame) = self.pending_frames.pop() {
                return Ok(Some(frame));
            }

            // Wait indefinitely for a frame — callers wrap this with
            // tokio::time::timeout().  fill_buf is cancellation safe,
            // so a caller's timeout firing mid-line retains the
            // partial data in line_buffer for the next call.
            loop {
                let available = self.reader.fill_buf().await?;
                if available.is_empty() {
                    return Err(Error::Io(std::io::Error::new(
                        std::io::ErrorKind::UnexpectedEof,
                        "Serial port closed",
                    )));
                }

                let (take, complete) = match available.iter().position(|&b| b == b'\n') {
                    Some(pos) => (pos + 1, true),
                    None => (available.len(), false),
                };
                self.line_buffer
                    .push_str(&String::from_utf8_lossy(&available[..take]));
                self.reader.consume(take);

                if !complete {
                    continue;
                }

                if !self.apply_checksum_policy() {
                    self.line_buffer.clear();
                    continue;
                }

                let frame = FdcanusbProtocol::parse_frame(&self.line_buffer);
                self.line_buffer.clear();
                if frame.is_some() {
                    return Ok(frame);
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

            while let Ok(true) = self.read_line_deadline(deadline).await {
                self.line_buffer.clear(); // Discard
            }

            // Discard any partial line left by the deadline expiring.
            self.line_buffer.clear();
            self.needs_flush = false;

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
    use crate::transport::transaction::FrameFilter;

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

    fn test_frame() -> CanFdFrame {
        let mut frame = CanFdFrame::new();
        frame.arbitration_id = 0x0105;
        frame.data[..4].copy_from_slice(b"0123");
        frame.size = 4;
        frame
    }

    /// Formats a response line with a valid checksum appended.
    fn checksummed(content: &str) -> String {
        let with_space = format!("{} ", content);
        format!(
            "{}*{:02X}\n",
            with_space,
            codec::compute_crc8(with_space.as_bytes())
        )
    }

    fn uart_options() -> FdcanusbOptions {
        FdcanusbOptions::new()
            .uart_mode(true)
            .timeout(std::time::Duration::from_millis(20))
    }

    /// Reads one full command line from the device side of the duplex.
    async fn read_command(device_side: &mut tokio::io::DuplexStream) -> String {
        use tokio::io::AsyncReadExt;
        let mut line = Vec::new();
        let mut byte = [0u8; 1];
        loop {
            device_side.read_exact(&mut byte).await.unwrap();
            if byte[0] == b'\n' {
                break;
            }
            line.push(byte[0]);
        }
        String::from_utf8(line).unwrap()
    }

    #[tokio::test]
    async fn test_uart_checksum_send_and_receive() {
        use tokio::io::AsyncWriteExt;

        let (host_side, mut device_side) = tokio::io::duplex(1024);
        let mut device = AsyncFdcanusbDevice::from_stream_with(
            host_side,
            &uart_options().checksum_enabled(true),
        );

        let server = tokio::spawn(async move {
            let command = read_command(&mut device_side).await;

            // The sent command must end with a valid " *XX" checksum.
            let (content, had, valid) = codec::strip_checksum(command.as_bytes());
            assert!(had, "no checksum on: {}", command);
            assert!(valid);
            assert!(content.starts_with(b"can send 0105 30313233"));

            let mut response = checksummed("OK");
            response.push_str(&checksummed("rcv 0105 2030"));
            device_side.write_all(response.as_bytes()).await.unwrap();
            device_side
        });

        let mut requests = [Request::new(test_frame())
            .with_filter(FrameFilter::Any)
            .with_expected_replies(1)];
        device.transaction(&mut requests).await.unwrap();

        let responses = requests[0].responses.take();
        assert_eq!(responses.len(), 1);
        assert_eq!(responses[0].payload(), &[0x20, 0x30]);

        server.await.unwrap();
    }

    #[tokio::test]
    async fn test_uart_retry_on_timeout() {
        use tokio::io::AsyncWriteExt;

        let (host_side, mut device_side) = tokio::io::duplex(1024);
        let mut device = AsyncFdcanusbDevice::from_stream_with(host_side, &uart_options());

        let server = tokio::spawn(async move {
            // Drop the first command, acknowledge the second.
            let _ = read_command(&mut device_side).await;
            let _ = read_command(&mut device_side).await;
            device_side.write_all(b"OK\n").await.unwrap();
            device_side
        });

        device.write(&test_frame()).await.unwrap();
        server.await.unwrap();
    }

    #[tokio::test]
    async fn test_uart_err_checksum_enables_checksums() {
        use tokio::io::AsyncWriteExt;

        let (host_side, mut device_side) = tokio::io::duplex(1024);
        let mut device = AsyncFdcanusbDevice::from_stream_with(host_side, &uart_options());
        assert!(!device.checksum_active());

        let server = tokio::spawn(async move {
            let first = read_command(&mut device_side).await;
            assert!(!first.contains('*'), "first send unchecksummed");
            device_side
                .write_all(b"ERR missing checksum\n")
                .await
                .unwrap();

            let second = read_command(&mut device_side).await;
            let (_, had, valid) = codec::strip_checksum(second.as_bytes());
            assert!(had && valid, "retry must carry a checksum");
            device_side
                .write_all(checksummed("OK").as_bytes())
                .await
                .unwrap();
            device_side
        });

        device.write(&test_frame()).await.unwrap();
        assert!(device.checksum_active());
        server.await.unwrap();
    }

    #[tokio::test]
    async fn test_uart_retries_exhausted() {
        let (host_side, _device_side) = tokio::io::duplex(1024);
        let mut device =
            AsyncFdcanusbDevice::from_stream_with(host_side, &uart_options().max_retries(2));

        // No response ever arrives.
        assert!(matches!(
            device.write(&test_frame()).await,
            Err(Error::Timeout)
        ));
    }

    #[tokio::test]
    async fn test_partial_line_survives_deadline() {
        use tokio::io::AsyncWriteExt;

        let (host_side, mut device_side) = tokio::io::duplex(1024);
        let mut device = AsyncFdcanusbDevice::from_stream_with(host_side, &uart_options());

        let server = tokio::spawn(async move {
            // First command: reply with only part of the OK line, then
            // stall past the per-attempt deadline.
            let _ = read_command(&mut device_side).await;
            device_side.write_all(b"O").await.unwrap();

            // Second (retried) command: complete the line.  If the
            // partial 'O' had been lost to cancellation, this would
            // arrive as a bare 'K' garbage line and the send would
            // exhaust its retries.
            let _ = read_command(&mut device_side).await;
            tokio::time::sleep(std::time::Duration::from_millis(5)).await;
            device_side.write_all(b"K\n").await.unwrap();

            // Keep the device side open until the host finishes.
            tokio::time::sleep(std::time::Duration::from_millis(200)).await;
            device_side
        });

        device.write(&test_frame()).await.unwrap();
        server.await.unwrap();
    }
}
