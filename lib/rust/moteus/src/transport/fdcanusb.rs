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

//! fdcanusb transport for communicating with moteus controllers.
//!
//! The fdcanusb is a USB-to-CAN-FD adapter that uses a simple text protocol:
//! - Send: `can send AAAA HEXDATA FLAGS\n`
//! - Receive: `rcv AAAA HEXDATA FLAGS`
//! - OK response after each send

use crate::error::{Error, Result};
use crate::transport::device::{TransportDevice, TransportDeviceInfo};
use crate::transport::transaction::{dispatch_frame, Request};
use moteus_protocol::fdcanusb as codec;
use moteus_protocol::CanFdFrame;
use std::io::{BufRead, BufReader};
use std::time::{Duration, Instant};

/// Default serial baud rate.
///
/// fdcanusb devices ignore this since they are USB CDC ACM.  For UART
/// connections it must match the moteus configuration, which defaults
/// to 921600.
pub const DEFAULT_BAUDRATE: u32 = 921600;

/// Default number of retry attempts for UART mode.
pub const DEFAULT_MAX_RETRIES: u32 = 3;

/// Options for opening an [`FdcanusbDevice`].
#[non_exhaustive]
#[derive(Debug, Clone)]
pub struct FdcanusbOptions {
    /// Communication timeout.  In UART mode this is the per-attempt
    /// OK timeout; it grows by 1.5x on each retry.
    pub timeout: Duration,
    /// Disable CAN-FD bit rate switching.
    pub disable_brs: bool,
    /// Serial baud rate; see [`DEFAULT_BAUDRATE`].
    pub baudrate: u32,
    /// Treat the device as a moteus connected directly over UART
    /// rather than an fdcanusb: non-pipelined operation with
    /// per-frame retries.
    ///
    /// `None` auto-detects: devices that cannot be conclusively
    /// identified as fdcanusbs by USB VID/PID are treated as UARTs
    /// (with checksums enabled).
    pub uart_mode: Option<bool>,
    /// Append CRC-8 checksums to sent lines and require them on
    /// received lines.  Even when false, checksums are enabled
    /// dynamically if the device reports a checksum error.
    pub checksum_enabled: bool,
    /// Maximum number of retry attempts in UART mode.
    pub max_retries: u32,
}

impl Default for FdcanusbOptions {
    fn default() -> Self {
        Self::new()
    }
}

impl FdcanusbOptions {
    /// Creates options with default values.
    pub fn new() -> Self {
        Self {
            timeout: crate::transport::factory::DEFAULT_TIMEOUT,
            disable_brs: false,
            baudrate: DEFAULT_BAUDRATE,
            uart_mode: None,
            checksum_enabled: false,
            max_retries: DEFAULT_MAX_RETRIES,
        }
    }

    /// Sets the communication timeout.
    #[must_use]
    pub fn timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Disables CAN-FD bit rate switching.
    #[must_use]
    pub fn disable_brs(mut self, disable: bool) -> Self {
        self.disable_brs = disable;
        self
    }

    /// Sets the serial baud rate.
    #[must_use]
    pub fn baudrate(mut self, baudrate: u32) -> Self {
        self.baudrate = baudrate;
        self
    }

    /// Explicitly enables or disables UART mode, skipping
    /// auto-detection.
    #[must_use]
    pub fn uart_mode(mut self, uart_mode: bool) -> Self {
        self.uart_mode = Some(uart_mode);
        self
    }

    /// Enables CRC-8 checksums from the start.
    #[must_use]
    pub fn checksum_enabled(mut self, enabled: bool) -> Self {
        self.checksum_enabled = enabled;
        self
    }

    /// Sets the maximum number of UART retry attempts.
    #[must_use]
    pub fn max_retries(mut self, max_retries: u32) -> Self {
        self.max_retries = max_retries;
        self
    }
}

/// fdcanusb text protocol encoder/decoder.
///
/// These are thin `String`-based wrappers over the allocation-free
/// codec in [`moteus_protocol::fdcanusb`], which embedded users can
/// use directly.
pub struct FdcanusbProtocol;

impl FdcanusbProtocol {
    /// Encodes a CAN-FD frame into the fdcanusb text protocol format.
    ///
    /// Returns a string like: `can send 8001 0123456789ABCDEF bF\n`
    pub fn encode_frame(frame: &CanFdFrame, disable_brs: bool) -> String {
        Self::encode_frame_with_options(frame, disable_brs, false)
    }

    /// Encodes a CAN-FD frame, optionally appending a ` *XX` CRC-8
    /// checksum as used over UART connections.
    pub fn encode_frame_with_options(
        frame: &CanFdFrame,
        disable_brs: bool,
        checksum: bool,
    ) -> String {
        let mut buf = [0u8; codec::MAX_LINE_LENGTH];
        let len = codec::encode_can_send(
            frame,
            &codec::EncodeOptions {
                disable_brs,
                checksum,
            },
            &mut buf,
        )
        .expect("MAX_LINE_LENGTH is always sufficient");
        // The encoded line is pure ASCII.
        String::from_utf8_lossy(&buf[..len]).into_owned()
    }

    /// Parses a received frame line from fdcanusb.
    ///
    /// Input format: `rcv AAAA HEXDATA [E] [B] [F]`
    pub fn parse_frame(line: &str) -> Option<CanFdFrame> {
        codec::parse_rcv(line.as_bytes())
    }

    /// Checks if a line is an OK response.
    pub fn is_ok_response(line: &str) -> bool {
        line.trim().starts_with("OK")
    }

    /// Checks if a line is an error response.
    pub fn is_error_response(line: &str) -> bool {
        let trimmed = line.trim();
        !trimmed.is_empty() && !trimmed.starts_with("OK") && !trimmed.starts_with("rcv")
    }

    /// Checks if an error response indicates that the device requires
    /// checksums.
    pub fn is_checksum_error(line: &str) -> bool {
        codec::is_checksum_error(line.as_bytes())
    }

    /// Checks if an error response is likely to be resolved by
    /// retransmitting the command (possibly with checksums enabled):
    /// the device either demanded checksums or received a corrupted
    /// command it could not interpret.
    pub fn is_retryable_error_response(line: &str) -> bool {
        let lower = line.to_lowercase();
        lower.contains("checksum") || lower.contains("unknown command")
    }
}

/// fdcanusb transport implementation.
///
/// This provides a synchronous interface for communicating with moteus
/// controllers through an fdcanusb device.
pub struct FdcanusbDevice<S: std::io::Read + std::io::Write> {
    reader: BufReader<S>,
    timeout: Duration,
    disable_brs: bool,
    /// When true, non-pipelined operation with per-frame retry is
    /// used (a moteus connected directly over UART).
    uart_mode: bool,
    /// When true, CRC-8 checksums are appended to sent lines and
    /// required on received lines.
    checksum_active: bool,
    max_retries: u32,
    line_buffer: String,
    /// Buffer for frames received during wait_for_ok.
    pending_frames: Vec<CanFdFrame>,
    /// Set when a previous operation may have left late responses in
    /// flight (a timeout, retry, or reply shortfall); the next
    /// transaction flushes them first so they are not misattributed.
    needs_flush: bool,
    /// Device info for TransportDevice trait.
    pub(crate) info: TransportDeviceInfo,
}

impl<S: std::io::Read + std::io::Write> std::fmt::Debug for FdcanusbDevice<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FdcanusbDevice")
            .field("info", &self.info)
            .field("timeout", &self.timeout)
            .field("disable_brs", &self.disable_brs)
            .field("uart_mode", &self.uart_mode)
            .field("checksum_active", &self.checksum_active)
            .field("pending_frames", &self.pending_frames.len())
            .field("needs_flush", &self.needs_flush)
            .finish()
    }
}

impl<S: std::io::Read + std::io::Write> FdcanusbDevice<S> {
    /// Creates an fdcanusb transport from a pre-opened stream.
    ///
    /// This is primarily useful for testing with mock streams.
    /// For normal use, prefer [`FdcanusbDevice::new`] which opens the serial port
    /// directly from a device path.
    pub fn from_stream(stream: S) -> Self {
        Self::from_stream_with(stream, &FdcanusbOptions::new())
    }

    /// Creates an fdcanusb transport from a pre-opened stream with options.
    ///
    /// This is primarily useful for testing with mock streams.
    /// For normal use, prefer [`FdcanusbDevice::new`] which opens the serial port
    /// directly from a device path.
    pub fn from_stream_with_options(stream: S, timeout: Duration, disable_brs: bool) -> Self {
        Self::from_stream_with(
            stream,
            &FdcanusbOptions::new()
                .timeout(timeout)
                .disable_brs(disable_brs),
        )
    }

    /// Creates an fdcanusb transport from a pre-opened stream with
    /// full options.
    ///
    /// No auto-detection is performed: `uart_mode: None` is treated as
    /// false.
    pub fn from_stream_with(stream: S, options: &FdcanusbOptions) -> Self {
        FdcanusbDevice {
            reader: BufReader::new(stream),
            timeout: options.timeout,
            disable_brs: options.disable_brs,
            uart_mode: options.uart_mode.unwrap_or(false),
            checksum_active: options.checksum_enabled,
            max_retries: options.max_retries,
            line_buffer: String::new(),
            pending_frames: Vec::new(),
            needs_flush: false,
            info: TransportDeviceInfo::new(0, "Fdcanusb"),
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
    fn write_frame(&mut self, frame: &CanFdFrame) -> Result<()> {
        let cmd = FdcanusbProtocol::encode_frame_with_options(
            frame,
            self.disable_brs,
            self.checksum_active,
        );
        self.reader.get_mut().write_all(cmd.as_bytes())?;
        Ok(())
    }

    /// Sends a single frame and waits for OK response, applying UART
    /// retry semantics when enabled.
    fn send_frame(&mut self, frame: &CanFdFrame) -> Result<()> {
        let mut timeout = self.timeout;
        let mut attempt = 0;

        loop {
            self.write_frame(frame)?;
            self.reader.get_mut().flush()?;

            match self.wait_for_ok_timeout(timeout) {
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
    /// Returns `Ok(true)` when a complete line is available and `Ok(false)`
    /// when the deadline expires first.  Partial data received before the
    /// deadline is retained in `line_buffer` so it is not lost across
    /// timeout-bounded reads.
    ///
    /// `line_buffer` never holds more than one line: `read_line` only
    /// consumes bytes up to and including the first '\n', so any
    /// following lines remain buffered in the underlying reader.  The
    /// caller clears `line_buffer` once it has processed the line,
    /// which therefore discards nothing.
    fn read_line_deadline(&mut self, deadline: Instant) -> Result<bool> {
        loop {
            if Instant::now() > deadline {
                return Ok(false);
            }

            match self.reader.read_line(&mut self.line_buffer) {
                Ok(0) => {
                    // EOF: the device went away (serial ports signal
                    // "no data yet" with TimedOut/WouldBlock errors,
                    // never a zero-length read).
                    return Err(Error::Io(std::io::Error::new(
                        std::io::ErrorKind::UnexpectedEof,
                        "Serial port closed",
                    )));
                }
                Ok(_) => {
                    if self.line_buffer.ends_with('\n') {
                        return Ok(true);
                    }
                    // Partial line - keep accumulating
                }
                Err(e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut =>
                {
                    continue
                }
                Err(e) => return Err(Error::Io(e)),
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

    /// Waits for an OK response.
    /// Any received frames (rcv lines) are buffered for later retrieval.
    fn wait_for_ok(&mut self) -> Result<()> {
        self.wait_for_ok_timeout(self.timeout)
    }

    /// Waits for an OK response with an explicit timeout.
    fn wait_for_ok_timeout(&mut self, timeout: Duration) -> Result<()> {
        let deadline = Instant::now() + timeout;

        loop {
            if !self.read_line_deadline(deadline)? {
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
    fn receive_frames(&mut self, expected_count: usize) -> Result<Vec<CanFdFrame>> {
        // First, drain any frames that were buffered during wait_for_ok
        let mut frames: Vec<CanFdFrame> = self.pending_frames.drain(..).collect();

        if frames.len() >= expected_count {
            return Ok(frames);
        }

        let deadline = Instant::now() + self.timeout;

        while frames.len() < expected_count {
            if !self.read_line_deadline(deadline)? {
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
}

/// Returns true for errors that UART mode should retry: timeouts, and
/// device errors that a retransmission (possibly now with checksums)
/// can resolve.
pub(crate) fn is_retryable_error(error: &Error) -> bool {
    match error {
        Error::Timeout => true,
        Error::Device { retryable, .. } => *retryable,
        _ => false,
    }
}

impl<S: std::io::Read + std::io::Write> FdcanusbDevice<S> {
    /// Executes a transaction in pipelined mode: all frames are
    /// written before waiting for any acknowledgment.
    fn transaction_pipelined(&mut self, requests: &mut [Request]) -> Result<()> {
        // Pipeline: write all frames first
        let mut frames_sent = 0usize;
        for req in requests.iter() {
            if let Some(frame) = &req.frame {
                self.write_frame(frame)?;
                frames_sent += 1;
            }
        }

        // Single flush for all frames
        if frames_sent > 0 {
            self.reader.get_mut().flush()?;
        }

        // Wait for all OKs
        for _ in 0..frames_sent {
            self.wait_for_ok()?;
        }

        // Calculate expected replies
        let expected: usize = Request::total_expected_replies(requests);

        // Receive responses and dispatch to matching requests
        if expected > 0 {
            let responses = self.receive_frames(expected)?;
            for frame in responses {
                dispatch_frame(&frame, requests);
            }
        }

        Ok(())
    }

    /// Executes a transaction in UART mode: one frame at a time, each
    /// acknowledged (with retries) before its replies are awaited.
    ///
    /// A UART link is lossy, so pipelining is avoided and reply
    /// timeouts are tolerated — the caller receives whatever responses
    /// arrived.
    fn transaction_uart(&mut self, requests: &mut [Request]) -> Result<()> {
        for i in 0..requests.len() {
            let Some(frame) = requests[i].frame.clone() else {
                continue;
            };

            self.send_frame(&frame)?;

            if requests[i].expected_reply_count > 0 {
                self.receive_replies_uart(requests, i)?;
            }
        }

        Ok(())
    }

    /// Waits for the replies to request `target`, dispatching every
    /// received frame to whichever request it matches.
    ///
    /// Only frames matching `target`'s filter count toward its
    /// expected reply count; a late reply belonging to a different
    /// request is routed there instead of being miscounted.
    fn receive_replies_uart(&mut self, requests: &mut [Request], target: usize) -> Result<()> {
        // Dispatch any frames buffered during wait_for_ok first.
        for frame in std::mem::take(&mut self.pending_frames) {
            dispatch_frame(&frame, requests);
        }

        let expected = requests[target].expected_reply_count as usize;
        let deadline = Instant::now() + self.timeout;

        while requests[target].responses.len() < expected {
            if !self.read_line_deadline(deadline)? {
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

    /// Discards buffered frames and drains incoming data for a short
    /// period.
    fn drain_input(&mut self) {
        self.pending_frames.clear();

        // Drain any incoming data for a short period, swallowing errors
        let deadline = Instant::now() + Duration::from_millis(50);

        while let Ok(true) = self.read_line_deadline(deadline) {
            self.line_buffer.clear();
        }

        // Discard any partial line left by the deadline expiring
        self.line_buffer.clear();
        self.needs_flush = false;
    }

    /// Discards any late responses from a previous failed or retried
    /// operation, so they are not misattributed to the next one.
    fn flush_if_needed(&mut self) {
        if self.needs_flush {
            self.drain_input();
        }
    }
}

impl<S: std::io::Read + std::io::Write + Send> TransportDevice for FdcanusbDevice<S> {
    fn transaction(&mut self, requests: &mut [Request]) -> Result<()> {
        debug_assert!(
            requests.iter().all(|r| r.child_device.is_none()),
            "FdcanusbDevice does not support child devices"
        );

        self.flush_if_needed();

        let result = if self.uart_mode {
            self.transaction_uart(requests)
        } else {
            self.transaction_pipelined(requests)
        };

        if result.is_err() {
            self.needs_flush = true;
        }
        result
    }

    fn write(&mut self, frame: &CanFdFrame) -> Result<()> {
        self.flush_if_needed();
        self.send_frame(frame)
    }

    fn read(&mut self) -> Result<Option<CanFdFrame>> {
        // Check pending frames first
        if let Some(frame) = self.pending_frames.pop() {
            return Ok(Some(frame));
        }

        // Try to read one frame with a short timeout
        let deadline = Instant::now() + self.timeout;

        loop {
            if !self.read_line_deadline(deadline)? {
                return Ok(None);
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
    }

    fn flush(&mut self) -> Result<()> {
        self.drain_input();
        Ok(())
    }

    fn info(&self) -> &TransportDeviceInfo {
        &self.info
    }

    fn set_timeout(&mut self, timeout: Duration) {
        self.timeout = timeout;
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }
}

// =============================================================================
// Serial port (via the `serialport` crate)
// =============================================================================

/// Read timeout configured on the serial port itself.
///
/// The outer deadline in each read loop governs the total wait; this just
/// bounds how long a single read may block before the deadline is
/// rechecked.
#[cfg(feature = "serialport")]
const PORT_READ_TIMEOUT: Duration = Duration::from_millis(10);

/// Opens and configures a serial port for fdcanusb use.
///
/// fdcanusb devices ignore the baud rate since they are USB CDC ACM;
/// for UART connections it must match the moteus configuration.
#[cfg(feature = "serialport")]
fn open_serial_port(path: &str, baudrate: u32) -> Result<Box<dyn serialport::SerialPort>> {
    let mut port = serialport::new(path, baudrate)
        .timeout(PORT_READ_TIMEOUT)
        .open()
        .map_err(|e| Error::Io(e.into()))?;

    // Some platforms (notably Windows) do not deliver data from a CDC
    // ACM device until DTR is asserted.  Failure is non-fatal.
    let _ = port.write_data_terminal_ready(true);

    Ok(port)
}

#[cfg(feature = "serialport")]
impl FdcanusbDevice<Box<dyn serialport::SerialPort>> {
    /// Opens an fdcanusb device at the given serial port path.
    ///
    /// The path is a serial device such as `/dev/ttyACM0` or
    /// `/dev/serial/by-id/usb-mjbots_fdcanusb_...` on Linux,
    /// `/dev/cu.usbmodem...` on macOS, or `COM3` on Windows.
    ///
    /// The path may also refer to a moteus connected directly over a
    /// TTL UART (see `docs/integration/uart.md`).  Devices that cannot
    /// be conclusively identified as fdcanusbs by USB VID/PID are
    /// treated as UARTs: non-pipelined operation with per-frame
    /// retries and CRC-8 checksums.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::transport::fdcanusb::FdcanusbDevice;
    ///
    /// fn main() -> Result<(), moteus::Error> {
    ///     let fdcanusb = FdcanusbDevice::new("/dev/ttyACM0")?;
    ///     Ok(())
    /// }
    /// ```
    pub fn new(path: &str) -> Result<Self> {
        Self::open_with(path, &FdcanusbOptions::new())
    }

    /// Opens an fdcanusb device with explicit timeout and BRS settings.
    pub fn with_options(path: &str, timeout: Duration, disable_brs: bool) -> Result<Self> {
        Self::open_with(
            path,
            &FdcanusbOptions::new()
                .timeout(timeout)
                .disable_brs(disable_brs),
        )
    }

    /// Opens an fdcanusb or moteus UART device with full options.
    pub fn open_with(path: &str, options: &FdcanusbOptions) -> Result<Self> {
        let port = open_serial_port(path, options.baudrate)?;
        let mut device = Self::from_stream_with(port, options);

        // Auto-detect UART connections unless the caller decided.
        if options.uart_mode.is_none() && !crate::transport::discovery::is_fdcanusb_path(path) {
            device.uart_mode = true;
            device.checksum_active = true;
        }

        Ok(device)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::transaction::FrameFilter;
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    #[derive(Default)]
    struct MockSerialInner {
        /// Queued device responses, delivered one per host write.
        responses: VecDeque<Vec<u8>>,
        /// Number of initial host writes to drop without a response,
        /// simulating a lossy UART.
        drop_writes: usize,
        /// Data waiting to be read by the host.
        read_buffer: Vec<u8>,
        /// Everything the host wrote.
        written: Vec<u8>,
    }

    /// A scriptable mock serial port.  Clones share state so tests can
    /// inspect traffic after the device takes ownership of the stream.
    #[derive(Clone, Default)]
    struct MockSerial(Arc<Mutex<MockSerialInner>>);

    impl MockSerial {
        fn queue_response(&self, response: &[u8]) {
            self.0
                .lock()
                .unwrap()
                .responses
                .push_back(response.to_vec());
        }

        /// Makes data readable immediately, without waiting for a host
        /// write — simulating a late response arriving on its own.
        fn inject(&self, data: &[u8]) {
            self.0.lock().unwrap().read_buffer.extend_from_slice(data);
        }

        fn drop_writes(&self, count: usize) {
            self.0.lock().unwrap().drop_writes = count;
        }

        fn written(&self) -> Vec<u8> {
            self.0.lock().unwrap().written.clone()
        }

        fn command_count(&self) -> usize {
            let written = self.written();
            written
                .windows(b"can send".len())
                .filter(|w| w == b"can send")
                .count()
        }
    }

    impl std::io::Read for MockSerial {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let mut inner = self.0.lock().unwrap();
            if inner.read_buffer.is_empty() {
                return Err(std::io::Error::from(std::io::ErrorKind::TimedOut));
            }
            let n = buf.len().min(inner.read_buffer.len());
            buf[..n].copy_from_slice(&inner.read_buffer[..n]);
            inner.read_buffer.drain(..n);
            Ok(n)
        }
    }

    impl std::io::Write for MockSerial {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            let mut inner = self.0.lock().unwrap();
            inner.written.extend_from_slice(buf);
            if inner.drop_writes > 0 {
                inner.drop_writes -= 1;
            } else if let Some(response) = inner.responses.pop_front() {
                inner.read_buffer.extend_from_slice(&response);
            }
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
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
            .timeout(Duration::from_millis(10))
    }

    #[test]
    fn test_uart_checksum_send() {
        let mock = MockSerial::default();
        let mut device =
            FdcanusbDevice::from_stream_with(mock.clone(), &uart_options().checksum_enabled(true));

        mock.queue_response(b"OK *BD\n");
        device.write(&test_frame()).unwrap();

        // The sent command must end with a valid " *XX" checksum.
        let written = mock.written();
        let line = std::str::from_utf8(&written).unwrap().trim_end();
        let (content, had, valid) = codec::strip_checksum(line.as_bytes());
        assert!(had, "no checksum on: {}", line);
        assert!(valid);
        assert!(content.starts_with(b"can send 0105 30313233"));
    }

    #[test]
    fn test_uart_checksum_receive() {
        let mock = MockSerial::default();
        let mut device =
            FdcanusbDevice::from_stream_with(mock.clone(), &uart_options().checksum_enabled(true));

        let mut response = checksummed("OK");
        response.push_str(&checksummed("rcv 0105 2030"));
        mock.queue_response(response.as_bytes());

        let mut requests = [Request::new(test_frame())
            .with_filter(FrameFilter::Any)
            .with_expected_replies(1)];
        device.transaction(&mut requests).unwrap();

        let responses = requests[0].responses.take();
        assert_eq!(responses.len(), 1);
        assert_eq!(responses[0].payload(), &[0x20, 0x30]);
    }

    #[test]
    fn test_uart_invalid_checksum_discarded() {
        let mock = MockSerial::default();
        let mut device =
            FdcanusbDevice::from_stream_with(mock.clone(), &uart_options().checksum_enabled(true));

        // An invalid checksum line is skipped; the valid one that
        // follows is accepted.
        let mut response = String::from("OK *FF\n");
        response.push_str(&checksummed("OK"));
        mock.queue_response(response.as_bytes());

        device.write(&test_frame()).unwrap();
        assert_eq!(mock.command_count(), 1);
    }

    #[test]
    fn test_uart_missing_checksum_discarded() {
        let mock = MockSerial::default();
        let mut device = FdcanusbDevice::from_stream_with(
            mock.clone(),
            &uart_options().checksum_enabled(true).max_retries(0),
        );

        // In checksum mode, lines without checksums are discarded, so
        // a bare OK times out.
        mock.queue_response(b"OK\n");
        assert!(matches!(device.write(&test_frame()), Err(Error::Timeout)));
    }

    #[test]
    fn test_uart_err_checksum_enables_checksums() {
        let mock = MockSerial::default();
        let mut device = FdcanusbDevice::from_stream_with(mock.clone(), &uart_options());
        assert!(!device.checksum_active());

        // The device demands checksums; the retry must carry one.
        mock.queue_response(b"ERR missing checksum\n");
        mock.queue_response(b"OK *BD\n");

        device.write(&test_frame()).unwrap();
        assert!(device.checksum_active());

        let written = mock.written();
        let lines: Vec<&str> = std::str::from_utf8(&written)
            .unwrap()
            .trim_end()
            .split('\n')
            .collect();
        assert_eq!(lines.len(), 2);
        assert!(!lines[0].contains('*'), "first send unchecksummed");
        let (_, had, valid) = codec::strip_checksum(lines[1].as_bytes());
        assert!(had && valid, "retry must carry a checksum");
    }

    #[test]
    fn test_uart_retry_on_timeout() {
        let mock = MockSerial::default();
        let mut device = FdcanusbDevice::from_stream_with(mock.clone(), &uart_options());

        // First two commands are lost; the third is acknowledged.
        mock.drop_writes(2);
        mock.queue_response(b"OK\n");

        device.write(&test_frame()).unwrap();
        assert_eq!(mock.command_count(), 3);
    }

    #[test]
    fn test_uart_retries_exhausted() {
        let mock = MockSerial::default();
        let mut device =
            FdcanusbDevice::from_stream_with(mock.clone(), &uart_options().max_retries(2));

        // No response ever arrives.
        assert!(matches!(device.write(&test_frame()), Err(Error::Timeout)));
        assert_eq!(mock.command_count(), 3); // initial + 2 retries
    }

    #[test]
    fn test_fdcanusb_mode_does_not_retry() {
        let mock = MockSerial::default();
        let mut device = FdcanusbDevice::from_stream_with(
            mock.clone(),
            &FdcanusbOptions::new()
                .uart_mode(false)
                .timeout(Duration::from_millis(10)),
        );

        assert!(matches!(device.write(&test_frame()), Err(Error::Timeout)));
        assert_eq!(mock.command_count(), 1);
    }

    /// A filter matching frames from a specific source CAN ID.
    fn source_filter(source: u8) -> FrameFilter {
        FrameFilter::custom(move |f| ((f.arbitration_id >> 8) & 0x7F) as u8 == source)
    }

    #[test]
    fn test_uart_stale_reply_flushed_before_next_transaction() {
        let mock = MockSerial::default();
        let mut device = FdcanusbDevice::from_stream_with(mock.clone(), &uart_options());

        // Transaction 1: acknowledged, but its reply never arrives in
        // time.
        mock.queue_response(b"OK\n");
        let mut requests = [Request::new(test_frame())
            .with_filter(FrameFilter::Any)
            .with_expected_replies(1)];
        device.transaction(&mut requests).unwrap();
        assert!(requests[0].responses.is_empty());

        // The reply arrives late, after the transaction gave up...
        mock.inject(b"rcv 0105 AAAA\n");

        // ...and must NOT be attributed to the next transaction.
        mock.queue_response(b"OK\nrcv 0105 BBBB\n");
        let mut requests = [Request::new(test_frame())
            .with_filter(FrameFilter::Any)
            .with_expected_replies(1)];
        device.transaction(&mut requests).unwrap();

        let responses = requests[0].responses.take();
        assert_eq!(responses.len(), 1);
        assert_eq!(responses[0].payload(), &[0xBB, 0xBB], "stale reply leaked");
    }

    #[test]
    fn test_uart_reply_counted_per_request_filter() {
        let mock = MockSerial::default();
        let mut device = FdcanusbDevice::from_stream_with(mock.clone(), &uart_options());

        // Request 0 (servo 1) is acknowledged but its reply is
        // delayed; it arrives during request 1's (servo 2) window,
        // together with request 1's own reply.
        mock.queue_response(b"OK\n");
        mock.queue_response(b"OK\nrcv 0100 11\nrcv 0200 22\n");

        let mut requests = [
            Request::new(test_frame())
                .with_filter(source_filter(1))
                .with_expected_replies(1),
            Request::new(test_frame())
                .with_filter(source_filter(2))
                .with_expected_replies(1),
        ];
        device.transaction(&mut requests).unwrap();

        // The late servo-1 frame is routed to request 0 and must not
        // satisfy request 1's expected count: request 1 keeps waiting
        // and receives its own reply.
        let r0 = requests[0].responses.take();
        let r1 = requests[1].responses.take();
        assert_eq!(r0.len(), 1);
        assert_eq!(r0[0].payload(), &[0x11]);
        assert_eq!(r1.len(), 1);
        assert_eq!(r1[0].payload(), &[0x22]);
    }

    #[test]
    fn test_eof_reported_as_error() {
        /// A stream that is immediately at EOF.
        struct EofStream;
        impl std::io::Read for EofStream {
            fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
                Ok(0)
            }
        }
        impl std::io::Write for EofStream {
            fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
                Ok(buf.len())
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }

        let mut device = FdcanusbDevice::from_stream_with(
            EofStream,
            &FdcanusbOptions::new().timeout(Duration::from_millis(10)),
        );

        // A closed stream reports EOF rather than spinning until the
        // timeout and reporting Timeout.
        assert!(matches!(
            device.write(&test_frame()),
            Err(Error::Io(e)) if e.kind() == std::io::ErrorKind::UnexpectedEof
        ));
    }

    #[test]
    fn test_fdcanusb_mode_accepts_checksummed_responses() {
        // Even outside checksum mode, valid checksums are stripped and
        // invalid ones discarded.
        let mock = MockSerial::default();
        let mut device = FdcanusbDevice::from_stream_with(
            mock.clone(),
            &FdcanusbOptions::new().timeout(Duration::from_millis(10)),
        );

        mock.queue_response(checksummed("OK").as_bytes());
        device.write(&test_frame()).unwrap();
    }

    #[test]
    fn test_encode_frame() {
        let mut frame = CanFdFrame::new();
        frame.arbitration_id = 0x8001;
        frame.data[0..3].copy_from_slice(&[0x01, 0x00, 0x0A]);
        frame.size = 3;
        frame.set_brs(true);
        frame.set_fdcan(true);

        let encoded = FdcanusbProtocol::encode_frame(&frame, false);
        assert!(encoded.starts_with("can send 8001 01000A"));
        assert!(encoded.contains("BF"));
    }

    #[test]
    fn test_parse_frame() {
        let line = "rcv 8001 01000A0000000000 B F";
        let frame = FdcanusbProtocol::parse_frame(line).unwrap();

        assert_eq!(frame.arbitration_id, 0x8001);
        assert_eq!(frame.data[0], 0x01);
        assert_eq!(frame.data[1], 0x00);
        assert_eq!(frame.data[2], 0x0A);
        assert!(frame.brs_enabled());
        assert!(frame.fdcan_enabled());
    }

    #[test]
    fn test_round_up_dlc() {
        assert_eq!(CanFdFrame::round_up_dlc(0), 0);
        assert_eq!(CanFdFrame::round_up_dlc(8), 8);
        assert_eq!(CanFdFrame::round_up_dlc(9), 12);
        assert_eq!(CanFdFrame::round_up_dlc(12), 12);
        assert_eq!(CanFdFrame::round_up_dlc(13), 16);
        assert_eq!(CanFdFrame::round_up_dlc(33), 48);
        assert_eq!(CanFdFrame::round_up_dlc(49), 64);
    }

    #[test]
    fn test_is_ok_response() {
        assert!(FdcanusbProtocol::is_ok_response("OK\n"));
        assert!(FdcanusbProtocol::is_ok_response("OK"));
        assert!(FdcanusbProtocol::is_ok_response("  OK  "));
        assert!(!FdcanusbProtocol::is_ok_response("rcv 8001 00"));
    }
}
