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

//! FdCanUSB transport for communicating with moteus controllers.
//!
//! The fdcanusb is a USB-to-CAN-FD adapter that uses a simple text protocol:
//! - Send: `can send AAAA HEXDATA FLAGS\n`
//! - Receive: `rcv AAAA HEXDATA FLAGS`
//! - OK response after each send

use crate::error::{Error, Result};
use crate::transport::device::{TransportDevice, TransportDeviceInfo};
use crate::transport::transaction::{dispatch_frame, Request};
use moteus_protocol::CanFdFrame;
use std::io::{BufRead, BufReader};
use std::time::{Duration, Instant};

/// FdCanUSB text protocol encoder/decoder.
pub struct FdcanusbProtocol;

impl FdcanusbProtocol {
    /// Encodes a CAN-FD frame into the fdcanusb text protocol format.
    ///
    /// Returns a string like: `can send 8001 0123456789ABCDEF bF\n`
    pub fn encode_frame(frame: &CanFdFrame, disable_brs: bool) -> String {
        let hex_data = Self::hexify(&frame.data[..frame.size as usize]);

        // Pad to valid CAN-FD DLC size
        let on_wire_size = CanFdFrame::round_up_dlc(frame.size as usize);
        let padding = "50".repeat(on_wire_size - frame.size as usize);

        // Build flags
        let mut flags = String::new();
        if frame.brs_enabled() && !disable_brs {
            flags.push('B');
        } else {
            flags.push('b');
        }
        if frame.fdcan_enabled() {
            flags.push('F');
        }

        format!(
            "can send {:04x} {}{} {}\n",
            frame.arbitration_id, hex_data, padding, flags
        )
    }

    /// Parses a received frame line from fdcanusb.
    ///
    /// Input format: `rcv AAAA HEXDATA [E] [B] [F]`
    ///
    /// When the servo responds with no data, the line has an empty data
    /// field: `rcv 0100  e B F`.  Split on single space (matching
    /// Python's `str.split(' ')`) so the empty field is preserved.
    pub fn parse_frame(line: &str) -> Option<CanFdFrame> {
        let parts: Vec<&str> = line.trim().split(' ').collect();
        if parts.len() < 3 || parts[0] != "rcv" {
            return None;
        }

        let arbitration_id = u32::from_str_radix(parts[1], 16).ok()?;
        let data = Self::dehexify(parts[2])?;

        let mut frame = CanFdFrame::new();
        frame.arbitration_id = arbitration_id;
        frame.size = data.len() as u8;
        frame.data[..data.len()].copy_from_slice(&data);

        // Parse flags
        for flag in parts.iter().skip(3) {
            match *flag {
                "E" => frame.arbitration_id |= 0x80000000, // Extended ID marker
                "B" => frame.set_brs(true),
                "F" => frame.set_fdcan(true),
                _ => {}
            }
        }

        Some(frame)
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

    /// Converts bytes to hex string.
    fn hexify(data: &[u8]) -> String {
        use std::fmt::Write;
        let mut s = String::with_capacity(data.len() * 2);
        for b in data {
            write!(s, "{:02X}", b).unwrap();
        }
        s
    }

    /// Converts hex string to bytes.
    fn dehexify(hex: &str) -> Option<Vec<u8>> {
        let hex = hex.trim();
        if hex.len() % 2 != 0 {
            return None;
        }

        (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).ok())
            .collect()
    }
}

/// FdCanUSB transport implementation.
///
/// This provides a synchronous interface for communicating with moteus
/// controllers through an fdcanusb device.
pub struct FdcanusbDevice<S: std::io::Read + std::io::Write> {
    reader: BufReader<S>,
    timeout: Duration,
    disable_brs: bool,
    line_buffer: String,
    /// Buffer for frames received during wait_for_ok.
    pending_frames: Vec<CanFdFrame>,
    /// Device info for TransportDevice trait.
    pub(crate) info: TransportDeviceInfo,
}

impl<S: std::io::Read + std::io::Write> std::fmt::Debug for FdcanusbDevice<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FdcanusbDevice")
            .field("info", &self.info)
            .field("timeout", &self.timeout)
            .field("disable_brs", &self.disable_brs)
            .field("pending_frames", &self.pending_frames.len())
            .finish()
    }
}

impl<S: std::io::Read + std::io::Write> FdcanusbDevice<S> {
    /// Creates an FdCanUSB transport from a pre-opened stream.
    ///
    /// This is primarily useful for testing with mock streams.
    /// For normal use, prefer [`FdcanusbDevice::new`] which opens the serial port
    /// directly from a device path.
    pub fn from_stream(stream: S) -> Self {
        Self::from_stream_with_options(stream, crate::transport::factory::DEFAULT_TIMEOUT, false)
    }

    /// Creates an FdCanUSB transport from a pre-opened stream with options.
    ///
    /// This is primarily useful for testing with mock streams.
    /// For normal use, prefer [`FdcanusbDevice::new`] which opens the serial port
    /// directly from a device path.
    pub fn from_stream_with_options(stream: S, timeout: Duration, disable_brs: bool) -> Self {
        FdcanusbDevice {
            reader: BufReader::new(stream),
            timeout,
            disable_brs,
            line_buffer: String::new(),
            pending_frames: Vec::new(),
            info: TransportDeviceInfo::new(0, "Fdcanusb"),
        }
    }

    /// Enables or disables bit rate switching.
    pub fn set_disable_brs(&mut self, disable: bool) {
        self.disable_brs = disable;
    }

    /// Writes a frame without waiting for OK response.
    fn write_frame(&mut self, frame: &CanFdFrame) -> Result<()> {
        let cmd = FdcanusbProtocol::encode_frame(frame, self.disable_brs);
        self.reader.get_mut().write_all(cmd.as_bytes())?;
        Ok(())
    }

    /// Sends a single frame and waits for OK response.
    fn send_frame(&mut self, frame: &CanFdFrame) -> Result<()> {
        self.write_frame(frame)?;
        self.reader.get_mut().flush()?;
        self.wait_for_ok()?;
        Ok(())
    }

    /// Reads one complete line (terminated by '\n') into `line_buffer`,
    /// waiting until `deadline`.
    ///
    /// Returns `Ok(true)` when a complete line is available and `Ok(false)`
    /// when the deadline expires first.  Partial data received before the
    /// deadline is retained in `line_buffer` so it is not lost across
    /// timeout-bounded reads; the caller must clear the buffer after
    /// consuming a complete line.
    fn read_line_deadline(&mut self, deadline: Instant) -> Result<bool> {
        loop {
            if Instant::now() > deadline {
                return Ok(false);
            }

            match self.reader.read_line(&mut self.line_buffer) {
                Ok(0) => continue, // No data yet
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

    /// Waits for an OK response.
    /// Any received frames (rcv lines) are buffered for later retrieval.
    fn wait_for_ok(&mut self) -> Result<()> {
        let deadline = Instant::now() + self.timeout;

        loop {
            if !self.read_line_deadline(deadline)? {
                return Err(Error::Timeout);
            }

            let line = self.line_buffer.trim();
            if FdcanusbProtocol::is_ok_response(line) {
                self.line_buffer.clear();
                return Ok(());
            }
            if FdcanusbProtocol::is_error_response(line) {
                let message = format!("fdcanusb error: {}", line);
                self.line_buffer.clear();
                return Err(Error::Protocol(message));
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
                break; // Timeout - return what we have
            }

            if let Some(frame) = FdcanusbProtocol::parse_frame(&self.line_buffer) {
                frames.push(frame);
            }
            self.line_buffer.clear();
        }

        Ok(frames)
    }
}

impl<S: std::io::Read + std::io::Write + Send> TransportDevice for FdcanusbDevice<S> {
    fn transaction(&mut self, requests: &mut [Request]) -> Result<()> {
        debug_assert!(
            requests.iter().all(|r| r.child_device.is_none()),
            "FdcanusbDevice does not support child devices"
        );

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

    fn write(&mut self, frame: &CanFdFrame) -> Result<()> {
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

            let frame = FdcanusbProtocol::parse_frame(&self.line_buffer);
            self.line_buffer.clear();
            if frame.is_some() {
                return Ok(frame);
            }
        }
    }

    fn flush(&mut self) -> Result<()> {
        // Clear pending frames
        self.pending_frames.clear();

        // Drain any incoming data for a short period, swallowing errors
        let deadline = Instant::now() + Duration::from_millis(50);

        while let Ok(true) = self.read_line_deadline(deadline) {
            self.line_buffer.clear();
        }

        // Discard any partial line left by the deadline expiring
        self.line_buffer.clear();

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
/// The fdcanusb is a USB CDC ACM device, so the baud rate is arbitrary.
#[cfg(feature = "serialport")]
fn open_serial_port(path: &str) -> Result<Box<dyn serialport::SerialPort>> {
    let mut port = serialport::new(path, 115_200)
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
        Self::with_options(path, crate::transport::factory::DEFAULT_TIMEOUT, false)
    }

    /// Opens an fdcanusb device with explicit timeout and BRS settings.
    pub fn with_options(path: &str, timeout: Duration, disable_brs: bool) -> Result<Self> {
        let port = open_serial_port(path)?;
        Ok(Self::from_stream_with_options(port, timeout, disable_brs))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hexify() {
        assert_eq!(FdcanusbProtocol::hexify(&[0x01, 0x23, 0xAB]), "0123AB");
        assert_eq!(FdcanusbProtocol::hexify(&[]), "");
    }

    #[test]
    fn test_dehexify() {
        assert_eq!(
            FdcanusbProtocol::dehexify("0123AB"),
            Some(vec![0x01, 0x23, 0xAB])
        );
        assert_eq!(FdcanusbProtocol::dehexify(""), Some(vec![]));
        assert_eq!(FdcanusbProtocol::dehexify("0"), None); // Odd length
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
