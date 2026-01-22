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
        !trimmed.is_empty()
            && !trimmed.starts_with("OK")
            && !trimmed.starts_with("rcv")
    }

    /// Converts bytes to hex string.
    fn hexify(data: &[u8]) -> String {
        data.iter().map(|b| format!("{:02X}", b)).collect()
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
pub struct Fdcanusb<S: std::io::Read + std::io::Write> {
    reader: BufReader<S>,
    writer: S,
    timeout_ms: u32,
    disable_brs: bool,
    line_buffer: String,
    /// Buffer for frames received during wait_for_ok.
    pending_frames: Vec<CanFdFrame>,
    /// Device info for TransportDevice trait.
    pub(crate) info: TransportDeviceInfo,
}

impl<S: std::io::Read + std::io::Write + Clone> Fdcanusb<S> {
    /// Creates an FdCanUSB transport from a pre-opened stream.
    ///
    /// This is primarily useful for testing with mock streams.
    /// For normal use, prefer [`Fdcanusb::new`] which opens the serial port
    /// directly from a device path.
    pub fn from_stream(stream: S) -> Self {
        let reader = BufReader::new(stream.clone());
        Fdcanusb {
            reader,
            writer: stream,
            timeout_ms: 100,
            disable_brs: false,
            line_buffer: String::new(),
            pending_frames: Vec::new(),
            info: TransportDeviceInfo::new(0, "Fdcanusb"),
        }
    }

    /// Creates an FdCanUSB transport from a pre-opened stream with options.
    ///
    /// This is primarily useful for testing with mock streams.
    /// For normal use, prefer [`Fdcanusb::new`] which opens the serial port
    /// directly from a device path.
    pub fn from_stream_with_options(stream: S, timeout_ms: u32, disable_brs: bool) -> Self {
        let reader = BufReader::new(stream.clone());
        Fdcanusb {
            reader,
            writer: stream,
            timeout_ms,
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
        self.writer.write_all(cmd.as_bytes())?;
        Ok(())
    }

    /// Sends a single frame and waits for OK response.
    fn send_frame(&mut self, frame: &CanFdFrame) -> Result<()> {
        self.write_frame(frame)?;
        self.writer.flush()?;
        self.wait_for_ok()?;
        Ok(())
    }

    /// Waits for an OK response.
    /// Any received frames (rcv lines) are buffered for later retrieval.
    fn wait_for_ok(&mut self) -> Result<()> {
        let deadline = Instant::now() + Duration::from_millis(self.timeout_ms as u64);

        loop {
            if Instant::now() > deadline {
                return Err(Error::Timeout);
            }

            self.line_buffer.clear();
            match self.reader.read_line(&mut self.line_buffer) {
                Ok(0) => continue, // No data yet
                Ok(_) => {
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
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(e) => return Err(Error::Io(e)),
            }
        }
    }

    /// Receives frames with timeout.
    fn receive_frames(&mut self, expected_count: usize) -> Result<Vec<CanFdFrame>> {
        // First, drain any frames that were buffered during wait_for_ok
        let mut frames: Vec<CanFdFrame> = self.pending_frames.drain(..).collect();

        if frames.len() >= expected_count {
            return Ok(frames);
        }

        let deadline = Instant::now() + Duration::from_millis(self.timeout_ms as u64);

        while frames.len() < expected_count {
            if Instant::now() > deadline {
                break; // Timeout - return what we have
            }

            self.line_buffer.clear();
            match self.reader.read_line(&mut self.line_buffer) {
                Ok(0) => continue,
                Ok(_) => {
                    if let Some(frame) = FdcanusbProtocol::parse_frame(&self.line_buffer) {
                        frames.push(frame);
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(e) => return Err(Error::Io(e)),
            }
        }

        Ok(frames)
    }
}

impl<S: std::io::Read + std::io::Write + Clone + Send> TransportDevice for Fdcanusb<S> {
    fn transaction(&mut self, requests: &mut [Request]) -> Result<()> {
        debug_assert!(
            requests.iter().all(|r| r.child_device.is_none()),
            "Fdcanusb does not support child devices"
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
            self.writer.flush()?;
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
        let deadline = Instant::now() + Duration::from_millis(self.timeout_ms as u64);

        while Instant::now() < deadline {
            self.line_buffer.clear();
            match self.reader.read_line(&mut self.line_buffer) {
                Ok(0) => continue,
                Ok(_) => {
                    if let Some(frame) = FdcanusbProtocol::parse_frame(&self.line_buffer) {
                        return Ok(Some(frame));
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(e) => return Err(Error::Io(e)),
            }
        }

        Ok(None)
    }

    fn flush(&mut self) -> Result<()> {
        // Clear pending frames
        self.pending_frames.clear();

        // Drain any incoming data for a short period
        let deadline = Instant::now() + Duration::from_millis(50);

        while Instant::now() < deadline {
            self.line_buffer.clear();
            match self.reader.read_line(&mut self.line_buffer) {
                Ok(0) => break, // No more data
                Ok(_) => continue, // Discard
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }

        Ok(())
    }

    fn info(&self) -> &TransportDeviceInfo {
        &self.info
    }

    fn set_timeout(&mut self, timeout_ms: u32) {
        self.timeout_ms = timeout_ms;
    }

    fn timeout(&self) -> u32 {
        self.timeout_ms
    }
}

// =============================================================================
// Linux serial port
// =============================================================================

/// Linux serial port implementation for fdcanusb.
#[cfg(target_os = "linux")]
mod linux_serial {
    use std::fs::OpenOptions;
    use std::io::{Read, Write, Result};
    use std::os::unix::io::{AsRawFd, RawFd};
    use std::os::raw::{c_int, c_void};

    // termios constants
    const NCCS: usize = 32;
    const VMIN: usize = 6;
    const VTIME: usize = 5;
    const TCSANOW: c_int = 0;
    const TCIOFLUSH: c_int = 2;

    // c_iflag bits
    const IGNBRK: u32 = 0o000001;
    const BRKINT: u32 = 0o000002;
    const PARMRK: u32 = 0o000010;
    const ISTRIP: u32 = 0o000040;
    const INLCR: u32 = 0o000100;
    const IGNCR: u32 = 0o000200;
    const ICRNL: u32 = 0o000400;
    const IXON: u32 = 0o002000;

    // c_oflag bits
    const OPOST: u32 = 0o000001;

    // c_lflag bits
    const ECHO: u32 = 0o000010;
    const ECHONL: u32 = 0o000100;
    const ICANON: u32 = 0o000002;
    const ISIG: u32 = 0o000001;
    const IEXTEN: u32 = 0o100000;

    // c_cflag bits
    const CSIZE: u32 = 0o000060;
    const PARENB: u32 = 0o000400;
    const CS8: u32 = 0o000060;

    #[repr(C)]
    struct Termios {
        c_iflag: u32,
        c_oflag: u32,
        c_cflag: u32,
        c_lflag: u32,
        c_line: u8,
        c_cc: [u8; NCCS],
        c_ispeed: u32,
        c_ospeed: u32,
    }

    extern "C" {
        fn tcgetattr(fd: c_int, termios: *mut Termios) -> c_int;
        fn tcsetattr(fd: c_int, optional_actions: c_int, termios: *const Termios) -> c_int;
        fn tcflush(fd: c_int, queue_selector: c_int) -> c_int;
        fn dup(fd: c_int) -> c_int;
    }

    /// A simple serial port wrapper for Linux.
    pub struct LinuxSerialPort {
        fd: RawFd,
    }

    impl LinuxSerialPort {
        /// Opens a serial port with appropriate settings for fdcanusb.
        pub fn open(path: &str) -> Result<Self> {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .open(path)?;

            let fd = file.as_raw_fd();

            // Configure the serial port using termios
            unsafe {
                let mut termios: Termios = std::mem::zeroed();
                if tcgetattr(fd, &mut termios) != 0 {
                    return Err(std::io::Error::last_os_error());
                }

                // Raw mode - disable canonical processing, echo, signals
                termios.c_iflag &= !(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
                termios.c_oflag &= !OPOST;
                termios.c_lflag &= !(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
                termios.c_cflag &= !(CSIZE | PARENB);
                termios.c_cflag |= CS8;

                // Non-blocking reads: VMIN=0 means return immediately
                // VTIME=1 means 0.1s inter-character timeout
                termios.c_cc[VMIN] = 0;
                termios.c_cc[VTIME] = 1;

                if tcsetattr(fd, TCSANOW, &termios) != 0 {
                    return Err(std::io::Error::last_os_error());
                }

                // Flush any pending data
                tcflush(fd, TCIOFLUSH);
            }

            // Transfer ownership of fd - don't let File close it
            std::mem::forget(file);

            Ok(LinuxSerialPort { fd })
        }
    }

    impl Clone for LinuxSerialPort {
        fn clone(&self) -> Self {
            let new_fd = unsafe { dup(self.fd) };
            LinuxSerialPort { fd: new_fd }
        }
    }

    impl Drop for LinuxSerialPort {
        fn drop(&mut self) {
            unsafe {
                extern "C" {
                    fn close(fd: c_int) -> c_int;
                }
                close(self.fd);
            }
        }
    }

    impl Read for LinuxSerialPort {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
            let n = unsafe {
                extern "C" {
                    fn read(fd: c_int, buf: *mut c_void, count: usize) -> isize;
                }
                read(self.fd, buf.as_mut_ptr() as *mut c_void, buf.len())
            };
            if n < 0 {
                Err(std::io::Error::last_os_error())
            } else {
                Ok(n as usize)
            }
        }
    }

    impl Write for LinuxSerialPort {
        fn write(&mut self, buf: &[u8]) -> Result<usize> {
            let n = unsafe {
                extern "C" {
                    fn write(fd: c_int, buf: *const c_void, count: usize) -> isize;
                }
                write(self.fd, buf.as_ptr() as *const c_void, buf.len())
            };
            if n < 0 {
                Err(std::io::Error::last_os_error())
            } else {
                Ok(n as usize)
            }
        }

        fn flush(&mut self) -> Result<()> {
            // For serial ports, write is typically unbuffered
            Ok(())
        }
    }
}

#[cfg(target_os = "linux")]
use linux_serial::LinuxSerialPort;

#[cfg(target_os = "linux")]
impl Fdcanusb<LinuxSerialPort> {
    /// Opens an fdcanusb device at the given serial port path.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let fdcanusb = Fdcanusb::new("/dev/ttyACM0")?;
    /// ```
    pub fn new(path: &str) -> Result<Self> {
        let port = LinuxSerialPort::open(path)?;
        Ok(Self::from_stream(port))
    }

    /// Opens an fdcanusb device with explicit timeout and BRS settings.
    pub fn with_options(path: &str, timeout_ms: u32, disable_brs: bool) -> Result<Self> {
        let port = LinuxSerialPort::open(path)?;
        Ok(Self::from_stream_with_options(port, timeout_ms, disable_brs))
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
