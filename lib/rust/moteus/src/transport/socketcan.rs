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

//! SocketCAN transport for Linux.
//!
//! This module provides a transport implementation using the Linux kernel's
//! SocketCAN interface for CAN-FD communication.
//!
//! # Example
//!
//! ```ignore
//! use moteus::transport::socketcan::SocketCan;
//!
//! let mut transport = SocketCan::new("can0")?;
//! let responses = transport.cycle(&frames)?;
//! ```

#[cfg(target_os = "linux")]
mod linux {
    use crate::error::{Error, Result};
    use crate::transport::device::{TransportDevice, TransportDeviceInfo};
    use crate::transport::socketcan_common::linux::*;
    use crate::transport::transaction::{dispatch_frame, Request};
    use moteus_protocol::CanFdFrame;
    use std::io;
    use std::os::unix::io::{AsRawFd, RawFd};
    use std::time::{Duration, Instant};

    // Sync-only FFI: select and supporting types
    mod select_ffi {
        use std::os::raw::c_int;

        #[repr(C)]
        pub struct timeval {
            pub tv_sec: i64,
            pub tv_usec: i64,
        }

        #[repr(C)]
        pub struct fd_set {
            pub fds_bits: [u64; 16], // 1024 bits
        }

        impl fd_set {
            pub fn zero(&mut self) {
                for bit in &mut self.fds_bits {
                    *bit = 0;
                }
            }

            pub fn set(&mut self, fd: c_int) {
                let idx = fd as usize / 64;
                let bit = fd as usize % 64;
                if idx < 16 {
                    self.fds_bits[idx] |= 1 << bit;
                }
            }
        }

        extern "C" {
            pub fn select(
                nfds: c_int,
                readfds: *mut fd_set,
                writefds: *mut fd_set,
                exceptfds: *mut fd_set,
                timeout: *mut timeval,
            ) -> c_int;
        }
    }

    use select_ffi::*;

    /// SocketCAN transport for Linux.
    pub struct SocketCan {
        fd: RawFd,
        timeout_ms: u32,
        disable_brs: bool,
        /// Device info for TransportDevice trait.
        pub(crate) info: TransportDeviceInfo,
    }

    impl SocketCan {
        /// Creates a new SocketCAN transport.
        ///
        /// # Arguments
        /// * `interface` - CAN interface name (e.g., "can0", "vcan0")
        pub fn new(interface: &str) -> Result<Self> {
            Self::with_options(interface, 100, false)
        }

        /// Creates a new SocketCAN transport with custom timeout.
        pub fn with_timeout(interface: &str, timeout_ms: u32) -> Result<Self> {
            Self::with_options(interface, timeout_ms, false)
        }

        /// Creates a new SocketCAN transport with custom timeout and BRS control.
        pub fn with_options(interface: &str, timeout_ms: u32, disable_brs: bool) -> Result<Self> {
            // Create socket
            let fd = unsafe { socket(PF_CAN, SOCK_RAW, CAN_RAW) };
            if fd < 0 {
                return Err(Error::Io(io::Error::last_os_error()));
            }

            // Get interface index
            let ifindex = match get_ifindex(interface) {
                Ok(idx) => idx,
                Err(e) => {
                    unsafe { close(fd) };
                    return Err(e);
                }
            };

            // Bind to the interface
            let addr = SockAddrCan {
                can_family: AF_CAN as u16,
                can_ifindex: ifindex,
                rx_id: 0,
                tx_id: 0,
            };

            let ret = unsafe {
                bind(
                    fd,
                    &addr as *const SockAddrCan as *const std::ffi::c_void,
                    std::mem::size_of::<SockAddrCan>() as u32,
                )
            };
            if ret < 0 {
                unsafe { close(fd) };
                return Err(Error::Io(io::Error::last_os_error()));
            }

            // Enable CAN FD frames
            let enable: i32 = 1;
            let ret = unsafe {
                setsockopt(
                    fd,
                    SOL_CAN_RAW,
                    CAN_RAW_FD_FRAMES,
                    &enable as *const i32 as *const std::ffi::c_void,
                    std::mem::size_of::<i32>() as u32,
                )
            };
            if ret < 0 {
                unsafe { close(fd) };
                return Err(Error::Io(io::Error::last_os_error()));
            }

            // Set non-blocking mode for timeout handling
            let flags = unsafe { fcntl(fd, F_GETFL) };
            unsafe { fcntl(fd, F_SETFL, flags | O_NONBLOCK) };

            Ok(SocketCan {
                fd,
                timeout_ms,
                disable_brs,
                info: TransportDeviceInfo::new(0, "SocketCan")
                    .with_serial(interface.to_string())
                    .with_detail(format!("'{}'", interface)),
            })
        }

        /// Sends a single frame.
        fn send_frame(&self, frame: &CanFdFrame) -> Result<()> {
            let raw = frame_to_raw(frame, self.disable_brs);

            let ret = unsafe {
                write(
                    self.fd,
                    &raw as *const CanFdFrameRaw as *const std::ffi::c_void,
                    CANFD_MTU,
                )
            };

            if ret < 0 {
                return Err(Error::Io(io::Error::last_os_error()));
            }

            Ok(())
        }

        /// Receives frames until timeout.
        fn receive_frames(&self, expected_count: usize) -> Result<Vec<CanFdFrame>> {
            let mut frames = Vec::new();
            let deadline = Instant::now() + Duration::from_millis(self.timeout_ms as u64);

            while frames.len() < expected_count {
                if Instant::now() > deadline {
                    break;
                }

                // Use select for timeout
                let remaining = deadline.saturating_duration_since(Instant::now());
                let mut tv = timeval {
                    tv_sec: remaining.as_secs() as i64,
                    tv_usec: remaining.subsec_micros() as i64,
                };

                let mut readfds = fd_set { fds_bits: [0; 16] };
                readfds.zero();
                readfds.set(self.fd);

                let ret = unsafe {
                    select(
                        self.fd + 1,
                        &mut readfds,
                        std::ptr::null_mut(),
                        std::ptr::null_mut(),
                        &mut tv,
                    )
                };

                if ret <= 0 {
                    break; // Timeout or error
                }

                // Read frame
                let mut raw = CanFdFrameRaw::default();
                let ret = unsafe {
                    read(
                        self.fd,
                        &mut raw as *mut CanFdFrameRaw as *mut std::ffi::c_void,
                        CANFD_MTU,
                    )
                };

                if ret > 0 {
                    frames.push(frame_from_raw(&raw));
                }
            }

            Ok(frames)
        }
    }

    impl Drop for SocketCan {
        fn drop(&mut self) {
            unsafe { close(self.fd) };
        }
    }

    impl AsRawFd for SocketCan {
        fn as_raw_fd(&self) -> RawFd {
            self.fd
        }
    }

    impl TransportDevice for SocketCan {
        fn transaction(&mut self, requests: &mut [Request]) -> Result<()> {
            debug_assert!(
                requests.iter().all(|r| r.child_device.is_none()),
                "SocketCan does not support child devices"
            );
            // Send all frames
            for req in requests.iter() {
                if let Some(frame) = &req.frame {
                    self.send_frame(frame)?;
                }
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
            // Try to read one frame with timeout
            let deadline = Instant::now() + Duration::from_millis(self.timeout_ms as u64);

            let remaining = deadline.saturating_duration_since(Instant::now());
            let mut tv = timeval {
                tv_sec: remaining.as_secs() as i64,
                tv_usec: remaining.subsec_micros() as i64,
            };

            let mut readfds = fd_set { fds_bits: [0; 16] };
            readfds.zero();
            readfds.set(self.fd);

            let ret = unsafe {
                select(
                    self.fd + 1,
                    &mut readfds,
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    &mut tv,
                )
            };

            if ret <= 0 {
                return Ok(None); // Timeout or error
            }

            // Read frame
            let mut raw = CanFdFrameRaw::default();
            let ret = unsafe {
                read(
                    self.fd,
                    &mut raw as *mut CanFdFrameRaw as *mut std::ffi::c_void,
                    CANFD_MTU,
                )
            };

            if ret > 0 {
                return Ok(Some(frame_from_raw(&raw)));
            }

            Ok(None)
        }

        fn flush(&mut self) -> Result<()> {
            // Drain any incoming data with a short timeout
            let deadline = Instant::now() + Duration::from_millis(50);

            while Instant::now() < deadline {
                let mut tv = timeval {
                    tv_sec: 0,
                    tv_usec: 1000, // 1ms
                };

                let mut readfds = fd_set { fds_bits: [0; 16] };
                readfds.zero();
                readfds.set(self.fd);

                let ret = unsafe {
                    select(
                        self.fd + 1,
                        &mut readfds,
                        std::ptr::null_mut(),
                        std::ptr::null_mut(),
                        &mut tv,
                    )
                };

                if ret <= 0 {
                    break; // No more data
                }

                // Read and discard
                let mut raw = CanFdFrameRaw::default();
                unsafe {
                    read(
                        self.fd,
                        &mut raw as *mut CanFdFrameRaw as *mut std::ffi::c_void,
                        CANFD_MTU,
                    )
                };
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
}

#[cfg(target_os = "linux")]
pub use linux::SocketCan;

/// Stub for non-Linux platforms.
#[cfg(not(target_os = "linux"))]
pub struct SocketCan {
    _private: (),
}

#[cfg(not(target_os = "linux"))]
impl SocketCan {
    /// SocketCAN is only available on Linux.
    pub fn new(_interface: &str) -> crate::error::Result<Self> {
        Err(crate::error::Error::Protocol(
            "SocketCAN is only available on Linux".to_string(),
        ))
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_socketcan_unavailable_on_non_linux() {
        // This test just verifies the module compiles
        #[cfg(not(target_os = "linux"))]
        {
            let result = super::SocketCan::new("can0");
            assert!(result.is_err());
        }
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn test_socketcan_interface_not_found() {
        // This should fail because the interface doesn't exist
        let result = super::SocketCan::new("nonexistent_can_interface_12345");
        assert!(result.is_err());
    }
}
