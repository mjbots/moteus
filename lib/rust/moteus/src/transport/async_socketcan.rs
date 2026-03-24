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

//! Async SocketCAN transport using tokio (Linux only).
//!
//! This module provides an async implementation of the SocketCAN transport
//! using tokio's `AsyncFd` wrapper for non-blocking I/O on the raw CAN socket.
//!
//! # Example
//!
//! ```ignore
//! use moteus::transport::async_socketcan::AsyncSocketCan;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), moteus::Error> {
//!     let mut transport = AsyncSocketCan::new("can0").await?;
//!     let responses = transport.transaction(&frames).await?;
//!     Ok(())
//! }
//! ```

#[cfg(target_os = "linux")]
mod linux {
    use crate::error::{Error, Result};
    use crate::transport::async_transport::BoxFuture;
    use crate::transport::device::{AsyncTransportDevice, TransportDeviceInfo};
    use crate::transport::socketcan_common::linux::*;
    use crate::transport::transaction::{dispatch_frame, Request};
    use moteus_protocol::CanFdFrame;

    use std::io;
    use std::os::unix::io::{AsRawFd, RawFd};
    use tokio::io::unix::AsyncFd;
    use tokio::io::Interest;

    /// Raw socket wrapper that can be used with AsyncFd.
    pub(crate) struct SocketCanRaw {
        fd: RawFd,
        disable_brs: bool,
    }

    impl SocketCanRaw {
        fn new(interface: &str, disable_brs: bool) -> Result<Self> {
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

            // Set non-blocking mode (required for AsyncFd)
            let flags = unsafe { fcntl(fd, F_GETFL) };
            unsafe { fcntl(fd, F_SETFL, flags | O_NONBLOCK) };

            Ok(SocketCanRaw { fd, disable_brs })
        }

        fn try_send(&self, frame: &CanFdFrame) -> io::Result<()> {
            let raw = frame_to_raw(frame, self.disable_brs);

            let ret = unsafe {
                write(
                    self.fd,
                    &raw as *const CanFdFrameRaw as *const std::ffi::c_void,
                    CANFD_MTU,
                )
            };

            if ret < 0 {
                Err(io::Error::last_os_error())
            } else {
                Ok(())
            }
        }

        fn try_recv(&self) -> io::Result<CanFdFrame> {
            let mut raw = CanFdFrameRaw::default();
            let ret = unsafe {
                read(
                    self.fd,
                    &mut raw as *mut CanFdFrameRaw as *mut std::ffi::c_void,
                    CANFD_MTU,
                )
            };

            if ret < 0 {
                return Err(io::Error::last_os_error());
            }

            Ok(frame_from_raw(&raw))
        }
    }

    impl Drop for SocketCanRaw {
        fn drop(&mut self) {
            unsafe { close(self.fd) };
        }
    }

    impl AsRawFd for SocketCanRaw {
        fn as_raw_fd(&self) -> RawFd {
            self.fd
        }
    }

    /// Async SocketCAN transport using tokio.
    pub struct AsyncSocketCan {
        async_fd: AsyncFd<SocketCanRaw>,
        timeout_ms: u32,
        pub(crate) info: TransportDeviceInfo,
        needs_recovery: bool,
    }

    impl AsyncSocketCan {
        /// Creates a new async SocketCAN transport.
        ///
        /// # Arguments
        /// * `interface` - CAN interface name (e.g., "can0", "vcan0")
        pub async fn new(interface: &str) -> Result<Self> {
            Self::with_options(interface, 100, false).await
        }

        /// Creates a new async SocketCAN transport with options.
        pub async fn with_options(interface: &str, timeout_ms: u32, disable_brs: bool) -> Result<Self> {
            let raw = SocketCanRaw::new(interface, disable_brs)?;
            let async_fd = AsyncFd::new(raw).map_err(Error::Io)?;

            Ok(AsyncSocketCan {
                async_fd,
                timeout_ms,
                info: TransportDeviceInfo::new(0, "AsyncSocketCan")
                    .with_serial(interface.to_string())
                    .with_detail(format!("'{}'", interface)),
                needs_recovery: false,
            })
        }

        /// Sends a single frame asynchronously.
        async fn send_frame(&self, frame: &CanFdFrame) -> Result<()> {
            loop {
                let mut guard = self.async_fd.ready(Interest::WRITABLE).await.map_err(Error::Io)?;

                match guard.try_io(|inner| inner.get_ref().try_send(frame)) {
                    Ok(result) => return result.map_err(Error::Io),
                    Err(_would_block) => continue,
                }
            }
        }

        /// Receives frames with timeout.
        async fn receive_frames(&self, expected_count: usize) -> Result<Vec<CanFdFrame>> {
            let mut frames = Vec::new();
            let timeout = std::time::Duration::from_millis(self.timeout_ms as u64);
            let deadline = tokio::time::Instant::now() + timeout;

            while frames.len() < expected_count {
                let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
                if remaining.is_zero() {
                    break;
                }

                let recv_result = tokio::time::timeout(remaining, async {
                    loop {
                        let mut guard = self.async_fd.ready(Interest::READABLE).await?;

                        match guard.try_io(|inner| inner.get_ref().try_recv()) {
                            Ok(Ok(frame)) => return Ok::<_, io::Error>(frame),
                            Ok(Err(e)) => return Err(e),
                            Err(_would_block) => continue,
                        }
                    }
                })
                .await;

                match recv_result {
                    Ok(Ok(frame)) => frames.push(frame),
                    Ok(Err(_)) | Err(_) => break,
                }
            }

            Ok(frames)
        }

        /// Execute a cycle sending frames and collecting responses.
        async fn execute_cycle(&mut self, requests: &mut [Request]) -> Result<()> {
            debug_assert!(
                requests.iter().all(|r| r.child_device.is_none()),
                "AsyncSocketCan does not support child devices"
            );

            self.needs_recovery = true;

            // Send all frames
            for req in requests.iter() {
                if let Some(frame) = &req.frame {
                    self.send_frame(frame).await?;
                }
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

    impl AsyncTransportDevice for AsyncSocketCan {
        fn recover(&mut self) -> BoxFuture<'_, Result<()>> {
            Box::pin(async move {
                if !self.needs_recovery {
                    return Ok(());
                }
                // Drain stale frames from kernel buffer (non-blocking)
                loop {
                    match self.async_fd.get_ref().try_recv() {
                        Ok(_) => continue,
                        Err(_) => break,
                    }
                }
                self.needs_recovery = false;
                Ok(())
            })
        }

        fn transaction<'a>(
            &'a mut self,
            requests: &'a mut [Request],
        ) -> BoxFuture<'a, Result<()>> {
            Box::pin(self.execute_cycle(requests))
        }

        fn write<'a>(&'a mut self, frame: &'a CanFdFrame) -> BoxFuture<'a, Result<()>> {
            Box::pin(self.send_frame(frame))
        }

        fn read(&mut self) -> BoxFuture<'_, Result<Option<CanFdFrame>>> {
            Box::pin(async move {
                // Wait indefinitely for a frame — caller wraps with tokio::time::timeout()
                loop {
                    let mut guard = self.async_fd.ready(Interest::READABLE).await.map_err(Error::Io)?;

                    match guard.try_io(|inner| inner.get_ref().try_recv()) {
                        Ok(Ok(frame)) => return Ok(Some(frame)),
                        Ok(Err(e)) => return Err(Error::Io(e)),
                        Err(_would_block) => continue,
                    }
                }
            })
        }

        fn flush(&mut self) -> BoxFuture<'_, Result<()>> {
            Box::pin(async move {
                let flush_timeout = std::time::Duration::from_millis(50);
                let deadline = tokio::time::Instant::now() + flush_timeout;

                while tokio::time::Instant::now() < deadline {
                    let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
                    if remaining.is_zero() {
                        break;
                    }

                    let recv_result = tokio::time::timeout(remaining, async {
                        let mut guard = self.async_fd.ready(Interest::READABLE).await?;

                        match guard.try_io(|inner| inner.get_ref().try_recv()) {
                            Ok(Ok(_)) => Ok::<_, io::Error>(true), // Got frame, keep flushing
                            Ok(Err(e)) => Err(e),
                            Err(_would_block) => Ok(false),
                        }
                    })
                    .await;

                    match recv_result {
                        Ok(Ok(true)) => continue, // Keep flushing
                        _ => break,
                    }
                }

                Ok(())
            })
        }

        fn info(&self) -> &TransportDeviceInfo {
            &self.info
        }
    }

}

#[cfg(target_os = "linux")]
pub use linux::AsyncSocketCan;

/// Stub for non-Linux platforms.
#[cfg(not(target_os = "linux"))]
pub struct AsyncSocketCan {
    _private: (),
}

#[cfg(not(target_os = "linux"))]
impl AsyncSocketCan {
    /// SocketCAN is only available on Linux.
    pub async fn new(_interface: &str) -> crate::error::Result<Self> {
        Err(crate::error::Error::Protocol(
            "SocketCAN is only available on Linux".to_string(),
        ))
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_async_socketcan_unavailable_on_non_linux() {
        #[cfg(not(target_os = "linux"))]
        {
            // This test verifies the module compiles on non-Linux
        }
    }

    #[cfg(target_os = "linux")]
    #[tokio::test]
    async fn test_async_socketcan_interface_not_found() {
        let result = super::AsyncSocketCan::new("nonexistent_can_interface_12345").await;
        assert!(result.is_err());
    }
}
