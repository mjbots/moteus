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

//! Single-bus convenience transports.
//!
//! Each type here is a thin wrapper that holds a
//! [`Router`](crate::transport::Router) over a single device and presents the
//! [`Transport`](crate::transport::Transport) interface directly, so it can be
//! handed straight to a controller without first constructing a `Router`.
//!
//! They mirror the Python library's `Fdcanusb`/`PythonCan` convenience
//! transports (which wrap a single `TransportDevice` in a `Transport`). For
//! multi-bus configurations, build a [`Router`](crate::transport::Router)
//! explicitly instead.

#[allow(unused_imports)]
use crate::error::Result;
#[allow(unused_imports)]
use moteus_protocol::CanFdFrame;
#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use crate::transport::transaction::Request;

/// Implement the blocking [`Transport`](crate::transport::Transport) trait for a
/// newtype `$t(Router)` by forwarding to the inner `Router`.
#[cfg(any(feature = "serialport", target_os = "linux"))]
macro_rules! forward_transport {
    ($t:ty) => {
        impl $crate::transport::Transport for $t {
            fn cycle(&mut self, requests: &mut [Request]) -> Result<()> {
                self.0.cycle(requests)
            }
            fn write(&mut self, frame: &CanFdFrame) -> Result<()> {
                self.0.write(frame)
            }
            fn read(&mut self, channel: Option<usize>) -> Result<Option<CanFdFrame>> {
                self.0.read(channel)
            }
            fn flush_read(&mut self, channel: Option<usize>) -> Result<()> {
                self.0.flush_read(channel)
            }
            fn set_timeout(&mut self, timeout: Duration) {
                self.0.set_timeout(timeout)
            }
            fn timeout(&self) -> Duration {
                self.0.timeout()
            }
        }
    };
}

/// Implement the async [`AsyncTransport`](crate::transport::async_transport::AsyncTransport)
/// trait for a newtype `$t(AsyncRouter)` by forwarding to the inner `AsyncRouter`.
#[cfg(feature = "tokio")]
macro_rules! forward_async_transport {
    ($t:ty) => {
        impl $crate::transport::async_transport::AsyncTransport for $t {
            fn cycle<'a>(
                &'a mut self,
                requests: &'a mut [Request],
            ) -> $crate::transport::async_transport::BoxFuture<'a, Result<()>> {
                Box::pin(self.0.cycle(requests))
            }
            fn write<'a>(
                &'a mut self,
                frame: &'a CanFdFrame,
            ) -> $crate::transport::async_transport::BoxFuture<'a, Result<()>> {
                Box::pin(self.0.write(frame))
            }
            fn read(
                &mut self,
                channel: Option<usize>,
            ) -> $crate::transport::async_transport::BoxFuture<'_, Result<Option<CanFdFrame>>> {
                Box::pin(self.0.read(channel))
            }
            fn flush_read(
                &mut self,
                channel: Option<usize>,
            ) -> $crate::transport::async_transport::BoxFuture<'_, Result<()>> {
                Box::pin(self.0.flush_read(channel))
            }
        }
    };
}

// ===========================================================================
// Blocking convenience transports
// ===========================================================================

/// A ready-to-use blocking transport over a single fdcanusb.
///
/// Available on all platforms (requires the default `serialport` feature).
///
/// ```no_run
/// use moteus::{BlockingController, Fdcanusb};
///
/// # fn main() -> Result<(), moteus::Error> {
/// let mut ctrl = BlockingController::with_transport(1, Fdcanusb::open("/dev/fdcanusb")?);
/// ctrl.set_stop()?;
/// # Ok(())
/// # }
/// ```
#[cfg(feature = "serialport")]
pub struct Fdcanusb(crate::transport::Router);

#[cfg(feature = "serialport")]
impl Fdcanusb {
    /// Open an fdcanusb at the given serial port path.
    ///
    /// The path may also refer to a moteus connected directly over a
    /// TTL UART (see `docs/integration/uart.md`); such devices are
    /// auto-detected and handled with per-frame retries and CRC-8
    /// checksums.
    pub fn open(path: &str) -> Result<Self> {
        let device = crate::transport::fdcanusb::FdcanusbDevice::new(path)?;
        Ok(Self(crate::transport::Router::from_device(device)))
    }

    /// Open an fdcanusb with an explicit timeout and BRS setting.
    pub fn with_options(path: &str, timeout: Duration, disable_brs: bool) -> Result<Self> {
        let device =
            crate::transport::fdcanusb::FdcanusbDevice::with_options(path, timeout, disable_brs)?;
        Ok(Self(crate::transport::Router::from_device(device)))
    }

    /// Open an fdcanusb or moteus UART with full options.
    pub fn open_with(
        path: &str,
        options: &crate::transport::fdcanusb::FdcanusbOptions,
    ) -> Result<Self> {
        let device = crate::transport::fdcanusb::FdcanusbDevice::open_with(path, options)?;
        Ok(Self(crate::transport::Router::from_device(device)))
    }
}

#[cfg(feature = "serialport")]
forward_transport!(Fdcanusb);

/// A ready-to-use blocking transport over a single SocketCAN interface
/// (Linux only).
///
/// ```no_run
/// use moteus::{BlockingController, SocketCan};
///
/// # fn main() -> Result<(), moteus::Error> {
/// let mut ctrl = BlockingController::with_transport(1, SocketCan::open("can0")?);
/// ctrl.set_stop()?;
/// # Ok(())
/// # }
/// ```
#[cfg(target_os = "linux")]
pub struct SocketCan(crate::transport::Router);

#[cfg(target_os = "linux")]
impl SocketCan {
    /// Open a SocketCAN interface by name (e.g. `"can0"`).
    pub fn open(interface: &str) -> Result<Self> {
        let device = crate::transport::socketcan::SocketCanDevice::new(interface)?;
        Ok(Self(crate::transport::Router::from_device(device)))
    }

    /// Open a SocketCAN interface with an explicit timeout and BRS setting.
    pub fn with_options(interface: &str, timeout: Duration, disable_brs: bool) -> Result<Self> {
        let device = crate::transport::socketcan::SocketCanDevice::with_options(
            interface,
            timeout,
            disable_brs,
        )?;
        Ok(Self(crate::transport::Router::from_device(device)))
    }
}

#[cfg(target_os = "linux")]
forward_transport!(SocketCan);

// ===========================================================================
// Async convenience transports
// ===========================================================================

/// A ready-to-use async transport over a single fdcanusb.
///
/// Available on all platforms (uses `tokio-serial`).
#[cfg(feature = "tokio")]
pub struct AsyncFdcanusb(crate::transport::async_transport::AsyncRouter);

#[cfg(feature = "tokio")]
impl AsyncFdcanusb {
    /// Open an fdcanusb at the given serial port path.
    ///
    /// The path may also refer to a moteus connected directly over a
    /// TTL UART (see `docs/integration/uart.md`); such devices are
    /// auto-detected and handled with per-frame retries and CRC-8
    /// checksums.
    pub async fn open(path: &str) -> Result<Self> {
        let device = crate::transport::async_fdcanusb::AsyncFdcanusbDevice::open(path).await?;
        Ok(Self(
            crate::transport::async_transport::AsyncRouter::from_device(device),
        ))
    }

    /// Open an fdcanusb with an explicit BRS setting.
    pub async fn open_with_brs(path: &str, disable_brs: bool) -> Result<Self> {
        let device =
            crate::transport::async_fdcanusb::AsyncFdcanusbDevice::open_with_brs(path, disable_brs)
                .await?;
        Ok(Self(
            crate::transport::async_transport::AsyncRouter::from_device(device),
        ))
    }

    /// Open an fdcanusb or moteus UART with full options.
    pub async fn open_with(
        path: &str,
        options: &crate::transport::fdcanusb::FdcanusbOptions,
    ) -> Result<Self> {
        let device =
            crate::transport::async_fdcanusb::AsyncFdcanusbDevice::open_with(path, options).await?;
        Ok(Self(
            crate::transport::async_transport::AsyncRouter::from_device(device),
        ))
    }
}

#[cfg(feature = "tokio")]
forward_async_transport!(AsyncFdcanusb);

/// A ready-to-use async transport over a single SocketCAN interface.
#[cfg(all(feature = "tokio", target_os = "linux"))]
pub struct AsyncSocketCan(crate::transport::async_transport::AsyncRouter);

#[cfg(all(feature = "tokio", target_os = "linux"))]
impl AsyncSocketCan {
    /// Open a SocketCAN interface by name (e.g. `"can0"`).
    pub async fn open(interface: &str) -> Result<Self> {
        let device =
            crate::transport::async_socketcan::AsyncSocketCanDevice::new(interface).await?;
        Ok(Self(
            crate::transport::async_transport::AsyncRouter::from_device(device),
        ))
    }
}

#[cfg(all(feature = "tokio", target_os = "linux"))]
forward_async_transport!(AsyncSocketCan);
