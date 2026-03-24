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

//! # moteus
//!
//! Rust client library for moteus brushless motor controllers.
//!
//! This crate provides a high-level API for communicating with moteus
//! controllers over CAN-FD. It supports multiple transport backends and
//! automatic device discovery.
//!
//! ## Quick Start - Auto-Discovery (Recommended)
//!
//! The simplest way to use moteus is with automatic transport discovery:
//!
//! ```rust,ignore
//! use moteus::{BlockingController, command::PositionCommand};
//!
//! fn main() -> Result<(), moteus::Error> {
//!     // Auto-discovers transport on first use
//!     let mut ctrl = BlockingController::new(1);
//!
//!     // Clear any faults
//!     ctrl.set_stop()?;
//!
//!     // Command position mode
//!     let result = ctrl.set_position(PositionCommand::new().position(0.5))?;
//!     println!("Position: {}", result.position);
//!     Ok(())
//! }
//! ```
//!
//! ## Explicit Transport
//!
//! For more control, you can specify a transport explicitly:
//!
//! ```rust,ignore
//! use moteus::{BlockingController, transport::socketcan::SocketCan};
//!
//! let transport = SocketCan::new("can0")?;
//! let mut ctrl = BlockingController::new(1)
//!     .transport(transport);
//! ```
//!
//! ## Transport Options
//!
//! Configure transport auto-detection with options:
//!
//! ```rust,ignore
//! use moteus::{BlockingController, transport::factory::TransportOptions};
//!
//! let opts = TransportOptions::new()
//!     .socketcan_interfaces(vec!["can0"])
//!     .timeout_ms(200);
//!
//! let mut ctrl = BlockingController::with_options(1, &opts);
//! ```
//!
//! ## Low-Level Protocol Access
//!
//! For custom transports or embedded use, the `Controller` type provides
//! frame building without transport:
//!
//! ```rust
//! use moteus::{Controller, command::PositionCommand};
//!
//! let controller = Controller::new(1);
//! let cmd = PositionCommand::new().position(0.5).velocity(1.0);
//! let frame = controller.make_position_command(&cmd, true);
//! // Send frame via your own transport...
//! ```
//!
//! ## Supported Transports
//!
//! - **FdCanUSB**: USB-to-CAN adapter (serial CDC)
//! - **SocketCAN**: Linux kernel CAN interface
//!
//! ## Features
//!
//! - `tokio`: Async I/O transports and `AsyncController` (optional, adds tokio dependency)
//! - `clap`: CLI argument parsing support (optional)
//!
//! ### Tokio Feature
//!
//! When the `tokio` feature is enabled, you get access to `AsyncController`
//! and async transports:
//!
//! ```rust,ignore
//! use moteus::AsyncController;
//! use moteus::command::PositionCommand;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), moteus::Error> {
//!     let mut ctrl = AsyncController::new(1);
//!
//!     ctrl.set_stop().await?;
//!     let result = ctrl.set_position(PositionCommand::new().position(0.5)).await?;
//!     println!("Position: {}", result.position);
//!     Ok(())
//! }
//! ```

mod async_controller;
mod blocking_controller;
mod command_ext;
mod command_types;
mod controller;
mod device_address;
pub mod diagnostic;
mod error;
pub mod move_to;

pub mod transport;

#[cfg(feature = "tokio")]
pub use async_controller::AsyncController;
pub use blocking_controller::BlockingController;
pub use command_ext::{CommandExt, MaybeQuery, WithQuery};
pub use command_types::Command;
pub use controller::{Controller, GpioResult};
pub use device_address::DeviceAddress;
pub use diagnostic::DiagnosticStream;
pub use error::Error;
pub use move_to::{move_to, MoveToOptions, ServoResult, Setpoint};

// Async diagnostic stream (tokio feature)
#[cfg(feature = "tokio")]
pub use diagnostic::AsyncDiagnosticStream;
#[cfg(feature = "tokio")]
pub use move_to::async_move_to;

// Re-export transport types for convenience
pub use transport::args::{transport_arg_specs, ArgSpec, ArgType, TransportArgs, COMMON_ARG_SPECS};
#[cfg(feature = "clap")]
pub use transport::args::add_transport_args;
pub use transport::factory::{register, TransportOptions};
pub use transport::{make_uuid_prefix, DeviceInfo, Transport, TransportOps};
pub use transport::singleton::{create_default_transport, get_singleton_transport};
pub use transport::transaction::{dispatch_frame, FrameFilter, Request, ResponseCollector};

// Transport traits (for polymorphism or custom transport implementations)
// Note: Most users don't need to import these - use Transport/AsyncTransport directly
pub use transport::async_transport::{AsyncTransportOps, BoxFuture};

// Re-export derive macros
pub use moteus_derive::Setters;

// Re-export protocol types for convenience
pub use moteus_protocol::{
    calculate_arbitration_id, command, parse_arbitration_id, query, CanFdFrame, HomeState, Mode,
    Register, Resolution, Scaling, Toggle, CURRENT_REGISTER_MAP_VERSION,
};

// Tokio async transport re-exports
#[cfg(feature = "tokio")]
pub use transport::async_factory::{
    create_async_transports, register_async, AsyncTransportFactory, AsyncTransportOptions,
};
#[cfg(feature = "tokio")]
pub use transport::async_fdcanusb::AsyncFdcanusb;
#[cfg(feature = "tokio")]
pub use transport::async_transport::{AsyncTransport, SharedDevice};
#[cfg(feature = "tokio")]
pub use transport::async_singleton::get_async_singleton_transport;
#[cfg(all(feature = "tokio", target_os = "linux"))]
pub use transport::async_socketcan::AsyncSocketCan;
