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

//! # moteus-protocol
//!
//! Low-level protocol types for moteus brushless motor controllers.
//!
//! This crate provides `no_std` compatible types for encoding and decoding
//! CAN-FD frames used to communicate with moteus controllers. It is designed
//! to be usable on embedded systems without an allocator, as well as in
//! standard environments.
//!
//! ## Features
//!
//! - `std` (default): When disabled, the crate is built as `no_std`
//!
//! ## Example
//!
//! ```rust
//! use moteus_protocol::{Register, Mode, Resolution, CanFdFrame};
//! use moteus_protocol::command::PositionCommand;
//! use moteus_protocol::query::QueryFormat;
//!
//! // Build a position command frame
//! let mut frame = CanFdFrame::new();
//! let cmd = PositionCommand {
//!     position: Some(0.5),
//!     velocity: Some(1.0),
//!     ..Default::default()
//! };
//! cmd.serialize(&mut frame, &Default::default());
//!
//! // Add a query for telemetry
//! let query = QueryFormat::default();
//! query.serialize(&mut frame);
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

mod frame;
mod mode;
mod multiplex;
mod register;
mod resolution;
mod scaling;

pub mod command;
pub mod query;

pub use frame::{CanFdFrame, Toggle, calculate_arbitration_id, parse_arbitration_id};
pub use mode::{HomeState, Mode};
pub use multiplex::{
    FrameParser, Multiplex, Subframe, SubframeType, Value, WriteCanData, WriteCombiner,
    parse_frame,
};
pub use register::Register;
pub use resolution::Resolution;
pub use scaling::{read_scaled, Scaling};

/// The current register map version expected from moteus controllers.
/// If the version differs, semantics of one or more registers may have changed.
pub const CURRENT_REGISTER_MAP_VERSION: u8 = 5;
