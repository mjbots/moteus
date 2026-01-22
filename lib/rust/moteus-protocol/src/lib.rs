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
//! Low-level protocol encoding for moteus brushless motor controllers.
//!
//! This crate provides the core types and encoding/decoding functions for
//! communicating with moteus controllers over CAN-FD.
//!
//! ## no_std Support
//!
//! This crate supports `no_std` environments when the `std` feature is disabled.

#![cfg_attr(not(feature = "std"), no_std)]

mod frame;
mod mode;
mod multiplex;
mod register;
mod resolution;
mod scaling;

pub use frame::{CanFdFrame, Toggle, calculate_arbitration_id, parse_arbitration_id};
pub use mode::{HomeState, Mode};
pub use multiplex::{
    FrameParser, Multiplex, Subframe, SubframeType, Value, WriteCanData, WriteCombiner,
    parse_frame,
};
pub use register::Register;
pub use resolution::Resolution;
pub use scaling::{read_scaled, Scaling};
