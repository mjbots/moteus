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

#![doc = include_str!("../README.md")]
#![no_std]

mod frame;
mod mode;
pub(crate) mod multiplex;
mod register;
mod resolution;
pub mod scaling;

pub mod command;
pub mod query;

pub use frame::{calculate_arbitration_id, parse_arbitration_id, CanFdFrame, Toggle};
pub use mode::{HomeState, Mode};
pub use multiplex::{
    parse_frame, FrameParser, Subframe, SubframeType, Value, WriteCanData, WriteCombiner,
    CLIENT_POLL_SERVER, CLIENT_TO_SERVER, NOP, READ_ERROR, READ_FLOAT, READ_INT16, READ_INT32,
    READ_INT8, REPLY_FLOAT, REPLY_INT16, REPLY_INT32, REPLY_INT8, SERVER_TO_CLIENT, WRITE_ERROR,
    WRITE_FLOAT, WRITE_INT16, WRITE_INT32, WRITE_INT8,
};
pub use register::Register;
pub use resolution::Resolution;
pub use scaling::Scaling;

/// The current register map version expected from moteus controllers.
/// If the version differs, semantics of one or more registers may have changed.
pub const CURRENT_REGISTER_MAP_VERSION: u8 = 5;
