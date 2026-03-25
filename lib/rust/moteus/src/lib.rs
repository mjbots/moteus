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

pub mod command_ext;
pub mod command_types;
pub mod controller;
pub mod device_address;
pub mod diagnostic;
pub mod error;
pub mod move_to;
pub mod transport;

// Core types that most users need
#[cfg(feature = "tokio")]
pub use async_controller::AsyncController;
pub use blocking_controller::BlockingController;
pub use controller::Controller;
pub use device_address::DeviceAddress;
pub use error::Error;
pub use transport::factory::TransportOptions;
pub use transport::{Transport, TransportOps};

// Re-export protocol types for convenience
pub use moteus_protocol::{command, query, CanFdFrame, Mode, Register, Resolution};

// These modules are public for advanced use but not re-exported at the root.
// Access via moteus::diagnostic::DiagnosticStream, moteus::move_to::Setpoint, etc.
mod async_controller;
mod blocking_controller;
