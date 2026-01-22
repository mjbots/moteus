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
//! controllers over CAN-FD.

mod error;
pub mod transport;

pub use error::Error;

// Re-export protocol types for convenience
pub use moteus_protocol::{
    calculate_arbitration_id, command, parse_arbitration_id, query, CanFdFrame, HomeState, Mode,
    Register, Resolution, Scaling, Toggle, CURRENT_REGISTER_MAP_VERSION,
};

// Re-export transport types
pub use transport::{
    dispatch_frame, FrameFilter, NullTransport, Request, ResponseCollector, TransportDevice,
    TransportDeviceInfo, TransportOps,
};

// Transport traits (for polymorphism or custom transport implementations)
pub use transport::async_transport::{AsyncTransportOps, BoxFuture};
