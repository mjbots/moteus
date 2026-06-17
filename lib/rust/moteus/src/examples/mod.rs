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

//! Runnable bodies of the example programs, exposed as a library so
//! that downstream binaries can reuse them with a different set of
//! transports.
//!
//! Each submodule exposes a single
//! `pub fn run(register_transports: impl FnOnce()) -> Result<(), Error>`.
//! The corresponding `examples/<name>.rs` binary is a thin wrapper that
//! calls it with a no-op hook; an out-of-tree transport (e.g. a pi3hat)
//! can instead pass its own registration function so the example talks
//! over that transport:
//!
//! ```no_run
//! # #[cfg(feature = "examples")]
//! # fn demo() -> Result<(), moteus::Error> {
//! moteus::examples::simple::run(|| { /* register extra transports */ })
//! # }
//! ```
//!
//! This module is gated behind the `examples` feature.

pub mod bandwidth_test;
pub mod diagnostic_protocol;
pub mod discover;
pub mod gpio;
pub mod move_to;
pub mod multiservo;
pub mod simple;
