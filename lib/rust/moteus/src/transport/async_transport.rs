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

//! Async transport traits for moteus communication.
//!
//! This module defines the `AsyncTransportOps` trait and `BoxFuture` type alias
//! used by async transport implementations.

use crate::error::Result;
use crate::transport::transaction::Request;
use moteus_protocol::CanFdFrame;
use std::future::Future;
use std::pin::Pin;

/// Type alias for a boxed future that can be sent across threads.
pub type BoxFuture<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;

// =============================================================================
// AsyncTransportOps Trait
// =============================================================================

/// Trait for async transport operations.
///
/// This trait defines the common interface for async transport implementations.
///
/// This trait is useful for:
/// - Writing generic code that works with any async transport
/// - Implementing custom async transports
/// - Using `Box<dyn AsyncTransportOps>` for polymorphism
pub trait AsyncTransportOps: Send {
    /// Executes a cycle: sends frames and collects responses asynchronously.
    fn cycle<'a>(&'a mut self, requests: &'a mut [Request]) -> BoxFuture<'a, Result<()>>;

    /// Sends a frame without waiting for a response.
    fn write<'a>(&'a mut self, frame: &'a CanFdFrame) -> BoxFuture<'a, Result<()>>;

    /// Receives an unsolicited frame, if available.
    fn read<'a>(&'a mut self, channel: Option<usize>) -> BoxFuture<'a, Result<Option<CanFdFrame>>>;

    /// Flushes any pending unsolicited frames.
    fn flush_read<'a>(&'a mut self, channel: Option<usize>) -> BoxFuture<'a, Result<()>>;
}
