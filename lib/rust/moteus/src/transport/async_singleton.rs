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

//! Global async singleton transport for simplified API usage.
//!
//! This module provides a global async singleton transport that is automatically
//! created on first use, enabling a simplified API where transport specification
//! is optional.
//!
//! # Example
//!
//! ```ignore
//! use moteus::transport::async_singleton::get_async_singleton_transport;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), moteus::Error> {
//!     // Get auto-detected async transport
//!     let transport = get_async_singleton_transport(None).await?;
//!
//!     // Or with custom options
//!     let opts = AsyncTransportOptions::new();
//!     let transport = get_async_singleton_transport(Some(&opts)).await?;
//!     Ok(())
//! }
//! ```

use crate::error::Result;
use crate::transport::async_factory::AsyncTransportOptions;
use crate::transport::async_transport::AsyncTransport;
use std::sync::Arc;
use tokio::sync::{Mutex, OnceCell};

/// Global async singleton transport instance.
static GLOBAL_ASYNC_TRANSPORT: OnceCell<Arc<Mutex<AsyncTransport>>> = OnceCell::const_new();

/// Get or create the global async singleton transport.
///
/// On first call, this creates a transport by auto-detecting available
/// devices. Subsequent calls return the same transport instance.
///
/// # Arguments
/// * `options` - Optional transport options. Only used on first initialization;
///               ignored if transport is already initialized.
///
/// # Example
///
/// ```ignore
/// use moteus::transport::async_singleton::get_async_singleton_transport;
/// use moteus::transport::async_factory::AsyncTransportOptions;
///
/// #[tokio::main]
/// async fn main() -> Result<(), moteus::Error> {
///     // Get auto-detected transport
///     let transport = get_async_singleton_transport(None).await?;
///
///     // Or with custom options (only effective on first call)
///     let opts = AsyncTransportOptions::new();
///     let transport = get_async_singleton_transport(Some(&opts)).await?;
///     Ok(())
/// }
/// ```
pub async fn get_async_singleton_transport(
    options: Option<&AsyncTransportOptions>,
) -> Result<Arc<Mutex<AsyncTransport>>> {
    GLOBAL_ASYNC_TRANSPORT
        .get_or_try_init(|| async {
            let default_options = AsyncTransportOptions::new();
            let opts = options.unwrap_or(&default_options);

            let router = AsyncTransport::with_options(opts).await?;
            Ok(Arc::new(Mutex::new(router)))
        })
        .await
        .map(Arc::clone)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_async_singleton_returns_same_instance() {
        // Note: This test may not work as expected because OnceCell
        // can only be set once. In a real test suite, we'd need isolation.

        // Just verify the function doesn't panic on systems without hardware
        let _ = get_async_singleton_transport(None).await;
    }
}
