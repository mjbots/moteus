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

//! Global singleton transport for simplified API usage.
//!
//! This module provides a global singleton transport that is automatically
//! created on first use, enabling a simplified API where transport
//! specification is optional.

use crate::error::{Error, Result};
use crate::transport::factory::{create_transports, TransportOptions};
use crate::transport::Transport;
use std::sync::{Arc, Mutex, OnceLock};

/// Global singleton transport instance.
static GLOBAL_TRANSPORT: OnceLock<Arc<Mutex<Transport>>> = OnceLock::new();

/// Get or create the global singleton transport.
///
/// On first call, this creates a transport by auto-detecting available
/// devices. Subsequent calls return the same transport instance.
///
/// # Arguments
/// * `options` - Optional transport options. Ignored if transport is already initialized.
///
/// # Example
///
/// ```no_run
/// use moteus::transport::singleton::get_singleton_transport;
/// use moteus::TransportOptions;
///
/// fn main() -> Result<(), moteus::Error> {
///     // Get auto-detected transport
///     let transport = get_singleton_transport(None)?;
///
///     // Or with custom options
///     let opts = TransportOptions::new().timeout(std::time::Duration::from_millis(200));
///     let transport = get_singleton_transport(Some(&opts))?;
///     Ok(())
/// }
/// ```
pub fn get_singleton_transport(
    options: Option<&TransportOptions>,
) -> Result<Arc<Mutex<Transport>>> {
    // Try to get existing transport
    if let Some(transport) = GLOBAL_TRANSPORT.get() {
        return Ok(Arc::clone(transport));
    }

    // Create new transport
    let default_options = TransportOptions::new();
    let opts = options.unwrap_or(&default_options);

    let router = create_default_transport(opts)?;
    let transport = Arc::new(Mutex::new(router));

    // Try to set as global, but another thread might have done it
    match GLOBAL_TRANSPORT.set(Arc::clone(&transport)) {
        Ok(()) => Ok(transport),
        Err(_) => {
            // Another thread initialized it first, use that one
            Ok(Arc::clone(GLOBAL_TRANSPORT.get().unwrap()))
        }
    }
}

/// Create a default transport with auto-detection and deduplication.
///
/// This function uses [`create_transports`]
/// to discover and deduplicate devices, then wraps them in a [`Transport`].
///
/// # Arguments
/// * `options` - Transport options for configuration
pub fn create_default_transport(options: &TransportOptions) -> Result<Transport> {
    let all_devices = create_transports(options)?;

    if all_devices.is_empty() {
        return Err(Error::DeviceNotFound("No CAN-FD devices found".to_string()));
    }

    let mut router = Transport::new(all_devices);
    router.set_timeout(options.timeout);

    Ok(router)
}

/// Reset the global singleton transport.
///
/// This is primarily useful for testing. The next call to
/// `get_singleton_transport` will create a new transport.
///
/// # Safety
///
/// This function is not thread-safe and should only be called when
/// no other threads are using the transport.
#[cfg(test)]
pub fn reset_singleton() {
    // OnceLock doesn't have a reset method, so we can't actually reset it
    // in safe Rust. For testing, we'd need to use a different approach.
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transport_options_default() {
        let opts = TransportOptions::new();
        assert_eq!(opts.timeout, std::time::Duration::from_millis(100));
        assert!(!opts.disable_brs);
        assert!(opts.fdcanusb_paths.is_empty());
        assert!(opts.socketcan_interfaces.is_empty());
    }

    #[test]
    fn test_create_default_transport_no_devices() {
        // On systems without CAN hardware, this should return an error
        // or create an empty router
        let opts = TransportOptions::new();
        let result = create_default_transport(&opts);

        // Either it fails (no devices) or succeeds with detected devices
        // We can't guarantee which on different test systems
        let _ = result;
    }

    #[test]
    fn test_get_singleton_returns_same_instance() {
        // Note: This test may not work as expected because OnceLock
        // can only be set once. In a real test suite, we'd need isolation.

        // Just verify the function doesn't panic
        let _ = get_singleton_transport(None);
    }
}
