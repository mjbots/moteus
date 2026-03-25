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

//! Async transport factory system for creating async CAN-FD transports.
//!
//! This module provides the factory pattern for creating async transport devices
//! from different backends (fdcanusb, socketcan, etc.) using tokio.
//!
//! External crates can register additional async factories via [`register_async()`].

use std::sync::{Arc, Mutex, OnceLock};

use crate::error::Result;
use crate::transport::args::ArgSpec;
use crate::transport::async_transport::BoxFuture;
use crate::transport::device::AsyncTransportDevice;

/// Options for configuring async transport creation.
///
/// This is the same type as [`super::factory::TransportOptions`], re-exported
/// for convenience in async contexts.
pub use super::factory::TransportOptions as AsyncTransportOptions;

/// A factory for creating async transport devices.
///
/// Factories are tried in priority order (lower numbers first).
/// External crates implement this trait and call [`register_async()`] to
/// add themselves to the async auto-detection flow.
pub trait AsyncTransportFactory: Send + Sync {
    /// The priority of this factory (lower = tried first).
    fn priority(&self) -> u32;

    /// The name of this transport type.
    fn name(&self) -> &'static str;

    /// Command-line argument specifications for this factory.
    ///
    /// Override this to declare factory-specific CLI arguments.
    fn arg_specs(&self) -> Vec<ArgSpec> {
        Vec::new()
    }

    /// Create async transport devices using this factory.
    ///
    /// Returns a future that resolves to a list of devices.
    fn create<'a>(
        &'a self,
        options: &'a AsyncTransportOptions,
    ) -> BoxFuture<'a, Result<Vec<Box<dyn AsyncTransportDevice>>>>;
}

/// Factory for async fdcanusb devices.
#[derive(Debug, Default)]
pub struct AsyncFdcanusbFactory;

impl AsyncFdcanusbFactory {
    /// Create a new async fdcanusb factory.
    pub fn new() -> Self {
        Self
    }
}

impl AsyncTransportFactory for AsyncFdcanusbFactory {
    fn priority(&self) -> u32 {
        10 // Higher priority than socketcan
    }

    fn name(&self) -> &'static str {
        "fdcanusb"
    }

    fn arg_specs(&self) -> Vec<ArgSpec> {
        use crate::transport::args::ArgType;
        vec![ArgSpec {
            name: "fdcanusb",
            help: "Path to fdcanusb device (can be specified multiple times)",
            arg_type: ArgType::MultiString,
            default: None,
            possible_values: None,
        }]
    }

    #[cfg(target_os = "linux")]
    fn create<'a>(
        &'a self,
        options: &'a AsyncTransportOptions,
    ) -> BoxFuture<'a, Result<Vec<Box<dyn AsyncTransportDevice>>>> {
        Box::pin(async move {
            use crate::transport::async_fdcanusb::AsyncFdcanusb;
            use crate::transport::device::TransportDeviceInfo;
            use crate::transport::discovery::{detect_fdcanusbs, FdcanusbInfo};

            let infos: Vec<FdcanusbInfo> = if options.fdcanusb_paths.is_empty() {
                detect_fdcanusbs()
            } else {
                options
                    .fdcanusb_paths
                    .iter()
                    .map(|path| FdcanusbInfo {
                        path: path.clone(),
                        serial_number: None,
                    })
                    .collect()
            };

            let mut devices: Vec<Box<dyn AsyncTransportDevice>> = Vec::new();
            for (idx, info) in infos.iter().enumerate() {
                match AsyncFdcanusb::open_with_options(
                    &info.path,
                    options.timeout,
                    options.disable_brs,
                )
                .await
                {
                    Ok(mut transport) => {
                        // Update device info
                        let mut dev_info = TransportDeviceInfo::new(idx, "AsyncFdcanusb");
                        if let Some(ref sn) = info.serial_number {
                            dev_info.detail = Some(format!("sn='{}'", sn));
                            dev_info.serial_number = Some(sn.clone());
                        }
                        transport.info = dev_info;
                        devices.push(Box::new(transport));
                    }
                    Err(_) => continue,
                }
            }

            Ok(devices)
        })
    }

    #[cfg(not(target_os = "linux"))]
    fn create<'a>(
        &'a self,
        _options: &'a AsyncTransportOptions,
    ) -> BoxFuture<'a, Result<Vec<Box<dyn AsyncTransportDevice>>>> {
        Box::pin(async move { Ok(Vec::new()) })
    }
}

/// Factory for async socketcan devices (Linux only).
#[cfg(target_os = "linux")]
#[derive(Debug, Default)]
pub struct AsyncSocketCanFactory;

#[cfg(target_os = "linux")]
impl AsyncSocketCanFactory {
    /// Create a new async socketcan factory.
    pub fn new() -> Self {
        Self
    }
}

#[cfg(target_os = "linux")]
impl AsyncTransportFactory for AsyncSocketCanFactory {
    fn priority(&self) -> u32 {
        11 // Lower priority than fdcanusb
    }

    fn name(&self) -> &'static str {
        "socketcan"
    }

    fn arg_specs(&self) -> Vec<ArgSpec> {
        use crate::transport::args::ArgType;
        vec![ArgSpec {
            name: "can-chan",
            help: "SocketCAN interface (can be specified multiple times)",
            arg_type: ArgType::MultiString,
            default: None,
            possible_values: None,
        }]
    }

    fn create<'a>(
        &'a self,
        options: &'a AsyncTransportOptions,
    ) -> BoxFuture<'a, Result<Vec<Box<dyn AsyncTransportDevice>>>> {
        Box::pin(async move {
            use crate::transport::async_socketcan::AsyncSocketCan;
            use crate::transport::device::TransportDeviceInfo;
            use crate::transport::discovery::detect_socketcan_interfaces;

            let interfaces = if options.socketcan_interfaces.is_empty() {
                detect_socketcan_interfaces()
                    .into_iter()
                    .map(|info| info.interface)
                    .collect()
            } else {
                options.socketcan_interfaces.clone()
            };

            let mut devices: Vec<Box<dyn AsyncTransportDevice>> = Vec::new();
            for (idx, interface) in interfaces.iter().enumerate() {
                match AsyncSocketCan::with_options(interface, options.timeout, options.disable_brs)
                    .await
                {
                    Ok(mut transport) => {
                        transport.info = TransportDeviceInfo::new(idx, "AsyncSocketCan")
                            .with_serial(interface)
                            .with_detail(format!("'{}'", interface));
                        devices.push(Box::new(transport));
                    }
                    Err(_) => continue,
                }
            }

            Ok(devices)
        })
    }
}

// -- Global async transport factory registry --

static ASYNC_REGISTRY: OnceLock<Mutex<Vec<Arc<dyn AsyncTransportFactory>>>> = OnceLock::new();

fn get_async_registry() -> &'static Mutex<Vec<Arc<dyn AsyncTransportFactory>>> {
    ASYNC_REGISTRY.get_or_init(|| {
        let mut factories: Vec<Arc<dyn AsyncTransportFactory>> =
            vec![Arc::new(AsyncFdcanusbFactory)];
        #[cfg(target_os = "linux")]
        factories.push(Arc::new(AsyncSocketCanFactory));
        Mutex::new(factories)
    })
}

/// Register an external async transport factory.
///
/// The factory will be included in all subsequent calls to
/// [`get_async_factories()`], [`create_async_transports()`], and
/// async singleton creation.
pub fn register_async(factory: Arc<dyn AsyncTransportFactory>) {
    get_async_registry().lock().unwrap().push(factory);
}

/// Get the built-in async transport factories.
///
/// Returns fresh instances of the built-in factories. For the full set
/// of registered factories (including external ones), use
/// [`create_async_transports()`] which reads the global registry.
pub fn get_async_factories() -> Vec<Box<dyn AsyncTransportFactory>> {
    let mut factories: Vec<Box<dyn AsyncTransportFactory>> =
        vec![Box::new(AsyncFdcanusbFactory::new())];
    #[cfg(target_os = "linux")]
    factories.push(Box::new(AsyncSocketCanFactory::new()));
    factories
}

/// Create async transport devices using all registered factories.
///
/// This is the main entry point for creating async transports with auto-discovery.
/// It tries each factory in priority order and returns all successfully created devices.
/// Duplicate socketcan interfaces backed by fdcanusb devices are filtered out.
pub async fn create_async_transports(
    options: &AsyncTransportOptions,
) -> Result<Vec<Box<dyn AsyncTransportDevice>>> {
    use std::collections::HashSet;

    // Snapshot the registry under lock, then release the lock before awaiting.
    let mut factories: Vec<Arc<dyn AsyncTransportFactory>> = {
        let registry = get_async_registry().lock().unwrap();
        registry.clone()
    };

    // Sort by priority
    factories.sort_by_key(|f| f.priority());

    // Filter by forced transport if specified
    if let Some(ref force) = options.force_transport {
        factories.retain(|f| f.name() == force.as_str());
    }

    let mut all_devices = Vec::new();
    let mut fdcanusb_serials: HashSet<String> = HashSet::new();

    for factory in &factories {
        match factory.create(options).await {
            Ok(devices) => {
                // Track fdcanusb serial numbers for deduplication
                if factory.name() == "fdcanusb" {
                    for device in &devices {
                        if let Some(serial) = device.info().serial_number.as_ref() {
                            fdcanusb_serials.insert(serial.clone());
                        }
                    }
                }
                all_devices.extend(devices);
            }
            Err(_) => continue,
        }
    }

    // Deduplicate: remove socketcan interfaces that are backed by fdcanusb
    // devices we're already using via CDC serial
    #[cfg(target_os = "linux")]
    {
        use crate::transport::discovery::detect_socketcan_interfaces;

        let socketcan_infos = detect_socketcan_interfaces();
        let mut filtered_devices = Vec::new();

        for device in all_devices {
            // Check if this is a socketcan device that duplicates an fdcanusb
            let should_skip = socketcan_infos.iter().any(|info| {
                // For SocketCan devices, the interface name is stored in serial_number
                (device.info().serial_number.as_ref() == Some(&info.interface))
                    && info
                        .fdcanusb_serial
                        .as_ref()
                        .is_some_and(|serial| fdcanusb_serials.contains(serial))
            });

            if !should_skip {
                filtered_devices.push(device);
            }
        }

        all_devices = filtered_devices;
    }

    Ok(all_devices)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_async_factory_priorities() {
        let fdcanusb = AsyncFdcanusbFactory::new();
        assert_eq!(fdcanusb.priority(), 10);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn test_async_socketcan_factory_priority() {
        let socketcan = AsyncSocketCanFactory::new();
        assert!(AsyncFdcanusbFactory::new().priority() < socketcan.priority());
    }

    #[test]
    fn test_async_factory_arg_specs() {
        let fdcanusb = AsyncFdcanusbFactory::new();
        let specs = fdcanusb.arg_specs();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].name, "fdcanusb");
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn test_async_socketcan_factory_arg_specs() {
        let socketcan = AsyncSocketCanFactory::new();
        let specs = socketcan.arg_specs();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].name, "can-chan");
    }

    #[test]
    fn test_get_async_factories() {
        let factories = get_async_factories();
        assert!(!factories.is_empty());

        let names: Vec<_> = factories.iter().map(|f| f.name()).collect();
        assert!(names.contains(&"fdcanusb"));
    }
}
