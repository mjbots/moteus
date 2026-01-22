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

//! Transport factory system for creating CAN-FD transports.
//!
//! This module provides the factory pattern for creating transport devices
//! from different backends (fdcanusb, socketcan, etc.).

use crate::error::Result;
use crate::transport::device::TransportDevice;

/// Default communication timeout in milliseconds.
pub const DEFAULT_TIMEOUT_MS: u32 = 100;

/// Options for configuring transport creation.
#[derive(Debug, Clone, Default)]
pub struct TransportOptions {
    /// Explicit fdcanusb device paths to use.
    pub fdcanusb_paths: Vec<String>,
    /// Explicit socketcan interfaces to use.
    pub socketcan_interfaces: Vec<String>,
    /// Disable bit rate switching (BRS) for CAN-FD.
    pub disable_brs: bool,
    /// Force a specific transport type ("fdcanusb" or "socketcan").
    pub force_transport: Option<String>,
    /// Communication timeout in milliseconds.
    pub timeout_ms: u32,
}

impl TransportOptions {
    /// Create new transport options with default values.
    pub fn new() -> Self {
        Self {
            timeout_ms: DEFAULT_TIMEOUT_MS,
            ..Default::default()
        }
    }

    /// Set explicit fdcanusb paths.
    pub fn fdcanusb_paths(mut self, paths: impl IntoIterator<Item = impl Into<String>>) -> Self {
        self.fdcanusb_paths = paths.into_iter().map(Into::into).collect();
        self
    }

    /// Set explicit socketcan interfaces.
    pub fn socketcan_interfaces(
        mut self,
        interfaces: impl IntoIterator<Item = impl Into<String>>,
    ) -> Self {
        self.socketcan_interfaces = interfaces.into_iter().map(Into::into).collect();
        self
    }

    /// Disable bit rate switching.
    pub fn disable_brs(mut self, disable: bool) -> Self {
        self.disable_brs = disable;
        self
    }

    /// Force a specific transport type.
    pub fn force_transport(mut self, transport: impl Into<String>) -> Self {
        self.force_transport = Some(transport.into());
        self
    }

    /// Set the communication timeout.
    pub fn timeout_ms(mut self, timeout: u32) -> Self {
        self.timeout_ms = timeout;
        self
    }

    /// Parse transport options from key-value pairs.
    ///
    /// This method allows creating transport options from any CLI parser by
    /// converting parsed arguments to key-value pairs. Keys match the standard
    /// command-line argument names (without leading dashes).
    ///
    /// # Supported Keys
    ///
    /// - `fdcanusb`: Path to fdcanusb device (can appear multiple times)
    /// - `can-chan`: SocketCAN interface name (can appear multiple times)
    /// - `can-disable-brs`: "true" to disable CAN-FD bit rate switching
    /// - `force-transport`: "fdcanusb" or "socketcan"
    /// - `timeout-ms`: Communication timeout in milliseconds
    ///
    /// # Example
    ///
    /// ```
    /// use moteus::transport::factory::TransportOptions;
    ///
    /// let opts = TransportOptions::from_pairs([
    ///     ("can-chan", "can0"),
    ///     ("timeout-ms", "200"),
    /// ]).unwrap();
    /// assert_eq!(opts.socketcan_interfaces, vec!["can0"]);
    /// assert_eq!(opts.timeout_ms, 200);
    /// ```
    pub fn from_pairs<'a>(
        pairs: impl IntoIterator<Item = (&'a str, &'a str)>,
    ) -> std::result::Result<Self, String> {
        let mut opts = TransportOptions::new();
        for (key, value) in pairs {
            match key {
                "fdcanusb" => opts.fdcanusb_paths.push(value.to_string()),
                "can-chan" => opts.socketcan_interfaces.push(value.to_string()),
                "can-disable-brs" => opts.disable_brs = value == "true",
                "force-transport" => {
                    if value != "fdcanusb" && value != "socketcan" {
                        return Err(format!("invalid transport: {}", value));
                    }
                    opts.force_transport = Some(value.to_string());
                }
                "timeout-ms" => {
                    opts.timeout_ms = value
                        .parse()
                        .map_err(|_| format!("invalid timeout: {}", value))?;
                }
                _ => return Err(format!("unknown option: {}", key)),
            }
        }
        Ok(opts)
    }
}

/// A factory for creating transport devices.
///
/// Factories are tried in priority order (lower numbers first).
pub trait TransportFactory: Send + Sync {
    /// The priority of this factory (lower = tried first).
    fn priority(&self) -> u32;

    /// The name of this transport type.
    fn name(&self) -> &'static str;

    /// Create transport devices using this factory.
    ///
    /// Returns a list of devices that were successfully created.
    fn create(&self, options: &TransportOptions) -> Result<Vec<Box<dyn TransportDevice>>>;
}

/// Factory for fdcanusb devices (CDC serial).
#[derive(Debug, Default)]
pub struct FdcanusbFactory;

impl FdcanusbFactory {
    /// Create a new fdcanusb factory.
    pub fn new() -> Self {
        Self
    }
}

impl TransportFactory for FdcanusbFactory {
    fn priority(&self) -> u32 {
        10 // Higher priority than socketcan
    }

    fn name(&self) -> &'static str {
        "fdcanusb"
    }

    #[cfg(target_os = "linux")]
    fn create(&self, options: &TransportOptions) -> Result<Vec<Box<dyn TransportDevice>>> {
        use crate::transport::discovery::{detect_fdcanusbs, FdcanusbInfo};
        use crate::transport::fdcanusb::Fdcanusb;

        let infos: Vec<FdcanusbInfo> = if options.fdcanusb_paths.is_empty() {
            // Auto-detect fdcanusb devices
            detect_fdcanusbs()
        } else {
            // Convert explicit paths to FdcanusbInfo (no serial number)
            options.fdcanusb_paths
                .iter()
                .map(|path| FdcanusbInfo {
                    path: path.clone(),
                    serial_number: None,
                })
                .collect()
        };

        let mut devices: Vec<Box<dyn TransportDevice>> = Vec::new();
        for (idx, info) in infos.iter().enumerate() {
            match Fdcanusb::with_options(&info.path, options.timeout_ms, options.disable_brs) {
                Ok(mut transport) => {
                    transport.info.id = idx;
                    transport.info.serial_number = info.serial_number.clone();
                    transport.info.detail =
                        info.serial_number.as_ref().map(|sn| format!("sn='{}'", sn));
                    devices.push(Box::new(transport));
                }
                Err(_) => continue, // Skip devices that fail to open
            }
        }

        Ok(devices)
    }

    #[cfg(not(target_os = "linux"))]
    fn create(&self, _options: &TransportOptions) -> Result<Vec<Box<dyn TransportDevice>>> {
        // fdcanusb requires platform-specific serial port implementation
        Ok(Vec::new())
    }
}

/// Factory for socketcan devices (Linux only).
#[derive(Debug, Default)]
pub struct SocketCanFactory;

impl SocketCanFactory {
    /// Create a new socketcan factory.
    pub fn new() -> Self {
        Self
    }
}

impl TransportFactory for SocketCanFactory {
    fn priority(&self) -> u32 {
        11 // Lower priority than fdcanusb
    }

    fn name(&self) -> &'static str {
        "socketcan"
    }

    #[cfg(target_os = "linux")]
    fn create(&self, options: &TransportOptions) -> Result<Vec<Box<dyn TransportDevice>>> {
        use crate::transport::discovery::detect_socketcan_interfaces;
        use crate::transport::socketcan::SocketCan;

        let interfaces = if options.socketcan_interfaces.is_empty() {
            // Auto-detect interfaces
            detect_socketcan_interfaces()
                .into_iter()
                .map(|info| info.interface)
                .collect()
        } else {
            options.socketcan_interfaces.clone()
        };

        let mut devices: Vec<Box<dyn TransportDevice>> = Vec::new();
        for (idx, interface) in interfaces.iter().enumerate() {
            match SocketCan::with_options(interface, options.timeout_ms, options.disable_brs) {
                Ok(mut transport) => {
                    transport.info.id = idx;
                    devices.push(Box::new(transport));
                }
                Err(_) => continue, // Skip interfaces that fail to open
            }
        }

        Ok(devices)
    }

    #[cfg(not(target_os = "linux"))]
    fn create(&self, _options: &TransportOptions) -> Result<Vec<Box<dyn TransportDevice>>> {
        Ok(Vec::new())
    }
}

/// Get all available transport factories.
pub fn get_factories() -> Vec<Box<dyn TransportFactory>> {
    vec![
        Box::new(FdcanusbFactory::new()),
        Box::new(SocketCanFactory::new()),
    ]
}

/// Create transport devices using all available factories.
///
/// This is the main entry point for creating transports with auto-discovery.
/// It tries each factory in priority order and returns all successfully created
/// devices. Duplicate socketcan interfaces backed by fdcanusb devices are
/// filtered out.
pub fn create_transports(
    options: &TransportOptions,
) -> Result<Vec<Box<dyn TransportDevice>>> {
    use std::collections::HashSet;

    let mut factories = get_factories();

    // Sort by priority
    factories.sort_by_key(|f| f.priority());

    // Filter by forced transport if specified
    if let Some(ref force) = options.force_transport {
        factories.retain(|f| f.name() == force.as_str());
    }

    let mut all_devices = Vec::new();
    let mut fdcanusb_serials: HashSet<String> = HashSet::new();

    for factory in &factories {
        match factory.create(options) {
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
            let should_skip = socketcan_infos.iter().any(|info| {
                device
                    .info()
                    .serial_number
                    .as_ref()
                    .map_or(false, |s| s == &info.interface)
                    && info.fdcanusb_serial.as_ref().map_or(false, |serial| {
                        fdcanusb_serials.contains(serial)
                    })
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
    fn test_transport_options_builder() {
        let opts = TransportOptions::new()
            .fdcanusb_paths(vec!["/dev/ttyACM0", "/dev/ttyACM1"])
            .socketcan_interfaces(vec!["can0"])
            .disable_brs(true)
            .timeout_ms(200);

        assert_eq!(opts.fdcanusb_paths, vec!["/dev/ttyACM0", "/dev/ttyACM1"]);
        assert_eq!(opts.socketcan_interfaces, vec!["can0"]);
        assert!(opts.disable_brs);
        assert_eq!(opts.timeout_ms, 200);
    }

    #[test]
    fn test_factory_priorities() {
        let fdcanusb = FdcanusbFactory::new();
        let socketcan = SocketCanFactory::new();

        assert!(fdcanusb.priority() < socketcan.priority());
    }

    #[test]
    fn test_get_factories() {
        let factories = get_factories();
        assert!(!factories.is_empty());

        // Verify they are sorted by priority implicitly
        let names: Vec<_> = factories.iter().map(|f| f.name()).collect();
        assert!(names.contains(&"fdcanusb"));
        assert!(names.contains(&"socketcan"));
    }

    #[test]
    fn test_from_pairs_basic() {
        let opts = TransportOptions::from_pairs([
            ("can-chan", "can0"),
            ("timeout-ms", "200"),
        ])
        .unwrap();
        assert_eq!(opts.socketcan_interfaces, vec!["can0"]);
        assert_eq!(opts.timeout_ms, 200);
    }

    #[test]
    fn test_from_pairs_multiple_devices() {
        let opts = TransportOptions::from_pairs([
            ("fdcanusb", "/dev/ttyACM0"),
            ("fdcanusb", "/dev/ttyACM1"),
            ("can-chan", "can0"),
            ("can-chan", "can1"),
        ])
        .unwrap();
        assert_eq!(opts.fdcanusb_paths, vec!["/dev/ttyACM0", "/dev/ttyACM1"]);
        assert_eq!(opts.socketcan_interfaces, vec!["can0", "can1"]);
    }

    #[test]
    fn test_from_pairs_flags() {
        let opts = TransportOptions::from_pairs([
            ("can-disable-brs", "true"),
            ("force-transport", "socketcan"),
        ])
        .unwrap();
        assert!(opts.disable_brs);
        assert_eq!(opts.force_transport, Some("socketcan".to_string()));
    }

    #[test]
    fn test_from_pairs_invalid_transport() {
        let result = TransportOptions::from_pairs([("force-transport", "invalid")]);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("invalid transport"));
    }

    #[test]
    fn test_from_pairs_invalid_timeout() {
        let result = TransportOptions::from_pairs([("timeout-ms", "not_a_number")]);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("invalid timeout"));
    }

    #[test]
    fn test_from_pairs_unknown_key() {
        let result = TransportOptions::from_pairs([("unknown-key", "value")]);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("unknown option"));
    }
}
