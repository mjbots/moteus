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
//!
//! External crates can register additional factories via [`register()`]:
//!
//! ```rust,ignore
//! moteus::transport::factory::register(Box::new(MyFactory));
//! ```

use std::collections::HashMap;
use std::sync::{Mutex, OnceLock};
use std::time::Duration;

use crate::error::Result;
use crate::transport::args::ArgSpec;
use crate::transport::device::TransportDevice;

/// Default communication timeout.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

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
    /// Communication timeout.
    pub timeout: Duration,
    /// Extra transport-specific options for external factories.
    ///
    /// Keys are argument names, values are lists of values (to support
    /// multi-value arguments). Built-in factories ignore this field;
    /// external factories read from it.
    pub extra: HashMap<String, Vec<String>>,
}

impl TransportOptions {
    /// Create new transport options with default values.
    pub fn new() -> Self {
        Self {
            timeout: DEFAULT_TIMEOUT,
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
    pub fn timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Set an extra transport-specific option.
    pub fn extra(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.extra
            .entry(key.into())
            .or_default()
            .push(value.into());
        self
    }

    /// Parse transport options from key-value pairs.
    ///
    /// This method allows creating transport options from any CLI parser by
    /// converting parsed arguments to key-value pairs. Keys match the standard
    /// command-line argument names (without leading dashes).
    ///
    /// Unknown keys are stored in the `extra` field for external factories.
    ///
    /// # Supported Keys
    ///
    /// - `fdcanusb`: Path to fdcanusb device (can appear multiple times)
    /// - `can-chan`: SocketCAN interface name (can appear multiple times)
    /// - `can-disable-brs`: "true" to disable CAN-FD bit rate switching
    /// - `force-transport`: Transport type name (validated against registered factories)
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
    /// assert_eq!(opts.timeout, std::time::Duration::from_millis(200));
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
                    opts.force_transport = Some(value.to_string());
                }
                "timeout-ms" => {
                    let ms: u32 = value
                        .parse()
                        .map_err(|_| format!("invalid timeout: {}", value))?;
                    opts.timeout = Duration::from_millis(ms as u64);
                }
                _ => {
                    opts.extra
                        .entry(key.to_string())
                        .or_default()
                        .push(value.to_string());
                }
            }
        }
        Ok(opts)
    }
}

/// A factory for creating transport devices.
///
/// Factories are tried in priority order (lower numbers first).
/// External crates implement this trait and call [`register()`] to
/// add themselves to the auto-detection flow.
pub trait TransportFactory: Send + Sync {
    /// The priority of this factory (lower = tried first).
    fn priority(&self) -> u32;

    /// The name of this transport type.
    fn name(&self) -> &'static str;

    /// Command-line argument specifications for this factory.
    ///
    /// Override this to declare factory-specific CLI arguments. These are
    /// included by [`transport_arg_specs()`](super::args::transport_arg_specs)
    /// and [`add_transport_args()`](super::args::add_transport_args).
    fn arg_specs(&self) -> Vec<ArgSpec> {
        Vec::new()
    }

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
            match Fdcanusb::with_options(&info.path, options.timeout, options.disable_brs) {
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
#[cfg(target_os = "linux")]
#[derive(Debug, Default)]
pub struct SocketCanFactory;

#[cfg(target_os = "linux")]
impl SocketCanFactory {
    /// Create a new socketcan factory.
    pub fn new() -> Self {
        Self
    }
}

#[cfg(target_os = "linux")]
impl TransportFactory for SocketCanFactory {
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
            match SocketCan::with_options(interface, options.timeout, options.disable_brs) {
                Ok(mut transport) => {
                    transport.info.id = idx;
                    devices.push(Box::new(transport));
                }
                Err(_) => continue, // Skip interfaces that fail to open
            }
        }

        Ok(devices)
    }
}

// -- Global transport factory registry --

static REGISTRY: OnceLock<Mutex<Vec<Box<dyn TransportFactory>>>> = OnceLock::new();

fn get_registry() -> &'static Mutex<Vec<Box<dyn TransportFactory>>> {
    REGISTRY.get_or_init(|| {
        let mut factories: Vec<Box<dyn TransportFactory>> = vec![
            Box::new(FdcanusbFactory),
        ];
        #[cfg(target_os = "linux")]
        factories.push(Box::new(SocketCanFactory));
        Mutex::new(factories)
    })
}

/// Register an external transport factory.
///
/// The factory will be included in all subsequent calls to
/// [`get_factories()`], [`create_transports()`], and singleton creation.
///
/// # Example
///
/// ```rust,ignore
/// use moteus::transport::factory::{register, TransportFactory, TransportOptions};
///
/// struct Pi3HatFactory;
/// impl TransportFactory for Pi3HatFactory {
///     fn priority(&self) -> u32 { 5 }
///     fn name(&self) -> &'static str { "pi3hat" }
///     fn create(&self, opts: &TransportOptions) -> moteus::Result<Vec<Box<dyn TransportDevice>>> {
///         // ...
///     }
/// }
///
/// register(Box::new(Pi3HatFactory));
/// ```
pub fn register(factory: Box<dyn TransportFactory>) {
    get_registry().lock().unwrap().push(factory);
}

/// Get the built-in transport factories.
///
/// Returns fresh instances of the built-in factories. For the full set
/// of registered factories (including external ones), use
/// [`create_transports()`] which reads the global registry.
pub fn get_factories() -> Vec<Box<dyn TransportFactory>> {
    vec![
        Box::new(FdcanusbFactory::new()),
        Box::new(SocketCanFactory::new()),
    ]
}

/// Create transport devices using all registered factories.
///
/// This is the main entry point for creating transports with auto-discovery.
/// It tries each factory in priority order and returns all successfully created
/// devices. Duplicate socketcan interfaces backed by fdcanusb devices are
/// filtered out.
pub fn create_transports(
    options: &TransportOptions,
) -> Result<Vec<Box<dyn TransportDevice>>> {
    use std::collections::HashSet;

    let registry = get_registry().lock().unwrap();

    // Collect priorities and filter/sort
    let mut indices: Vec<usize> = (0..registry.len()).collect();
    indices.sort_by_key(|&i| registry[i].priority());

    // Filter by forced transport if specified
    if let Some(ref force) = options.force_transport {
        indices.retain(|&i| registry[i].name() == force.as_str());
    }

    let mut all_devices = Vec::new();
    let mut fdcanusb_serials: HashSet<String> = HashSet::new();

    for &idx in &indices {
        match registry[idx].create(options) {
            Ok(devices) => {
                // Track fdcanusb serial numbers for deduplication
                if registry[idx].name() == "fdcanusb" {
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

    // Drop the lock before doing more work
    drop(registry);

    // Deduplicate: remove socketcan interfaces that are backed by fdcanusb
    // devices we're already using via CDC serial
    #[cfg(target_os = "linux")]
    {
        use crate::transport::discovery::detect_socketcan_interfaces;

        let socketcan_infos = detect_socketcan_interfaces();
        let mut filtered_devices = Vec::new();

        for device in all_devices {
            let should_skip = socketcan_infos.iter().any(|info| {
                (device
                    .info()
                    .serial_number
                    .as_ref() == Some(&info.interface))
                    && info.fdcanusb_serial.as_ref().is_some_and(|serial| {
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
            .timeout(Duration::from_millis(200));

        assert_eq!(opts.fdcanusb_paths, vec!["/dev/ttyACM0", "/dev/ttyACM1"]);
        assert_eq!(opts.socketcan_interfaces, vec!["can0"]);
        assert!(opts.disable_brs);
        assert_eq!(opts.timeout, Duration::from_millis(200));
    }

    #[test]
    fn test_transport_options_extra() {
        let opts = TransportOptions::new()
            .extra("pi3hat-cfg", "1=1,2")
            .extra("pi3hat-cfg", "3=3,4");

        assert_eq!(
            opts.extra.get("pi3hat-cfg").unwrap(),
            &vec!["1=1,2".to_string(), "3=3,4".to_string()]
        );
    }

    #[test]
    fn test_factory_priorities() {
        let fdcanusb = FdcanusbFactory::new();
        let socketcan = SocketCanFactory::new();

        assert!(fdcanusb.priority() < socketcan.priority());
    }

    #[test]
    fn test_factory_arg_specs() {
        let fdcanusb = FdcanusbFactory::new();
        let specs = fdcanusb.arg_specs();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].name, "fdcanusb");

        let socketcan = SocketCanFactory::new();
        let specs = socketcan.arg_specs();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].name, "can-chan");
    }

    #[test]
    fn test_get_factories() {
        let factories = get_factories();
        assert!(!factories.is_empty());

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
        assert_eq!(opts.timeout, Duration::from_millis(200));
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
    fn test_from_pairs_unknown_key_goes_to_extra() {
        let opts = TransportOptions::from_pairs([
            ("pi3hat-cfg", "1=1,2"),
            ("custom-opt", "value"),
        ])
        .unwrap();
        assert_eq!(
            opts.extra.get("pi3hat-cfg").unwrap(),
            &vec!["1=1,2".to_string()]
        );
        assert_eq!(
            opts.extra.get("custom-opt").unwrap(),
            &vec!["value".to_string()]
        );
    }

    #[test]
    fn test_from_pairs_force_transport_any_value() {
        // force-transport now accepts any value (validated dynamically at creation time)
        let opts =
            TransportOptions::from_pairs([("force-transport", "pi3hat")]).unwrap();
        assert_eq!(opts.force_transport, Some("pi3hat".to_string()));
    }

    #[test]
    fn test_from_pairs_invalid_timeout() {
        let result = TransportOptions::from_pairs([("timeout-ms", "not_a_number")]);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("invalid timeout"));
    }
}
