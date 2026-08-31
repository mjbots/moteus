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

//! Command-line argument support for transport options.
//!
//! This module provides two approaches for CLI integration:
//!
//! # Approach 1: With clap (recommended)
//!
//! When the `clap` feature is enabled, use [`TransportArgs`] with derive:
//!
//! ```ignore
//! use clap::Parser;
//! use moteus::transport::args::TransportArgs;
//!
//! #[derive(Parser)]
//! struct Args {
//!     #[arg(long, default_value = "1")]
//!     id: u8,
//!
//!     #[command(flatten)]
//!     transport: TransportArgs,
//! }
//!
//! let args = Args::parse();
//! let opts: moteus::TransportOptions = args.transport.into();
//! ```
//!
//! # Approach 2: With any CLI parser
//!
//! Use [`transport_arg_specs()`] to generate arguments for any parser,
//! then pass results to [`TransportOptions::from_pairs()`]:
//!
//! ```
//! use moteus::transport::args::{transport_arg_specs, ArgType};
//!
//! // Print what arguments are available (includes registered factory args)
//! for spec in transport_arg_specs() {
//!     println!("--{}: {} ({:?})", spec.name, spec.help, spec.arg_type);
//! }
//! ```
//!
//! With clap's builder API:
//!
//! ```ignore
//! use clap::{Arg, ArgAction, Command};
//! use moteus::transport::args::{transport_arg_specs, ArgType};
//! use moteus::TransportOptions;
//!
//! fn main() -> Result<(), String> {
//!     let mut cmd = Command::new("myapp");
//!     for spec in transport_arg_specs() {
//!         cmd = cmd.arg(spec.to_clap_arg());
//!     }
//!     let matches = cmd.get_matches();
//!     let opts = TransportOptions::from_arg_matches(&matches)?;
//!     Ok(())
//! }
//! ```

use crate::transport::factory::TransportOptions;
use std::time::Duration;

/// Type of a command-line argument.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArgType {
    /// A string value (single occurrence)
    String,
    /// A boolean flag (presence = true)
    Bool,
    /// An unsigned integer value
    Integer,
    /// A string that can be specified multiple times
    MultiString,
}

/// Specification for a single command-line argument.
#[non_exhaustive]
#[derive(Debug, Clone)]
pub struct ArgSpec {
    /// Argument name (without dashes), e.g., "fdcanusb"
    pub name: &'static str,
    /// Short help text
    pub help: &'static str,
    /// Argument type
    pub arg_type: ArgType,
    /// Default value as a string (if any)
    pub default: Option<&'static str>,
    /// Valid values for enum-like arguments
    pub possible_values: Option<&'static [&'static str]>,
}

impl ArgSpec {
    /// Create a new argument specification.
    ///
    /// This is the only way for external transport factories to
    /// construct an `ArgSpec`, since the struct is `#[non_exhaustive]`.
    pub const fn new(name: &'static str, help: &'static str, arg_type: ArgType) -> Self {
        Self {
            name,
            help,
            arg_type,
            default: None,
            possible_values: None,
        }
    }

    /// Set the default value (builder pattern).
    #[must_use]
    pub const fn with_default(mut self, default: &'static str) -> Self {
        self.default = Some(default);
        self
    }

    /// Set the valid values for enum-like arguments (builder pattern).
    #[must_use]
    pub const fn with_possible_values(mut self, values: &'static [&'static str]) -> Self {
        self.possible_values = Some(values);
        self
    }

    /// Create a clap Arg from this specification.
    #[cfg(feature = "clap")]
    pub fn to_clap_arg(&self) -> clap::Arg {
        use clap::{Arg, ArgAction};

        let mut arg = Arg::new(self.name).long(self.name).help(self.help);

        match self.arg_type {
            ArgType::Bool => {
                arg = arg.action(ArgAction::SetTrue);
            }
            ArgType::MultiString => {
                arg = arg.action(ArgAction::Append);
            }
            ArgType::String | ArgType::Integer => {
                arg = arg.action(ArgAction::Set);
            }
        }

        if let Some(default) = self.default {
            arg = arg.default_value(default);
        }

        if let Some(values) = self.possible_values {
            arg = arg.value_parser(values.to_vec());
        }

        arg
    }
}

/// Common transport arguments not owned by any specific factory.
pub static COMMON_ARG_SPECS: &[ArgSpec] = &[
    ArgSpec {
        name: "can-disable-brs",
        help: "Disable CAN-FD bit rate switching",
        arg_type: ArgType::Bool,
        default: None,
        possible_values: None,
    },
    ArgSpec {
        name: "force-transport",
        help: "Force specific transport type",
        arg_type: ArgType::String,
        default: None,
        possible_values: None,
    },
    ArgSpec {
        name: "timeout-ms",
        help: "Communication timeout in milliseconds",
        arg_type: ArgType::Integer,
        default: Some("100"),
        possible_values: None,
    },
];

/// Get all transport-related argument specifications.
///
/// Returns the common args plus args from every registered transport
/// factory -- built-in and external, blocking and (with the `tokio`
/// feature) async.  This is dynamic: register external factories
/// (e.g. via [`register()`](super::factory::register)) before calling
/// this so their arg specs are included.  A spec name declared by
/// several factories (e.g. by a factory's blocking and async
/// variants) appears once, first declaration wins.
pub fn transport_arg_specs() -> Vec<ArgSpec> {
    let mut specs: Vec<ArgSpec> = COMMON_ARG_SPECS.to_vec();
    specs.extend(super::factory::registered_arg_specs());
    #[cfg(feature = "tokio")]
    specs.extend(super::async_factory::registered_arg_specs());

    let mut seen = std::collections::HashSet::new();
    specs.retain(|spec| seen.insert(spec.name));
    specs
}

impl TransportOptions {
    /// Create transport options from clap ArgMatches.
    ///
    /// This works with arguments created via [`ArgSpec::to_clap_arg()`] or
    /// any clap arguments using the standard transport argument names.
    ///
    /// Exactly the arguments declared by [`transport_arg_specs()`] are
    /// extracted: their values are converted to `(name, value)` string
    /// pairs and handed to [`TransportOptions::from_pairs()`], which
    /// owns the one mapping of argument names onto typed fields and
    /// routes everything else (e.g. registered factory args) into the
    /// `extra` field.  Declared arguments the command does not define
    /// -- or defines itself with a different value type -- are simply
    /// skipped, so this can be used with commands that carry any
    /// subset of the transport args.
    #[cfg(feature = "clap")]
    pub fn from_arg_matches(matches: &clap::ArgMatches) -> std::result::Result<Self, String> {
        // Absent ids and host-defined args with non-string value
        // types both surface as Err from the try_ accessors; treat
        // them as "not provided".
        let mut pairs: Vec<(&'static str, String)> = Vec::new();
        for spec in transport_arg_specs() {
            match spec.arg_type {
                ArgType::MultiString => {
                    if let Ok(Some(values)) = matches.try_get_many::<String>(spec.name) {
                        pairs.extend(values.map(|value| (spec.name, value.clone())));
                    }
                }
                ArgType::Bool => {
                    if matches!(matches.try_get_one::<bool>(spec.name), Ok(Some(true))) {
                        pairs.push((spec.name, "true".to_string()));
                    }
                }
                ArgType::String | ArgType::Integer => {
                    if let Ok(Some(value)) = matches.try_get_one::<String>(spec.name) {
                        pairs.push((spec.name, value.clone()));
                    }
                }
            }
        }

        Self::from_pairs(pairs.iter().map(|(name, value)| (*name, value.as_str())))
    }
}

/// Add all transport arguments to a clap Command.
///
/// This is a convenience function for clap's builder API. It includes
/// common args plus args from all registered transport factories.
#[cfg(feature = "clap")]
pub fn add_transport_args(mut cmd: clap::Command) -> clap::Command {
    for spec in transport_arg_specs() {
        cmd = cmd.arg(spec.to_clap_arg());
    }
    cmd
}

/// Parse `T` plus every registered transport argument from the
/// process command line.
///
/// This is the derive-API companion to [`add_transport_args`]: `T`'s
/// clap command is augmented with [`transport_arg_specs()`] (skipping
/// any argument `T` already defines itself), the combined command
/// line is parsed, and both the typed `T` and the resulting
/// [`TransportOptions`] -- registered factory args included in
/// `extra` -- are returned.  `T` should not also flatten
/// [`TransportArgs`]; the transport arguments are provided by the
/// augmentation.
///
/// Register any external transport factories (e.g. a pi3hat) *before*
/// calling this, so their arguments participate in parsing and appear
/// in `--help`.
///
/// Exits the process on a command-line parse error, like
/// [`clap::Parser::parse`].
///
/// ```ignore
/// use clap::Parser;
/// use moteus::transport::args::parse_with_transport_args;
///
/// #[derive(Parser)]
/// struct Args {
///     #[arg(long, default_value = "1")]
///     id: u8,
/// }
///
/// moteus_pi3hat::transport::register();
/// let (args, transport) = parse_with_transport_args::<Args>()?;
/// ```
#[cfg(feature = "clap")]
pub fn parse_with_transport_args<T: clap::Parser>(
) -> std::result::Result<(T, TransportOptions), String> {
    let mut cmd = T::command();
    let existing: std::collections::HashSet<String> = cmd
        .get_arguments()
        .map(|arg| arg.get_id().to_string())
        .collect();
    for spec in transport_arg_specs() {
        if !existing.contains(spec.name) {
            cmd = cmd.arg(spec.to_clap_arg());
        }
    }

    let matches = cmd.get_matches();
    let args = T::from_arg_matches(&matches).unwrap_or_else(|err| err.exit());
    let options = TransportOptions::from_arg_matches(&matches)?;
    Ok((args, options))
}

/// Command-line arguments for transport configuration.
///
/// This struct can be used with clap's derive API via `#[command(flatten)]`.
/// For the builder API, use [`transport_arg_specs()`].
///
/// Note: This covers built-in transport args only. External factory args
/// should be handled via `TransportOptions::from_arg_matches()` or
/// [`TransportOptions::from_pairs()`] with the builder API.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "clap", derive(clap::Args))]
pub struct TransportArgs {
    /// Path to fdcanusb device (can be specified multiple times).
    #[cfg_attr(feature = "clap", arg(long = "fdcanusb", action = clap::ArgAction::Append))]
    pub fdcanusb: Vec<String>,

    /// Serial baud rate (only matters for UART connections).
    #[cfg_attr(feature = "clap", arg(long = "fdcanusb-baudrate"))]
    pub fdcanusb_baudrate: Option<u32>,

    /// SocketCAN interface (can be specified multiple times).
    #[cfg_attr(feature = "clap", arg(long = "can-chan", action = clap::ArgAction::Append))]
    pub can_chan: Vec<String>,

    /// Disable CAN-FD bit rate switching.
    #[cfg_attr(feature = "clap", arg(long = "can-disable-brs"))]
    pub can_disable_brs: bool,

    /// Force specific transport type.
    #[cfg_attr(feature = "clap", arg(long = "force-transport"))]
    pub force_transport: Option<String>,

    /// Communication timeout in milliseconds.
    #[cfg_attr(feature = "clap", arg(long = "timeout-ms", default_value = "100"))]
    pub timeout_ms: u32,
}

impl TransportArgs {
    /// Create new transport args with default values.
    pub fn new() -> Self {
        Self {
            timeout_ms: 100,
            ..Default::default()
        }
    }

    /// Convert these args into TransportOptions.
    pub fn into_options(self) -> TransportOptions {
        TransportOptions {
            fdcanusb_paths: self.fdcanusb,
            socketcan_interfaces: self.can_chan,
            disable_brs: self.can_disable_brs,
            force_transport: self.force_transport,
            fdcanusb_baudrate: self.fdcanusb_baudrate,
            timeout: Duration::from_millis(self.timeout_ms as u64),
            extra: Default::default(),
        }
    }
}

impl From<TransportArgs> for TransportOptions {
    fn from(args: TransportArgs) -> Self {
        args.into_options()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transport_args_default() {
        let args = TransportArgs::new();
        assert_eq!(args.timeout_ms, 100);
        assert!(args.fdcanusb.is_empty());
        assert!(args.can_chan.is_empty());
        assert!(!args.can_disable_brs);
        assert!(args.force_transport.is_none());
    }

    #[test]
    fn test_transport_args_to_options() {
        let args = TransportArgs {
            fdcanusb: vec!["/dev/ttyACM0".to_string()],
            fdcanusb_baudrate: Some(3_000_000),
            can_chan: vec!["can0".to_string(), "can1".to_string()],
            can_disable_brs: true,
            force_transport: Some("socketcan".to_string()),
            timeout_ms: 200,
        };

        let opts: TransportOptions = args.into();
        assert_eq!(opts.fdcanusb_paths, vec!["/dev/ttyACM0"]);
        assert_eq!(opts.fdcanusb_baudrate, Some(3_000_000));
        assert_eq!(opts.socketcan_interfaces, vec!["can0", "can1"]);
        assert!(opts.disable_brs);
        assert_eq!(opts.force_transport, Some("socketcan".to_string()));
        assert_eq!(opts.timeout, Duration::from_millis(200));
    }

    #[test]
    fn test_common_arg_specs_complete() {
        let names: Vec<_> = COMMON_ARG_SPECS.iter().map(|s| s.name).collect();
        assert!(names.contains(&"can-disable-brs"));
        assert!(names.contains(&"force-transport"));
        assert!(names.contains(&"timeout-ms"));
    }

    #[test]
    fn test_transport_arg_specs_includes_factory_args() {
        let specs = transport_arg_specs();
        let names: Vec<_> = specs.iter().map(|s| s.name).collect();
        // Common args
        assert!(names.contains(&"can-disable-brs"));
        assert!(names.contains(&"force-transport"));
        assert!(names.contains(&"timeout-ms"));
        // Factory-provided args
        #[cfg(feature = "serialport")]
        assert!(names.contains(&"fdcanusb"));
        #[cfg(target_os = "linux")]
        assert!(names.contains(&"can-chan"));
    }

    #[test]
    fn test_arg_spec_new() {
        let spec = ArgSpec::new("pi3hat-cpu", "CPU to pin to", ArgType::Integer)
            .with_default("0")
            .with_possible_values(&["0", "1", "2", "3"]);
        assert_eq!(spec.name, "pi3hat-cpu");
        assert_eq!(spec.help, "CPU to pin to");
        assert_eq!(spec.arg_type, ArgType::Integer);
        assert_eq!(spec.default, Some("0"));
        assert_eq!(spec.possible_values, Some(&["0", "1", "2", "3"][..]));
    }

    #[test]
    fn test_arg_specs_types() {
        let specs = transport_arg_specs();
        for spec in &specs {
            match spec.name {
                "fdcanusb" | "can-chan" => {
                    assert_eq!(spec.arg_type, ArgType::MultiString);
                }
                "can-disable-brs" => {
                    assert_eq!(spec.arg_type, ArgType::Bool);
                }
                "force-transport" => {
                    assert_eq!(spec.arg_type, ArgType::String);
                }
                "timeout-ms" => {
                    assert_eq!(spec.arg_type, ArgType::Integer);
                    assert_eq!(spec.default, Some("100"));
                }
                _ => {} // External factory args - don't panic
            }
        }
    }

    /// The clap adapter extracts exactly the declared specs and
    /// delegates the name-to-field mapping to `from_pairs()`.
    #[test]
    #[cfg(all(feature = "clap", feature = "serialport"))]
    fn test_from_arg_matches_round_trip() {
        let cmd = add_transport_args(clap::Command::new("test"));
        let matches = cmd.get_matches_from([
            "test",
            "--fdcanusb",
            "/dev/ttyACM0",
            "--fdcanusb",
            "/dev/ttyACM1",
            "--can-disable-brs",
            "--force-transport",
            "fdcanusb",
            "--timeout-ms",
            "250",
        ]);

        let opts = TransportOptions::from_arg_matches(&matches).unwrap();
        assert_eq!(opts.fdcanusb_paths, vec!["/dev/ttyACM0", "/dev/ttyACM1"]);
        assert!(opts.disable_brs);
        assert_eq!(opts.force_transport, Some("fdcanusb".to_string()));
        assert_eq!(opts.timeout, Duration::from_millis(250));
    }
}
