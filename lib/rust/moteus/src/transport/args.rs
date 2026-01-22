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
//! use moteus::TransportArgs;
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
//! let opts = args.transport.into();
//! ```
//!
//! # Approach 2: With any CLI parser
//!
//! Use [`TRANSPORT_ARG_SPECS`] to generate arguments for any parser,
//! then pass results to [`TransportOptions::from_pairs()`]:
//!
//! ```
//! use moteus::transport::args::{TRANSPORT_ARG_SPECS, ArgType};
//!
//! // Print what arguments are available
//! for spec in TRANSPORT_ARG_SPECS {
//!     println!("--{}: {} ({:?})", spec.name, spec.help, spec.arg_type);
//! }
//! ```
//!
//! With clap's builder API:
//!
//! ```ignore
//! use clap::{Arg, ArgAction, Command};
//! use moteus::transport::args::{TRANSPORT_ARG_SPECS, ArgType};
//! use moteus::TransportOptions;
//!
//! let mut cmd = Command::new("myapp");
//! for spec in TRANSPORT_ARG_SPECS {
//!     cmd = cmd.arg(spec.to_clap_arg());
//! }
//! let matches = cmd.get_matches();
//! let opts = TransportOptions::from_arg_matches(&matches)?;
//! ```

use crate::transport::factory::TransportOptions;

/// Type of a command-line argument.
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
            arg = arg.value_parser(values.iter().copied().collect::<Vec<_>>());
        }

        arg
    }
}

/// Specifications for all transport-related command-line arguments.
///
/// Use this to programmatically generate CLI arguments for any parser.
pub static TRANSPORT_ARG_SPECS: &[ArgSpec] = &[
    ArgSpec {
        name: "fdcanusb",
        help: "Path to fdcanusb device (can be specified multiple times)",
        arg_type: ArgType::MultiString,
        default: None,
        possible_values: None,
    },
    ArgSpec {
        name: "can-chan",
        help: "SocketCAN interface (can be specified multiple times)",
        arg_type: ArgType::MultiString,
        default: None,
        possible_values: None,
    },
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
        possible_values: Some(&["fdcanusb", "socketcan"]),
    },
    ArgSpec {
        name: "timeout-ms",
        help: "Communication timeout in milliseconds",
        arg_type: ArgType::Integer,
        default: Some("100"),
        possible_values: None,
    },
];

impl TransportOptions {
    /// Create transport options from clap ArgMatches.
    ///
    /// This works with arguments created via [`ArgSpec::to_clap_arg()`] or
    /// any clap arguments using the standard transport argument names.
    #[cfg(feature = "clap")]
    pub fn from_arg_matches(matches: &clap::ArgMatches) -> std::result::Result<Self, String> {
        let mut opts = TransportOptions::new();

        if let Some(values) = matches.get_many::<String>("fdcanusb") {
            opts.fdcanusb_paths = values.cloned().collect();
        }

        if let Some(values) = matches.get_many::<String>("can-chan") {
            opts.socketcan_interfaces = values.cloned().collect();
        }

        if matches.get_flag("can-disable-brs") {
            opts.disable_brs = true;
        }

        if let Some(value) = matches.get_one::<String>("force-transport") {
            opts.force_transport = Some(value.clone());
        }

        if let Some(value) = matches.get_one::<String>("timeout-ms") {
            opts.timeout_ms = value
                .parse()
                .map_err(|_| format!("invalid timeout: {}", value))?;
        }

        Ok(opts)
    }
}

/// Add all transport arguments to a clap Command.
///
/// This is a convenience function for clap's builder API.
#[cfg(feature = "clap")]
pub fn add_transport_args(mut cmd: clap::Command) -> clap::Command {
    for spec in TRANSPORT_ARG_SPECS {
        cmd = cmd.arg(spec.to_clap_arg());
    }
    cmd
}

/// Command-line arguments for transport configuration.
///
/// This struct can be used with clap's derive API via `#[command(flatten)]`.
/// For the builder API, use [`TRANSPORT_ARG_SPECS`] and [`add_transport_args()`].
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "clap", derive(clap::Args))]
pub struct TransportArgs {
    /// Path to fdcanusb device (can be specified multiple times).
    #[cfg_attr(feature = "clap", arg(long = "fdcanusb", action = clap::ArgAction::Append))]
    pub fdcanusb: Vec<String>,

    /// SocketCAN interface (can be specified multiple times).
    #[cfg_attr(feature = "clap", arg(long = "can-chan", action = clap::ArgAction::Append))]
    pub can_chan: Vec<String>,

    /// Disable CAN-FD bit rate switching.
    #[cfg_attr(feature = "clap", arg(long = "can-disable-brs"))]
    pub can_disable_brs: bool,

    /// Force specific transport type (fdcanusb or socketcan).
    #[cfg_attr(feature = "clap", arg(long = "force-transport", value_parser = ["fdcanusb", "socketcan"]))]
    pub force_transport: Option<String>,

    /// Communication timeout in milliseconds.
    #[cfg_attr(feature = "clap", arg(long = "timeout-ms", default_value = "100"))]
    pub timeout_ms: u32,
}

impl TransportArgs {
    /// Create new transport args with default values.
    pub fn new() -> Self {
        Self {
            timeout_ms: crate::transport::factory::DEFAULT_TIMEOUT_MS,
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
            timeout_ms: self.timeout_ms,
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
            can_chan: vec!["can0".to_string(), "can1".to_string()],
            can_disable_brs: true,
            force_transport: Some("socketcan".to_string()),
            timeout_ms: 200,
        };

        let opts: TransportOptions = args.into();
        assert_eq!(opts.fdcanusb_paths, vec!["/dev/ttyACM0"]);
        assert_eq!(opts.socketcan_interfaces, vec!["can0", "can1"]);
        assert!(opts.disable_brs);
        assert_eq!(opts.force_transport, Some("socketcan".to_string()));
        assert_eq!(opts.timeout_ms, 200);
    }

    #[test]
    fn test_arg_specs_complete() {
        // Verify all expected args are present
        let names: Vec<_> = TRANSPORT_ARG_SPECS.iter().map(|s| s.name).collect();
        assert!(names.contains(&"fdcanusb"));
        assert!(names.contains(&"can-chan"));
        assert!(names.contains(&"can-disable-brs"));
        assert!(names.contains(&"force-transport"));
        assert!(names.contains(&"timeout-ms"));
    }

    #[test]
    fn test_arg_specs_types() {
        for spec in TRANSPORT_ARG_SPECS {
            match spec.name {
                "fdcanusb" | "can-chan" => {
                    assert_eq!(spec.arg_type, ArgType::MultiString);
                }
                "can-disable-brs" => {
                    assert_eq!(spec.arg_type, ArgType::Bool);
                }
                "force-transport" => {
                    assert_eq!(spec.arg_type, ArgType::String);
                    assert!(spec.possible_values.is_some());
                }
                "timeout-ms" => {
                    assert_eq!(spec.arg_type, ArgType::Integer);
                    assert_eq!(spec.default, Some("100"));
                }
                _ => panic!("unexpected arg: {}", spec.name),
            }
        }
    }
}
