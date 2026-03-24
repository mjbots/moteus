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

//! Error types for the moteus client library.

use std::fmt;

/// Error type for moteus operations.
#[derive(Debug)]
pub enum Error {
    /// I/O error from transport layer
    Io(std::io::Error),
    /// Timeout waiting for response
    Timeout,
    /// No response received from controller
    NoResponse,
    /// Controller reported a fault
    Fault { mode: u8, code: i8 },
    /// Invalid response from controller
    InvalidResponse(String),
    /// Transport not connected
    NotConnected,
    /// Device not found
    DeviceNotFound(String),
    /// Protocol error
    Protocol(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "I/O error: {}", e),
            Error::Timeout => write!(f, "Timeout waiting for response"),
            Error::NoResponse => write!(f, "No response from controller"),
            Error::Fault { mode, code } => {
                write!(f, "Controller fault: mode={}, code={}", mode, code)
            }
            Error::InvalidResponse(msg) => write!(f, "Invalid response: {}", msg),
            Error::NotConnected => write!(f, "Transport not connected"),
            Error::DeviceNotFound(dev) => write!(f, "Device not found: {}", dev),
            Error::Protocol(msg) => write!(f, "Protocol error: {}", msg),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for Error {
    fn from(err: std::io::Error) -> Self {
        Error::Io(err)
    }
}

/// Result type for moteus operations.
pub type Result<T> = std::result::Result<T, Error>;
