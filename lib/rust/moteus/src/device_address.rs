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

//! Device addressing for moteus controllers.
//!
//! Devices can be addressed by CAN ID, UUID, or both. An optional
//! transport device index specifies which transport device to route through.

use core::fmt;

/// Address for a moteus device.
///
/// A device address can include a CAN ID, a UUID (or UUID prefix), and a
/// transport device index. All fields are optional, and both `can_id` and
/// `uuid` can be set simultaneously so that clients can see both identifiers.
///
/// The `transport_device` field specifies which transport device to route
/// commands through, bypassing automatic device discovery.
///
/// # Examples
///
/// ```rust
/// use moteus::DeviceAddress;
///
/// // Address by CAN ID (most common)
/// let addr = DeviceAddress::can_id(1);
/// assert_eq!(addr.as_can_id(), Some(1));
///
/// // Using From trait
/// let addr: DeviceAddress = 1.into();
///
/// // Address by UUID
/// let uuid_bytes = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
///                   0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10];
/// let addr = DeviceAddress::uuid(uuid_bytes);
///
/// // UUID prefix (for matching)
/// let addr = DeviceAddress::uuid([0x01, 0x02, 0x03, 0x04]);
///
/// // With transport device routing
/// let addr = DeviceAddress::can_id(1).with_transport_device(0);
/// ```
#[non_exhaustive]
#[derive(Clone, PartialEq, Eq, Hash)]
pub struct DeviceAddress {
    /// CAN bus ID (1-127), if known.
    pub can_id: Option<u8>,
    /// UUID or UUID prefix, if known.
    pub uuid: Option<Vec<u8>>,
    /// Index of the transport device to route through, if known.
    pub transport_device: Option<usize>,
}

impl DeviceAddress {
    /// Create an address from a CAN ID.
    ///
    /// # Arguments
    /// * `id` - CAN bus ID (typically 1-127)
    pub fn can_id(id: u8) -> Self {
        DeviceAddress {
            can_id: Some(id),
            uuid: None,
            transport_device: None,
        }
    }

    /// Create an address from a UUID or UUID prefix.
    ///
    /// # Arguments
    /// * `uuid` - Full 16-byte UUID or a prefix for matching
    pub fn uuid(uuid: impl Into<Vec<u8>>) -> Self {
        DeviceAddress {
            can_id: None,
            uuid: Some(uuid.into()),
            transport_device: None,
        }
    }

    /// Set the transport device index (builder pattern).
    pub fn with_transport_device(mut self, idx: usize) -> Self {
        self.transport_device = Some(idx);
        self
    }

    /// Returns the CAN ID if set.
    pub fn as_can_id(&self) -> Option<u8> {
        self.can_id
    }

    /// Returns the UUID bytes if set.
    pub fn as_uuid(&self) -> Option<&[u8]> {
        self.uuid.as_deref()
    }

    /// Returns true if this address has a CAN ID.
    pub fn is_can_id(&self) -> bool {
        self.can_id.is_some()
    }

    /// Returns true if this address has a UUID.
    pub fn is_uuid(&self) -> bool {
        self.uuid.is_some()
    }

    /// Check if this address matches a device with the given CAN ID and UUID.
    ///
    /// All set fields must match. A CAN ID address matches by exact ID,
    /// a UUID address matches by prefix. If both are set, both must match.
    pub fn matches(&self, can_id: u8, uuid: Option<&[u8]>) -> bool {
        if self.can_id.is_none() && self.uuid.is_none() {
            return false;
        }
        let can_match = self.can_id.is_none_or(|id| id == can_id);
        let uuid_match = match (&self.uuid, uuid) {
            (Some(prefix), Some(device_uuid)) => device_uuid.starts_with(prefix),
            (Some(_), None) => false,
            (None, _) => true,
        };
        can_match && uuid_match
    }
}

impl fmt::Debug for DeviceAddress {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "DeviceAddress(can_id={:?}", self.can_id)?;
        if let Some(uuid) = &self.uuid {
            write!(f, ", uuid=")?;
            for (i, byte) in uuid.iter().enumerate() {
                if i > 0 {
                    write!(f, ":")?;
                }
                write!(f, "{:02x}", byte)?;
            }
        }
        if let Some(td) = self.transport_device {
            write!(f, ", td={}", td)?;
        }
        write!(f, ")")
    }
}

impl fmt::Display for DeviceAddress {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match (&self.can_id, &self.uuid) {
            (Some(id), None) => write!(f, "{}", id),
            (None, Some(uuid)) => {
                for (i, byte) in uuid.iter().enumerate() {
                    if i > 0 {
                        write!(f, ":")?;
                    }
                    write!(f, "{:02x}", byte)?;
                }
                Ok(())
            }
            (Some(id), Some(uuid)) => {
                write!(f, "{}(", id)?;
                for (i, byte) in uuid.iter().enumerate() {
                    if i > 0 {
                        write!(f, ":")?;
                    }
                    write!(f, "{:02x}", byte)?;
                }
                write!(f, ")")
            }
            (None, None) => write!(f, "<empty>"),
        }
    }
}

// Allow converting from integer types for convenience
impl From<u8> for DeviceAddress {
    fn from(id: u8) -> Self {
        DeviceAddress::can_id(id)
    }
}

impl From<i32> for DeviceAddress {
    fn from(id: i32) -> Self {
        DeviceAddress::can_id(id.clamp(0, 127) as u8)
    }
}

// Allow converting from byte slices/arrays for UUID
impl From<&[u8]> for DeviceAddress {
    fn from(uuid: &[u8]) -> Self {
        DeviceAddress::uuid(uuid)
    }
}

impl<const N: usize> From<[u8; N]> for DeviceAddress {
    fn from(uuid: [u8; N]) -> Self {
        DeviceAddress::uuid(uuid)
    }
}

impl From<Vec<u8>> for DeviceAddress {
    fn from(uuid: Vec<u8>) -> Self {
        DeviceAddress::uuid(uuid)
    }
}

/// Parse a device address from a string.
///
/// Supports:
/// - Integer CAN IDs: "1", "15", "127"
/// - Hex UUID or prefix: "01:02:03:04" or "01020304"
impl core::str::FromStr for DeviceAddress {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // Try parsing as integer first
        if let Ok(id) = s.parse::<u8>() {
            return Ok(DeviceAddress::can_id(id));
        }

        // Try parsing as hex UUID (with or without colons)
        let hex_str: String = s.chars().filter(|c| *c != ':' && *c != '-').collect();
        if hex_str.len() % 2 != 0 {
            return Err(format!("invalid address: {}", s));
        }

        let bytes: Result<Vec<u8>, _> = (0..hex_str.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex_str[i..i + 2], 16))
            .collect();

        match bytes {
            Ok(uuid) if !uuid.is_empty() => Ok(DeviceAddress::uuid(uuid)),
            _ => Err(format!("invalid address: {}", s)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_can_id_address() {
        let addr = DeviceAddress::can_id(1);
        assert_eq!(addr.as_can_id(), Some(1));
        assert_eq!(addr.as_uuid(), None);
        assert!(addr.is_can_id());
        assert!(!addr.is_uuid());
        assert!(addr.transport_device.is_none());
    }

    #[test]
    fn test_uuid_address() {
        let uuid = [0x01, 0x02, 0x03, 0x04];
        let addr = DeviceAddress::uuid(uuid);
        assert_eq!(addr.as_can_id(), None);
        assert_eq!(addr.as_uuid(), Some(&[0x01, 0x02, 0x03, 0x04][..]));
        assert!(!addr.is_can_id());
        assert!(addr.is_uuid());
    }

    #[test]
    fn test_with_transport_device() {
        let addr = DeviceAddress::can_id(1).with_transport_device(2);
        assert_eq!(addr.can_id, Some(1));
        assert_eq!(addr.transport_device, Some(2));
    }

    #[test]
    fn test_from_u8() {
        let addr: DeviceAddress = 5u8.into();
        assert_eq!(addr, DeviceAddress::can_id(5));
    }

    #[test]
    fn test_from_i32() {
        let addr: DeviceAddress = 5i32.into();
        assert_eq!(addr, DeviceAddress::can_id(5));
    }

    #[test]
    fn test_from_array() {
        let addr: DeviceAddress = [0x01, 0x02, 0x03, 0x04].into();
        assert_eq!(addr.as_uuid(), Some(&[0x01, 0x02, 0x03, 0x04][..]));
    }

    #[test]
    fn test_matches_can_id() {
        let addr = DeviceAddress::can_id(5);
        assert!(addr.matches(5, None));
        assert!(!addr.matches(6, None));
        assert!(addr.matches(5, Some(&[0x01, 0x02])));
    }

    #[test]
    fn test_matches_uuid() {
        let addr = DeviceAddress::uuid([0x01, 0x02]);
        assert!(!addr.matches(5, None));
        assert!(addr.matches(5, Some(&[0x01, 0x02, 0x03, 0x04])));
        assert!(!addr.matches(5, Some(&[0x02, 0x03, 0x04, 0x05])));
    }

    #[test]
    fn test_matches_both() {
        let addr = DeviceAddress {
            can_id: Some(5),
            uuid: Some(vec![0x01, 0x02]),
            transport_device: None,
        };
        // Both must match
        assert!(addr.matches(5, Some(&[0x01, 0x02, 0x03])));
        assert!(!addr.matches(6, Some(&[0x01, 0x02, 0x03])));
        assert!(!addr.matches(5, Some(&[0x02, 0x03])));
        assert!(!addr.matches(5, None));
    }

    #[test]
    fn test_parse_can_id() {
        let addr: DeviceAddress = "5".parse().unwrap();
        assert_eq!(addr, DeviceAddress::can_id(5));

        let addr: DeviceAddress = "127".parse().unwrap();
        assert_eq!(addr, DeviceAddress::can_id(127));
    }

    #[test]
    fn test_parse_uuid_with_colons() {
        let addr: DeviceAddress = "01:02:03:04".parse().unwrap();
        assert_eq!(addr.as_uuid(), Some(&[0x01, 0x02, 0x03, 0x04][..]));
    }

    #[test]
    fn test_parse_uuid_without_colons() {
        let addr: DeviceAddress = "01020304".parse().unwrap();
        assert_eq!(addr.as_uuid(), Some(&[0x01, 0x02, 0x03, 0x04][..]));
    }

    #[test]
    fn test_display() {
        assert_eq!(format!("{}", DeviceAddress::can_id(5)), "5");
        assert_eq!(
            format!("{}", DeviceAddress::uuid([0x01, 0x02, 0x03])),
            "01:02:03"
        );
    }

    #[test]
    fn test_display_both() {
        let addr = DeviceAddress {
            can_id: Some(5),
            uuid: Some(vec![0x01, 0x02]),
            transport_device: None,
        };
        assert_eq!(format!("{}", addr), "5(01:02)");
    }

    #[test]
    fn test_debug() {
        assert_eq!(
            format!("{:?}", DeviceAddress::can_id(5)),
            "DeviceAddress(can_id=Some(5))"
        );
        assert_eq!(
            format!("{:?}", DeviceAddress::uuid([0x01, 0x02])),
            "DeviceAddress(can_id=None, uuid=01:02)"
        );
        assert_eq!(
            format!("{:?}", DeviceAddress::can_id(5).with_transport_device(0)),
            "DeviceAddress(can_id=Some(5), td=0)"
        );
    }

    #[test]
    fn test_equality_with_transport_device() {
        let a = DeviceAddress::can_id(5);
        let b = DeviceAddress::can_id(5).with_transport_device(0);
        // Different transport_device means different addresses
        assert_ne!(a, b);
    }
}
