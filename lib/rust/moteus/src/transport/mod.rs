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

//! Transport layer for communicating with moteus controllers.
//!
//! This module provides the `Transport` struct that manages communication
//! with moteus controllers over CAN-FD, routing frames across multiple channels.
//!
//! ## API Overview
//!
//! The Transport API provides several methods matching the Python API:
//!
//! - [`cycle()`](Transport::cycle): Execute a batch transaction (send frames, collect responses)
//! - [`write()`](Transport::write): Fire-and-forget single frame send
//! - [`read()`](Transport::read): Receive an unsolicited frame
//! - [`flush_read()`](Transport::flush_read): Clear any pending unsolicited frames
//!
//! ## Module Structure
//!
//! - [`args`]: Command-line argument support for transport options
//! - [`fdcanusb`]: FdCanUSB serial protocol implementation
//! - [`socketcan`]: Linux SocketCAN transport (Linux only)
//! - [`device`]: TransportDevice trait for hardware abstraction
//! - [`discovery`]: Device enumeration and detection
//! - [`factory`]: Factory pattern for transport creation
//! - [`singleton`]: Global singleton transport
//!
//! ## Tokio Feature
//!
//! When the `tokio` feature is enabled, additional async transport modules are available:
//!
//! - [`async_fdcanusb`]: Tokio-based async FdCanUSB transport
//! - [`async_socketcan`]: Tokio-based async SocketCAN transport (Linux only)
//! - [`async_factory`]: Async transport factory for device creation
//! - [`async_transport`]: Async Transport for multi-device routing
//! - [`async_singleton`]: Global async singleton transport

pub mod args;
pub mod async_transport;
pub mod device;
pub mod discovery;
pub mod factory;
pub mod fdcanusb;
pub mod singleton;
pub mod socketcan;
pub(crate) mod socketcan_common;
pub mod transaction;

// Tokio-based async transports (requires tokio feature)
#[cfg(feature = "tokio")]
pub mod async_factory;
#[cfg(feature = "tokio")]
pub mod async_fdcanusb;
#[cfg(feature = "tokio")]
pub mod async_singleton;
#[cfg(feature = "tokio")]
pub mod async_socketcan;

use crate::device_address::DeviceAddress;
use crate::error::{Error, Result};
use crate::transport::device::{TransportDevice, TransportDeviceInfo};
use moteus_protocol::{CanFdFrame, Register, Resolution};
use std::collections::HashMap;

// Re-export transaction types
pub use transaction::{dispatch_frame, FrameFilter, Request, ResponseCollector};

// =============================================================================
// TransportOps Trait
// =============================================================================

/// Trait for transport operations.
///
/// This trait defines the common interface for transport implementations.
/// Most users should use the concrete [`Transport`] struct directly, which
/// provides these methods as inherent methods.
///
/// This trait is useful for:
/// - Writing generic code that works with any transport
/// - Implementing custom transports
/// - Using `Box<dyn TransportOps>` for polymorphism
pub trait TransportOps {
    /// Executes a cycle: sends frames and collects responses.
    fn cycle(&mut self, requests: &mut [Request]) -> Result<()>;

    /// Sends a frame without waiting for a response.
    fn write(&mut self, frame: &CanFdFrame) -> Result<()>;

    /// Receives an unsolicited frame, if available.
    fn read(&mut self, channel: Option<usize>) -> Result<Option<CanFdFrame>>;

    /// Flushes any pending unsolicited frames.
    fn flush_read(&mut self, channel: Option<usize>) -> Result<()>;

    /// Sets the communication timeout in milliseconds.
    fn set_timeout(&mut self, timeout_ms: u32);

    /// Returns the current timeout in milliseconds.
    fn timeout(&self) -> u32;
}

// =============================================================================
// DeviceInfo
// =============================================================================

/// Information about a discovered device on the CAN bus.
#[derive(Debug, Clone)]
pub struct DeviceInfo {
    /// The CAN ID of the device.
    pub can_id: u8,
    /// The device's UUID (16 bytes), if available.
    pub uuid: Option<[u8; 16]>,
    /// Index of the transport device this controller is connected to.
    pub transport_device_idx: usize,
    /// Human-readable description of the transport device.
    pub transport_device: String,
    /// The minimal address needed to uniquely identify this device.
    pub address: Option<DeviceAddress>,
}

impl DeviceInfo {
    /// Create a new DeviceInfo.
    pub fn new(can_id: u8, transport_device_idx: usize, transport_device: String) -> Self {
        Self {
            can_id,
            uuid: None,
            transport_device_idx,
            transport_device,
            address: Some(DeviceAddress::can_id(can_id)),
        }
    }

    /// Create a DeviceInfo with UUID.
    pub fn with_uuid(
        can_id: u8,
        uuid: [u8; 16],
        transport_device_idx: usize,
        transport_device: String,
    ) -> Self {
        Self {
            can_id,
            uuid: Some(uuid),
            transport_device_idx,
            transport_device,
            address: Some(DeviceAddress::can_id(can_id)),
        }
    }

    /// Format UUID as a standard UUID string (e.g., "01020304-0506-0708-090a-0b0c0d0e0f10").
    pub fn uuid_string(&self) -> Option<String> {
        self.uuid.map(|bytes| {
            format!(
                "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
                bytes[0], bytes[1], bytes[2], bytes[3],
                bytes[4], bytes[5],
                bytes[6], bytes[7],
                bytes[8], bytes[9],
                bytes[10], bytes[11], bytes[12], bytes[13], bytes[14], bytes[15]
            )
        })
    }
}

impl std::fmt::Display for DeviceInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let uuid_str = self
            .uuid_string()
            .unwrap_or_else(|| "None".to_string());

        write!(
            f,
            "DeviceInfo(can_id={}, uuid={}, td={})",
            self.can_id, uuid_str, self.transport_device
        )
    }
}

// =============================================================================
// UUID Helper Functions
// =============================================================================

/// Create a query frame for reading UUID registers.
///
/// Returns the frame and expected reply size.
pub(crate) fn make_uuid_query_frame(source: u8, can_prefix: u16) -> (CanFdFrame, u8) {
    use moteus_protocol::{WriteCanData, WriteCombiner};

    let mut frame = CanFdFrame::new();
    frame.arbitration_id = moteus_protocol::calculate_arbitration_id(
        source as i8, 0x7F_i8, can_prefix, true);

    let mut writer = WriteCanData::new(&mut frame);

    // Query UUID1-4 registers (0x150-0x153) as INT32 and UUID_MASK_CAPABLE (0x158) as INT8
    let resolutions = [
        Resolution::Int32, // UUID1
        Resolution::Int32, // UUID2
        Resolution::Int32, // UUID3
        Resolution::Int32, // UUID4
        Resolution::Ignore, // UUID_MASK1 (skip)
        Resolution::Ignore, // UUID_MASK2 (skip)
        Resolution::Ignore, // UUID_MASK3 (skip)
        Resolution::Ignore, // UUID_MASK4 (skip)
        Resolution::Int8,  // UUID_MASK_CAPABLE
    ];

    let mut combiner = WriteCombiner::new(
        0x10, // Read base
        Register::Uuid1.address(),
        &resolutions,
    );

    for _ in 0..resolutions.len() {
        combiner.maybe_write(&mut writer);
    }

    let reply_size = combiner.reply_size();

    (frame, reply_size)
}

/// Create UUID prefix data for addressing a device by UUID.
///
/// This creates the command bytes that should be prepended to any command
/// sent to a device addressed by UUID. It writes the UUID prefix to the
/// UUID_MASK registers, which causes only matching devices to respond.
///
/// # Arguments
///
/// * `uuid_prefix` - The UUID prefix bytes (must be a multiple of 4 bytes, max 16)
///
/// # Returns
///
/// The command bytes to prepend to the frame data.
pub fn make_uuid_prefix(uuid_prefix: &[u8]) -> Vec<u8> {
    use moteus_protocol::{WriteCanData, WriteCombiner};

    if uuid_prefix.is_empty() {
        return Vec::new();
    }

    assert!(
        uuid_prefix.len() % 4 == 0 && uuid_prefix.len() <= 16,
        "UUID prefix must be a multiple of 4 bytes and at most 16 bytes"
    );

    let mut frame = CanFdFrame::new();
    let mut writer = WriteCanData::new(&mut frame);

    // Number of 32-bit registers to write
    let reg_count = uuid_prefix.len() / 4;

    // Build resolutions array (all INT32 for the registers we're writing)
    let resolutions = [Resolution::Int32; 4];

    let mut combiner = WriteCombiner::new(
        0x00, // Write base
        Register::UuidMask1.address(),
        &resolutions[..reg_count],
    );

    // Convert UUID prefix bytes to 32-bit little-endian integers
    for i in 0..reg_count {
        if combiner.maybe_write(&mut writer) {
            let offset = i * 4;
            let val = u32::from_le_bytes([
                uuid_prefix[offset],
                uuid_prefix[offset + 1],
                uuid_prefix[offset + 2],
                uuid_prefix[offset + 3],
            ]);
            writer.write_i32(val as i32);
        }
    }

    frame.data[..frame.size as usize].to_vec()
}

/// Extract UUID bytes from a response frame.
///
/// Returns the 16-byte UUID if the frame contains valid UUID data and
/// the device supports UUID mask capability.
pub(crate) fn extract_uuid_from_response(frame: &CanFdFrame) -> Option<[u8; 16]> {
    use moteus_protocol::{parse_frame, Subframe};

    let data = &frame.data[..frame.size as usize];

    let mut uuid_parts: [Option<i32>; 4] = [None; 4];
    let mut uuid_mask_capable: Option<i8> = None;

    for subframe in parse_frame(data) {
        let (register, value) = match subframe {
            Subframe::Register { register, value: Some(value), .. } => (register, value),
            _ => continue,
        };

        match register {
            r if r == Register::Uuid1.address() => {
                uuid_parts[0] = Some(value.to_i32());
            }
            r if r == Register::Uuid2.address() => {
                uuid_parts[1] = Some(value.to_i32());
            }
            r if r == Register::Uuid3.address() => {
                uuid_parts[2] = Some(value.to_i32());
            }
            r if r == Register::Uuid4.address() => {
                uuid_parts[3] = Some(value.to_i32());
            }
            r if r == Register::UuidMaskCapable.address() => {
                uuid_mask_capable = Some(value.to_i32() as i8);
            }
            _ => {}
        }
    }

    // Check if UUID mask is capable (non-zero), matching Python's `== 0` check
    match uuid_mask_capable {
        None => return None,
        Some(0) => return None,
        _ => {}
    }

    // Check if all UUID parts are present
    if uuid_parts.iter().any(|p| p.is_none()) {
        return None;
    }

    // Combine into 16-byte UUID
    let mut uuid = [0u8; 16];
    for (i, part) in uuid_parts.iter().enumerate() {
        let val = part.unwrap() as u32;
        uuid[i * 4] = (val & 0xFF) as u8;
        uuid[i * 4 + 1] = ((val >> 8) & 0xFF) as u8;
        uuid[i * 4 + 2] = ((val >> 16) & 0xFF) as u8;
        uuid[i * 4 + 3] = ((val >> 24) & 0xFF) as u8;
    }

    Some(uuid)
}

/// Resolve unique addresses for discovered devices.
///
/// CAN ID uniqueness is checked within the same transport device, matching
/// the Python library behavior. Each resulting address includes the
/// transport device index for direct routing.
pub(crate) fn resolve_addresses(devices: &mut [DeviceInfo]) {
    for i in 0..devices.len() {
        let can_id = devices[i].can_id;
        let transport_idx = devices[i].transport_device_idx;

        // Check if this CAN ID is unique within the same transport device
        let can_id_conflicts: Vec<usize> = devices
            .iter()
            .enumerate()
            .filter(|(j, d)| {
                *j != i && d.can_id == can_id
                    && d.transport_device_idx == transport_idx
            })
            .map(|(j, _)| j)
            .collect();

        if can_id_conflicts.is_empty() {
            // CAN ID is unique within this transport device, use it
            devices[i].address = Some(
                DeviceAddress::can_id(can_id)
                    .with_transport_device(transport_idx),
            );
        } else if let Some(uuid) = devices[i].uuid {
            // CAN ID conflicts within same transport, need to use UUID prefix
            for prefix_len in [4usize, 8, 12, 16] {
                let prefix = &uuid[..prefix_len];

                let has_conflict = devices.iter().enumerate().any(|(j, d)| {
                    if j == i {
                        return false;
                    }
                    if let Some(other_uuid) = d.uuid {
                        other_uuid[..prefix_len] == *prefix
                    } else {
                        false
                    }
                });

                if !has_conflict {
                    devices[i].address = Some(
                        DeviceAddress::uuid(prefix.to_vec())
                            .with_transport_device(transport_idx),
                    );
                    break;
                }
            }
        } else {
            // No UUID and CAN ID conflicts - device is not uniquely addressable
            devices[i].address = None;
        }
    }
}

// =============================================================================
// Transport Struct
// =============================================================================

/// A transport that manages multiple CAN-FD devices.
///
/// The `Transport` routes frames to the appropriate device based on
/// a `DeviceAddress` lookup, matching the Python library's
/// `Dict[DeviceAddress, TransportDevice]` routing table.
///
/// When a destination is not in the routing table, on-demand discovery
/// probes all buses.  If more than one bus responds, an error is returned
/// — the caller must use UUID-based addressing or set up the routing
/// table manually.
///
/// # Example
///
/// ```ignore
/// use moteus::Transport;
/// use moteus::transport::factory::TransportOptions;
///
/// // Create transport with auto-discovery
/// let opts = TransportOptions::new();
/// let mut transport = Transport::with_options(&opts)?;
///
/// // Discover devices
/// let devices = transport.discover(0, 0)?;
/// for dev in &devices {
///     println!("Found: CAN ID {}", dev.can_id);
/// }
///
/// // Send commands
/// let mut requests = vec![Request::new(frame)];
/// transport.cycle(&mut requests)?;
/// ```
pub struct Transport {
    /// The underlying devices.
    devices: Vec<Box<dyn TransportDevice>>,
    /// Deduplicated parent device indices for read/flush/broadcast.
    ///
    /// For each device, if `parent_index` is `Some(p)`, use `p`; otherwise
    /// use the device's own index. This list is deduplicated and sorted.
    parent_indices: Vec<usize>,
    /// Routing table: DeviceAddress -> device index.
    ///
    /// Each unique address maps to exactly one transport device, matching
    /// the Python library's `Dict[DeviceAddress, TransportDevice]`.
    routing_table: HashMap<DeviceAddress, usize>,
    /// Timeout in milliseconds.
    timeout_ms: u32,
}

/// Compute the deduplicated parent indices from a list of devices.
///
/// For each device, if `parent_index` is `Some(p)`, use `p`; otherwise
/// use the device's own index. The result is deduplicated and sorted.
fn compute_parent_indices(devices: &[Box<dyn TransportDevice>]) -> Vec<usize> {
    let mut indices: Vec<usize> = devices
        .iter()
        .enumerate()
        .map(|(i, d)| d.info().parent_index.unwrap_or(i))
        .collect();
    indices.sort_unstable();
    indices.dedup();
    indices
}

impl Transport {
    /// Creates a new transport from a list of devices.
    ///
    /// The devices will be assigned sequential IDs starting from 0.
    pub fn new(devices: Vec<Box<dyn TransportDevice>>) -> Self {
        let parent_indices = compute_parent_indices(&devices);
        Self {
            devices,
            parent_indices,
            routing_table: HashMap::new(),
            timeout_ms: 100,
        }
    }

    /// Creates a transport from TransportDevice trait objects.
    pub fn from_devices<I, T>(devices: I) -> Self
    where
        I: IntoIterator<Item = T>,
        T: TransportDevice + 'static,
    {
        let boxed: Vec<Box<dyn TransportDevice>> = devices
            .into_iter()
            .map(|d| Box::new(d) as Box<dyn TransportDevice>)
            .collect();
        Self::new(boxed)
    }

    /// Returns the number of devices in this transport.
    pub fn device_count(&self) -> usize {
        self.devices.len()
    }

    /// Returns information about all devices.
    pub fn device_info(&self) -> Vec<&TransportDeviceInfo> {
        self.devices.iter().map(|d| d.info()).collect()
    }

    /// Manually add a routing entry.
    ///
    /// Maps a device address to a specific transport device index.
    /// Each address maps to exactly one device, matching the Python
    /// library's `Dict[DeviceAddress, TransportDevice]` routing table.
    pub fn add_route(&mut self, address: impl Into<DeviceAddress>, device_idx: usize) -> Result<()> {
        if device_idx >= self.devices.len() {
            return Err(Error::Protocol(format!(
                "Device index {} out of range (have {} devices)",
                device_idx,
                self.devices.len()
            )));
        }
        self.routing_table.insert(address.into(), device_idx);
        Ok(())
    }

    /// Clear all routing entries.
    pub fn clear_routes(&mut self) {
        self.routing_table.clear();
    }

    /// Look up the device for a destination address.
    fn get_device_for_address(&self, address: &DeviceAddress) -> Option<usize> {
        if self.devices.len() == 1 {
            return Some(0);
        }
        self.routing_table.get(address).copied()
    }

    /// Discover which device a destination address is connected to.
    ///
    /// Probes all transport devices. If more than one responds, returns
    /// an error — the caller must use UUID-based addressing or set up
    /// the routing table manually. Matching Python's
    /// `_get_devices_for_command` discovery behavior.
    fn discover_device(&mut self, address: &DeviceAddress, source: u8, can_prefix: u16) -> Result<usize> {
        if self.devices.is_empty() {
            return Err(Error::NotConnected);
        }

        if self.devices.len() == 1 {
            return Ok(0);
        }

        let can_id = address.as_can_id().unwrap_or(0x7F);

        // Build probe frame
        let mut query_frame = CanFdFrame::new();
        query_frame.arbitration_id =
            moteus_protocol::calculate_arbitration_id(source as i8, can_id as i8, can_prefix, true);

        // For UUID-only addresses, prepend UUID mask prefix data
        if address.can_id.is_none() {
            if let Some(uuid) = address.as_uuid() {
                let prefix_data = make_uuid_prefix(uuid);
                query_frame.data[..prefix_data.len()].copy_from_slice(&prefix_data);
                query_frame.size = prefix_data.len() as u8;
            }
        }

        let offset = query_frame.size as usize;
        query_frame.data[offset] = 0x50; // Read mode
        query_frame.size += 1;

        // Flush all devices first
        for device in &mut self.devices {
            let _ = device.flush();
        }

        let mut found: Vec<usize> = Vec::new();

        // Try each device that supports broadcast
        for (idx, device) in self.devices.iter_mut().enumerate() {
            if !device.empty_bus_tx_safe() {
                continue;
            }

            let mut requests = vec![Request::new(query_frame.clone())];
            if device.transaction(&mut requests).is_ok() {
                for response in requests[0].responses.take() {
                    let source_id = ((response.arbitration_id >> 8) & 0x7F) as u8;
                    if source_id != 0x7F && !found.contains(&idx) {
                        found.push(idx);
                        break;
                    }
                }
            }
        }

        match found.len() {
            0 => Err(Error::DeviceNotFound(format!(
                "{} not found on any CAN bus",
                address
            ))),
            1 => {
                self.routing_table.insert(address.clone(), found[0]);
                Ok(found[0])
            }
            _ => Err(Error::Protocol(format!(
                "More than one {} found across connected CAN busses",
                address
            ))),
        }
    }

    /// Get from cache or discover the device for a destination address.
    fn get_or_discover_device(&mut self, address: &DeviceAddress, source: u8, can_prefix: u16) -> Result<usize> {
        if let Some(idx) = self.get_device_for_address(address) {
            return Ok(idx);
        }
        self.discover_device(address, source, can_prefix)
    }

    /// Discover all controllers attached to any transport devices.
    ///
    /// Sends a broadcast UUID query to all devices and collects responses.
    /// Matching the Python library, this does **not** populate the routing
    /// table — the first command to each destination will trigger on-demand
    /// discovery.
    ///
    /// # Returns
    ///
    /// A list of `DeviceInfo` structures containing the CAN ID, UUID (if available),
    /// and transport device index for each discovered controller.
    pub fn discover(
        &mut self,
        can_prefix: u16,
        source: u8,
    ) -> Result<Vec<DeviceInfo>> {
        if self.devices.is_empty() {
            return Err(Error::NotConnected);
        }

        // Flush any stale frames before discovering
        let _ = self.flush_read(None);

        // Build a broadcast UUID query frame
        let (query_frame, _reply_size) = make_uuid_query_frame(source, can_prefix);

        let mut discovered = Vec::new();

        // Send to each device that supports broadcast and collect responses
        for (idx, device) in self.devices.iter_mut().enumerate() {
            if !device.empty_bus_tx_safe() {
                continue;
            }

            // Get transport device description before transaction
            let transport_device = device.info().to_string();

            // For broadcast, we don't know how many devices will respond.
            // Use a large expected_replies to force waiting for the full timeout.
            let mut requests = vec![
                Request::new(query_frame.clone())
                    .with_filter(FrameFilter::Any) // Accept all responses
                    .with_expected_replies(127)    // Max possible CAN IDs
            ];

            if device.transaction(&mut requests).is_ok() {
                for response in requests[0].responses.take() {
                    let source_id = ((response.arbitration_id >> 8) & 0x7F) as u8;
                    let dest_id = (response.arbitration_id & 0x7F) as u8;

                    // Skip frames not addressed to us
                    if source_id == 0x7F || dest_id != source {
                        continue;
                    }

                    // Try to extract UUID from response
                    let uuid = extract_uuid_from_response(&response);

                    let device_info = if let Some(uuid) = uuid {
                        DeviceInfo::with_uuid(source_id, uuid, idx, transport_device.clone())
                    } else {
                        DeviceInfo::new(source_id, idx, transport_device.clone())
                    };

                    discovered.push(device_info);
                }
            }
        }

        // Determine unique addresses for each device
        resolve_addresses(&mut discovered);

        // Sort by CAN ID, then by UUID for consistent ordering
        discovered.sort_by(|a, b| {
            match a.can_id.cmp(&b.can_id) {
                std::cmp::Ordering::Equal => a.uuid.cmp(&b.uuid),
                other => other,
            }
        });

        Ok(discovered)
    }

    /// Execute a cycle, routing requests to appropriate devices.
    fn execute_cycle(&mut self, requests: &mut [Request]) -> Result<()> {
        if self.devices.is_empty() {
            return Err(Error::NotConnected);
        }

        if self.devices.len() == 1 {
            // Simple case: just use the only device
            let result = self.devices[0].transaction(requests);
            for req in requests.iter() {
                let frames = req.responses.take();
                for mut frame in frames {
                    frame.channel = Some(0);
                    req.responses.push(frame);
                }
            }
            return result;
        }

        // Group request indices by destination device.
        //
        // This produces a map from device index to (is_broadcast_with_reply, orig_index)
        // so that within each device we can sequence broadcast-with-reply requests
        // before the non-broadcast batch, matching Python's _cycle_batch.
        let mut device_request_indices: HashMap<usize, Vec<(bool, usize)>> = HashMap::new();

        for i in 0..requests.len() {
            // Explicit channel bypasses routing table
            if let Some(device_idx) = requests[i].channel {
                device_request_indices
                    .entry(device_idx)
                    .or_default()
                    .push((false, i));
                continue;
            }

            let dest_id = match &requests[i].frame {
                Some(frame) => Some((frame.arbitration_id & 0x7F) as u8),
                None => None,
            };

            if let Some(dest_id) = dest_id {
                // Determine if this is a true broadcast (0x7F and no UUID)
                // or a UUID-only addressed frame (0x7F with UUID).
                let is_true_broadcast = dest_id == 0x7F
                    && !matches!(
                        &requests[i].address,
                        Some(addr) if addr.uuid.is_some()
                    );

                if is_true_broadcast {
                    // Broadcast - send to all parent devices
                    let is_bwr = requests[i].expected_reply_count > 0;
                    for &device_idx in &self.parent_indices.clone() {
                        if self.devices[device_idx].empty_bus_tx_safe() {
                            device_request_indices
                                .entry(device_idx)
                                .or_default()
                                .push((is_bwr, i));
                        }
                    }
                } else {
                    // Addressed frame - look up single device
                    let lookup_addr = match &requests[i].address {
                        Some(addr) => addr.clone(),
                        None => DeviceAddress::can_id(dest_id),
                    };

                    let (source, can_prefix) = match &requests[i].frame {
                        Some(f) => {
                            let (src, _dest, pfx) = moteus_protocol::parse_arbitration_id(f.arbitration_id);
                            (src as u8, pfx)
                        }
                        None => (0, 0),
                    };
                    let mut target_idx = self.get_or_discover_device(&lookup_addr, source, can_prefix)?;

                    // If this device has a parent, route through the parent
                    if let Some(parent_idx) = self.devices[target_idx].info().parent_index {
                        requests[i].child_device = Some(target_idx);
                        target_idx = parent_idx;
                    }

                    device_request_indices
                        .entry(target_idx)
                        .or_default()
                        .push((false, i));
                }
            }
        }

        // Execute grouped requests per device.
        //
        // Within each device, broadcast-with-reply requests are sent one at
        // a time before running the remaining non-broadcast batch in a single
        // transaction, matching Python's _cycle_batch sequencing.
        for (device_idx, items) in device_request_indices {
            if let Some(device) = self.devices.get_mut(device_idx) {
                // Separate broadcast-with-reply from everything else
                let bwr_indices: Vec<usize> = items
                    .iter()
                    .filter(|(bwr, _)| *bwr)
                    .map(|(_, i)| *i)
                    .collect();
                let other_indices: Vec<usize> = items
                    .iter()
                    .filter(|(bwr, _)| !*bwr)
                    .map(|(_, i)| *i)
                    .collect();

                // Send broadcast-with-reply one at a time
                for &orig_idx in &bwr_indices {
                    let mut req = requests[orig_idx].clone();
                    req.responses = ResponseCollector::new();
                    let mut reqs = vec![req];

                    if device.transaction(&mut reqs).is_ok() {
                        for mut frame in reqs[0].responses.take() {
                            frame.channel = Some(device_idx);
                            requests[orig_idx].responses.push(frame);
                        }
                    }
                }

                // Send remaining requests as a batch
                if !other_indices.is_empty() {
                    let mut device_requests: Vec<Request> = other_indices
                        .iter()
                        .map(|&i| {
                            let mut req = requests[i].clone();
                            req.responses = ResponseCollector::new();
                            req
                        })
                        .collect();

                    device.transaction(&mut device_requests)?;

                    for (local_idx, &orig_idx) in other_indices.iter().enumerate() {
                        for mut frame in device_requests[local_idx].responses.take() {
                            frame.channel = Some(device_idx);
                            requests[orig_idx].responses.push(frame);
                        }
                    }
                }
            }
        }

        Ok(())
    }

    // =========================================================================
    // Public API Methods (matching Python Transport)
    // =========================================================================

    /// Executes a cycle: sends frames and collects responses.
    ///
    /// This is the primary method for communicating with moteus controllers.
    /// Each request contains a frame to send and a collector for responses.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut requests = vec![Request::new(frame)];
    /// transport.cycle(&mut requests)?;
    /// for response in requests[0].responses.take() {
    ///     // Process response
    /// }
    /// ```
    pub fn cycle(&mut self, requests: &mut [Request]) -> Result<()> {
        self.execute_cycle(requests)
    }

    /// Sends a frame without waiting for a response.
    ///
    /// This is a fire-and-forget operation. The frame is routed to the
    /// appropriate device based on the destination CAN ID.
    pub fn write(&mut self, frame: &CanFdFrame) -> Result<()> {
        if self.devices.is_empty() {
            return Err(Error::NotConnected);
        }

        // Route to appropriate device based on destination
        let dest_id = (frame.arbitration_id & 0x7F) as u8;

        if dest_id == 0x7F {
            // Broadcast - send to all devices
            for device in &mut self.devices {
                if device.empty_bus_tx_safe() {
                    device.write(frame)?;
                }
            }
            Ok(())
        } else {
            let addr = DeviceAddress::can_id(dest_id);
            let (src, _dest, pfx) = moteus_protocol::parse_arbitration_id(frame.arbitration_id);
            let device_idx = self.get_or_discover_device(&addr, src as u8, pfx)?;
            self.devices[device_idx].write(frame)
        }
    }

    /// Receives an unsolicited frame, if available.
    ///
    /// # Arguments
    /// * `channel` - Optional channel index to read from. If None, reads from any channel.
    pub fn read(&mut self, channel: Option<usize>) -> Result<Option<CanFdFrame>> {
        if self.devices.is_empty() {
            return Err(Error::NotConnected);
        }

        if let Some(idx) = channel {
            // Read from specific channel
            if let Some(device) = self.devices.get_mut(idx) {
                match device.read()? {
                    Some(mut frame) => {
                        frame.channel = Some(idx);
                        Ok(Some(frame))
                    }
                    None => Ok(None),
                }
            } else {
                Err(Error::DeviceNotFound(format!("Channel {} not found", idx)))
            }
        } else {
            // Try to read from parent devices only
            for &idx in &self.parent_indices {
                if let Ok(Some(mut frame)) = self.devices[idx].read() {
                    frame.channel = Some(idx);
                    return Ok(Some(frame));
                }
            }
            Ok(None)
        }
    }

    /// Flushes any pending unsolicited frames.
    ///
    /// # Arguments
    /// * `channel` - Optional channel index to flush. If None, flushes all channels.
    pub fn flush_read(&mut self, channel: Option<usize>) -> Result<()> {
        if self.devices.is_empty() {
            return Err(Error::NotConnected);
        }

        if let Some(idx) = channel {
            // Flush specific channel
            if let Some(device) = self.devices.get_mut(idx) {
                device.flush()
            } else {
                Err(Error::DeviceNotFound(format!("Channel {} not found", idx)))
            }
        } else {
            // Flush parent devices only
            for &idx in &self.parent_indices {
                self.devices[idx].flush()?;
            }
            Ok(())
        }
    }

    /// Sets the communication timeout in milliseconds.
    pub fn set_timeout(&mut self, timeout_ms: u32) {
        self.timeout_ms = timeout_ms;
        for device in &mut self.devices {
            device.set_timeout(timeout_ms);
        }
    }

    /// Returns the current timeout in milliseconds.
    pub fn timeout(&self) -> u32 {
        self.timeout_ms
    }
}

impl TransportOps for Transport {
    fn cycle(&mut self, requests: &mut [Request]) -> Result<()> {
        Transport::cycle(self, requests)
    }

    fn write(&mut self, frame: &CanFdFrame) -> Result<()> {
        Transport::write(self, frame)
    }

    fn read(&mut self, channel: Option<usize>) -> Result<Option<CanFdFrame>> {
        Transport::read(self, channel)
    }

    fn flush_read(&mut self, channel: Option<usize>) -> Result<()> {
        Transport::flush_read(self, channel)
    }

    fn set_timeout(&mut self, timeout_ms: u32) {
        Transport::set_timeout(self, timeout_ms)
    }

    fn timeout(&self) -> u32 {
        Transport::timeout(self)
    }
}

// =============================================================================
// NullTransport
// =============================================================================

/// A null transport that doesn't send anything.
///
/// Useful for testing frame construction without hardware.
pub struct NullTransport {
    timeout_ms: u32,
}

impl NullTransport {
    /// Creates a new null transport.
    pub fn new() -> Self {
        NullTransport { timeout_ms: 100 }
    }
}

impl Default for NullTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl TransportOps for NullTransport {
    fn cycle(&mut self, _requests: &mut [Request]) -> Result<()> {
        // NullTransport doesn't send or receive anything
        Ok(())
    }

    fn write(&mut self, _frame: &CanFdFrame) -> Result<()> {
        // NullTransport doesn't send anything
        Ok(())
    }

    fn read(&mut self, _channel: Option<usize>) -> Result<Option<CanFdFrame>> {
        // NullTransport doesn't receive anything
        Ok(None)
    }

    fn flush_read(&mut self, _channel: Option<usize>) -> Result<()> {
        // NullTransport has nothing to flush
        Ok(())
    }

    fn set_timeout(&mut self, timeout_ms: u32) {
        self.timeout_ms = timeout_ms;
    }

    fn timeout(&self) -> u32 {
        self.timeout_ms
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::transaction::dispatch_frame;

    use std::sync::{Arc, Mutex};

    /// Shared state for observing what a MockDevice received.
    #[derive(Clone, Default)]
    struct MockDeviceLog(Arc<Mutex<Vec<CanFdFrame>>>);

    impl MockDeviceLog {
        fn new() -> Self {
            Self::default()
        }

        fn len(&self) -> usize {
            self.0.lock().unwrap().len()
        }
    }

    struct MockDevice {
        info: TransportDeviceInfo,
        timeout_ms: u32,
        responses: Vec<CanFdFrame>,
        /// Shared log of frames sent to this device, accessible after
        /// the device has been moved into a Transport.
        log: MockDeviceLog,
    }

    impl MockDevice {
        fn new(id: usize) -> Self {
            Self {
                info: TransportDeviceInfo::new(id, format!("mock{}", id)),
                timeout_ms: 100,
                responses: Vec::new(),
                log: MockDeviceLog::new(),
            }
        }

        fn with_response(mut self, response: CanFdFrame) -> Self {
            self.responses.push(response);
            self
        }
    }

    impl TransportDevice for MockDevice {
        fn transaction(&mut self, requests: &mut [Request]) -> Result<()> {
            // Record sent frames
            for req in requests.iter() {
                if let Some(frame) = &req.frame {
                    self.log.0.lock().unwrap().push(frame.clone());
                }
            }
            // Dispatch pre-configured responses to matching requests
            for frame in std::mem::take(&mut self.responses) {
                dispatch_frame(&frame, requests);
            }
            Ok(())
        }

        fn write(&mut self, frame: &CanFdFrame) -> Result<()> {
            self.log.0.lock().unwrap().push(frame.clone());
            Ok(())
        }

        fn read(&mut self) -> Result<Option<CanFdFrame>> {
            Ok(None)
        }

        fn flush(&mut self) -> Result<()> {
            Ok(())
        }

        fn info(&self) -> &TransportDeviceInfo {
            &self.info
        }

        fn set_timeout(&mut self, timeout_ms: u32) {
            self.timeout_ms = timeout_ms;
        }

        fn timeout(&self) -> u32 {
            self.timeout_ms
        }
    }

    #[test]
    fn test_null_transport_cycle() {
        let mut transport = NullTransport::new();
        let frame = CanFdFrame::new();
        let mut requests = vec![Request::new(frame)];
        transport.cycle(&mut requests).unwrap();
        // NullTransport produces no responses
        assert!(requests[0].responses.is_empty());
    }

    #[test]
    fn test_null_transport_write() {
        let mut transport = NullTransport::new();
        let frame = CanFdFrame::new();
        transport.write(&frame).unwrap();
    }

    #[test]
    fn test_null_transport_read() {
        let mut transport = NullTransport::new();
        let result = transport.read(None).unwrap();
        assert!(result.is_none());
    }

    #[test]
    fn test_null_transport_flush_read() {
        let mut transport = NullTransport::new();
        transport.flush_read(None).unwrap();
    }

    #[test]
    fn test_null_transport_timeout() {
        let mut transport = NullTransport::new();
        assert_eq!(transport.timeout(), 100);
        transport.set_timeout(500);
        assert_eq!(transport.timeout(), 500);
    }

    #[test]
    fn test_transport_single_device() {
        let device = MockDevice::new(0);
        let mut transport = Transport::from_devices(vec![device]);

        assert_eq!(transport.device_count(), 1);

        let frame = CanFdFrame::new();
        let mut requests = vec![Request::new(frame)];
        let result = transport.cycle(&mut requests);
        assert!(result.is_ok());
    }

    #[test]
    fn test_transport_multiple_devices() {
        let device0 = MockDevice::new(0);
        let device1 = MockDevice::new(1);

        let mut transport = Transport::from_devices(vec![device0, device1]);

        assert_eq!(transport.device_count(), 2);

        // Add manual routes
        transport.add_route(1u8, 0).unwrap();
        transport.add_route(2u8, 1).unwrap();

        // Invalid route should fail
        let result = transport.add_route(3u8, 5);
        assert!(result.is_err());
    }

    #[test]
    fn test_transport_timeout() {
        let device0 = MockDevice::new(0);
        let device1 = MockDevice::new(1);

        let mut transport = Transport::from_devices(vec![device0, device1]);

        assert_eq!(transport.timeout(), 100);

        transport.set_timeout(500);
        assert_eq!(transport.timeout(), 500);
    }

    #[test]
    fn test_transport_empty() {
        let devices: Vec<MockDevice> = vec![];
        let mut transport = Transport::from_devices(devices);

        let frame = CanFdFrame::new();
        let mut requests = vec![Request::new(frame)];
        let result = transport.cycle(&mut requests);
        assert!(matches!(result, Err(Error::NotConnected)));
    }

    #[test]
    fn test_mock_device_with_response() {
        use moteus_protocol::calculate_arbitration_id;

        // Create a request targeting servo ID 1
        let mut cmd_frame = CanFdFrame::new();
        cmd_frame.arbitration_id = calculate_arbitration_id(0, 1, 0, true);

        // Create a response frame from servo ID 1
        let mut resp_frame = CanFdFrame::new();
        resp_frame.arbitration_id = calculate_arbitration_id(1, 0, 0, false);
        resp_frame.size = 3;

        let device = MockDevice::new(0).with_response(resp_frame);
        let mut transport = Transport::from_devices(vec![device]);

        let mut requests = vec![Request::new(cmd_frame)];
        transport.cycle(&mut requests).unwrap();

        assert_eq!(requests[0].responses.len(), 1);
        let responses = requests[0].responses.peek();
        assert_eq!(responses[0].size, 3);
        assert_eq!(responses[0].channel, Some(0));
    }

    #[test]
    fn test_transport_channel_routing() {
        let device0 = MockDevice::new(0);
        let device1 = MockDevice::new(1);

        let mut transport = Transport::from_devices(vec![device0, device1]);

        // Create requests explicitly routed to different channels
        let frame0 = CanFdFrame::new();
        let frame1 = CanFdFrame::new();

        let mut requests = vec![
            Request::new(frame0).with_channel(0),
            Request::new(frame1).with_channel(1),
        ];

        let result = transport.cycle(&mut requests);
        assert!(result.is_ok());
    }

    #[test]
    fn test_add_route_overwrites() {
        // In the Python-matching design, each DeviceAddress maps to
        // exactly one device. A second add_route for the same address
        // overwrites the previous entry.
        let device0 = MockDevice::new(0);
        let device1 = MockDevice::new(1);

        let mut transport = Transport::from_devices(vec![device0, device1]);

        transport.add_route(1u8, 0).unwrap();
        assert_eq!(transport.get_device_for_address(&DeviceAddress::can_id(1)), Some(0));

        transport.add_route(1u8, 1).unwrap();
        assert_eq!(transport.get_device_for_address(&DeviceAddress::can_id(1)), Some(1));
    }

    #[test]
    fn test_cycle_explicit_channel_bypasses_routing_table() {
        use moteus_protocol::calculate_arbitration_id;

        let device0 = MockDevice::new(0);
        let log0 = device0.log.clone();
        let device1 = MockDevice::new(1);
        let log1 = device1.log.clone();

        let mut transport = Transport::from_devices(vec![device0, device1]);
        transport.add_route(1u8, 0).unwrap();

        let mut cmd_frame = CanFdFrame::new();
        cmd_frame.arbitration_id = calculate_arbitration_id(0, 1, 0, true);

        // Explicit channel routes to device 1 despite routing table saying 0
        let mut requests = vec![Request::new(cmd_frame).with_channel(1)];
        transport.cycle(&mut requests).unwrap();

        assert_eq!(log0.len(), 0);
        assert_eq!(log1.len(), 1);
    }

    #[test]
    fn test_clear_routes() {
        let device0 = MockDevice::new(0);
        let device1 = MockDevice::new(1);

        let mut transport = Transport::from_devices(vec![device0, device1]);
        transport.add_route(1u8, 0).unwrap();

        transport.clear_routes();
        assert!(transport.get_device_for_address(&DeviceAddress::can_id(1)).is_none());
    }

    #[test]
    fn test_unique_can_id_single_device_routing() {
        use moteus_protocol::calculate_arbitration_id;

        // Unique CAN IDs: ID 1 on device 0, ID 2 on device 1
        let device0 = MockDevice::new(0);
        let log0 = device0.log.clone();
        let device1 = MockDevice::new(1);
        let log1 = device1.log.clone();

        let mut transport = Transport::from_devices(vec![device0, device1]);
        transport.add_route(1u8, 0).unwrap();
        transport.add_route(2u8, 1).unwrap();

        // Request to CAN ID 1 should only go to device 0
        let mut frame1 = CanFdFrame::new();
        frame1.arbitration_id = calculate_arbitration_id(0, 1, 0, true);
        let mut requests = vec![Request::new(frame1)];
        transport.cycle(&mut requests).unwrap();

        assert_eq!(log0.len(), 1);
        assert_eq!(log1.len(), 0);

        // Request to CAN ID 2 should only go to device 1
        let mut frame2 = CanFdFrame::new();
        frame2.arbitration_id = calculate_arbitration_id(0, 2, 0, true);
        let mut requests = vec![Request::new(frame2)];
        transport.cycle(&mut requests).unwrap();

        assert_eq!(log0.len(), 1);
        assert_eq!(log1.len(), 1);
    }

    #[test]
    fn test_uuid_address_routes_to_single_device() {
        use moteus_protocol::calculate_arbitration_id;

        // Two devices, UUID-addressed controllers
        let device0 = MockDevice::new(0);
        let log0 = device0.log.clone();
        let device1 = MockDevice::new(1);
        let log1 = device1.log.clone();

        let uuid_a = DeviceAddress { can_id: Some(1), uuid: Some(vec![0x01, 0x02, 0x03, 0x04]), transport_device: None };
        let uuid_b = DeviceAddress { can_id: Some(1), uuid: Some(vec![0x05, 0x06, 0x07, 0x08]), transport_device: None };

        let mut transport = Transport::from_devices(vec![device0, device1]);
        transport.add_route(uuid_a.clone(), 0).unwrap();
        transport.add_route(uuid_b.clone(), 1).unwrap();

        // A request with UUID A address should only go to device 0
        let mut cmd_frame = CanFdFrame::new();
        cmd_frame.arbitration_id = calculate_arbitration_id(0, 1, 0, true);
        let mut req = Request::new(cmd_frame);
        req.address = Some(uuid_a.clone());

        let mut requests = vec![req];
        transport.cycle(&mut requests).unwrap();

        assert_eq!(log0.len(), 1);
        assert_eq!(log1.len(), 0);

        // A request with UUID B address should only go to device 1
        let mut cmd_frame = CanFdFrame::new();
        cmd_frame.arbitration_id = calculate_arbitration_id(0, 1, 0, true);
        let mut req = Request::new(cmd_frame);
        req.address = Some(uuid_b.clone());

        let mut requests = vec![req];
        transport.cycle(&mut requests).unwrap();

        assert_eq!(log0.len(), 1);
        assert_eq!(log1.len(), 1);
    }

    #[test]
    fn test_write_routes_to_single_device() {
        use moteus_protocol::calculate_arbitration_id;

        let device0 = MockDevice::new(0);
        let log0 = device0.log.clone();
        let device1 = MockDevice::new(1);
        let log1 = device1.log.clone();

        let mut transport = Transport::from_devices(vec![device0, device1]);
        transport.add_route(1u8, 0).unwrap();

        let mut frame = CanFdFrame::new();
        frame.arbitration_id = calculate_arbitration_id(0, 1, 0, false);
        transport.write(&frame).unwrap();

        assert_eq!(log0.len(), 1);
        assert_eq!(log1.len(), 0);
    }

    #[test]
    fn test_write_broadcast_sends_to_all() {
        use moteus_protocol::calculate_arbitration_id;

        let device0 = MockDevice::new(0);
        let log0 = device0.log.clone();
        let device1 = MockDevice::new(1);
        let log1 = device1.log.clone();

        let mut transport = Transport::from_devices(vec![device0, device1]);

        let mut frame = CanFdFrame::new();
        frame.arbitration_id = calculate_arbitration_id(0, 0x7F, 0, false);
        transport.write(&frame).unwrap();

        assert_eq!(log0.len(), 1);
        assert_eq!(log1.len(), 1);
    }
}
