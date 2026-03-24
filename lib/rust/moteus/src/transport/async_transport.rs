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

//! Async transport for moteus communication.
//!
//! This module provides the `AsyncTransport` struct that manages async
//! communication with moteus controllers over CAN-FD, routing frames
//! across multiple channels.
//!
//! The routing table is a single `HashMap<DeviceAddress, usize>` matching
//! the Python library's `Dict[DeviceAddress, TransportDevice]`.  Device
//! transactions are run in parallel across transport devices using
//! `tokio::task::JoinSet`, matching Python's `asyncio.gather`.

use crate::error::Result;
use crate::transport::transaction::Request;
use moteus_protocol::CanFdFrame;
use std::future::Future;
use std::pin::Pin;

#[cfg(feature = "tokio")]
use crate::device_address::DeviceAddress;
#[cfg(feature = "tokio")]
use crate::error::Error;
#[cfg(feature = "tokio")]
use crate::transport::async_factory::{create_async_transports, AsyncTransportOptions};
#[cfg(feature = "tokio")]
use crate::transport::device::AsyncTransportDevice;
#[cfg(feature = "tokio")]
use crate::transport::device::TransportDeviceInfo;
#[cfg(feature = "tokio")]
use crate::transport::transaction::{FrameFilter, ResponseCollector};
#[cfg(feature = "tokio")]
use crate::transport::DeviceInfo;
#[cfg(feature = "tokio")]
use super::{extract_uuid_from_response, make_uuid_prefix, make_uuid_query_frame, resolve_addresses};
#[cfg(feature = "tokio")]
use std::collections::HashMap;
#[cfg(feature = "tokio")]
use std::sync::Arc;
#[cfg(feature = "tokio")]
use tokio::sync::Mutex;

/// Type alias for a boxed future that can be sent across threads.
pub type BoxFuture<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;

// =============================================================================
// AsyncTransportOps Trait
// =============================================================================

/// A shared, cancellation-safe handle to an async transport device.
#[cfg(feature = "tokio")]
pub type SharedDevice = Arc<Mutex<Box<dyn AsyncTransportDevice>>>;

/// Trait for async transport operations.
///
/// This trait defines the common interface for async transport implementations.
/// Most users should use the concrete `AsyncTransport` struct directly, which
/// provides these methods as inherent async methods.
///
/// This trait is useful for:
/// - Writing generic code that works with any async transport
/// - Implementing custom async transports
/// - Using `Box<dyn AsyncTransportOps>` for polymorphism
///
/// # Cancel safety
///
/// All methods are cancel safe.
pub trait AsyncTransportOps: Send {
    /// Executes a cycle: sends frames and collects responses asynchronously.
    fn cycle<'a>(&'a mut self, requests: &'a mut [Request]) -> BoxFuture<'a, Result<()>>;

    /// Sends a frame without waiting for a response.
    fn write<'a>(&'a mut self, frame: &'a CanFdFrame) -> BoxFuture<'a, Result<()>>;

    /// Receives an unsolicited frame, if available.
    fn read(&mut self, channel: Option<usize>) -> BoxFuture<'_, Result<Option<CanFdFrame>>>;

    /// Flushes any pending unsolicited frames.
    fn flush_read(&mut self, channel: Option<usize>) -> BoxFuture<'_, Result<()>>;
}

// =============================================================================
// AsyncTransport Struct (tokio feature only)
// =============================================================================

/// An async transport that manages multiple CAN-FD devices.
#[cfg(feature = "tokio")]
///
/// The `AsyncTransport` routes frames to the appropriate device based on
/// a `DeviceAddress` lookup, matching the Python library's
/// `Dict[DeviceAddress, TransportDevice]` routing table.
///
/// Device transactions are run in parallel across transport devices using
/// `tokio::task::JoinSet`, matching Python's `asyncio.gather`.
///
/// # Cancel safety
///
/// All public async methods are cancel safe.
///
/// # Example
///
/// ```no_run
/// use moteus::AsyncTransport;
/// use moteus::transport::async_factory::AsyncTransportOptions;
///
/// #[tokio::main]
/// async fn main() -> Result<(), moteus::Error> {
///     let opts = AsyncTransportOptions::new();
///     let mut transport = AsyncTransport::with_options(&opts).await?;
///
///     // Discover devices
///     let devices = transport.discover(0, 0).await?;
///     for dev in &devices {
///         println!("Found: CAN ID {}", dev.can_id);
///     }
///
///     // Send commands
///     let mut requests = vec![Request::new(frame)];
///     transport.cycle(&mut requests).await?;
///     Ok(())
/// }
/// ```
pub struct AsyncTransport {
    /// The underlying async devices, wrapped for cancellation safety.
    devices: Vec<SharedDevice>,
    /// Cached device info (avoids locking to read immutable metadata).
    device_infos: Vec<TransportDeviceInfo>,
    /// Deduplicated parent device indices for read/flush/broadcast.
    parent_indices: Vec<usize>,
    /// Routing table: DeviceAddress -> device index.
    ///
    /// Each unique address maps to exactly one transport device, matching
    /// the Python library's `Dict[DeviceAddress, TransportDevice]`.
    routing_table: HashMap<DeviceAddress, usize>,
}

/// Compute the deduplicated parent indices from device info.
#[cfg(feature = "tokio")]
fn compute_parent_indices(infos: &[TransportDeviceInfo]) -> Vec<usize> {
    let mut indices: Vec<usize> = infos
        .iter()
        .enumerate()
        .map(|(i, d)| d.parent_index.unwrap_or(i))
        .collect();
    indices.sort_unstable();
    indices.dedup();
    indices
}

/// Per-device work item for parallel execution.
///
/// Groups all requests destined for one device, with broadcast-with-reply
/// requests separated for sequential execution.
#[cfg(feature = "tokio")]
struct DeviceWork {
    /// Index of the device in the transport's device list.
    device_idx: usize,
    /// Broadcast-with-reply requests: (original_request_index, cloned request).
    /// These are executed one at a time.
    broadcast_with_reply: Vec<(usize, Request)>,
    /// All other requests: (original_request_index, cloned request).
    /// These are executed in a single batch transaction.
    other: Vec<(usize, Request)>,
}

/// Result of running a device's work.
#[cfg(feature = "tokio")]
struct DeviceWorkResult {
    device_idx: usize,
    /// (original_request_index, collected responses)
    responses: Vec<(usize, Vec<CanFdFrame>)>,
    error: Option<Error>,
}

/// Execute one device's work: recover, broadcast-with-reply one at a time,
/// then issue remaining requests in a batch.
#[cfg(feature = "tokio")]
async fn run_device_work(
    device: SharedDevice,
    mut work: DeviceWork,
) -> DeviceWorkResult {
    let device_idx = work.device_idx;
    let mut responses: Vec<(usize, Vec<CanFdFrame>)> = Vec::new();
    let mut error: Option<Error> = None;

    let mut guard = device.lock().await;

    // Recover from any prior cancellation
    if let Err(e) = guard.recover().await {
        return DeviceWorkResult {
            device_idx,
            responses: vec![],
            error: Some(e),
        };
    }

    // Broadcast-with-reply: one at a time
    for (orig_idx, req) in work.broadcast_with_reply.drain(..) {
        let mut reqs = vec![req];
        match guard.transaction(&mut reqs).await {
            Ok(()) => {
                let frames = reqs[0].responses.take();
                responses.push((orig_idx, frames));
            }
            Err(e) => {
                if error.is_none() {
                    error = Some(e);
                }
            }
        }
    }

    // Remaining: single batch
    if !work.other.is_empty() {
        let orig_indices: Vec<usize> = work.other.iter().map(|(i, _)| *i).collect();
        let mut reqs: Vec<Request> = work.other.into_iter().map(|(_, r)| r).collect();

        match guard.transaction(&mut reqs).await {
            Ok(()) => {
                for (local_idx, orig_idx) in orig_indices.into_iter().enumerate() {
                    let frames = reqs[local_idx].responses.take();
                    responses.push((orig_idx, frames));
                }
            }
            Err(e) => {
                if error.is_none() {
                    error = Some(e);
                }
            }
        }
    }

    DeviceWorkResult {
        device_idx,
        responses,
        error,
    }
}

#[cfg(feature = "tokio")]
impl AsyncTransport {
    /// Creates a new async transport from a list of devices.
    pub fn new(devices: Vec<Box<dyn AsyncTransportDevice>>) -> Self {
        let device_infos: Vec<TransportDeviceInfo> = devices
            .iter()
            .map(|d| {
                let mut info = d.info().clone();
                info.empty_bus_tx_safe = d.empty_bus_tx_safe();
                info
            })
            .collect();
        let parent_indices = compute_parent_indices(&device_infos);
        let shared = devices
            .into_iter()
            .map(|d| Arc::new(Mutex::new(d)))
            .collect();
        Self {
            devices: shared,
            device_infos,
            parent_indices,
            routing_table: HashMap::new(),
        }
    }

    /// Creates an async transport from the default transport options.
    ///
    /// This auto-discovers available transports.
    pub async fn with_options(options: &AsyncTransportOptions) -> Result<Self> {
        let devices = create_async_transports(options).await?;
        if devices.is_empty() {
            return Err(Error::NotConnected);
        }
        Ok(Self::new(devices))
    }

    /// Returns the number of devices in this transport.
    pub fn device_count(&self) -> usize {
        self.devices.len()
    }

    /// Returns information about all devices.
    pub fn device_info(&self) -> Vec<&TransportDeviceInfo> {
        self.device_infos.iter().collect()
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
    /// Sends a probe to all buses in parallel and reads responses.
    /// If more than one bus responds, returns an error — matching
    /// Python's `_get_devices_for_command` discovery behavior.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    async fn discover_device(&mut self, address: &DeviceAddress, source: u8, can_prefix: u16) -> Result<usize> {
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

        // Flush all devices sequentially
        for device in &self.devices {
            let mut guard = device.lock().await;
            let _ = guard.flush().await;
        }

        // Send probe frame to all buses sequentially
        for (idx, device) in self.devices.iter().enumerate() {
            if self.device_infos[idx].empty_bus_tx_safe {
                let mut guard = device.lock().await;
                let _ = guard.write(&query_frame).await;
            }
        }

        // Read responses from all buses in parallel using JoinSet.
        // Use receive-only requests since probes were already sent above.
        let mut join_set = tokio::task::JoinSet::new();
        for (idx, device) in self.devices.iter().enumerate() {
            let device = Arc::clone(device);
            join_set.spawn(async move {
                let mut guard = device.lock().await;
                let _ = guard.recover().await;
                let mut requests = vec![Request::receive_only(FrameFilter::Any)];
                let ok = guard.transaction(&mut requests).await.is_ok();
                let mut found = false;
                if ok {
                    for response in requests[0].responses.take() {
                        let source_id = ((response.arbitration_id >> 8) & 0x7F) as u8;
                        if source_id != 0x7F {
                            found = true;
                            break;
                        }
                    }
                }
                (idx, found)
            });
        }

        let mut found: Vec<usize> = Vec::new();
        while let Some(result) = join_set.join_next().await {
            if let Ok((idx, was_found)) = result {
                if was_found {
                    found.push(idx);
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
    async fn get_or_discover_device(&mut self, address: &DeviceAddress, source: u8, can_prefix: u16) -> Result<usize> {
        if let Some(idx) = self.get_device_for_address(address) {
            return Ok(idx);
        }
        self.discover_device(address, source, can_prefix).await
    }

    /// Discover all controllers attached to any transport devices.
    ///
    /// Sends a broadcast UUID query to all devices and reads responses
    /// in parallel, matching Python's `discover()`.  Does **not** populate
    /// the routing table — the first command to each destination will
    /// trigger on-demand discovery.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub async fn discover(
        &mut self,
        can_prefix: u16,
        source: u8,
    ) -> Result<Vec<DeviceInfo>> {
        if self.devices.is_empty() {
            return Err(Error::NotConnected);
        }

        // Flush any stale frames before discovering
        let _ = self.flush_read(None).await;

        let (query_frame, _reply_size) = make_uuid_query_frame(source, can_prefix);

        // Send broadcast to all devices first (sequential, need lock)
        for (idx, device) in self.devices.iter().enumerate() {
            if self.device_infos[idx].empty_bus_tx_safe {
                let mut guard = device.lock().await;
                let _ = guard.write(&query_frame).await;
            }
        }

        // Read responses from all devices in parallel.
        // Use receive-only requests since probes were already sent above.
        let mut join_set = tokio::task::JoinSet::new();
        for (idx, device) in self.devices.iter().enumerate() {
            let device = Arc::clone(device);
            let transport_device = self.device_infos[idx].to_string();
            join_set.spawn(async move {
                let mut guard = device.lock().await;
                let _ = guard.recover().await;
                let mut requests = vec![
                    Request::receive_only(FrameFilter::Any)
                        .with_expected_replies(127)
                ];
                let _ = guard.transaction(&mut requests).await;
                let responses = requests[0].responses.take();
                (idx, transport_device, responses)
            });
        }

        let mut discovered = Vec::new();
        while let Some(result) = join_set.join_next().await {
            if let Ok((idx, transport_device, responses)) = result {
                for response in responses {
                    let source_id = ((response.arbitration_id >> 8) & 0x7F) as u8;
                    let dest_id = (response.arbitration_id & 0x7F) as u8;

                    // Skip frames not addressed to us
                    if source_id == 0x7F || dest_id != source {
                        continue;
                    }

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

        resolve_addresses(&mut discovered);

        discovered.sort_by(|a, b| match a.can_id.cmp(&b.can_id) {
            std::cmp::Ordering::Equal => a.uuid.cmp(&b.uuid),
            other => other,
        });

        Ok(discovered)
    }

    /// Execute a cycle, routing requests to appropriate devices.
    ///
    /// Device transactions are run in parallel using `tokio::task::JoinSet`,
    /// matching Python's `asyncio.gather(*tasks)`.  Within each device,
    /// broadcast-with-reply requests are sent one at a time before running
    /// the remaining non-broadcast batch, matching Python's `_cycle_batch`.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    async fn execute_cycle(&mut self, requests: &mut [Request]) -> Result<()> {
        if self.devices.is_empty() {
            return Err(Error::NotConnected);
        }

        if self.devices.len() == 1 {
            let mut guard = self.devices[0].lock().await;
            guard.recover().await?;
            let result = guard.transaction(requests).await;
            for req in requests.iter() {
                let frames = req.responses.take();
                for mut frame in frames {
                    frame.channel = Some(0);
                    req.responses.push(frame);
                }
            }
            return result;
        }

        // Group requests by destination device, matching Python's _cycle_batch.
        let mut device_groups: HashMap<usize, DeviceWork> = HashMap::new();

        #[allow(clippy::needless_range_loop, clippy::manual_map)]
        for i in 0..requests.len() {
            // Explicit channel bypasses routing table
            if let Some(device_idx) = requests[i].channel {
                let work = device_groups.entry(device_idx).or_insert_with(|| DeviceWork {
                    device_idx,
                    broadcast_with_reply: Vec::new(),
                    other: Vec::new(),
                });
                let mut req = requests[i].clone();
                req.responses = ResponseCollector::new();
                work.other.push((i, req));
                continue;
            }

            let dest_id = requests[i].frame.as_ref().map(|frame| (frame.arbitration_id & 0x7F) as u8);

            if let Some(dest_id) = dest_id {
                // Determine if this is a true broadcast (0x7F and no UUID)
                // or a UUID-only addressed frame (0x7F with UUID).
                let is_true_broadcast = dest_id == 0x7F
                    && !matches!(
                        &requests[i].address,
                        Some(addr) if addr.uuid.is_some()
                    );

                if is_true_broadcast {
                    let is_bwr = requests[i].expected_reply_count > 0;
                    for &device_idx in &self.parent_indices.clone() {
                        if self.device_infos[device_idx].empty_bus_tx_safe {
                            let work = device_groups.entry(device_idx).or_insert_with(|| DeviceWork {
                                device_idx,
                                broadcast_with_reply: Vec::new(),
                                other: Vec::new(),
                            });
                            let mut req = requests[i].clone();
                            req.responses = ResponseCollector::new();
                            if is_bwr {
                                work.broadcast_with_reply.push((i, req));
                            } else {
                                work.other.push((i, req));
                            }
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
                    let mut target_idx = self.get_or_discover_device(&lookup_addr, source, can_prefix).await?;

                    if let Some(parent_idx) = self.device_infos[target_idx].parent_index {
                        requests[i].child_device = Some(target_idx);
                        target_idx = parent_idx;
                    }

                    let work = device_groups.entry(target_idx).or_insert_with(|| DeviceWork {
                        device_idx: target_idx,
                        broadcast_with_reply: Vec::new(),
                        other: Vec::new(),
                    });
                    let mut req = requests[i].clone();
                    req.responses = ResponseCollector::new();
                    work.other.push((i, req));
                }
            }
        }

        // Execute all device groups in parallel using JoinSet.
        // Devices are behind Arc<Mutex<..>> — no drain/restore needed.
        let mut join_set = tokio::task::JoinSet::new();
        for (_, work) in device_groups {
            let device = Arc::clone(&self.devices[work.device_idx]);
            join_set.spawn(run_device_work(device, work));
        }

        // Collect results.
        let mut first_error: Option<Error> = None;
        while let Some(result) = join_set.join_next().await {
            match result {
                Ok(work_result) => {
                    for (orig_idx, frames) in work_result.responses {
                        for mut frame in frames {
                            frame.channel = Some(work_result.device_idx);
                            requests[orig_idx].responses.push(frame);
                        }
                    }
                    if let Some(e) = work_result.error {
                        if first_error.is_none() {
                            first_error = Some(e);
                        }
                    }
                }
                Err(join_err) => {
                    if first_error.is_none() {
                        first_error = Some(Error::Protocol(format!("task join error: {}", join_err)));
                    }
                }
            }
        }

        if let Some(e) = first_error {
            return Err(e);
        }

        Ok(())
    }

    // =========================================================================
    // Public API Methods (matching Python Transport)
    // =========================================================================

    /// Executes a cycle: sends frames and collects responses asynchronously.
    ///
    /// This is the primary method for communicating with moteus controllers.
    /// Each request contains a frame to send and a collector for responses.
    ///
    /// # Example
    ///
    /// ```no_run
    /// let mut requests = vec![Request::new(frame)];
    /// transport.cycle(&mut requests).await?;
    /// for response in requests[0].responses.take() {
    ///     // Process response
    /// }
    /// ```
    pub async fn cycle(&mut self, requests: &mut [Request]) -> Result<()> {
        self.execute_cycle(requests).await
    }

    /// Sends a frame without waiting for a response.
    ///
    /// This is a fire-and-forget operation. The frame is routed to the
    /// appropriate device based on the destination CAN ID.
    pub async fn write(&mut self, frame: &CanFdFrame) -> Result<()> {
        if self.devices.is_empty() {
            return Err(Error::NotConnected);
        }

        let dest_id = (frame.arbitration_id & 0x7F) as u8;

        if dest_id == 0x7F {
            // Broadcast - send to all devices
            for (idx, device) in self.devices.iter().enumerate() {
                if self.device_infos[idx].empty_bus_tx_safe {
                    let mut guard = device.lock().await;
                    guard.write(frame).await?;
                }
            }
            Ok(())
        } else {
            let addr = DeviceAddress::can_id(dest_id);
            let (src, _dest, pfx) = moteus_protocol::parse_arbitration_id(frame.arbitration_id);
            let device_idx = self.get_or_discover_device(&addr, src as u8, pfx).await?;
            let mut guard = self.devices[device_idx].lock().await;
            guard.write(frame).await
        }
    }

    /// Receives an unsolicited frame, if available.
    ///
    /// # Arguments
    /// * `channel` - Optional channel index to read from. If None, reads from any channel.
    pub async fn read(&mut self, channel: Option<usize>) -> Result<Option<CanFdFrame>> {
        if self.devices.is_empty() {
            return Err(Error::NotConnected);
        }

        if let Some(idx) = channel {
            if idx < self.devices.len() {
                let mut guard = self.devices[idx].lock().await;
                match guard.read().await? {
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
            for &idx in &self.parent_indices {
                let mut guard = self.devices[idx].lock().await;
                if let Ok(Some(mut frame)) = guard.read().await {
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
    pub async fn flush_read(&mut self, channel: Option<usize>) -> Result<()> {
        if self.devices.is_empty() {
            return Err(Error::NotConnected);
        }

        if let Some(idx) = channel {
            if idx < self.devices.len() {
                let mut guard = self.devices[idx].lock().await;
                guard.flush().await
            } else {
                Err(Error::DeviceNotFound(format!("Channel {} not found", idx)))
            }
        } else {
            for &idx in &self.parent_indices {
                let mut guard = self.devices[idx].lock().await;
                guard.flush().await?;
            }
            Ok(())
        }
    }
}

#[cfg(feature = "tokio")]
impl AsyncTransportOps for AsyncTransport {
    fn cycle<'a>(&'a mut self, requests: &'a mut [Request]) -> BoxFuture<'a, Result<()>> {
        Box::pin(AsyncTransport::cycle(self, requests))
    }

    fn write<'a>(&'a mut self, frame: &'a CanFdFrame) -> BoxFuture<'a, Result<()>> {
        Box::pin(AsyncTransport::write(self, frame))
    }

    fn read<'a>(&'a mut self, channel: Option<usize>) -> BoxFuture<'a, Result<Option<CanFdFrame>>> {
        Box::pin(AsyncTransport::read(self, channel))
    }

    fn flush_read<'a>(&'a mut self, channel: Option<usize>) -> BoxFuture<'a, Result<()>> {
        Box::pin(AsyncTransport::flush_read(self, channel))
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    #[cfg(feature = "tokio")]
    #[tokio::test]
    async fn test_async_transport_empty() {
        use crate::error::Error;
        let devices: Vec<Box<dyn AsyncTransportDevice>> = vec![];
        let mut transport = AsyncTransport::new(devices);

        let frame = CanFdFrame::new();
        let mut requests = vec![Request::new(frame)];
        let result = transport.cycle(&mut requests).await;
        assert!(matches!(result, Err(Error::NotConnected)));
    }

    #[cfg(feature = "tokio")]
    mod cancel_safety {
        use super::super::*;
        use crate::transport::device::{AsyncTransportDevice, TransportDeviceInfo};
        use crate::transport::transaction::Request;
        use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
        use std::sync::Arc;
        use std::time::Duration;

        /// Mock device that sleeps forever in transaction(), used to test
        /// cancellation.  Tracks recover() calls via shared atomic counter.
        struct CancellableMockDevice {
            info: TransportDeviceInfo,
            recover_count: Arc<AtomicUsize>,
            needs_recovery: bool,
            fast_mode: Arc<AtomicBool>,
        }

        impl CancellableMockDevice {
            fn new(fast_mode: Arc<AtomicBool>) -> Self {
                Self {
                    info: TransportDeviceInfo::new(0, "CancellableMock"),
                    recover_count: Arc::new(AtomicUsize::new(0)),
                    needs_recovery: false,
                    fast_mode,
                }
            }

            fn with_counter(
                recover_count: Arc<AtomicUsize>,
                fast_mode: Arc<AtomicBool>,
            ) -> Self {
                Self {
                    info: TransportDeviceInfo::new(0, "CancellableMock"),
                    recover_count,
                    needs_recovery: false,
                    fast_mode,
                }
            }
        }

        impl AsyncTransportDevice for CancellableMockDevice {
            fn recover(&mut self) -> BoxFuture<'_, Result<()>> {
                Box::pin(async move {
                    if self.needs_recovery {
                        self.recover_count.fetch_add(1, Ordering::Release);
                        self.needs_recovery = false;
                    }
                    Ok(())
                })
            }

            fn transaction<'a>(
                &'a mut self,
                _requests: &'a mut [Request],
            ) -> BoxFuture<'a, Result<()>> {
                Box::pin(async move {
                    self.needs_recovery = true;
                    if self.fast_mode.load(Ordering::Acquire) {
                        self.needs_recovery = false;
                        return Ok(());
                    }
                    // Sleep forever — caller cancels via timeout
                    tokio::time::sleep(Duration::from_secs(86400)).await;
                    self.needs_recovery = false;
                    Ok(())
                })
            }

            fn write<'a>(
                &'a mut self,
                _frame: &'a CanFdFrame,
            ) -> BoxFuture<'a, Result<()>> {
                Box::pin(async { Ok(()) })
            }

            fn read(&mut self) -> BoxFuture<'_, Result<Option<CanFdFrame>>> {
                Box::pin(async { Ok(None) })
            }

            fn flush(&mut self) -> BoxFuture<'_, Result<()>> {
                Box::pin(async { Ok(()) })
            }

            fn info(&self) -> &TransportDeviceInfo {
                &self.info
            }
        }

        /// Mock device that always completes immediately.
        struct FastMockDevice {
            info: TransportDeviceInfo,
            recover_count: Arc<AtomicUsize>,
            needs_recovery: bool,
        }

        impl FastMockDevice {
            fn with_counter(recover_count: Arc<AtomicUsize>) -> Self {
                Self {
                    info: TransportDeviceInfo::new(0, "FastMock"),
                    recover_count,
                    needs_recovery: false,
                }
            }
        }

        impl AsyncTransportDevice for FastMockDevice {
            fn recover(&mut self) -> BoxFuture<'_, Result<()>> {
                Box::pin(async move {
                    if self.needs_recovery {
                        self.recover_count.fetch_add(1, Ordering::Release);
                        self.needs_recovery = false;
                    }
                    Ok(())
                })
            }

            fn transaction<'a>(
                &'a mut self,
                _requests: &'a mut [Request],
            ) -> BoxFuture<'a, Result<()>> {
                Box::pin(async { Ok(()) })
            }

            fn write<'a>(
                &'a mut self,
                _frame: &'a CanFdFrame,
            ) -> BoxFuture<'a, Result<()>> {
                Box::pin(async { Ok(()) })
            }

            fn read(&mut self) -> BoxFuture<'_, Result<Option<CanFdFrame>>> {
                Box::pin(async { Ok(None) })
            }

            fn flush(&mut self) -> BoxFuture<'_, Result<()>> {
                Box::pin(async { Ok(()) })
            }

            fn info(&self) -> &TransportDeviceInfo {
                &self.info
            }
        }

        #[tokio::test]
        async fn test_single_device_survives_cancellation() {
            let fast_mode = Arc::new(AtomicBool::new(false));
            let device = CancellableMockDevice::new(fast_mode);
            let mut transport = AsyncTransport::new(vec![Box::new(device)]);

            let mut requests = vec![Request::new(CanFdFrame::new())];
            let result = tokio::time::timeout(
                Duration::from_millis(10),
                transport.cycle(&mut requests),
            )
            .await;
            assert!(result.is_err()); // timed out

            // Transport still usable — devices not lost
            assert_eq!(transport.device_count(), 1);
        }

        #[tokio::test]
        async fn test_multi_device_survives_cancellation() {
            let fast1 = Arc::new(AtomicBool::new(false));
            let fast2 = Arc::new(AtomicBool::new(false));
            let mut transport = AsyncTransport::new(vec![
                Box::new(CancellableMockDevice::new(fast1)),
                Box::new(CancellableMockDevice::new(fast2)),
            ]);
            transport.add_route(crate::DeviceAddress::can_id(1), 0).unwrap();
            transport.add_route(crate::DeviceAddress::can_id(2), 1).unwrap();

            // Build requests targeting different devices
            let mut frame1 = CanFdFrame::new();
            frame1.arbitration_id = 0x0001; // dest=1
            let mut frame2 = CanFdFrame::new();
            frame2.arbitration_id = 0x0002; // dest=2
            let mut requests = vec![Request::new(frame1), Request::new(frame2)];

            let result = tokio::time::timeout(
                Duration::from_millis(10),
                transport.cycle(&mut requests),
            )
            .await;
            assert!(result.is_err()); // timed out

            // Both devices survive
            assert_eq!(transport.device_count(), 2);
        }

        #[tokio::test]
        async fn test_recover_called_after_cancellation() {
            let recover_count = Arc::new(AtomicUsize::new(0));
            let fast_mode = Arc::new(AtomicBool::new(false));
            let device = CancellableMockDevice::with_counter(
                recover_count.clone(),
                fast_mode.clone(),
            );
            let mut transport = AsyncTransport::new(vec![Box::new(device)]);

            // Cancel a cycle
            let mut requests = vec![Request::new(CanFdFrame::new()).with_expected_replies(0)];
            let _ = tokio::time::timeout(
                Duration::from_millis(10),
                transport.cycle(&mut requests),
            )
            .await;

            // Switch to fast mode so next cycle completes
            fast_mode.store(true, Ordering::Release);

            // Run another cycle — recover() should be called with
            // needs_recovery=true
            let mut requests = vec![Request::new(CanFdFrame::new()).with_expected_replies(0)];
            transport.cycle(&mut requests).await.unwrap();

            assert!(recover_count.load(Ordering::Acquire) > 0);
        }

        #[tokio::test]
        async fn test_recover_noop_without_cancellation() {
            let recover_count = Arc::new(AtomicUsize::new(0));
            let device = FastMockDevice::with_counter(recover_count.clone());
            let mut transport = AsyncTransport::new(vec![Box::new(device)]);

            // Run two successful cycles
            let mut requests = vec![Request::new(CanFdFrame::new()).with_expected_replies(0)];
            transport.cycle(&mut requests).await.unwrap();
            let mut requests = vec![Request::new(CanFdFrame::new()).with_expected_replies(0)];
            transport.cycle(&mut requests).await.unwrap();

            // recover() was called but needs_recovery was false, so
            // recover_count should be 0
            assert_eq!(recover_count.load(Ordering::Acquire), 0);
        }
    }
}
