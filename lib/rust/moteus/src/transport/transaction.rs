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

//! Cycle request types for the moteus transport layer.
//!
//! This module defines the core types for the cycle-based transport API:
//!
//! - [`ResponseCollector`]: Thread-safe collector for response frames
//! - [`FrameFilter`]: Filter criteria for matching response frames
//! - [`Request`]: A request combining a frame, filter, and response collector
//!
//! # Example
//!
//! ```rust
//! use moteus::transport::transaction::{Request, FrameFilter, ResponseCollector};
//! use moteus_protocol::CanFdFrame;
//!
//! // Create a request with auto-filtering by destination
//! let mut frame = CanFdFrame::new();
//! frame.arbitration_id = moteus_protocol::calculate_arbitration_id(0, 1, 0, true);
//! let request = Request::new(frame);
//!
//! // After cycle, access responses
//! // let responses = request.responses.take();
//! ```

use crate::command_types::Command;
use crate::device_address::DeviceAddress;
use moteus_protocol::CanFdFrame;
use std::sync::{Arc, Mutex};

/// A thread-safe collector for response frames.
///
/// This type wraps a `Vec<CanFdFrame>` in an `Arc<Mutex<...>>` to allow
/// multiple references and thread-safe mutation. This enables subscription
/// handlers to capture and share collectors.
///
/// # Example
///
/// ```rust
/// use moteus::transport::transaction::ResponseCollector;
/// use moteus_protocol::CanFdFrame;
///
/// let collector = ResponseCollector::new();
///
/// // Push a frame
/// let frame = CanFdFrame::new();
/// collector.push(frame);
///
/// // Check length
/// assert_eq!(collector.len(), 1);
///
/// // Take collected frames
/// let frames = collector.take();
/// assert_eq!(frames.len(), 1);
/// assert!(collector.is_empty());
/// ```
#[derive(Clone, Default)]
pub struct ResponseCollector(Arc<Mutex<Vec<CanFdFrame>>>);

impl ResponseCollector {
    /// Creates a new empty response collector.
    pub fn new() -> Self {
        Self(Arc::new(Mutex::new(Vec::new())))
    }

    /// Pushes a frame into the collector.
    ///
    /// This method takes `&self` (not `&mut self`) to allow handlers to
    /// capture and use the collector without mutable access.
    pub fn push(&self, frame: CanFdFrame) {
        if let Ok(mut guard) = self.0.lock() {
            guard.push(frame);
        }
    }

    /// Takes all collected frames, leaving the collector empty.
    ///
    /// Returns the frames that have been collected so far.
    pub fn take(&self) -> Vec<CanFdFrame> {
        if let Ok(mut guard) = self.0.lock() {
            std::mem::take(&mut *guard)
        } else {
            Vec::new()
        }
    }

    /// Returns the number of collected frames.
    pub fn len(&self) -> usize {
        self.0.lock().map(|g| g.len()).unwrap_or(0)
    }

    /// Returns true if no frames have been collected.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns a copy of the collected frames without removing them.
    pub fn peek(&self) -> Vec<CanFdFrame> {
        if let Ok(guard) = self.0.lock() {
            guard.clone()
        } else {
            Vec::new()
        }
    }

    /// Clears all collected frames.
    pub fn clear(&self) {
        if let Ok(mut guard) = self.0.lock() {
            guard.clear();
        }
    }
}

impl std::fmt::Debug for ResponseCollector {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let len = self.len();
        f.debug_struct("ResponseCollector")
            .field("len", &len)
            .finish()
    }
}

/// Filter criteria for matching response frames.
///
/// This enum defines how incoming frames are matched to requests.
/// The most common case is filtering by source CAN ID.
///
/// # Example
///
/// ```rust
/// use moteus::transport::transaction::FrameFilter;
/// use moteus_protocol::CanFdFrame;
///
/// // Filter by source ID
/// let filter = FrameFilter::BySource(1);
///
/// let mut frame = CanFdFrame::new();
/// frame.arbitration_id = moteus_protocol::calculate_arbitration_id(1, 0, 0, false);
/// assert!(filter.matches(&frame));
///
/// // Accept any frame
/// let any_filter = FrameFilter::Any;
/// assert!(any_filter.matches(&frame));
/// ```
#[non_exhaustive]
#[derive(Clone, Default)]
pub enum FrameFilter {
    /// Match frames from a specific source CAN ID.
    BySource(u8),
    /// Accept all frames.
    #[default]
    Any,
    /// Custom filter function.
    Custom(Arc<dyn Fn(&CanFdFrame) -> bool + Send + Sync>),
}

impl FrameFilter {
    /// Creates a filter that matches frames from a specific source.
    pub fn by_source(source_id: u8) -> Self {
        FrameFilter::BySource(source_id)
    }

    /// Creates a filter that accepts all frames.
    pub fn any() -> Self {
        FrameFilter::Any
    }

    /// Creates a custom filter from a closure.
    pub fn custom<F>(f: F) -> Self
    where
        F: Fn(&CanFdFrame) -> bool + Send + Sync + 'static,
    {
        FrameFilter::Custom(Arc::new(f))
    }

    /// Tests whether a frame matches this filter.
    pub fn matches(&self, frame: &CanFdFrame) -> bool {
        match self {
            FrameFilter::BySource(source) => {
                // Extract source from arbitration ID (bits 14:8)
                let frame_source = ((frame.arbitration_id >> 8) & 0x7F) as u8;
                frame_source == *source
            }
            FrameFilter::Any => true,
            FrameFilter::Custom(f) => f(frame),
        }
    }
}

impl std::fmt::Debug for FrameFilter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FrameFilter::BySource(id) => write!(f, "BySource({})", id),
            FrameFilter::Any => write!(f, "Any"),
            FrameFilter::Custom(_) => write!(f, "Custom(...)"),
        }
    }
}

/// A request in a transport cycle.
///
/// A request represents a single CAN frame to send (or receive-only operation)
/// along with filtering criteria and a collector for responses. Multiple
/// requests can be batched in a single cycle.
///
/// # Example
///
/// ```rust
/// use moteus::transport::transaction::{Request, FrameFilter};
/// use moteus_protocol::CanFdFrame;
///
/// // Create a request to send a frame
/// let mut frame = CanFdFrame::new();
/// frame.arbitration_id = moteus_protocol::calculate_arbitration_id(0, 1, 0, true);
/// let request = Request::new(frame);
///
/// // Create a receive-only request
/// let recv_request = Request::receive_only(FrameFilter::by_source(1));
///
/// // Use builder pattern to customize
/// let mut frame2 = CanFdFrame::new();
/// frame2.arbitration_id = moteus_protocol::calculate_arbitration_id(0, 2, 0, true);
/// let request = Request::new(frame2)
///     .with_channel(1)                // Route to channel 1
///     .with_expected_replies(2);      // Expect 2 responses
/// ```
#[non_exhaustive]
#[derive(Clone)]
pub struct Request {
    /// The frame to send, or None for receive-only.
    pub frame: Option<CanFdFrame>,
    /// The channel index for multi-channel routing.
    pub channel: Option<usize>,
    /// Filter for matching response frames.
    pub filter: FrameFilter,
    /// Number of expected replies for this request.
    pub expected_reply_count: u8,
    /// Expected size of the reply payload in bytes.
    ///
    /// This is informational — transport implementations may use it for
    /// buffer sizing or timeout strategies. Set from
    /// `Command.expected_reply_size` by [`Request::from_command()`].
    pub expected_reply_size: u8,
    /// Child device index for parent dispatch.
    ///
    /// When a request is routed through a parent device, this field
    /// indicates which child device the request is intended for.
    /// Set automatically by the Transport when dispatching to a parent.
    pub child_device: Option<usize>,
    /// Device address for routing table lookup.
    ///
    /// When set, the transport can use the UUID in this address to
    /// route directly to a single device instead of sending to all
    /// devices sharing the same CAN ID.
    pub address: Option<DeviceAddress>,
    /// Collector for response frames.
    pub responses: ResponseCollector,
}

impl Request {
    /// Creates a new request with a frame to send.
    ///
    /// The filter is automatically set to match responses from the
    /// frame's destination (which becomes the source in responses).
    pub fn new(frame: CanFdFrame) -> Self {
        // Extract destination from arbitration ID (bits 7:0)
        let dest = (frame.arbitration_id & 0x7F) as u8;
        // For broadcast (0x7F), accept responses from any source.
        // This matches the Python filter which skips the source check
        // when dest_id == 0x7f.
        let filter = if dest == 0x7F {
            FrameFilter::Any
        } else {
            FrameFilter::BySource(dest)
        };

        let expected = if frame.arbitration_id & 0x8000 != 0 {
            1
        } else {
            0
        };

        Self {
            frame: Some(frame),
            channel: None,
            filter,
            expected_reply_count: expected,
            expected_reply_size: 0,
            child_device: None,
            address: None,
            responses: ResponseCollector::new(),
        }
    }

    /// Creates a request from a `Command`, building a compound filter
    /// matching Python's `Transport._make_response_filter`.
    ///
    /// The filter checks four conditions (matching the Python transport):
    /// 1. CAN prefix matches (bits 28:16)
    /// 2. Source ID matches command destination (bits 14:8) — unless broadcast 0x7F
    /// 3. Destination matches command source (bits 6:0)
    /// 4. `command.reply_filter` content-level check (if set)
    ///
    /// For non-reply commands, sets `expected_reply_count: 0` with `FrameFilter::Any`.
    pub fn from_command(cmd: Command) -> Self {
        let channel = cmd.channel;
        let address = cmd.address.clone();
        let expected_reply_size = cmd.expected_reply_size;

        if !cmd.reply_required {
            let frame = cmd.into_frame();
            return Self {
                frame: Some(frame),
                channel,
                filter: FrameFilter::Any,
                expected_reply_count: 0,
                expected_reply_size,
                child_device: None,
                address,
                responses: ResponseCollector::new(),
            };
        }

        if cmd.raw {
            let frame = cmd.into_frame();
            return Self {
                frame: Some(frame),
                channel,
                filter: FrameFilter::Any,
                expected_reply_count: 1,
                expected_reply_size,
                child_device: None,
                address,
                responses: ResponseCollector::new(),
            };
        }

        let dest_id = (cmd.destination as u8) & 0x7F;
        let source_id = (cmd.source as u8) & 0x7F;
        let prefix = cmd.can_prefix & 0x1FFF;
        let reply_filter = cmd.reply_filter.clone();

        let frame = cmd.into_frame();

        let filter = FrameFilter::custom(move |f| {
            // Check CAN prefix (bits 28:16)
            if ((f.arbitration_id >> 16) & 0x1FFF) as u16 != prefix {
                return false;
            }

            // Check source ID matches dest (bits 14:8), unless broadcast
            if dest_id != 0x7F {
                let frame_source = ((f.arbitration_id >> 8) & 0x7F) as u8;
                if frame_source != dest_id {
                    return false;
                }
            }

            // Check destination matches our source (bits 6:0)
            let frame_dest = (f.arbitration_id & 0x7F) as u8;
            if frame_dest != source_id {
                return false;
            }

            // Check content-level reply filter
            if let Some(ref rf) = reply_filter {
                if !rf.matches(f) {
                    return false;
                }
            }

            true
        });

        Self {
            frame: Some(frame),
            channel,
            filter,
            expected_reply_count: 1,
            expected_reply_size,
            child_device: None,
            address,
            responses: ResponseCollector::new(),
        }
    }

    /// Creates a receive-only request with no frame to send.
    ///
    /// This is useful for listening to traffic without sending.
    pub fn receive_only(filter: FrameFilter) -> Self {
        Self {
            frame: None,
            channel: None,
            filter,
            expected_reply_count: 1,
            expected_reply_size: 0,
            child_device: None,
            address: None,
            responses: ResponseCollector::new(),
        }
    }

    /// Sets the channel index for multi-channel routing (builder pattern).
    ///
    /// When using a transport with multiple devices, this specifies
    /// which channel (device) to use for this request.
    #[must_use]
    pub fn with_channel(mut self, idx: usize) -> Self {
        self.channel = Some(idx);
        self
    }

    /// Sets the response filter (builder pattern).
    ///
    /// Overrides the auto-detected filter based on frame destination.
    #[must_use]
    pub fn with_filter(mut self, f: FrameFilter) -> Self {
        self.filter = f;
        self
    }

    /// Sets the expected number of replies (builder pattern).
    #[must_use]
    pub fn with_expected_replies(mut self, n: u8) -> Self {
        self.expected_reply_count = n;
        self
    }

    /// Returns true if this request has a frame to send.
    pub fn has_frame(&self) -> bool {
        self.frame.is_some()
    }

    /// Returns the total number of expected replies across all requests.
    pub fn total_expected_replies(requests: &[Request]) -> usize {
        requests
            .iter()
            .map(|r| r.expected_reply_count as usize)
            .sum()
    }
}

impl std::fmt::Debug for Request {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Request")
            .field("has_frame", &self.frame.is_some())
            .field("channel", &self.channel)
            .field("address", &self.address)
            .field("filter", &self.filter)
            .field("expected_replies", &self.expected_reply_count)
            .field("responses", &self.responses)
            .finish()
    }
}

/// Dispatches a frame to the first matching request's response collector.
///
/// This is a helper function used by transport implementations to route
/// incoming frames to the appropriate request.
///
/// Returns true if the frame was dispatched, false if no filter matched.
pub fn dispatch_frame(frame: &CanFdFrame, requests: &[Request]) -> bool {
    for req in requests {
        if req.filter.matches(frame) {
            req.responses.push(frame.clone());
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_response_collector_new() {
        let collector = ResponseCollector::new();
        assert!(collector.is_empty());
        assert_eq!(collector.len(), 0);
    }

    #[test]
    fn test_response_collector_push_and_take() {
        let collector = ResponseCollector::new();

        let mut frame1 = CanFdFrame::new();
        frame1.arbitration_id = 0x100;
        let mut frame2 = CanFdFrame::new();
        frame2.arbitration_id = 0x200;

        collector.push(frame1);
        collector.push(frame2);

        assert_eq!(collector.len(), 2);
        assert!(!collector.is_empty());

        let frames = collector.take();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].arbitration_id, 0x100);
        assert_eq!(frames[1].arbitration_id, 0x200);

        assert!(collector.is_empty());
    }

    #[test]
    fn test_response_collector_clone() {
        let collector = ResponseCollector::new();

        let mut frame = CanFdFrame::new();
        frame.arbitration_id = 0x100;
        collector.push(frame);

        // Clone should share the same underlying storage
        let clone = collector.clone();
        assert_eq!(clone.len(), 1);

        let mut frame2 = CanFdFrame::new();
        frame2.arbitration_id = 0x200;
        clone.push(frame2);

        // Both should see the new frame
        assert_eq!(collector.len(), 2);
        assert_eq!(clone.len(), 2);
    }

    #[test]
    fn test_response_collector_peek() {
        let collector = ResponseCollector::new();

        let mut frame = CanFdFrame::new();
        frame.arbitration_id = 0x100;
        collector.push(frame);

        let frames = collector.peek();
        assert_eq!(frames.len(), 1);

        // peek should not remove frames
        assert_eq!(collector.len(), 1);
    }

    #[test]
    fn test_response_collector_clear() {
        let collector = ResponseCollector::new();

        collector.push(CanFdFrame::new());
        collector.push(CanFdFrame::new());
        assert_eq!(collector.len(), 2);

        collector.clear();
        assert!(collector.is_empty());
    }

    #[test]
    fn test_frame_filter_by_source() {
        let filter = FrameFilter::BySource(5);

        // Source ID is in bits 14:8 of arbitration_id
        let mut matching = CanFdFrame::new();
        matching.arbitration_id = 0x0500; // Source = 5, Dest = 0

        let mut non_matching = CanFdFrame::new();
        non_matching.arbitration_id = 0x0600; // Source = 6, Dest = 0

        assert!(filter.matches(&matching));
        assert!(!filter.matches(&non_matching));
    }

    #[test]
    fn test_frame_filter_any() {
        let filter = FrameFilter::Any;

        let mut frame1 = CanFdFrame::new();
        frame1.arbitration_id = 0x100;

        let mut frame2 = CanFdFrame::new();
        frame2.arbitration_id = 0x200;

        assert!(filter.matches(&frame1));
        assert!(filter.matches(&frame2));
    }

    #[test]
    fn test_frame_filter_custom() {
        // Filter that only accepts even arbitration IDs
        let filter = FrameFilter::custom(|f| f.arbitration_id % 2 == 0);

        let mut even = CanFdFrame::new();
        even.arbitration_id = 0x100;

        let mut odd = CanFdFrame::new();
        odd.arbitration_id = 0x101;

        assert!(filter.matches(&even));
        assert!(!filter.matches(&odd));
    }

    #[test]
    fn test_request_new() {
        let mut frame = CanFdFrame::new();
        frame.arbitration_id = moteus_protocol::calculate_arbitration_id(0, 5, 0, true);

        let request = Request::new(frame);

        assert!(request.has_frame());
        assert!(request.channel.is_none());
        assert_eq!(request.expected_reply_count, 1);
        assert!(request.responses.is_empty());

        // Filter should match source=5 (destination becomes source in response)
        let mut response = CanFdFrame::new();
        response.arbitration_id = 0x0500; // source=5, dest=0
        assert!(request.filter.matches(&response));
    }

    #[test]
    fn test_request_receive_only() {
        let request = Request::receive_only(FrameFilter::BySource(3));

        assert!(!request.has_frame());
        assert_eq!(request.expected_reply_count, 1);
    }

    #[test]
    fn test_request_builder_pattern() {
        let mut frame = CanFdFrame::new();
        frame.arbitration_id = moteus_protocol::calculate_arbitration_id(0, 1, 0, true);

        let request = Request::new(frame)
            .with_channel(2)
            .with_filter(FrameFilter::Any)
            .with_expected_replies(3);

        assert_eq!(request.channel, Some(2));
        assert_eq!(request.expected_reply_count, 3);
        assert!(matches!(request.filter, FrameFilter::Any));
    }

    #[test]
    fn test_total_expected_replies() {
        let requests = vec![
            Request::receive_only(FrameFilter::Any).with_expected_replies(2),
            Request::receive_only(FrameFilter::Any).with_expected_replies(3),
            Request::receive_only(FrameFilter::Any).with_expected_replies(1),
        ];

        assert_eq!(Request::total_expected_replies(&requests), 6);
    }

    #[test]
    fn test_dispatch_frame() {
        let req1 = Request::receive_only(FrameFilter::BySource(1));
        let req2 = Request::receive_only(FrameFilter::BySource(2));
        let requests = vec![req1.clone(), req2.clone()];

        let mut frame1 = CanFdFrame::new();
        frame1.arbitration_id = 0x0100; // Source = 1

        let mut frame2 = CanFdFrame::new();
        frame2.arbitration_id = 0x0200; // Source = 2

        let mut frame3 = CanFdFrame::new();
        frame3.arbitration_id = 0x0300; // Source = 3 (no match)

        assert!(dispatch_frame(&frame1, &requests));
        assert!(dispatch_frame(&frame2, &requests));
        assert!(!dispatch_frame(&frame3, &requests));

        // Since we cloned, check the clones have the frames
        // (dispatch_frame uses the requests slice, not our clones)
        assert_eq!(requests[0].responses.len(), 1);
        assert_eq!(requests[1].responses.len(), 1);
    }

    #[test]
    fn test_frame_filter_debug() {
        let f1 = FrameFilter::BySource(5);
        let f2 = FrameFilter::Any;
        let f3 = FrameFilter::custom(|_| true);

        assert_eq!(format!("{:?}", f1), "BySource(5)");
        assert_eq!(format!("{:?}", f2), "Any");
        assert_eq!(format!("{:?}", f3), "Custom(...)");
    }

    #[test]
    fn test_request_no_reply_expected() {
        let mut frame = CanFdFrame::new();
        frame.arbitration_id = moteus_protocol::calculate_arbitration_id(0, 1, 0, false);

        let request = Request::new(frame);
        assert_eq!(request.expected_reply_count, 0);
    }

    #[test]
    fn test_response_collector_thread_safety() {
        use std::thread;

        let collector = ResponseCollector::new();
        let collector_clone = collector.clone();

        let handle = thread::spawn(move || {
            for i in 0..100 {
                let mut frame = CanFdFrame::new();
                frame.arbitration_id = i;
                collector_clone.push(frame);
            }
        });

        for i in 100..200 {
            let mut frame = CanFdFrame::new();
            frame.arbitration_id = i;
            collector.push(frame);
        }

        handle.join().unwrap();

        assert_eq!(collector.len(), 200);
    }
}
