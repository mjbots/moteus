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

//! Payload framing for the tunneled diagnostic stream.
//!
//! The diagnostic stream is a bidirectional byte stream tunneled in
//! CAN frame payloads using the `0x40`-series multiplex subframes.
//! This module encodes and decodes those payloads; arbitration ID
//! handling and transport concerns live in the `moteus` crate.
//!
//! Two poll variants exist:
//!
//! - [`write_client_poll`] / [`parse_server_to_client`]: the original
//!   protocol.  Polling is destructive — data is consumed from the
//!   device as soon as it is sent, so a lost response loses data.
//! - [`write_client_poll_flow`] / [`parse_server_to_client_flow`]:
//!   flow-controlled variant for lossy transports such as UART.  Each
//!   server response carries a packet number; the client acknowledges
//!   the last packet number it received in each poll, allowing the
//!   server to retransmit unacknowledged data.

use crate::multiplex::{
    read_varuint_slice, CLIENT_POLL_SERVER, CLIENT_POLL_SERVER_FLOW, CLIENT_TO_SERVER,
    SERVER_TO_CLIENT, SERVER_TO_CLIENT_FLOW,
};

/// Maximum data bytes in a single client-to-server write subframe:
/// a 64-byte CAN-FD payload minus the 3-byte header (action, channel,
/// length).
pub const MAX_WRITE: usize = 61;

/// Writes a `CLIENT_TO_SERVER` subframe (action, channel, length,
/// data) into `out`, returning the number of bytes written.
///
/// # Panics
///
/// Panics if `data` exceeds [`MAX_WRITE`] bytes.
pub fn write_client_to_server(channel: u8, data: &[u8], out: &mut [u8; 64]) -> u8 {
    assert!(data.len() <= MAX_WRITE);

    out[0] = CLIENT_TO_SERVER;
    out[1] = channel;
    out[2] = data.len() as u8;
    out[3..3 + data.len()].copy_from_slice(data);
    (3 + data.len()) as u8
}

/// Writes a `CLIENT_POLL_SERVER` subframe (action, channel,
/// max_length) into `out`, returning the number of bytes written.
pub fn write_client_poll(channel: u8, max_length: u8, out: &mut [u8; 64]) -> u8 {
    out[0] = CLIENT_POLL_SERVER;
    out[1] = channel;
    out[2] = max_length;
    3
}

/// Writes a `CLIENT_POLL_SERVER_FLOW` subframe (action, channel,
/// packet_number, max_length) into `out`, returning the number of
/// bytes written.
///
/// `packet_number` acknowledges the last packet number received from
/// the server on this channel.
pub fn write_client_poll_flow(
    channel: u8,
    packet_number: u8,
    max_length: u8,
    out: &mut [u8; 64],
) -> u8 {
    out[0] = CLIENT_POLL_SERVER_FLOW;
    out[1] = channel;
    out[2] = packet_number;
    out[3] = max_length;
    4
}

/// Parses a `SERVER_TO_CLIENT` subframe, returning the tunneled data
/// if the payload is a well-formed response for `channel`.
pub fn parse_server_to_client(payload: &[u8], channel: u8) -> Option<&[u8]> {
    let (action, rest) = payload.split_first()?;
    if *action != SERVER_TO_CLIENT {
        return None;
    }
    let (msg_channel, rest) = rest.split_first()?;
    if *msg_channel != channel {
        return None;
    }
    let (len, rest) = read_varuint_slice(rest)?;
    rest.get(..len as usize)
}

/// A parsed flow-controlled diagnostic response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FlowData<'a> {
    /// The packet number of this response, to be acknowledged in the
    /// next poll.  Wraps from 255 to 0.
    pub packet_number: u8,
    /// The tunneled data (may be empty).
    pub data: &'a [u8],
}

/// Parses a `SERVER_TO_CLIENT_FLOW` subframe (action, channel,
/// packet_number, varuint length, data), returning the packet number
/// and tunneled data if the payload is a well-formed flow response for
/// `channel`.
pub fn parse_server_to_client_flow(payload: &[u8], channel: u8) -> Option<FlowData<'_>> {
    let (action, rest) = payload.split_first()?;
    if *action != SERVER_TO_CLIENT_FLOW {
        return None;
    }
    let (msg_channel, rest) = rest.split_first()?;
    if *msg_channel != channel {
        return None;
    }
    let (packet_number, rest) = rest.split_first()?;
    let (len, rest) = read_varuint_slice(rest)?;
    Some(FlowData {
        packet_number: *packet_number,
        data: rest.get(..len as usize)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_write_client_to_server() {
        let mut out = [0u8; 64];
        let size = write_client_to_server(1, b"hello", &mut out);
        assert_eq!(size, 8);
        assert_eq!(&out[..8], &[0x40, 0x01, 0x05, b'h', b'e', b'l', b'l', b'o']);
    }

    #[test]
    fn test_write_client_poll() {
        let mut out = [0u8; 64];
        let size = write_client_poll(1, 48, &mut out);
        assert_eq!(size, 3);
        assert_eq!(&out[..3], &[0x42, 0x01, 0x30]);
    }

    #[test]
    fn test_write_client_poll_flow() {
        // Matches C++ DiagnosticReadFlow::Make: action, channel,
        // packet_number, max_length.
        let mut out = [0u8; 64];
        let size = write_client_poll_flow(1, 0xAB, 48, &mut out);
        assert_eq!(size, 4);
        assert_eq!(&out[..4], &[0x44, 0x01, 0xAB, 0x30]);
    }

    #[test]
    fn test_parse_server_to_client() {
        assert_eq!(
            parse_server_to_client(&[0x41, 0x01, 0x03, b'a', b'b', b'c'], 1),
            Some(b"abc".as_slice())
        );

        // Empty data is valid.
        assert_eq!(
            parse_server_to_client(&[0x41, 0x01, 0x00], 1),
            Some(b"".as_slice())
        );

        // Wrong channel, wrong action, truncated data.
        assert_eq!(parse_server_to_client(&[0x41, 0x02, 0x01, b'a'], 1), None);
        assert_eq!(parse_server_to_client(&[0x43, 0x01, 0x01, b'a'], 1), None);
        assert_eq!(parse_server_to_client(&[0x41, 0x01, 0x05, b'a'], 1), None);
        assert_eq!(parse_server_to_client(&[], 1), None);
    }

    #[test]
    fn test_parse_server_to_client_flow() {
        let result =
            parse_server_to_client_flow(&[0x43, 0x01, 0x07, 0x03, b'a', b'b', b'c'], 1).unwrap();
        assert_eq!(result.packet_number, 0x07);
        assert_eq!(result.data, b"abc");

        // Empty data still carries a packet number.
        let result = parse_server_to_client_flow(&[0x43, 0x01, 0xFF, 0x00], 1).unwrap();
        assert_eq!(result.packet_number, 0xFF);
        assert_eq!(result.data, b"");

        // A plain response is not a flow response.
        assert_eq!(
            parse_server_to_client_flow(&[0x41, 0x01, 0x01, b'a'], 1),
            None
        );
        // Wrong channel.
        assert_eq!(
            parse_server_to_client_flow(&[0x43, 0x02, 0x07, 0x00], 1),
            None
        );
        // Truncated.
        assert_eq!(parse_server_to_client_flow(&[0x43, 0x01, 0x07], 1), None);
        assert_eq!(
            parse_server_to_client_flow(&[0x43, 0x01, 0x07, 0x05, b'a'], 1),
            None
        );
    }

    #[test]
    fn test_round_trip_via_write_combiner_sizes() {
        // A poll followed by parse of a synthesized response.
        let mut out = [0u8; 64];
        let size = write_client_poll_flow(2, 5, 10, &mut out) as usize;
        assert_eq!(&out[..size], &[0x44, 0x02, 0x05, 0x0A]);

        let response = [0x43, 0x02, 0x06, 0x02, 0x11, 0x22];
        let parsed = parse_server_to_client_flow(&response, 2).unwrap();
        assert_eq!(parsed.packet_number, 6);
        assert_eq!(parsed.data, &[0x11, 0x22]);
    }

    #[test]
    fn test_read_varuint_multibyte() {
        // Lengths >= 0x80 use a multi-byte varuint.
        let (value, rest) = read_varuint_slice(&[0x80, 0x01, 0xAA]).unwrap();
        assert_eq!(value, 128);
        assert_eq!(rest, &[0xAA]);

        // Unterminated varuint.
        assert!(read_varuint_slice(&[0x80, 0x80, 0x80, 0x80, 0x80]).is_none());
    }
}
