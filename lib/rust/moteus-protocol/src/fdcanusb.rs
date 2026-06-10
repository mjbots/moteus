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

//! Allocation-free codec for the fdcanusb text line protocol.
//!
//! The fdcanusb USB-to-CAN-FD adapter uses a simple line-oriented text
//! protocol:
//!
//! - Send: `can send <id-hex> <data-hex> <flags>\n`
//! - Receive: `rcv <id-hex> <data-hex> <flags>`
//! - `OK` acknowledgment after each send, `ERR ...` on failure
//!
//! moteus controllers can also speak this protocol directly over a TTL
//! UART (see `docs/integration/uart.md`), optionally protected by a
//! CRC-8 checksum appended to each line as ` *XX`.
//!
//! This module contains the stateless parts of that protocol — line
//! encoding, parsing, and checksum computation — in a `no_std`,
//! allocation-free form suitable for embedded hosts.  Buffers are
//! provided by the caller; [`MAX_LINE_LENGTH`] bounds the worst-case
//! line size.  The stateful pieces (timeouts, retries, and the policy
//! for when to enable checksums) live in the `moteus` crate's
//! transports.

use crate::frame::CanFdFrame;

/// The maximum length of an encoded or received protocol line.
///
/// The worst case is a `can send` line carrying a 64-byte CAN-FD
/// payload addressed with a 29-bit extended arbitration ID (which is
/// used whenever a CAN prefix is configured):
///
/// `can send ` (9) + 8 id digits + space + 128 data digits + space +
/// `EBF` flags (3) + ` *XX` checksum (4) + newline = 155 bytes.
pub const MAX_LINE_LENGTH: usize = 160;

/// CRC-8 lookup table for polynomial 0x97, indexed by nybble.
const CRC8_TABLE: [u8; 16] = [
    0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb, //
    0x5d, 0xca, 0xe4, 0x73, 0xb8, 0x2f, 0x01, 0x96,
];

/// Computes the CRC-8 of `data` using polynomial 0x97, nybble-at-a-time.
///
/// This is the checksum used by the fdcanusb text protocol when
/// communicating with a moteus controller over UART.
pub const fn compute_crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    let mut i = 0;
    while i < data.len() {
        let b = data[i];
        crc = CRC8_TABLE[((crc >> 4) ^ (b >> 4)) as usize & 0x0f] ^ (crc << 4);
        crc = CRC8_TABLE[((crc >> 4) ^ (b & 0x0f)) as usize & 0x0f] ^ (crc << 4);
        i += 1;
    }
    crc
}

/// Options controlling [`encode_can_send`].
#[derive(Debug, Clone, Copy, Default)]
pub struct EncodeOptions {
    /// Force bit rate switching off even if the frame requests it.
    pub disable_brs: bool,
    /// Append a ` *XX` CRC-8 checksum to the line.
    pub checksum: bool,
}

/// Errors from [`encode_can_send`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// The provided buffer was too small.  A buffer of
    /// [`MAX_LINE_LENGTH`] bytes is always sufficient.
    BufferTooSmall,
}

/// One parsed line of the fdcanusb protocol, as classified by
/// [`parse_line`].
#[derive(Debug, Clone, PartialEq)]
pub enum Line<'a> {
    /// An `OK` acknowledgment.
    Ok,
    /// A received CAN frame (`rcv ...`).
    Rcv(CanFdFrame),
    /// An error report from the device (`ERR ...`).
    Err(&'a [u8]),
    /// Anything else, including empty and malformed `rcv` lines.
    Other(&'a [u8]),
}

struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl Writer<'_> {
    fn write(&mut self, data: &[u8]) -> Result<(), EncodeError> {
        if self.pos + data.len() > self.buf.len() {
            return Err(EncodeError::BufferTooSmall);
        }
        self.buf[self.pos..self.pos + data.len()].copy_from_slice(data);
        self.pos += data.len();
        Ok(())
    }

    /// Writes a byte as two uppercase hex digits.
    fn write_hex_byte(&mut self, value: u8) -> Result<(), EncodeError> {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        self.write(&[HEX[(value >> 4) as usize], HEX[(value & 0x0f) as usize]])
    }

    /// Writes an arbitration ID as lowercase hex with a minimum width
    /// of 4 digits.  29-bit extended IDs (used when a CAN prefix is
    /// configured) produce up to 8 digits.
    fn write_hex_id(&mut self, value: u32) -> Result<(), EncodeError> {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let digits = {
            let mut n = 4;
            while n < 8 && (value >> (4 * n)) != 0 {
                n += 1;
            }
            n
        };
        for i in (0..digits).rev() {
            self.write(&[HEX[((value >> (4 * i)) & 0x0f) as usize]])?;
        }
        Ok(())
    }
}

/// Encodes a CAN-FD frame as a `can send` line into `buf`, returning
/// the encoded length.
///
/// The payload is padded to a valid CAN-FD DLC size with 0x50 (NOP)
/// bytes.  A buffer of [`MAX_LINE_LENGTH`] bytes is always large
/// enough.
pub fn encode_can_send(
    frame: &CanFdFrame,
    options: &EncodeOptions,
    buf: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut w = Writer { buf, pos: 0 };

    w.write(b"can send ")?;
    w.write_hex_id(frame.arbitration_id)?;
    w.write(b" ")?;

    for b in frame.payload() {
        w.write_hex_byte(*b)?;
    }
    // Pad to a valid CAN-FD DLC size.
    let on_wire_size = CanFdFrame::round_up_dlc(frame.size as usize);
    for _ in frame.size as usize..on_wire_size {
        w.write(b"50")?;
    }

    w.write(b" ")?;
    if frame.brs_enabled() && !options.disable_brs {
        w.write(b"B")?;
    } else {
        w.write(b"b")?;
    }
    if frame.fdcan_enabled() {
        w.write(b"F")?;
    }

    if options.checksum {
        // The CRC covers the line content including the space that
        // separates it from the '*'.
        w.write(b" ")?;
        let crc = compute_crc8(&w.buf[..w.pos]);
        w.write(b"*")?;
        w.write_hex_byte(crc)?;
    }

    w.write(b"\n")?;

    Ok(w.pos)
}

/// Strips and validates a trailing ` *XX` checksum from a line.
///
/// Returns `(content, had_checksum, checksum_valid)`:
/// - `content`: the line with the checksum and any trailing spaces
///   removed (unchanged if no checksum was found)
/// - `had_checksum`: a `*XX` pattern was found at the end of the line
/// - `checksum_valid`: the checksum matched (only meaningful when
///   `had_checksum`)
///
/// The CRC is computed over everything before the `*`, including any
/// trailing space.
pub fn strip_checksum(line: &[u8]) -> (&[u8], bool, bool) {
    // The checksum must be exactly '*' followed by two hex digits at
    // the very end of the line.
    if line.len() < 3 || line[line.len() - 3] != b'*' {
        return (line, false, false);
    }

    let star_pos = line.len() - 3;
    let claimed = {
        let hi = parse_hex_digit(line[star_pos + 1]);
        let lo = parse_hex_digit(line[star_pos + 2]);
        match (hi, lo) {
            (Some(hi), Some(lo)) => (hi << 4) | lo,
            _ => return (line, false, false),
        }
    };

    let computed = compute_crc8(&line[..star_pos]);

    let mut content = &line[..star_pos];
    while let [head @ .., b' '] = content {
        content = head;
    }

    (content, true, claimed == computed)
}

/// Classifies one checksum-stripped protocol line.
///
/// Callers are responsible for line assembly (splitting the input on
/// `\r`/`\n`) and for checksum handling via [`strip_checksum`].
pub fn parse_line(line: &[u8]) -> Line<'_> {
    let line = trim(line);

    if line.starts_with(b"OK") {
        return Line::Ok;
    }
    if line.starts_with(b"rcv") {
        if let Some(frame) = parse_rcv(line) {
            return Line::Rcv(frame);
        }
        return Line::Other(line);
    }
    if line.starts_with(b"ERR") {
        return Line::Err(line);
    }
    Line::Other(line)
}

/// Parses a received frame line: `rcv <id-hex> <data-hex> [E] [B] [F]`.
///
/// When the device responds with no data, the data field is empty
/// (`rcv 0100  e B F`), so fields are split on *single* spaces and
/// empty fields are preserved.
///
/// Returns `None` if the line is not a well-formed `rcv` line.
pub fn parse_rcv(line: &[u8]) -> Option<CanFdFrame> {
    let line = trim(line);
    let mut fields = line.split(|&b| b == b' ');

    if fields.next() != Some(b"rcv".as_slice()) {
        return None;
    }

    let arbitration_id = parse_hex_u32(fields.next()?)?;

    let mut frame = CanFdFrame::new();
    frame.arbitration_id = arbitration_id;

    let data = fields.next()?;
    if data.len() % 2 != 0 || data.len() > 2 * frame.data.len() {
        return None;
    }
    for (i, pair) in data.chunks_exact(2).enumerate() {
        frame.data[i] = ((parse_hex_digit(pair[0])? as u8) << 4) | parse_hex_digit(pair[1])? as u8;
    }
    frame.size = (data.len() / 2) as u8;

    for flag in fields {
        match flag {
            b"E" => frame.arbitration_id |= 0x80000000, // Extended ID marker
            b"B" => frame.set_brs(true),
            b"F" => frame.set_fdcan(true),
            _ => {}
        }
    }

    Some(frame)
}

/// Returns true if an `ERR` line indicates that the device requires
/// checksums (case-insensitive search for "checksum").
pub fn is_checksum_error(line: &[u8]) -> bool {
    const NEEDLE: &[u8] = b"checksum";
    line.windows(NEEDLE.len())
        .any(|w| w.eq_ignore_ascii_case(NEEDLE))
}

fn trim(mut line: &[u8]) -> &[u8] {
    while let [b, rest @ ..] = line {
        if b.is_ascii_whitespace() {
            line = rest;
        } else {
            break;
        }
    }
    while let [rest @ .., b] = line {
        if b.is_ascii_whitespace() {
            line = rest;
        } else {
            break;
        }
    }
    line
}

fn parse_hex_digit(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

fn parse_hex_u32(digits: &[u8]) -> Option<u32> {
    if digits.is_empty() || digits.len() > 8 {
        return None;
    }
    let mut value = 0u32;
    for &c in digits {
        value = (value << 4) | parse_hex_digit(c)? as u32;
    }
    Some(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Test vectors verified against the python crcmod package; they
    // match lib/cpp/mjbots/moteus/test/moteus_test.cc and
    // lib/python/moteus/test/fdcanusb_test.py.
    #[test]
    fn test_compute_crc8() {
        assert_eq!(compute_crc8(b""), 0x00);
        assert_eq!(compute_crc8(b"OK "), 0xBD);
        assert_eq!(compute_crc8(b"can send 0001 20 "), 0xAE);
        assert_eq!(compute_crc8(b"rcv 0502 1234 "), 0x3C);
        assert_eq!(compute_crc8(b"rcv 00000100 230b0a00 "), 0xD2);

        // Different data produces different CRCs.
        assert_eq!(compute_crc8(b"rcv 00000100 00 "), 0x3E);
        assert_eq!(compute_crc8(b"rcv 00000100 01 "), 0xED);
        assert_eq!(compute_crc8(b"rcv 00000100 ff "), 0x57);
    }

    fn make_frame(id: u32, data: &[u8]) -> CanFdFrame {
        let mut frame = CanFdFrame::new();
        frame.arbitration_id = id;
        frame.data[..data.len()].copy_from_slice(data);
        frame.size = data.len() as u8;
        frame
    }

    fn encode<'a>(frame: &CanFdFrame, options: &EncodeOptions, buf: &'a mut [u8]) -> &'a [u8] {
        let len = encode_can_send(frame, options, buf).unwrap();
        &buf[..len]
    }

    #[test]
    fn test_encode_basic() {
        let frame = make_frame(0x8001, &[0x01, 0x00, 0x0A]);
        let mut buf = [0u8; MAX_LINE_LENGTH];
        assert_eq!(
            encode(&frame, &EncodeOptions::default(), &mut buf),
            b"can send 8001 01000A BF\n"
        );
    }

    #[test]
    fn test_encode_dlc_padding() {
        // Sizes above 8 bytes round up to the next valid CAN-FD DLC,
        // padded with 0x50 (NOP) bytes.
        let frame = make_frame(0x0001, &[0x11; 9]);
        let mut buf = [0u8; MAX_LINE_LENGTH];
        assert_eq!(
            encode(&frame, &EncodeOptions::default(), &mut buf),
            b"can send 0001 111111111111111111505050 BF\n"
        );
    }

    #[test]
    fn test_encode_disable_brs() {
        let frame = make_frame(0x8001, &[0x01]);
        let mut buf = [0u8; MAX_LINE_LENGTH];
        assert_eq!(
            encode(
                &frame,
                &EncodeOptions {
                    disable_brs: true,
                    ..Default::default()
                },
                &mut buf
            ),
            b"can send 8001 01 bF\n"
        );
    }

    #[test]
    fn test_encode_checksum() {
        let frame = make_frame(0x0001, &[0x20]);
        let mut buf = [0u8; MAX_LINE_LENGTH];
        let line = encode(
            &frame,
            &EncodeOptions {
                disable_brs: true,
                checksum: true,
            },
            &mut buf,
        );
        // The line ends with " *XX\n" where the CRC covers everything
        // before the '*', including the trailing space.
        assert!(line.starts_with(b"can send 0001 20 bF *"));
        assert!(line.ends_with(b"\n"));

        // The receive side validates and strips the checksum.
        let (content, had, valid) = strip_checksum(&line[..line.len() - 1]);
        assert_eq!(content, b"can send 0001 20 bF");
        assert!(had);
        assert!(valid);
    }

    #[test]
    fn test_encode_extended_id() {
        // With a CAN prefix configured, the arbitration ID exceeds 16
        // bits and must be emitted with more than 4 hex digits.
        let frame = make_frame(0x12345, &[0xFF]);
        let mut buf = [0u8; MAX_LINE_LENGTH];
        assert_eq!(
            encode(&frame, &EncodeOptions::default(), &mut buf),
            b"can send 12345 FF BF\n"
        );

        let frame = make_frame(0x1234_5678, &[]);
        let mut buf = [0u8; MAX_LINE_LENGTH];
        assert_eq!(
            encode(&frame, &EncodeOptions::default(), &mut buf),
            b"can send 12345678  BF\n"
        );
    }

    #[test]
    fn test_encode_buffer_too_small() {
        let frame = make_frame(0x8001, &[0x01]);
        let mut buf = [0u8; 8];
        assert_eq!(
            encode_can_send(&frame, &EncodeOptions::default(), &mut buf),
            Err(EncodeError::BufferTooSmall)
        );
    }

    #[test]
    fn test_max_line_length_is_sufficient() {
        let frame = make_frame(0x1FFF_FFFF, &[0xAB; 64]);
        let mut buf = [0u8; MAX_LINE_LENGTH];
        let len = encode_can_send(
            &frame,
            &EncodeOptions {
                disable_brs: false,
                checksum: true,
            },
            &mut buf,
        )
        .unwrap();
        assert!(len <= MAX_LINE_LENGTH);
    }

    #[test]
    fn test_strip_checksum_valid() {
        let (content, had, valid) = strip_checksum(b"OK *BD");
        assert_eq!(content, b"OK");
        assert!(had);
        assert!(valid);
    }

    #[test]
    fn test_strip_checksum_invalid() {
        let (content, had, valid) = strip_checksum(b"OK *FF");
        assert_eq!(content, b"OK");
        assert!(had);
        assert!(!valid);
    }

    #[test]
    fn test_strip_checksum_absent() {
        let (content, had, valid) = strip_checksum(b"OK");
        assert_eq!(content, b"OK");
        assert!(!had);
        assert!(!valid);

        // '*' not in the right position is not a checksum.
        let (content, had, _) = strip_checksum(b"rcv 0001 2a *1");
        assert_eq!(content, b"rcv 0001 2a *1");
        assert!(!had);

        // Non-hex digits after '*' are not a checksum.
        let (content, had, _) = strip_checksum(b"some *zz");
        assert_eq!(content, b"some *zz");
        assert!(!had);
    }

    #[test]
    fn test_parse_line() {
        assert_eq!(parse_line(b"OK"), Line::Ok);
        assert_eq!(parse_line(b"OK\r\n"), Line::Ok);
        assert!(matches!(parse_line(b"rcv 0502 1234"), Line::Rcv(_)));
        assert!(matches!(parse_line(b"ERR unknown command"), Line::Err(_)));
        assert!(matches!(parse_line(b""), Line::Other(_)));
        // Malformed rcv lines are Other, not Err.
        assert!(matches!(parse_line(b"rcv zz"), Line::Other(_)));
    }

    #[test]
    fn test_parse_rcv() {
        let frame = parse_rcv(b"rcv 8001 01000A B F").unwrap();
        assert_eq!(frame.arbitration_id, 0x8001);
        assert_eq!(frame.payload(), &[0x01, 0x00, 0x0A]);
        assert!(frame.brs_enabled());
        assert!(frame.fdcan_enabled());
    }

    #[test]
    fn test_parse_rcv_extended_id() {
        let frame = parse_rcv(b"rcv 00010001 20").unwrap();
        assert_eq!(frame.arbitration_id, 0x00010001);

        let frame = parse_rcv(b"rcv 0100 20 E").unwrap();
        assert_eq!(frame.arbitration_id, 0x80000100);
    }

    #[test]
    fn test_parse_rcv_empty_data() {
        // An empty data field is preserved by single-space splitting.
        let frame = parse_rcv(b"rcv 0100  e B F").unwrap();
        assert_eq!(frame.arbitration_id, 0x0100);
        assert_eq!(frame.size, 0);
    }

    #[test]
    fn test_parse_rcv_malformed() {
        assert!(parse_rcv(b"rcv").is_none());
        assert!(parse_rcv(b"rcv 0100").is_none());
        assert!(parse_rcv(b"rcv zz 00").is_none());
        assert!(parse_rcv(b"rcv 0100 0").is_none()); // Odd hex length
        assert!(parse_rcv(b"OK").is_none());
    }

    #[test]
    fn test_is_checksum_error() {
        assert!(is_checksum_error(b"ERR checksum required"));
        assert!(is_checksum_error(b"ERR invalid Checksum"));
        assert!(!is_checksum_error(b"ERR unknown command"));
    }

    #[test]
    fn test_encode_parse_round_trip() {
        let frame = make_frame(0x0542, &[0x12, 0x34]);
        let mut buf = [0u8; MAX_LINE_LENGTH];
        let len = encode_can_send(&frame, &EncodeOptions::default(), &mut buf).unwrap();

        // Re-interpret the encoded line as if it were received:
        // replace the "can send " prefix with "rcv " and drop the
        // trailing newline.
        let content = &buf[b"can send ".len()..len - 1];
        let mut line = [0u8; MAX_LINE_LENGTH];
        line[..4].copy_from_slice(b"rcv ");
        line[4..4 + content.len()].copy_from_slice(content);

        let parsed = parse_rcv(&line[..4 + content.len()]).unwrap();
        assert_eq!(parsed.arbitration_id, 0x0542);
        assert_eq!(parsed.payload(), &[0x12, 0x34]);
        assert!(parsed.brs_enabled());
        assert!(parsed.fdcan_enabled());
    }
}
