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

//! Multiplex protocol encoding and decoding.
//!
//! The moteus multiplex protocol is a register-based protocol that runs over
//! CAN-FD. It supports reading and writing multiple registers in a single
//! frame using efficient variable-length encoding.

use crate::frame::CanFdFrame;
use crate::resolution::Resolution;
use crate::scaling::{
    self, saturate_i8, saturate_i16, saturate_i32, Scaling,
};

/// Write Int8 values
pub const WRITE_INT8: u8 = 0x00;
/// Write Int16 values
pub const WRITE_INT16: u8 = 0x04;
/// Write Int32 values
pub const WRITE_INT32: u8 = 0x08;
/// Write Float values
pub const WRITE_FLOAT: u8 = 0x0c;

/// Read Int8 values
pub const READ_INT8: u8 = 0x10;
/// Read Int16 values
pub const READ_INT16: u8 = 0x14;
/// Read Int32 values
pub const READ_INT32: u8 = 0x18;
/// Read Float values
pub const READ_FLOAT: u8 = 0x1c;

/// Reply Int8 values
pub const REPLY_INT8: u8 = 0x20;
/// Reply Int16 values
pub const REPLY_INT16: u8 = 0x24;
/// Reply Int32 values
pub const REPLY_INT32: u8 = 0x28;
/// Reply Float values
pub const REPLY_FLOAT: u8 = 0x2c;

/// Write error
pub const WRITE_ERROR: u8 = 0x30;
/// Read error
pub const READ_ERROR: u8 = 0x31;

/// Tunneled stream: client to server
pub const CLIENT_TO_SERVER: u8 = 0x40;
/// Tunneled stream: server to client
pub const SERVER_TO_CLIENT: u8 = 0x41;
/// Tunneled stream: client poll server
pub const CLIENT_POLL_SERVER: u8 = 0x42;

/// No operation
pub const NOP: u8 = 0x50;

/// Writer for appending data to a CAN-FD frame.
pub struct WriteCanData<'a> {
    data: &'a mut [u8; 64],
    size: &'a mut u8,
}

impl<'a> WriteCanData<'a> {
    /// Creates a new writer from a frame.
    pub fn new(frame: &'a mut CanFdFrame) -> Self {
        WriteCanData {
            data: &mut frame.data,
            size: &mut frame.size,
        }
    }

    /// Creates a writer from raw pointers (for compatibility).
    pub fn from_parts(data: &'a mut [u8; 64], size: &'a mut u8) -> Self {
        WriteCanData { data, size }
    }

    /// Returns the current size of written data.
    pub fn size(&self) -> u8 {
        *self.size
    }

    /// Returns the remaining capacity.
    pub fn remaining(&self) -> usize {
        64 - *self.size as usize
    }

    /// Writes a single byte.
    pub fn write_u8(&mut self, value: u8) {
        if (*self.size as usize) < 64 {
            self.data[*self.size as usize] = value;
            *self.size += 1;
        }
    }

    /// Writes a signed byte.
    pub fn write_i8(&mut self, value: i8) {
        self.write_u8(value as u8);
    }

    /// Writes a little-endian i16.
    pub fn write_i16(&mut self, value: i16) {
        let bytes = value.to_le_bytes();
        self.write_bytes(&bytes);
    }

    /// Writes a little-endian i32.
    pub fn write_i32(&mut self, value: i32) {
        let bytes = value.to_le_bytes();
        self.write_bytes(&bytes);
    }

    /// Writes a little-endian f32.
    pub fn write_f32(&mut self, value: f32) {
        let bytes = value.to_le_bytes();
        self.write_bytes(&bytes);
    }

    /// Writes raw bytes.
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        let start = *self.size as usize;
        let end = start + bytes.len();
        if end <= 64 {
            self.data[start..end].copy_from_slice(bytes);
            *self.size = end as u8;
        }
    }

    /// Writes a variable-length unsigned integer (varuint).
    pub fn write_varuint(&mut self, mut value: u32) {
        loop {
            let mut this_byte = (value & 0x7f) as u8;
            value >>= 7;
            if value != 0 {
                this_byte |= 0x80;
            }
            self.write_u8(this_byte);
            if value == 0 {
                break;
            }
        }
    }

    /// Writes an integer value with the specified resolution.
    pub fn write_int(&mut self, value: i32, res: Resolution) {
        match res {
            Resolution::Int8 => {
                let clamped = value.clamp(-127, 127) as i8;
                self.write_i8(clamped);
            }
            Resolution::Int16 => {
                let clamped = value.clamp(-32767, 32767) as i16;
                self.write_i16(clamped);
            }
            Resolution::Int32 => {
                self.write_i32(value);
            }
            Resolution::Float => {
                self.write_f32(value as f32);
            }
            Resolution::Ignore => {}
        }
    }

    /// Writes a scaled value with the specified resolution and scaling.
    pub fn write_mapped(&mut self, value: f64, scaling: &Scaling, res: Resolution) {
        match res {
            Resolution::Int8 => {
                self.write_i8(saturate_i8(value, scaling.int8));
            }
            Resolution::Int16 => {
                self.write_i16(saturate_i16(value, scaling.int16));
            }
            Resolution::Int32 => {
                self.write_i32(saturate_i32(value, scaling.int32));
            }
            Resolution::Float => {
                self.write_f32(value as f32);
            }
            Resolution::Ignore => {}
        }
    }

    // === Convenience methods for common register types ===

    /// Writes a position value (revolutions).
    pub fn write_position(&mut self, value: f64, res: Resolution) {
        self.write_mapped(value, &scaling::POSITION, res);
    }

    /// Writes a velocity value (revolutions/second).
    pub fn write_velocity(&mut self, value: f64, res: Resolution) {
        self.write_mapped(value, &scaling::VELOCITY, res);
    }

    /// Writes an acceleration value (revolutions/second^2).
    pub fn write_accel(&mut self, value: f64, res: Resolution) {
        self.write_mapped(value, &scaling::ACCELERATION, res);
    }

    /// Writes a torque value (Nm).
    pub fn write_torque(&mut self, value: f64, res: Resolution) {
        self.write_mapped(value, &scaling::TORQUE, res);
    }

    /// Writes a PWM/normalized value (0-1).
    pub fn write_pwm(&mut self, value: f64, res: Resolution) {
        self.write_mapped(value, &scaling::PWM, res);
    }

    /// Writes a voltage value (V).
    pub fn write_voltage(&mut self, value: f64, res: Resolution) {
        self.write_mapped(value, &scaling::VOLTAGE, res);
    }

    /// Writes a temperature value (C).
    pub fn write_temperature(&mut self, value: f64, res: Resolution) {
        self.write_mapped(value, &scaling::TEMPERATURE, res);
    }

    /// Writes a time value (seconds).
    pub fn write_time(&mut self, value: f64, res: Resolution) {
        self.write_mapped(value, &scaling::TIME, res);
    }

    /// Writes a current value (A).
    pub fn write_current(&mut self, value: f64, res: Resolution) {
        self.write_mapped(value, &scaling::CURRENT, res);
    }
}

/// Combines consecutive register writes of the same resolution for efficiency.
///
/// This helper determines how to group registers when encoding them to minimize
/// the required bytes in the frame. It tracks state and writes framing bytes
/// when the resolution changes.
pub struct WriteCombiner<'a> {
    base_command: u8,
    start_register: u16,
    resolutions: &'a [Resolution],
    current_resolution: Resolution,
    offset: usize,
    reply_size: u8,
}

impl<'a> WriteCombiner<'a> {
    /// Creates a new WriteCombiner.
    ///
    /// # Arguments
    /// * `base_command` - Base command (0x00 for write, 0x10 for read)
    /// * `start_register` - First register address in the sequence
    /// * `resolutions` - Resolution for each register in sequence
    pub fn new(
        base_command: u8,
        start_register: u16,
        resolutions: &'a [Resolution],
    ) -> Self {
        WriteCombiner {
            base_command,
            start_register,
            resolutions,
            current_resolution: Resolution::Ignore,
            offset: 0,
            reply_size: 0,
        }
    }

    /// Returns the expected reply size so far.
    pub fn reply_size(&self) -> u8 {
        self.reply_size
    }

    /// Advances to the next register and writes framing if needed.
    ///
    /// Returns `true` if the caller should write the register value.
    /// Returns `false` if the register should be skipped (Ignore resolution).
    pub fn maybe_write(&mut self, frame: &mut WriteCanData) -> bool {
        let this_offset = self.offset;
        self.offset += 1;

        if this_offset >= self.resolutions.len() {
            return false;
        }

        let new_resolution = self.resolutions[this_offset];

        // Same resolution as before - no framing needed
        if self.current_resolution == new_resolution {
            return new_resolution != Resolution::Ignore;
        }

        // Update current resolution
        self.current_resolution = new_resolution;

        // Ignore means skip this register
        if new_resolution == Resolution::Ignore {
            return false;
        }

        // Count how many consecutive registers have this resolution
        let mut count = 1i16;
        for i in (this_offset + 1)..self.resolutions.len() {
            if self.resolutions[i] == new_resolution {
                count += 1;
            } else {
                break;
            }
        }

        // Build the command byte
        let write_command = self.base_command + new_resolution.type_code();

        let start_size = frame.size();

        if count <= 3 {
            // Short form: command encodes count in lower 2 bits
            frame.write_u8(write_command + count as u8);
        } else {
            // Long form: count follows command byte
            frame.write_u8(write_command);
            frame.write_u8(count as u8);
        }

        // Write register address
        frame.write_varuint((self.start_register + this_offset as u16) as u32);

        let end_size = frame.size();

        // Update expected reply size
        self.reply_size += end_size - start_size;
        self.reply_size += (count as u8) * (new_resolution.size() as u8);

        true
    }
}

/// A decoded register value from the multiplex protocol.
#[derive(Debug, Clone, Copy)]
pub enum Value {
    /// 8-bit signed integer
    Int8(i8),
    /// 16-bit signed integer
    Int16(i16),
    /// 32-bit signed integer
    Int32(i32),
    /// 32-bit IEEE 754 float
    Float(f32),
}

impl Value {
    /// Returns the raw integer value.
    ///
    /// Float values are cast to i32.  This should only be used for
    /// registers which are intended to hold an integer type, and thus
    /// can not store a non-finite value.
    pub fn to_i32(&self) -> i32 {
        match *self {
            Value::Int8(v) => v as i32,
            Value::Int16(v) => v as i32,
            Value::Int32(v) => v,
            Value::Float(v) => v as i32,
        }
    }

    /// Returns the value as f64, applying NaN mapping and scaling.
    ///
    /// Integer minimum values (e.g., -128 for Int8) are mapped to NaN.
    /// Integer values are multiplied by the appropriate scaling factor.
    /// Float values are returned directly as f64.
    pub fn to_f64(&self, scaling: &Scaling) -> f64 {
        use crate::scaling::{nanify_i8, nanify_i16, nanify_i32};

        match *self {
            Value::Int8(v) => nanify_i8(v) * scaling.int8,
            Value::Int16(v) => nanify_i16(v) * scaling.int16,
            Value::Int32(v) => nanify_i32(v) * scaling.int32,
            Value::Float(v) => v as f64,
        }
    }
}

/// The type of a parsed subframe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubframeType {
    /// Write register values
    Write,
    /// Read register request
    Read,
    /// Response to a read request
    Response,
    /// Write error
    WriteError,
    /// Read error
    ReadError,
    /// Tunneled stream: client to server
    StreamClientToServer,
    /// Tunneled stream: server to client
    StreamServerToClient,
    /// Tunneled stream: client poll server
    StreamClientPollServer,
}

/// A single parsed subframe from a multiplex protocol frame.
#[derive(Debug, Clone)]
pub enum Subframe<'a> {
    /// A register read, write, or response subframe.
    Register {
        /// The type of operation
        subframe_type: SubframeType,
        /// The register address
        register: u16,
        /// The resolution of the value
        resolution: Resolution,
        /// The decoded value (None for Read requests)
        value: Option<Value>,
    },
    /// An error subframe.
    Error {
        /// The type of error (WriteError or ReadError)
        subframe_type: SubframeType,
        /// The register that caused the error
        register: u16,
        /// The error code
        error_code: u16,
    },
    /// A tunneled stream subframe.
    Stream {
        /// The stream direction
        subframe_type: SubframeType,
        /// The stream channel number
        channel: u16,
        /// The stream data (borrowed from the input)
        data: &'a [u8],
    },
}

/// Iterator over subframes in a multiplex protocol frame.
///
/// Handles all opcode families: WRITE (0x00-0x0f), READ (0x10-0x1f),
/// REPLY (0x20-0x2f), ERROR (0x30-0x31), STREAM (0x40-0x42), NOP (0x50).
pub struct FrameParser<'a> {
    data: &'a [u8],
    offset: usize,
    remaining: u16,
    current_register: u16,
    current_resolution: Resolution,
    current_type: SubframeType,
}

impl<'a> FrameParser<'a> {
    /// Creates a parser from a byte slice.
    pub fn new(data: &'a [u8]) -> Self {
        FrameParser {
            data,
            offset: 0,
            remaining: 0,
            current_register: 0,
            current_resolution: Resolution::Ignore,
            current_type: SubframeType::Response,
        }
    }

    /// Creates a parser from a frame.
    pub fn from_frame(frame: &'a CanFdFrame) -> Self {
        Self::new(&frame.data[..frame.size as usize])
    }

    /// Reads a varuint from the current position.
    fn read_varuint(&mut self) -> u16 {
        let mut result: u16 = 0;
        let mut shift = 0;

        for _ in 0..5 {
            if self.offset >= self.data.len() {
                return result;
            }

            let byte = self.data[self.offset];
            self.offset += 1;

            result |= ((byte & 0x7f) as u16) << shift;
            shift += 7;

            if byte & 0x80 == 0 {
                return result;
            }
        }

        result
    }

    /// Reads a value at the current offset with the given resolution.
    fn read_value(&mut self, res: Resolution) -> Option<Value> {
        match res {
            Resolution::Int8 => {
                if self.offset < self.data.len() {
                    let v = self.data[self.offset] as i8;
                    self.offset += 1;
                    Some(Value::Int8(v))
                } else {
                    None
                }
            }
            Resolution::Int16 => {
                if self.offset + 2 <= self.data.len() {
                    let bytes = [self.data[self.offset], self.data[self.offset + 1]];
                    self.offset += 2;
                    Some(Value::Int16(i16::from_le_bytes(bytes)))
                } else {
                    None
                }
            }
            Resolution::Int32 => {
                if self.offset + 4 <= self.data.len() {
                    let bytes = [
                        self.data[self.offset],
                        self.data[self.offset + 1],
                        self.data[self.offset + 2],
                        self.data[self.offset + 3],
                    ];
                    self.offset += 4;
                    Some(Value::Int32(i32::from_le_bytes(bytes)))
                } else {
                    None
                }
            }
            Resolution::Float => {
                if self.offset + 4 <= self.data.len() {
                    let bytes = [
                        self.data[self.offset],
                        self.data[self.offset + 1],
                        self.data[self.offset + 2],
                        self.data[self.offset + 3],
                    ];
                    self.offset += 4;
                    Some(Value::Float(f32::from_le_bytes(bytes)))
                } else {
                    None
                }
            }
            Resolution::Ignore => None,
        }
    }

    /// Yields the next register subframe from the current block.
    fn next_from_block(&mut self) -> Option<Subframe<'a>> {
        if self.remaining == 0 {
            return None;
        }

        self.remaining -= 1;
        let register = self.current_register;
        self.current_register += 1;

        // For Read requests, no value data
        if self.current_type == SubframeType::Read {
            return Some(Subframe::Register {
                subframe_type: self.current_type,
                register,
                resolution: self.current_resolution,
                value: None,
            });
        }

        // For Write/Response, read the value
        let value = self.read_value(self.current_resolution);
        if value.is_none() {
            // Not enough data, stop iteration
            self.offset = self.data.len();
            self.remaining = 0;
            return None;
        }

        Some(Subframe::Register {
            subframe_type: self.current_type,
            register,
            resolution: self.current_resolution,
            value,
        })
    }

    /// Parses a register block header (WRITE, READ, or REPLY).
    fn parse_register_block(&mut self, cmd: u8) -> Option<Subframe<'a>> {
        let family = cmd & 0xf0;
        let subframe_type = match family {
            0x00 => SubframeType::Write,
            0x10 => SubframeType::Read,
            0x20 => SubframeType::Response,
            _ => return None,
        };

        let resolution = match (cmd >> 2) & 0x03 {
            0 => Resolution::Int8,
            1 => Resolution::Int16,
            2 => Resolution::Int32,
            3 => Resolution::Float,
            _ => Resolution::Int8,
        };

        let mut count = (cmd & 0x03) as u16;
        if count == 0 {
            if self.offset >= self.data.len() {
                return None;
            }
            count = self.data[self.offset] as u16;
            self.offset += 1;
        }

        if count == 0 {
            return None;
        }

        let register = self.read_varuint();

        self.current_type = subframe_type;
        self.current_resolution = resolution;
        self.current_register = register + 1;
        self.remaining = count - 1;

        // For Read requests, no value data
        if subframe_type == SubframeType::Read {
            return Some(Subframe::Register {
                subframe_type,
                register,
                resolution,
                value: None,
            });
        }

        // For Write/Response, read the value
        let value = self.read_value(resolution);
        if value.is_none() {
            self.offset = self.data.len();
            self.remaining = 0;
            return None;
        }

        Some(Subframe::Register {
            subframe_type,
            register,
            resolution,
            value,
        })
    }

    /// Parses an error subframe (0x30 or 0x31).
    fn parse_error(&mut self, cmd: u8) -> Option<Subframe<'a>> {
        let subframe_type = match cmd {
            WRITE_ERROR => SubframeType::WriteError,
            READ_ERROR => SubframeType::ReadError,
            _ => return None,
        };

        let register = self.read_varuint();
        let error_code = self.read_varuint();

        Some(Subframe::Error {
            subframe_type,
            register,
            error_code,
        })
    }

    /// Parses a stream subframe (0x40-0x42).
    fn parse_stream(&mut self, cmd: u8) -> Option<Subframe<'a>> {
        let subframe_type = match cmd {
            CLIENT_TO_SERVER => SubframeType::StreamClientToServer,
            SERVER_TO_CLIENT => SubframeType::StreamServerToClient,
            CLIENT_POLL_SERVER => SubframeType::StreamClientPollServer,
            _ => return None,
        };

        let channel = self.read_varuint();
        let size = self.read_varuint() as usize;

        if self.offset + size > self.data.len() {
            self.offset = self.data.len();
            return None;
        }

        let data = &self.data[self.offset..self.offset + size];
        self.offset += size;

        Some(Subframe::Stream {
            subframe_type,
            channel,
            data,
        })
    }
}

impl<'a> Iterator for FrameParser<'a> {
    type Item = Subframe<'a>;

    fn next(&mut self) -> Option<Subframe<'a>> {
        // Continue yielding from current block
        if self.remaining > 0 {
            return self.next_from_block();
        }

        // Look for next command
        while self.offset < self.data.len() {
            let cmd = self.data[self.offset];
            self.offset += 1;

            // NOP: skip silently
            if cmd == NOP {
                continue;
            }

            match cmd & 0xf0 {
                // WRITE (0x00-0x0f), READ (0x10-0x1f), REPLY (0x20-0x2f)
                0x00 | 0x10 | 0x20 => {
                    if let Some(subframe) = self.parse_register_block(cmd) {
                        return Some(subframe);
                    }
                    // parse_register_block returned None (e.g., count=0), try next cmd
                    continue;
                }
                // ERROR (0x30-0x31)
                0x30 => {
                    if let Some(subframe) = self.parse_error(cmd) {
                        return Some(subframe);
                    }
                    continue;
                }
                // STREAM (0x40-0x42)
                0x40 => {
                    if let Some(subframe) = self.parse_stream(cmd) {
                        return Some(subframe);
                    }
                    continue;
                }
                // Unknown: stop iteration
                _ => {
                    self.offset = self.data.len();
                    return None;
                }
            }
        }

        None
    }
}

/// Creates a parser that iterates over all subframes in a multiplex protocol frame.
pub fn parse_frame(data: &[u8]) -> FrameParser<'_> {
    FrameParser::new(data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_write_varuint() {
        // Test varuint 0
        let mut frame = CanFdFrame::new();
        {
            let mut writer = WriteCanData::new(&mut frame);
            writer.write_varuint(0);
        }
        assert_eq!(frame.data[0], 0x00);
        assert_eq!(frame.size, 1);

        // Test varuint 127
        frame.size = 0;
        {
            let mut writer = WriteCanData::new(&mut frame);
            writer.write_varuint(127);
        }
        assert_eq!(frame.data[0], 0x7f);

        // Test varuint 128
        frame.size = 0;
        {
            let mut writer = WriteCanData::new(&mut frame);
            writer.write_varuint(128);
        }
        assert_eq!(frame.data[0], 0x80);
        assert_eq!(frame.data[1], 0x01);
        assert_eq!(frame.size, 2);
    }

    #[test]
    fn test_write_combiner() {
        let mut frame = CanFdFrame::new();
        let mut writer = WriteCanData::new(&mut frame);

        let resolutions = [Resolution::Float, Resolution::Float, Resolution::Ignore];
        let mut combiner = WriteCombiner::new(0x00, 0x020, &resolutions);

        // First register should trigger framing
        assert!(combiner.maybe_write(&mut writer));
        writer.write_f32(1.0);
        // Second register same resolution - no framing
        assert!(combiner.maybe_write(&mut writer));
        writer.write_f32(2.0);
        // Third register is Ignore
        assert!(!combiner.maybe_write(&mut writer));

        // Expected: [0x0e] [0x20] [1.0 as f32 LE] [2.0 as f32 LE]
        //   0x0e = Write Float (0x0c) + count 2
        //   0x20 = register 0x020 as varuint
        assert_eq!(frame.size, 10); // 1 cmd + 1 reg + 4 float + 4 float
        assert_eq!(frame.data[0], 0x0e);
        assert_eq!(frame.data[1], 0x20);
        assert_eq!(
            f32::from_le_bytes(frame.data[2..6].try_into().unwrap()),
            1.0
        );
        assert_eq!(
            f32::from_le_bytes(frame.data[6..10].try_into().unwrap()),
            2.0
        );

        // Verify reply_size: 2 framing bytes + 2 * 4 value bytes = 10
        assert_eq!(combiner.reply_size(), 10);
    }

    #[test]
    fn test_parser_basic() {
        // Build a simple reply frame: mode=10 (position mode)
        let data = [
            0x21, // Reply Int8, count=1
            0x00, // Register 0 (MODE)
            0x0a, // Value 10 (Position mode)
        ];

        let mut iter = parse_frame(&data);

        let subframe = iter.next().unwrap();
        match subframe {
            Subframe::Register { subframe_type, register, resolution, value } => {
                assert_eq!(subframe_type, SubframeType::Response);
                assert_eq!(register, 0);
                assert_eq!(resolution, Resolution::Int8);
                assert_eq!(value.unwrap().to_i32(), 10);
            }
            _ => panic!("Expected Register subframe"),
        }

        assert!(iter.next().is_none());
    }

    #[test]
    fn test_parser_write_subframes() {
        // Write Int16, count=2, register 0x20, values 100 and 200
        let data = [
            0x06, // Write Int16 (0x04), count=2
            0x20, // Register 0x20
            0x64, 0x00, // 100 as i16 LE
            0xc8, 0x00, // 200 as i16 LE
        ];

        let mut iter = parse_frame(&data);

        match iter.next().unwrap() {
            Subframe::Register { subframe_type, register, resolution, value } => {
                assert_eq!(subframe_type, SubframeType::Write);
                assert_eq!(register, 0x20);
                assert_eq!(resolution, Resolution::Int16);
                assert_eq!(value.unwrap().to_i32(), 100);
            }
            _ => panic!("Expected Register subframe"),
        }

        match iter.next().unwrap() {
            Subframe::Register { subframe_type, register, value, .. } => {
                assert_eq!(subframe_type, SubframeType::Write);
                assert_eq!(register, 0x21);
                assert_eq!(value.unwrap().to_i32(), 200);
            }
            _ => panic!("Expected Register subframe"),
        }

        assert!(iter.next().is_none());
    }

    #[test]
    fn test_parser_read_subframes() {
        // Read Float, count=1, register 5
        let data = [
            0x1d, // Read Float (0x1c), count=1
            0x05, // Register 5
        ];

        let mut iter = parse_frame(&data);

        match iter.next().unwrap() {
            Subframe::Register { subframe_type, register, resolution, value } => {
                assert_eq!(subframe_type, SubframeType::Read);
                assert_eq!(register, 5);
                assert_eq!(resolution, Resolution::Float);
                assert!(value.is_none());
            }
            _ => panic!("Expected Register subframe"),
        }

        assert!(iter.next().is_none());
    }

    #[test]
    fn test_parser_nop_skipped() {
        let data = [
            0x50, // NOP
            0x21, // Reply Int8, count=1
            0x00, // Register 0
            0x05, // Value 5
        ];

        let mut iter = parse_frame(&data);

        match iter.next().unwrap() {
            Subframe::Register { register, value, .. } => {
                assert_eq!(register, 0);
                assert_eq!(value.unwrap().to_i32(), 5);
            }
            _ => panic!("Expected Register subframe"),
        }

        assert!(iter.next().is_none());
    }

    #[test]
    fn test_parser_multiple_blocks() {
        // Reply Int8 count=1 reg 0 value 10, then Reply Float count=1 reg 1 value 0.5
        let data = [
            0x21, 0x00, 0x0a,
            0x2d, 0x01, 0x00, 0x00, 0x00, 0x3f,
        ];

        let mut iter = parse_frame(&data);

        match iter.next().unwrap() {
            Subframe::Register { register, value, .. } => {
                assert_eq!(register, 0);
                assert_eq!(value.unwrap().to_i32(), 10);
            }
            _ => panic!("Expected Register subframe"),
        }

        match iter.next().unwrap() {
            Subframe::Register { register, value, .. } => {
                assert_eq!(register, 1);
                let val = match value.unwrap() {
                    Value::Float(f) => f as f64,
                    _ => panic!("Expected Float"),
                };
                assert!((val - 0.5).abs() < 0.001);
            }
            _ => panic!("Expected Register subframe"),
        }

        assert!(iter.next().is_none());
    }

    #[test]
    fn test_value_to_f64() {
        // Int8 with position scaling: 50 * 0.01 = 0.5
        let v = Value::Int8(50);
        assert!((v.to_f64(&scaling::POSITION) - 0.5).abs() < 1e-9);

        // Int8 min = NaN
        let v = Value::Int8(i8::MIN);
        assert!(v.to_f64(&scaling::POSITION).is_nan());

        // Float passes through
        let v = Value::Float(1.5);
        assert!((v.to_f64(&scaling::POSITION) - 1.5).abs() < 1e-9);
    }
}
