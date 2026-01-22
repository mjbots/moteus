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

//! Scaling constants and functions for moteus register values.
//!
//! The moteus protocol uses fixed-point encoding for register values.
//! Each register type has specific scaling factors for Int8, Int16,
//! and Int32 resolutions. Float values are transmitted directly
//! without scaling.

use crate::Resolution;

/// Scaling factors for a register type.
///
/// Each tuple contains (int8_scale, int16_scale, int32_scale).
/// The actual wire value is `physical_value / scale`.
#[derive(Debug, Clone, Copy)]
pub struct Scaling {
    /// Scale factor for Int8 resolution
    pub int8: f64,
    /// Scale factor for Int16 resolution
    pub int16: f64,
    /// Scale factor for Int32 resolution
    pub int32: f64,
}

impl Scaling {
    /// Creates a new Scaling with the given factors.
    pub const fn new(int8: f64, int16: f64, int32: f64) -> Self {
        Scaling { int8, int16, int32 }
    }

    /// Returns the scale factor for the given resolution.
    pub fn for_resolution(&self, res: Resolution) -> f64 {
        match res {
            Resolution::Int8 => self.int8,
            Resolution::Int16 => self.int16,
            Resolution::Int32 => self.int32,
            Resolution::Float => 1.0,
            Resolution::Ignore => 1.0,
        }
    }

    // === Standard scaling types ===

    /// Position scaling: revolutions
    pub const POSITION: Scaling = Scaling::new(0.01, 0.0001, 0.00001);

    /// Velocity scaling: revolutions/second
    pub const VELOCITY: Scaling = Scaling::new(0.1, 0.00025, 0.00001);

    /// Acceleration scaling: revolutions/second^2
    pub const ACCELERATION: Scaling = Scaling::new(0.05, 0.001, 0.00001);

    /// Torque scaling: Nm
    pub const TORQUE: Scaling = Scaling::new(0.5, 0.01, 0.001);

    /// PWM/normalized scaling: 0-1 range
    pub const PWM: Scaling = Scaling::new(1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0);

    /// Voltage scaling: Volts
    pub const VOLTAGE: Scaling = Scaling::new(0.5, 0.1, 0.001);

    /// Temperature scaling: Celsius
    pub const TEMPERATURE: Scaling = Scaling::new(1.0, 0.1, 0.001);

    /// Time scaling: seconds
    pub const TIME: Scaling = Scaling::new(0.01, 0.001, 0.000001);

    /// Current scaling: Amps
    pub const CURRENT: Scaling = Scaling::new(1.0, 0.1, 0.001);

    /// Power scaling: Watts
    pub const POWER: Scaling = Scaling::new(10.0, 0.05, 0.0001);

    /// Integer scaling: no scaling applied
    pub const INT: Scaling = Scaling::new(1.0, 1.0, 1.0);
}

/// Saturates a floating-point value to the range representable by the target type.
///
/// Non-finite values are mapped to the minimum value (which represents NaN in the protocol).
#[inline]
pub fn saturate_i8(value: f64, scale: f64) -> i8 {
    if !value.is_finite() {
        return i8::MIN;
    }
    let scaled = value / scale;
    if scaled < -127.0 {
        -127
    } else if scaled > 127.0 {
        127
    } else {
        scaled as i8
    }
}

/// Saturates a value to i16 range.
#[inline]
pub fn saturate_i16(value: f64, scale: f64) -> i16 {
    if !value.is_finite() {
        return i16::MIN;
    }
    let scaled = value / scale;
    if scaled < -32767.0 {
        -32767
    } else if scaled > 32767.0 {
        32767
    } else {
        scaled as i16
    }
}

/// Saturates a value to i32 range.
#[inline]
pub fn saturate_i32(value: f64, scale: f64) -> i32 {
    if !value.is_finite() {
        return i32::MIN;
    }
    let scaled = value / scale;
    let max = i32::MAX as f64;
    if scaled < -max {
        -(i32::MAX)
    } else if scaled > max {
        i32::MAX
    } else {
        scaled as i32
    }
}

/// Converts an integer value to NaN if it's the minimum representable value.
#[inline]
pub fn nanify_i8(value: i8) -> f64 {
    if value == i8::MIN {
        f64::NAN
    } else {
        value as f64
    }
}

/// Converts an i16 value, handling NaN.
#[inline]
pub fn nanify_i16(value: i16) -> f64 {
    if value == i16::MIN {
        f64::NAN
    } else {
        value as f64
    }
}

/// Converts an i32 value, handling NaN.
#[inline]
pub fn nanify_i32(value: i32) -> f64 {
    if value == i32::MIN {
        f64::NAN
    } else {
        value as f64
    }
}

/// Reads a scaled value from raw bytes based on resolution.
#[allow(dead_code)] // Public API for external use
pub fn read_scaled(value: i32, resolution: Resolution, scaling: &Scaling) -> f64 {
    match resolution {
        Resolution::Int8 => nanify_i8(value as i8) * scaling.int8,
        Resolution::Int16 => nanify_i16(value as i16) * scaling.int16,
        Resolution::Int32 => nanify_i32(value) * scaling.int32,
        Resolution::Float => f32::from_bits(value as u32) as f64,
        Resolution::Ignore => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_saturate_i8() {
        assert_eq!(saturate_i8(0.5, 0.01), 50);
        assert_eq!(saturate_i8(2.0, 0.01), 127); // Clamped
        assert_eq!(saturate_i8(-2.0, 0.01), -127); // Clamped
        assert_eq!(saturate_i8(f64::NAN, 0.01), i8::MIN); // NaN
        assert_eq!(saturate_i8(f64::INFINITY, 0.01), i8::MIN);
    }

    #[test]
    fn test_saturate_i16() {
        assert_eq!(saturate_i16(0.5, 0.0001), 5000);
        assert_eq!(saturate_i16(10.0, 0.0001), 32767); // Clamped
    }

    #[test]
    fn test_saturate_i32() {
        assert_eq!(saturate_i32(500.0, 0.01), 50000);
        assert_eq!(saturate_i32(f64::NAN, 0.01), i32::MIN); // NaN
        assert_eq!(saturate_i32(f64::INFINITY, 0.01), i32::MIN);
    }

    #[test]
    fn test_nanify() {
        assert!(nanify_i8(i8::MIN).is_nan());
        assert_eq!(nanify_i8(0), 0.0);
        assert_eq!(nanify_i8(100), 100.0);

        assert!(nanify_i16(i16::MIN).is_nan());
        assert_eq!(nanify_i16(1000), 1000.0);

        assert!(nanify_i32(i32::MIN).is_nan());
        assert_eq!(nanify_i32(100000), 100000.0);
    }

    #[test]
    fn test_scaling_for_resolution() {
        let scaling = Scaling::POSITION;
        assert_eq!(scaling.for_resolution(Resolution::Int8), 0.01);
        assert_eq!(scaling.for_resolution(Resolution::Int16), 0.0001);
        assert_eq!(scaling.for_resolution(Resolution::Float), 1.0);
    }

    #[test]
    fn test_read_scaled() {
        let scaling = Scaling::POSITION;

        // Int8: value 50 * 0.01 = 0.5
        assert_eq!(read_scaled(50, Resolution::Int8, &scaling), 0.5);

        // Int16: value 5000 * 0.0001 = 0.5
        assert_eq!(read_scaled(5000, Resolution::Int16, &scaling), 0.5);

        // Int32: value 50000 * 0.00001 = 0.5
        assert_eq!(read_scaled(50000, Resolution::Int32, &scaling), 0.5);

        // NaN handling: min value should produce NaN
        assert!(read_scaled(i8::MIN as i32, Resolution::Int8, &scaling).is_nan());
    }
}
