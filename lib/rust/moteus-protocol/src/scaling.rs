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

/// Scaling factors for a register type.
///
/// The actual wire value is `physical_value / scale`.
#[derive(Debug, Clone, Copy)]
pub struct Scaling {
    pub(crate) int8: f32,
    pub(crate) int16: f32,
    pub(crate) int32: f32,
}

/// Position scaling: revolutions
pub const POSITION: Scaling = Scaling {
    int8: 0.01,
    int16: 0.0001,
    int32: 0.00001,
};

/// Velocity scaling: revolutions/second
pub const VELOCITY: Scaling = Scaling {
    int8: 0.1,
    int16: 0.00025,
    int32: 0.00001,
};

/// Acceleration scaling: revolutions/second^2
pub const ACCELERATION: Scaling = Scaling {
    int8: 0.05,
    int16: 0.001,
    int32: 0.00001,
};

/// Torque scaling: Nm
pub const TORQUE: Scaling = Scaling {
    int8: 0.5,
    int16: 0.01,
    int32: 0.001,
};

/// PWM/normalized scaling: 0-1 range
pub const PWM: Scaling = Scaling {
    int8: 1.0 / 127.0,
    int16: 1.0 / 32767.0,
    int32: 1.0 / 2147483647.0,
};

/// Voltage scaling: Volts
pub const VOLTAGE: Scaling = Scaling {
    int8: 0.5,
    int16: 0.1,
    int32: 0.001,
};

/// Temperature scaling: Celsius
pub const TEMPERATURE: Scaling = Scaling {
    int8: 1.0,
    int16: 0.1,
    int32: 0.001,
};

/// Time scaling: seconds
pub const TIME: Scaling = Scaling {
    int8: 0.01,
    int16: 0.001,
    int32: 0.000001,
};

/// Current scaling: Amps
pub const CURRENT: Scaling = Scaling {
    int8: 1.0,
    int16: 0.1,
    int32: 0.001,
};

/// Power scaling: Watts
pub const POWER: Scaling = Scaling {
    int8: 10.0,
    int16: 0.05,
    int32: 0.0001,
};

/// Integer scaling: no scaling applied
pub const INT: Scaling = Scaling {
    int8: 1.0,
    int16: 1.0,
    int32: 1.0,
};

/// Saturates a floating-point value to the range representable by the target type.
///
/// Non-finite values are mapped to the minimum value (which represents NaN in the protocol).
#[inline]
pub fn saturate_i8(value: f32, scale: f32) -> i8 {
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
pub fn saturate_i16(value: f32, scale: f32) -> i16 {
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
pub fn saturate_i32(value: f32, scale: f32) -> i32 {
    if !value.is_finite() {
        return i32::MIN;
    }
    let scaled = value / scale;
    let max = i32::MAX as f32;
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
pub fn nanify_i8(value: i8) -> f32 {
    if value == i8::MIN {
        f32::NAN
    } else {
        value as f32
    }
}

/// Converts an i16 value, handling NaN.
#[inline]
pub fn nanify_i16(value: i16) -> f32 {
    if value == i16::MIN {
        f32::NAN
    } else {
        value as f32
    }
}

/// Converts an i32 value, handling NaN.
#[inline]
pub fn nanify_i32(value: i32) -> f32 {
    if value == i32::MIN {
        f32::NAN
    } else {
        value as f32
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
        assert_eq!(saturate_i8(f32::NAN, 0.01), i8::MIN); // NaN
        assert_eq!(saturate_i8(f32::INFINITY, 0.01), i8::MIN);
    }

    #[test]
    fn test_saturate_i16() {
        assert_eq!(saturate_i16(0.5, 0.0001), 5000);
        assert_eq!(saturate_i16(10.0, 0.0001), 32767); // Clamped
    }

    #[test]
    fn test_saturate_i32() {
        assert_eq!(saturate_i32(500.0, 0.01), 50000);
        assert_eq!(saturate_i32(f32::NAN, 0.01), i32::MIN); // NaN
        assert_eq!(saturate_i32(f32::INFINITY, 0.01), i32::MIN);
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
}
