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

//! Mode and state enumerations for moteus controllers.

use num_enum::{IntoPrimitive, TryFromPrimitive};

/// The operating mode of a moteus controller.
///
/// This corresponds to the value in the `Register::Mode` register.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum Mode {
    /// Motor is stopped and unpowered
    #[default]
    Stopped = 0,
    /// Controller is in a fault state
    Fault = 1,
    /// Controller is enabling (transitioning to active)
    Enabling = 2,
    /// Controller is calibrating
    Calibrating = 3,
    /// Calibration is complete
    CalibrationComplete = 4,
    /// Raw PWM mode
    Pwm = 5,
    /// Voltage mode
    Voltage = 6,
    /// Voltage FOC mode
    VoltageFoc = 7,
    /// Voltage DQ mode
    VoltageDq = 8,
    /// Current (torque) mode
    Current = 9,
    /// Position mode (primary operating mode)
    Position = 10,
    /// Position timeout (watchdog expired)
    Timeout = 11,
    /// Zero velocity mode (hold position with damping)
    ZeroVelocity = 12,
    /// Stay within bounds mode
    StayWithin = 13,
    /// Measure inductance mode (calibration)
    MeasureInd = 14,
    /// Brake mode (short motor windings)
    Brake = 15,
}

impl core::fmt::Display for Mode {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Mode::Stopped => write!(f, "stopped"),
            Mode::Fault => write!(f, "fault"),
            Mode::Enabling => write!(f, "enabling"),
            Mode::Calibrating => write!(f, "calibrating"),
            Mode::CalibrationComplete => write!(f, "calibration complete"),
            Mode::Pwm => write!(f, "pwm"),
            Mode::Voltage => write!(f, "voltage"),
            Mode::VoltageFoc => write!(f, "voltage foc"),
            Mode::VoltageDq => write!(f, "voltage dq"),
            Mode::Current => write!(f, "current"),
            Mode::Position => write!(f, "position"),
            Mode::Timeout => write!(f, "timeout"),
            Mode::ZeroVelocity => write!(f, "zero velocity"),
            Mode::StayWithin => write!(f, "stay within"),
            Mode::MeasureInd => write!(f, "measure inductance"),
            Mode::Brake => write!(f, "brake"),
        }
    }
}

impl Mode {
    /// Returns `true` if the controller is in an error state.
    pub fn is_error(&self) -> bool {
        matches!(self, Mode::Fault | Mode::Timeout)
    }

    /// Returns `true` if the controller is actively controlling the motor.
    pub fn is_active(&self) -> bool {
        matches!(
            self,
            Mode::Position
                | Mode::Current
                | Mode::VoltageFoc
                | Mode::ZeroVelocity
                | Mode::StayWithin
                | Mode::Brake
        )
    }
}

impl From<Mode> for i8 {
    fn from(mode: Mode) -> i8 {
        let v: u8 = mode.into();
        v as i8
    }
}

/// The homing/rezero state of the controller.
///
/// This indicates the reference frame for position values.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum HomeState {
    /// Position is relative to power-on position
    #[default]
    Relative = 0,
    /// Position is relative to rotor position
    Rotor = 1,
    /// Position is absolute (output-referenced)
    Output = 2,
}

impl core::fmt::Display for HomeState {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            HomeState::Relative => write!(f, "relative"),
            HomeState::Rotor => write!(f, "rotor"),
            HomeState::Output => write!(f, "output"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mode_try_from() {
        assert_eq!(Mode::try_from(0), Ok(Mode::Stopped));
        assert_eq!(Mode::try_from(10), Ok(Mode::Position));
        assert!(Mode::try_from(255).is_err());
    }

    #[test]
    fn test_mode_into() {
        let v: u8 = Mode::Position.into();
        assert_eq!(v, 10);
    }

    #[test]
    fn test_mode_is_error() {
        assert!(Mode::Fault.is_error());
        assert!(Mode::Timeout.is_error());
        assert!(!Mode::Position.is_error());
        assert!(!Mode::Stopped.is_error());
    }

    #[test]
    fn test_mode_is_active() {
        assert!(Mode::Position.is_active());
        assert!(Mode::Current.is_active());
        assert!(Mode::Brake.is_active());
        assert!(!Mode::Stopped.is_active());
        assert!(!Mode::Fault.is_active());
    }

    #[test]
    fn test_home_state_try_from() {
        assert_eq!(HomeState::try_from(0), Ok(HomeState::Relative));
        assert_eq!(HomeState::try_from(2), Ok(HomeState::Output));
        assert!(HomeState::try_from(3).is_err());
    }

    #[test]
    fn test_home_state_into() {
        let v: u8 = HomeState::Output.into();
        assert_eq!(v, 2);
    }
}
