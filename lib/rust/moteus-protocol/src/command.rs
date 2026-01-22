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

//! Command types for moteus motor control.
//!
//! This module provides structures for each control mode supported by moteus:
//! - Position mode: Primary servo control mode
//! - Current mode: Direct torque/current control
//! - VFOC mode: Voltage-mode field-oriented control
//! - Stay-within mode: Position bounds enforcement
//! - Zero-velocity mode: Hold position with damping
//! - Brake mode: Short motor windings
//! - Stop mode: Disable motor output

use crate::frame::CanFdFrame;
use crate::mode::Mode;
use crate::multiplex::{Multiplex, WriteCanData, WriteCombiner};
use crate::register::Register;
use crate::resolution::Resolution;
use moteus_derive::Setters;

/// Format (resolution) configuration for position mode commands.
#[derive(Debug, Clone)]
pub struct PositionFormat {
    pub position: Resolution,
    pub velocity: Resolution,
    pub feedforward_torque: Resolution,
    pub kp_scale: Resolution,
    pub kd_scale: Resolution,
    pub maximum_torque: Resolution,
    pub stop_position: Resolution,
    pub watchdog_timeout: Resolution,
    pub velocity_limit: Resolution,
    pub accel_limit: Resolution,
    pub fixed_voltage_override: Resolution,
    pub ilimit_scale: Resolution,
    pub fixed_current_override: Resolution,
    pub ignore_position_bounds: Resolution,
}

impl Default for PositionFormat {
    fn default() -> Self {
        PositionFormat {
            position: Resolution::Float,
            velocity: Resolution::Float,
            feedforward_torque: Resolution::Float,
            kp_scale: Resolution::Float,
            kd_scale: Resolution::Float,
            maximum_torque: Resolution::Float,
            stop_position: Resolution::Float,
            watchdog_timeout: Resolution::Float,
            velocity_limit: Resolution::Float,
            accel_limit: Resolution::Float,
            fixed_voltage_override: Resolution::Float,
            ilimit_scale: Resolution::Float,
            fixed_current_override: Resolution::Float,
            ignore_position_bounds: Resolution::Float,
        }
    }
}

/// Position mode command.
///
/// This is the primary control mode for moteus. All fields are optional;
/// fields set to `None` will not be transmitted (using Ignore resolution).
#[derive(Debug, Clone, Default, Setters)]
pub struct PositionCommand {
    /// Target position in revolutions
    pub position: Option<f64>,
    /// Target velocity in revolutions/second
    pub velocity: Option<f64>,
    /// Feedforward torque in Nm
    pub feedforward_torque: Option<f64>,
    /// Kp scale factor (0-1)
    pub kp_scale: Option<f64>,
    /// Kd scale factor (0-1)
    pub kd_scale: Option<f64>,
    /// Maximum torque in Nm
    pub maximum_torque: Option<f64>,
    /// Stop position for trajectory mode
    pub stop_position: Option<f64>,
    /// Watchdog timeout in seconds
    pub watchdog_timeout: Option<f64>,
    /// Velocity limit in revolutions/second
    pub velocity_limit: Option<f64>,
    /// Acceleration limit in revolutions/second^2
    pub accel_limit: Option<f64>,
    /// Fixed voltage override
    pub fixed_voltage_override: Option<f64>,
    /// Current limit scale factor
    pub ilimit_scale: Option<f64>,
    /// Fixed current override in A
    pub fixed_current_override: Option<f64>,
    /// Ignore position bounds flag
    pub ignore_position_bounds: Option<f64>,
}

impl PositionCommand {
    /// Creates a new position command with all fields set to None.
    pub fn new() -> Self {
        Self::default()
    }

    /// Serializes this command to a CAN frame.
    pub fn serialize(&self, frame: &mut CanFdFrame, format: &PositionFormat) {
        let mut writer = WriteCanData::new(frame);

        // Write mode
        writer.write_u8(Multiplex::WRITE_INT8 | 0x01);
        writer.write_u8(Register::Mode.address() as u8);
        writer.write_i8(Mode::Position as i8);

        // Build resolution array based on which fields are set
        let resolutions = [
            if self.position.is_some() { format.position } else { Resolution::Ignore },
            if self.velocity.is_some() { format.velocity } else { Resolution::Ignore },
            if self.feedforward_torque.is_some() { format.feedforward_torque } else { Resolution::Ignore },
            if self.kp_scale.is_some() { format.kp_scale } else { Resolution::Ignore },
            if self.kd_scale.is_some() { format.kd_scale } else { Resolution::Ignore },
            if self.maximum_torque.is_some() { format.maximum_torque } else { Resolution::Ignore },
            if self.stop_position.is_some() { format.stop_position } else { Resolution::Ignore },
            if self.watchdog_timeout.is_some() { format.watchdog_timeout } else { Resolution::Ignore },
            if self.velocity_limit.is_some() { format.velocity_limit } else { Resolution::Ignore },
            if self.accel_limit.is_some() { format.accel_limit } else { Resolution::Ignore },
            if self.fixed_voltage_override.is_some() { format.fixed_voltage_override } else { Resolution::Ignore },
            if self.ilimit_scale.is_some() { format.ilimit_scale } else { Resolution::Ignore },
            if self.fixed_current_override.is_some() { format.fixed_current_override } else { Resolution::Ignore },
            if self.ignore_position_bounds.is_some() { format.ignore_position_bounds } else { Resolution::Ignore },
        ];

        let mut combiner = WriteCombiner::new(
            0x00,
            Register::CommandPosition.address(),
            &resolutions,
        );

        if combiner.maybe_write(&mut writer) {
            writer.write_position(self.position.unwrap_or(0.0), format.position);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_velocity(self.velocity.unwrap_or(0.0), format.velocity);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_torque(self.feedforward_torque.unwrap_or(0.0), format.feedforward_torque);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.kp_scale.unwrap_or(1.0), format.kp_scale);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.kd_scale.unwrap_or(1.0), format.kd_scale);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_torque(self.maximum_torque.unwrap_or(f64::NAN), format.maximum_torque);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_position(self.stop_position.unwrap_or(f64::NAN), format.stop_position);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_time(self.watchdog_timeout.unwrap_or(f64::NAN), format.watchdog_timeout);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_velocity(self.velocity_limit.unwrap_or(f64::NAN), format.velocity_limit);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_accel(self.accel_limit.unwrap_or(f64::NAN), format.accel_limit);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_voltage(self.fixed_voltage_override.unwrap_or(f64::NAN), format.fixed_voltage_override);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.ilimit_scale.unwrap_or(1.0), format.ilimit_scale);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_current(self.fixed_current_override.unwrap_or(f64::NAN), format.fixed_current_override);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_int(self.ignore_position_bounds.unwrap_or(0.0) as i32, format.ignore_position_bounds);
        }
    }
}

/// Format for VFOC mode commands.
#[derive(Debug, Clone)]
pub struct VFOCFormat {
    pub theta: Resolution,
    pub voltage: Resolution,
    pub theta_rate: Resolution,
}

impl Default for VFOCFormat {
    fn default() -> Self {
        VFOCFormat {
            theta: Resolution::Float,
            voltage: Resolution::Float,
            theta_rate: Resolution::Float,
        }
    }
}

/// Voltage-mode FOC command.
#[derive(Debug, Clone, Setters)]
pub struct VFOCCommand {
    /// Electrical angle in radians
    pub theta: f64,
    /// Voltage to apply
    pub voltage: f64,
    /// Electrical angle rate in radians/second
    pub theta_rate: Option<f64>,
}

impl Default for VFOCCommand {
    fn default() -> Self {
        VFOCCommand {
            theta: 0.0,
            voltage: 0.0,
            theta_rate: None,
        }
    }
}

impl VFOCCommand {
    /// Creates a new VFOC command with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Serializes this command to a CAN frame.
    pub fn serialize(&self, frame: &mut CanFdFrame, format: &VFOCFormat) {
        let mut writer = WriteCanData::new(frame);

        // Write mode
        writer.write_u8(Multiplex::WRITE_INT8 | 0x01);
        writer.write_u8(Register::Mode.address() as u8);
        writer.write_i8(Mode::VoltageFoc as i8);

        // Registers are: theta, voltage, (gaps), theta_rate at offset 6
        let resolutions = [
            format.theta,
            format.voltage,
            Resolution::Ignore, // VoltageDqD
            Resolution::Ignore, // VoltageDqQ
            Resolution::Ignore, // CommandQCurrent
            Resolution::Ignore, // CommandDCurrent
            if self.theta_rate.is_some() && self.theta_rate != Some(0.0) {
                format.theta_rate
            } else {
                Resolution::Ignore
            },
        ];

        let mut combiner = WriteCombiner::new(
            0x00,
            Register::VFocTheta.address(),
            &resolutions,
        );

        if combiner.maybe_write(&mut writer) {
            // Theta is stored as PWM-scaled (divided by PI)
            writer.write_pwm(self.theta / core::f64::consts::PI, format.theta);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_voltage(self.voltage, format.voltage);
        }
        // Skip gaps
        combiner.maybe_write(&mut writer);
        combiner.maybe_write(&mut writer);
        combiner.maybe_write(&mut writer);
        combiner.maybe_write(&mut writer);
        if combiner.maybe_write(&mut writer) {
            writer.write_velocity(self.theta_rate.unwrap_or(0.0) / core::f64::consts::PI, format.theta_rate);
        }
    }
}

/// Format for current mode commands.
#[derive(Debug, Clone)]
pub struct CurrentFormat {
    pub d_a: Resolution,
    pub q_a: Resolution,
}

impl Default for CurrentFormat {
    fn default() -> Self {
        CurrentFormat {
            d_a: Resolution::Float,
            q_a: Resolution::Float,
        }
    }
}

/// DQ-axis current command.
#[derive(Debug, Clone)]
pub struct CurrentCommand {
    /// D-axis current in Amps
    pub d_a: f64,
    /// Q-axis current in Amps
    pub q_a: f64,
}

impl Default for CurrentCommand {
    fn default() -> Self {
        CurrentCommand { d_a: 0.0, q_a: 0.0 }
    }
}

impl CurrentCommand {
    /// Creates a new current command with default values (0, 0).
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the D-axis current in Amps.
    pub fn d_current(mut self, v: f64) -> Self {
        self.d_a = v;
        self
    }

    /// Sets the Q-axis current in Amps (torque-producing).
    pub fn q_current(mut self, v: f64) -> Self {
        self.q_a = v;
        self
    }

    /// Serializes this command to a CAN frame.
    pub fn serialize(&self, frame: &mut CanFdFrame, format: &CurrentFormat) {
        let mut writer = WriteCanData::new(frame);

        // Write mode
        writer.write_u8(Multiplex::WRITE_INT8 | 0x01);
        writer.write_u8(Register::Mode.address() as u8);
        writer.write_i8(Mode::Current as i8);

        // Note: register order is Q then D
        let resolutions = [format.q_a, format.d_a];

        let mut combiner = WriteCombiner::new(
            0x00,
            Register::CommandQCurrent.address(),
            &resolutions,
        );

        if combiner.maybe_write(&mut writer) {
            writer.write_current(self.q_a, format.q_a);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_current(self.d_a, format.d_a);
        }
    }
}

/// Format for stay-within mode commands.
#[derive(Debug, Clone)]
pub struct StayWithinFormat {
    pub lower_bound: Resolution,
    pub upper_bound: Resolution,
    pub feedforward_torque: Resolution,
    pub kp_scale: Resolution,
    pub kd_scale: Resolution,
    pub maximum_torque: Resolution,
    pub watchdog_timeout: Resolution,
    pub ilimit_scale: Resolution,
    pub ignore_position_bounds: Resolution,
}

impl Default for StayWithinFormat {
    fn default() -> Self {
        StayWithinFormat {
            lower_bound: Resolution::Float,
            upper_bound: Resolution::Float,
            feedforward_torque: Resolution::Float,
            kp_scale: Resolution::Float,
            kd_scale: Resolution::Float,
            maximum_torque: Resolution::Float,
            watchdog_timeout: Resolution::Float,
            ilimit_scale: Resolution::Float,
            ignore_position_bounds: Resolution::Float,
        }
    }
}

/// Stay-within mode command.
#[derive(Debug, Clone, Default, Setters)]
pub struct StayWithinCommand {
    /// Lower position bound in revolutions
    pub lower_bound: Option<f64>,
    /// Upper position bound in revolutions
    pub upper_bound: Option<f64>,
    /// Feedforward torque in Nm
    pub feedforward_torque: Option<f64>,
    /// Kp scale factor
    pub kp_scale: Option<f64>,
    /// Kd scale factor
    pub kd_scale: Option<f64>,
    /// Maximum torque in Nm
    pub maximum_torque: Option<f64>,
    /// Watchdog timeout in seconds
    pub watchdog_timeout: Option<f64>,
    /// Current limit scale
    pub ilimit_scale: Option<f64>,
    /// Ignore position bounds flag
    pub ignore_position_bounds: Option<f64>,
}

impl StayWithinCommand {
    /// Creates a new stay-within command with all fields set to None.
    pub fn new() -> Self {
        Self::default()
    }

    /// Serializes this command to a CAN frame.
    pub fn serialize(&self, frame: &mut CanFdFrame, format: &StayWithinFormat) {
        let mut writer = WriteCanData::new(frame);

        // Write mode
        writer.write_u8(Multiplex::WRITE_INT8 | 0x01);
        writer.write_u8(Register::Mode.address() as u8);
        writer.write_i8(Mode::StayWithin as i8);

        let resolutions = [
            if self.lower_bound.is_some() { format.lower_bound } else { Resolution::Ignore },
            if self.upper_bound.is_some() { format.upper_bound } else { Resolution::Ignore },
            if self.feedforward_torque.is_some() { format.feedforward_torque } else { Resolution::Ignore },
            if self.kp_scale.is_some() { format.kp_scale } else { Resolution::Ignore },
            if self.kd_scale.is_some() { format.kd_scale } else { Resolution::Ignore },
            if self.maximum_torque.is_some() { format.maximum_torque } else { Resolution::Ignore },
            if self.watchdog_timeout.is_some() { format.watchdog_timeout } else { Resolution::Ignore },
            if self.ilimit_scale.is_some() { format.ilimit_scale } else { Resolution::Ignore },
            if self.ignore_position_bounds.is_some() { format.ignore_position_bounds } else { Resolution::Ignore },
        ];

        let mut combiner = WriteCombiner::new(
            0x00,
            Register::CommandStayWithinLowerBound.address(),
            &resolutions,
        );

        if combiner.maybe_write(&mut writer) {
            writer.write_position(self.lower_bound.unwrap_or(f64::NAN), format.lower_bound);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_position(self.upper_bound.unwrap_or(f64::NAN), format.upper_bound);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_torque(self.feedforward_torque.unwrap_or(0.0), format.feedforward_torque);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.kp_scale.unwrap_or(1.0), format.kp_scale);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.kd_scale.unwrap_or(1.0), format.kd_scale);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_torque(self.maximum_torque.unwrap_or(0.0), format.maximum_torque);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_time(self.watchdog_timeout.unwrap_or(f64::NAN), format.watchdog_timeout);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.ilimit_scale.unwrap_or(1.0), format.ilimit_scale);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_int(self.ignore_position_bounds.unwrap_or(0.0) as i32, format.ignore_position_bounds);
        }
    }
}

/// Format for zero-velocity mode commands.
#[derive(Debug, Clone)]
pub struct ZeroVelocityFormat {
    pub kd_scale: Resolution,
}

impl Default for ZeroVelocityFormat {
    fn default() -> Self {
        ZeroVelocityFormat {
            kd_scale: Resolution::Float,
        }
    }
}

/// Zero-velocity mode command.
#[derive(Debug, Clone, Default, Setters)]
pub struct ZeroVelocityCommand {
    /// Kd scale factor for damping
    pub kd_scale: Option<f64>,
}

impl ZeroVelocityCommand {
    /// Creates a new zero-velocity command with all fields set to None.
    pub fn new() -> Self {
        Self::default()
    }

    /// Serializes this command to a CAN frame.
    pub fn serialize(&self, frame: &mut CanFdFrame, format: &ZeroVelocityFormat) {
        let mut writer = WriteCanData::new(frame);

        // Write mode
        writer.write_u8(Multiplex::WRITE_INT8 | 0x01);
        writer.write_u8(Register::Mode.address() as u8);
        writer.write_i8(Mode::ZeroVelocity as i8);

        // Only write kd_scale if provided
        if let Some(kd) = self.kd_scale {
            let resolutions = [format.kd_scale];
            let mut combiner = WriteCombiner::new(
                0x00,
                Register::CommandKdScale.address(),
                &resolutions,
            );

            if combiner.maybe_write(&mut writer) {
                writer.write_pwm(kd, format.kd_scale);
            }
        }
    }
}

/// Stop mode command - disables motor output.
pub struct StopCommand;

impl StopCommand {
    /// Serializes this command to a CAN frame.
    pub fn serialize(frame: &mut CanFdFrame) {
        let mut writer = WriteCanData::new(frame);

        writer.write_u8(Multiplex::WRITE_INT8 | 0x01);
        writer.write_u8(Register::Mode.address() as u8);
        writer.write_i8(Mode::Stopped as i8);
    }
}

/// Brake mode command - shorts motor windings for braking.
pub struct BrakeCommand;

impl BrakeCommand {
    /// Serializes this command to a CAN frame.
    pub fn serialize(frame: &mut CanFdFrame) {
        let mut writer = WriteCanData::new(frame);

        writer.write_u8(Multiplex::WRITE_INT8 | 0x01);
        writer.write_u8(Register::Mode.address() as u8);
        writer.write_i8(Mode::Brake as i8);
    }
}

/// Output position setting command.
pub struct OutputNearestCommand {
    /// Position to set
    pub position: f64,
}

impl OutputNearestCommand {
    /// Creates a new output nearest command.
    pub fn new(position: f64) -> Self {
        OutputNearestCommand { position }
    }

    /// Serializes this command to a CAN frame.
    pub fn serialize(&self, frame: &mut CanFdFrame) {
        let mut writer = WriteCanData::new(frame);

        writer.write_u8(Multiplex::WRITE_FLOAT | 0x01);
        writer.write_varuint(Register::SetOutputNearest.address() as u32);
        writer.write_f32(self.position as f32);
    }
}

/// Set output position to exact value.
pub struct OutputExactCommand {
    /// Position to set
    pub position: f64,
}

impl OutputExactCommand {
    /// Creates a new output exact command.
    pub fn new(position: f64) -> Self {
        OutputExactCommand { position }
    }

    /// Serializes this command to a CAN frame.
    pub fn serialize(&self, frame: &mut CanFdFrame) {
        let mut writer = WriteCanData::new(frame);

        writer.write_u8(Multiplex::WRITE_FLOAT | 0x01);
        writer.write_varuint(Register::SetOutputExact.address() as u32);
        writer.write_f32(self.position as f32);
    }
}

/// Require reindex command.
pub struct RequireReindexCommand;

impl RequireReindexCommand {
    /// Serializes this command to a CAN frame.
    pub fn serialize(frame: &mut CanFdFrame) {
        let mut writer = WriteCanData::new(frame);

        writer.write_u8(Multiplex::WRITE_INT8 | 0x01);
        writer.write_varuint(Register::RequireReindex.address() as u32);
        writer.write_i8(1);
    }
}

/// Recapture position and velocity command.
pub struct RecapturePositionVelocityCommand;

impl RecapturePositionVelocityCommand {
    /// Serializes this command to a CAN frame.
    pub fn serialize(frame: &mut CanFdFrame) {
        let mut writer = WriteCanData::new(frame);

        writer.write_u8(Multiplex::WRITE_INT8 | 0x01);
        writer.write_varuint(Register::RecapturePositionVelocity.address() as u32);
        writer.write_i8(1);
    }
}

/// Format for AUX PWM commands.
#[derive(Debug, Clone)]
pub struct AuxPwmFormat {
    pub aux1_pwm1: Resolution,
    pub aux1_pwm2: Resolution,
    pub aux1_pwm3: Resolution,
    pub aux1_pwm4: Resolution,
    pub aux1_pwm5: Resolution,
    pub aux2_pwm1: Resolution,
    pub aux2_pwm2: Resolution,
    pub aux2_pwm3: Resolution,
    pub aux2_pwm4: Resolution,
    pub aux2_pwm5: Resolution,
}

impl Default for AuxPwmFormat {
    fn default() -> Self {
        AuxPwmFormat {
            aux1_pwm1: Resolution::Float,
            aux1_pwm2: Resolution::Float,
            aux1_pwm3: Resolution::Float,
            aux1_pwm4: Resolution::Float,
            aux1_pwm5: Resolution::Float,
            aux2_pwm1: Resolution::Float,
            aux2_pwm2: Resolution::Float,
            aux2_pwm3: Resolution::Float,
            aux2_pwm4: Resolution::Float,
            aux2_pwm5: Resolution::Float,
        }
    }
}

/// AUX PWM output command.
///
/// Sets PWM duty cycles on the AUX1 and AUX2 ports.
/// Each port supports up to 5 PWM channels.
/// Values are 0.0-1.0 duty cycle.
#[derive(Debug, Clone, Default, Setters)]
pub struct AuxPwmCommand {
    /// AUX1 PWM channel 1 duty cycle (0.0-1.0)
    pub aux1_pwm1: Option<f64>,
    /// AUX1 PWM channel 2 duty cycle (0.0-1.0)
    pub aux1_pwm2: Option<f64>,
    /// AUX1 PWM channel 3 duty cycle (0.0-1.0)
    pub aux1_pwm3: Option<f64>,
    /// AUX1 PWM channel 4 duty cycle (0.0-1.0)
    pub aux1_pwm4: Option<f64>,
    /// AUX1 PWM channel 5 duty cycle (0.0-1.0)
    pub aux1_pwm5: Option<f64>,
    /// AUX2 PWM channel 1 duty cycle (0.0-1.0)
    pub aux2_pwm1: Option<f64>,
    /// AUX2 PWM channel 2 duty cycle (0.0-1.0)
    pub aux2_pwm2: Option<f64>,
    /// AUX2 PWM channel 3 duty cycle (0.0-1.0)
    pub aux2_pwm3: Option<f64>,
    /// AUX2 PWM channel 4 duty cycle (0.0-1.0)
    pub aux2_pwm4: Option<f64>,
    /// AUX2 PWM channel 5 duty cycle (0.0-1.0)
    pub aux2_pwm5: Option<f64>,
}

impl AuxPwmCommand {
    /// Creates a new AUX PWM command with all channels unset.
    pub fn new() -> Self {
        Self::default()
    }

    /// Serializes this command to a CAN frame.
    pub fn serialize(&self, frame: &mut CanFdFrame, format: &AuxPwmFormat) {
        let mut writer = WriteCanData::new(frame);

        let resolutions = [
            if self.aux1_pwm1.is_some() { format.aux1_pwm1 } else { Resolution::Ignore },
            if self.aux1_pwm2.is_some() { format.aux1_pwm2 } else { Resolution::Ignore },
            if self.aux1_pwm3.is_some() { format.aux1_pwm3 } else { Resolution::Ignore },
            if self.aux1_pwm4.is_some() { format.aux1_pwm4 } else { Resolution::Ignore },
            if self.aux1_pwm5.is_some() { format.aux1_pwm5 } else { Resolution::Ignore },
            if self.aux2_pwm1.is_some() { format.aux2_pwm1 } else { Resolution::Ignore },
            if self.aux2_pwm2.is_some() { format.aux2_pwm2 } else { Resolution::Ignore },
            if self.aux2_pwm3.is_some() { format.aux2_pwm3 } else { Resolution::Ignore },
            if self.aux2_pwm4.is_some() { format.aux2_pwm4 } else { Resolution::Ignore },
            if self.aux2_pwm5.is_some() { format.aux2_pwm5 } else { Resolution::Ignore },
        ];

        let mut combiner = WriteCombiner::new(
            0x00,
            Register::Aux1Pwm1.address(),
            &resolutions,
        );

        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.aux1_pwm1.unwrap_or(0.0), format.aux1_pwm1);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.aux1_pwm2.unwrap_or(0.0), format.aux1_pwm2);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.aux1_pwm3.unwrap_or(0.0), format.aux1_pwm3);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.aux1_pwm4.unwrap_or(0.0), format.aux1_pwm4);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.aux1_pwm5.unwrap_or(0.0), format.aux1_pwm5);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.aux2_pwm1.unwrap_or(0.0), format.aux2_pwm1);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.aux2_pwm2.unwrap_or(0.0), format.aux2_pwm2);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.aux2_pwm3.unwrap_or(0.0), format.aux2_pwm3);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.aux2_pwm4.unwrap_or(0.0), format.aux2_pwm4);
        }
        if combiner.maybe_write(&mut writer) {
            writer.write_pwm(self.aux2_pwm5.unwrap_or(0.0), format.aux2_pwm5);
        }
    }
}

/// Set clock trim command.
pub struct SetTrimCommand {
    /// Clock trim value
    pub trim: i32,
}

impl SetTrimCommand {
    /// Creates a new set trim command.
    pub fn new(trim: i32) -> Self {
        SetTrimCommand { trim }
    }

    /// Serializes this command to a CAN frame.
    pub fn serialize(&self, frame: &mut CanFdFrame) {
        let mut writer = WriteCanData::new(frame);

        writer.write_u8(Multiplex::WRITE_INT32 | 0x01);
        writer.write_varuint(Register::ClockTrim.address() as u32);
        writer.write_i32(self.trim);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bytes(frame: &CanFdFrame) -> &[u8] {
        &frame.data[..frame.size as usize]
    }

    #[test]
    fn test_stop_command() {
        let mut f = CanFdFrame::new();
        StopCommand::serialize(&mut f);
        assert_eq!(bytes(&f), [1, 0, 0]);
    }

    #[test]
    fn test_brake_command() {
        let mut f = CanFdFrame::new();
        BrakeCommand::serialize(&mut f);
        assert_eq!(bytes(&f), [1, 0, 15]);
    }

    #[test]
    fn test_position_command() {
        let mut f = CanFdFrame::new();
        PositionCommand::new().position(0.5).velocity(1.0)
            .serialize(&mut f, &PositionFormat::default());
        assert_eq!(bytes(&f), [1, 0, 10, 14, 32, 0, 0, 0, 63, 0, 0, 128, 63]);
    }

    #[test]
    fn test_vfoc_command() {
        let mut f = CanFdFrame::new();
        VFOCCommand::new().theta(1.0).voltage(2.0)
            .serialize(&mut f, &VFOCFormat::default());
        assert_eq!(bytes(&f), [1, 0, 7, 14, 24, 131, 249, 162, 62, 0, 0, 0, 64]);
    }

    #[test]
    fn test_current_command() {
        let mut f = CanFdFrame::new();
        CurrentCommand::new().q_current(1.5).d_current(0.5)
            .serialize(&mut f, &CurrentFormat::default());
        assert_eq!(bytes(&f), [1, 0, 9, 14, 28, 0, 0, 192, 63, 0, 0, 0, 63]);
    }

    #[test]
    fn test_stay_within_command() {
        let mut f = CanFdFrame::new();
        StayWithinCommand::new().lower_bound(-1.0).upper_bound(1.0).maximum_torque(0.3)
            .serialize(&mut f, &StayWithinFormat::default());
        assert_eq!(bytes(&f),
            [1, 0, 13, 14, 64, 0, 0, 128, 191, 0, 0, 128, 63, 13, 69, 154, 153, 153, 62]);
    }

    #[test]
    fn test_zero_velocity_command() {
        let mut f = CanFdFrame::new();
        ZeroVelocityCommand::new().kd_scale(0.5)
            .serialize(&mut f, &ZeroVelocityFormat::default());
        assert_eq!(bytes(&f), [1, 0, 12, 13, 36, 0, 0, 0, 63]);
    }

    #[test]
    fn test_output_nearest_command() {
        let mut f = CanFdFrame::new();
        OutputNearestCommand::new(2.5).serialize(&mut f);
        assert_eq!(bytes(&f), [13, 176, 2, 0, 0, 32, 64]);
    }

    #[test]
    fn test_output_exact_command() {
        let mut f = CanFdFrame::new();
        OutputExactCommand::new(3.0).serialize(&mut f);
        assert_eq!(bytes(&f), [13, 177, 2, 0, 0, 64, 64]);
    }

    #[test]
    fn test_require_reindex_command() {
        let mut f = CanFdFrame::new();
        RequireReindexCommand::serialize(&mut f);
        assert_eq!(bytes(&f), [1, 178, 2, 1]);
    }

    #[test]
    fn test_recapture_position_velocity_command() {
        let mut f = CanFdFrame::new();
        RecapturePositionVelocityCommand::serialize(&mut f);
        assert_eq!(bytes(&f), [1, 179, 2, 1]);
    }

    #[test]
    fn test_aux_pwm_command() {
        let mut f = CanFdFrame::new();
        AuxPwmCommand::new().aux1_pwm1(0.75).aux2_pwm3(0.25)
            .serialize(&mut f, &AuxPwmFormat::default());
        assert_eq!(bytes(&f), [13, 118, 0, 0, 64, 63, 13, 125, 0, 0, 128, 62]);
    }

    #[test]
    fn test_set_trim_command() {
        let mut f = CanFdFrame::new();
        SetTrimCommand::new(42).serialize(&mut f);
        assert_eq!(bytes(&f), [9, 113, 42, 0, 0, 0]);
    }
}
