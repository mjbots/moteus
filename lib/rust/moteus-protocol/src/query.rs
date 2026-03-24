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

//! Query types for reading telemetry from moteus controllers.
//!
//! Queries request register values from the controller and parse the response.

use crate::frame::CanFdFrame;
use crate::mode::{HomeState, Mode};
use crate::multiplex::{parse_frame, Subframe, Value, WriteCanData, WriteCombiner};
use crate::register::Register;
use crate::resolution::Resolution;
use crate::scaling;

/// Maximum number of extra (custom) registers that can be queried.
pub const MAX_EXTRA: usize = 16;

/// Format configuration for queries.
///
/// Specifies which registers to query and at what resolution.
/// Setting a field to `Resolution::Ignore` means that register will not be queried.
#[derive(Debug, Clone)]
pub struct QueryFormat {
    pub mode: Resolution,
    pub position: Resolution,
    pub velocity: Resolution,
    pub torque: Resolution,
    pub q_current: Resolution,
    pub d_current: Resolution,
    pub abs_position: Resolution,
    pub power: Resolution,
    pub motor_temperature: Resolution,
    pub trajectory_complete: Resolution,
    pub home_state: Resolution,
    pub voltage: Resolution,
    pub temperature: Resolution,
    pub fault: Resolution,

    pub aux1_gpio: Resolution,
    pub aux2_gpio: Resolution,

    pub aux1_pwm_input_period_us: Resolution,
    pub aux1_pwm_input_duty_cycle: Resolution,
    pub aux2_pwm_input_period_us: Resolution,
    pub aux2_pwm_input_duty_cycle: Resolution,

    /// Extra registers to query, as (register_number, resolution) pairs.
    /// Entries with `Resolution::Ignore` are unused. Active entries must be
    /// sorted by register number (maintained by `add_extra`).
    pub extra: [(u16, Resolution); MAX_EXTRA],
}

impl Default for QueryFormat {
    fn default() -> Self {
        QueryFormat {
            mode: Resolution::Int8,
            position: Resolution::Float,
            velocity: Resolution::Float,
            torque: Resolution::Float,
            q_current: Resolution::Ignore,
            d_current: Resolution::Ignore,
            abs_position: Resolution::Ignore,
            power: Resolution::Ignore,
            motor_temperature: Resolution::Ignore,
            trajectory_complete: Resolution::Ignore,
            home_state: Resolution::Ignore,
            voltage: Resolution::Int8,
            temperature: Resolution::Int8,
            fault: Resolution::Int8,
            aux1_gpio: Resolution::Ignore,
            aux2_gpio: Resolution::Ignore,
            aux1_pwm_input_period_us: Resolution::Ignore,
            aux1_pwm_input_duty_cycle: Resolution::Ignore,
            aux2_pwm_input_period_us: Resolution::Ignore,
            aux2_pwm_input_duty_cycle: Resolution::Ignore,
            extra: [(u16::MAX, Resolution::Ignore); MAX_EXTRA],
        }
    }
}

impl QueryFormat {
    /// Creates a minimal query format (mode, position, velocity, torque only).
    pub fn minimal() -> Self {
        QueryFormat {
            voltage: Resolution::Ignore,
            temperature: Resolution::Ignore,
            fault: Resolution::Ignore,
            ..Default::default()
        }
    }

    /// Creates a comprehensive query format with all common fields.
    pub fn comprehensive() -> Self {
        QueryFormat {
            q_current: Resolution::Float,
            d_current: Resolution::Float,
            abs_position: Resolution::Float,
            power: Resolution::Float,
            motor_temperature: Resolution::Float,
            trajectory_complete: Resolution::Int8,
            home_state: Resolution::Int8,
            ..Default::default()
        }
    }

    /// Returns the number of active extra register entries.
    pub fn extra_count(&self) -> usize {
        self.extra.iter().filter(|(_, r)| *r != Resolution::Ignore).count()
    }

    /// Adds an extra register to query.
    pub fn add_extra(&mut self, register: u16, resolution: Resolution) {
        // Find the first unused slot
        let slot = self.extra.iter().position(|(_, r)| *r == Resolution::Ignore);
        if let Some(idx) = slot {
            self.extra[idx] = (register, resolution);
            let count = idx + 1;
            // Sort active entries by register number using insertion sort
            for i in (1..count).rev() {
                if self.extra[i].0 < self.extra[i - 1].0 {
                    self.extra.swap(i, i - 1);
                } else {
                    break;
                }
            }
        }
    }

    /// Serializes the query to a CAN frame and returns expected reply size.
    pub fn serialize(&self, frame: &mut CanFdFrame) -> u8 {
        let mut writer = WriteCanData::new(frame);
        let mut reply_size: u8 = 0;

        // First block: Mode through Power (registers 0x000-0x007)
        {
            let resolutions = [
                self.mode,
                self.position,
                self.velocity,
                self.torque,
                self.q_current,
                self.d_current,
                self.abs_position,
                self.power,
            ];
            let mut combiner = WriteCombiner::new(
                0x10, // Read base
                Register::Mode.address(),
                &resolutions,
            );
            for _ in 0..resolutions.len() {
                combiner.maybe_write(&mut writer);
            }
            reply_size += combiner.reply_size();
        }

        // Second block: MotorTemperature through Fault (registers 0x00a-0x00f)
        {
            let resolutions = [
                self.motor_temperature,
                self.trajectory_complete,
                self.home_state,
                self.voltage,
                self.temperature,
                self.fault,
            ];
            let mut combiner = WriteCombiner::new(
                0x10,
                Register::MotorTemperature.address(),
                &resolutions,
            );
            for _ in 0..resolutions.len() {
                combiner.maybe_write(&mut writer);
            }
            reply_size += combiner.reply_size();
        }

        // Third block: GPIO status (registers 0x05e-0x05f)
        {
            let resolutions = [self.aux1_gpio, self.aux2_gpio];
            let mut combiner = WriteCombiner::new(
                0x10,
                Register::Aux1GpioStatus.address(),
                &resolutions,
            );
            for _ in 0..resolutions.len() {
                combiner.maybe_write(&mut writer);
            }
            reply_size += combiner.reply_size();
        }

        // Fourth block: PWM input (registers 0x072-0x075)
        {
            let resolutions = [
                self.aux1_pwm_input_period_us,
                self.aux1_pwm_input_duty_cycle,
                self.aux2_pwm_input_period_us,
                self.aux2_pwm_input_duty_cycle,
            ];
            let mut combiner = WriteCombiner::new(
                0x10,
                Register::Aux1PwmInputPeriod.address(),
                &resolutions,
            );
            for _ in 0..resolutions.len() {
                combiner.maybe_write(&mut writer);
            }
            reply_size += combiner.reply_size();
        }

        // Extra registers — group nearby registers into spans of up to
        // MAX_GROUP_SPAN so the resolution array stays on the stack.
        let extra_count = self.extra_count();
        if extra_count > 0 {
            const MAX_GROUP_SPAN: usize = 64;
            let mut group_start = 0;

            while group_start < extra_count {
                let base_reg = self.extra[group_start].0;
                let mut group_end = group_start + 1;

                while group_end < extra_count
                    && (self.extra[group_end].0 - base_reg) < MAX_GROUP_SPAN as u16
                {
                    group_end += 1;
                }

                let last_reg = self.extra[group_end - 1].0;
                let span = (last_reg - base_reg + 1) as usize;

                let mut resolutions = [Resolution::Ignore; MAX_GROUP_SPAN];
                for i in group_start..group_end {
                    let (reg, res) = self.extra[i];
                    resolutions[(reg - base_reg) as usize] = res;
                }

                let mut combiner =
                    WriteCombiner::new(0x10, base_reg, &resolutions[..span]);
                for _ in 0..span {
                    combiner.maybe_write(&mut writer);
                }
                reply_size += combiner.reply_size();

                group_start = group_end;
            }
        }

        reply_size
    }
}

/// Extra register value pair.
#[derive(Debug, Clone, Copy, Default)]
pub struct ExtraValue {
    /// Register number
    pub register: u16,
    /// Register value
    pub value: f64,
}

/// Result of parsing a query response.
#[derive(Debug, Clone, Default)]
pub struct QueryResult {
    pub mode: Mode,
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub q_current: f64,
    pub d_current: f64,
    pub abs_position: f64,
    pub power: f64,
    pub motor_temperature: f64,
    pub trajectory_complete: bool,
    pub home_state: HomeState,
    pub voltage: f64,
    pub temperature: f64,
    pub fault: i8,

    pub aux1_gpio: i8,
    pub aux2_gpio: i8,

    pub aux1_pwm_input_period_us: i32,
    pub aux1_pwm_input_duty_cycle: f32,
    pub aux2_pwm_input_period_us: i32,
    pub aux2_pwm_input_duty_cycle: f32,

    /// Extra register values. `None` entries are unused.
    pub extra: [Option<ExtraValue>; MAX_EXTRA],
}

impl QueryResult {
    /// Creates a new QueryResult with NaN values.
    pub fn new() -> Self {
        QueryResult {
            mode: Mode::Stopped,
            position: f64::NAN,
            velocity: f64::NAN,
            torque: f64::NAN,
            q_current: f64::NAN,
            d_current: f64::NAN,
            abs_position: f64::NAN,
            power: f64::NAN,
            motor_temperature: f64::NAN,
            trajectory_complete: false,
            home_state: HomeState::Relative,
            voltage: f64::NAN,
            temperature: f64::NAN,
            fault: 0,
            aux1_gpio: 0,
            aux2_gpio: 0,
            aux1_pwm_input_period_us: 0,
            aux1_pwm_input_duty_cycle: 0.0,
            aux2_pwm_input_period_us: 0,
            aux2_pwm_input_duty_cycle: 0.0,
            extra: [None; MAX_EXTRA],
        }
    }

    /// Parses a query response from a CAN frame.
    pub fn parse(frame: &CanFdFrame) -> Self {
        Self::parse_data(&frame.data[..frame.size as usize])
    }

    /// Parses a query response from raw bytes.
    pub fn parse_data(data: &[u8]) -> Self {
        let mut result = QueryResult::new();

        for subframe in parse_frame(data) {
            let (register, value) = match subframe {
                Subframe::Register { register, value: Some(value), .. } => (register, value),
                _ => continue,
            };

            match register {
                r if r == Register::Mode.address() => {
                    result.mode = Mode::try_from(value.to_i32() as u8).unwrap_or(Mode::Stopped);
                }
                r if r == Register::Position.address() => {
                    result.position = value.to_f64(&scaling::POSITION);
                }
                r if r == Register::Velocity.address() => {
                    result.velocity = value.to_f64(&scaling::VELOCITY);
                }
                r if r == Register::Torque.address() => {
                    result.torque = value.to_f64(&scaling::TORQUE);
                }
                r if r == Register::QCurrent.address() => {
                    result.q_current = value.to_f64(&scaling::CURRENT);
                }
                r if r == Register::DCurrent.address() => {
                    result.d_current = value.to_f64(&scaling::CURRENT);
                }
                r if r == Register::AbsPosition.address() => {
                    result.abs_position = value.to_f64(&scaling::POSITION);
                }
                r if r == Register::Power.address() => {
                    result.power = value.to_f64(&scaling::POWER);
                }
                r if r == Register::MotorTemperature.address() => {
                    result.motor_temperature = value.to_f64(&scaling::TEMPERATURE);
                }
                r if r == Register::TrajectoryComplete.address() => {
                    result.trajectory_complete = value.to_i32() != 0;
                }
                r if r == Register::HomeState.address() => {
                    result.home_state = HomeState::try_from(value.to_i32() as u8).unwrap_or(HomeState::Relative);
                }
                r if r == Register::Voltage.address() => {
                    result.voltage = value.to_f64(&scaling::VOLTAGE);
                }
                r if r == Register::Temperature.address() => {
                    result.temperature = value.to_f64(&scaling::TEMPERATURE);
                }
                r if r == Register::Fault.address() => {
                    result.fault = value.to_i32() as i8;
                }
                r if r == Register::Aux1GpioStatus.address() => {
                    result.aux1_gpio = value.to_i32() as i8;
                }
                r if r == Register::Aux2GpioStatus.address() => {
                    result.aux2_gpio = value.to_i32() as i8;
                }
                r if r == Register::Aux1PwmInputPeriod.address() => {
                    result.aux1_pwm_input_period_us = value.to_i32();
                }
                r if r == Register::Aux1PwmInputDutyCycle.address() => {
                    result.aux1_pwm_input_duty_cycle = value.to_f64(&scaling::PWM) as f32;
                }
                r if r == Register::Aux2PwmInputPeriod.address() => {
                    result.aux2_pwm_input_period_us = value.to_i32();
                }
                r if r == Register::Aux2PwmInputDutyCycle.address() => {
                    result.aux2_pwm_input_duty_cycle = value.to_f64(&scaling::PWM) as f32;
                }
                _ => {
                    if let Some(slot) = result.extra.iter().position(|e| e.is_none()) {
                        result.extra[slot] = Some(ExtraValue {
                            register,
                            value: parse_generic(register, value),
                        });
                    } else {
                        debug_assert!(false, "MAX_EXTRA ({}) exceeded, register 0x{:x} dropped", MAX_EXTRA, register);
                    }
                }
            }
        }

        result
    }

    /// Gets an extra value by register number.
    pub fn get_extra(&self, register: u16) -> Option<f64> {
        for entry in &self.extra {
            match entry {
                Some(ev) if ev.register == register => return Some(ev.value),
                None => return None,
                _ => {}
            }
        }
        None
    }
}

/// Parses a generic register value based on register number.
fn parse_generic(register: u16, value: Value) -> f64 {
    use crate::scaling;

    // Determine scaling based on register
    let reg = Register::from_address(register);

    let scaling = match reg {
        Some(Register::Position)
        | Some(Register::AbsPosition)
        | Some(Register::CommandPosition)
        | Some(Register::CommandStopPosition)
        | Some(Register::CommandStayWithinLowerBound)
        | Some(Register::CommandStayWithinUpperBound)
        | Some(Register::ControlPosition)
        | Some(Register::ControlPositionError)
        | Some(Register::Encoder0Position)
        | Some(Register::Encoder1Position)
        | Some(Register::Encoder2Position) => &scaling::POSITION,

        Some(Register::Velocity)
        | Some(Register::CommandVelocity)
        | Some(Register::CommandVelocityLimit)
        | Some(Register::ControlVelocity)
        | Some(Register::ControlVelocityError)
        | Some(Register::Encoder0Velocity)
        | Some(Register::Encoder1Velocity)
        | Some(Register::Encoder2Velocity) => &scaling::VELOCITY,

        Some(Register::Torque)
        | Some(Register::CommandFeedforwardTorque)
        | Some(Register::CommandPositionMaxTorque)
        | Some(Register::ControlTorque)
        | Some(Register::ControlTorqueError)
        | Some(Register::PositionKp)
        | Some(Register::PositionKi)
        | Some(Register::PositionKd)
        | Some(Register::PositionFeedforward)
        | Some(Register::PositionCommand) => &scaling::TORQUE,

        Some(Register::QCurrent)
        | Some(Register::DCurrent)
        | Some(Register::CommandQCurrent)
        | Some(Register::CommandDCurrent)
        | Some(Register::CommandFixedCurrentOverride) => &scaling::CURRENT,

        Some(Register::Voltage)
        | Some(Register::VoltagePhaseA)
        | Some(Register::VoltagePhaseB)
        | Some(Register::VoltagePhaseC)
        | Some(Register::VFocVoltage)
        | Some(Register::VoltageDqD)
        | Some(Register::VoltageDqQ)
        | Some(Register::CommandFixedVoltageOverride) => &scaling::VOLTAGE,

        Some(Register::Temperature) | Some(Register::MotorTemperature) => &scaling::TEMPERATURE,

        Some(Register::Power) => &scaling::POWER,

        Some(Register::CommandTimeout) | Some(Register::CommandStayWithinTimeout) => {
            &scaling::TIME
        }

        Some(Register::CommandAccelLimit) => &scaling::ACCELERATION,

        Some(Register::CommandKpScale)
        | Some(Register::CommandKdScale)
        | Some(Register::CommandIlimitScale)
        | Some(Register::PwmPhaseA)
        | Some(Register::PwmPhaseB)
        | Some(Register::PwmPhaseC)
        | Some(Register::Aux1PwmInputDutyCycle)
        | Some(Register::Aux2PwmInputDutyCycle) => &scaling::PWM,

        _ => &scaling::INT,
    };

    value.to_f64(scaling)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_query_format_default() {
        let format = QueryFormat::default();
        assert_eq!(format.mode, Resolution::Int8);
        assert_eq!(format.position, Resolution::Float);
        assert_eq!(format.q_current, Resolution::Ignore);
    }

    fn bytes(frame: &CanFdFrame) -> &[u8] {
        &frame.data[..frame.size as usize]
    }

    #[test]
    fn test_query_format_serialize() {
        let format = QueryFormat::default();
        let mut frame = CanFdFrame::new();
        let reply_size = format.serialize(&mut frame);

        assert_eq!(reply_size, 22);
        assert_eq!(bytes(&frame), &[0x11, 0x00, 0x1f, 0x01, 0x13, 0x0d]);
    }

    #[test]
    fn test_query_format_serialize_with_extras() {
        let mut format = QueryFormat::minimal();
        format.add_extra(0x100, Resolution::Float);
        format.add_extra(0x102, Resolution::Int16);
        let mut frame = CanFdFrame::new();
        let reply_size = format.serialize(&mut frame);

        assert_eq!(reply_size, 29);
        assert_eq!(
            bytes(&frame),
            &[0x11, 0x00, 0x1f, 0x01, 0x1d, 0x80, 0x02, 0x15, 0x82, 0x02]
        );
    }

    #[test]
    fn test_query_format_serialize_with_distant_extras() {
        // Extras spanning more than 64 registers get separate groups.
        let mut format = QueryFormat::minimal();
        format.add_extra(0x010, Resolution::Int32);
        format.add_extra(0x100, Resolution::Float);
        let mut frame = CanFdFrame::new();
        let reply_size = format.serialize(&mut frame);

        assert_eq!(reply_size, 30);
        assert_eq!(
            bytes(&frame),
            &[0x11, 0x00, 0x1f, 0x01, 0x19, 0x10, 0x1d, 0x80, 0x02]
        );
    }

    #[test]
    fn test_query_result_parse() {
        // Build a simple reply: mode=10, position=0.5
        let data = [
            0x21,             // Reply Int8, count=1
            0x00,             // Register 0 (MODE)
            0x0a,             // Mode::Position (10)
            0x2d,             // Reply Float, count=1
            0x01,             // Register 1 (POSITION)
            0x00, 0x00, 0x00, 0x3f, // 0.5 as f32
        ];

        let result = QueryResult::parse_data(&data);
        assert_eq!(result.mode, Mode::Position);
        assert!((result.position - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_query_result_parse_extras() {
        // Reply with mode + two extra registers (0x100 as Float, 0x102 as Int16)
        let data = [
            0x21,             // Reply Int8, count=1
            0x00,             // Register 0 (MODE)
            0x0a,             // Mode::Position (10)
            0x2d,             // Reply Float, count=1
            0x80, 0x02,       // Register 0x100 (varuint)
            0x00, 0x00, 0xc8, 0x42, // 100.0 as f32
            0x25,             // Reply Int16, count=1
            0x82, 0x02,       // Register 0x102 (varuint)
            0x39, 0x30,       // 12345 as i16
        ];

        let result = QueryResult::parse_data(&data);
        assert_eq!(result.mode, Mode::Position);
        assert!(result.extra[0].is_some());
        let e0 = result.extra[0].unwrap();
        assert_eq!(e0.register, 0x100);
        assert!((e0.value - 100.0).abs() < 0.001);
        assert!(result.extra[1].is_some());
        let e1 = result.extra[1].unwrap();
        assert_eq!(e1.register, 0x102);
        assert_eq!(e1.value as i32, 12345);
        assert!(result.extra[2].is_none());

        assert!((result.get_extra(0x100).unwrap() - 100.0).abs() < 0.001);
        assert_eq!(result.get_extra(0x102).unwrap() as i32, 12345);
        assert!(result.get_extra(0x999).is_none());
    }
}
