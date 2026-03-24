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

//! Controller API for moteus devices.

use crate::command_types::Command;
use crate::device_address::DeviceAddress;
use moteus_protocol::command::{
    AuxPwmCommand, AuxPwmFormat, BrakeCommand, CurrentCommand, CurrentFormat,
    OutputExactCommand, OutputNearestCommand, PositionCommand, PositionFormat,
    RecapturePositionVelocityCommand, RequireReindexCommand, SetTrimCommand,
    StayWithinCommand, StayWithinFormat, StopCommand, VFOCCommand, VFOCFormat,
    ZeroVelocityCommand, ZeroVelocityFormat,
};
use moteus_protocol::query::{QueryFormat, QueryResult};
use moteus_protocol::{CanFdFrame, Register, Resolution, WriteCanData, WriteCombiner};

/// Controller for a single moteus device.
///
/// This struct provides methods for building commands (make_* methods)
/// that can be sent to a moteus controller. The actual transport is handled
/// separately, allowing the same Controller to be used with different backends.
///
/// # Example
///
/// ```rust
/// use moteus::Controller;
///
/// let controller = Controller::new(1);
///
/// // Build a stop command
/// let cmd = controller.make_stop(false);
/// let frame = cmd.into_frame(); // Convert to wire frame for transport
/// ```
pub struct Controller {
    /// Device address (CAN ID or UUID)
    pub address: DeviceAddress,
    /// CAN bus ID used for framing (resolved from address).
    /// For UUID-only addresses this is 0x7F (broadcast).
    pub id: u8,
    /// Source CAN ID for this client
    pub source_id: u8,
    /// 13-bit CAN prefix
    pub can_prefix: u16,
    /// Query resolution format
    pub query_format: QueryFormat,
    /// Position command format
    pub position_format: PositionFormat,
    /// VFOC command format
    pub vfoc_format: VFOCFormat,
    /// Current command format
    pub current_format: CurrentFormat,
    /// Stay-within command format
    pub stay_within_format: StayWithinFormat,
    /// Zero-velocity command format
    pub zero_velocity_format: ZeroVelocityFormat,
    /// AUX PWM command format
    pub aux_pwm_format: AuxPwmFormat,

    /// UUID prefix data prepended to every command frame when addressing
    /// by UUID. Empty when addressing by CAN ID.
    uuid_prefix_data: Vec<u8>,
}

impl Controller {
    /// Creates a new Controller for the given device address.
    ///
    /// # Arguments
    /// * `address` - Device address (CAN ID, UUID, or both). Integers are
    ///               automatically converted to CAN ID addresses. UUID-only
    ///               addresses use 0x7F (broadcast) as the CAN ID, with
    ///               UUID mask prefix data prepended to each command frame.
    ///
    /// # Example
    ///
    /// ```rust
    /// use moteus::{Controller, DeviceAddress};
    /// use moteus_protocol::query::QueryFormat;
    ///
    /// // Using integer CAN ID (most common)
    /// let controller = Controller::new(1);
    ///
    /// // Using explicit DeviceAddress
    /// let controller = Controller::new(DeviceAddress::can_id(1));
    ///
    /// // Using UUID address
    /// let uuid = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    ///             0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10];
    /// let controller = Controller::new(DeviceAddress::uuid(uuid));
    ///
    /// // With builder pattern
    /// let controller = Controller::new(1)
    ///     .source_id(0x10)
    ///     .query_format(QueryFormat::comprehensive());
    /// ```
    pub fn new(address: impl Into<DeviceAddress>) -> Self {
        let address = address.into();
        let id = address.as_can_id().unwrap_or(0x7F);

        let uuid_prefix_data = if address.can_id.is_none() {
            if let Some(uuid) = address.as_uuid() {
                crate::transport::make_uuid_prefix(uuid)
            } else {
                Vec::new()
            }
        } else {
            Vec::new()
        };

        Controller {
            address,
            id,
            source_id: 0,
            can_prefix: 0x0000,
            query_format: QueryFormat::default(),
            position_format: PositionFormat::default(),
            vfoc_format: VFOCFormat::default(),
            current_format: CurrentFormat::default(),
            stay_within_format: StayWithinFormat::default(),
            zero_velocity_format: ZeroVelocityFormat::default(),
            aux_pwm_format: AuxPwmFormat::default(),
            uuid_prefix_data,
        }
    }

    /// Sets the source CAN ID (builder pattern).
    pub fn source_id(mut self, id: u8) -> Self {
        self.source_id = id;
        self
    }

    /// Sets the CAN prefix (builder pattern).
    pub fn can_prefix(mut self, prefix: u16) -> Self {
        self.can_prefix = prefix & 0x1fff;
        self
    }

    /// Sets the query format (builder pattern).
    pub fn query_format(mut self, format: QueryFormat) -> Self {
        self.query_format = format;
        self
    }

    /// Sets the position command format (builder pattern).
    pub fn position_format(mut self, format: PositionFormat) -> Self {
        self.position_format = format;
        self
    }

    /// Sets the VFOC command format (builder pattern).
    pub fn vfoc_format(mut self, format: VFOCFormat) -> Self {
        self.vfoc_format = format;
        self
    }

    /// Sets the current command format (builder pattern).
    pub fn current_format(mut self, format: CurrentFormat) -> Self {
        self.current_format = format;
        self
    }

    /// Sets the stay-within command format (builder pattern).
    pub fn stay_within_format(mut self, format: StayWithinFormat) -> Self {
        self.stay_within_format = format;
        self
    }

    /// Sets the zero-velocity command format (builder pattern).
    pub fn zero_velocity_format(mut self, format: ZeroVelocityFormat) -> Self {
        self.zero_velocity_format = format;
        self
    }

    /// Sets the AUX PWM command format (builder pattern).
    pub fn aux_pwm_format(mut self, format: AuxPwmFormat) -> Self {
        self.aux_pwm_format = format;
        self
    }

    /// Prepares a Command with common routing fields set.
    ///
    /// If the device address has a `transport_device` set, it is propagated
    /// to the command's `channel` field for direct transport routing.
    ///
    /// For UUID-addressed controllers, the UUID mask prefix data is
    /// pre-filled into the frame so that subsequent serialization appends
    /// after it.
    pub(crate) fn prepare_command(&self, query: bool) -> Command {
        let mut cmd = Command::new(self.id as i8, self.source_id as i8, self.can_prefix)
            .reply_required(query);
        cmd.channel = self.address.transport_device;
        cmd.address = Some(self.address.clone());

        if query {
            cmd.reply_filter = Some(Command::query_reply_filter());
        }

        if !self.uuid_prefix_data.is_empty() {
            let frame = cmd.frame_mut();
            let len = self.uuid_prefix_data.len();
            frame.data[..len].copy_from_slice(&self.uuid_prefix_data);
            frame.size = len as u8;
        }

        cmd
    }

    // === Query Methods ===

    /// Builds a query-only command.
    ///
    /// Returns a command that requests telemetry data from the controller
    /// without sending any motor commands.
    pub fn make_query(&self) -> Command {
        self.make_query_with_format(&self.query_format)
    }

    /// Builds a query command with a custom format.
    pub fn make_query_with_format(&self, format: &QueryFormat) -> Command {
        let mut cmd = self.prepare_command(true);
        cmd.expected_reply_size = format.serialize(cmd.frame_mut());
        cmd
    }

    /// Parses a query response from a received frame.
    pub fn parse_query(&self, frame: &CanFdFrame) -> QueryResult {
        QueryResult::parse(frame)
    }

    // === Stop/Brake Methods ===

    /// Builds a stop command.
    ///
    /// Stop mode disables the motor output, allowing it to coast.
    ///
    /// # Arguments
    /// * `query` - If true, also request telemetry in the response
    pub fn make_stop(&self, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        StopCommand::serialize(cmd.frame_mut());
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    /// Builds a brake command.
    ///
    /// Brake mode shorts the motor windings for active braking.
    pub fn make_brake(&self, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        BrakeCommand::serialize(cmd.frame_mut());
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    // === Position Mode Methods ===

    /// Builds a position mode command.
    ///
    /// This is the primary control mode for moteus.
    ///
    /// # Arguments
    /// * `cmd` - Position command built with the builder pattern
    /// * `query` - If true, also request telemetry in the response
    ///
    /// # Example
    ///
    /// ```rust
    /// use moteus::Controller;
    /// use moteus_protocol::command::PositionCommand;
    ///
    /// let controller = Controller::new(1);
    /// let cmd = controller.make_position_command(
    ///     &PositionCommand::new().position(0.5).velocity(1.0),
    ///     true,
    /// );
    /// ```
    pub fn make_position_command(&self, pos_cmd: &PositionCommand, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        pos_cmd.serialize(cmd.frame_mut(), &self.position_format);
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    // === Current Mode Methods ===

    /// Builds a current (torque) mode command.
    ///
    /// # Arguments
    /// * `cur_cmd` - Current command built with the builder pattern
    /// * `query` - If true, also request telemetry
    pub fn make_current_command(&self, cur_cmd: &CurrentCommand, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        cur_cmd.serialize(cmd.frame_mut(), &self.current_format);
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    // === VFOC Mode Methods ===

    /// Builds a voltage FOC mode command.
    ///
    /// # Arguments
    /// * `vfoc_cmd` - VFOC command built with the builder pattern
    /// * `query` - If true, also request telemetry
    pub fn make_vfoc_command(&self, vfoc_cmd: &VFOCCommand, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        vfoc_cmd.serialize(cmd.frame_mut(), &self.vfoc_format);
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    // === Stay-Within Mode Methods ===

    /// Builds a stay-within mode command.
    ///
    /// # Arguments
    /// * `sw_cmd` - Stay-within command built with the builder pattern
    /// * `query` - If true, also request telemetry
    pub fn make_stay_within_command(&self, sw_cmd: &StayWithinCommand, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        sw_cmd.serialize(cmd.frame_mut(), &self.stay_within_format);
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    // === Zero-Velocity Mode Methods ===

    /// Builds a zero-velocity mode command.
    ///
    /// # Arguments
    /// * `zv_cmd` - Zero-velocity command built with the builder pattern
    /// * `query` - If true, also request telemetry
    pub fn make_zero_velocity_command(
        &self,
        zv_cmd: &ZeroVelocityCommand,
        query: bool,
    ) -> Command {
        let mut cmd = self.prepare_command(query);
        zv_cmd.serialize(cmd.frame_mut(), &self.zero_velocity_format);
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    // === Output Position Methods ===

    /// Builds a set-output-nearest command.
    ///
    /// Sets the output position to the nearest value matching the given position.
    pub fn make_set_output_nearest(&self, position: f32, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        OutputNearestCommand::new(position).serialize(cmd.frame_mut());
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    /// Builds a set-output-exact command.
    ///
    /// Sets the output position to exactly the given value.
    pub fn make_set_output_exact(&self, position: f32, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        OutputExactCommand::new(position).serialize(cmd.frame_mut());
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    /// Builds a require-reindex command.
    pub fn make_require_reindex(&self, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        RequireReindexCommand::serialize(cmd.frame_mut());
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    /// Builds a recapture-position-velocity command.
    pub fn make_recapture_position_velocity(&self, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        RecapturePositionVelocityCommand::serialize(cmd.frame_mut());
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    // === GPIO Methods ===

    /// Builds a command to read GPIO digital inputs.
    ///
    /// The response will contain `aux1_gpio` and `aux2_gpio` values in the QueryResult.
    /// Each bit represents a pin's state, with bit 0 being pin 0.
    pub fn make_read_gpio(&self) -> Command {
        let mut cmd = self.prepare_command(true);
        let frame = cmd.frame_mut();
        let mut writer = WriteCanData::new(frame);

        // Read GPIO status registers (0x05e and 0x05f)
        let resolutions = [Resolution::Int8, Resolution::Int8];
        let mut combiner = WriteCombiner::new(
            0x10, // Read base
            Register::Aux1GpioStatus.address(),
            &resolutions,
        );

        for _ in 0..resolutions.len() {
            combiner.maybe_write(&mut writer);
        }
        cmd.expected_reply_size = combiner.reply_size();
        cmd
    }

    /// Builds a command to write GPIO digital outputs.
    ///
    /// # Arguments
    /// * `aux1` - Optional value to write to AUX1 GPIO outputs (bit 0 = pin 0)
    /// * `aux2` - Optional value to write to AUX2 GPIO outputs (bit 0 = pin 0)
    /// * `query` - If true, also request telemetry in the response
    ///
    /// # Example
    ///
    /// ```rust
    /// use moteus::Controller;
    ///
    /// let controller = Controller::new(1);
    /// // Set all GPIO outputs to high
    /// let cmd = controller.make_write_gpio(Some(0x7f), Some(0x7f), false);
    /// ```
    pub fn make_write_gpio(&self, aux1: Option<u8>, aux2: Option<u8>, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        {
            let frame = cmd.frame_mut();
            let mut writer = WriteCanData::new(frame);

            // Write to GPIO command registers (0x05c and 0x05d)
            let resolutions = [
                if aux1.is_some() { Resolution::Int8 } else { Resolution::Ignore },
                if aux2.is_some() { Resolution::Int8 } else { Resolution::Ignore },
            ];

            let mut combiner = WriteCombiner::new(
                0x00, // Write base
                Register::Aux1GpioCommand.address(),
                &resolutions,
            );

            if combiner.maybe_write(&mut writer) {
                writer.write_i8(aux1.unwrap_or(0) as i8);
            }
            if combiner.maybe_write(&mut writer) {
                writer.write_i8(aux2.unwrap_or(0) as i8);
            }
        }

        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    /// Result of reading GPIO values.
    ///
    /// Each value is a bitfield where bit N represents pin N's state.
    pub fn parse_gpio(&self, frame: &CanFdFrame) -> GpioResult {
        let result = QueryResult::parse(frame);
        GpioResult {
            aux1: result.aux1_gpio as u8,
            aux2: result.aux2_gpio as u8,
        }
    }

    // === Custom Query Methods ===

    /// Builds a command to query specific registers.
    ///
    /// This allows querying arbitrary registers by specifying them as
    /// (register_address, resolution) pairs.
    ///
    /// # Arguments
    /// * `registers` - Slice of (register address, resolution) tuples to query
    ///
    /// # Example
    ///
    /// ```rust
    /// use moteus::Controller;
    /// use moteus_protocol::{Register, Resolution};
    ///
    /// let controller = Controller::new(1);
    /// // Query position and velocity with specific resolutions
    /// let cmd = controller.make_custom_query(&[
    ///     (Register::Position.address(), Resolution::Float),
    ///     (Register::Velocity.address(), Resolution::Float),
    /// ]);
    /// ```
    pub fn make_custom_query(&self, registers: &[(u16, Resolution)]) -> Command {
        let mut cmd = self.prepare_command(true);

        if registers.is_empty() {
            return cmd;
        }

        let frame = cmd.frame_mut();
        let mut writer = WriteCanData::new(frame);

        // Find min and max register addresses
        let min_addr = registers.iter().map(|(a, _)| *a).min().unwrap();
        let max_addr = registers.iter().map(|(a, _)| *a).max().unwrap();

        // Build resolution array for the register range
        let range_len = (max_addr - min_addr + 1) as usize;
        let mut resolutions = vec![Resolution::Ignore; range_len];
        for (addr, res) in registers {
            let idx = (*addr - min_addr) as usize;
            resolutions[idx] = *res;
        }

        let mut combiner = WriteCombiner::new(
            0x10, // Read base
            min_addr,
            &resolutions,
        );

        for _ in 0..range_len {
            combiner.maybe_write(&mut writer);
        }
        cmd.expected_reply_size = combiner.reply_size();
        cmd
    }

    // === AUX PWM Methods ===

    /// Builds an AUX PWM command.
    ///
    /// Sets PWM duty cycles on the AUX1 and AUX2 ports.
    ///
    /// # Arguments
    /// * `pwm_cmd` - AUX PWM command built with the builder pattern
    /// * `query` - If true, also request telemetry in the response
    ///
    /// # Example
    ///
    /// ```rust
    /// use moteus::Controller;
    /// use moteus_protocol::command::AuxPwmCommand;
    ///
    /// let controller = Controller::new(1);
    /// let cmd = controller.make_aux_pwm(
    ///     &AuxPwmCommand::new().aux1_pwm1(0.5).aux1_pwm2(0.75),
    ///     false,
    /// );
    /// ```
    pub fn make_aux_pwm(&self, pwm_cmd: &AuxPwmCommand, query: bool) -> Command {
        let mut cmd = self.prepare_command(query);
        pwm_cmd.serialize(cmd.frame_mut(), &self.aux_pwm_format);
        if query {
            cmd.expected_reply_size = self.query_format.serialize(cmd.frame_mut());
        }
        cmd
    }

    // === Clock Trim Methods ===

    /// Builds a set-trim command.
    ///
    /// Sets the clock trim value used for timing correction.
    ///
    /// # Arguments
    /// * `trim` - Clock trim value (i32)
    pub fn make_set_trim(&self, trim: i32) -> Command {
        let mut cmd = self.prepare_command(false);
        SetTrimCommand::new(trim).serialize(cmd.frame_mut());
        cmd
    }
}

/// Result of reading GPIO digital inputs.
#[derive(Debug, Clone, Copy, Default)]
pub struct GpioResult {
    /// AUX1 GPIO input state (bit N = pin N)
    pub aux1: u8,
    /// AUX2 GPIO input state (bit N = pin N)
    pub aux2: u8,
}

impl Default for Controller {
    fn default() -> Self {
        Self::new(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_controller() {
        let controller = Controller::new(5);
        assert_eq!(controller.id, 5);
        assert_eq!(controller.address, DeviceAddress::can_id(5));
        assert_eq!(controller.source_id, 0);
    }

    #[test]
    fn test_new_controller_with_device_address() {
        let controller = Controller::new(DeviceAddress::can_id(7));
        assert_eq!(controller.id, 7);
        assert_eq!(controller.address, DeviceAddress::can_id(7));
    }

    #[test]
    fn test_make_stop() {
        let controller = Controller::new(1);
        let cmd = controller.make_stop(false);

        assert_eq!(cmd.destination, 1);
        assert!(!cmd.reply_required);
        assert!(cmd.data_size() >= 3);
    }

    #[test]
    fn test_make_stop_into_frame() {
        let controller = Controller::new(1);
        let frame = controller.make_stop(false).into_frame();

        // No reply: arb_id = 0x0001
        assert_eq!(frame.arbitration_id, 0x0001);
        assert!(frame.size >= 3);
    }

    #[test]
    fn test_make_position_command() {
        let controller = Controller::new(1);
        let pos_cmd = PositionCommand::new().position(0.5).velocity(1.0);
        let cmd = controller.make_position_command(&pos_cmd, true);

        assert_eq!(cmd.destination, 1);
        assert!(cmd.reply_required);
        assert!(cmd.data_size() > 3);
    }

    #[test]
    fn test_make_query() {
        let controller = Controller::new(1);
        let cmd = controller.make_query();

        assert!(cmd.reply_required);
        assert!(cmd.expected_reply_size > 0);
    }

    #[test]
    fn test_make_query_into_frame() {
        let controller = Controller::new(1);
        let frame = controller.make_query().into_frame();

        // Reply required: arb_id = 0x8001
        assert_eq!(frame.arbitration_id, 0x8001);
    }

    #[test]
    fn test_new_controller_uuid_only() {
        let uuid = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10];
        let controller = Controller::new(DeviceAddress::uuid(uuid));
        assert_eq!(controller.id, 0x7F);
        assert!(!controller.uuid_prefix_data.is_empty());
    }

    #[test]
    fn test_uuid_controller_frame_has_prefix() {
        let uuid = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10];
        let controller = Controller::new(DeviceAddress::uuid(uuid));

        let cmd = controller.make_stop(false);
        let frame = cmd.into_frame();

        // Destination is 0x7F (broadcast)
        assert_eq!(frame.arbitration_id & 0x7F, 0x7F);

        // Frame data should start with UUID prefix, followed by stop command
        let prefix_len = controller.uuid_prefix_data.len();
        assert!(frame.size as usize > prefix_len);
        assert_eq!(&frame.data[..prefix_len], &controller.uuid_prefix_data[..]);
    }

    #[test]
    fn test_uuid_controller_query_frame() {
        let uuid = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10];
        let controller = Controller::new(DeviceAddress::uuid(uuid));

        let cmd = controller.make_query();
        let frame = cmd.into_frame();

        // Reply required with broadcast: arb_id = 0x807F
        assert_eq!(frame.arbitration_id, 0x807F);

        // Frame should have UUID prefix followed by query
        let prefix_len = controller.uuid_prefix_data.len();
        assert!(frame.size as usize > prefix_len);
    }

    #[test]
    fn test_can_id_controller_no_uuid_prefix() {
        let controller = Controller::new(1);
        assert!(controller.uuid_prefix_data.is_empty());

        let cmd = controller.make_stop(false);
        // Stop command starts at byte 0 (no prefix)
        assert!(cmd.data_size() >= 3);
    }

    #[test]
    fn test_uuid_controller_with_transport_device() {
        let addr = DeviceAddress::uuid([0x01, 0x02, 0x03, 0x04])
            .with_transport_device(2);
        let controller = Controller::new(addr);
        assert_eq!(controller.id, 0x7F);

        let cmd = controller.make_stop(false);
        let frame = cmd.into_frame();
        assert_eq!(frame.channel, Some(2));
    }
}
