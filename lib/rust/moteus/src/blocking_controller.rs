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

//! Blocking controller API for moteus devices.
//!
//! This module provides the `BlockingController` which combines frame building
//! with a synchronous transport for direct, blocking communication with moteus
//! controllers. This is the simplest API to use when async is not needed.
//!
//! # Auto-Discovery
//!
//! The controller can automatically discover and use available transports:
//!
//! ```ignore
//! use moteus::BlockingController;
//! use moteus::command::PositionCommand;
//!
//! // Auto-discover transport (simplest usage)
//! let mut ctrl = BlockingController::new(1);
//! ctrl.set_stop()?;
//! ```
//!
//! # Explicit Transport
//!
//! You can also specify a transport explicitly:
//!
//! ```ignore
//! use moteus::{BlockingController, transport::socketcan::SocketCan};
//!
//! let transport = SocketCan::new("can0")?;
//! let mut ctrl = BlockingController::new(1)
//!     .transport(transport);
//! ```

use crate::command_ext::MaybeQuery;
use crate::controller::Controller;
use crate::device_address::DeviceAddress;
use crate::error::{Error, Result};
use crate::transport::factory::TransportOptions;
use crate::transport::singleton::get_singleton_transport;
use crate::transport::transaction::Request;
use crate::transport::{Transport, TransportOps};
use moteus_protocol::command::{
    AuxPwmCommand, CurrentCommand, PositionCommand, StayWithinCommand, VFOCCommand,
    ZeroVelocityCommand,
};
use moteus_protocol::Resolution;
use moteus_protocol::query::{QueryFormat, QueryResult};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Holds the transport backing a `BlockingController`.
///
/// Transport resolution is lazy: when created with `Deferred`, the actual
/// transport is not created until the first operation that requires it.
/// This matches the Python and C++ library behavior.
pub(crate) enum TransportHolder {
    /// Transport not yet resolved; will auto-discover on first use.
    Deferred(Option<TransportOptions>),
    /// User-provided explicit transport (type-erased).
    Explicit(Box<dyn TransportOps + Send>),
    /// Global singleton transport.
    Singleton(Arc<Mutex<Transport>>),
}

impl TransportHolder {
    /// Ensure the transport is resolved, performing auto-discovery if needed.
    fn resolve(&mut self) -> Result<()> {
        if let TransportHolder::Deferred(opts) = self {
            let transport = get_singleton_transport(opts.as_ref())?;
            *self = TransportHolder::Singleton(transport);
        }
        Ok(())
    }

    /// Execute a cycle on the held transport, resolving lazily if needed.
    pub(crate) fn cycle(&mut self, requests: &mut [Request]) -> Result<()> {
        self.resolve()?;
        match self {
            TransportHolder::Explicit(t) => t.cycle(requests),
            TransportHolder::Singleton(t) => {
                let mut guard = t.lock().map_err(|_| {
                    Error::Protocol("Transport mutex poisoned".to_string())
                })?;
                guard.cycle(requests)
            }
            TransportHolder::Deferred(_) => unreachable!(),
        }
    }

    /// Set the timeout on the held transport.
    fn set_timeout(&mut self, timeout: Duration) {
        match self {
            TransportHolder::Deferred(Some(opts)) => opts.timeout = timeout,
            TransportHolder::Deferred(None) => {
                let mut opts = TransportOptions::new();
                opts.timeout = timeout;
                *self = TransportHolder::Deferred(Some(opts));
            }
            TransportHolder::Explicit(t) => t.set_timeout(timeout),
            TransportHolder::Singleton(t) => {
                if let Ok(mut guard) = t.lock() {
                    guard.set_timeout(timeout);
                }
            }
        }
    }

    /// Get the timeout from the held transport.
    fn timeout(&self) -> Duration {
        match self {
            TransportHolder::Deferred(Some(opts)) => opts.timeout,
            TransportHolder::Deferred(None) => crate::transport::factory::DEFAULT_TIMEOUT,
            TransportHolder::Explicit(t) => t.timeout(),
            TransportHolder::Singleton(t) => {
                t.lock().map(|g| g.timeout()).unwrap_or(crate::transport::factory::DEFAULT_TIMEOUT)
            }
        }
    }
}

/// A blocking controller for a single moteus device.
///
/// This combines the frame-building capabilities of `Controller` with
/// a synchronous transport for blocking communication. This is the simplest
/// API for controlling a moteus when you don't need async.
///
/// Transport resolution is lazy: auto-discovery happens on the first
/// operation that needs the transport, not at construction time.
///
/// # Example
///
/// ```ignore
/// use moteus::{BlockingController, command::PositionCommand};
///
/// // Create with auto-discovered transport (simplest)
/// let mut ctrl = BlockingController::new(1);
///
/// // Query the controller (transport is discovered here)
/// let result = ctrl.query()?;
/// println!("Position: {}", result.position);
///
/// // Move to a position using builder pattern
/// let result = ctrl.set_position(
///     PositionCommand::new().position(0.5).velocity(1.0)
/// )?;
///
/// // Or with explicit transport
/// use moteus::transport::socketcan::SocketCan;
/// let transport = SocketCan::new("can0")?;
/// let mut ctrl = BlockingController::new(1)
///     .transport(transport);
/// ```
pub struct BlockingController {
    /// Frame builder
    pub controller: Controller,
    /// Transport for communication
    pub(crate) transport: TransportHolder,
}

impl BlockingController {
    /// Creates a new blocking controller.
    ///
    /// Transport is not resolved until the first operation that needs it.
    /// If no explicit transport is provided via `.transport()`, the global
    /// singleton transport will be auto-discovered on first use.
    ///
    /// # Arguments
    /// * `address` - Device address (CAN ID or UUID). Integers are automatically
    ///   converted to CAN ID addresses.
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Using integer CAN ID
    /// let mut ctrl = BlockingController::new(1);
    ///
    /// // Using explicit DeviceAddress
    /// let mut ctrl = BlockingController::new(DeviceAddress::can_id(1));
    /// ```
    pub fn new(address: impl Into<DeviceAddress>) -> Self {
        Self {
            controller: Controller::new(address),
            transport: TransportHolder::Deferred(None),
        }
    }

    /// Creates a controller with specific transport options.
    ///
    /// Transport is not resolved until the first operation that needs it.
    /// The options are used to configure auto-discovery when it occurs.
    ///
    /// # Arguments
    /// * `address` - Device address (CAN ID or UUID). Integers are automatically
    ///   converted to CAN ID addresses.
    /// * `options` - Transport options for device selection and configuration
    ///
    /// # Example
    ///
    /// ```ignore
    /// let opts = TransportOptions::new()
    ///     .socketcan_interfaces(vec!["can0"])
    ///     .timeout(std::time::Duration::from_millis(200));
    /// let mut ctrl = BlockingController::with_options(1, &opts);
    /// ```
    pub fn with_options(address: impl Into<DeviceAddress>, options: &TransportOptions) -> Self {
        Self {
            controller: Controller::new(address),
            transport: TransportHolder::Deferred(Some(options.clone())),
        }
    }

    /// Creates a blocking controller with a pre-configured Controller.
    ///
    /// Transport is not resolved until the first operation that needs it.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use moteus::{BlockingController, Controller};
    /// use moteus::query::QueryFormat;
    ///
    /// let ctrl = BlockingController::with_controller(
    ///     Controller::new(1)
    ///         .query_format(QueryFormat::comprehensive())
    ///         .source_id(0x10),
    /// );
    /// ```
    pub fn with_controller(controller: Controller) -> Self {
        Self {
            controller,
            transport: TransportHolder::Deferred(None),
        }
    }

    /// Set an explicit transport (builder pattern).
    ///
    /// This bypasses auto-discovery and uses the provided transport directly.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use moteus::{BlockingController, transport::socketcan::SocketCan};
    ///
    /// let transport = SocketCan::new("can0")?;
    /// let mut ctrl = BlockingController::new(1)
    ///     .transport(transport);
    /// ```
    pub fn transport<T: TransportOps + Send + 'static>(mut self, t: T) -> Self {
        self.transport = TransportHolder::Explicit(Box::new(t));
        self
    }

    /// Sets the communication timeout.
    pub fn set_timeout(&mut self, timeout: Duration) {
        self.transport.set_timeout(timeout);
    }

    /// Returns the current timeout.
    pub fn timeout(&self) -> Duration {
        self.transport.timeout()
    }

    // === Query Methods ===

    /// Queries the controller for current state.
    pub fn query(&mut self) -> Result<QueryResult> {
        let mut requests = [Request::from_command(self.controller.make_query())];
        self.transport.cycle(&mut requests)?;

        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Queries with a custom format.
    pub fn query_with_format(&mut self, format: &QueryFormat) -> Result<QueryResult> {
        let mut requests = [Request::from_command(self.controller.make_query_with_format(format))];
        self.transport.cycle(&mut requests)?;

        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    // === Stop/Brake Methods ===

    /// Sends a stop command and returns the query result.
    pub fn set_stop(&mut self) -> Result<QueryResult> {
        let mut requests = [Request::from_command(self.controller.make_stop(true))];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Sends a stop command without waiting for response.
    pub fn set_stop_no_query(&mut self) -> Result<()> {
        let mut requests = [Request::from_command(self.controller.make_stop(false))];
        self.transport.cycle(&mut requests)?;
        Ok(())
    }

    /// Sends a brake command and returns the query result.
    pub fn set_brake(&mut self) -> Result<QueryResult> {
        let mut requests = [Request::from_command(self.controller.make_brake(true))];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Sends a brake command without waiting for response.
    pub fn set_brake_no_query(&mut self) -> Result<()> {
        let mut requests = [Request::from_command(self.controller.make_brake(false))];
        self.transport.cycle(&mut requests)?;
        Ok(())
    }

    // === Position Mode Methods ===

    /// Commands position mode and returns the query result.
    ///
    /// # Arguments
    /// * `cmd` - Position command built with the builder pattern, optionally
    ///   with `.with_query()` to override the query format
    ///
    /// # Example
    ///
    /// ```ignore
    /// use moteus::{BlockingController, CommandExt};
    /// use moteus::command::PositionCommand;
    /// use moteus::query::QueryFormat;
    ///
    /// // Simple position command
    /// let result = ctrl.set_position(
    ///     PositionCommand::new().position(0.5)
    /// )?;
    ///
    /// // With velocity
    /// let result = ctrl.set_position(
    ///     PositionCommand::new().position(0.5).velocity(1.0)
    /// )?;
    ///
    /// // With full control
    /// let result = ctrl.set_position(
    ///     PositionCommand::new()
    ///         .position(0.5)
    ///         .velocity(1.0)
    ///         .kp_scale(0.8)
    ///         .maximum_torque(2.0)
    /// )?;
    ///
    /// // With query format override
    /// let result = ctrl.set_position(
    ///     PositionCommand::new()
    ///         .position(0.5)
    ///         .with_query(QueryFormat::comprehensive())
    /// )?;
    /// ```
    pub fn set_position(
        &mut self,
        cmd: impl Into<MaybeQuery<PositionCommand>>,
    ) -> Result<QueryResult> {
        let maybe = cmd.into();
        let (command, query_override) = maybe.into_parts();
        let query_format = query_override.as_ref().unwrap_or(&self.controller.query_format);

        let mut cmd = self.controller.prepare_command(true);
        command.serialize(cmd.frame_mut(), &self.controller.position_format);
        cmd.expected_reply_size = query_format.serialize(cmd.frame_mut());

        let mut requests = [Request::from_command(cmd)];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Commands position mode without waiting for response.
    ///
    /// This is a bandwidth optimization for when you don't need feedback.
    pub fn set_position_no_query(&mut self, cmd: &PositionCommand) -> Result<()> {
        let mut requests = [Request::from_command(self.controller.make_position_command(cmd, false))];
        self.transport.cycle(&mut requests)?;
        Ok(())
    }

    /// Waits for a position move to complete.
    ///
    /// Polls the controller until trajectory_complete is true or timeout.
    ///
    /// # Arguments
    /// * `poll_interval` - How often to poll
    /// * `timeout` - Maximum time to wait
    pub fn wait_for_trajectory_complete(
        &mut self,
        poll_interval: Duration,
        timeout: Duration,
    ) -> Result<QueryResult> {
        let start = std::time::Instant::now();

        loop {
            let result = self.query()?;

            if result.trajectory_complete {
                return Ok(result);
            }

            if start.elapsed() > timeout {
                return Err(Error::Timeout);
            }

            std::thread::sleep(poll_interval);
        }
    }

    // === Current Mode Methods ===

    /// Commands current (torque) mode and returns the query result.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use moteus::command::CurrentCommand;
    ///
    /// let result = ctrl.set_current(
    ///     CurrentCommand::new().q_current(0.5).d_current(0.0)
    /// )?;
    /// ```
    pub fn set_current(
        &mut self,
        cmd: impl Into<MaybeQuery<CurrentCommand>>,
    ) -> Result<QueryResult> {
        let maybe = cmd.into();
        let (command, query_override) = maybe.into_parts();
        let query_format = query_override.as_ref().unwrap_or(&self.controller.query_format);

        let mut cmd = self.controller.prepare_command(true);
        command.serialize(cmd.frame_mut(), &self.controller.current_format);
        cmd.expected_reply_size = query_format.serialize(cmd.frame_mut());

        let mut requests = [Request::from_command(cmd)];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Commands current mode without waiting for response.
    pub fn set_current_no_query(&mut self, cmd: &CurrentCommand) -> Result<()> {
        let mut requests = [Request::from_command(self.controller.make_current_command(cmd, false))];
        self.transport.cycle(&mut requests)?;
        Ok(())
    }

    // === VFOC Mode Methods ===

    /// Commands voltage FOC mode and returns the query result.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use moteus::command::VFOCCommand;
    ///
    /// let result = ctrl.set_vfoc(
    ///     VFOCCommand::new().theta(0.0).voltage(1.0)
    /// )?;
    /// ```
    pub fn set_vfoc(&mut self, cmd: impl Into<MaybeQuery<VFOCCommand>>) -> Result<QueryResult> {
        let maybe = cmd.into();
        let (command, query_override) = maybe.into_parts();
        let query_format = query_override.as_ref().unwrap_or(&self.controller.query_format);

        let mut cmd = self.controller.prepare_command(true);
        command.serialize(cmd.frame_mut(), &self.controller.vfoc_format);
        cmd.expected_reply_size = query_format.serialize(cmd.frame_mut());

        let mut requests = [Request::from_command(cmd)];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Commands VFOC mode without waiting for response.
    pub fn set_vfoc_no_query(&mut self, cmd: &VFOCCommand) -> Result<()> {
        let mut requests = [Request::from_command(self.controller.make_vfoc_command(cmd, false))];
        self.transport.cycle(&mut requests)?;
        Ok(())
    }

    // === Stay-Within Mode Methods ===

    /// Commands stay-within mode and returns the query result.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use moteus::command::StayWithinCommand;
    ///
    /// let result = ctrl.set_stay_within(
    ///     StayWithinCommand::new()
    ///         .lower_bound(-0.5)
    ///         .upper_bound(0.5)
    ///         .maximum_torque(2.0)
    /// )?;
    /// ```
    pub fn set_stay_within(
        &mut self,
        cmd: impl Into<MaybeQuery<StayWithinCommand>>,
    ) -> Result<QueryResult> {
        let maybe = cmd.into();
        let (command, query_override) = maybe.into_parts();
        let query_format = query_override.as_ref().unwrap_or(&self.controller.query_format);

        let mut cmd = self.controller.prepare_command(true);
        command.serialize(cmd.frame_mut(), &self.controller.stay_within_format);
        cmd.expected_reply_size = query_format.serialize(cmd.frame_mut());

        let mut requests = [Request::from_command(cmd)];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Commands stay-within mode without waiting for response.
    pub fn set_stay_within_no_query(&mut self, cmd: &StayWithinCommand) -> Result<()> {
        let mut requests = [Request::from_command(self.controller.make_stay_within_command(cmd, false))];
        self.transport.cycle(&mut requests)?;
        Ok(())
    }

    // === Zero-Velocity Mode Methods ===

    /// Commands zero-velocity mode (hold position with damping) and returns the query result.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use moteus::command::ZeroVelocityCommand;
    ///
    /// // With default kd_scale
    /// let result = ctrl.set_zero_velocity(ZeroVelocityCommand::new())?;
    ///
    /// // With custom kd_scale
    /// let result = ctrl.set_zero_velocity(
    ///     ZeroVelocityCommand::new().kd_scale(0.5)
    /// )?;
    /// ```
    pub fn set_zero_velocity(
        &mut self,
        cmd: impl Into<MaybeQuery<ZeroVelocityCommand>>,
    ) -> Result<QueryResult> {
        let maybe = cmd.into();
        let (command, query_override) = maybe.into_parts();
        let query_format = query_override.as_ref().unwrap_or(&self.controller.query_format);

        let mut cmd = self.controller.prepare_command(true);
        command.serialize(cmd.frame_mut(), &self.controller.zero_velocity_format);
        cmd.expected_reply_size = query_format.serialize(cmd.frame_mut());

        let mut requests = [Request::from_command(cmd)];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Commands zero-velocity mode without waiting for response.
    pub fn set_zero_velocity_no_query(&mut self, cmd: &ZeroVelocityCommand) -> Result<()> {
        let mut requests = [Request::from_command(self.controller.make_zero_velocity_command(cmd, false))];
        self.transport.cycle(&mut requests)?;
        Ok(())
    }

    // === Output Position Methods ===

    /// Sets output position to nearest value.
    pub fn set_output_nearest(&mut self, position: f32) -> Result<QueryResult> {
        let mut requests = [Request::from_command(self.controller.make_set_output_nearest(position, true))];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Sets output position to exact value.
    pub fn set_output_exact(&mut self, position: f32) -> Result<QueryResult> {
        let mut requests = [Request::from_command(self.controller.make_set_output_exact(position, true))];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Requires re-indexing of the encoder.
    pub fn set_require_reindex(&mut self) -> Result<QueryResult> {
        let mut requests = [Request::from_command(self.controller.make_require_reindex(true))];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Recaptures position and velocity.
    pub fn set_recapture_position_velocity(&mut self) -> Result<QueryResult> {
        let mut requests = [Request::from_command(self.controller.make_recapture_position_velocity(true))];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    // === GPIO Methods ===

    /// Reads GPIO digital inputs from both AUX ports.
    ///
    /// Returns a tuple of (aux1, aux2) where each byte represents pin states.
    /// Bit N corresponds to pin N's state.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut ctrl = BlockingController::new(1);
    /// let (aux1, aux2) = ctrl.read_gpio()?;
    ///
    /// // Check individual pins
    /// if aux1 & 0x01 != 0 {
    ///     println!("AUX1 Pin 0 is HIGH");
    /// }
    /// ```
    pub fn read_gpio(&mut self) -> Result<(u8, u8)> {
        let mut requests = [Request::from_command(self.controller.make_read_gpio())];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| {
                let gpio = self.controller.parse_gpio(&f);
                (gpio.aux1, gpio.aux2)
            })
            .ok_or(Error::NoResponse)
    }

    /// Writes GPIO digital outputs.
    ///
    /// # Arguments
    /// * `aux1` - Optional value for AUX1 GPIO outputs (bit N = pin N)
    /// * `aux2` - Optional value for AUX2 GPIO outputs (bit N = pin N)
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut ctrl = BlockingController::new(1);
    ///
    /// // Set all GPIO outputs to high
    /// ctrl.set_write_gpio(Some(0x7f), Some(0x7f))?;
    ///
    /// // Set only AUX1 outputs
    /// ctrl.set_write_gpio(Some(0x1f), None)?;
    /// ```
    pub fn set_write_gpio(&mut self, aux1: Option<u8>, aux2: Option<u8>) -> Result<()> {
        let mut requests = [Request::from_command(self.controller.make_write_gpio(aux1, aux2, false))];
        self.transport.cycle(&mut requests)?;
        Ok(())
    }

    /// Writes GPIO digital outputs and returns query result.
    pub fn set_write_gpio_query(
        &mut self,
        aux1: Option<u8>,
        aux2: Option<u8>,
    ) -> Result<QueryResult> {
        let mut requests = [Request::from_command(self.controller.make_write_gpio(aux1, aux2, true))];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    // === Custom Query Methods ===

    /// Queries specific registers by address.
    ///
    /// This allows querying arbitrary registers by specifying them as
    /// (register_address, resolution) pairs.
    ///
    /// # Arguments
    /// * `registers` - Slice of (register address, resolution) tuples to query
    ///
    /// # Example
    ///
    /// ```ignore
    /// use moteus::{BlockingController, Register, Resolution};
    ///
    /// let mut ctrl = BlockingController::new(1);
    /// let result = ctrl.custom_query(&[
    ///     (Register::Position.address(), Resolution::Float),
    ///     (Register::Velocity.address(), Resolution::Float),
    /// ])?;
    /// ```
    pub fn custom_query(&mut self, registers: &[(u16, Resolution)]) -> Result<QueryResult> {
        let mut requests = [Request::from_command(self.controller.make_custom_query(registers))];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    // === AUX PWM Methods ===

    /// Sets AUX PWM outputs and returns query result.
    ///
    /// # Arguments
    /// * `cmd` - AUX PWM command built with the builder pattern
    ///
    /// # Example
    ///
    /// ```ignore
    /// use moteus::BlockingController;
    /// use moteus::command::AuxPwmCommand;
    ///
    /// let mut ctrl = BlockingController::new(1);
    /// let result = ctrl.set_aux_pwm(
    ///     &AuxPwmCommand::new().aux1_pwm1(0.5).aux1_pwm2(0.75)
    /// )?;
    /// ```
    pub fn set_aux_pwm(&mut self, cmd: &AuxPwmCommand) -> Result<QueryResult> {
        let mut requests = [Request::from_command(self.controller.make_aux_pwm(cmd, true))];
        self.transport.cycle(&mut requests)?;
        requests[0]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|f| self.controller.parse_query(&f))
            .ok_or(Error::NoResponse)
    }

    /// Sets AUX PWM outputs without waiting for response.
    pub fn set_aux_pwm_no_query(&mut self, cmd: &AuxPwmCommand) -> Result<()> {
        let mut requests = [Request::from_command(self.controller.make_aux_pwm(cmd, false))];
        self.transport.cycle(&mut requests)?;
        Ok(())
    }

    // === Clock Trim Methods ===

    /// Sets the clock trim value.
    pub fn set_trim(&mut self, trim: i32) -> Result<()> {
        let mut requests = [Request::from_command(self.controller.make_set_trim(trim))];
        self.transport.cycle(&mut requests)?;
        Ok(())
    }

    // === Position Wait Complete Methods ===

    /// Commands position mode and waits for trajectory completion.
    ///
    /// Unlike `wait_for_trajectory_complete`, this re-sends the position command
    /// each cycle until the trajectory is complete.
    ///
    /// # Arguments
    /// * `cmd` - Position command built with the builder pattern
    /// * `poll_interval` - How often to poll
    /// * `timeout` - Maximum time to wait
    ///
    /// # Example
    ///
    /// ```ignore
    /// use moteus::BlockingController;
    /// use moteus::command::PositionCommand;
    /// use std::time::Duration;
    ///
    /// let mut ctrl = BlockingController::new(1);
    /// let result = ctrl.set_position_wait_complete(
    ///     &PositionCommand::new().position(0.5).stop_position(0.5),
    ///     Duration::from_millis(25),   // poll every 25ms
    ///     Duration::from_secs(5),      // timeout after 5 seconds
    /// )?;
    /// ```
    pub fn set_position_wait_complete(
        &mut self,
        cmd: &PositionCommand,
        poll_interval: Duration,
        timeout: Duration,
    ) -> Result<QueryResult> {
        let start = std::time::Instant::now();

        // We need trajectory_complete in the response
        let mut query_format = self.controller.query_format.clone();
        query_format.trajectory_complete = moteus_protocol::Resolution::Int8;
        if query_format.mode == moteus_protocol::Resolution::Ignore {
            query_format.mode = moteus_protocol::Resolution::Int8;
        }
        if query_format.fault == moteus_protocol::Resolution::Ignore {
            query_format.fault = moteus_protocol::Resolution::Int8;
        }

        // Need a few successful reads before declaring complete
        let mut success_count: i32 = 2;

        loop {
            // Send position command with query
            let mut command = self.controller.prepare_command(true);
            cmd.serialize(command.frame_mut(), &self.controller.position_format);
            command.expected_reply_size = query_format.serialize(command.frame_mut());

            let mut requests = [Request::from_command(command)];
            self.transport.cycle(&mut requests)?;
            let result = requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f));

            if let Some(ref r) = result {
                success_count = success_count.saturating_sub(1);

                // Check for faults
                if r.mode == moteus_protocol::Mode::Fault
                    || r.mode == moteus_protocol::Mode::Timeout
                {
                    return Err(Error::Fault {
                        mode: r.mode as u8,
                        code: r.fault,
                    });
                }

                // Check if trajectory is complete
                if success_count == 0 && r.trajectory_complete {
                    return result.ok_or(Error::NoResponse);
                }
            }

            if start.elapsed() > timeout {
                return Err(Error::Timeout);
            }

            std::thread::sleep(poll_interval);
        }
    }

    // === Transport Methods ===

    /// Flushes any pending data from the transport.
    ///
    /// This is useful when recovering from errors or resynchronizing
    /// with the device.
    pub fn flush_transport(&mut self) -> Result<()> {
        // Attempt a read with a short timeout to clear any pending data
        let old_timeout = self.transport.timeout();
        self.transport.set_timeout(Duration::from_millis(20));
        let mut requests: [Request; 0] = [];
        let _ = self.transport.cycle(&mut requests); // Ignore any errors/responses
        self.transport.set_timeout(old_timeout);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::NullTransport;

    #[test]
    fn test_new_is_infallible() {
        // Construction never fails — transport is resolved lazily
        let _ctrl = BlockingController::new(1);
    }

    #[test]
    fn test_with_options_is_infallible() {
        let opts = TransportOptions::new().timeout(Duration::from_millis(200));
        let _ctrl = BlockingController::with_options(1, &opts);
    }

    #[test]
    fn test_with_controller_is_infallible() {
        let _ctrl = BlockingController::with_controller(Controller::new(1));
    }

    #[test]
    fn test_explicit_transport() {
        let mut ctrl = BlockingController::new(1)
            .transport(NullTransport::new());
        assert_eq!(ctrl.controller.id, 1);

        // NullTransport returns no responses, so we get NoResponse error
        let result = ctrl.set_stop();
        assert!(result.is_err());
    }

    #[test]
    fn test_timeout() {
        let mut ctrl = BlockingController::new(1)
            .transport(NullTransport::new());

        // Default timeout
        assert_eq!(ctrl.timeout(), Duration::from_millis(100));

        // Set new timeout
        ctrl.set_timeout(Duration::from_millis(500));
        assert_eq!(ctrl.timeout(), Duration::from_millis(500));
    }

    #[test]
    fn test_deferred_timeout() {
        // Timeout can be set before transport is resolved
        let mut ctrl = BlockingController::new(1);
        assert_eq!(ctrl.timeout(), Duration::from_millis(100));
        ctrl.set_timeout(Duration::from_millis(500));
        assert_eq!(ctrl.timeout(), Duration::from_millis(500));
    }

    #[test]
    fn test_set_position_no_query() {
        let mut ctrl = BlockingController::new(1)
            .transport(NullTransport::new());

        let result = ctrl.set_position_no_query(&PositionCommand::new().position(0.5));
        assert!(result.is_ok());
    }
}
