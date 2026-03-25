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

//! Async controller API for moteus devices (requires `tokio` feature).
//!
//! This module provides the `AsyncController` which combines frame building
//! with a true async transport for direct communication with moteus controllers.
//!
//! # Auto-Discovery
//!
//! The controller can automatically discover and use available transports:
//!
//! ```no_run
//! use moteus::AsyncController;
//! use moteus::command::PositionCommand;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), moteus::Error> {
//!     let mut ctrl = AsyncController::new(1).await?;
//!     ctrl.set_stop().await?;  // transport discovered here
//!     ctrl.set_position(PositionCommand::new().position(0.5)).await?;
//!     Ok(())
//! }
//! ```
//!
//! # Custom Transport Options
//!
//! ```no_run
//! use moteus::AsyncController;
//! use moteus::transport::async_factory::AsyncTransportOptions;
//!
//! # async fn example() -> Result<(), moteus::Error> {
//! let opts = AsyncTransportOptions::new();
//! let mut ctrl = AsyncController::with_options(1, &opts).await?;
//! # Ok(())
//! # }
//! ```

#![cfg(feature = "tokio")]

use crate::command_ext::MaybeQuery;
use crate::controller::Controller;
use crate::device_address::DeviceAddress;
use crate::error::{Error, Result};
use crate::transport::async_singleton::get_async_singleton_transport;
use crate::transport::async_transport::{AsyncTransport, AsyncTransportOps, BoxFuture};
use crate::transport::transaction::Request;
use moteus_protocol::command::{
    AuxPwmCommand, CurrentCommand, PositionCommand, StayWithinCommand, VFOCCommand,
    ZeroVelocityCommand,
};
use moteus_protocol::query::{QueryFormat, QueryResult};
use moteus_protocol::Resolution;
use std::sync::Arc;
use tokio::sync::Mutex;

/// An async controller for a single moteus device.
///
/// This combines the frame-building capabilities of `Controller` with
/// an async transport for actual communication.
///
/// # Example
///
/// ```no_run
/// use moteus::AsyncController;
/// use moteus::command::PositionCommand;
///
/// #[tokio::main]
/// async fn main() -> Result<(), moteus::Error> {
///     let mut ctrl = AsyncController::new(1).await?;
///
///     // Query the controller
///     let result = ctrl.query().await?;
///     println!("Position: {}", result.position);
///
///     // Move to a position using builder pattern
///     let result = ctrl.set_position(
///         PositionCommand::new().position(0.5).velocity(1.0)
///     ).await?;
///     Ok(())
/// }
/// ```
pub struct AsyncController<T: AsyncTransportOps = Arc<Mutex<AsyncTransport>>> {
    /// Frame builder
    pub controller: Controller,
    /// Async transport
    pub(crate) transport: T,
}

impl<T: AsyncTransportOps> std::fmt::Debug for AsyncController<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AsyncController")
            .field("id", &self.controller.id)
            .field("address", &self.controller.address)
            .finish()
    }
}

// =============================================================================
// Singleton constructors (use global shared transport)
// =============================================================================

impl AsyncController<Arc<Mutex<AsyncTransport>>> {
    /// Creates a new async controller with auto-discovered transport.
    ///
    /// # Arguments
    /// * `address` - Device address (CAN ID or UUID). Integers are automatically
    ///   converted to CAN ID addresses.
    ///
    /// # Errors
    ///
    /// Returns an error if no async transport can be discovered.
    pub async fn new(address: impl Into<DeviceAddress>) -> Result<Self> {
        Ok(Self {
            controller: Controller::new(address),
            transport: get_async_singleton_transport(None).await?,
        })
    }

    /// Creates a controller with specific async transport options.
    ///
    /// # Arguments
    /// * `address` - Device address (CAN ID or UUID). Integers are automatically
    ///   converted to CAN ID addresses.
    /// * `options` - Async transport options for device selection and configuration
    ///
    /// # Errors
    ///
    /// Returns an error if no async transport can be discovered with the given options.
    pub async fn with_options(
        address: impl Into<DeviceAddress>,
        options: &crate::transport::async_factory::AsyncTransportOptions,
    ) -> Result<Self> {
        Ok(Self {
            controller: Controller::new(address),
            transport: get_async_singleton_transport(Some(options)).await?,
        })
    }

    /// Creates an async controller with a pre-configured Controller.
    ///
    /// # Errors
    ///
    /// Returns an error if no async transport can be discovered.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::{AsyncController, Controller};
    /// use moteus::query::QueryFormat;
    ///
    /// # async fn example() -> Result<(), moteus::Error> {
    /// let ctrl = AsyncController::with_controller(
    ///     Controller::new(1).query_format(QueryFormat::comprehensive()),
    /// ).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub async fn with_controller(controller: Controller) -> Result<Self> {
        Ok(Self {
            controller,
            transport: get_async_singleton_transport(None).await?,
        })
    }
}

// =============================================================================
// Generic methods (work with any transport)
// =============================================================================

impl<T: AsyncTransportOps> AsyncController<T> {
    /// Creates an async controller with an explicit transport.
    ///
    /// This bypasses auto-discovery and uses the provided transport directly.
    pub fn with_transport(address: impl Into<DeviceAddress>, transport: T) -> Self {
        Self {
            controller: Controller::new(address),
            transport,
        }
    }

    // === Async Query Methods ===

    /// Queries the controller for current state.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn query(&mut self) -> BoxFuture<'_, Result<QueryResult>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(self.controller.make_query())];
            self.transport.cycle(&mut requests).await?;

            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Queries with a custom format.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn query_with_format<'a>(
        &'a mut self,
        format: &'a QueryFormat,
    ) -> BoxFuture<'a, Result<QueryResult>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_query_with_format(format),
            )];
            self.transport.cycle(&mut requests).await?;

            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    // === Async Stop/Brake Methods ===

    /// Sends a stop command and returns the query result.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_stop(&mut self) -> BoxFuture<'_, Result<QueryResult>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(self.controller.make_stop(true))];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Sends a stop command without waiting for response.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication fails.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_stop_no_query(&mut self) -> BoxFuture<'_, Result<()>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(self.controller.make_stop(false))];
            self.transport.cycle(&mut requests).await?;
            Ok(())
        })
    }

    /// Sends a brake command and returns the query result.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_brake(&mut self) -> BoxFuture<'_, Result<QueryResult>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(self.controller.make_brake(true))];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Sends a brake command without waiting for response.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication fails.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_brake_no_query(&mut self) -> BoxFuture<'_, Result<()>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(self.controller.make_brake(false))];
            self.transport.cycle(&mut requests).await?;
            Ok(())
        })
    }

    // === Async Position Mode Methods ===

    /// Commands position mode and returns the query result.
    ///
    /// # Arguments
    /// * `cmd` - Position command built with the builder pattern, optionally
    ///   with `.with_query()` to override the query format
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::AsyncController;
    /// use moteus::command_ext::CommandExt;
    /// use moteus::command::PositionCommand;
    /// use moteus::query::QueryFormat;
    ///
    /// // Simple position command
    /// let result = ctrl.set_position(
    ///     PositionCommand::new().position(0.5)
    /// ).await?;
    ///
    /// // With query format override
    /// let result = ctrl.set_position(
    ///     PositionCommand::new()
    ///         .position(0.5)
    ///         .with_query(QueryFormat::comprehensive())
    /// ).await?;
    /// ```
    pub fn set_position(
        &mut self,
        cmd: impl Into<MaybeQuery<PositionCommand>>,
    ) -> BoxFuture<'_, Result<QueryResult>> {
        let maybe = cmd.into();
        Box::pin(async move {
            let (command, query_override) = maybe.into_parts();
            let query_format = query_override
                .as_ref()
                .unwrap_or(&self.controller.query_format);

            let mut cmd = self.controller.prepare_command(true);
            command.serialize(cmd.frame_mut(), &self.controller.position_format);
            cmd.expected_reply_size = query_format.serialize(cmd.frame_mut());

            let mut requests = [Request::from_command(cmd)];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Commands position mode without waiting for response.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication fails.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_position_no_query<'a>(
        &'a mut self,
        cmd: &'a PositionCommand,
    ) -> BoxFuture<'a, Result<()>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_position_command(cmd, false),
            )];
            self.transport.cycle(&mut requests).await?;
            Ok(())
        })
    }

    /// Waits for a position move to complete.
    ///
    /// Polls the controller until trajectory_complete is true or timeout.
    ///
    /// # Arguments
    /// * `poll_interval` - How often to poll
    /// * `timeout` - Maximum time to wait
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    /// Returns `Error::Timeout` if the operation exceeds the timeout.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe. If cancelled, the servo continues its current
    /// trajectory until any configured watchdog timeout activates.
    pub fn wait_for_trajectory_complete(
        &mut self,
        poll_interval: std::time::Duration,
        timeout: std::time::Duration,
    ) -> BoxFuture<'_, Result<QueryResult>> {
        Box::pin(async move {
            let start = std::time::Instant::now();
            let timeout_dur = timeout;
            let interval = poll_interval;

            loop {
                let mut requests = [Request::from_command(self.controller.make_query())];
                self.transport.cycle(&mut requests).await?;

                let result = requests[0]
                    .responses
                    .take()
                    .into_iter()
                    .next()
                    .map(|f| self.controller.parse_query(&f))
                    .ok_or(Error::NoResponse)?;

                if result.trajectory_complete {
                    return Ok(result);
                }

                if start.elapsed() > timeout_dur {
                    return Err(Error::Timeout);
                }

                tokio::time::sleep(interval).await;
            }
        })
    }

    // === Async Current Mode Methods ===

    /// Commands current (torque) mode and returns the query result.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::command::CurrentCommand;
    ///
    /// let result = ctrl.set_current(
    ///     CurrentCommand::new().q_current(0.5).d_current(0.0)
    /// ).await?;
    /// ```
    pub fn set_current(
        &mut self,
        cmd: impl Into<MaybeQuery<CurrentCommand>>,
    ) -> BoxFuture<'_, Result<QueryResult>> {
        let maybe = cmd.into();
        Box::pin(async move {
            let (command, query_override) = maybe.into_parts();
            let query_format = query_override
                .as_ref()
                .unwrap_or(&self.controller.query_format);

            let mut cmd = self.controller.prepare_command(true);
            command.serialize(cmd.frame_mut(), &self.controller.current_format);
            cmd.expected_reply_size = query_format.serialize(cmd.frame_mut());

            let mut requests = [Request::from_command(cmd)];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Commands current mode without waiting for response.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication fails.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_current_no_query<'a>(
        &'a mut self,
        cmd: &'a CurrentCommand,
    ) -> BoxFuture<'a, Result<()>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_current_command(cmd, false),
            )];
            self.transport.cycle(&mut requests).await?;
            Ok(())
        })
    }

    // === Async VFOC Mode Methods ===

    /// Commands voltage FOC mode and returns the query result.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::command::VFOCCommand;
    ///
    /// let result = ctrl.set_vfoc(
    ///     VFOCCommand::new().theta(0.0).voltage(1.0)
    /// ).await?;
    /// ```
    pub fn set_vfoc(
        &mut self,
        cmd: impl Into<MaybeQuery<VFOCCommand>>,
    ) -> BoxFuture<'_, Result<QueryResult>> {
        let maybe = cmd.into();
        Box::pin(async move {
            let (command, query_override) = maybe.into_parts();
            let query_format = query_override
                .as_ref()
                .unwrap_or(&self.controller.query_format);

            let mut cmd = self.controller.prepare_command(true);
            command.serialize(cmd.frame_mut(), &self.controller.vfoc_format);
            cmd.expected_reply_size = query_format.serialize(cmd.frame_mut());

            let mut requests = [Request::from_command(cmd)];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Commands VFOC mode without waiting for response.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication fails.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_vfoc_no_query<'a>(&'a mut self, cmd: &'a VFOCCommand) -> BoxFuture<'a, Result<()>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_vfoc_command(cmd, false),
            )];
            self.transport.cycle(&mut requests).await?;
            Ok(())
        })
    }

    // === Async Stay-Within Mode Methods ===

    /// Commands stay-within mode and returns the query result.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::command::StayWithinCommand;
    ///
    /// let result = ctrl.set_stay_within(
    ///     StayWithinCommand::new()
    ///         .lower_bound(-0.5)
    ///         .upper_bound(0.5)
    /// ).await?;
    /// ```
    pub fn set_stay_within(
        &mut self,
        cmd: impl Into<MaybeQuery<StayWithinCommand>>,
    ) -> BoxFuture<'_, Result<QueryResult>> {
        let maybe = cmd.into();
        Box::pin(async move {
            let (command, query_override) = maybe.into_parts();
            let query_format = query_override
                .as_ref()
                .unwrap_or(&self.controller.query_format);

            let mut cmd = self.controller.prepare_command(true);
            command.serialize(cmd.frame_mut(), &self.controller.stay_within_format);
            cmd.expected_reply_size = query_format.serialize(cmd.frame_mut());

            let mut requests = [Request::from_command(cmd)];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Commands stay-within mode without waiting for response.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication fails.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_stay_within_no_query<'a>(
        &'a mut self,
        cmd: &'a StayWithinCommand,
    ) -> BoxFuture<'a, Result<()>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_stay_within_command(cmd, false),
            )];
            self.transport.cycle(&mut requests).await?;
            Ok(())
        })
    }

    // === Async Zero-Velocity Mode Methods ===

    /// Commands zero-velocity mode (hold position with damping) and returns the query result.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::command::ZeroVelocityCommand;
    ///
    /// let result = ctrl.set_zero_velocity(ZeroVelocityCommand::new()).await?;
    ///
    /// // With custom kd_scale
    /// let result = ctrl.set_zero_velocity(
    ///     ZeroVelocityCommand::new().kd_scale(0.5)
    /// ).await?;
    /// ```
    pub fn set_zero_velocity(
        &mut self,
        cmd: impl Into<MaybeQuery<ZeroVelocityCommand>>,
    ) -> BoxFuture<'_, Result<QueryResult>> {
        let maybe = cmd.into();
        Box::pin(async move {
            let (command, query_override) = maybe.into_parts();
            let query_format = query_override
                .as_ref()
                .unwrap_or(&self.controller.query_format);

            let mut cmd = self.controller.prepare_command(true);
            command.serialize(cmd.frame_mut(), &self.controller.zero_velocity_format);
            cmd.expected_reply_size = query_format.serialize(cmd.frame_mut());

            let mut requests = [Request::from_command(cmd)];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Commands zero-velocity mode without waiting for response.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication fails.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_zero_velocity_no_query<'a>(
        &'a mut self,
        cmd: &'a ZeroVelocityCommand,
    ) -> BoxFuture<'a, Result<()>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_zero_velocity_command(cmd, false),
            )];
            self.transport.cycle(&mut requests).await?;
            Ok(())
        })
    }

    // === Async Output Position Methods ===

    /// Sets output position to nearest value.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_output_nearest(&mut self, position: f32) -> BoxFuture<'_, Result<QueryResult>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_set_output_nearest(position, true),
            )];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Sets output position to exact value.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_output_exact(&mut self, position: f32) -> BoxFuture<'_, Result<QueryResult>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_set_output_exact(position, true),
            )];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Requires re-indexing of the encoder.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_require_reindex(&mut self) -> BoxFuture<'_, Result<QueryResult>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_require_reindex(true),
            )];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Recaptures position and velocity.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_recapture_position_velocity(&mut self) -> BoxFuture<'_, Result<QueryResult>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_recapture_position_velocity(true),
            )];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    // === Async GPIO Methods ===

    /// Reads GPIO digital inputs from both AUX ports.
    ///
    /// Returns a tuple of (aux1, aux2) where each byte represents pin states.
    /// Bit N corresponds to pin N's state.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::AsyncController;
    ///
    /// # async fn example() -> Result<(), moteus::Error> {
    /// let mut ctrl = AsyncController::new(1).await?;
    /// let (aux1, aux2) = ctrl.read_gpio().await?;
    ///
    /// // Check individual pins
    /// if aux1 & 0x01 != 0 {
    ///     println!("AUX1 Pin 0 is HIGH");
    /// }
    /// # Ok(())
    /// # }
    /// ```
    pub fn read_gpio(&mut self) -> BoxFuture<'_, Result<(u8, u8)>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(self.controller.make_read_gpio())];
            self.transport.cycle(&mut requests).await?;
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
        })
    }

    /// Writes GPIO digital outputs.
    ///
    /// # Arguments
    /// * `aux1` - Optional value for AUX1 GPIO outputs (bit N = pin N)
    /// * `aux2` - Optional value for AUX2 GPIO outputs (bit N = pin N)
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication fails.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::AsyncController;
    ///
    /// # async fn example() -> Result<(), moteus::Error> {
    /// let mut ctrl = AsyncController::new(1).await?;
    ///
    /// // Set all GPIO outputs to high
    /// ctrl.set_write_gpio(Some(0x7f), Some(0x7f)).await?;
    ///
    /// // Set only AUX1 outputs
    /// ctrl.set_write_gpio(Some(0x1f), None).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn set_write_gpio(
        &mut self,
        aux1: Option<u8>,
        aux2: Option<u8>,
    ) -> BoxFuture<'_, Result<()>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_write_gpio(aux1, aux2, false),
            )];
            self.transport.cycle(&mut requests).await?;
            Ok(())
        })
    }

    /// Writes GPIO digital outputs and returns query result.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_write_gpio_query(
        &mut self,
        aux1: Option<u8>,
        aux2: Option<u8>,
    ) -> BoxFuture<'_, Result<QueryResult>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_write_gpio(aux1, aux2, true),
            )];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    // === Async Custom Query Methods ===

    /// Queries specific registers by address.
    ///
    /// This allows querying arbitrary registers by specifying them as
    /// (register_address, resolution) pairs.
    ///
    /// # Arguments
    /// * `registers` - Slice of (register address, resolution) tuples to query
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::{AsyncController, Register, Resolution};
    ///
    /// # async fn example() -> Result<(), moteus::Error> {
    /// let mut ctrl = AsyncController::new(1).await?;
    /// let result = ctrl.custom_query(&[
    ///     (Register::Position.address(), Resolution::Float),
    ///     (Register::Velocity.address(), Resolution::Float),
    /// ]).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn custom_query<'a>(
        &'a mut self,
        registers: &'a [(u16, Resolution)],
    ) -> BoxFuture<'a, Result<QueryResult>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_custom_query(registers),
            )];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    // === Async AUX PWM Methods ===

    /// Sets AUX PWM outputs and returns query result.
    ///
    /// # Arguments
    /// * `cmd` - AUX PWM command built with the builder pattern
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::NoResponse` if the device does not reply.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::AsyncController;
    /// use moteus::command::AuxPwmCommand;
    ///
    /// # async fn example() -> Result<(), moteus::Error> {
    /// let mut ctrl = AsyncController::new(1).await?;
    /// let result = ctrl.set_aux_pwm(
    ///     &AuxPwmCommand::new().aux1_pwm1(0.5).aux1_pwm2(0.75)
    /// ).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn set_aux_pwm<'a>(
        &'a mut self,
        cmd: &'a AuxPwmCommand,
    ) -> BoxFuture<'a, Result<QueryResult>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_aux_pwm(cmd, true),
            )];
            self.transport.cycle(&mut requests).await?;
            requests[0]
                .responses
                .take()
                .into_iter()
                .next()
                .map(|f| self.controller.parse_query(&f))
                .ok_or(Error::NoResponse)
        })
    }

    /// Sets AUX PWM outputs without waiting for response.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication fails.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_aux_pwm_no_query<'a>(
        &'a mut self,
        cmd: &'a AuxPwmCommand,
    ) -> BoxFuture<'a, Result<()>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(
                self.controller.make_aux_pwm(cmd, false),
            )];
            self.transport.cycle(&mut requests).await?;
            Ok(())
        })
    }

    // === Async Clock Trim Methods ===

    /// Sets the clock trim value.
    ///
    /// # Arguments
    /// * `trim` - Clock trim value (i32)
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication fails.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe.
    pub fn set_trim(&mut self, trim: i32) -> BoxFuture<'_, Result<()>> {
        Box::pin(async move {
            let mut requests = [Request::from_command(self.controller.make_set_trim(trim))];
            self.transport.cycle(&mut requests).await?;
            Ok(())
        })
    }

    // === Async Position Wait Complete Methods ===

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
    /// # Errors
    ///
    /// Returns an error if the transport is unavailable or communication
    /// fails. Returns `Error::Timeout` if the operation exceeds the
    /// timeout. Returns `Error::Fault` if the device reports a fault or
    /// timeout mode.
    ///
    /// # Cancel safety
    ///
    /// Cancel safe. If cancelled between polling iterations, the servo
    /// continues its current trajectory.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use moteus::AsyncController;
    /// use moteus::command::PositionCommand;
    /// use std::time::Duration;
    ///
    /// # async fn example() -> Result<(), moteus::Error> {
    /// let mut ctrl = AsyncController::new(1).await?;
    /// let result = ctrl.set_position_wait_complete(
    ///     &PositionCommand::new().position(0.5).stop_position(0.5),
    ///     Duration::from_millis(25),   // poll every 25ms
    ///     Duration::from_secs(5),      // timeout after 5 seconds
    /// ).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn set_position_wait_complete<'a>(
        &'a mut self,
        cmd: &'a PositionCommand,
        poll_interval: std::time::Duration,
        timeout: std::time::Duration,
    ) -> BoxFuture<'a, Result<QueryResult>> {
        Box::pin(async move {
            let start = std::time::Instant::now();
            let timeout_dur = timeout;
            let interval = poll_interval;

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
                self.transport.cycle(&mut requests).await?;
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

                if start.elapsed() > timeout_dur {
                    return Err(Error::Timeout);
                }

                tokio::time::sleep(interval).await;
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::async_transport::AsyncTransportOps;
    use crate::transport::transaction::dispatch_frame;
    use moteus_protocol::CanFdFrame;

    struct MockTransport {
        call_count: std::cell::Cell<usize>,
    }

    impl MockTransport {
        fn new() -> Self {
            MockTransport {
                call_count: std::cell::Cell::new(0),
            }
        }
    }

    impl AsyncTransportOps for MockTransport {
        fn cycle<'a>(&'a mut self, requests: &'a mut [Request]) -> BoxFuture<'a, Result<()>> {
            self.call_count.set(self.call_count.get() + 1);

            // Return a mock response frame
            let mut response = CanFdFrame::new();
            response.arbitration_id = 0x8100; // Source=1, Dest=0
                                              // Minimal valid response: mode + position
            response.data[0..3].copy_from_slice(&[0x21, 0x00, 0x0A]); // Mode = 10 (Position)
            response.size = 3;

            Box::pin(async move {
                // Dispatch response to matching requests
                dispatch_frame(&response, requests);
                Ok(())
            })
        }

        fn write<'a>(&'a mut self, _frame: &'a CanFdFrame) -> BoxFuture<'a, Result<()>> {
            Box::pin(async move { Ok(()) })
        }

        fn read<'a>(
            &'a mut self,
            _channel: Option<usize>,
        ) -> BoxFuture<'a, Result<Option<CanFdFrame>>> {
            Box::pin(async move { Ok(None) })
        }

        fn flush_read<'a>(&'a mut self, _channel: Option<usize>) -> BoxFuture<'a, Result<()>> {
            Box::pin(async move { Ok(()) })
        }
    }

    #[test]
    fn test_with_transport_is_infallible() {
        // with_transport never fails — no discovery needed
        let ctrl = AsyncController::with_transport(1, MockTransport::new());
        assert_eq!(ctrl.controller.id, 1);
    }

    #[test]
    fn test_explicit_transport() {
        let ctrl = AsyncController::with_transport(1, MockTransport::new());
        assert_eq!(ctrl.controller.id, 1);
    }
}
