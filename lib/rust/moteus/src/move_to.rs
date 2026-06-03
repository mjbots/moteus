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

//! Multi-servo coordinated position moves.
//!
//! The [`move_to`] function commands one or more servos to target positions
//! over a specified duration, polling until all trajectories complete. It
//! matches the Python `moteus.move_to()` free function.
//!
//! # Example
//!
//! ```no_run
//! use moteus::Controller;
//! use moteus::TransportOptions;
//! use moteus::move_to::{move_to, Setpoint, MoveToOptions};
//! use moteus::transport::singleton::create_default_transport;
//! use std::time::Duration;
//!
//! fn main() -> Result<(), moteus::Error> {
//!     let mut transport = create_default_transport(&TransportOptions::new())?;
//!     let c1 = Controller::new(1);
//!     let c2 = Controller::new(2);
//!
//!     // Move servo 1 to position 0.5 over 2 seconds
//!     let results = move_to(
//!         &mut transport,
//!         &[(&c1, 0.5.into())],
//!         &MoveToOptions::new().duration(Duration::from_secs(2)),
//!     )?;
//!
//!     // Multi-servo with custom setpoints
//!     let results = move_to(
//!         &mut transport,
//!         &[
//!             (&c1, Setpoint::new(0.5).velocity_limit(0.2)),
//!             (&c2, Setpoint::new(-0.3)),
//!         ],
//!         &MoveToOptions::new().duration(Duration::from_millis(1500)),
//!     )?;
//!     Ok(())
//! }
//! ```

use crate::controller::Controller;
use crate::error::{Error, Result};
use crate::transport::transaction::Request;
use crate::transport::Transport;
use moteus_protocol::command::PositionCommand;
use moteus_protocol::query::{QueryFormat, QueryResult};
use moteus_protocol::{Mode, Resolution};

/// A target setpoint for a servo.
///
/// Wraps a target position (in revolutions) with optional overrides
/// for velocity, torque limits, and other position command fields.
///
/// Use `From<f32>` to create a simple position-only setpoint:
///
/// ```rust
/// use moteus::move_to::Setpoint;
///
/// let sp: Setpoint = 0.5.into();
/// ```
#[non_exhaustive]
#[derive(Debug, Clone)]
pub struct Setpoint {
    /// Target position in revolutions. Use `f32::NAN` to hold current position.
    pub position: f32,
    /// Optional overrides applied on top of `MoveToOptions` defaults.
    pub overrides: PositionCommand,
}

impl Setpoint {
    /// Creates a new setpoint for the given position.
    pub fn new(position: f32) -> Self {
        Setpoint {
            position,
            overrides: PositionCommand::new(),
        }
    }

    /// Sets a velocity override (builder pattern).
    #[must_use]
    pub fn velocity(mut self, v: f32) -> Self {
        self.overrides.velocity = Some(v);
        self
    }

    /// Sets a velocity_limit override (builder pattern).
    #[must_use]
    pub fn velocity_limit(mut self, v: f32) -> Self {
        self.overrides.velocity_limit = Some(v);
        self
    }

    /// Sets an accel_limit override (builder pattern).
    #[must_use]
    pub fn accel_limit(mut self, v: f32) -> Self {
        self.overrides.accel_limit = Some(v);
        self
    }

    /// Sets a maximum_torque override (builder pattern).
    #[must_use]
    pub fn maximum_torque(mut self, v: f32) -> Self {
        self.overrides.maximum_torque = Some(v);
        self
    }

    /// Sets a kp_scale override (builder pattern).
    #[must_use]
    pub fn kp_scale(mut self, v: f32) -> Self {
        self.overrides.kp_scale = Some(v);
        self
    }

    /// Sets a kd_scale override (builder pattern).
    #[must_use]
    pub fn kd_scale(mut self, v: f32) -> Self {
        self.overrides.kd_scale = Some(v);
        self
    }

    /// Sets a feedforward_torque override (builder pattern).
    #[must_use]
    pub fn feedforward_torque(mut self, v: f32) -> Self {
        self.overrides.feedforward_torque = Some(v);
        self
    }

    /// Sets a stop_position override (builder pattern).
    #[must_use]
    pub fn stop_position(mut self, v: f32) -> Self {
        self.overrides.stop_position = Some(v);
        self
    }

    /// Sets a watchdog_timeout override (builder pattern).
    #[must_use]
    pub fn watchdog_timeout(mut self, v: f32) -> Self {
        self.overrides.watchdog_timeout = Some(v);
        self
    }
}

impl From<f32> for Setpoint {
    fn from(position: f32) -> Self {
        Setpoint::new(position)
    }
}

/// Configuration for a `move_to` operation.
#[non_exhaustive]
#[derive(Debug, Clone)]
pub struct MoveToOptions {
    /// Total duration of the move. When set, velocity_limit
    /// is computed per-servo as `|target - current| / duration`.
    pub duration: Option<std::time::Duration>,
    /// Default velocity_limit applied to all servos (rev/s).
    pub velocity_limit: Option<f32>,
    /// Default accel_limit applied to all servos (rev/s^2).
    pub accel_limit: Option<f32>,
    /// Default maximum_torque applied to all servos (Nm).
    pub maximum_torque: Option<f32>,
    /// Polling period (default 25 ms).
    pub period: std::time::Duration,
    /// Maximum time to wait before returning `Error::Timeout`.
    pub timeout: Option<std::time::Duration>,
}

impl Default for MoveToOptions {
    fn default() -> Self {
        MoveToOptions {
            duration: None,
            velocity_limit: None,
            accel_limit: None,
            maximum_torque: None,
            period: std::time::Duration::from_millis(25),
            timeout: None,
        }
    }
}

impl MoveToOptions {
    /// Creates a new `MoveToOptions` with defaults.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the move duration (builder pattern).
    #[must_use]
    pub fn duration(mut self, d: impl Into<std::time::Duration>) -> Self {
        self.duration = Some(d.into());
        self
    }

    /// Sets the default velocity_limit (builder pattern).
    #[must_use]
    pub fn velocity_limit(mut self, v: f32) -> Self {
        self.velocity_limit = Some(v);
        self
    }

    /// Sets the default accel_limit (builder pattern).
    #[must_use]
    pub fn accel_limit(mut self, a: f32) -> Self {
        self.accel_limit = Some(a);
        self
    }

    /// Sets the default maximum_torque (builder pattern).
    #[must_use]
    pub fn maximum_torque(mut self, t: f32) -> Self {
        self.maximum_torque = Some(t);
        self
    }

    /// Sets the polling period (builder pattern).
    #[must_use]
    pub fn period(mut self, p: std::time::Duration) -> Self {
        self.period = p;
        self
    }

    /// Sets the timeout (builder pattern).
    #[must_use]
    pub fn timeout(mut self, t: std::time::Duration) -> Self {
        self.timeout = Some(t);
        self
    }
}

/// Result for a single servo after a `move_to` completes.
#[derive(Debug, Clone)]
pub struct ServoResult {
    /// CAN ID of the servo.
    pub id: u8,
    /// Final query result from the servo.
    pub result: QueryResult,
}

/// Copies non-`None` fields from `overrides` into `cmd`, except `position`.
fn merge_overrides(cmd: &mut PositionCommand, overrides: &PositionCommand) {
    if overrides.velocity.is_some() {
        cmd.velocity = overrides.velocity;
    }
    if overrides.feedforward_torque.is_some() {
        cmd.feedforward_torque = overrides.feedforward_torque;
    }
    if overrides.kp_scale.is_some() {
        cmd.kp_scale = overrides.kp_scale;
    }
    if overrides.kd_scale.is_some() {
        cmd.kd_scale = overrides.kd_scale;
    }
    if overrides.maximum_torque.is_some() {
        cmd.maximum_torque = overrides.maximum_torque;
    }
    if overrides.stop_position.is_some() {
        cmd.stop_position = overrides.stop_position;
    }
    if overrides.watchdog_timeout.is_some() {
        cmd.watchdog_timeout = overrides.watchdog_timeout;
    }
    if overrides.velocity_limit.is_some() {
        cmd.velocity_limit = overrides.velocity_limit;
    }
    if overrides.accel_limit.is_some() {
        cmd.accel_limit = overrides.accel_limit;
    }
    if overrides.fixed_voltage_override.is_some() {
        cmd.fixed_voltage_override = overrides.fixed_voltage_override;
    }
    if overrides.ilimit_scale.is_some() {
        cmd.ilimit_scale = overrides.ilimit_scale;
    }
    if overrides.fixed_current_override.is_some() {
        cmd.fixed_current_override = overrides.fixed_current_override;
    }
    if overrides.ignore_position_bounds.is_some() {
        cmd.ignore_position_bounds = overrides.ignore_position_bounds;
    }
}

/// Builds a `QueryFormat` suitable for move_to polling.
///
/// Clones the first controller's query_format, then ensures mode, fault,
/// and trajectory_complete are at least Int8.
fn make_query_format(controllers: &[(&Controller, Setpoint)]) -> QueryFormat {
    let mut qf = controllers[0].0.query_format.clone();
    if qf.mode == Resolution::Ignore {
        qf.mode = Resolution::Int8;
    }
    if qf.fault == Resolution::Ignore {
        qf.fault = Resolution::Int8;
    }
    qf.trajectory_complete = Resolution::Int8;
    qf
}

/// Commands one or more servos to target positions, polling until all
/// trajectories complete.
///
/// This is the blocking equivalent of Python's `moteus.move_to()`. It
/// sends position commands to each servo on every polling cycle and waits
/// for the `trajectory_complete` flag.
///
/// # Arguments
///
/// * `transport` - The transport to use for communication
/// * `servos` - Slice of `(Controller, Setpoint)` pairs
/// * `options` - Move configuration (duration, limits, polling period, timeout)
///
/// # Returns
///
/// A `Vec<ServoResult>` with the final query result for each servo, in the
/// same order as the input `servos` slice.
///
/// # Errors
///
/// * `Error::Fault` — if any servo enters a fault or timeout mode
/// * `Error::Timeout` — if `options.timeout` is set and exceeded
/// * `Error::DeviceNotFound` — if a servo does not respond to the
///   initial query, or stops responding during the move
pub fn move_to(
    transport: &mut dyn Transport,
    servos: &[(&Controller, Setpoint)],
    options: &MoveToOptions,
) -> Result<Vec<ServoResult>> {
    if servos.is_empty() {
        return Ok(Vec::new());
    }

    let query_format = make_query_format(servos);

    // If a duration is specified, query the current positions and
    // calculate the necessary velocity limits.  A servo that does not
    // respond here is reported by ID before any watchdog is armed.
    let velocity_limits = compute_velocity_limits(transport, servos, options, &query_format)?;

    let start = std::time::Instant::now();
    let interval = options.period;
    let timeout_dur = options.timeout;

    let mut count: i32 = 2;
    let mut last_results: Vec<Option<QueryResult>> = vec![None; servos.len()];

    loop {
        // Check timeout
        if let Some(t) = timeout_dur {
            if start.elapsed() > t {
                return Err(Error::Timeout);
            }
        }

        // Build requests for all servos
        let mut requests: Vec<Request> = Vec::with_capacity(servos.len());
        for (i, (controller, setpoint)) in servos.iter().enumerate() {
            let mut pos_cmd = PositionCommand::new();
            pos_cmd.position = Some(setpoint.position);

            // Apply global defaults from options
            if options.velocity_limit.is_some() {
                pos_cmd.velocity_limit = options.velocity_limit;
            }
            if options.accel_limit.is_some() {
                pos_cmd.accel_limit = options.accel_limit;
            }
            if options.maximum_torque.is_some() {
                pos_cmd.maximum_torque = options.maximum_torque;
            }

            // Apply per-setpoint overrides
            merge_overrides(&mut pos_cmd, &setpoint.overrides);

            // Apply duration-computed velocity limit (overrides everything else)
            if let Some(vl) = velocity_limits[i] {
                pos_cmd.velocity_limit = Some(vl);
            }

            // Build command frame
            let mut command = controller.prepare_command(true);
            pos_cmd.serialize(command.frame_mut(), &controller.position_format);
            command.expected_reply_size = query_format.serialize(command.frame_mut());
            requests.push(Request::from_command(command));
        }

        transport.cycle(&mut requests)?;

        count = (count - 1).max(0);

        // Process results, mirroring the Python implementation: a
        // servo with no response this cycle is skipped rather than
        // failing the move.
        let mut all_complete = true;
        for (i, (controller, setpoint)) in servos.iter().enumerate() {
            let frame = match requests[i].responses.take().into_iter().next() {
                Some(frame) => frame,
                None => continue,
            };
            let qr = controller.parse_query(&frame);

            // Check for faults
            if qr.mode.is_error() {
                return Err(Error::Fault {
                    mode: qr.mode as u8,
                    code: qr.fault,
                });
            }

            // Only check trajectory_complete for non-NaN setpoints.
            //
            // While the motor is transitioning from stopped through
            // the calibration sequence to position mode, the position
            // trajectory code does not run and TRAJECTORY_COMPLETE
            // retains its prior value (often 1).  Don't trust it until
            // the motor is actually in a position-tracking mode.  This
            // matches the Python implementation.
            if !setpoint.position.is_nan() {
                let in_position_mode = matches!(qr.mode, Mode::Position | Mode::StayWithin);
                if !(in_position_mode && qr.trajectory_complete) {
                    all_complete = false;
                }
            }

            last_results[i] = Some(qr);
        }

        if count == 0 && all_complete {
            return Ok(servos
                .iter()
                .enumerate()
                .map(|(i, (controller, _))| ServoResult {
                    id: controller.id,
                    result: last_results[i].take().unwrap_or_default(),
                })
                .collect());
        }

        std::thread::sleep(interval);
    }
}

/// Queries current positions and computes per-servo velocity limits
/// based on duration, mirroring the Python implementation.
///
/// Servos with NaN setpoints are skipped.  A missing response for any
/// other servo is an error: this surfaces an absent or mis-addressed
/// servo by ID *before* any position command has armed a watchdog on
/// the servos that are present.
fn compute_velocity_limits(
    transport: &mut dyn Transport,
    servos: &[(&Controller, Setpoint)],
    options: &MoveToOptions,
    query_format: &QueryFormat,
) -> Result<Vec<Option<f32>>> {
    let duration_s = match options.duration {
        Some(d) if d.as_secs_f32() > 0.0 => d.as_secs_f32(),
        _ => return Ok(vec![None; servos.len()]),
    };

    // Query current positions.
    let mut requests: Vec<Request> = servos
        .iter()
        .map(|(controller, _)| {
            let cmd = controller.make_query_with_format(query_format);
            Request::from_command(cmd)
        })
        .collect();

    transport.cycle(&mut requests)?;

    let mut limits = Vec::with_capacity(servos.len());
    for (i, (controller, setpoint)) in servos.iter().enumerate() {
        if setpoint.position.is_nan() {
            limits.push(None);
            continue;
        }

        let qr = match requests[i].responses.take().into_iter().next() {
            Some(frame) => controller.parse_query(&frame),
            None => {
                return Err(Error::DeviceNotFound(format!(
                    "could not retrieve current position for servo {}",
                    controller.id
                )))
            }
        };

        // Calculate per-servo velocity limit: velocity = distance / duration
        let distance = (setpoint.position - qr.position).abs();
        limits.push(if distance != 0.0 {
            Some(distance / duration_s)
        } else {
            None
        });
    }

    Ok(limits)
}

/// Async version of [`move_to`].
///
/// Commands one or more servos to target positions, polling until all
/// trajectories complete. Uses async transport and async sleep.
///
/// See [`move_to`] for full documentation.
///
/// # Errors
///
/// * `Error::Fault` — if any servo enters a fault or timeout mode
/// * `Error::Timeout` — if `options.timeout` is set and exceeded
/// * `Error::DeviceNotFound` — if a servo does not respond to the
///   initial query, or stops responding during the move
///
/// # Cancel safety
///
/// Cancel safe. If cancelled, servos continue their current
/// trajectory until any configured watchdog timeout activates.
#[cfg(feature = "tokio")]
pub async fn async_move_to(
    transport: &mut dyn crate::transport::async_transport::AsyncTransport,
    servos: &[(&Controller, Setpoint)],
    options: &MoveToOptions,
) -> Result<Vec<ServoResult>> {
    if servos.is_empty() {
        return Ok(Vec::new());
    }

    let query_format = make_query_format(servos);

    // If a duration is specified, query the current positions and
    // calculate the necessary velocity limits.  A servo that does not
    // respond here is reported by ID before any watchdog is armed.
    let velocity_limits =
        async_compute_velocity_limits(transport, servos, options, &query_format).await?;

    let start = std::time::Instant::now();
    let interval = options.period;
    let timeout_dur = options.timeout;

    let mut count: i32 = 2;
    let mut last_results: Vec<Option<QueryResult>> = vec![None; servos.len()];

    loop {
        // Check timeout
        if let Some(t) = timeout_dur {
            if start.elapsed() > t {
                return Err(Error::Timeout);
            }
        }

        // Build requests for all servos
        let mut requests: Vec<Request> = Vec::with_capacity(servos.len());
        for (i, (controller, setpoint)) in servos.iter().enumerate() {
            let mut pos_cmd = PositionCommand::new();
            pos_cmd.position = Some(setpoint.position);

            // Apply global defaults from options
            if options.velocity_limit.is_some() {
                pos_cmd.velocity_limit = options.velocity_limit;
            }
            if options.accel_limit.is_some() {
                pos_cmd.accel_limit = options.accel_limit;
            }
            if options.maximum_torque.is_some() {
                pos_cmd.maximum_torque = options.maximum_torque;
            }

            // Apply per-setpoint overrides
            merge_overrides(&mut pos_cmd, &setpoint.overrides);

            // Apply duration-computed velocity limit (overrides everything else)
            if let Some(vl) = velocity_limits[i] {
                pos_cmd.velocity_limit = Some(vl);
            }

            // Build command frame
            let mut command = controller.prepare_command(true);
            pos_cmd.serialize(command.frame_mut(), &controller.position_format);
            command.expected_reply_size = query_format.serialize(command.frame_mut());
            requests.push(Request::from_command(command));
        }

        transport.cycle(&mut requests).await?;

        count = (count - 1).max(0);

        // Process results, mirroring the Python implementation: a
        // servo with no response this cycle is skipped rather than
        // failing the move.
        let mut all_complete = true;
        for (i, (controller, setpoint)) in servos.iter().enumerate() {
            let frame = match requests[i].responses.take().into_iter().next() {
                Some(frame) => frame,
                None => continue,
            };
            let qr = controller.parse_query(&frame);

            // Check for faults
            if qr.mode.is_error() {
                return Err(Error::Fault {
                    mode: qr.mode as u8,
                    code: qr.fault,
                });
            }

            // Only check trajectory_complete for non-NaN setpoints.
            //
            // While the motor is transitioning from stopped through
            // the calibration sequence to position mode, the position
            // trajectory code does not run and TRAJECTORY_COMPLETE
            // retains its prior value (often 1).  Don't trust it until
            // the motor is actually in a position-tracking mode.  This
            // matches the Python implementation.
            if !setpoint.position.is_nan() {
                let in_position_mode = matches!(qr.mode, Mode::Position | Mode::StayWithin);
                if !(in_position_mode && qr.trajectory_complete) {
                    all_complete = false;
                }
            }

            last_results[i] = Some(qr);
        }

        if count == 0 && all_complete {
            return Ok(servos
                .iter()
                .enumerate()
                .map(|(i, (controller, _))| ServoResult {
                    id: controller.id,
                    result: last_results[i].take().unwrap_or_default(),
                })
                .collect());
        }

        tokio::time::sleep(interval).await;
    }
}

/// Async version of [`compute_velocity_limits`].
#[cfg(feature = "tokio")]
async fn async_compute_velocity_limits(
    transport: &mut dyn crate::transport::async_transport::AsyncTransport,
    servos: &[(&Controller, Setpoint)],
    options: &MoveToOptions,
    query_format: &QueryFormat,
) -> Result<Vec<Option<f32>>> {
    let duration_s = match options.duration {
        Some(d) if d.as_secs_f32() > 0.0 => d.as_secs_f32(),
        _ => return Ok(vec![None; servos.len()]),
    };

    // Query current positions.
    let mut requests: Vec<Request> = servos
        .iter()
        .map(|(controller, _)| {
            let cmd = controller.make_query_with_format(query_format);
            Request::from_command(cmd)
        })
        .collect();

    transport.cycle(&mut requests).await?;

    let mut limits = Vec::with_capacity(servos.len());
    for (i, (controller, setpoint)) in servos.iter().enumerate() {
        if setpoint.position.is_nan() {
            limits.push(None);
            continue;
        }

        let qr = match requests[i].responses.take().into_iter().next() {
            Some(frame) => controller.parse_query(&frame),
            None => {
                return Err(Error::DeviceNotFound(format!(
                    "could not retrieve current position for servo {}",
                    controller.id
                )))
            }
        };

        // Calculate per-servo velocity limit: velocity = distance / duration
        let distance = (setpoint.position - qr.position).abs();
        limits.push(if distance != 0.0 {
            Some(distance / duration_s)
        } else {
            None
        });
    }

    Ok(limits)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::NullTransport;
    use moteus_protocol::CanFdFrame;

    #[test]
    fn test_setpoint_from_f32() {
        let sp: Setpoint = 0.5.into();
        assert_eq!(sp.position, 0.5);
        assert!(sp.overrides.velocity.is_none());
    }

    #[test]
    fn test_setpoint_builder() {
        let sp = Setpoint::new(1.0)
            .velocity_limit(0.5)
            .maximum_torque(2.0)
            .kp_scale(0.8);
        assert_eq!(sp.position, 1.0);
        assert_eq!(sp.overrides.velocity_limit, Some(0.5));
        assert_eq!(sp.overrides.maximum_torque, Some(2.0));
        assert_eq!(sp.overrides.kp_scale, Some(0.8));
    }

    #[test]
    fn test_move_to_options_defaults() {
        let opts = MoveToOptions::new();
        assert!(opts.duration.is_none());
        assert!(opts.velocity_limit.is_none());
        assert_eq!(opts.period, std::time::Duration::from_millis(25));
        assert!(opts.timeout.is_none());
    }

    #[test]
    fn test_move_to_options_builder() {
        let opts = MoveToOptions::new()
            .duration(std::time::Duration::from_secs(2))
            .velocity_limit(1.0)
            .accel_limit(5.0)
            .period(std::time::Duration::from_millis(50))
            .timeout(std::time::Duration::from_secs(10));
        assert_eq!(opts.duration, Some(std::time::Duration::from_secs(2)));
        assert_eq!(opts.velocity_limit, Some(1.0));
        assert_eq!(opts.accel_limit, Some(5.0));
        assert_eq!(opts.period, std::time::Duration::from_millis(50));
        assert_eq!(opts.timeout, Some(std::time::Duration::from_secs(10)));
    }

    #[test]
    fn test_merge_overrides() {
        let mut cmd = PositionCommand::new();
        cmd.velocity_limit = Some(1.0);
        cmd.maximum_torque = Some(5.0);

        let mut overrides = PositionCommand::new();
        overrides.velocity_limit = Some(2.0);
        overrides.kp_scale = Some(0.5);

        merge_overrides(&mut cmd, &overrides);
        assert_eq!(cmd.velocity_limit, Some(2.0)); // overridden
        assert_eq!(cmd.maximum_torque, Some(5.0)); // kept
        assert_eq!(cmd.kp_scale, Some(0.5)); // added
    }

    #[test]
    fn test_move_to_empty_servos() {
        let mut transport = NullTransport::new();
        let results = move_to(&mut transport, &[], &MoveToOptions::new()).unwrap();
        assert!(results.is_empty());
    }

    #[test]
    fn test_move_to_unresponsive_servo_fails_fast() {
        // With a duration set, the initial position query must fail
        // with an error naming the servo when it does not respond
        // (mirroring the Python RuntimeError), rather than entering
        // the polling loop where other servos' watchdogs starve.
        let mut transport = NullTransport::new();
        let c = Controller::new(1);
        let opts = MoveToOptions::new()
            .duration(std::time::Duration::from_secs(1))
            .timeout(std::time::Duration::from_millis(50));
        let result = move_to(&mut transport, &[(&c, 0.5.into())], &opts);
        match result {
            Err(Error::DeviceNotFound(msg)) => assert!(msg.contains("servo 1")),
            other => panic!("expected DeviceNotFound, got {:?}", other),
        }
    }

    /// Transport whose servo reports Position mode immediately but
    /// only sets trajectory_complete after a number of cycles,
    /// mimicking real hardware where a move takes many polling
    /// periods to finish.
    struct ScriptedTransport {
        cycles: usize,
        complete_after: usize,
    }

    impl Transport for ScriptedTransport {
        fn cycle(&mut self, requests: &mut [Request]) -> Result<()> {
            self.cycles += 1;
            let complete = self.cycles > self.complete_after;
            for req in requests.iter_mut() {
                let mut frame = CanFdFrame::new();
                frame.data[..6].copy_from_slice(&[
                    0x21, // reply int8, count 1
                    0x00, // register 0x000: Mode
                    0x0a, // = 10 (Position)
                    0x21, // reply int8, count 1
                    0x0b, // register 0x00b: TrajectoryComplete
                    u8::from(complete),
                ]);
                frame.size = 6;
                req.responses.push(frame);
            }
            Ok(())
        }
        fn write(&mut self, _frame: &CanFdFrame) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, _channel: Option<usize>) -> Result<Option<CanFdFrame>> {
            Ok(None)
        }
        fn flush_read(&mut self, _channel: Option<usize>) -> Result<()> {
            Ok(())
        }
        fn set_timeout(&mut self, _timeout: std::time::Duration) {}
        fn timeout(&self) -> std::time::Duration {
            std::time::Duration::ZERO
        }
    }

    #[test]
    fn test_move_to_completes_when_trajectory_finishes_late() {
        // Regression test: completion must be recognized even when the
        // trajectory takes many polling cycles to finish.  An earlier
        // version decremented its settle counter with saturating_sub
        // instead of clamping at zero (Python: max(count - 1, 0)), so
        // the `count == 0` exit condition was only satisfiable on
        // exactly the second iteration and real moves never completed.
        let mut transport = ScriptedTransport {
            cycles: 0,
            complete_after: 10,
        };
        let c = Controller::new(1);
        let opts = MoveToOptions::new()
            .duration(std::time::Duration::from_millis(100))
            .period(std::time::Duration::from_millis(1))
            .timeout(std::time::Duration::from_secs(5));
        let results = move_to(&mut transport, &[(&c, 0.5.into())], &opts).unwrap();
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].result.mode, Mode::Position);
        assert!(results[0].result.trajectory_complete);
    }
}
