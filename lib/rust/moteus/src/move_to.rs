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
//! ```rust,ignore
//! use moteus::{Controller, move_to, Setpoint, MoveToOptions};
//! use moteus::transport::factory::TransportOptions;
//! use moteus::Transport;
//!
//! let mut transport = Transport::with_options(&TransportOptions::new())?;
//! let c1 = Controller::new(1);
//! let c2 = Controller::new(2);
//!
//! // Move servo 1 to position 0.5 over 2 seconds
//! let results = move_to(
//!     &mut transport,
//!     &[(&c1, 0.5.into())],
//!     &MoveToOptions::new().duration(2.0),
//! )?;
//!
//! // Multi-servo with custom setpoints
//! let results = move_to(
//!     &mut transport,
//!     &[
//!         (&c1, Setpoint::new(0.5).velocity_limit(0.2)),
//!         (&c2, Setpoint::new(-0.3)),
//!     ],
//!     &MoveToOptions::new().duration(1.5),
//! )?;
//! ```

use crate::controller::Controller;
use crate::error::{Error, Result};
use crate::transport::transaction::Request;
use crate::transport::TransportOps;
use moteus_protocol::command::PositionCommand;
use moteus_protocol::query::{QueryFormat, QueryResult};
use moteus_protocol::Resolution;

/// A target setpoint for a servo.
///
/// Wraps a target position (in revolutions) with optional overrides
/// for velocity, torque limits, and other position command fields.
///
/// Use `From<f64>` to create a simple position-only setpoint:
///
/// ```rust
/// use moteus::move_to::Setpoint;
///
/// let sp: Setpoint = 0.5.into();
/// ```
#[derive(Debug, Clone)]
pub struct Setpoint {
    /// Target position in revolutions. Use `f64::NAN` to hold current position.
    pub position: f64,
    /// Optional overrides applied on top of `MoveToOptions` defaults.
    pub overrides: PositionCommand,
}

impl Setpoint {
    /// Creates a new setpoint for the given position.
    pub fn new(position: f64) -> Self {
        Setpoint {
            position,
            overrides: PositionCommand::new(),
        }
    }

    /// Sets a velocity override (builder pattern).
    pub fn velocity(mut self, v: f64) -> Self {
        self.overrides.velocity = Some(v);
        self
    }

    /// Sets a velocity_limit override (builder pattern).
    pub fn velocity_limit(mut self, v: f64) -> Self {
        self.overrides.velocity_limit = Some(v);
        self
    }

    /// Sets an accel_limit override (builder pattern).
    pub fn accel_limit(mut self, v: f64) -> Self {
        self.overrides.accel_limit = Some(v);
        self
    }

    /// Sets a maximum_torque override (builder pattern).
    pub fn maximum_torque(mut self, v: f64) -> Self {
        self.overrides.maximum_torque = Some(v);
        self
    }

    /// Sets a kp_scale override (builder pattern).
    pub fn kp_scale(mut self, v: f64) -> Self {
        self.overrides.kp_scale = Some(v);
        self
    }

    /// Sets a kd_scale override (builder pattern).
    pub fn kd_scale(mut self, v: f64) -> Self {
        self.overrides.kd_scale = Some(v);
        self
    }

    /// Sets a feedforward_torque override (builder pattern).
    pub fn feedforward_torque(mut self, v: f64) -> Self {
        self.overrides.feedforward_torque = Some(v);
        self
    }

    /// Sets a stop_position override (builder pattern).
    pub fn stop_position(mut self, v: f64) -> Self {
        self.overrides.stop_position = Some(v);
        self
    }

    /// Sets a watchdog_timeout override (builder pattern).
    pub fn watchdog_timeout(mut self, v: f64) -> Self {
        self.overrides.watchdog_timeout = Some(v);
        self
    }
}

impl From<f64> for Setpoint {
    fn from(position: f64) -> Self {
        Setpoint::new(position)
    }
}

/// Configuration for a `move_to` operation.
#[derive(Debug, Clone)]
pub struct MoveToOptions {
    /// Total duration of the move in seconds. When set, velocity_limit
    /// is computed per-servo as `|target - current| / duration`.
    pub duration: Option<f64>,
    /// Default velocity_limit applied to all servos (rev/s).
    pub velocity_limit: Option<f64>,
    /// Default accel_limit applied to all servos (rev/s^2).
    pub accel_limit: Option<f64>,
    /// Default maximum_torque applied to all servos (Nm).
    pub maximum_torque: Option<f64>,
    /// Polling period in seconds (default 0.025 = 25 ms).
    pub period_s: f64,
    /// Maximum time to wait before returning `Error::Timeout`.
    pub timeout: Option<f64>,
}

impl Default for MoveToOptions {
    fn default() -> Self {
        MoveToOptions {
            duration: None,
            velocity_limit: None,
            accel_limit: None,
            maximum_torque: None,
            period_s: 0.025,
            timeout: None,
        }
    }
}

impl MoveToOptions {
    /// Creates a new `MoveToOptions` with defaults.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the move duration in seconds (builder pattern).
    pub fn duration(mut self, d: f64) -> Self {
        self.duration = Some(d);
        self
    }

    /// Sets the default velocity_limit (builder pattern).
    pub fn velocity_limit(mut self, v: f64) -> Self {
        self.velocity_limit = Some(v);
        self
    }

    /// Sets the default accel_limit (builder pattern).
    pub fn accel_limit(mut self, a: f64) -> Self {
        self.accel_limit = Some(a);
        self
    }

    /// Sets the default maximum_torque (builder pattern).
    pub fn maximum_torque(mut self, t: f64) -> Self {
        self.maximum_torque = Some(t);
        self
    }

    /// Sets the polling period in seconds (builder pattern).
    pub fn period_s(mut self, p: f64) -> Self {
        self.period_s = p;
        self
    }

    /// Sets the timeout in seconds (builder pattern).
    pub fn timeout(mut self, t: f64) -> Self {
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
pub fn move_to(
    transport: &mut dyn TransportOps,
    servos: &[(&Controller, Setpoint)],
    options: &MoveToOptions,
) -> Result<Vec<ServoResult>> {
    if servos.is_empty() {
        return Ok(Vec::new());
    }

    let query_format = make_query_format(servos);

    // Per-servo velocity limits computed from duration
    let velocity_limits = compute_velocity_limits(transport, servos, options, &query_format)?;

    let start = std::time::Instant::now();
    let interval = std::time::Duration::from_secs_f64(options.period_s);
    let timeout_dur = options.timeout.map(std::time::Duration::from_secs_f64);

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

        // Parse results
        let mut all_complete = true;
        for (i, (controller, setpoint)) in servos.iter().enumerate() {
            if let Some(frame) = requests[i].responses.take().into_iter().next() {
                let qr = controller.parse_query(&frame);

                // Check for faults
                if qr.mode.is_error() {
                    return Err(Error::Fault {
                        mode: qr.mode as u8,
                        code: qr.fault,
                    });
                }

                // Only check trajectory_complete for non-NaN setpoints
                if !setpoint.position.is_nan() && !qr.trajectory_complete {
                    all_complete = false;
                }

                last_results[i] = Some(qr);
            } else {
                all_complete = false;
            }
        }

        count = count.saturating_sub(1);

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
/// based on duration.
fn compute_velocity_limits(
    transport: &mut dyn TransportOps,
    servos: &[(&Controller, Setpoint)],
    options: &MoveToOptions,
    query_format: &QueryFormat,
) -> Result<Vec<Option<f64>>> {
    let duration = match options.duration {
        Some(d) if d > 0.0 => d,
        _ => return Ok(vec![None; servos.len()]),
    };

    // Query current positions
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

        let vl = requests[i]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|frame| {
                let qr = controller.parse_query(&frame);
                let distance = (setpoint.position - qr.position).abs();
                distance / duration
            });
        limits.push(vl);
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
/// # Cancel safety
///
/// Cancel safe. If cancelled, servos continue their current
/// trajectory until any configured watchdog timeout activates.
#[cfg(feature = "tokio")]
pub async fn async_move_to(
    transport: &mut dyn crate::transport::async_transport::AsyncTransportOps,
    servos: &[(&Controller, Setpoint)],
    options: &MoveToOptions,
) -> Result<Vec<ServoResult>> {
    if servos.is_empty() {
        return Ok(Vec::new());
    }

    let query_format = make_query_format(servos);

    // Per-servo velocity limits computed from duration
    let velocity_limits =
        async_compute_velocity_limits(transport, servos, options, &query_format).await?;

    let start = std::time::Instant::now();
    let interval = std::time::Duration::from_secs_f64(options.period_s);
    let timeout_dur = options.timeout.map(std::time::Duration::from_secs_f64);

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

        // Parse results
        let mut all_complete = true;
        for (i, (controller, setpoint)) in servos.iter().enumerate() {
            if let Some(frame) = requests[i].responses.take().into_iter().next() {
                let qr = controller.parse_query(&frame);

                // Check for faults
                if qr.mode.is_error() {
                    return Err(Error::Fault {
                        mode: qr.mode as u8,
                        code: qr.fault,
                    });
                }

                // Only check trajectory_complete for non-NaN setpoints
                if !setpoint.position.is_nan() && !qr.trajectory_complete {
                    all_complete = false;
                }

                last_results[i] = Some(qr);
            } else {
                all_complete = false;
            }
        }

        count = count.saturating_sub(1);

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

/// Async version of velocity limit computation.
#[cfg(feature = "tokio")]
async fn async_compute_velocity_limits(
    transport: &mut dyn crate::transport::async_transport::AsyncTransportOps,
    servos: &[(&Controller, Setpoint)],
    options: &MoveToOptions,
    query_format: &QueryFormat,
) -> Result<Vec<Option<f64>>> {
    let duration = match options.duration {
        Some(d) if d > 0.0 => d,
        _ => return Ok(vec![None; servos.len()]),
    };

    // Query current positions
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

        let vl = requests[i]
            .responses
            .take()
            .into_iter()
            .next()
            .map(|frame| {
                let qr = controller.parse_query(&frame);
                let distance = (setpoint.position - qr.position).abs();
                distance / duration
            });
        limits.push(vl);
    }

    Ok(limits)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::NullTransport;

    #[test]
    fn test_setpoint_from_f64() {
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
        assert_eq!(opts.period_s, 0.025);
        assert!(opts.timeout.is_none());
    }

    #[test]
    fn test_move_to_options_builder() {
        let opts = MoveToOptions::new()
            .duration(2.0)
            .velocity_limit(1.0)
            .accel_limit(5.0)
            .period_s(0.05)
            .timeout(10.0);
        assert_eq!(opts.duration, Some(2.0));
        assert_eq!(opts.velocity_limit, Some(1.0));
        assert_eq!(opts.accel_limit, Some(5.0));
        assert_eq!(opts.period_s, 0.05);
        assert_eq!(opts.timeout, Some(10.0));
    }

    #[test]
    fn test_merge_overrides() {
        let mut cmd = PositionCommand::new();
        cmd.velocity_limit = Some(1.0);
        cmd.maximum_torque = Some(5.0);

        let overrides = PositionCommand {
            velocity_limit: Some(2.0),
            kp_scale: Some(0.5),
            ..Default::default()
        };

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
    fn test_move_to_timeout() {
        let mut transport = NullTransport::new();
        let c = Controller::new(1);
        let opts = MoveToOptions::new().timeout(0.05);
        let result = move_to(&mut transport, &[(&c, 0.5.into())], &opts);
        assert!(matches!(result, Err(Error::Timeout)));
    }
}
