# Control Modes

The moteus controller offers flexible control modes to suit different robotics applications. By adjusting control gains and command parameters, you can optimize the controller for various use cases—from precise positioning and velocity control to high-bandwidth torque applications. This guide covers the recommended configurations for common control scenarios.

## Understanding Position Mode

The primary control mode for the moteus controller is an integrated position/velocity controller. The semantics of the control command are somewhat unorthodox, so as to be easily amenable to idempotent commands sent from a higher level controller. Each command includes the following parameters:

- **Position**: The desired position in revolutions
- **Velocity**: The rate at which the desired position changes in revolutions/s
- **Maximum torque**: Never use more than this amount of torque when controlling
- **Feedforward torque**: Give this much extra torque beyond what the normal control loop says
- **Stop position**: If non-special, never move the desired position away from this target
- **kp scale**: Scale the proportional constant by this factor
- **kd scale**: Scale the derivative constant by this factor
- **ilimit scale**: Scale the integrative limit by this factor
- **Velocity limit override**: If non-special, override the configured velocity limit, which constrains how quickly the target position is reached
- **Acceleration limit override**: If non-special, override the configured acceleration limit, which constrains how quickly the target position is reached

Additionally, the position may be set as a "special value" (NaN for floating point and the debug interface, maximal negative for integer encodings). In that case, the position selected is "wherever you are right now".

A pure velocity mode can be obtained by setting the kp scale to 0 (or permanently so by configuring the kp constant to 0). In this case, using the `servo.max_position_slip` configurable parameter may be valuable as per the velocity control section below.

## Constant Acceleration Trajectories

Velocity and acceleration limits can be configured either globally, or
on a per-command basis which will cause moteus to internally generate
continuous acceleration limited trajectories to reach the given
position and velocity.  Once the trajectory is complete, the command
velocity is continued indefinitely.

=== "Diagnostic Protocol"

    ```
    # Move to position 1 then stop.  Accelerate/decelerate at 2Hz/s
    # and use a maximum velocity of 0.5Hz.
    d pos 1 0 nan a2 v0.5
    ```

=== "Python"

    ```python
    await controller.set_position(
        position=1,
        velocity=0,
        accel_limit=2,
        velocity_limit=0.5,
    )
    ```

=== "C++"

    ```cpp
    mjbots::moteus::Controller::Options options;
    options.position_format.accel_limit = mjbots::moteus::kFloat;
    options.position_format.velocity_limit = mjbots::moteus::kFloat;

    mjbots::moteus::Controller controller(options);

    mjbots::moteus::PositionMode::Command cmd;
    cmd.position = 1.0;
    cmd.velocity = 0.0;
    cmd.accel_limit = 2.0;
    cmd_velocity_limit = 0.5;

    auto result = controller.SetPosition(cmd);
    ```

Default values for acceleration and velocity limits can also be set in
configuration.

* `servo.default_accel_limit`
* `servo.default_velocity_limit`


## Velocity Control

To implement a velocity controller, each command should have the
"position" set to NaN (or equivalent integral encoding).  It is
recommended to configure `servo.max_position_slip` to a finite value
greater than or equal to 0 ([reference](../reference/configuration.md#servomax_position_slip)).
When it is larger, more external disturbances will be rejected, but
the controller will also "catch up" when the magnitude of external
disturbances is decreased.

**Example:**

=== "Diagnostic Protocol"

    ```
    d pos nan 2 nan
    ```

=== "Python"

    ```python
    await controller.set_position(
        position=math.nan,  # NaN for velocity mode
        velocity=2.0,       # 2 revolutions/second
        query=True
    )
    ```

=== "C++"

    ```cpp
    mjbots::moteus::PositionMode::Command cmd;
    cmd.position = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity = 2.0;  // 2 revolutions/second

    auto result = controller.SetPosition(cmd);
    ```


## Torque Control

For a pure torque control application, the PID gains of the position control loop must be set to 0.  One way to accomplish that is by sending the `kp_scale`, `kd_scale`, and `ilimit_scale` values to 0.

=== "Diagnostic Protocol"

    ```
    # Command a torque of 0.1 Nm
    d pos nan 0 nan p0 d0 i0 f0.1
    ```

=== "Python"

    ```python
    await controller.set_position(
        position=math.nan,
        velocity=0,
        kp_scale=0.0,
        kd_scale=0.0,
        ilimit_scale=0.0,
        feedforward_torque=0.1,
    )
    ```

=== "C++"

    ```cpp
    mjbots::moteus::Controller::Options options;
    options.position_format.kp_scale = mjbots::moteus::kFloat;
    options.position_format.kd_scale = mjbots::moteus::kFloat;
    options.position_format.ilimit_scale = mjbots::moteus::kFloat;
    options.position_format.feedforward_torque = mjbots::moteus::kFloat;

    mjbots::moteus::Controller controller(options);

    mjbots::moteus::PositionMode::Command cmd;
    cmd.position = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity = 0.0;
    cmd.kp_scale = 0.0;
    cmd.kd_scale = 0.0;
    cmd.ilimit_scale = 0.0;
    cmd.feedforward_torque = 0.1;

    auto result = controller.SetPosition(cmd);
    ```

**Caveat**: Using external torque control will be much lower
bandwidth than using the internal position controller of moteus (~15x
lower bandwidth if maximal CAN-FD update rate is achieved).  Thus, a
system will usually perform better if as much of the desired control
law as possible is formulated in terms of the built in position
controller.

If the system will never perform anything *but* torque control, then the PID gains can be set to 0 in configuration.

* `servo.pid_position.kp`
* `servo.pid_position.kd`
* `servo.pid_position.ilimit`

## Jerk Limited Trajectories

moteus only supports acceleration limited internal trajectories.  To
approximate a constant jerk trajectory, the host processor should send
a sequence of piecewise linear constant velocity trajectories which
approximate the desired one.  This would be done by sending commands
consisting of at least a position and velocity at some moderate to
high rate while disabling the internal velocity and acceleration
limits.

## Low Speed or Precise Positioning

For either operation at very low speeds, or when precise positioning
performance is desired, it is recommended to configure a non-zero `ki`
and `ilimit` term in the position controller
([reference](../reference/configuration.md#servopid_position)).  This will compensate for cogging
torque (at the expense of overall torque bandwidth).  It may also be
beneficial to select a higher value for calibration bandwidth.
