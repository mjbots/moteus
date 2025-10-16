# Theory of Operation

The moteus controller is intended to drive 3 phase brushless motors
using field oriented control.  It has an integrated magnetic encoder
for sensing the rotor position, 3 half-H bridges for switching power
to each of the three phases, and current sense capability on each of
the three phases.

The primary control mode, labeled as "position" mode in the rest of
this document is a two stage cascaded controller, with both running at
the switching frequency (by default 30kHz).

The outermost stage is an optional limited acceleration and velocity
trajectory planner.  Within that is an integrated position/velocity
PID controller with optional feedforward torque.  The output of that
loop is a desired torque/current for the Q phase of the FOC
controller.

The inner stage is a current mode PI controller.  Its output is the
desired voltage value for the Q phase.  Then the magnetic encoder is
used to map the D/Q phase voltage values to the 3 phases of the motor.

![Control Structure](control_structure.png)

More precisely, the "Position Controller" implements the following
control law:

```
acceleration = trajectory_follower(command_position, command_velocity)
control_velocity = command_velocity OR control_velocity + acceleration * dt OR 0.0
control_position = command_position OR control_position + control_velocity * dt
position_error = control_position - feedback_position
velocity_error = control_velocity - feedback_velocity
position_integrator = limit(position_integrator + ki * position_error * dt, ilimit)
torque = position_integrator +
         kp * kp_scale * position_error +
         kd * kd_scale * velocity_error +
         command_torque
```

And the "Current Controller" implements the following control law:

```
current_error = command_current - feedback_current
current_integrator = limit(current_integrator + ki * current_error, ilimit)
voltage = current_integrator + kp * current_error
```

Since PID scaling for the position mode loop can be adjusted on a
cycle-by-cycle basis with the `kp_scale` and `kd_scale` terms, this
allows you to operate the controller with full
position/velocity/torque control, or velocity/torque, or just torque,
or any combination seamlessly throughout the control cycle.
