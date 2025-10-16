# Troubleshooting Limited Torque

If moteus is unable to control a motor, but is unable produce sufficient torque for your application's needs, this post describes diagnostic steps.

## Output Limit Reasons

There are many factors that the moteus firmware can use to determine when to limit torque.  As of firmware version 2025-07-21, the firmware will report what factor is the current limiter via the "fault" register while in position mode.  The set of possible values are:

- **96 max_velocity**: Maximum velocity: If the observed velocity exceeds the configured maximum velocity, torque is limited in the direction that would cause the velocity to increase.  This is configured with `servo.max_velocity`.
- **97 max_power**: Maximum power: The total power of the output is limited through a combination of a board power profile and `servo.max_power_W`.  [See this post for more](https://blog.mjbots.com/2025/03/29/moaar-power/).
- **98 BEMF**: Maximum system voltage: The supply voltage, combined with the back EMF of the motor and the phase resistance does not permit more current to be applied.
- **99 max_current**: Maximum current: There is a system wide maximum output phase current that is configured in `servo.max_current_A`.
- **100 fault_temperature**:  Temperature: The onboard FET temperature sensor can limit the output current.  This threshold is set in the current firmware with `servo.fault_temperature`/`servo.temperature_margin`.
- **101 motor_temperature**: Temperature: The motor temperature sensor can also limit the output current.  This is configured in `servo.motor_fault_temperature`/`servo.motor_temperature_margin`.
- **102 max_torque**: Maximum torque: With each position mode command, the client can specify a maximum torque to use.  If the combination of the onboard PID and commanded feedforward torque exceeds this, then limiting occurs.
- **103 position_bounds**: Position bounds: If the current position is outside of `servopos.position_min` and `servopos.position_max`, then torque is limited in the direction that would push the motor further outside the bounds.

## Other possibilities

There are some "limiting-like" features that do not report through
this mechanism.  If position bounds are configured, the control
position is limited to stay within those bounds.  This does not
directly limit torque, so is not reported via this mechanism.

Similarly, if the commanded velocity exceeds either the velocity
limit, `velocity.max_velocity`, or `servo_stats.motor_max_velocity`,
then it is truncated to stay within.  This also is not reported via
this mechanism.
