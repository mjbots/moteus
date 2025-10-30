# Troubleshooting Limited Torque

If moteus is able to control a motor, but is unable produce sufficient torque for your application's needs, this post describes diagnostic steps.

## Outline

There are a few main classes of things that can limit torque.

1. **Misconfiguration**:  Things that fall into this category include PID constants that are not tuned or an overly aggressive current limit or thermal limit.
2. **Under-cooled controller or motor**: If the controller and motor are properly sized for the application, they may still require thermal management to achieve those goals.
3. **Under-sized controller or motor**: If thermal management alone is insufficient, or is not practical, then either a larger controller or larger motor may be required.
4. **Insufficient supply voltage**: With low-Kv motors, it is possible for supply voltage to be a limiting factor.

## Diagnosing

First you need to understand which of the above three classes you are in.  To do so, there are some possible diagnostics and other things to check.

### Output Limit Reasons

As of firmware version 2025-07-21, the firmware will report if a factor is limiting the output current via the "fault" register while in position mode.  The set of possible values are:

- **96 max_velocity**: Maximum velocity: If the observed velocity exceeds the configured maximum velocity, torque is limited in the direction that would cause the velocity to increase.  This is configured with `servo.max_velocity`.
- **97 max_power**: Maximum power: The total power of the output is limited through a combination of a board power profile and `servo.max_power_W`.  [See this post for more](https://blog.mjbots.com/2025/03/29/moaar-power/).
- **98 BEMF**: Maximum system voltage: The supply voltage, combined with the back EMF of the motor and the phase resistance does not permit more current to be applied.
- **99 max_current**: Maximum current: There is a system wide maximum output phase current that is configured in `servo.max_current_A`.
- **100 fault_temperature**:  Temperature: The onboard FET temperature sensor can limit the output current.  This threshold is set in the current firmware with `servo.fault_temperature`/`servo.temperature_margin`.
- **101 motor_temperature**: Temperature: The motor temperature sensor can also limit the output current.  This is configured in `servo.motor_fault_temperature`/`servo.motor_temperature_margin`.
- **102 max_torque**: Maximum torque: With each position mode command, the client can specify a maximum torque to use.  If the combination of the onboard PID and commanded feedforward torque exceeds this, then limiting occurs.
- **103 position_bounds**: Position bounds: If the current position is outside of `servopos.position_min` and `servopos.position_max`, then torque is limited in the direction that would push the motor further outside the bounds.

### Other possibilities

If none of the above limiting flags are present, then the remaining options include:

#### Not stiff enough PID tuning

The torque applied by moteus (aside from any feedforward torque), is determined by the error between the desired and measured position and the error between the desired and measured velocity.  If the constants controlling this process are too low, then moteus will apply very little torque even for large errors.

One common way this occurs with moteus is:

1. A PID tuning is created
2. Later, a reducer is configured using `motor_position.rotor_to_output_ratio`
3. Wow, now moteus doesn't output much torque

The important point to note here is that the PID terms for the moteus control loop are measured at the *output* of any reducer.  Thus if kp=1, that means apply 1Nm of torque *at the output* for 1 revolution of error *at the output*.  With a reducer, that means the effective gains at the rotor are reduced by the reduction squared.

So, after configuring a reducer, you can either re-tune the PID gains, or increase the gains by the square of the reduction.

Sample:

```
servo.pid_position.kp = 4
servo.pid_position.kd = 0.05
motor_position.rotor_to_output_ratio = 1
```

You test this and it works great.  Then you configure the 8x reducer which was always present.

```
motor_position.rotor_to_output_ratio = 0.125
```

And now all of a sudden the controller barely does anything at all.
To have equivalent stiffness, you now need to increase the kp and kd
gains by a factor of `8 ** 2 = 64`.

```
servo.pid_position.kp = 256
servo.pid_position.kd = 3.2
```

#### Invalidated calibration

If your calibration was invalidated, it can result in lack of torque.  Invalidation can happen for any of the reasons in the [calibration troubleshooting guide](calibration.md), and include:

- A sense magnet that was not rigidly affixed to the rotor, say because adhesive not used at a joint
- A sense magnet that exceeded its rated temperature
- The controller mounting shifted

To test these hypotheses, re-calibrate the motor and check again.  If
the problem is resolved, then you need to identify which of the above
issues was the most likely culprit and resolve it.

#### Other "limiting like things"

There are some "limiting-like" features that do not report through the
fault code mechanism.  If position bounds are configured, the control
position is limited to stay within those bounds.  This does not
directly limit torque, so is not reported via this mechanism.

Similarly, if the commanded velocity exceeds either the velocity
limit, `velocity.max_velocity`, or `servo_stats.motor_max_velocity`,
then it is truncated to stay within.  This also is not reported via
this mechanism.

## Resolution

If the error was mis-configuration, fix it and you are done.

If the error was not mis-configuration, then the resolution depends upon what was causing the limiting.

### Thermal limiting

If the limit was thermal, then your options are:

1. apply cooling to whatever was hot
2. use a bigger controller or motor for whatever was hot
3. use a reducer, or if already using a reducer, use a bigger one

### Maximum current

If you hit the controller maximum current limit, then your options are:

1. use a different motor
2. use a controller with a higher current limit
3. use a reducer, or if already using a reducer, use a bigger one

### BEMF or `servo_stats.motor_max_velocity`

If you are limited by the back EMF of your motor, then your options are:

1. use a higher supply voltage
2. use a different motor with a higher Kv rating

### max power

Here, your options are:

1. configure a higher PWM frequency
2. use a bigger controller
3. use a different motor
4. use a reducer, or if already using a reducer, use a bigger one

## Resolution caveats

### Reducers

If considering using a reducer or a bigger reducer, the biggest caveat is that the maximum speed will be decreased by a corresponding amount.

Secondarily, most reducers have backlash and will add static and dynamic friction to your system and may have limits on maximum torque or speed that become limiters.

### PWM frequency

If increasing the PWM frequency to improve peak power, this will also decrease the efficiency of moteus.  That will result in lower continuous output current and power ratings.

### Cooling

The obvious downsides of cooling are noise, lower reliability, and increased power consumption even when under no load.

## moteus performance analysis tool

If attempting to design a motor system to achieve a particular goal, it may be useful to use the moteus performance analysis too, which lets you vary these parameters and measure the resulting performance of the system.

- [https://mjbots.github.io/moteus/mpat.html](https://mjbots.github.io/moteus/mpat.html)
- [mpat blog post](https://blog.mjbots.com/2025/07/17/moteus-performance-analysis-tool-v2/)
