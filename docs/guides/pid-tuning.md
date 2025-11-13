# PID Tuning

When operating in position or velocity mode, moteus uses a P(I)D controller to determine what torque to apply at each instant.  The constants that control this operation need to be selected by the user.  Tuning does not have to be challenging, but does require that you understand what you are trying to accomplish.

## Theory

The purpose of the three gains:

**Proportional**: This term applies a restorative torque that is proportional to the error between the current desired position and the current measured position.  It is measured in Nm per revolution of error as measured at the output of the reducer.  It directly controls the "stiffness" of the controller.  If used in isolation, the motor will apply a fixed torque for any given error, i.e. if you manually move the motor 0.1 revolution away from the setpoint, it will apply 0.1 * the torque indefinitely, and then if you move it to be 0.2 revolution away from the setpoint, it will apply 0.2 * the torque.

**Derivative**: This term acts as a "damping" term, or viscious resistance.  As an analogy, if the proportional term is the spring in a suspension system, the derivative term is how thick the oil in the suspension is.  The torque applied from this term scales with the magnitude of the error between the desired velocity and the measured velocity.  If all other gains were zero and the motor is disturbed by hand, a larger gain makes it feel like you are working against a thicker fluid resistance.

**Integrative**:  The integrative term applies a torque that scales with the error in position integrated over time.  Many, if not most, moteus systems use no integrative term at all.  Integrative terms are hard to tune, and result in poor transient response.  However, they are the only way to achieve 0 steady state error in a loaded system.  Without an integrative term, the steady state error, while possibly very small, will never be zero.  For the integrative term there are two parameters to select, the gain itself, and the anti-windup gain term.

## Procedure

The position control P(I)D loop parameters are set in `servo.pid_position` and are selected by the user to achieve a desired control response.  Most users want a PID controller that is well damped and sufficiently stiff for their application.

Here is a possible procedure for tuning.  It is best executed using tview in a configuration where you can move the motor with your hand.  A load that is close to representative in terms of inertia, damping, and friction is best, but something is better than nothing.

1. Set kp to be small and kd, ki, and ilimit to be 0.
```
servo.pid_position.kp = 0.001
servo.pid_position.kd = 0
servo.pid_position.ki = 0
servo.pid_position.ilimit = 0
```
If you have an understanding of the system, you could select an initial kp that is larger, but you want it to be a value such that the restorative torque is very gradual and you can easily disturb the system by hand.
2. Increase kp by 50% at a time until the desired stiffness is reached.
    1. Test the current value.  A command like `d pos nan 0 nan` can be used to "hold position".  Then use your hand to move the rotor to judge the stiffness.
    2. If insufficiently stiff, increase `servo.pid_position.kp` by 50% and go back to (a)
    3. If vibration or instability results, decrease `servo.pid_position.kp` by 2x and that is roughly a good stable maximum.
3. Tune the derivative term
    1. Set kd initially to 1/100th of the kp value. For example, the following where FOO is 0.01 * the selected value for `servo.pid_position.kp`.
`servo.pid_position.kd = FOO`
    2. Test the current value.  Use `d pos nan 0 nan` to hold position.  Manually move the motor away from the setpoint, then quickly release it.  If "damped" the motor will smoothly come to a stop at the original position.  If underdamped, the motor may oscillate.
    3. If insufficiently damped, increase `servo.pid_position.kd` by 50% and go back to (a)
    4. If high frequency vibration or oscillation results, decrease `servo.pid_position.kd` by 2x and that is roughly a good stable maximum.
4. If zero steady state error is required, then an integrative term is required.  If not, leave `ilimit` at 0 and you are done.
5. Configure the integrative term.
    1. Set `servo.pid_position.ilimit` to be 20% higher than the maximum expected steady state torque.
    2. Set an initial value of `servo.pid_position.ki` equal to the `servo.pid_position.kp` term.
    3. Increase `servo.pid_position.ki` until the desired response is achieved.  `ki` will likely need to be numerically much larger than kp, on the order of 10-1000x larger.  A more representative control profile than just a "hold position" may be needed to determine if the selected integrative constants are sufficient.

## Acceleration limited trajectories

Further, it is *strongly* recommended to configure and use an acceleration limit with moteus that is physically achievable in your system.  Doing so means that the "desired" position and "desired" velocity will follow continuous, physically achievable trajectories.

- `servo.default_accel_limit` can be set in Hz / s.

If you do not set an acceleration limit, or select one that is too high to be physically achievable, your system will experience overshoot during movements and you will find it challenging to tune the P(I)D parameters to achieve a desired response.

## What not to tune

The "current loop" gains are set in `servo.pid_dq`, and are set during
calibration based on the desired torque bandwidth.  They are not
normally changed by the user.  Instead, they are selected by specifying an appropriate `--cal-bw-hz` during calibration.

## Next steps

Now you are ready to control moteus using one of the possible control modes:

- [Control](../guides/control-modes.md)
