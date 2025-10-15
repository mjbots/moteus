# Configuration Parameters

This section describes the configurable values which are most likely
to be modified for end-user applications.  Changes to all values take
effect *immediately*.  This may mean, that for instance, it is wise to
stop control loops before drastically changing control parameters.  Or
maybe not, it depends upon your goals.

## `id.id`

The servo ID presented on the CAN bus.  After this is modified, you need to immediately adjust which servo ID you communicate with in order to continue communication or save the parameters.

## `can.prefix`

A 13 bit integer used as the upper 13 bits for the ID of all CAN
communication.  As with `id.id` this takes effect immediately, so
after changing it, communication must be restarted with the correct
prefix in order to do things like save the configuration.

## `servopos.position_min`

The minimum allowed control position value, measured in rotations.  If
NaN, then no limit is applied.

## `servopos.position_max`

The maximum allowed control position value, measured in rotations.  If
NaN, then no limit is applied.

## `servo.pid_position`

These configure the position mode PID controller.

* `kp/ki/kd` - PID gains with units of:
  * kp - Nm per rotation
  * ki - Nm/s per rotation
  * kd - Nm per rotation/s
* `iratelimit` - The maximum rate at which the integral term can
   wind up, in N*m/s.  <0 means "no limit"
* `ilimit` - The total maximum I term, in Nm
* `max_desired_rate` - If non-zero, the commanded position is
  limited to change at this rate in Hz.

Note, these values are in physical units.  Thus a `kp` value of 1,
means that for 1 revolution of error at the output, 1 Nm of corrective
torque will be applied.  Similarly, with a `kd` value of 1, 1
revolution per second of error will result in 1 Nm of corrective
torque.  Doubly note that these values are measured at the output,
thus *after* any scaling in position, velocity, and torque implied by
`motor_position.rotor_to_output_ratio`.

## `servo.pid_dq`

These have the same semantics as the position mode PID controller, and
affect the current control loop.

## `servo.default_velocity_limit` / `servo.default_accel_limit`

Limits to be placed on trajectories generated within moteus.  If
either is `nan`, then that limit is unset.  The limits may also be
overriden individually on a per command basis.  The semantics of the
limits are as follows:

- *Neither set (both nan)* In this case, position and velocity
  commands take immediate effect.  The control position will be
  initialized to the command position, and the control velocity will
  be set to the command velocity.  The control position will advance
  at the given velocity indefinitely, or until the command stop
  position is reached.

- *Either set*: If x_c is the command position, v_c is the command
  velocity, and t is the time from receipt of the command, the
  semantics can be described as: "match the trajectory defined by x =
  x_c + v_c * t".

  If the acceleration limit is set, the above is effected by
  commanding accelerations of either [-accel_limit, 0, accel_limit].
  If an acceleration limit is not set, then the velocity will change
  instantaneously.

  If the velocity limit is set, then the intermediate velocities will
  obey "-velocity_limit < velocity < +velocity_limit".  If it is not
  set, then the velocities may grow to arbitrary magnitude.

NOTE: This is limited internally to be no more than
`servo.max_velocity`.

## `servo.inertia_feedforward`

When set to non-zero, and acceleration limits are currently in effect,
this will apply a feedforward torque equal to the current acceleration
multiplied by this configurable value.  This can be used to improve
response transients for short movements, where the acceleration period
is not long enough for the normal position PID to track well.

In an ideal world, you would set this to the moment of inertia of your
system in measured in kg * m^2.

## `servo.voltage_mode_control`

When set to non-zero, the current control loop is not closed, and all
current commands in amperes are instead treated as voltage mode
commands in volts related by the calibrated phase resistance.  For
high winding resistance motors, the default current sense resistors
are too small for accurate current sensing, resulting in significant
cogging torque and current sense noise.  If replacing the current
sense resistors is not an option, this flag can be used to achieve
smooth control.  The downside is that the actual torque will no longer
follow the applied torque accurately at speed, or in the face of
external disturbances.

When set, the `servo.pid_dq` configuration values no longer affect
anything.

## `servo.fixed_voltage_mode`

If non-zero, then no feedback based control of either position or
current is done.  Instead, a fixed voltage is applied to the phase
terminals based on the current commanded position and the configured
number of motor poles.  In this mode, the encoder and current sense
resistors are not used at all for control.

This is a similar control mode to inexpensive brushless gimbal
controllers, and relies on burning a fixed amount of power in the
motor windings continuously.

When this mode is active, the reported position and velocity will be 0
when the drive is disabled, and exactly equal to the control position
when it is enabled.

Various derating limits are inoperative in this mode:
 * torque derating for temperature
 * torque derating when outside position bounds
 * the maximum current limit
 * the commanded maximum torque

A fault will still be triggered for over-temperature.

## `servo.fixed_voltage_control_V`

In the fixed voltage control mode, the voltage to apply to the output.


## `servo.max_position_slip`

When finite, this enforces a limit on the difference between the
control position and the current measured position measured in
revolutions.  It can be used to prevent "catching up" when the
controller is used in velocity mode.

## `servo.max_velocity_slip`

When finite, this enforces a limit on the difference between the
control velocity and the current measured velocity, measured i Hz.  It
can be used to ensure that acceleration limits are obeyed in velocity
mode when external torques exceed the maximum.  If used, typically
`servo.max_position_slip` must be relatively small to avoid
instability.

## `servo.max_voltage`

If the input voltage reaches this value, a fault is triggered and all
torque is stopped.

## `servo.max_power_W`

If set, set the allowable maximum power to the lower of this and the
factory board power profile.

## `servo.override_board_max_power`

If true, then `servo.max_power_W` is used as the power limit even if
it is larger than the factory board power profile.

## `servo.pwm_rate_hz`

The PWM rate to use, defaulting to 30000.  Allowable values are
between 15000 and 60000.  Lower values increase efficiency, but limit
peak power and reduce the maximum speed and control bandwidth.

## `servo.derate_temperature`

Torque begins to be limited when the temperature reaches this value.

## `servo.fault_temperature`

If the temperature reaches this value, a fault is triggered and all
torque is stopped.

## `servo.enable_motor_temperature`

If true, then the motor temperature will be sensed via the TEMP pads
on the board.

## `servo.motor_derate_temperature`

Torque begins to be limited when the motor temperature reaches this value.

## `servo.motor_fault_temperature`

If the motor temperature reaches this value, a fault is triggered and
all torque is stopped.

## `servo.flux_brake_margin_voltage`

Selects the flux braking point relative to the currently configured `servo.max_voltage`.  `flux braking point = max_voltage - flux_brake_margin_voltage`.

When the input voltage is above the braking point, the controller
causes the motor to act as a "virtual resistor" with resistance
`servo.flux_brake_resistance_ohm`.  All extra energy is dumped into
the D phase of the motor.  This can be used to handle excess
regenerative energy if the input DC link is incapable of accepting
sufficient energy.

## `servo.max_current_A`

Phase current will never be used more than this value.  It can be
decreased to limit the total power used by the controller.  Increasing
beyond the factory configured value can result in hardware damage.

## `servo.max_velocity`

Output power will be limited if the velocity exceeds this threshold.

## `servo.max_velocity_derate`

Once velocity reaches the max_velocity plus this value, allowed output
power is reduced to 0.

## `servo.rotation_*`

These values configure a higher order torque model for a motor.

* `servo.rotation_current_cutoff_A` if the phase current is less than
  this value, then the linear relationship implied by `motor.v_per_hz`
  is used.

Once above that cutoff, the following formula is used to determine
torque from phase current:

```
torque = cutoff * tc + torque_scale * log2(1 + (I - cutoff) * current_scale)
```

Where `tc` is the torque constant derived from `motor.v_per_hz`.

This model is not automatically calibrated, and needs to be determined
and configured manually.

## `servo.default_timeout_s`

When sending position mode commands over CAN, there is an optional
watchdog timeout.  If commands are not received at a certain rate,
then the controller will latch into the "position timeout" state,
requiring a stop command to resume operation.  This configuration
value controls the length of time that the controller will wait after
receiving a command before entering this state.  If set to `nan` then
the controller will never enter this timeout state.

It may be overidden on a per-command basis with the 0x027 register or
the `t` optional flag of the `d pos` command for the diagnostic
interface.

## `servo.timeout_max_torque_Nm`

When in the "position timeout" mode the controller acts to damp the
output.  This parameter controls the maximum torque available for such
damping.

## `servo.timeout_mode`

Selects what behavior will take place in the position timeout mode.
The allowable values are a subset of the top level modes.

* 0 - "stopped" - the driver is disengaged
* 10 - "decelerate to 0 velocity and hold position"
* 12 - "zero velocity"
* 15 - "brake"

For mode 10, `servo.default_velocity_limit` and
`servo.default_accel_limit` are used to control the deceleration
profile to zero speed.  The default PID gains are used.  The only
limit on torque when in this timeout mode is `servo.max_current_A`.

## `servo.motor_thermistor_ohm`

The resistance of any attached motor NTC thermistor as measured at 25C
in ohms.

## `aux[12].pins.X.mode`

Selects what functionality will be used on the given pin.

* 0 - NC - Not connected (or used for onboard SPI)
* 1 - SPI - Used for one of CLK, MISO, or MOSI
* 2 - SPI CS - Used for SPI CS
* 3 - UART
* 4 - Software quadrature
* 5 - Hardware quadrature
* 6 - Hall
* 7 - Index
* 8 - Sine
* 9 - Cosine
* 10 - Step (not implemented)
* 11 - Dir (not implemented)
* 12 - RC PWM (not implemented)
* 13 - I2C
* 14 - Digital input
* 15 - Digital output
* 16 - Analog input
* 17 - PWM output

## `aux[12].pins.X.pull`

Configures optional pullup or pulldown on each pin.  Not all pullup
options will be used with every mode.  Additionally, the 2 aux2 pins
on moteus 4.5/8/11 have hard-installed 2k ohm pullups regardless of
these settings.

* 0 - no pullup or pulldown
* 1 - pull up
* 2 - pull down
* 3 - open drain (not implemented)

## `aux[12].i2c.i2c_hz`

The frequency to operate the I2C bus at.  Between 50000 and 400000.

## `aux[12].i2c.i2c_mode`

What I2C mode to use.

## `aux[12].i2c.devices.X.type`

What I2C device to expect.

* 0 - disabled
* 1 - AS5048
* 2 - AS5600

## `aux[12].i2c.devices.X.address`

The I2C address to use.

## `aux[12].i2c.devices.X.poll_ms`

How often in milliseconds to poll the device for more data.  Must no
less than 5.

## `aux[12].spi.mode`

The type of SPI device.

* 0 - The onboard AS5047P (CPR == 16384). Only valid for aux1.  If
  selected, the CLK, MOSI, and MISO lines must be either NC or
  selected as SPI.
* 1 - Disabled.
* 2 - AS5047P (CPR == 16384)
* 3 - iC-PZ
* 4 - MA732 (CPR == 65536)
* 5 - MA600 (CPR == 65536)
* 8 - AMT22 (CPR == 16384)

NOTE: iC-PZ devices require significant configuration and calibration
before use.  Diagnostic mode commands are provided for low level
access.

## `aux[12].spi.rate_hz`

The frequency to operate the SPI bus at.  The default is 12000000.

## `aux[12].uart.mode`

The type of UART device.

* 0 - Disabled
* 1 - RLS AksIM-2
* 2 - Tunnel
* 3 - Per-control cycle debug information (undocumented)
* 4 - CUI AMT21x series RS422

When the tunnel mode is selected, data may be sent or received using
the CAN diagnostic protocol.  For aux1, use diagnostic channel 2.  For
aux2, use diagnostic channel 3.

## `aux[12].uart.baud_rate`

The baud rate to use for the UART.

## `aux[12].uart.poll_rate_us`

For encoder modes, the interval at which to poll the encoder for new
position information.

## `aux[12].uart.rs422`

Enable the RS422 transceiver.  This is only valid for 'aux1', and
requires that pin D and E (`aux1.pins.3` and `aux1.pins.4`) be
used for UART.

## `aux[12].uart.cui_amt21_address`

Select the CUI AMT21 address to communicate with.  The default is 0x54
(84 decimal), which is the default address CUI AMT21 encoders are
configured with.

## `aux[12].quadrature.enabled`

True/non-zero if quadrature input should be read from this port.

## `aux[12].quadrature.cpr`

The number of counts per revolution of the quadrature input.  If used
as a source, then this CPR must match the one configured in the
source.

## `aux[12].hall.enabled`

True/non-zero if hall effect sensors should be read from this port.

## `aux[12].hall.polarity`

A bitmask to XOR with the 3 hall phases.

## `aux[12].index.enabled`

True/non-zero if an index input be read from this port.

## `aux[12].sine_cosine.enabled`

True/non-zero if a sine/cosine input should be read from this port.

## `aux[12].sine_cosine.common`

The common mode voltage to use for the sine cosine.  The sampling is
done with 12 bits, so 2048 would be exactly 0.5 * 3.3V.  However, it
is best to calibrate this with actual readings as observed over the
diagnostic protocol for optimal performance.

## `aux[12].bissc.enabled`

True/non-zero if a BiSS-C sensor should be attached to this port.
BiSS-C encoders require two pins, one of which is suitable both as a
UART-TX pin and PWM pin.

## `aux[12].bissc.rate_hz`

The bitrate used for BiSS-C communication.  1,000,000 (1Mbit) is the
maximum supported rate.

## `aux[12].bissc.data_bits`

The number of data bits reported by the BiSS-C encoder.

## `aux[12].bissc.crc_bits`

The number of CRC bits reported by the BiSS-C encoder.

## `aux[12].bissc.poll_rate_us`

The minimum period with which to poll the BiSS-C encoder.

## `aux[12].i2c_startup_delay_ms`

A delay in milliseconds after power-on (or upon reconfiguring), before
I2C devices associated with this auxiliary port are first used.

## `aux[12].pwm_period_us`

The period in microseconds to be used for PWM outputs on this
auxiliary port.

## `motor_position.sources.X.aux_number`

1 for an aux1 device, or 2 for an aux2 device.

## `motor_position.sources.X.type`

One of:

* 0 - disabled
* 1 - SPI
* 2 - UART
* 3 - Quadrature
* 4 - Hall
* 5 - Index
* 6 - Sine/Cosine
* 7 - I2C
* 8 - Sensorless (not implemented)

Note: The "Index" source type is only allowed for an output referenced
encoder, and can be used as an alternate way to enter the "output"
homed state.

## `motor_position.sources.X.i2c_device`

If `type` was "7/I2C", this is a 0 based index specifying *which* I2C
device on that port should be used.

## `motor_position.sources.X.incremental_index`

If the specified auxiliary port has an incremental encoder, like a
quadrature encoder, this can be set to either 1 or 2 in order to use
an "index" pin to reference the data to a given position.  It will
allow the source to provide theta readings for commutation.

## `motor_position.sources.X.cpr`

The CPR of the given input.  In some cases this is automatically set,
but in most it will need to be manually entered.

## `motor_position.sources.X.offset/sign`

An integer offset and inversion to apply.  The resulting value is:
`(raw + offset) * sign / cpr`.

## `motor_position.sources.X.reference`

* 0 - this source is relative to the rotor
* 1 - this source is relative to the output

## `motor_position.sources.X.pll_filter_hz`

Selects the 3dB cutoff frequency of a low-pass filter used on this
source.  It should typically be less than 10X the update rate of the
input and if used as the commutation or output sensor, should be
higher than the mechanical bandwidth of the plant.  Within that range,
it can be tuned for audible noise versus performance.

If set to 0, then no filter is applied.  In that case, sensors which
do not natively measure velocity will produce no velocity readings
(most of them).

## `motor_position.commutation_source`

A 0-based index into the source list that selects the source to use
for commutation.  This means it should have an accurate measure of the
relationship between the rotor and stator.

It is not recommended to use quadrature sources for commutation as
they can lose counts and require additional application level homing
support at each power-on.

## `motor_position.output.source`

A 0-based index into the source list that selects the source to use
for the output position.  The position and velocity from this source
will be used for control in "position" mode.

## `motor_position.output.offset/sign`

The offset is a floating point value measured in output revolutions.
Combined with the sign of -1/1, they can be used to position the 0
point of the output and control its direction of rotation.

## `motor_position.output.reference_source`

If non-negative, this is a 0-based index into the source list.  The
selected source is used at power on to disambiguate the output
position for multi-turn scenarios or when a reducer is configured.

## `motor_position.rotor_to_output_ratio`

The number of times the output turns for each revolution of the rotor.
For gear reducers (almost all configurations), this will be less than
one.  For example, a 4x gear reduction would be entered as 0.25.

## `motor_position.rotor_to_output_override`

If you *REALLY* know what you are doing, and want to configure a
non-reducing ratio, this can be set to true/non-zero.  Otherwise, a
ratio of greater than 1.0 will cause a fault.  This should only be
used if the system has a gearbox which is not a reducer, but speeds up
the output.
