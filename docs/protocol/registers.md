# Register Reference

## Register Usage

Each register may be accessed as potentially multiple data types.
This section describes the common mappings, and the semantics of each
register.

### Mappings

When registers are accessed as integer types, the following mappings
are used to encode the underlying floating point values into integer
values.

#### Current (measured in Amps)

- int8 => 1 LSB => 1A
- int16 => 1 LSB => 0.1A
- int32 => 1 LSB => 0.001A

#### Torque (measured in N*m)

- int8 => 1 LSB => 0.5 N*m
- int16 => 1 LSB => 0.01 N*m
- int32 => 1 LSB => 0.001 N*m

#### Voltage (measured in Volts)

- int8 => 1 LSB => 0.5V
- int16 => 1 LSB => 0.1V
- int32 => 1 LSB => 0.001 V

#### Temperature (measured in degrees Celsius)

- int8 => 1 LSB => 1 C
- int16 => 1 LSB => 0.1C
- int32 => 1 LSB => 0.001 C

#### Time (measured in seconds)

- int8 => 1 LSB => 0.01s
- int16 => 1 LSB => 0.001s
- int32 => 1 LSB => 0.000001s

#### Position (measured in revolutions)

- int8 => 1 LSB => 0.01 rotation => 3.6 degrees (range of -1.27 to 1.27)
- int16 => 1 LSB => 0.0001 rotation => 0.036 degrees (range of -3.2767 to 3.2767)
- int32 => 1 LSB => 0.00001 rotation => 0.0036 degrees

#### Velocity (measured in revolutions / s)

- int8 => 1 LSB => 0.1Hz / 36 dps
- int16 => 1 LSB => 0.00025 Hz > 0.09 dps
- int32 => 1 LSB => 0.00001 Hz => 0.0036 dps

#### Acceleration (measured in revolutions / s^2)

- int8 => 1 LSB => 0.05 l/s^2
- int16 => 1 LSB => 0.001 l/s^2
- int32 => 1 LSB => 0.00001 l/s^2

#### PWM and kp/kd scale (unitless)

- int8 => 1 LSB => (1/127) - 0.007874
- int16 => 1 LSB => (1/32767) - 0.000030519
- int32 => 1 LSB => (1/2147483647) - 4.657e-10

#### Power (W)

- int8 => 1 LSB => 10.0 W
- int16 => 1 LSB => 0.05 W
- int32 => 1 LSB => 0.0001 W

## Registers

### 0x000 - Mode

Mode: Read/write

The current operational mode of the servo.  Not all values are valid
to write.

- 0 => stopped = writeable, clears faults
- 1 => fault
- 2,3,4 => preparing to operate
- 5 => PWM mode
- 6 => voltage mode
- 7 => voltage FOC
- 8 => voltage DQ
- 9 => current
- 10 => position
- 11 => timeout
- 12 => zero velocity
- 13 => stay within
- 14 => measure inductance
- 15 => brake

When sending a command, this register must be written before any other
registers in the command frame.

### 0x001 - Position

Mode: Read only

The current position of the servo, measured in rotations of the output
shaft.  The maximum negative integer is reserved and will not be
reported.

### 0x002 - Velocity

Mode: Read only

The current velocity of the servo, measured in Hz at the output shaft.

### 0x003 - Torque

Mode: Read only

The current applied torque as measured at the output shaft.

### 0x004 - Measured Q phase current

Mode: Read only

The current in the Q phase measured in amperes.

### 0x005 - Measured D phase current

Mode: Read only

The current in the D phase measured in amperes.

### 0x006 - Measured absolution position

Mode: Read only

If an absolute encoder is configured on the ABS port, its value will
be reported here in revolutions.

### 0x007 - Measured electrical power

Mode: Read only

The estimated electrical power applied to the motor if positive.  If
negative, power applied to the DC input bus.

### 0x00a - Motor temperature

Mode: Read only

The current motor temperature, measured in degrees celsius.  This will
only be valid if an NTC thermistor is connected to the TEMP pads,
`servo.motor_thermistor_ohm` is set to the correct resistance, and
`servo.enable_motor_temperature` is set to 1.

### 0x00b - Trajectory complete

Mode: Read only

Non-zero if the current acceleration or velocity limited trajectory is
complete, and the controller is following the final velocity.

### 0x00c - Home state

Mode: Read only

* 0 - *relative only* - the position is not referenced to anything
* 1 - *rotor* - the position is referenced to the rotor
* 2 - *output* - the position has been referenced to the output,
  either with an output referenced encoder, or with a "set output
  nearest" or "set output exact" command


### 0x00d - Voltage

Mode: Read only

The current input voltage.

### 0x00e - Temperature

Mode: Read only

The current board temperature, measured in degrees celsius.

### 0x00f - Fault code

Mode: Read only

A fault code which will be set if the primary mode is 1 (Fault).

* 32 - *calibration fault* - the encoder was not able to sense a
  magnet during calibration
* 33 - *motor driver fault* - the most common reason for this is
  undervoltage, moteus attempted to draw more current than the supply
  could provide.  Other electrical faults may also report this error,
  the `drv8323` diagnostic tree has more information.
* 34 - *over voltage* - the bus voltage exceeded `servo.max_voltage`.
  This can happen due to misconfiguration, or if the controller
  regenerated power with a supply that cannot sink power and no flux
  braking was configured.
* 35 - *encoder fault* - the encoder readings are not consistent with
  a magnet being present.
* 36 - *motor not configured* - the `moteus_tool --calibrate`
  procedure has not been run on this motor.
* 37 - *pwm cycle overrun* - an internal firmware error
* 38 - *over temperature* - the maximum configured temperature has
  been exceeded
* 39 - *outside limit* - an attempt was made to start position control
  while outside the bounds configured by `servopos.position_min` and
  `servopos.position_max`.
* 40 - *under voltage* - the voltage was too low
* 41 - *config changed* - a configuration value was changed during
  operation that requires a stop
* 42 - *theta invalid* - no valid commutation encoder is available
* 43 - *position invalid* - no valid output encoder is available
* 44 - *driver enable fault* - the MOSFET gate driver could not be
  enabled
* 45 - *stop position deprecated* - an attempt was made to use the
  deprecated "stop position" feature along with velocity or
  acceleration limits.  Prefer to instead command the desired position
  directly with a target velocity of 0.0, or secondarily, disable
  acceleration and velocity limits.
* 46 - *timing violation* - internal checks are enabled, and the
  controller violated an internal timing constraint
* 47 - *bemf feedforward no accel* - `servo.bemf_feedforward` is
  configured, but no acceleration limit was specified.  If you
  *really* know what you are doing, you can disable this with
  `servo.bemf_feedforward_override`.
* 48 - *invalid limits* - `servopos.position_min` or
  `servopos.position_max` are finite and outside the available
  position range

Some non-zero codes can be presented during valid control modes
without a fault.  These indicate which, if any, function is limiting
the output power of the controller.

* 96 - `servo.max_velocity`
* 97 - `servo.max_power_W`
* 98 - the maximum system voltage
* 99 - `servo.max_current_A`
* 100 - `servo.fault_temperature`
* 101 - `servo.motor_fault_temperature`
* 102 - the commanded maximum torque
* 103 - `servopos.position_min` or `servopos.position_max`


### 0x010 / 0x011 / 0x012 - PWM phase A / B / C

Mode: Read/write

When in Pwm mode, this controls the raw PWM value for phase A, B, and
C.  If unspecified, 0.0 is used.

### 0x014 / 0x15 / 0x16 - Voltage phase A / B / C

Mode: Read/write

When in Voltage mode, this controls the voltage applied to phase A,
B, and C.  If unspecified, 0.0 is used.


### 0x018 - Voltage FOC Theta

Mode: Read/write

When in Voltage Foc mode, this controls the desired electrical phase.
Integral types use the PWM mapping.  If unspecified, 0.0 is used.

### 0x019 - Voltage FOC Voltage

Mode: Read/write

When in Voltage Foc mode, this controls the desired applied phase
voltage.  If unspecified, 0.0 is used.

### 0x01a - D Voltage

Mode: Read/write

When in Voltage Dq mode, this controls the desired applied D voltage.
If unspecified, 0.0 is used.

### 0x01b - Q Voltage

Mode: Read/write

When in kVoltageDq mode, this controls the desired applied Q voltage.
If unspecified, 0.0 is used.

### 0x01c - Commanded Q Phase Current

Mode: Read/write

When in Current mode, this controls the desired Q phase current.  If
unspecified, 0.0 is used.

### 0x01d - Commanded D Phase Current

Mode: Read/write

When in Current mode, this controls the desired D phase current.  Unless
you like burning power, with a BLDC motor you will typically want this
set to 0.  If unspecified, 0.0 is used.

### 0x1e - Voltage FOC Theta Rate

Mode: Read/write

When in Voltage Foc mode, this controls the rate of change of
electrical phase.  Integral types use the velocity mapping.


### 0x020 - Position command

Mode: Read/write

When in Position mode, this controls the desired position.

The maximally negative integer, or NaN for float represents, "use the
current position value".

If unspecified, 0.0 is used.

Note, in the absence of any configured or commanded velocity or
acceleration limits, the controller will attempt to achieve this
position *right now* subject to the kp and kd constants.

### 0x021 - Velocity command

Mode: Read/write

When in Position mode, advance the desired position at the given
velocity in Hz.

As a special case, if the 0x020 position is unset, and 0x026 stop
position is set, the sign of this is ignored, and is instead selected
so that the motor will move towards the stop position.

The maximally negative integer, or NaN for float, is treated the same
as 0.0.

If unspecified, 0.0 is used.

### 0x022 - Feedforward torque

Mode: Read/write

When in Position mode, add the given feedforward torque after applying
all regular control loops.  Note, this is torque at the output shaft.
If unspecified, 0.0 is used.

### 0x023 - Kp scale

Mode: Read/write

When in Position mode, shrink the proportional control term by the
given factor.  Integral types are applied as for PWM.  If unspecified,
1.0 is used.

### 0x024 - Kd scale

Mode: Read/write

When in Position mode, shrink the derivative control term by the given
factor.  Integral types are applied as for PWM.  This is internally
limited to be no more than the kp scale.  If unspecified, 1.0 is used.

### 0x025 - Maximum torque

Mode: Read/write

When in Position mode, the maximum torque to be applied.

The maximally negative integer, or NaN for float, uses the system-wide
configured maximum torque derived from `servo.max_current_A`.

If unspecified, NaN is used.

### 0x026 - Commanded stop position

Mode: Read/write

When in Position mode, and a non-zero velocity is commanded, stop
motion when reaching the given position.  NaN / maximally negative
means no limit is applied.  If unspecified, NaN is used.

Note, if the controller is ever commanded to move *away* from the stop
position, say with a velocity command that is inconsistent with the
start and stop position, then it will act as if a 0 velocity has been
commanded and the current command position equals the stop position.

NOTE: This register is deprecated.  Most users of this option should
instead use acceleration or velocity limits.

### 0x027 - Watchdog timeout

Mode: Read/write

This determines the length of time for which this command is valid.
If this timeout expires before another command is received, the
controller will enter the Timeout state.  The default is 0.0, which
means to use the system-wide configured default.  NaN / maximally
negative means apply no enforced timeout.

### 0x028 - Velocity limit

Mode: Read/write

This can be used to override the global velocity limit for internally
generated trajectories.

If negative, then no limit is applied.

If unspecified, it is NaN / maximally negative, which implies to use
the global configurable default.

### 0x029 - Acceleration limit

Mode: Read/write

This can be used to override the global acceleration limit for
internally generated trajectories.

If negative, then no limit is applied.

If unspecified, it is NaN / maximally negative, which implies to use
the global configurable default.

### 0x02a - Fixed voltage override

Mode: Read/write

If specified and not-NaN, then the control mode will temporarily be in
the "fixed voltage" mode, regardless of the current setting of
`servo.fixed_voltage_mode`.

### 0x02b - Ki ilimit scale

Mode: Read/write

When in Position mode, shrink the integral term's windup limit by the
given factor.  Integral types are applied as for PWM.  If unspecified,
1.0 is used.

### 0x02c - Fixed current override

Mode: Read/write

If specified, then the control mode will temporarily be in the "fixed
current" mode.  This is parallel to "fixed voltage" mode, but instead
of a fixed voltage, a fixed D axis current is controlled.

### 0x02d - Ignore position bounds

Mode: Read/write

If specified and non-zero, `servopos.position_min` and
`servopos.position_max` will be ignore.

### 0x030 - Proportional torque

Mode: Read

This reports the torque contribution from the proportional term in the
PID controller.

### 0x031 - Integral torque

Mode: Read

This reports the torque contribution from the integral term in the
PID controller.

### 0x032 - Derivative torque

Mode: Read

This reports the torque contribution from the derivative term in the
PID controller.

### 0x033 - Feedforward torque

Mode: Read

This reports the feedforward contribution in the PID controller.

### 0x034 - Total control torque

Mode: Read

This reports the total commanded torque from the position mode
controller.  This is also reported in 0x03a.

### 0x038 - Control Position

Mode: Read

This reports the current trajectory control position, in modes where
that is valid.  When velocity or acceleration limiting is enabled, the
control position will follow the desired limits to achieve the command
position.

### 0x039 - Control Velocity

Mode: Read

This reports the current velocity control value, in modes where that
is valid.  When velocity or acceleration limiting is enabled, the
control velocity will follow the desired limits to achieve the control
position and velocity.

### 0x03a - Control Torque

Mode: Read

The torque commanded by the control loop.  This is the same as 0x034.

### 0x03b - Position Error

Mode: Read

The current sensed position minus the control position.

### 0x03c - Velocity Error

Mode: Read

The current sensed velocity minus the control velocity.

### 0x03d - Torque Error

Mode: Read

The current sensed torque minus the control torque.


### 0x040 - Stay within lower bound

Mode: Read/write

When in Stay Within mode, this controls the minimum allowable
position.  The maximally negative integer or NaN for float represents,
"there is no lower bound".  When special or the position is above this
bound (and also respecting the optional upper bound), only a
feedforward torque is applied.  When outside this bound, the PID
controller is used to force the position back to the bound.  If
unspecified, 0.0 is used.

### 0x041 - Stay within upper bound

Mode: Read/write

When in Stay Within mode, this controls the maximum allowable
position.  The maximally negative integer, or NaN for float
represents, "there is no upper bound". When special or the position is
below this bound (and also respecting the optional lower bound), only
a feedforward torque is applied.  When outside this bound, the PID
controller is used to force the position back to the bound.  If
unspecified, 0.0 is used.

### 0x042 - Feedforward torque

A shadow of the 0x022 register.

### 0x043 - Kp scale

A shadow of the 0x023 register.

### 0x044 - Kd scale

A shadow of the 0x024 register.

### 0x045 - Maximum torque

A shadow of the 0x025 register.

### 0x046 - Watchdog timeout

A shadow of the 0x027 register.

### 0x047 - Ki ilimit scale

A shadow of the 0x02b register.

### 0x048 - Ignore position bounds

A shadow of the 0x02d register.

### 0x050 - Encoder 0 Position

Mode: Read only

Reports the current filtered position of the encoder configured in
slot 0.

### 0x051 - Encoder 0 Velocity

Mode: Read only

Reports the current filtered velocity of the encoder configured in
slot 0.

### 0x052 - Encoder 1 Position

Mode: Read only

Reports the current filtered position of the encoder configured in
slot 1.

### 0x053 - Encoder 1 Velocity

Mode: Read only

Reports the current filtered velocity of the encoder configured in
slot 1.

### 0x054 - Encoder 2 Position

Mode: Read only

Reports the current filtered position of the encoder configured in
slot 2.

### 0x055 - Encoder 2 Velocity

Mode: Read only

Reports the current filtered velocity of the encoder configured in
slot 2.

### 0x058 - Encoder Validity

Mode: Read only

Returns a bitfield, where bit 0 indicates whether encoder 0 is active,
bit 1 indicates whether encoder 1 is active, etc.

| Bit | Value             |
|-----|-------------------|
| 0   | Source 0 Theta    |
| 1   | Source 0 Velocity |
| 2   | Source 1 Theta    |
| 3   | Source 1 Velocity |
| 4   | Source 2 Theta    |
| 5   | Source 2 Velocity |

### 0x05c - Aux1 GPIO Command

Mode: Read/write

The current output command for any GPIOs configured as an output on
aux1 as a bitfield.  Not all bits may be used, as bit 0 is always for
pin 1, whether or not it is configured as a GPIO output.

### 0x05d - Aux2 GPIO Command

Mode: Read/write

The current output command for any GPIOs configured as an output on
aux2 as a bitfield.  Not all bits may be used, as bit 0 is always for
pin 1, whether or not it is configured as a GPIO output.

### 0x05e - Aux1 GPIO Status

Mode: Read only

The current input value of any GPIOs configured as an input on aux1 as
a bitfield.  Not all bits may be used, as bit 0 is always for pin 1,
whether or not it is configured as a GPIO input.

### 0x05f - Aux2 GPIO Status

Mode: Read only

The current input value of any GPIOs configured as an input on aux2 as
a bitfield.  Not all bits may be used, as bit 0 is always for pin 1,
whether or not it is configured as a GPIO input.

### 0x060/0x064 - Aux1 Analog Inputs

Mode: Read only

The current input value of any analog inputs configured on aux1.  The
registers are associated with pins 1-5, regardless of whether they are
configured as an analog input.  Each value is scaled as a PWM from 0
to 1.

### 0x068/0x06c - Aux2 Analog Inputs

Mode: Read only

The current input value of any analog inputs configured on aux2.  The
registers are associated with pins 1-5, regardless of whether they are
configured as an analog input.  Each value is scaled as a PWM from 0
to 1.

### 0x070 - Millisecond Counter

Mode: Read only

Increments once per millisecond.  It wraps at the maximum value for
the queried type to the minimum value for that type.  For floating
point types, it counts integers from 0 to 8388608.

### 0x071 - Clock Trim

Mode: Read/write

An integer which can trim the clock rate of the microprocessor on the
moteus controller.  Positive values speed it up and negative values
slow it down.  Each integer step roughly corresponds to a 0.25% change
in speed.

WARNING: Changing the speed affects all processes driven by the
microcontroller, including CAN communication.  Thus setting this to a
non-zero value may prevent future CAN communications.

### 0x072 - Aux1 PWM Input Period

Mode: Read only, int16/int32/float supported

Reports the period in microseconds between rising edges on the Aux1
PWM input pin, when a pin is configured in `pwm_in` mode.  Returns 0
if no signal detected for approximately 100ms.  Can be used to
calculate fan RPM: `RPM = 60000000 / (period_us * pulses_per_rev)`.
Typical PC fans output 2 pulses per revolution.

The measurable period range is approximately 10us to 65535us.  For a
2-pulse/rev fan, this corresponds to roughly 460 to 3,000,000 RPM.

### 0x073 - Aux1 PWM Input Duty Cycle

Mode: Read only

Reports the duty cycle of the signal on the Aux1 PWM input pin as a
floating point fraction from 0.0 to 1.0.  PWM mapping is used for
integral types.  Returns NaN (or maximally negative for integral
types) if no signal is detected.

### 0x074 - Aux2 PWM Input Period

Mode: Read only, int16/int32/float supported

Reports the period in microseconds between rising edges on the Aux2
PWM input pin, when a pin is configured in `pwm_in` mode.  Returns 0
if no signal detected for approximately 100ms.  Can be used to
calculate fan RPM: `RPM = 60000000 / (period_us * pulses_per_rev)`.
Typical PC fans output 2 pulses per revolution.

The measurable period range is approximately 10us to 65535us.  For a
2-pulse/rev fan, this corresponds to roughly 460 to 3,000,000 RPM.

### 0x075 - Aux2 PWM Input Duty Cycle

Mode: Read only

Reports the duty cycle of the signal on the Aux2 PWM input pin as a
floating point fraction from 0.0 to 1.0.  PWM mapping is used for
integral types.  Returns NaN (or maximally negative for integral
types) if no signal is detected.

### 0x076/0x07a - Aux1 PWM Outputs

Mode: Read/write

The current output PWM value for the given pin on Aux1.  PWM mapping
is used for integral types.

### 0x07b/0x07f - Aux2 PWM Outputs

Mode: Read/write

The current output PWM value for the given pin on Aux1.  PWM mapping
is used for integral types.

### 0x100 - Model Number

Name: Model Number
Mode: Read only

This returns a 32 bit model number.

### 0x101 - Firmware Version

Mode: Read only

This returns a 32 bit firmware version, encoded bytewise as
major.minor.micro.  i.e. 0x010304 is version 1.3.4


### 0x102 - Register map version

Mode: Read only

This returns a number that indicates how to interpret all registers.


### 0x110 - Multiplex ID

Name: Multiplex ID
Mode: Configurable

This controls the primary ID used to access the device over the
multiplex RS485 bus.  It can only be between 1 and 127.  (0 is
reserved as the broadcast address).

### 0x120 - 0x122 - Serial Number

Name: Serial Number
Mode: Read only

This returns a 96 bit serial number, least significant word first.


### 0x130 - Set Output Nearest

Mode: Write only

When sent, this causes the servo to select a whole number of internal
motor rotations so that the final position is as close to the given
position as possible.

### 0x131 - Set Output Exact

Mode: Write only

When sent, the servo will force the output position to be the exact
specified value.

### 0x132 - Require Reindex

Mode: Write only

When sent with any value, the servo will require that any index
position be re-located before control can begin.  Regardless, the
position will reset to an arbitrary value consistent with the current
encoder settings.

### 0x133 - Recapture Position and Velocity

Mode: Write only

When sent with any value, and if in "position" mode, the servo will
re-initialize the current control position and velocity to the sensed
values.  This also forces any pre-existing position integrative term
to zero.  It is expected that this will mostly be used when the
current applied torque is 0, either because of an in-place maximum
torque limit of zero, or because of an in-place kp and kd scale of
zero.

### 0x140 - Driver Fault 1

Mode: Read only

The exact bitfield reported by the motor driver in fault conditions
for fault register 1.  Up to 16 bits may be set.  This will only be
non-zero if the current mode is fault (1) and the fault code is 33
(motor driver fault).

### 0x141 - Driver Fault 2

Mode: Read only.

The exact bitfield reported by the motor driver in fault conditions
for fault register 2.  Up to 16 bits may be set.  This will only be
non-zero if the current mode is fault (1) and the fault code is 33
(motor driver fault).

### 0x150 - 0x153 - UUID

Name: UUID
Mode: Read only, int32 only

This returns a 128 bit UUID, this is the value printed on mjbots
packaging and returned by `moteus_tool --info`

### 0x154 - 0x157 - UUID Mask

Name: UUID
Mode: Write only, int32 only

If one or more of these fields are written, then the entire frame
after this point will be discarded unless the devices corresponding
UUID matches what was written.

### 0x158 UUID Mask Functional

Name: UUID Filter capable
Mode: Read only

Returns non-zero if the UUID mask is usable.
