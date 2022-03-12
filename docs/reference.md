moteus controller reference

# Overview #

## Theory of Operation ##

The moteus controller is intended to drive 3 phase brushless motors
using field oriented control.  It has an integrated magnetic encoder
for sensing the rotor position, 3 half-H bridges for switching power
to each of the three phases, and current sense capability on each of
the three phases.

The primary control mode, labeled as "position" mode in the rest of
this document is a two stage cascaded controller, with both running at
the switching frequency of 40kHz.  The outer stage is an integrated
position/velocity PID controller with optional feedforward torque.
The output of that loop is a desired torque/current for the Q phase of
the FOC controller.  The inner stage is a current mode PI controller.
Its output is the desired voltage value for the Q phase.  Then the
magnetic encoder is used to map the D/Q phase voltage values to the 3
phases of the motor.

![Control Structure](control_structure.png)

More precisely, the "Position Controller" implements the following
control law:

```
control_position = command_position OR control_position + control_velocity * dt
control_velocity = command_velocity OR control_velocity OR 0.0
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
current_integrator = limit(current_integrator + kp * current_error, ilimit)
voltage = current_integrator + kp * current_error
```

Since PID scaling for the position mode loop can be adjusted on a
cycle-by-cycle basis with the `kp_scale` and `kd_scale` terms, this
allows you to operate the controller with full
position/velocity/torque control, or velocity/torque, or just torque,
or any combination seamlessly throughout the control cycle.

## Usage Modes ##

The available knobs to the moteus controller allow it to implement a
few different types of control schemes.  From the factory, the control
gains are selected for medium torque bandwidth.  For other
applications, here are suggestions for gains and control options to
use.

### Velocity Control ###

To implement a velocity controller, each command should have the
"position" set to NaN (or equivalent integral encoding).  It is
recommended to configure `servo.max_position_slip` to a finite value
greater than 0 ([reference](#servomax_position_slip)).  When it is
larger, more external disturbances will be rejected, but the
controller will also "catch up" when the magnitude of external
disturbances is decreased.

### Low Speed or Precise Positioning ###

For either operation at very low speeds, or when precise positioning
performance is desired, it is recommended to configure a non-zero `ki`
and `ilimit` term in the position controller
([reference](#servopid_position)).  This will compensate for cogging
torque (at the expense of overall torque bandwidth).  It may also be
beneficial to select a lower value than default for `moteus_tool
--cal-bw-hz` during calibration.

### Internally Generated Trajectories ###

moteus currently only supports constant velocity trajectories.
Normally, the velocity commanded will continue indefinitely, either
until a watchdog timeout occurs or the configured position limit is
reached.

Through the use of the optional "stop_position" of the position
controller, moteus can set the velocity to zero when a specific
control position is achieved.  This feature can be used in
applications where the host commands the controller at a low update
rate.

### Constant Acceleration or Jerk ###

moteus only supports constant velocity internal trajectories.  To
approximate a constant acceleration or constant jerk trajectory, the
host processor should send a sequence of piecewise linear constant
velocity trajectories which approximate the desired one.  This would
be done by sending commands consisting of at least a position and
velocity at some moderate to high rate.

### High Torque Bandwidth ###

High torque bandwidth is desired for legged robots, or other
applications where it is necessary to respond to external disturbances
as rapidly as possible or accelerate the load maximally.  For these
applications, it is recommended to have no integrative term in the
position controller ([reference](#servopid_position)).  When
calibrating the motor, you may use the `moteus_tool --cal-bw-hz`
option to select a bandwidth higher than the default of 50Hz (or
manually select servo.pid_dq.kp/ki after calibration).

### Torque Control ###

For a pure torque control application, configure the position control
loop kp, kd, and ki to 0
([reference](#servopid_position)). Alternately you can set the
kp_scale and kd_scale to 0 in each command.

Note, however, that external torque control will be much lower
bandwidth than using the internal position controller of moteus (~50x
lower bandwidth if maximal CAN-FD update rate is achieved).  Thus, a
system will usually perform better if as much of the desired control
law as possible is formulated in terms of the built in position
controller.

# A. register command set #

The register command set is intended for use in real-time
applications.  It provides multiple levels of conciseness, and is
possible to operate at over 1kHz on the provided FD-CAN communications
bus.

Common definitions:

- *endian-ness*: All primitive types are in least significant byte
  first
- *varuint*: A sequence of one or more uint8 values, in least
  significant byte first order.  For each value, the 7 LSBs contain
  data and if the MSB is set, it means there are more bytes remaining.
  At most, it may represent a single uint32 and thus 5 bytes is the
  maximum valid length.
- *float*: An IEEE 754 32-bit floating point number.

## A.1 CAN Format ##

Communication with moteus is conducted via CAN-FD frames with a 1Mbit
standard bitrate and a 5Mbit data bitrate.

### CAN ID ###

The ID is structured as a 16 bit number, with the high 8 bits being
the "source" and the low 8 bits being the "destination".  The
destination is the 7 bit servo ID with 0 as the highest bit.  The
source is an arbitrary 7 bit number, with the high bit being 1 if
moteus should reply to the message.

The CAN frame must be an extended one if the ID would be greater than
0x7fff, but otherwise whether or not a frame is extended is not
considered.

Example:

ID: 0x8001
 * Send from source 0
 * To destination 1
 * 16th bit is set, so a reply is requested

ID: 0x100
 * Send from source 1
 * To destination 0
 * No reply is requested

### Subframes ###

Each CAN-FD frame contains one or more "subframes".  A short
description of the allowable subframe types are described below.  The
canonical reference is located at
[multiplex/format.h](https://github.com/mjbots/mjlib/blob/master/mjlib/multiplex/format.h)

Any extra trailing padding bytes required in the CAN-FD frame should
be set to NOP (0x50).

#### A.1.a Write Registers ####

*0x00, 0x04, 0x08, 0x0c* - write (int8|int16|int32|float)

- `varuint` => number of registers (may be optionally encoded as a
  non-zero 2 LSBS of the subframe type)
- `varuint` => start register number
- N x (int8|int16|int32|float) => values

#### A.1.b Read Registers ####

*0x10, 0x14, 0x18, 0x1c* - read (int8|int16|int32|float)

- `varuint` => number of registers (may be optionally encoded as a non-zero 2 LSBs)
- `varuint` => start register number

#### A.1.c Reply ####

*0x20, 0x24, 0x28, 0x2c* - reply (int8|int16|int32|float)

- `varuint` => number of registers (may be optionally encoded as a non-zero 2 LSBs)
- `varuint` => start register number
- N x (int8|int16|int32|float) => values

#### A.1.d Errors ####

*0x30, 0x31* - read/write error

- `varuint` => register number
- `varuint` => error number

#### A.1.e NOP ####

*0x50* - no operation


## A.2 Register Usage ##

Each register may be accessed as potentially multiple data types.
This section describes the common mappings, and the semantics of each
register.

### A.2.a Mappings ###

When registers are accessed as integer types, the following mappings
are used to encode the underlying floating point values into integer
values.

#### A.2.a.1 Current (measured in Amps) ####

- int8 => 1 LSB => 1A
- int16 => 1 LSB => 0.1A
- int32 => 1 LSB => 0.001A

#### A.2.a.2 Torque (measured in N*m) ####

- int8 => 1 LSB => 0.5 N*m
- int16 => 1 LSB => 0.01 N*m
- int32 => 1 LSB => 0.001 N*m

#### A.2.a.3 Voltage (measured in Volts) ####

- int8 => 1 LSB => 0.5V
- int16 => 1 LSB => 0.1V
- int32 => 1 LSB => 0.001 V

#### A.2.a.4 Temperature (measured in degrees Celsius) ####

- int8 => 1 LSB => 1 C
- int16 => 1 LSB => 0.1C
- int32 => 1 LSB => 0.001 C

#### A.2.a.5 Time (measured in seconds) ####

- int8 => 1 LSB => 0.01s
- int16 => 1 LSB => 0.001s
- int32 => 1 LSB => 0.000001s

#### A.2.a.6 Position (measured in revolutions) ####

- int8 => 1 LSB => 0.01 rotation => 3.6 degrees (range of -1.27 to 1.27)
- int16 => 1 LSB => 0.0001 rotation => 0.036 degrees (range of -3.2767 to 3.2767)
- int32 => 1 LSB => 0.00001 rotation => 0.0036 degrees

#### A.2.a.7 Velocity (measured in revolutions / s) ####

- int8 => 1 LSB => 0.1Hz / 36 dps
- int16 => 1 LSB => 0.00025 Hz > 0.09 dps
- int32 => 1 LSB => 0.00001 Hz => 0.0036 dps

### A.2.b Registers ###

#### 0x000 - Mode ####

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

#### 0x001 - Position ####

Mode: Read only

The current position of the servo, measured in rotations of the output
shaft.  The maximum negative integer is reserved and will not be
reported.

#### 0x002 - Velocity ####

Mode: Read only

The current velocity of the servo, measured in Hz at the output shaft.

#### 0x003 - Torque ####

Mode: Read only

The current applied torque as measured at the output shaft.

#### 0x004 - Measured Q phase current ####

Mode: Read only

The current in the Q phase measured in amperes.

#### 0x005 - Measured D phase current ####

Mode: Read only

The current in the D phase measured in amperes.

#### 0x006 - Measured absolution position ####

Mode: Read only

If an absolute encoder is configured on the ABS port, its value will
be reported here in revolutions.

#### 0x00d - Voltage ####

Mode: Read only

The current input voltage.

#### 0x00e - Temperature ####

Mode: Read only

The current board temperature, measured in degrees celsius.

#### 0x00f - Fault code ####

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

The full list can be found at: [fw/error.h](../fw/error.h#L25)


### 0x010 / 0x011 / 0x012 - PWM phase A / B / C ###

Mode: Read/write

When in Pwm mode, this controls the raw PWM value for phase A, B, and
C.  If unspecified, 0.0 is used.

### 0x014 / 0x15 / 0x16 - Voltage phase A / B / C ###

Mode: Read/write

When in Voltage mode, this controls the voltage applied to phase A,
B, and C.  If unspecified, 0.0 is used.


### 0x018 - Voltage FOC Theta ###

Mode: Read/write

When in Voltage Foc mode, this controls the desired electrical phase.
Integral types use the PWM mapping.  If unspecified, 0.0 is used.

### 0x019 - Voltage FOC Voltage ###

Mode: Read/write

When in Voltage Foc mode, this controls the desired applied phase
voltage.  If unspecified, 0.0 is used.

### 0x01a - D Voltage ###

Mode: Read/write

When in Voltage Dq mode, this controls the desired applied D voltage.
If unspecified, 0.0 is used.

### 0x01b - Q Voltage ###

Mode: Read/write

When in kVoltageDq mode, this controls the desired applied Q voltage.
If unspecified, 0.0 is used.

### 0x01c - Commanded Q Phase Current ###

Mode: Read/write

When in Current mode, this controls the desired Q phase current.  If
unspecified, 0.0 is used.

### 0x01d - Commanded D Phase Current ###

Mode: Read/write

When in Current mode, this controls the desired D phase current.  Unless
you like burning power, with a BLDC motor you will typically want this
set to 0.  If unspecified, 0.0 is used.


#### 0x020 - Position command ####

Mode: Read/write

When in Position mode, this controls the desired position.  The
maximally negative integer, or NaN for float represents, "use the
current position value".  If unspecified, 0.0 is used.  Note, the
controller will attempt to achieve this position *right now* subject
to the kp and kd constants.

#### 0x021 - Velocity command ####

Mode: Read/write

When in Position mode, advance the desired position at the given
velocity in Hz.

As a special case, if the 0x020 position is unset, and 0x026 stop
position is set, the sign of this is ignored, and is instead selected
so that the motor will move towards the stop position.  If
unspecified, 0.0 is used.

#### 0x022 - Feedforward torque ####

Mode: Read/write

When in Position mode, add the given feedforward torque after applying
all regular control loops.  Note, this is torque at the output shaft.
If unspecified, 0.0 is used.

#### 0x023 - Kp scale ####

Mode: Read/write

When in Position mode, shrink the proportional control term by the
given factor.  Integral types are applied as for PWM.  If unspecified,
1.0 is used.

#### 0x024 - Kd scale ####

Mode: Read/write

When in Position mode, shrink the derivative control term by the given
factor.  Integral types are applied as for PWM.  This is internally
limited to be no more than the kp scale.  If unspecified, 1.0 is used.

#### 0x025 - Maximum torque ####

When in Position mode, the maximum torque to be applied.  If
unspecified, this defaults to the system-wide configured maximum
torque.

#### 0x026 - Commanded stop position ####

When in Position mode, and a non-zero velocity is commanded, stop
motion when reaching the given position.  NaN / maximally negative
means no limit is applied.  If unspecified, NaN is used.

Note, if the controller is ever commanded to move *away* from the stop
position, say with a velocity command that is inconsistent with the
start and stop position, then it will act as if a 0 velocity has been
commanded and the current command position equals the stop position.

#### 0x027 - Watchdog timeout ####

Mode: Read/write

This determines the length of time for which this command is valid.
If this timeout expires before another command is received, the
controller will enter the Timeout state.  The default is 0.0, which
means to use the system-wide configured default.  NaN / maximally
negative means apply no enforced timeout.

### 0x030 - Proportional torque ###

Mode: Read

This reports the torque contribution from the proportional term in the
PID controller.

### 0x031 - Integral torque ###

Mode: Read

This reports the torque contribution from the integral term in the
PID controller.

### 0x032 - Derivative torque ###

Mode: Read

This reports the torque contribution from the derivative term in the
PID controller.

### 0x033 - Feedforward torque ###

Mode: Read

This reports the feedforward contribution in the PID controller.

### 0x034 - Total control torque ###

Mode: Read

This reports the total commanded torque from the position mode
controller.

### 0x040 - Stay within lower bound ###

Mode: Read/write

When in Stay Within mode, this controls the minimum allowable
position.  The maximally negative integer or NaN for float represents,
"there is no lower bound".  When special or the position is above this
bound (and also respecting the optional upper bound), only a
feedforward torque is applied.  When outside this bound, the PID
controller is used to force the position back to the bound.  If
unspecified, 0.0 is used.

### 0x041 - Stay within upper bound ###

Mode: Read/write

When in Stay Within mode, this controls the maximum allowable
position.  The maximally negative integer, or NaN for float
represents, "there is no upper bound". When special or the position is
below this bound (and also respecting the optional lower bound), only
a feedforward torque is applied.  When outside this bound, the PID
controller is used to force the position back to the bound.  If
unspecified, 0.0 is used.

### 0x042 - Feedforward torque ###

A shadow of the 0x022 register.

### 0x043 - Kp scale ###

A shadow of the 0x023 register.

### 0x044 - Kd scale ###

A shadow of the 0x024 register.

### 0x045 - Maximum torque ###

A shadow of the 0x025 register.

### 0x046 - Watchdog timeout ###

A shadow of the 0x027 register.

### 0x100 - Model Number ###

Name: Model Number
Mode: Read only

This returns a 32 bit model number.

### 0x101 - Firmware Version ###

Mode: Read only

This returns a 32 bit firmware version, encoded bytewise as
major.minor.micro.  i.e. 0x010304 is version 1.3.4


### 0x102 - Register map version ###

Mode: Read only

This returns a number that indicates how to interpret all registers.


### 0x110 - Multiplex ID ###

Name: Multiplex ID
Mode: Configurable

This controls the primary ID used to access the device over the
multiplex RS485 bus.  It can only be between 1 and 127.  (0 is
reserved as the broadcast address).

### 0x120 - 0x122 - Serial Number ###

Name: Serial Number
Mode: Read only

This returns a 96 bit serial number, least significant word first.


### 0x130 - Rezero ###

Mode: Write only

When sent, this causes the servo to select a whole number of internal
motor rotations so that the final position is as close to the given
position as possible.

## A.3 Example ##

A single CAN-FD frame can be used to command the servo, and initiate a
query of certain registers.  An example frame might look like the
following, encoded in hex with annotations.

- `01` - write a single int8 register (number of registers is encoded
  in the 2 LSBs)
 - `00` - start register number "Mode"
 - `0a` - "position" mode
- `07` - write 3x int16 registers (number of registers is encoded in
  the 2 LSBs)
 - `20` - register 0x020
 - `6000` - position = 0x0060 = 96 = 3.456 degrees
 - `2001` - velocity = 0x0120 = 288 = 25.92 dps
 - `50ff` - feedforward torque = 0xff50 = -176 = 1.76 N*m
- `14` - read int16 registers
 - `04` - read 4 registers
 - `00` - starting at 0x000 (so 0x000 Mode, 0x001 Position, 0x002
   Velocity, 0x003 Torque)
- `13` - read 3x int8 registers
 - `0d` - starting at 0x00d (so 0x00d Voltage, 0x00e Temperature,
    0x00f Fault code)

Thus the whole CAN-FD message would be (in hex):

`01000a07206000200150ff140400130d`

To send this using the fdcanusb converter to a device configured at
the default address of 1, you could write.

`can send 8001 01000a07206000200150ff140400130d`

The `80` in ID is used for two purposes.  The high bit being set
forces the device to respond (otherwise it will not respond, even if
query commands are sent).  The remaining bits are the "ID" to respond
to.  In response to this command, a possible response from the servo
would look like:

`rcv 100 2404000a005000000170ff230d181400`

Decoded, that means:

- `100` from device "1" to device "0"

- `24` reply with int16 values
 - `04` 4 registers
 - `00` starting at register 0
 - `0a00` in mode 10 - Position
 - `5000` position is 0x0050 = 80 = 2.88 degrees
 - `0001` velocity is 0x0100 = 256 = 23.04 dps
 - `70ff` torque is 0xff70 = -144 = -1.44 Nm
- `23` reply with 3 int8 values
 - `0d` starting at register 0x00d
 - `18` voltage is 24V
 - `14` temperature is 20C
 - `00` no fault

# B. diagnostic command set #

The following command set is intended for debugging and diagnostics.
It can be entered from `tview` or the `--console` mode of
`multiplex_tool` or `moteus_tool`.

## B.1 `d` - board debug ##

### `d stop` ###

This causes the controller to enter the "stopped" state, which
disables the motor driver.

### `d raw` ###

This enters the "raw" PWM mode.  Syntax:

```
d raw <pwm_a> <pwm_b> <pwm_c>
```

Where the pwm values are between 0.0 and 1.0.  A command to the idle
state is thus: `d raw 0.5 0.5 0.5`.

### `d pwm` ###

This enters voltage-FOC mode.  Syntax:

```
d pwm <phase> <magnitude>
```

Where phase is in radians, and magnitude is in volts.

### `d dq` ###

This enters the current controlled FOC mode.  Syntax:

```
d dq <d_A> <q_A>
```

### `d pos` ###

This enters the position control FOC mode.  Syntax:

```
d pos <pos> <vel> <max_torque> [options...]
```

Each optional element consists of a prefix character followed by a value.  Permissible options are:

- `p` - kp scale: the configured kp value is multiplied by this
  constant for the duration of this command
- `d` - kd scale: the configured kd value is multiplied by this
  constant for the duration of this command
- `s` - stop position: when a non-zero velocity is given, motion stops
  when the control position reaches this value.
- `f` - feedforward torque in Nm
- `t` - timeout: If another command is not received in this many
  seconds, enter the timeout mode.

The position, velocity, maximum torque, and all optional fields have
the same semantics as for the register protocol documented above.

### `d tmt` ###

Enter the timeout mode.  This mode commands a zero velocity and can only be exited through the stopped state.

```
d tmt <pos> <vel> <max_torque> [options...]
```

Available options are identical to `d pos`.

### `d zero` ###

Enter the zero velocity state.  A zero velocity is commanded
regardless of position.

```
d zero <pos> <vel> <max_torque> [options...]
```

Available options are identical to `d pos`.  `pos` and `vel` are
ignored.

### `d within` ###

Enter the "stay within" state.  When the position is contained within
the given bounds, only the feedforward torque is applied.  Otherwise,
the position mode controller is used to hold the position at the
violated boundary.

```
d within <lowbound> <highbound> <max_torque> [options...]
```

The fields have the same semantics as for the register protocol
documented above.  The options are the same as for `d pos`, with the
exception of stop position which is not supported.

### `d brake` ###

Enter the "brake" state.  In this mode, all motor phases are shorted
to ground, resulting in a passive "braking" action.

### `d index` ###

Force the current recorded position to match exactly the given value.

```
d index <position>
```

### `d rezero` ###

Assuming that `motor.position_offset` is configured correctly, update
the current position to the closest one which is consistent with a
given output position.

```
d rezero <position>
```

### `d cal` ###

INTERNAL moteus_tool USE ONLY: Enter the encoder calibration mode.
Upon starting this command, the controller will spin the motor in
voltage-FOC mode until the encoder covers a full revolution, then
repeat the process in the other direction.  During the process, the
current commanded phase and encoder value are periodically emitted to
the console.

Syntax:

```
d cal <magnitude> [options...]
```

Each optional element consists of a prefix character followed by a value.  Permissible options are:

* `s` - calibration speed in electrical revolutions per second

NOTE: This command is for internal moteus_tool use only.  It performs
only part of the calibration process, and saves nothing to the
persistent storage.


### `d flash` ###

Enter the bootloader.

NOTE: This is only intended for internal use.  Users who want to flash new firmware should use `moteus_tool`.

## B.2 `tel` - telemetry ##

### `tel get` ###

Retrieve the contents of a given channel.  Text or binary mode is
determined via the `tel fmt` or `tel text` commands, and defaults to
binary mode.

```
tel get <channel>
```

### `tel list` ###

List all available telemetry channels.

### `tel schema` ###

Report the binary schema associated with the given channel.

```
tel schema <channel>
```

The schema is reported as follows:

```
emit <channel>\r\n
<LE uint32 size><data>
```

### `tel rate` ###

Control the rate at which a given channel is emitted.

```
tel rate <channel> <rate_ms>
```

### `tel fmt` ###

Select whether a given channel will be emitted in binary or text form.

```
tel fmt <channel> <format>
```

`format` is an integer, non-zero signifies the data should be emitted
as text.

### `tel stop` ###

Stop emitting all periodic telemetry data.

### `tel text` ###

Switch all channels to text mode.

## B.3 `conf` - configuration ##

NOTE: Any commands that change parameters, such as `conf set`, `conf
load`, or `conf default`, if executed manually in `tview` will not
automatically update the UI.  `tview` must be restarted to display the
new parameters in the UI.

### `conf enumerate` ###

Print the current value of all configurable parameters.

### `conf get` ###

Get the value of a single configurable parameter.

```
conf get <item>
```

### `conf set` ###

Set the value of a single configurable parameter in RAM.

```
conf set <item> <value>
```

### `conf load` ###

Load all configurable values from persistent storage.  This will
overwrite their current values in RAM.

### `conf write` ###

Write the current value of all configurable parameters from RAM to
persistent storage.

### `conf default` ###

Update the RAM values of all configurable parameters to their firmware
default.

NOTE: This only updates RAM, not the persistent storage.  A `conf
write` will be necessary to save these to persistant storage.

# C. Configurable values #

This section describes the configurable values which are most likely
to be modified for end-user applications.  Changes to all values take
effect *immediately*.  This may mean, that for instance, it is wise to
stop control loops before drastically changing control parameters.  Or
maybe not, it depends upon your goals.

## `id.id` ##

The servo ID presented on the CAN bus.  After this is modified, you need to immediately adjust which servo ID you communicate with in order to continue communication or save the parameters.

## `can.prefix` ##

A 13 bit integer used as the upper 13 bits for the ID of all CAN
communication.  As with `id.id` this takes effect immediately, so
after changing it, communication must be restarted with the correct
prefix in order to do things like save the configuration.

## `motor.position_offset` ##

This value is added to `servo_stats.position` before reporting
`unwrapped_position_raw`.  It thus sets the `0` value for all position
control.

## `motor.unwrapped_position_scale` ##

This sets the reduction of any integrated gearbox.  Using this scales
all position, velocity, and torque commands and statuses accordingly.
A reducing gearbox will need a value between 0 and 1, so `0.25` for a
4x reduction gearbox.

## `servopos.position_min` ##

The minimum allowed control position value, measured in rotations.  If
NaN, then no limit is applied.

## `servopos.position_max` ##

The maximum allowed control position value, measured in rotations.  If
NaN, then no limit is applied.

## `servo.pid_position` ##

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
`motor.unwrapped_position_scale`.

## `servo.pid_dq` ##

These have the same semantics as the position mode PID controller, and
affect the current control loop.

## `servo.voltage_mode_control` ##

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

## `servo.fixed_voltage_mode` ##

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

## `servo.fixed_voltage_control_V` ##

In the fixed voltage control mode, the voltage to apply to the output.


## `servo.max_position_slip` ##

When finite, this enforces a limit on the difference between the
control position and the current measured position measured in
revolutions.  It can be used to prevent "catching up" when the
controller is used in velocity mode.

## `servo.max_voltage` ##

If the input voltage reaches this value, a fault is triggered and all
torque is stopped.

## `servo.max_power_W` ##

The controller will limit the output power to this value.  The value
is defined relative to a PWM rate of 40kHz and is scaled linearly with
respect to the PWM rate.

## `servo.pwm_rate_hz` ##

The PWM rate to use, defaulting to 40000.  Allowable values are
between 15000 and 60000.  Lower values increase efficiency, but limit
peak power and reduce the maximum speed and control bandwidth.

## `servo.derate_temperature` ##

Torque begins to be limited when the temperature reaches this value.

## `servo.fault_temperature` ##

If the temperature reaches this value, a fault is triggered and all
torque is stopped.

## `servo.flux_brake_min_voltage` ##

When the input voltage is above this value, the controller causes the
motor to act as a "virtual resistor" with resistance
`servo.flux_brake_resistance_ohm`.  All extra energy is dumped into
the D phase of the motor.  This can be used to handle excess
regenerative energy if the input DC link is incapable of accepting
sufficient energy.

## `servo.max_current_A` ##

Phase current will never be used more than this value.  It can be
decreased to limit the total power used by the controller.  Increasing
beyond the factory configured value can result in hardware damage.

## `servo.max_velocity` ##

Output power will be limited if the velocity exceeds this threshold.

## `servo.max_velocity_derate` ##

Once velocity reaches the max_velocity plus this value, allowed output
power is reduced to 0.

## `servo.rotation_*` ##

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

## `servo.default_timeout_s` ##

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

## `servo.timeout_max_torque_Nm` ##

When in the "position timeout" mode the controller acts to damp the
output.  This parameter controls the maximum torque available for such
damping.

## `servo.timeout_mode` ##

Selects what behavior will take place in the position timeout mode.
The allowable values are a subset of the top level modes.

* 0 - "stopped" - the driver is disengaged
* 12 - "zero velocity"
* 15 - "brake"

## `servo.rezero_from_abs` ##

If set to one, then shortly after startup, the value of the position
will be initialized to the closest one consistent with the position
measured at the ABS port.

## `abs_port.mode` ##

Configures the mode of operation of the ABS port:
* 0 - disabled
* 1 - AS5048B (I2C address is default 64)
* 2 - AS5600  (I2C address is default 54)

## `abs_port.i2c_mode` ##

Configures what I2C mode will be used:
* 0 - standard <= 100kHz
* 1 - fast <= 400kHz
* 2 - fast+ <= 1Mhz (note the AS5048B encoder does not support fast+)

## `abs_port.i2c_hz` ##

What rate to operate the I2C bus at.

## `abs_port.encoder_i2c_address` ##

The I2C address to communicate with the auxiliary encoder.

## `abs_port.encoder_poll_ms` ##

How often, in milliseconds to poll the auxiliary encoder.  Must be no
less than 5.

## `abs_port.position_offset` / `abs_port.position_scale` ##

The reported position is calculated from the raw value as follows:

```
position = (raw + offset) / 65536 * scale
```

## `encoder.mode` ##

Selects whether the onboard magnetic encoder or an external magnetic
encoder is used.

* 0 - onboard
* 1 - external AS5047 compatible SPI encoder

# D. Maintenance #

## tview usage ##

tview can monitor and control 1 or more devices simultaneously.  It can be started with:

```
python3 -m moteus_gui.tview --target 1[,2,3]...
```

When running, the configuration for each can be modified in the left hand tab, and live telemetry values can be displayed or plotted from the right hand tab.  diagnostic mode commands may be issued in the bottom terminal window.

A simple command and scripting language beyond the diagnostic protocol is available within the tview terminal window.

### Communicating with a specific device ###

If more than one device is available, commands can be sent to a specific device by prefixing the command with `ID>`.  For instance, to send a stop command to ID #2, you can do:

```
2>d stop
```

The 'A' character can be used to send to all devices simultaneously.

```
A>d stop
```

### Sending multiple commands at once ###

The `&&` token may be used to separate individual commands, which will be issued back to back.  For instance:

```
1>d stop && 2>d stop
```

Will send the command to both devices.

### Delays ###

A delay may be inserted into a sequence of commands by entering an integer number of milliseconds prefixed by the colon (':') character.  For instance:

```
d pos nan 0.5 1 s0.5 && :1000 && d stop
```

Will command a position mode, wait 1s, then command a stop.

## Calibration ##

Assuming your controller has firmware installed already, you can
calibrate the controller using the following procedure.

```
python3 -m moteus.moteus_tool --target 1 --calibrate
```

WARNING: Any attached motor must be able to spin freely.  It will be
spun in both directions and at high speed.

After calibrating for an entirely new type of motor, you may need to
adjust PID gains before the motor will perform acceptably, and/or
configure `motor.unwrapped_position_scale`.

## Setting the "zero offset" ##

The moteus controller can locate positions within one revolution after being power cycled, and will start with the reported position being between -0.5 and 0.5.  The physical zero position can be set using the following command:

```
python3 -m moteus.moteus_tool --target 1 --zero-offset
```

## Flashing and building firmware ##

### Flashing over CAN ###

The latest firmware can be downloaded from: https://github.com/mjbots/moteus/releases

You need the file named YYYYMMDD-moteus-HASH.elf NOT the one named
"bootloader".

Download that file and save it somewhere on your computer, then
substitute its path in place of `path/to/file.elf` in the following
command.

```
python3 -m moteus.moteus_tool --target 1 --flash path/to/file.elf
```

### From the debug port ###

The firmware can be built and flashed using:

```
tools/bazel build --config=target //fw:flash
```

Or, if already built, flashed using:

```
./fw/flash.py
```

### openocd ###

openocd 0.11.0 or newer is required.  You can find binaries for many
platforms at: https://xpack.github.io/openocd/releases/


### Building firmware ###

This will build, but not flash the firmware.  Only `curl` needs to be
installed.

```
tools/bazel test --config=target //:target
```


# E. Mechanical / Electrical #

## Mechanical ##

The current mechanical drawing for the controller can be found at:
[20210124-moteus-controller-r45-mechanical.pdf](https://github.com/mjbots/moteus/blob/main/hw/controller/r4.5/20210124-moteus-controller-r45-mechanical.pdf)

The current mechanical drawing for the qdd100 servo can be found at:
[20200315-qdd100-mechanical.pdf](https://drive.google.com/file/d/1KUQyR853e2uw8WOVrQHaeskWYHGSm3nI/view?usp=sharing)

## Electrical / Pinout ##

### JST PH-3 CAN ###

Looking at the pins of the connector with the bottom of the board up
the pins are numbered 1 to 3 from left to right.

 - 1 - CAN_H
 - 2 - CAN_L
 - 3 - GND

NOTE 1: CAN connections should be terminated by a 120 ohm resistor at
both ends of a bus.  Some mjbots products have built in termination
resistors, such as the pi3hat.  The fdcanusb has a software
configurable termination resistor that is by default on.  moteus
controllers have no termination resistors.  For very short runs, the
system will work terminated only on one side.  However, when runs
become longer than 0.5m, you will likely need to terminate both ends.
This can be done by crimping a 120 ohm resistor into a JST PH3
connector and connecting it to the open data connector.

NOTE 2: Ground may not be necessary, only one path through ground in a
system should exist to avoid ground loops.  In a typical robot
application with a common ground, that role is filled by the power
ground.  However, in desktop applications, it may be appropriate to
connect the CAN ground if the device power supply is otherwise
isolated.

### JST ZH-6 SWD ###

Looking at the pins of the connector with the top of the board up the
pins are numbered 1 to 6 from left to right.

 - 1 - NC
 - 2 - NRST
 - 3 - SWDIO
 - 4 - GND
 - 5 - SWCLK
 - 6 - 3.3V

### JST ZH-4 I2C ###

Pin 1 is closest to the ABS label.  They are assigned as follows:

 - 1 - 3.3V
 - 2 - SCL
 - 3 - SDA
 - 4 - GND

### XT30PW-M ###

Looking at the pins of the power connector with the top of the board
up, the ground pin is to the left with the chamfered corner and the
positive supply is to the right with the square corner.

### Pico-SPOX 6 ENC ###

Looking at the back of the board with the ENC connector at the top,
pins are numbered 1 as the rightmost, and 6 as the leftmost.

 - 1 - 3.3V
 - 2 - CS
 - 3 - GND
 - 4 - SCLK
 - 5 - MISO
 - 6 - MOSI

These pads can be populated with a Molex Pico-SPOX 6 connector PN
0874380643, or as an alternate, TE 5-1775444-6.

# F. CAN-FD communication #

## moteus_tool and tview configuration ##

`moteus_tool` and `tview` can be configured to communicate with a
moteus controller through a variety of transports.  By default, it
will attempt to autodetect either a fdcanusb or socketcan interface.

To force usage of a particular fdcanusb, you can use:

```
python3 -m moteus.moteus_tool --fdcanusb /path/to/fdcanusb
```

To force a particular python-can method, you can use:

```
python3 -m moteus.moteus_tool --can-iface socketcan --can-chan can0
```

where `--can-iface` specifies the "interface" for python-can and
`--can-chan` specifies the "channel".

Note, these are in addition to any other `moteus_tool` or `tview`
options that may be desired.

## Bit timings ##

Linux in particular appears to select very poor bit-timings for CAN-FD
when running at 5 MBps data rate.  The primary symptom is that no
communication is possible with a moteus controller or fdcanusb unless
you clock them a few percent slower than the requisite 5Mbps.  This
can be resolved by specifying custom bit timings to the linux
socketcan subsystem.  The following timings are known to work for at
least some systems:

### 40MHz clock systems ###

Chips such as the MCP2517/8 often use a 40MHz system clock.  The
following timings have been observed to work:

```
ip link set can0 up type can \
  tq 25 prop-seg 13 phase-seg1 12 phase-seg2 14 sjw 5 \
  dtq 25 dprop-seg 3 dphase-seg1 1 dphase-seg2 3 dsjw 3 \
  restart-ms 1000 fd on
```

### 80 MHz clock systems ###

Adapter such as the PEAK-CAN-FD use a 80MHz clock.  The following
timings have been observed to work:

```
ip link set can0 up type can \
  tq 12 prop-seg 25 phase-seg1 25 phase-seg2 29 sjw 10 \
  dtq 12 dprop-seg 6 dphase-seg1 2 dphase-seg2 7 dsjw 12 \
  restart-ms 1000 fd on
```

# G. Application Limitations #

## Position ##

The commanded position is limited to +-32767.0 revolutions before any
`unwrapped_position_scale` has been applied.  Reducers there will
further limit the range of the commandable revolutions.  For instance,
a 1/10 reducer configured as `unwrapped_position_scale=0.1` results in
a available position command range of +-3276.7 revolutions.

If either:

a. the position is commanded as the special value (NaN or maximally negative), or
b. the kp term is zero either through configuration or the "kp scale"

Then it is safe for the controller to "wrap around" from the maximal
possible position to the maximally negative position and vice versa.
This is useful in velocity control applications.

The commanded position is internally treated as a 32 bit floating
point value.  Thus the position resolution is reduced when the
magnitude of the position is large.  Resolution equal to the full
capabilities of the onboard encoder (~0.09degree) is maintained to
positions of +-2048.0 revolutions.  At the maximum possible position,
this resolution is reduced to ~1.44degrees.  Note, that this is only
for the "commanded position".  Velocity control and PID feedback on
the position works in an integral space and performs identically
throughout the available control envelope.

## Velocity ##

The smallest usable mechanical velocity which can be commanded is
0.0001 revolutions per second before any `unwrapped_position_scale`
has been applied.  This corresponds to 0.036 degrees per second.
Reducers will decrease the minimum usable velocity.

The maximum mechanical velocity which can be commanded is 28000 rpm,
or ~467 revolutions per second before any reducers.  Note, most motors
will be incapable of this speed either mechanically or electrically.

The maximum electrical frequency is 4kHz.

# Deployment Considerations #

## Phase Wire Soldering ##

The connections between moteus and the motor under control must be high quality.  Poor quality solder joints can cause intermittent changes in resistance, sparking, and high voltage transients.  High quality solder joints require:

* Sufficient flux to clean oxides off of the wire and the via
* Sufficient temperature so that the entire wire and via is hot enough for the solder to wet all surfaces

For the former, it is often necessary to add copious amounts of additional flux in addition to the rosin core inside many solders.  For the latter, a high power soldering iron, with a fat tip, and what seems like a long time may be necessary.  The temperature is definitely insufficient if solder touched to any part of the via or wire does not melt and wick instantly.  Usually a small amount of solder is placed between the tip and wire to act as a heat transfer agent.  Then the iron rests on the via and wire for between 5-30s until it looks like the initial solder has wicked fully into the via.  Then additional solder can be added until it forms a clean smooth fillet around the entire via wicking up to the wire.

When complete, the back side of the hole can be examined, and if successful, solder will have wicked partially down out of the via onto the other side forming a clean, smooth fillet there as well.

A demonstration of this can be found in the following video: https://www.youtube.com/watch?v=mZ9w_TaWmjQ

## Power Cable Construction ##

When constructing power cables using XT30 connectors, it is critical that solder joints be sound, otherwise intermittent connectivity can result.  This can cause sparking and high voltage transients.  The soldering principles are the same as in the phase wire soldering section above.

A demonstration of XT30 soldering can be found in the following video:
https://www.youtube.com/watch?v=f6WtDFWuxuQ

## Power Connectorization ##

For moteus to operate without damage, the XT30 connectors used to transmit power must make a solid connection that is non-intermittent.  As with poor soldering, an intermittent connection can cause inductive spikes, which will destroy components on the controller.

When connectors are functional, moderate insertion force should be required and the connectors should not "wiggle" too much after insertion.  Strain relief and mechanical capturing may be required if the cables are able to move during system motion.

The XT30 is not rated for any significant amount of mechanical force when mated, and may be damaged if a system goes "out of control" even in one instance, although milder mechanical stress may cause accelerated fatigue and failure.

If sparks are observed, that is *definitely* a problem and the system should be powered off until the connectors can be replaced and mitigations made for what led to that event.

Genuine AMASS connectors are rated for 1000 insertions assuming no other mechanical damage, however off-brand connectors may have worse tolerances and may not make a reliable connection for even one insertion.

## Regenerative Braking Safety ##

moteus can be commanded to sharply decelerate loads, either directly in response to commands, or due to external disturbances.  When braking a load, moteus by default applies the generated power to the input DC bus.

If there is nowhere for this power to go, this can cause problems.  The voltage can increase without bound, which in mild cases will cause the CAN transceiver on all devices connected to the bus to fail, and in severe cases can explode the main FETs or other components on the board.

Here's what you should know about the facilities moteus has to deal with this, and what you can do to make your design safer.

### Flux braking ###

The feature within moteus itself to deal with this is "flux braking".  The flux braking implementation will dissipate extra power in the windings of the motor when the bus voltage gets above a certain threshold.  This is controlled by the `servo.flux_brake_min_voltage` and `servo.flux_brake_resistance_ohm` parameters documented above.

### Design considerations ###

The following design considerations can be used to minimize the risk of damage to hardware in the event of overvoltage.  These are not a substitute for validation in progressively more demanding situations, but they can help you start off in a good place.

- *Configure Flux Braking*: To have optimal effect, the flux braking minimum voltage should be approximately only 1.5V above the maximum voltage you expect your supply to provide.  Additionally, the resistance may need to be lowered.  When adjusting the resistance, it is wise to test for stability by gradually increasing the voltage with the drivers engaged using a programmable supply and monitoring for instability in the voltage bus.  This can be identified either with an oscilloscope or audibly.  The default values are set to provide a baseline of protection without compromising the maximum voltage rating of the controller, but more aggressive parameters can be useful when your system voltage is lower and you are able to validate stability.

- *Power from a battery, not a PSU*: When not charged, batteries are capable of sinking current to minimize over-voltage transients.  However, if the battery is fully charged, most battery management systems drastically reduce the allowable charging current.  Thus, a battery is only useful as a mitigation if it is never charged above say 75 or 80% state of charge.

- *Decrease overall system voltage*: If you run the moteus controller with say a 10S battery, the peak input voltage can be as high as 42V.  That does not leave very much margin for regenerative loads.  For applications that experience sharp regenerative loads and do not have a battery capable of charging always attached, it is recommended not to exceed 8S (33.6V peak).

- *Lower the over-voltage fault*: The configuration parameter `servo.max_voltage` can be lowered for all devices on the bus.  If set above the highest expected transient, this can reduce the likelihood of severe transients causing damage.

- *Use a supply which can sink as well as source*: Powering from an inexpensive lab supply is the most dangerous, as they typically have no ability to sink current, only source it.  A "two quadrant" supply is the necessary device.
