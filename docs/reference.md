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
Its output is the desired PWM value for the Q phase.  Then the
magnetic encoder is used to map the D/Q phase PWM values to the 3
phases of the motor.

Since PID scaling for the integrated position mode loop can be
adjusted on a cycle-by-cycle basis, this allows you to operate the
controller with full position/velocity/torque control, or
velocity/torque, or just torque, or any combination seamlessly
throughout the control cycle.

## Limits ##

### Position ###

The commanded position is limited to +-32767.0 revolutions before any
`unwrapped_position_scale` has been applied.  Reducers there will
further limit the range of the commandable revolutions.  For instance,
a 1/10 reducer configured as `unwrapped_position_scale=0.1` results in
a available position command range of +-3276.7 revolutions.

If either:

a. the position is commanded as the special value (NaN or maximal negative), or
b. the kp term is zero either through configuration or the "kp scale"

Then it is safe for the controller to "wrap around" from the maximal
possible position to the maximal negative position and vice versa.
This is useful in velocity control applications.

The commanded position is internally treated as a 32 bit floating
point value.  Thus the position resolution is reduced when the
magnitude of the position is large.  Resolution equal to the full
capabilities of the onboard encoder (~0.09degree) is maintained to
positions of +-2048.0 revolutions.  At the maximum possible position,
this resolution is reduced to ~1.44degrees.  Note, that this is only
for the "commanded position".  Velocity control and PID feedback on
the position works in an integral space and performs identically at
all positions.

### Velocity ###

The smallest usable mechanical velocity which can be commanded is
0.0001 revolutions per second before any `unwrapped_position_scale`
has been applied.  This corresponds to 0.036 degrees per second.
Reducers will decrease the minimum usable velocity.

The maximum mechanical velocity which can be commanded is 28000 rpm,
or ~467 revolutions per second before any reducers.  Note, most motors
will be incapable of this speed either mechanically or electrically.

The maximum electrical frequency is 5kHz.


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

## A.1 Subframes ##

Each CAN-FD frame contains one or more "subframes".  A short
description of the allowable subframe types are described below.  The
canonical reference is located at
[multiplex/format.h](https://github.com/mjbots/mjlib/blob/master/mjlib/multiplex/format.h)

### A.1.a Write Registers ###

*0x00, 0x04, 0x08, 0x0c* - write (int8|int16|int32|float)

- `varuint` => number of registers (may be optionally encoded as a
  non-zero 2 LSBS of the subframe type)
- `varuint` => start register number
- N x (int8|int16|int32|float) => values

### A.1.b Read Registers ###

*0x10, 0x14, 0x18, 0x1c* - read (int8|int16|int32|float)

- `varuint` => number of registers (may be optionally encoded as a non-zero 2 LSBs)
- `varuint` => start register number

### A.1.c Reply ###

*0x20, 0x24, 0x28, 0x2c* - reply (int8|int16|int32|float)

- `varuint` => number of registers (may be optionally encoded as a non-zero 2 LSBs)
- `varuint` => start register number
- N x (int8|int16|int32|float) => values

### A.1.d Errors ###

*0x30, 0x31* - read/write error

- `varuint` => register number
- `varuint` => error number

### A.1.e NOP ###

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

- int8 => 1 LSB => 1V
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
- int16 => 1 LSB => 0.00025 Hz > 0.36 dps
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

#### 0x00d - Voltage ####

Mode: Read only

The current input voltage.

#### 0x00e - Temperature ####

Mode: Read only

The current board temperature, measured in degrees celsius.

#### 0x00f - Fault code ####

Mode: Read only

A fault code which will be set if the primary mode is 1 (Fault).

### 0x010 / 0x011 / 0x012 - PWM phase A / B / C ###

Mode: Read/write

When in kPwm mode, this controls the raw PWM value for phase A, B, and
C.  If unspecified, 0.0 is used.

### 0x014 / 0x15 / 0x16 - Voltage phase A / B / C ###

Mode: Read/write

When in kVoltage mode, this controls the voltage applied to phase A,
B, and C.  If unspecified, 0.0 is used.


### 0x018 - Voltage FOC Theta ###

Mode: Read/write

When in kVoltageFoc mode, this controls the desired electrical phase.
Integral types use the PWM mapping.  If unspecified, 0.0 is used.

### 0x019 - Voltage FOC Voltage ###

Mode: Read/write

When in kVoltageFoc mode, this controls the desired applied phase
voltage.  If unspecified, 0.0 is used.

### 0x01a - D Voltage ###

Mode: Read/write

When in kVoltageDq mode, this controls the desired applied D voltage.
If unspecified, 0.0 is used.

### 0x01b - Q Voltage ###

Mode: Read/write

When in kVoltageDq mode, this controls the desired applied Q voltage.
If unspecified, 0.0 is used.

### 0x01c - Commanded Q Phase Current ###

Mode: Read/write

When in kFoc mode, this controls the desired Q phase current.  If
unspecified, 0.0 is used.

### 0x01d - Commanded D Phase Current ###

Mode: Read/write

When in kFoc mode, this controls the desired D phase current.  Unless
you like burning power, with a BLDC motor you will typically want this
set to 0.  If unspecified, 0.0 is used.


#### 0x020 - Position command ####

Mode: Read/write

When in Position mode, this controls the desired position.  The
maximal negative integer, or NaN for float represents, "use the
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
motion when reaching the given position.  NaN / maximal negative means
no limit is applied.  If unspecified, NaN is used.

#### 0x027 - Watchdog timeout ####

Mode: Read/write

This determines the length of time for which this command is valid.
If this timeout expires before another command is received, the
controller will enter the Timeout state.  The default is 0.0, which
means to use the system-wide configured default.  NaN / maximal
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
position.  The maximal negative integer or NaN for float represents,
"there is no lower bound".  When special or the position is above this
bound (and also respecting the optional upper bound), only a
feedforward torque is applied.  When outside this bound, the PID
controller is used to force the position back to the bound.  If
unspecified, 0.0 is used.

### 0x041 - Stay within upper bound ###

Mode: Read/write

When in Stay Within mode, this controls the maximum allowable
position.  The maximal negative integer, or NaN for float represents,
"there is no upper bound". When special or the position is below this
bound (and also respecting the optional lower bound), only a
feedforward torque is applied.  When outside this bound, the PID
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
 - `6000` - position = 0x0060 = 96 = 34.56 degrees
 - `2001` - velocity = 0x0120 = 288 = 103.68 dps
 - `50ff` - feedforward torque = 0xff50 = -176 = 1.76 N*m
- `14` - read int16 registers
 - `04` - read 4 registers
 - `00` - starting at 0x000 (so 0x000 Mode, 0x001 Position, 0x002
   Velocity, 0x003 Torque)
- `17` - read 3x int8 registers
 - `0d` - starting at 0x00d (so 0x00d Voltage, 0x00e Temperature,
    0x00f Fault code)

Thus the whole CAN-FD message would be (in hex):

`01000a07206000200150ff140400170d`

To send this using the fdcanusb converter to a device configured at
the default address of 1, you could write.

`can send 8001 01000a07206000200150ff140400170d`

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
 - `5000` position is 0x0050 = 80 = 28.8 degrees
 - `0001` velocity is 0x0100 = 256 = 92.16 dps
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

### `d index` ###

Force the current recorded position to match exactly the given value.

```
d index <position>
```

### `d rezero` ###

Assuming that `motor.offset` is configured correctly, update the
current position to the closest one which is consistent with a given
output position.

```
d rezero <position>
```

### `d cal` ###

Enter the encoder calibration mode.  Upon starting this command, the
controller will spin the motor in voltage-FOC mode until the encoder
covers a full revolution, then repeat the process in the other
direction.  During the process, the current commanded phase and
encoder value are periodically emitted to the console.

Syntax:

```
d cal <magnitude> [options...]
```

Each optional element consists of a prefix character followed by a value.  Permissible options are:

* `s` - calibration speed in electrical revolutions per second


### `d flash` ###

Enter the bootloader.

## B.2 `tel` - telemetry ##

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


# C. Configurable values #

This section describes the configurable values which are most likely
to be modified for end-user applications.  Changes to all values take
effect *immediately*.  This may mean, that for instance, it is wise to
stop control loops before drastically changing control parameters.  Or
maybe not, it depends upon your goals.

## `id.id` ##

The servo ID presented on the CAN bus.  After this is modified, you need to immediately adjust which servo ID you communicate with in order to continue communication or save the parameters.

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

The minimum allowed control position value, measured in rotations.

## `servopos.position_max` ##

The maximum allowed control position value, measured in rotations.

## `servo.pid_position` ##

These configure the position mode PID controller.

* `kp/ki/kd` - PID gains with units of:
  * kp - Nm per rotation
  * ki - Nm/s per rotation
  * kd - Nm per rotation/s
* `iratelimit` - The maximum rate at which the integral term can
   wind up, in N*m/s.  <0 means "no limit"
* `ilimit` - The total maximum I term, in Nm
* `kpkd_limit` - The total maximum combined P and D terms, in Nm.
   <0 means "no limit"
* `max_desired_rate` - If non-zero, the commanded position is
  limited to change at this rate in Hz.

## `servo.pid_dq` ##

These have the same semantics as the position mode PID controller, and
affect the current control loop.

## `servo.max_voltage` ##

If the input voltage reaches this value, a fault is triggered and all
torque is stopped.

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


# D. Calibration #

Assuming your controller has firmware installed already, you can
calibrate the controller using the following procedure.

1. Set the `motor.unwrapped_position_scale` parameter using tview.
   This is the gearbox ratio.  A direct drive system with no gearbox
   should be 1.0.

2. Run the calibration tool:

```
./bazel-bin/moteus/tool/moteus_tool --target 1 --calibrate
```

WARNING: Any attached motor must be able to spin freely.  It will be spun in both directions and at high speed.


# E. Flashing firwmare #

The firmware can be flashed either using:

```
./moteus/flash.sh
```

Or using a bazel rule, which first ensures that all firmware sources
are built correctly.

```
tools/bazel test //moteus:flash
```

# F. Mechanical #

The current mechanical drawing for the controller can be found at:
[20200115-moteus-controller-r42-mechanical.pdf](https://github.com/mjbots/moteus/blob/master/hw/controller/r4.2/20200115-moteus-controller-r42-mechanical.pdf)

The current mechanical drawing for the qdd100 servo can be found at:
[20200315-qdd100-mechanical.pdf](https://github.com/mjbots/moteus/blob/master/hw/qdd100/20200315-qdd100-mechanical.pdf)

# G. Pinout #

## JST PH-3 CAN ##

Looking at the pins of the connector with the bottom of the board up
the pins are numbered 1 to 3 from left to right.

 - 1 - CAN_H
 - 2 - CAN_L
 - 3 - GND

NOTE: Ground may not be necessary, only one path through ground in a
system should exist to avoid ground loops.  In a typical robot
application with a common ground, that role is filled by the power
ground.  However, in desktop applications, it may be appropriate to
connect the CAN ground if the device power supply is otherwise
isolated.

## JST ZH-6 SWD ##

Looking at the pins of the connector with the top of the board up the
pins are numbered 1 to 6 from left to right.

 - 1 - NC
 - 2 - NRST
 - 3 - SWDIO
 - 4 - GND
 - 5 - SWCLK
 - 6 - 3.3V

## JST ZH-4 I2C ##

Looking at the pins of the connector with the bottom of the board up
the pins are numbered 1 to 4 from left to right.

 - 1 - 3.3V
 - 2 - SCL
 - 3 - SDA
 - 4 - GND

## XT30PW-M ##

Looking at the pins of the power connector with the top of the board
up, the ground pin is to the left with the chamfered corner and the
positive supply is to the right with the square corner.
