moteus controller reference

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

The full set of allowed registers are documented in
[moteus_register_map.md](../moteus/moteus_register_map.md).  An
abbreviated set are listed here.

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

### A.2.b Registers ###

#### 0x000 - Mode ####

Mode: Read/write

The current operational mode of the servo.  Not all values are valid
to write.

- 0 => stopped = writeable, clears faults
- 1 => fault
- 2,3,4 => preparing to operate
- 9 => position
- 10 => timeout
- 11 => zero velocity

#### 0x001 - Position ####

Mode: Read only

The current position of the servo, measured in rotations of the output
shaft.  The maximum negative integer is reserved and will not be
reported.  Mapping:

- int8 => 1 LSB => 0.01 rotation => 3.6 degrees (range of -1.27 to 1.27)
- int16 => 1 LSB => 0.0001 rotation => 0.036 degrees (range of -3.2767 to 3.2767)
- int32 => 1 LSB => 0.00001 rotation => 0.0036 degrees

#### 0x002 - Velocity ####

Mode: Read only

The current velocity of the servo, measured in Hz at the output shaft.
Mapping:

- int8 => 1 LSB => 0.1Hz / 36 dps
- int16 => 1 LSB => 0.00025 Hz > 0.36 dps
- int32 => 1 LSB => 0.00001 Hz => 0.0036 dps

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

#### 0x020 - Position command ####

Mode: Read/write

When in Position mode, this controls the desired position.  The same
integral mapping is used as for 0x001 Position.  The maximal negative
integer, or NaN for float represents, "use the current position
value".

#### 0x021 - Velocity command ####

Mode: Read/write

When in Position mode, advance the commanded position at the given
velocity in Hz.

As a special case, if the 0x020 position is unset, and 0x023 stop
position is set, the sign of this is ignored, and is instead selected
so that the motor will move towards the stop position.

#### 0x022 - Feedforward torque ####

Mode: Read/write

When in Position mode, add the given feedforward torque after applying
all regular control loops.  Note, this is torque at the output shaft.

#### 0x023 - Kp scale ####

Mode: Read/write

When in Position mode, shrink the proportional control term by the
given factor.  Integral types are applied as for PWM.

#### 0x024 - Kd scale ####

Mode: Read/write

When in Position mode, shrink the derivative control term by the given
factor.  Integral types are applied as for PWM.  This is internally
limited to be no more than the kp scale.

#### 0x025 - Maximum torque ####

When in Position mode, the maximum torque to be applied.  This
defaults to the system-wide configured maximum torque.

#### 0x026 - Commanded stop position ####

When in Position mode, and a non-zero velocity is commanded, stop
motion when reaching the given position.  NaN / maximal negative means
no limit is applied.

#### 0x027 - Watchdog timeout ####

Mode: Read/write

This determines the length of time for which this command is valid.
If this timeout expires before another command is received, the
controller will enter the Timeout state.  The default is 0.0, which
means to use the system-wide configured default.  NaN / maximal
negative means apply no enforced timeout.

## A.3 Example ##

A single CAN-FD frame can be used to command the servo, and initiate a
query of certain registers.  An example frame might look like the
following, encoded in hex with annotations.

- `01` - write a single int8 register (number of registers is encoded
  in the 2 LSBs)
 - `00` - start register number "Mode"
 - `09` - "position" mode
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

`01000907206000200150ff140400170d`

To send this using the fdcanusb converter to a device configured at
the default address of 1, you could write.

`can send 8001 01000907206000200150ff140400170d`

The `80` in ID is used for two purposes.  The high bit being set
forces the device to respond (otherwise it will not respond, even if
query commands are sent).  The remaining bits are the "ID" to respond
to.  In response to this command, a possible response from the servo
would look like:

`rcv 100 24040009005000000170ff230d181400`

Decoded, that means:

- `100` from device "1" to device "0"

- `24` reply with int16 values
 - `04` 4 registers
 - `00` starting at register 0
 - `0900` in mode 9 - Position
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


# Calibration #

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


# C. Flashing firwmare #

The firmware can be flashed either using:

```
./moteus/flash.sh
```

Or using a bazel rule, which first ensures that all firmware sources
are built correctly.

```
tools/bazel test //moteus:flash
```

# D. Mechanical #

The current mechanical drawing for the controller can be found at:
[20200115-moteus-controller-r42-mechanical.pdf](https://github.com/mjbots/moteus/blob/master/hw/controller/r4.2/20200115-moteus-controller-r42-mechanical.pdf)

The current mechanical drawing for the qdd100 servo can be foudn at:
[20200315-qdd100-mechanical.pdf](https://github.com/mjbots/moteus/blob/master/hw/qdd100/20200315-qdd100-mechanical.pdf)

# E. Pinout #

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
