# Diagnostic Protocol

The following command set is intended for debugging and diagnostics.
It can be entered from `tview` or the `--console` mode of
`multiplex_tool` or `moteus_tool`.

## `d` - board debug

### `d stop`

This causes the controller to enter the "stopped" state, which
disables the motor driver.

### `d raw`

This enters the "raw" PWM mode.  Syntax:

```
d raw <pwm_a> <pwm_b> <pwm_c>
```

Where the pwm values are between 0.0 and 1.0.  A command to the idle
state is thus: `d raw 0.5 0.5 0.5`.

### `d pwm`

This enters voltage-FOC mode.  Syntax:

```
d pwm <phase> <magnitude> [<phase_rate>]
```

Where phase is in radians, magnitude is in volts, and the optional
phase rate is in radians per second.

### `d dq`

This enters the current controlled FOC mode.  Syntax:

```
d dq <d_A> <q_A>
```

### `d pos`

This enters the position control FOC mode.  Syntax:

```
d pos <pos> <vel> <max_torque> [options...]
```

Each optional element consists of a prefix character followed by a value.  Permissible options are:

- `p` - kp scale: the configured kp value is multiplied by this
  constant for the duration of this command
- `d` - kd scale: the configured kd value is multiplied by this
  constant for the duration of this command
- `i` - ki ilimit scale: the configured ilimit value is multiplied by
  this constant for the duration of this command
- `s` - stop position: when a non-zero velocity is given, motion stops
  when the control position reaches this value.
- `f` - feedforward torque in Nm
- `t` - timeout: If another command is not received in this many
  seconds, enter the timeout mode.
- `v` - velocity limit: the given value will override the global
  velocity limit for the duration of this command.
- `a` - acceleration limit: the given value will override the global
  acceleration limit for the duration of this command.
- `o` - fixed voltage override: while in affect, treat the control as
  if `fixed_voltage_mode` were enabled with the given voltage
- `c` - fixed current override: while in affect, treat the control
  like `fixed_voltage_mode`, but instead commanding a fixed current.
- `b` - if non-zero, then ignore all `servopos` position bounds

The position, velocity, maximum torque, and all optional fields have
the same semantics as for the register protocol documented above.

### `d tmt`

Enter the timeout mode.  This mode commands a zero velocity and can only be exited through the stopped state.

```
d tmt <pos> <vel> <max_torque> [options...]
```

Available options are identical to `d pos`.

### `d zero`

Enter the zero velocity state.  A zero velocity is commanded
regardless of position.

```
d zero <pos> <vel> <max_torque> [options...]
```

Available options are identical to `d pos`.  `pos` and `vel` are
ignored.

### `d within`

Enter the "stay within" state.  When the position is contained within
the given bounds, only the feedforward torque is applied.  Otherwise,
the position mode controller is used to hold the position at the
violated boundary.

```
d within <lowbound> <highbound> <max_torque> [options...]
```

The fields have the same semantics as for the register protocol
documented above.  The options are largely the same as for `d pos`.
Unsupported options include:

 * `s` - stop position
 * `o` - fixed voltage override
 * `c` - fixed current override

### `d brake`

Enter the "brake" state.  In this mode, all motor phases are shorted
to ground, resulting in a passive "braking" action.

### `d nearest`

Update the current position to the closest one which is consistent
with a given output position.

```
d rezero <position>
```

### `d exact`

Update the current position to exactly the given value.

```
d exact <position>
```

### `d req-reindex`

Reset the homing state to relative based, requiring any homing
procedure to be re-run.

```
d req-reindex
```

### `d recapture`

When in position mode, reset the control position and velocity to the
currently sensed values.

```
d recapture
```

### `d cfg-set-output`

Modify the configuration as required so that the current observed
position will be equal to the given value.  Note, the configuration is
not written to persistent storage.

```
d cfg-set-output <position>
```

### `d cal`

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


### `d flash`

Enter the bootloader.

NOTE: This is only intended for internal use.  Users who want to flash new firmware should use `moteus_tool`.

## `aux[12]` - Aux port manipulation

All commands are supported on both `aux1` and `aux2`.

### `aux1 out` - Set GPIO Output Values

```
aux1 out <data>
```

'data' is a single decimal integer.  Only bits associated with pins
configured as digital outputs are used, the remainder are ignored.

### `aux1 pwm` - Set PWM Output Values

```
aux1 pwm <pin> <value>
```

'pin' is a number from 0 to 4 that gives the pin to set.

'value' is a floating point value between 0.0 and 1.0 that gives the
output duty cycle for the pin.

### `aux1 ic-cmd` - Initiate an iC-PZ command

```
aux1 ic-cmd <HEXBYTE>
```

An example of entering analog mode calibration.

```
aux1 ic-cmd B0
```

### `aux1 ic-wr` - Write iC-PZ register

```
aux1 ic-wr <reg> <data>
```

Register is one byte in hex, data is 1 or more bytes in hex.

Example of switching to memory page 0.

```
aux1 ic-wr 40 00
```

### `aux1 ic-rd` - Read iC-PZ register

```
aux1 ic-rd <reg> <length>
```

The register is one byte in hex, length is a decimal value indicating
the number of bytes to read.

Example of reading the temperature data:

```
aux1 ic-rd 4e 2
```

### `aux1 ic-extra` - Select alternate periodic data

```
aux1 ic-extra <fields>
```

"fields" is a decimal bitmask of items to read at 1000Hz and display
in the diagnostic stream.

bit 0 - the diagnosis command result
bit 1 - the contents of the AI_PHASES registers

Example of enabling the AI_PHASES registers only:

```
aux1 ic-extra 2
```

## `tel` - telemetry

### `tel get`

Retrieve the contents of a given channel.  Text or binary mode is
determined via the `tel fmt` or `tel text` commands, and defaults to
binary mode.

```
tel get <channel>
```

### `tel list`

List all available telemetry channels.

### `tel schema`

Report the binary schema associated with the given channel.

```
tel schema <channel>
```

The schema is reported as follows:

```
emit <channel>\r\n
<LE uint32 size><data>
```

### `tel rate`

Control the rate at which a given channel is emitted.

```
tel rate <channel> <rate_ms>
```

### `tel fmt`

Select whether a given channel will be emitted in binary or text form.

```
tel fmt <channel> <format>
```

`format` is an integer, non-zero signifies the data should be emitted
as text.

### `tel stop`

Stop emitting all periodic telemetry data.

### `tel text`

Switch all channels to text mode.

## `conf` - configuration

NOTE: Any commands that change parameters, such as `conf set`, `conf
load`, or `conf default`, if executed manually in `tview` will not
automatically update the UI.  `tview` must be restarted to display the
new parameters in the UI.

### `conf enumerate`

Print the current value of all configurable parameters.

### `conf get`

Get the value of a single configurable parameter.

```
conf get <item>
```

### `conf set`

Set the value of a single configurable parameter in RAM.

```
conf set <item> <value>
```

### `conf load`

Load all configurable values from persistent storage.  This will
overwrite their current values in RAM.

### `conf write`

Write the current value of all configurable parameters from RAM to
persistent storage.

### `conf default`

Update the RAM values of all configurable parameters to their firmware
default.

NOTE: This only updates RAM, not the persistent storage.  A `conf
write` will be necessary to save these to persistant storage.
