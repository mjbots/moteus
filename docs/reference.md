moteus controller reference

# register command set #

The register command set is intended for use in real-time
applications.  It provides multiple levels of conciseness, and is
possible to operate at over 1kHz on the provided FD-CAN communications
bus.

Each FD-CAN frame contains one or more "subframes" as defined in [multiplex/format.h](https://github.com/mjbots/mjlib/blob/master/mjlib/multiplex/format.h)

The allowed registers are documented in [moteus_register_map.md](../moteus/moteus_register_map.md)


# diagnostic command set #

The following command set is intended for debugging and diagnostics.
It can be entered from `tview` or the `--console` mode of
`multiplex_tool` or `moteus_tool`.

## `d` - board debug ##

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

* `p` - kp scale: the configured kp value is multiplied by this
  constant for the duration of this command
* `d` - kd scale: the configured kd value is multiplied by this
  constant for the duration of this command
* `s` - stop position: when a non-zero velocity is given, motion stops
  when the control position reaches this value.
* `f` - feedforward torque in Nm
* `t` - timeout: If another command is not received in this many
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

## `tel` - telemetry ##

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

## `conf` - configuration ##

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
./bazel-bin/moteus/moteus_tool --target 1 --calibrate
```

WARNING: Any attached motor must be able to spin freely.  It will be spun in both directions and at high speed.


# Flashing firwmare #

The firmware can be flashed either using:

```
./moteus/flash.sh
```

Or using a bazel rule, which first ensures that all firmware sources
are built correctly.

```
tools/bazel test //moteus:flash
```

# Mechanical #

The current mechanical drawing can be found at: [20200115-moteus-controller-r42-mechanical.pdf](https://github.com/mjbots/moteus/blob/master/hw/controller/r4.2/20200115-moteus-controller-r42-mechanical.pdf)

# Pinout #

## JST PH-3 CAN ##

Looking at the pins of the connector with the bottom of the board up
the pins are numbered 1 to 3 from left to right.

 * 1 - CAN_H
 * 2 - CAN_L
 * 3 - GND

NOTE: Ground may not be necessary, only one path through ground in a
system should exist to avoid ground loops.  In a typical robot
application with a common ground, that role is filled by the power
ground.  However, in desktop applications, it may be appropriate to
connect the CAN ground if the device power supply is otherwise
isolated.

## JST ZH-6 SWD ##

Looking at the pins of the connector with the top of the board up the
pins are numbered 1 to 6 from left to right.

 * 1 - NC
 * 2 - NRST
 * 3 - SWDIO
 * 4 - GND
 * 5 - SWCLK
 * 6 - 3.3V

## JST ZH-4 I2C ##

Looking at the pins of the connector with the bottom of the board up
the pins are numbered 1 to 4 from left to right.

 * 1 - 3.3V
 * 2 - SCL
 * 3 - SDA
 * 4 - GND

## XT30PW-M ##

Looking at the pins of the power connector with the top of the board
up, the ground pin is to the left with the chamfered corner and the
positive supply is to the right with the square corner.
