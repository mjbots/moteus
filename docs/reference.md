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
the switching frequency (by default 30kHz).

The outermost stage is an optional limited acceleration and velocity
trajectory planner.  Within that is an integrated position/velocity
PID controller with optional feedforward torque.  The output of that
loop is a desired torque/current for the Q phase of the FOC
controller.

The inner stage is a current mode PI controller.  Its output is the
desired voltage value for the Q phase.  Then the magnetic encoder is
used to map the D/Q phase voltage values to the 3 phases of the motor.

![Control Structure](control_structure.png)

More precisely, the "Position Controller" implements the following
control law:

```
acceleration = trajectory_follower(command_position, command_velocity)
control_velocity = command_velocity OR control_velocity + acceleration * dt OR 0.0
control_position = command_position OR control_position + control_velocity * dt
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
current_integrator = limit(current_integrator + ki * current_error, ilimit)
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
greater than or equal to 0 ([reference](#servomax_position_slip)).
When it is larger, more external disturbances will be rejected, but
the controller will also "catch up" when the magnitude of external
disturbances is decreased.

### Low Speed or Precise Positioning ###

For either operation at very low speeds, or when precise positioning
performance is desired, it is recommended to configure a non-zero `ki`
and `ilimit` term in the position controller
([reference](#servopid_position)).  This will compensate for cogging
torque (at the expense of overall torque bandwidth).  It may also be
beneficial to select a alternate values for `moteus_tool --cal-bw-hz`
during calibration (higher or lower).

### Constant Acceleration Trajectories ###

Velocity and acceleration limits can be configured either globally, or
on a per-command basis which will cause moteus to internally generate
continuous acceleration limited trajectories to reach the given
position and velocity.  Once the trajectory is complete, the command
velocity is continued indefinitely.

### Jerk Limited Trajectories ###

moteus only supports acceleration limited internal trajectories.  To
approximate a constant jerk trajectory, the host processor should send
a sequence of piecewise linear constant velocity trajectories which
approximate the desired one.  This would be done by sending commands
consisting of at least a position and velocity at some moderate to
high rate while disabling the internal velocity and acceleration
limits.

### High Torque Bandwidth ###

High torque bandwidth is desired for legged robots, or other
applications where it is necessary to respond to external disturbances
as rapidly as possible or accelerate the load maximally.  For these
applications, it is recommended to have no integrative term in the
position controller ([reference](#servopid_position)).  When
calibrating the motor, you may use the `moteus_tool --cal-bw-hz`
option to select a bandwidth higher than the default of 100Hz (or
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

## Encoder Configuration ##

moteus includes an onboard on-axis magnetic encoder and supports a
number of options for using external encoders.  To understand how to
use them, we first need to know the two things that moteus needs
encoders for:

- *Commutation*: moteus needs to know the electrical relationship
  between the stator and the rotor in order to apply torque.
- *Output*: moteus needs to know the position and velocity of the
  output shaft in order to follow commanded trajectories.

A variety of hardware encoder types can be used to fulfill those
functions.  Currently supported options include:

| Name            | Interface Method | Max Resolution | Mounting    | Price     |
|-----------------|------------------|----------------|-------------|-----------|
| Onboard AS5047P | SPI (integrated) | 14 bits        | on-axis     | Included  |
| AS5047x         | SPI              | 14 bits        | on-axis     | $         |
| AS5048B         | I2C              | 14 bits        | on-axis     | $         |
| AS5600          | I2C              | 12 bits        | on-axis     | $         |
| AksIM-2         | RS422 w/ 5V      | 20 bits        | off-axis    | $$$       |
| CUI AMT21x      | RS422 w/ 5V      | 14 bits        | shaft       | $$        |
| CUI AMT22x      | SPI w/ 5V        | 14 bits        | shaft       | $$        |
| MA600           | SPI              | 16 bits        | on/off-axis | $         |
| MA732           | SPI              | 14 bits        | on/off-axis | $         |
| iC-PZ           | SPI w/ 5V        | 22 bits        | off-axis    | $$$       |
| Quadrature      | Quadrature       | N/A            | x           | x         |
| Sine/Cosine     | Sine/Cosine      | 10 bits        | x           | x         |
| Hall effect     | Hall effect      | x              | x           | $         |

By default, the onboard magnetic encoder (AS5047P) is assumed to sense
the rotor.  It is also used as the source for position and velocity of
the output constrained by a configurable reduction ratio.

Configuration is performed in 3 stages, first is the auxiliary port
configuration, second is encoder source configuration, and finally is
the output configuration.

![IO Structure](io_structure.png)

### Auxiliary Port ###

There are two auxiliary ports on moteus, each with pins that can be
used for various functions.

ELECTRICAL NOTES:
 * The 3.3V supply pins can power external peripherals:
   * r4.5/c1: 50mA
   * r4.8/r4.11/n1/x1: 100mA
 * The 5V supply pins can power external peripherals:
   * r4: not present
   * c1: 100mA
   * n1/x1: 200mA
 * The 12V supply pins can power external peripherals:
   * r4/c1/n1: not present
   * x1: 150mA
 * Some pins are 5V tolerant.  Those not marked as such in the pin
   option table are 3.3V only.

The following capabilities can be used.  Some are supported on any
pins, others only on select pins.

#### Software Quadrature Input ####

Pins: any

The software quadrature feature uses GPIO pins to read incremental
quadrature encoders.  It is capable of counting at 200,000 counts per
second without error, but incurs processor overhead that increases
with count rate.  Higher overhead means the latency to respond to CAN
messages increases.

#### Hall sensor ####

Pins: any

3 hall sensor inputs are required.  For many hall sensors, the pullup
must be configured for the auxiliary port pin in question.

When using hall effect sensors as the commutation source, calibration
with moteus_tool requires the `--cal-hall` option be passed.

#### Index ####

Pins: any

This is a single pin that is high when the encoder is in a known
location.  It can be fed from the "I" signal of an ABI output, or a
dedicated homing sensor.

#### GPIO Input ####

Pins: any

Any pin may be designated as a GPIO input.  Its value may be read over
the diagnostic or register protocols.

#### I2C Master ####

Pins: select

Between 1 and 3 I2C devices may be periodically polled at rates up to
200Hz.  The associated pins on moteus r4.5/8/11 have permanently
configured 2kohm pullup resistors.

#### SPI Master ####

Pins: select

A variety of SPI based peripherals are supported.  This mode is also
used for the onboard encoder, which when enabled, claims the CLK,
MOSI, and MISO pins on auxiliary port 1.

#### Hardware Quadrature Input ####

Pins: select

Hardware quadrature pins use microcontroller peripherals to process
quadrature input at any speed with no processor overhead.

#### Sine/cosine ####

Pins: select

Analog sine/cosine inputs are supported with a configurable common
mode voltage.

#### Analog input ####

Pins: select

Arbitary analog inputs can be read either over the diagnostic or
register protocol.

#### UART ####

Pins: select

A variety of asynchronous serial encoders and debugging facilities are
supported.

The moteus-n1 and moteus-x1 additionally have a hardware RS422
transceiver connected to aux1's pins D and E which can be enabled
through configuration.  RS485 devices like the CUI AMT21x can be used
if the RS422 pin Y is connected to A and RS422 pin Z is connected to
B.

### Pin Options ###

The following table shows which pins can be used for the unique capabilities:

#### AUX1 / ENC ####

| moteus r4.5/8/11 | Con | Aux | SPI  | ADC/Sin/Cos | I2C | HW Quad/PWM | UART | 5VT |
|------------------|-----|-----|------|-------------|-----|-------------|------|-----|
| 3.3V  (3)        | 1   |     |      |             |     |             |      |     |
| C                | 2   | 0   | X    |             |     |             |      | X   |
| GND (G)          | 3   |     |      |             |     |             |      |     |
| K                | 4   | 1   | CLK  | X           |     |             |      |     |
| I                | 5   | 2   | MISO | X           |     |             |      |     |
| O                | 6   | 3   | MOSI | X           |     |             |      |     |

| moteus c1/n1/x1  | Con | AUX | SPI  | ADC/Sin/Cos | I2C | HW Quad/PWM | UART | 5VT |
|------------------|-----|-----|------|-------------|-----|-------------|------|-----|
| 5V (5)           | 1   |     |      |             |     |             |      |     |
| 3.3V (3)         | 2   |     |      |             |     |             |      |     |
| A                | 3   | 0   | CLK  | X           |     |             |      |     |
| B *              | 4   | 1   | MISO |             |     | 3.1         | RX   |     |
| C                | 5   | 2   | MOSI | X           |     | 3.2         |      |     |
| D                | 6   | 3   |      |             | SCL | 2.1         | RX   | X   |
| E                | 7   | 4   |      |             | SDA | 2.2         | TX   | X   |
| GND (G)          | 8   |     |      |             |     |             |      |     |

NOTE: For moteus n1, the B pin software configured pullup cannot be
used effectively.  Thus the B pin is unsuitable for open-drain inputs
like hall effect sensors unless external pullups are provided.

NOTE: For moteus c1, only the D and E pins are exposed, and only on an
unpopulated 0.05" through hole land pattern.  Additionally, I2C
pullups are not available on moteus-c1 for aux1.

#### AUX2 / ABS ####

| moteus r4.5/8/11 | Con | Aux | SPI  | ADC/Sin/Cos | I2C | HW Quad/PWM | UART | 5VT |
|------------------|-----|-----|------|-------------|-----|-------------|------|-----|
| 3.3V (3)         | 1   |     |      |             |     |             |      |     |
|                  | 2   | 0   |      |             | SCL |             | RX   | X   |
|                  | 3   | 1   |      |             | SDA |             | TX   | X   |
| GND (G)          | 4   |     |      |             |     |             |      |     |
| DBG 1            |     | 2   |      |             |     |             |      | X   |
| DBG 2            |     | 3   |      |             |     |             |      | X   |

| moteus c1/n1/x1  | Con | AUX | SPI  | ADC/Sin/Cos | I2C | HW Quad/PWM | UART | 5VT |
|------------------|-----|-----|------|-------------|-----|-------------|------|-----|
| 5V (5)           | 1   |     |      |             |     |             |      |     |
| 3.3V (3)         | 2   |     |      |             |     |             |      |     |
| A                | 3   | 0   | CLK  | X           |     |             |      | X   |
| B                | 4   | 1   | MISO | X           | SDA |             | RX   | X   |
| C                | 5   | 2   | MOSI | X           | SCL | 4.1         | TX   | X   |
| D                | 6   | 3   |      |             |     | 4.2         | RX   | X   |
| GND (G)          | 7   |     |      |             |     |             |      |     |

NOTE: For moteus r4.5/8/11, DBG 1/2 are not present on the ABS
connector, but are exposed pads on the circuit board.

#### Pin Configuration ####

Auxiliary port configuration is achieved in two steps.  First, the
`aux[12].pins.X.mode` value is set to the proper capability for each
pin.  `aux[12].pins.X.pull` can be used to configure an optional
pullup or pulldown for some modes.  Second, the corresponding
capabilities must be configured in one of the capability specific
sections of `aux[12]`.  For instance, for each auxiliary port, the SPI
configuration in `aux[12].spi` has a `mode` to select what the slave
device is and a `rate_hz` to define the frequency of the SPI
peripheral.

For I2C ports, up to 3 different slave devices may be configured, in
each of `aux[12].i2c.devices.[012]`.

The diagnostic values in `aux[12]` can be used to monitor for errors
from mis-configuration or mis-operation.

### Source Configuration ###

Once the auxiliary ports have been configured, next encoder sources
should be configured in `motor_position.sources`.  Up to 3 sources may
be configured, with the available types roughly corresponding to the
categories available from the auxiliary ports.  Typically, source 0 is
used for the sensor that is used for commutation.

For each source `motor_position.sources.[012].aux_number` should be
either "1" or "2", to select which auxiliary port the sensor should be
read from.  `motor_position.sources.[012].type` selects the type of
sensor.  For I2C based sources,
`motor_position.sources.[012].i2c_device` selects *which* I2C device
on that auxiliary port is used.

For sources that are incremental only, like quadrature, a source level
index may be configured in
`motor_position.sources.[012].incremental_index`.  This should be used
if the incremental source is needed to provide position for
commutation.

Each source can be marked as measuring the rotor or the output in
`motor_position.sources.[012].reference`.

Each source has configuration that determines how to map the raw value
provided by the device into a rotary angle.  `cpr` is the number of
counts per revolution, `offset` provides an integral count offset, and
`sign` can be used to invert the reading.  The final reading is
`(raw + offset) * sign / cpr`.

Finally, each source has a configurable low pass filter, with cutoff
frequency set by `motor_position.sources.[012].pll_filter_hz`.  It may
be set to 0 to disable the filter.  If disabled no velocity will be
interpolated on sensors that do not provide it natively.

### Output Configuration ###

The third major stage controls how the sources are used by the motor
controller.  Some source selections may be left at -1, which disables
that feature.

`motor_position.commutation_source` selects which source is used to
provide rotor position for commutation purposes.  It is typically left
at source 0 and is required.  Note, quadrature sources are not
recommended for commutation as they can lose counts and require
additional application level homing support on each power-on.

`motor_position.output.source` selects which source is used to provide
output position and velocity and is required.

`motor_position.output.offset` and `motor_position.output.sign` are
used to transform the output position to achieve a given zero point
and rotational direction.

`motor_position.output.reference_source` optionally configures a
source that is used solely to disambiguate the output position at
startup.  It could be a low-rate I2C based sensor for instance, or an
index source configured from a homing switch.

Finally, `motor_position.rotor_to_output_ratio` defines the number of
turns of the output for one turn of the rotor.  This is used to map
the readings from sensors that are defined relative to one into the
other.  For gear reducers (almost all configurations), this will be
less than one.  For example, a 4x gear reduction would be entered as
0.25.

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

*0x30, 0x31* - write/read error

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

#### A.2.a.8 Acceleration (measured in revolutions / s^2) ####

- int8 => 1 LSB => 0.05 l/s^2
- int16 => 1 LSB => 0.001 l/s^2
- int32 => 1 LSB => 0.00001 l/s^2

#### A.2.a.9 PWM and kp/kd scale (unitless) ####

- int8 => 1 LSB => (1/127) - 0.007874
- int16 => 1 LSB => (1/32767) - 0.000030519
- int32 => 1 LSB => (1/2147483647) - 4.657e-10

#### A.2.a.10 Power (W) ####

- int8 => 1 LSB => 10.0 W
- int16 => 1 LSB => 0.05 W
- int32 => 1 LSB => 0.0001 W

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

When sending a command, this register must be written before any other
registers in the command frame.

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

#### 0x007 - Measured electrical power ####

Mode: Read only

The estimated electrical power applied to the motor if positive.  If
negative, power applied to the DC input bus.

#### 0x00a - Motor temperature ####

Mode: Read only

The current motor temperature, measured in degrees celsius.  This will
only be valid if an NTC thermistor is connected to the TEMP pads,
`motor.thermistor_ohm` is set to the correct resistance, and
`servo.enable_motor_temperature` is set to 1.

#### 0x00b - Trajectory complete ####

Mode: Read only

Non-zero if the current acceleration or velocity limited trajectory is
complete, and the controller is following the final velocity.

#### 0x00c - Home state ####

Mode: Read only

* 0 - *relative only* - the position is not referenced to anything
* 1 - *rotor* - the position is referenced to the rotor
* 2 - *output* - the position has been referenced to the output,
  either with an output referenced encoder, or with a "set output
  nearest" or "set output exact" command


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

The full list can be found at: [fw/error.h](../fw/error.h#L25)


#### 0x010 / 0x011 / 0x012 - PWM phase A / B / C ####

Mode: Read/write

When in Pwm mode, this controls the raw PWM value for phase A, B, and
C.  If unspecified, 0.0 is used.

#### 0x014 / 0x15 / 0x16 - Voltage phase A / B / C ####

Mode: Read/write

When in Voltage mode, this controls the voltage applied to phase A,
B, and C.  If unspecified, 0.0 is used.


#### 0x018 - Voltage FOC Theta ####

Mode: Read/write

When in Voltage Foc mode, this controls the desired electrical phase.
Integral types use the PWM mapping.  If unspecified, 0.0 is used.

#### 0x019 - Voltage FOC Voltage ####

Mode: Read/write

When in Voltage Foc mode, this controls the desired applied phase
voltage.  If unspecified, 0.0 is used.

#### 0x01a - D Voltage ####

Mode: Read/write

When in Voltage Dq mode, this controls the desired applied D voltage.
If unspecified, 0.0 is used.

#### 0x01b - Q Voltage ####

Mode: Read/write

When in kVoltageDq mode, this controls the desired applied Q voltage.
If unspecified, 0.0 is used.

#### 0x01c - Commanded Q Phase Current ####

Mode: Read/write

When in Current mode, this controls the desired Q phase current.  If
unspecified, 0.0 is used.

#### 0x01d - Commanded D Phase Current ####

Mode: Read/write

When in Current mode, this controls the desired D phase current.  Unless
you like burning power, with a BLDC motor you will typically want this
set to 0.  If unspecified, 0.0 is used.

#### 0x1e - Voltage FOC Theta Rate ####

Mode: Read/write

When in Voltage Foc mode, this controls the rate of change of
electrical phase.  Integral types use the velocity mapping.


#### 0x020 - Position command ####

Mode: Read/write

When in Position mode, this controls the desired position.

The maximally negative integer, or NaN for float represents, "use the
current position value".

If unspecified, 0.0 is used.

Note, in the absence of any configured or commanded velocity or
acceleration limits, the controller will attempt to achieve this
position *right now* subject to the kp and kd constants.

#### 0x021 - Velocity command ####

Mode: Read/write

When in Position mode, advance the desired position at the given
velocity in Hz.

As a special case, if the 0x020 position is unset, and 0x026 stop
position is set, the sign of this is ignored, and is instead selected
so that the motor will move towards the stop position.

The maximally negative integer, or NaN for float, is treated the same
as 0.0.

If unspecified, 0.0 is used.

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

#### 0x028 - Velocity limit ####

Mode: Read/write

This can be used to override the global velocity limit for internally
generated trajectories.

If negative, then no limit is applied.

If unspecified, it is NaN / maximally negative, which implies to use
the global configurable default.

#### 0x029 - Acceleration limit ####

Mode: Read/write

This can be used to override the global acceleration limit for
internally generated trajectories.

If negative, then no limit is applied.

If unspecified, it is NaN / maximally negative, which implies to use
the global configurable default.

#### 0x02a - Fixed voltage override ####

Mode: Read/write

If specified and not-NaN, then the control mode will temporarily be in
the "fixed voltage" mode, regardless of the current setting of
`servo.fixed_voltage_mode`.

#### 0x02b - Ki ilimit scale ####

Mode: Read/write

When in Position mode, shrink the integral term's windup limit by the
given factor.  Integral types are applied as for PWM.  If unspecified,
1.0 is used.

#### 0x02c - Fixed current override ####

Mode: Read/write

If specified, then the control mode will temporarily be in the "fixed
current" mode.  This is parallel to "fixed voltage" mode, but instead
of a fixed voltage, a fixed D axis current is controlled.

#### 0x02d - Ignore position bounds ####

Mode: Read/write

If specified and non-zero, `servopos.position_min` and
`servopos.position_max` will be ignore.

#### 0x030 - Proportional torque ####

Mode: Read

This reports the torque contribution from the proportional term in the
PID controller.

#### 0x031 - Integral torque ####

Mode: Read

This reports the torque contribution from the integral term in the
PID controller.

#### 0x032 - Derivative torque ####

Mode: Read

This reports the torque contribution from the derivative term in the
PID controller.

#### 0x033 - Feedforward torque ####

Mode: Read

This reports the feedforward contribution in the PID controller.

#### 0x034 - Total control torque ####

Mode: Read

This reports the total commanded torque from the position mode
controller.  This is also reported in 0x03a.

#### 0x038 - Control Position ####

Mode: Read

This reports the current trajectory control position, in modes where
that is valid.  When velocity or acceleration limiting is enabled, the
control position will follow the desired limits to achieve the command
position.

#### 0x039 - Control Velocity ####

Mode: Read

This reports the current velocity control value, in modes where that
is valid.  When velocity or acceleration limiting is enabled, the
control velocity will follow the desired limits to achieve the control
position and velocity.

#### 0x03a - Control Torque ####

Mode: Read

The torque commanded by the control loop.  This is the same as 0x034.

#### 0x03b - Position Error ####

Mode: Read

The current sensed position minus the control position.

#### 0x03c - Velocity Error ####

Mode: Read

The current sensed velocity minus the control velocity.

#### 0x03d - Torque Error ####

Mode: Read

The current sensed torque minus the control torque.


#### 0x040 - Stay within lower bound ####

Mode: Read/write

When in Stay Within mode, this controls the minimum allowable
position.  The maximally negative integer or NaN for float represents,
"there is no lower bound".  When special or the position is above this
bound (and also respecting the optional upper bound), only a
feedforward torque is applied.  When outside this bound, the PID
controller is used to force the position back to the bound.  If
unspecified, 0.0 is used.

#### 0x041 - Stay within upper bound ####

Mode: Read/write

When in Stay Within mode, this controls the maximum allowable
position.  The maximally negative integer, or NaN for float
represents, "there is no upper bound". When special or the position is
below this bound (and also respecting the optional lower bound), only
a feedforward torque is applied.  When outside this bound, the PID
controller is used to force the position back to the bound.  If
unspecified, 0.0 is used.

#### 0x042 - Feedforward torque ####

A shadow of the 0x022 register.

#### 0x043 - Kp scale ####

A shadow of the 0x023 register.

#### 0x044 - Kd scale ####

A shadow of the 0x024 register.

#### 0x045 - Maximum torque ####

A shadow of the 0x025 register.

#### 0x046 - Watchdog timeout ####

A shadow of the 0x027 register.

#### 0x047 - Ki ilimit scale ####

A shadow of the 0x02b register.

#### 0x048 - Ignore position bounds ####

A shadow of the 0x02d register.

#### 0x050 - Encoder 0 Position ####

Mode: Read only

Reports the current filtered position of the encoder configured in
slot 0.

#### 0x051 - Encoder 0 Velocity ####

Mode: Read only

Reports the current filtered velocity of the encoder configured in
slot 0.

#### 0x052 - Encoder 1 Position ####

Mode: Read only

Reports the current filtered position of the encoder configured in
slot 1.

#### 0x053 - Encoder 1 Velocity ####

Mode: Read only

Reports the current filtered velocity of the encoder configured in
slot 1.

#### 0x054 - Encoder 2 Position ####

Mode: Read only

Reports the current filtered position of the encoder configured in
slot 2.

#### 0x055 - Encoder 2 Velocity ####

Mode: Read only

Reports the current filtered velocity of the encoder configured in
slot 2.

#### 0x058 - Encoder Validity ####

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

#### 0x05c - Aux1 GPIO Command ####

Mode: Read/write

The current output command for any GPIOs configured as an output on
aux1 as a bitfield.  Not all bits may be used, as bit 0 is always for
pin 1, whether or not it is configured as a GPIO output.

#### 0x05d - Aux2 GPIO Command ####

Mode: Read/write

The current output command for any GPIOs configured as an output on
aux2 as a bitfield.  Not all bits may be used, as bit 0 is always for
pin 1, whether or not it is configured as a GPIO output.

#### 0x05e - Aux1 GPIO Status ####

Mode: Read only

The current input value of any GPIOs configured as an input on aux1 as
a bitfield.  Not all bits may be used, as bit 0 is always for pin 1,
whether or not it is configured as a GPIO input.

#### 0x05f - Aux2 GPIO Status ####

Mode: Read only

The current input value of any GPIOs configured as an input on aux2 as
a bitfield.  Not all bits may be used, as bit 0 is always for pin 1,
whether or not it is configured as a GPIO input.

#### 0x060/0x064 - Aux1 Analog Inputs ####

Mode: Read only

The current input value of any analog inputs configured on aux1.  The
registers are associated with pins 1-5, regardless of whether they are
configured as an analog input.  Each value is scaled as a PWM from 0
to 1.

#### 0x068/0x06c - Aux2 Analog Inputs ####

Mode: Read only

The current input value of any analog inputs configured on aux2.  The
registers are associated with pins 1-5, regardless of whether they are
configured as an analog input.  Each value is scaled as a PWM from 0
to 1.

#### 0x070 - Millisecond Counter ####

Mode: Read only

Increments once per millisecond.  It wraps at the maximum value for
the queried type to the minimum value for that type.  For floating
point types, it counts integers from 0 to 8388608.

#### 0x071 - Clock Trim ####

Mode: Read/write

An integer which can trim the clock rate of the microprocessor on the
moteus controller.  Positive values speed it up and negative values
slow it down.  Each integer step roughly corresponds to a 0.25% change
in speed.

WARNING: Changing the speed affects all processes driven by the
microcontroller, including CAN communication.  Thus setting this to a
non-zero value may prevent future CAN communications.

#### 0x076/0x07a - Aux1 PWM Outputs ####

Mode: Read/write

The current output PWM value for the given pin on Aux1.  PWM mapping
is used for integral types.

#### 0x07b/0x07f - Aux2 PWM Outputs ####

Mode: Read/write

The current output PWM value for the given pin on Aux1.  PWM mapping
is used for integral types.

#### 0x100 - Model Number ####

Name: Model Number
Mode: Read only

This returns a 32 bit model number.

#### 0x101 - Firmware Version ####

Mode: Read only

This returns a 32 bit firmware version, encoded bytewise as
major.minor.micro.  i.e. 0x010304 is version 1.3.4


#### 0x102 - Register map version ####

Mode: Read only

This returns a number that indicates how to interpret all registers.


#### 0x110 - Multiplex ID ####

Name: Multiplex ID
Mode: Configurable

This controls the primary ID used to access the device over the
multiplex RS485 bus.  It can only be between 1 and 127.  (0 is
reserved as the broadcast address).

#### 0x120 - 0x122 - Serial Number ####

Name: Serial Number
Mode: Read only

This returns a 96 bit serial number, least significant word first.


#### 0x130 - Set Output Nearest ####

Mode: Write only

When sent, this causes the servo to select a whole number of internal
motor rotations so that the final position is as close to the given
position as possible.

#### 0x131 - Set Output Exact ####

Mode: Write only

When sent, the servo will force the output position to be the exact
specified value.

#### 0x132 - Require Reindex ####

Mode: Write only

When sent with any value, the servo will require that any index
position be re-located before control can begin.  Regardless, the
position will reset to an arbitrary value consistent with the current
encoder settings.

#### 0x133 - Recapture Position and Velocity ####

Mode: Write only

When sent with any value, and if in "position" mode, the servo will
re-initialize the current control position and velocity to the sensed
values.  This also forces any pre-existing position integrative term
to zero.  It is expected that this will mostly be used when the
current applied torque is 0, either because of an in-place maximum
torque limit of zero, or because of an in-place kp and kd scale of
zero.

#### 0x140 - Driver Fault 1 ####

Mode: Read only

The exact bitfield reported by the motor driver in fault conditions
for fault register 1.  Up to 16 bits may be set.  This will only be
non-zero if the current mode is fault (1) and the fault code is 33
(motor driver fault).

#### 0x141 - Driver Fault 2 ####

Mode: Read only.

The exact bitfield reported by the motor driver in fault conditions
for fault register 2.  Up to 16 bits may be set.  This will only be
non-zero if the current mode is fault (1) and the fault code is 33
(motor driver fault).

#### 0x150 - 0x153 - UUID ####

Name: UUID
Mode: Read only, int32 only

This returns a 128 bit UUID, this is the value printed on mjbots
packaging and returned by `moteus_tool --info`

#### 0x154 - 0x157 - UUID Mask ####

Name: UUID
Mode: Write only, int32 only

If one or more of these fields are written, then the entire frame
after this point will be discarded unless the devices corresponding
UUID matches what was written.

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
 - `18` voltage is 12V
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
d pwm <phase> <magnitude> [<phase_rate>]
```

Where phase is in radians, magnitude is in volts, and the optional
phase rate is in radians per second.

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
documented above.  The options are largely the same as for `d pos`.
Unsupported options include:

 * `s` - stop position
 * `o` - fixed voltage override
 * `c` - fixed current override

### `d brake` ###

Enter the "brake" state.  In this mode, all motor phases are shorted
to ground, resulting in a passive "braking" action.

### `d nearest` ###

Update the current position to the closest one which is consistent
with a given output position.

```
d rezero <position>
```

### `d exact` ###

Update the current position to exactly the given value.

```
d exact <position>
```

### `d req-reindex` ###

Reset the homing state to relative based, requiring any homing
procedure to be re-run.

```
d req-reindex
```

### `d recapture` ###

When in position mode, reset the control position and velocity to the
currently sensed values.

```
d recapture
```

### `d cfg-set-output` ###

Modify the configuration as required so that the current observed
position will be equal to the given value.  Note, the configuration is
not written to persistent storage.

```
d cfg-set-output <position>
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

## B.2 `aux[12]` - Aux port manipulation ##

All commands are supported on both `aux1` and `aux2`.

### `aux1 out` - Set GPIO Output Values ###

```
aux1 out <data>
```

'data' is a single decimal integer.  Only bits associated with pins
configured as digital outputs are used, the remainder are ignored.

### `aux1 pwm` - Set PWM Output Values ###

```
aux1 pwm <pin> <value>
```

'pin' is a number from 0 to 4 that gives the pin to set.

'value' is a floating point value between 0.0 and 1.0 that gives the
output duty cycle for the pin.

### `aux1 ic-cmd` - Initiate an iC-PZ command ###

```
aux1 ic-cmd <HEXBYTE>
```

An example of entering analog mode calibration.

```
aux1 ic-cmd B0
```

### `aux1 ic-wr` - Write iC-PZ register ###

```
aux1 ic-wr <reg> <data>
```

Register is one byte in hex, data is 1 or more bytes in hex.

Example of switching to memory page 0.

```
aux1 ic-wr 40 00
```

### `aux1 ic-rd` - Read iC-PZ register ###

```
aux1 ic-rd <reg> <length>
```

The register is one byte in hex, length is a decimal value indicating
the number of bytes to read.

Example of reading the temperature data:

```
aux1 ic-rd 4e 2
```

### `aux1 ic-extra` - Select alternate periodic data ###

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

## B.3 `tel` - telemetry ##

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

## B.4 `conf` - configuration ##

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
`motor_position.rotor_to_output_ratio`.

## `servo.pid_dq` ##

These have the same semantics as the position mode PID controller, and
affect the current control loop.

## `servo.default_velocity_limit` / `servo.default_accel_limit` ##

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

## `servo.max_velocity_slip` ##

When finite, this enforces a limit on the difference between the
control velocity and the current measured velocity, measured i Hz.  It
can be used to ensure that acceleration limits are obeyed in velocity
mode when external torques exceed the maximum.  If used, typically
`servo.max_position_slip` must be relatively small to avoid
instability.

## `servo.max_voltage` ##

If the input voltage reaches this value, a fault is triggered and all
torque is stopped.

## `servo.max_power_W` ##

If set, set the allowable maximum power to the lower of this and the
factory board power profile.

## `servo.override_board_max_power` ##

If true, then `servo.max_power_W` is used as the power limit even if
it is larger than the factory board power profile.

## `servo.pwm_rate_hz` ##

The PWM rate to use, defaulting to 30000.  Allowable values are
between 15000 and 60000.  Lower values increase efficiency, but limit
peak power and reduce the maximum speed and control bandwidth.

## `servo.derate_temperature` ##

Torque begins to be limited when the temperature reaches this value.

## `servo.fault_temperature` ##

If the temperature reaches this value, a fault is triggered and all
torque is stopped.

## `servo.enable_motor_temperature` ##

If true, then the motor temperature will be sensed via the TEMP pads
on the board.

## `servo.motor_derate_temperature` ##

Torque begins to be limited when the motor temperature reaches this value.

## `servo.motor_fault_temperature` ##

If the motor temperature reaches this value, a fault is triggered and
all torque is stopped.

## `servo.flux_brake_margin_voltage` ##

Selects the flux braking point relative to the currently configured `servo.max_voltage`.  `flux braking point = max_voltage - flux_brake_margin_voltage`.

When the input voltage is above the braking point, the controller
causes the motor to act as a "virtual resistor" with resistance
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
* 10 - "decelerate to 0 velocity and hold position"
* 12 - "zero velocity"
* 15 - "brake"

For mode 10, `servo.default_velocity_limit` and
`servo.default_accel_limit` are used to control the deceleration
profile to zero speed.  The default PID gains are used.  The only
limit on torque when in this timeout mode is `servo.max_current_A`.

## `motor.thermistor_ohm` ##

The resistance of any attached motor NTC thermistor as measured at 25C
in ohms.

## `aux[12].pins.X.mode` ##

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
* 15 - Digital output (not implemented)
* 16 - Analog input
* 17 - PWM output

## `aux[12].pins.X.pull` ##

Configures optional pullup or pulldown on each pin.  Not all pullup
options will be used with every mode.  Additionally, the 2 aux2 pins
on moteus 4.5/8/11 have hard-installed 2k ohm pullups regardless of
these settings.

* 0 - no pullup or pulldown
* 1 - pull up
* 2 - pull down
* 3 - open drain (not implemented)

## `aux[12].i2c.i2c_hz` ##

The frequency to operate the I2C bus at.  Between 50000 and 400000.

## `aux[12].i2c.i2c_mode` ##

What I2C mode to use.

## `aux[12].i2c.devices.X.type` ##

What I2C device to expect.

* 0 - disabled
* 1 - AS5048
* 2 - AS5600

## `aux[12].i2c.devices.X.address` ##

The I2C address to use.

## `aux[12].i2c.devices.X.poll_ms` ##

How often in milliseconds to poll the device for more data.  Must no
less than 5.

## `aux[12].spi.mode` ##

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

## `aux[12].spi.rate_hz` ##

The frequency to operate the SPI bus at.  The default is 12000000.

## `aux[12].uart.mode` ##

The type of UART device.

* 0 - Disabled
* 1 - RLS AksIM-2
* 2 - Tunnel
* 3 - Per-control cycle debug information (undocumented)
* 4 - CUI AMT21x series RS422

When the tunnel mode is selected, data may be sent or received using
the CAN diagnostic protocol.  For aux1, use diagnostic channel 2.  For
aux2, use diagnostic channel 3.

## `aux[12].uart.baud_rate` ##

The baud rate to use for the UART.

## `aux[12].uart.poll_rate_us` ##

For encoder modes, the interval at which to poll the encoder for new
position information.

## `aux[12].uart.rs422` ##

Enable the RS422 transceiver.  This is only valid for 'aux1', and
requires that pin D and E (`aux1.pins.3` and `aux1.pins.4`) be
used for UART.

## `aux[12].uart.cui_amt21_address` ##

Select the CUI AMT21 address to communicate with.  The default is 0x54
(84 decimal), which is the default address CUI AMT21 encoders are
configured with.

## `aux[12].quadrature.enabled` ##

True/non-zero if quadrature input should be read from this port.

## `aux[12].quadrature.cpr` ##

The number of counts per revolution of the quadrature input.  If used
as a source, then this CPR must match the one configured in the
source.

## `aux[12].hall.enabled` ##

True/non-zero if hall effect sensors should be read from this port.

## `aux[12].hall.polarity` ##

A bitmask to XOR with the 3 hall phases.

## `aux[12].index.enabled` ##

True/non-zero if an index input be read from this port.

## `aux[12].sine_cosine.enabled` ##

True/non-zero if a sine/cosine input should be read from this port.

## `aux[12].sine_cosine.common` ##

The common mode voltage to use for the sine cosine.  The sampling is
done with 12 bits, so 2048 would be exactly 0.5 * 3.3V.  However, it
is best to calibrate this with actual readings as observed over the
diagnostic protocol for optimal performance.

## `aux[12].i2c_startup_delay_ms` ##

A delay in milliseconds after power-on (or upon reconfiguring), before
I2C devices associated with this auxiliary port are first used.

## `aux[12].pwm_period_us` ##

The period in microseconds to be used for PWM outputs on this
auxiliary port.

## `motor_position.sources.X.aux_number` ##

1 for an aux1 device, or 2 for an aux2 device.

## `motor_position.sources.X.type` ##

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

## `motor_position.sources.X.i2c_device` ##

If `type` was "7/I2C", this is a 0 based index specifying *which* I2C
device on that port should be used.

## `motor_position.sources.X.incremental_index` ##

If the specified auxiliary port has an incremental encoder, like a
quadrature encoder, this can be set to either 1 or 2 in order to use
an "index" pin to reference the data to a given position.  It will
allow the source to provide theta readings for commutation.

## `motor_position.sources.X.cpr` ##

The CPR of the given input.  In some cases this is automatically set,
but in most it will need to be manually entered.

## `motor_position.sources.X.offset/sign` ##

An integer offset and inversion to apply.  The resulting value is:
`(raw + offset) * sign / cpr`.

## `motor_position.sources.X.reference` ##

* 0 - this source is relative to the rotor
* 1 - this source is relative to the output

## `motor_position.sources.X.pll_filter_hz` ##

Selects the cutoff frequency of a low-pass filter used on this source.
It should typically be less than 10X the update rate of the input and
if used as the commutation or output sensor, should be higher than the
mechanical bandwidth of the plant.  Within that range, it can be tuned
for audible noise versus performance.

If set to 0, then no filter is applied.  In that case, sensors which
do not natively measure velocity will produce no velocity readings
(most of them).

## `motor_position.commutation_source` ##

A 0-based index into the source list that selects the source to use
for commutation.  This means it should have an accurate measure of the
relationship between the rotor and stator.

It is not recommended to use quadrature sources for commutation as
they can lose counts and require additional application level homing
support at each power-on.

## `motor_position.output.source` ##

A 0-based index into the source list that selects the source to use
for the output position.  The position and velocity from this source
will be used for control in "position" mode.

## `motor_position.output.offset/sign` ##

The offset is a floating point value measured in output revolutions.
Combined with the sign of -1/1, they can be used to position the 0
point of the output and control its direction of rotation.

## `motor_position.output.reference_source` ##

If non-negative, this is a 0-based index into the source list.  The
selected source is used at power on to disambiguate the output
position for multi-turn scenarios or when a reducer is configured.

## `motor_position.rotor_to_output_ratio` ##

The number of times the output turns for each revolution of the rotor.
For gear reducers (almost all configurations), this will be less than
one.  For example, a 4x gear reduction would be entered as 0.25.

## `motor_position.rotor_to_output_override` ##

If you *REALLY* know what you are doing, and want to configure a
non-reducing ratio, this can be set to true/non-zero.  Otherwise, a
ratio of greater than 1.0 will cause a fault.  This should only be
used if the system has a gearbox which is not a reducer, but speeds up
the output.


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

### Waiting for a trajectory to complete ###

A sequence of commands can be paused until a controller has finished
its trajectory using the `?` character.

```
d pos 0 0 nan a2 && ? && d pos 1 0 nan a2
```

Will move to position 0, wait until that motion is complete, then move
to position 1, all with an acceleration limit of 2.

Specific devices can be queried by following the question mark with an
ID number.

```
2>d pos 0 0 nan a2 && 1>d pos 10 0 0 nan a2 && ?2 && 2>d stop
```

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
configure `motor_position.rotor_to_output_ratio`.

## Setting the "zero offset" ##

The moteus controller can locate positions within one revolution after being power cycled, and will start with the reported position being between -0.5 and 0.5.  The physical zero position can be set using the following command:

```
python3 -m moteus.moteus_tool --target 1 --zero-offset
```

## Flashing and building firmware ##

To build the moteus firmware, an x86-64 Ubuntu 20.04, 22.04, or 24.04
system is required.

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

This may require the appropriate binutils to be installed, on Ubuntu
this can be accomplished with:

```
sudo apt install binutils-arm-none-eabi
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

## Power ##

The allowable maximum power for each moteus controller depends upon
the input voltage and PWM switching frequency.  The below table gives
the maximum allowable power at `servo.pwm_rate_hz=30000`.

| Name       | Peak power   |                | High input power |
|------------|--------------|----------------|------------------|
| moteus-r4  | <= 30V 900W  | linear derated | >= 38V 400W      |
| moteus-c1  | <= 28V 250W  | linear derated | >= 41V 150W      |
| moteus-n1  | <= 36V 2000W | linear derated | >= 44V 1000W     |

For other values of `servo.pwm_rate_hz`, the allowable maximum power
changes linearly with the PWM rate, so that at 15000, the maximum
power is half of that in the above table and at 60000 it is double
that.  Note however, that efficiency of the controller goes down
significantly at higher PWM rates.

The current power limit is reported in `servo_stats.max_power_W`.  The
controller will attempt to limit output phase current so as to stay
within this reported power limit in either direction, i.e. applying
power or regenerating energy.


## Pinout ##

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
connector and connecting it to the open data connector or by
purchasing a CAN terminating cable.

### JST ZH-6 SWD ###

Looking at the pins of the connector with the top of the board up the
pins are numbered 1 to 6 from left to right.

 - 1 - NC
 - 2 - NRST
 - 3 - SWDIO
 - 4 - GND
 - 5 - SWCLK
 - 6 - 3.3V

### moteus r4 - ABS - JST ZH-4 ###

Pin 1 is closest to the ABS label.  They are assigned as follows:

 - 1 - 3.3V
 - 2 - SCL
 - 3 - SDA
 - 4 - GND

### moteus n1/x1 - J3 - JST GH-6 ###

RS422, configured by AUX1 D/E to USART and enabling RS422 on AUX1.

 - 1 - 5V
 - 2 - A
 - 3 - B
 - 4 - Y
 - 5 - Z
 - 6 - GND

### moteus n1/x1 - AUX1 - JST GH-8 ###

 - 1 - 5V
 - 2 - 3.3V
 - 3 - A
 - 4 - B
 - 5 - C
 - 6 - D
 - 7 - E
 - 8 - GND

### moteus c1/n1/x1 - AUX2 - JST GH-7 ###

 - 1 - 5V
 - 2 - 3.3V
 - 3 - A
 - 4 - B
 - 5 - C
 - 6 - D
 - 7 - GND

### XT30PW-M ###

Looking at the pins of the power connector with the top of the board
up, the ground pin is to the left with the chamfered corner and the
positive supply is to the right with the square corner.

### moteus-r4 - Pico-SPOX 6 ENC ###

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

### 20MHz clock systems ###

Some rare MCP2517/8 adapters have a 20MHz clock rate.  The following
timings have been observed to work.  Note, that with 20MHz adapters,
BRS cannot be used (so for tview/moteus_tool, specify
`--can-disable-brs`).

```
ip link set can0 up type can \
  bitrate 1000000 dbitrate 5000000 \
  sjw 2 dsjw 2 \
  sample-point 0.666 dsample-point 0.666 \
  restart-ms 1000 fd on
```

### 40MHz clock systems ###

More commonly, chips such as the MCP2517/8 use a 40MHz system clock.
The following timings have been observed to work:

```
ip link set can0 up type can \
  tq 25 prop-seg 13 phase-seg1 12 phase-seg2 14 sjw 5 \
  dtq 25 dprop-seg 3 dphase-seg1 1 dphase-seg2 3 dsjw 3 \
  restart-ms 1000 fd on
```

### 80 MHz clock systems ###

Adapters such as the PEAK-CAN-FD use a 80MHz clock.  The following
timings have been observed to work:

```
ip link set can0 up type can \
  bitrate 1000000 dbitrate 5000000 \
  sjw 10 dsjw 5 \
  sample-point 0.666 dsample-point 0.666 \
  restart-ms 1000 fd on
```

# G. Application Limitations #

## Position ##

The commanded position is limited to +-32767.0 revolutions before any
`rotor_to_output_ratio` has been applied.  Reducers there will
further limit the range of the commandable revolutions.  For instance,
a 1/10 reducer configured as `rotor_to_output_ratio=0.1` results in
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
0.0001 revolutions per second before any `rotor_to_output_ratio`
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

When connectors are functional, moderate insertion force should be required and the connectors should not "wiggle" much after insertion.

The XT30 is not rated for any significant amount of mechanical force when mated.  For any application where cables may flex, strain relief should be used such that no force is applied to the connector.  The connector may be damaged if a system goes "out of control" even in one instance, although milder mechanical stress may cause accelerated fatigue and failure.

If sparks are observed, that is *definitely* a problem and the system should be powered off until the connectors can be replaced and mitigations made for what led to that event.

Genuine AMASS connectors are rated for 1000 insertions assuming no other mechanical damage, however off-brand connectors may have worse tolerances and may not make a reliable connection for even one insertion.

## Long daisy chains ##

When connecting more than 3 moteus controllers or servos in a system on a single power or CAN chain, additional factors should be considered.

1. It is recommended to use T-spliced power instead of using the daisy chained power connectors for chains greater than 3 units.  Small connector mis-seating or micro-arcing will cause magnified effects with long chains of controllers.

2. CAN bus electrical performance can be a limiting factor.  To operate at the default 5Mbps, you will need to ensure that CAN wires are twisted, all crimps are high quality, and that termination is installed.  Split termination may be required, where two 60 ohm resistors are connected in series with a small filter capacitor connected between the center tap of the resistors and ground.  A 4.7nF capacitor is recommended.  If that is insufficient, BRS can be disabled to operate only at 1Mbps.  With `moteus_tool` or `tview`, the `--can-disable-brs` flag can be used.

mjbots sells a CAN terminator that can be used for this purpose: https://mjbots.com/products/jst-ph3-can-fd-terminator

## Regenerative Braking Safety ##

moteus can be commanded to sharply decelerate loads, either directly in response to commands, or due to external disturbances.  When braking a load, moteus by default applies the generated power to the input DC bus.

If there is nowhere for this power to go, this can cause problems.  The voltage can increase without bound, which in mild cases will cause the CAN transceiver on all devices connected to the bus to fail, and in severe cases can explode the main FETs or other components on the board.

Here's what you should know about the facilities moteus has to deal with this, and what you can do to make your design safer.

### Flux braking ###

The feature within moteus itself to deal with this is "flux braking".  The flux braking implementation will dissipate extra power in the windings of the motor when the bus voltage gets above a certain threshold.  This is controlled by the `servo.flux_brake_margin_voltage` and `servo.flux_brake_resistance_ohm` parameters documented above.

### Design considerations for regenerative braking ###

The following design considerations can be used to minimize the risk of damage to hardware in the event of overvoltage.  These are not a substitute for validation in progressively more demanding situations, but they can help you start off in a good place.

- *Tightly scope the over-voltage fault / flux braking*: The configuration parameter `servo.max_voltage` can be lowered for all devices on the bus.  This will both cause a fault if the voltage exceeds this value and in conjuction with `servo.flux_brake_margin_voltage`, select the point at which moteus will attempt to dissipate energy to prevent an overvoltage scenario.  It is recommended to set this to no less than 5V above the maximum expected supply voltage.

- *Power from a battery, not a PSU*: When not charged, batteries are capable of sinking current to minimize over-voltage transients.  However, if the battery is fully charged, most battery management systems drastically reduce the allowable charging current.  Thus, a battery is only useful as a mitigation if it is never charged above say 75 or 80% state of charge.

- *Decrease overall system voltage*: If you run the moteus controller with say a 10S battery, the peak input voltage can be as high as 42V.  That does not leave very much margin for regenerative loads.  For applications that experience sharp regenerative loads and do not have a battery capable of charging always attached, it is recommended not to exceed 8S (33.6V peak).

- *Use a supply which can sink as well as source*: Powering from an inexpensive lab supply is the most dangerous, as they typically have no ability to sink current, only source it.  A "two quadrant" supply is the necessary device.

## PID Tuning ##

There are two configurable PI(D) controllers on moteus.  The "current loop" gains are set in `servo.pid_dq`, and are set during calibration based on the desired torque bandwidth.  They are not normally changed by the user.

The position control PID loop parameters are set in `servo.pid_position` and are selected by the user to achieve a desired control response.

Rough tuning rules are as follows:

1. Set kp to be small and kd, ki, and ilimit to be 0.  The kp at this stage should be low enough that any restorative torque is very gradual.
2. Increase kp by 50% at a time until the desired stiffness is reached.  A command like `d pos nan 0 nan` can be used to "hold position", then manual external disturbances can be used to judge the response.  If the control becomes unstable or vibrates, back off by 50%.
3. Set kd initially to 1/100th of the kp value.  Increase it by roughly 50% at a time until the desired damping is reached.  If instability or vibration results, back off by 50%.
4. If zero steady state error is required, then an integrative term is required.  If not, leave `ilimit` at 0 and you are done.
5. If configuring an integrative term, set `ilimit` to be 20% higher than the maximum expected steady state torque.  Then increase ki until the desired response is achieved.  `ki` will likely need to be numerically much larger than kp, on the order of 10-1000x larger.  A more representative control profile than just a "hold position" may be needed to determine if the selected integrative constants are sufficient.
