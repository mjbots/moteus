# Moteus Controller Register Map #

The moteus controller exposes a register based interface for concise
RS485 communications.  Each register may be optionally accessed as
multiple types, depending upon how much precision is necessary.  Not
all registers support all types.

Writing to registers that affects the current control state will only
take effect if the 0x000 register is written within the same frame.
The 0x000 register should be listed first in the set of registers
written, as when it changes value, all other commands are reset to
their default.

## Integral Mappings ##

For registers which are accessible in integral types, unless otherwise
stated, the following mappings are used.

### Current (measured in Amps) ###

int8 - 1 LSB -> 1A
int16 - 1 LSB -> 0.1A
int32 - 1 LSB -> 0.001A

### Torque (measured in N*m) ###
int8 - 1 LSB -> 0.5 N*m
int16 - 1 LSB -> 0.01 N*m
int32 - 1 LSB -> 0.001 N*m

### Voltage (measured in Volts) ###

int8 - 1 LSB -> 0.5 V
int16 - 1 LSB -> 0.1 V
int32 - 1 LSB -> 0.001 V

### Temperature (measured in degrees Celsius) ###

int8 - 1 LSB -> 1 C
int16 - 1 LSB -> 0.1 C
int32 - 1 LSB -> 0.001 C

### PWM ###

int8 - 1 LSB = 1.0 / 127.0 ~= 0.00787
int16 - 1 LSB = 1.0 / 32767.0 ~= 3.052e-5
int32 - 1 LSB = 1.0 / 2147483647.0 ~= 4.657e-10

### Force (measured in Newtons) ###

int8 - 1 LSB -> 1 N
int16 - 1 LSB -> 0.1 N
int32 - 1 LSB -> 0.001 N

### Distance (measured in m) ###

int8 - 1 LSB -> 0.01m
int16 - 1 LSB -> 0.001m
int32 - 1 LSB -> 0.001m

### Velocity (measured in m/s) ###

int8 - 1 LSB -> 0.01m/s
int16 - 1 LSB -> 0.001m/s
int32 - 1 LSB -> 0.0001m/s

### Time (measured in s) ###

int8 - 1 LSB -> 0.01s
int16 - 1 LSB -> 0.001s
int32 - 1 LSB -> 0.000001s

## Definitions ##

### Modes ###

The following modes are possible:

 * *Read only* - The register may only be read.
 * *Read/write* - The register may be read or written.  Writes are
   never persistent across power cycles.
 * *Configurable* - The register may be read, or written.  The state
   may be saved to persistent configuration (but this requires an
   additional operation).



## Registers ##

### 0x000 - Mode ###

Types: int8, int16, int32, float
Mode: Read/write

The current operational mode of the servo.  Not all values are valid to write.

 * 0 - stopped = writeable, clears faults
 * 1 - fault
 * 2,3,4 - preparing to operate
 * 5 - PWM mode = writeable
 * 6 - Voltage mode = writeable
 * 7 - Voltage FOC = writeable
 * 8 - Voltage DQ = writeable
 * 9 - Current = writeable
 * 10 - Position = writeable
 * 11 - Position timeout
 * 12 - Zero Velocity = writeable

The registers associated with each control mode are reset when not in
that control mode.

### 0x001 - Position ###

Types: int8, int16, int32, float
Mode: Read only

The current position of the servo, measured in rotations of the end
effector.  The maximal negative integer is reserved and will not be
reported.  Integral mapping:

int8 - 1 LSB - 0.01 rotation/3.6 degrees (range of -1.28 to 1.27)
int16 - 1 LSB - 0.001 rotation/0.36 degrees (range of -32.768 to 32.767)
int32 - 1 LSB - 0.00001 rotation/0.0036 degrees

### 0x002 - Velocity ###

Types: int8, int16, int32, float
Mode: Read only

The current velocity of the servo, as measured in Hz at the end
effector.  Integral mapping:

int8 - 1 LSB - 0.1 Hz / 36 dps
int16 - 1 LSB - 0.001 Hz / 0.36 dps
int32 - 1 LSB - 0.00001 Hz / 0.0036 dps

### 0x003 - Torque ###

Types: int8, int16, int32, float
Mode: Read only

The current applied torque as measured after any gearbox.

### 0x004 - Measured Q phase current ###

Types: int8, int16, int32, float
Mode: Read only

The current in the Q phase measured in amps.

### 0x005 - Measured D phase current ###

Types: int8, int16, int32, float
Mode: Read only

The current in the D phase measured in amps.

### 0x00c - Rezero State ###

Types: int8, int16, int32, float
Mode: Read only

Non-zero if this servo has been rezeroed since the most recent power
on or reset.

### 0x00d - Voltage ###

Types: int8, int16, int32, float
Mode: Read only

The current input voltage.

### 0x00e - Temperature ###

Types: int8, int16, int32, float
Mode: Read only

The current board temperature, measured in degrees celsius.

### 0x00f - Fault code ###

Type: int8, int16, int32, float
Mode: Read only

An integer fault code which will be set if the primary mode is 1 (Fault).


### 0x010 / 0x011 / 0x012 - PWM phase A / B / C ###

Type: int8, int16, int32, float
Mode: Read/write

When in kPwm mode, this controls the raw PWM value for phase A, B, and C.

### 0x014 / 0x15 / 0x16 - Voltage phase A / B / C ###

Type: int8, int16, int32, float
Mode: Read/write

When in kVoltage mode, this controls the voltage applied to phase A,
B, and C.


### 0x018 - Voltage FOC Theta ###

Type: int8, int16, int32, float
Mode: Read/write

When in kVoltageFoc mode, this controls the desired electrical phase.
Integral types use the PWM mapping.

### 0x019 - Voltage FOC Voltage ###

Type: int8, int16, int32, float
Mode: Read/write

When in kVoltageFoc mode, this controls the desired applied phase voltage.

### 0x01a - D Voltage ###

Type: int8, int16, int32, float
Mode: Read/write

When in kVoltageDq mode, this controls the desired applied D voltage.

### 0x01b - Q Voltage ###

Type: int8, int16, int32, float
Mode: Read/write

When in kVoltageDq mode, this controls the desired applied Q voltage.

### 0x01c - Commanded Q Phase Current ###

Type: int8, int16, int32, float
Mode: Read/write

When in kFoc mode, this controls the desired Q phase current.

### 0x01d - Commanded D Phase Current ###

Type: int8, int16, int32, float
Mode: Read/write

When in kFoc mode, this controls the desired D phase current.  Unless
you like burning power, with a BLDC motor you will typically want this
set to 0.


### 0x020 - Position Command ###

Type: int8, int16, int32, float
Mode: Read/write

When in kPosition mode, this controls the desired position.  The same
integral mapping is used as for 0x001 Position.  The maximal negative
integer, or NaN for float represents "use the current position value".

### 0x021 - Commanded Velocity ###

Type: int8, int16, int32, float
Mode: Read/write

When in kPosition mode, advance the commanded position at the given
velocity in Hz.

As a special case, if the 0x020 position is unset, and 0x023 stop
position is set, the sign of this is ignored and is instead selected
so that the motor will move towards the stop position.

### 0x022 - Feedforward torque ###

Type: int8, int16, int32, float
Mode: Read/write

When in kPosition mode, add the given feedforward torque after
applying all regular control loops.  Note, this is the torque after
any gearboxes.

### 0x023 - Commanded kp scale ###

Type: int8, int16, int32, float
Mode: Read/write

When in kPosition mode, shrink the proportional control term by the
given factor.  integral types are applied as for PWM.

### 0x024 - Commanded kd scale ###

Type: int8, int16, int32, float
Mode: Read/write

When in kPosition mode, shrink the derivative control term by the
given factor.  integral types are applied as for PWM.  This is
internally limited to be no more than the kp scale in effect.


### 0x025 - Position maximum torque ###

Type: int8, int16, int32, float
Mode: Read/write

When in kPosition mode, the maximum torque to be applied.  Defaults to
maximum torque.  Note, this is the torque after any gearboxes.

### 0x026 - Commanded stop position ###

Type: int8, int16, int32, float
Mode: Read/write

When in kPosition mode, and a non-zero velocity is commanded, stop
motion when reaching the given position.  NaN / maximal negative mean
no limit is applied.

### 0x027 - Watchdog timeout ###

Type: int8, int16, int32, float
Mode: Read/write

This determines the length of time for which this command is valid.
If this timeout expires before another command is received, the
controller will enter the kPositionTimeout state.  The default is 0.0,
which means to use the configured default.  NaN / maximal negative
means apply no timeout is enforced.

### 0x100 - Model Number ###

Name: Model Number
Types: int32_t
Mode: Read only

This returns a 32 bit model number.

### 0x101 - Firmware Version ###

Types: int32
Mode: Read only

This returns a 32 bit firmware version, encoded bytewise as
major.minor.micro.  i.e. 0x010304 is version 1.3.4


### 0x102 - Register map version ###

Types: int32
Mode: Read only

This returns a number that indicates how to interpret all registers.


### 0x110 - Multiplex ID ###

Name: Multiplex ID
Types: int8, int16, int32
Mode: Configurable

This controls the primary ID used to access the device over the
multiplex RS485 bus.  It can only be between 1 and 127.  (0 is
reserved as the broadcast address).

### 0x120 - 0x122 - Serial Number ###

Name: Serial Number
Types: int32_t
Mode: Read only

This returns a 96 bit serial number, least significant word first.
