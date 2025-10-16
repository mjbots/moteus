# CAN Protocol

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

## CAN Format

Communication with moteus is conducted via CAN-FD frames with a 1Mbit
standard bitrate and a 5Mbit data bitrate.

### CAN ID

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

### Subframes

Each CAN-FD frame contains one or more "subframes".  A short
description of the allowable subframe types are described below.  The
canonical reference is located at
[multiplex/format.h](https://github.com/mjbots/mjlib/blob/master/mjlib/multiplex/format.h)

Any extra trailing padding bytes required in the CAN-FD frame should
be set to NOP (0x50).

#### Write Registers

*0x00, 0x04, 0x08, 0x0c* - write (int8|int16|int32|float)

- `varuint` => number of registers (may be optionally encoded as a
  non-zero 2 LSBS of the subframe type)
- `varuint` => start register number
- N x (int8|int16|int32|float) => values

#### Read Registers

*0x10, 0x14, 0x18, 0x1c* - read (int8|int16|int32|float)

- `varuint` => number of registers (may be optionally encoded as a non-zero 2 LSBs)
- `varuint` => start register number

#### Reply

*0x20, 0x24, 0x28, 0x2c* - reply (int8|int16|int32|float)

- `varuint` => number of registers (may be optionally encoded as a non-zero 2 LSBs)
- `varuint` => start register number
- N x (int8|int16|int32|float) => values

#### Errors

*0x30, 0x31* - write/read error

- `varuint` => register number
- `varuint` => error number

#### NOP

*0x50* - no operation


## Example

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
