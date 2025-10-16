# CAN-FD

The python and C++ libraries are not required to command and monitor moteus, they just make it easier.  Any system capable of generating and receiving CAN-FD frames with the appropriate timing can be used.  Here are some hints to make that easier.

## Bit timing and basic communication

Moteus requires 1Mbps/5Mbps CAN-FD timing, a sample point of 0.666, and works best if the SJW and DSJW settings are as large as possible.

If communication is not working, it is recommended to attach a mjcanfd-usb-1x to the bus at the same time as your host and use it to capture frames.  Any serial application (or `cat` on Linux) can be used to read the output.  Since they start with the same timings as moteus, you can use that to identify if your frames are being received and being responded to.

## Decoding frames observed on the bus

There exists a tool in the moteus repository to decode CAN-FD frames sent to or from moteus that can be invaluable in diagnosing what you are sending and why moteus is or is not responding to it.  First, capture a hex formatted dump of the CAN-FD frame in question.  Then:

```
moteus$ ./utils/decode_can_frame.py 1100110F
11 - READ_REGISTERS - INT8 1 registers
  00 - Starting at reg 0x000(MODE)
11 - READ_REGISTERS - INT8 1 registers
  0f - Starting at reg 0x00f(FAULT)
```

and the corresponding response:

```
moteus$ ./utils/decode_can_frame.py 210000210F00
21 - REPLY - 0 1 registers
  00 - Starting at reg 0x000(MODE)
   00 - Reg 0x000(MODE) = 0(STOPPED)
21 - REPLY - 0 1 registers
  0f - Starting at reg 0x00f(FAULT)
   00 - Reg 0x00f(FAULT) = 0
```


## Generating appropriate command frames

The [CAN-FD reference](../protocol/can.md) and [register reference](../protocol/registers.md) contains the information necessary to generate and parse frames.  However, it can be a lot to digest.  You can use the [python library](../integration/python.md) to quickly get example frames that accomplish a given task.  First, create a python script that accomplishes what you want.  Then, in an interactive python REPL:

```
>>> print(c.make_position(position=1, velocity=2, query=True).data.hex())
01000a0e200000803f0000004011001f01130d
```

Where the arguments to `make_position` correspond to the action you want to accomplish.  You can verify the contents of this frame using `decode_can_frame.py`:

```
moteus$ ./utils/decode_can_frame.py 01000a0e200000803f0000004011001f01130d
01 - WRITE_REGISTERS - 0 1 registers
  00 - Starting at reg 0x000(MODE)
   0a - Reg 0x000(MODE) = 10(POSITION)
0e - WRITE_REGISTERS - 3 2 registers
  20 - Starting at reg 0x020(COMMAND_POSITION)
   0000803f - Reg 0x020(COMMAND_POSITION) = 1.0
   00000040 - Reg 0x021(COMMAND_VELOCITY) = 2.0
11 - READ_REGISTERS - INT8 1 registers
  00 - Starting at reg 0x000(MODE)
1f - READ_REGISTERS - F32 3 registers
  01 - Starting at reg 0x001(POSITION)
13 - READ_REGISTERS - INT8 3 registers
  0d - Starting at reg 0x00d(VOLTAGE)
```
