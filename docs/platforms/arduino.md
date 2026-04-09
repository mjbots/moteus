# Arduino

moteus provides a simplified version of the C++ library for use with Arduino compatible microcontroller systems.

## Installation

Open the Arduino library manager, search for "moteus" and install.  Then open one of the examples and modify it.  "WaitComplete" is a good one to get started with.

## Supported hardware ##

**Microcontroller**: Nearly any board compatible with the Arduino software is supported

**CAN-FD**: The library supports:

 * CAN-FD controllers and transceivers compatible with the [acan2517FD library](https://github.com/pierremolinaro/acan2517fd)
 * The onboard ACAN_T4 peripheral on Teensy 4.x boards
 * The STM32 FDCAN HAL peripheral

### ACAN2517FD

There are two pieces of configuration that are board and controller specific.

#### Pin Assignments

The `ACAN2517FD` constructor requires that it be passed the correct pins that are used for the SPI peripheral connected to the MCP2517FD controller.  For an external controller, you can use the pins you have physically wired to the controller.  For an integrated MCP2517FD you will need to look at your board documentation.

```cpp
#define MCP2517_CS  17
#define MCP2517_INT 7
// For the Longan CANBed FD, the "SPI" peripheral determines which
// pins are used.
ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);
```

#### CAN-FD Timing

You need to match the CAN-FD base clock rate that your adapter uses.  Different adapters have different base clock rates.

**Longan Labs CANBed FD**: This board uses a 20MHz clock

**Mikro MCP2517FD Click**: This board defaults to a 40MHz clock, but can be configured by jumper.

Regardless, you need to pass the correct clock rate using the `ACAN2517FDSettings` object.

```cpp
ACAN2517FDSettings settings(
    ACAN2517FDSettings::OSC_20MHz,
    1000ll * 1000ll,
    DataBitRateFactor::x1);

settings.mArbitrationSJW = 2;
settings.mDriverTransmitFIFOSize = 1;
settings.mDriverReceiveFIFOSize = 2;

const uint32_t errorCode = can.begin(settings, [] { can.isr(); });
```

### Teensy 4.x - ACAN_T4

Configuring the Teensy 4.x library merely requires selecting which CAN-FD peripheral to use.

```
MoteusTeensyCanFd canBus(ACAN_T4::can3)
```

Where one of `can1`, `can2` or `can3` are possible.

### STM32 FDCAN HAL

Integration for the STM32 FDCAN HAL periphal is currently undocumented, but some source examples exist.

## Caveats

There are some caveats when controlling moteus from a microcontroller.

### Operating system connected CAN-FD adapter still required

While you can command and control moteus from the Arduino library, there is currently no mechanism to calibrate a new motor with a controller.  Thus you *must* have a CAN-FD adapter that is able to be connected to a computer with an operating system in order to run the [calibration process](../guides/calibration.md). It is also much easier to use such an adapter with tview to set the configuration and tune the PID gains.

### Teensy 4.x and STM32 FDCAN HAL require an external CAN-FD transceiver

The CAN-FD controller built into the Teensy 4.x and various STM32 peripherals is a logic level peripheral.  To operate on a CAN-FD bus, a transceiver is required in order to convert to line voltages.  Many off the shelf options are available, although beware that many require both 5V and 3V supplies.

### Termination

Many external MCP2517FD controller/transceivers do not have termination resistors.  To operate correctly, a CAN-FD bus is intended to have 2 120ohm resistors between CANL and CANH, one at each end of the bus.  Often short busses will work acceptly with only 1 termination resistor, but never with 0.

### Flash and storage limitations

With some smaller Arduino platforms, notably the Arduino Uno or Longan Labs CANBed FD, the combination of the ACAN2517FD library and the moteus library can consume a significant fraction of the flash and RAM available. For simple applications this isn’t a problem, but if you want to execute something more complex, you may be better served using a more capable Arduino compatible processor, like a [Teensy 4](https://www.pjrc.com/store/teensy41.html) or a [Nano Sense 33](https://store-usa.arduino.cc/products/nano-33-ble-sense-rev2).

## Overview video

A video overview of the process of using an Arduino with moteus can be found here:

{{ youtube("dbV0jIN8Ay4") }}
