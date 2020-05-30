mjbots power dist reference

# A. Operation #

This documentation refers to the most recent (r3.1) version of the
board.

The power dist board provides pre-charging, a soft power switch, and a
power connector fanout for use in robotic applications.  The input can
be directly connected to a battery or power supply.  The output can be
connected to high capacitance loads.  When the switch is enabled, the
board will first apply power through a limiting resistor to "charge
up" the output capacitance for 100ms before connecting the low
impedance source.

Additionally, a 125 kbps CAN port allows robot software to monitor the
state of the switch and request a "soft power down".

## Specifications ##

* Input voltage: 8-34V
* Peak current: 90A
* Continuous current: 45A
* Quiescent current (off): 3mA
* Dimensions: 45x70mm
* Mass: 28.3g
* Connectors:
  * Input: Optional Amass XT90
  * Output: 6x Amass XT30
  * CAN: 2x JST-PH3
  * Switch and LED: JST-PH4
  * STM32 SWD: JST-ZH6

## Mounting ##

4x M2.5 mounting holes are provided in a 64mm x 39mm rectangular
pattern.

# B. CAN protocol #

The power dist board emits the following CAN frame at 10Hz.

* CAN ID: 0x10004
* byte 0: Switch status (0 is off, 1 is on)
* byte 1: Lock time in 0.1s intervals

It listens for a frame of the following properties:

* CAN ID: 0x10005
* byte 0: Reserved, keep at 0
* byte 1: Lock time in 0.1s intervals

While the lock time is non-zero, the power switch will not turn off
the output load until the count does reach zero.  An application which
desires soft power down can periodically "kick" the lock interval to
keep power applied, then stop doing so when it is ready for the power
to turn off.

# C. pinout #

## XT-90 Input ##

The XT-90 connector has a `-` and a `+` imprint on the housing.

## XT-30 Output ##

The XT-30 connectors have a `-` and a `+` imprint on the housing.

## JST PH-4 Switch ##

Looking at the pins of the connector from the top with the mjbots logo
right side up the pins are numbered from right to left.

 - 1 - LED+ - LED positive
 - 2 - LED- - LED ground
 - 3 - SWP - Switch positive
 - 4 - SWG - Switch ground

## JST PH-3 CAN ##

Looking at the pins of the connector with the mjbots logo right side
up, the pins are numbered from the bottom to the top.

 - 1 - CAN_H
 - 2 - CAN_L
 - 3 - GND

NOTE: This ground IS NOT the same as the output ground.  DO NOT
connect ground from the CAN connector to an output load.  `CAN_H` and
`CAN_L` are fine to connect, just not ground.

## JST ZH-6 SWD ##

Looking at the pins of the connector with the mjbots logo right side
up the pins are numbered 1 to 6 from top to bottom.

 - 1 - NC
 - 2 - NRST
 - 3 - SWDIO
 - 4 - GND
 - 5 - SWCLK
 - 6 - 3.3V
