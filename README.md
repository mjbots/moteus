# moteus brushless servo #

This contains full designs for the moteus brushless servo actuator,
including firmware and PCBs, along with additional mechanical and
electrical designs allowing one to combine the actuators into a
quadruped robot.

**WARNING**: This is not just a software project.  It includes designs
for moderately high power electronics.  It has not yet burned down my
(or anyone's that I know of) house, but there are no guarantees.

 * travis-ci [![Build Status](https://travis-ci.org/mjbots/moteus.svg?branch=master)](https://travis-ci.org/mjbots/moteus)

# LICENSE #

All files contained in this repository, unless otherwise noted, are
available under an Apache 2.0 License:
https://www.apache.org/licenses/LICENSE-2.0

# Directory structure #

* hw/ - hardware (mechanical and electrical designs)
  * controller/ - PCB design for servo controller
  * gearbox_v2/ - integrated servo
  * full_rotation_leg/ - brackets and 3d printed parts for robot leg
  * power_dist/ - power distribution for quadruped
  * chassis_v2/ - quadruped chassis
* moteus/ - firmware for brushless controller
* moteus/tool/ - diagnostic tools
* tools/ - bazel build configure

# Getting started #

## Dependencies ##

At the moment, a custom openocd is required to flash images onto STM32G4 based controllers.  Clone and install from mjbots/openocde

```
git clone https://github.com/mjbots/openocd
cd openocd.git
./configure && make && sudo make install
```

Other dependencies which are required for the graphical debugging UI.  Instructions for ubuntu >=18.04 systems:

```
sudo apt install python3-serial python3-pyside
```

## Building the software ##

This will build the host and target binaries for the latest version of
the moteus controller.

```
tools/bazel test -c opt --cpu=stm32g4 //:target
tools/bazel test //:host
```

## fdcanusb ##

You can make life more convenient by installing the fdcanusb udev rule
so that it appears as `/dev/fdcanusb`.  See:
https://github.com/mjbots/fdcanusb/blob/master/70-fdcanusb.rules

## Running tview ##

tview lets you configure and inspect the state of the controller.  It
can be run like:

```
./bazel-out/k8-fastbuild/bin/moteus/tool/tview --devices=1
```

By default, it uses `/dev/fdcanusb` to communicate with targets.

## Initial configuration ##

Assuming your controller has firmware installed already, you can
calibrate the controller using the following procedure.

1. Set `motor.unwrapped_position_scale` using tview.  This is the
   gearbox ratio.  A direct drive system with no gearbox should be
   1.0.

2. Run the calibration tool:

```
./bazel-bin/moteus/moteus_tool --target 1 --calibrate
```

WARNING: Any attached motor must be able to spin freely.  It will be spun in both directions and at high speed.

## Flashing firwmare ##

The firmware can be flashed either using:

```
./moteus/flash.sh
```

Or using a bazel rule, which first ensures that all firmware sources
are built correctly.

```
tools/bazel test //moteus:flash
```

# Reference #

## Mechanical ##

The current mechanical drawing can be found at: [20200115-moteus-controller-r42-mechanical.pdf](https://github.com/mjbots/moteus/blob/master/hw/controller/r4.2/20200115-moteus-controller-r42-mechanical.pdf)

## Pinout ##

### JST PH-3 CAN ###

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

### JST ZH-6 SWD ###

Looking at the pins of the connector with the top of the board up the
pins are numbered 1 to 6 from left to right.

 * 1 - NC
 * 2 - NRST
 * 3 - SWDIO
 * 4 - GND
 * 5 - SWCLK
 * 6 - 3.3V

### JST ZH-4 I2C ###

Looking at the pins of the connector with the bottom of the board up
the pins are numbered 1 to 4 from left to right.

 * 1 - 3.3V
 * 2 - SCL
 * 3 - SDA
 * 4 - GND

### XT30PW-M ###

Looking at the pins of the power connector with the top of the board
up, the ground pin is to the left with the chamfered corner and the
positive supply is to the right with the square corner.
