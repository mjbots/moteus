# moteus brushless servo #

This contains full designs for the moteus brushless servo actuator,
including firmware and PCBs.

**WARNING**: This is not just a software project.  It includes designs
for moderately high power electronics.  It has not yet burned down my
(or anyone's that I know of) house, but there are no guarantees.


# Specifications #

| Name                  | Value          |
|-----------------------|----------------|
| Voltage Input         | 10-44V         |
| Peak Electrical Power | 500W           |
| Mass                  | 14.2g          |
| Control Rate          | 15-40kHz       |
| PWM Switching Rate    | 15-60kHz       |
| CPU                   | 170Mhz STM32G4 |
| Peak phase current    | 100A           |
| Communications        | 5Mbps CAN-FD   |
| Dimensions            | 46x53mm        |

Assembled and tested boards can be purchased at https://mjbots.com/products/moteus-r4-11


# Directory structure #

* hw/ - hardware (mechanical and electrical designs)
  * controller/ - PCB design for servo controller
* fw/ - firmware for brushless controller
* lib/ - client side software
* utils/ - diagnostic tools
* tools/ - bazel build configure
* docs/ - documentation

# Documentation #

* [Getting Started](docs/getting_started.md)
* [Reference](docs/reference.md)

# Misc #

 * travis-ci [![Build Status](https://travis-ci.org/mjbots/moteus.svg?branch=main)](https://travis-ci.org/mjbots/moteus)

# How to support moteus development #

The easiest way to support development the moteus hardware and firmware is as follows:

1) Buy things from https://mjbots.com
2) Build awesome machines!

That's it!  If for some reason you want to go above and beyond, you can sponsor mjbots through github: https://github.com/sponsors/mjbots

# License #

All files contained in this repository, unless otherwise noted, are
available under an Apache 2.0 License:
https://www.apache.org/licenses/LICENSE-2.0

# Trademark #

mjbots Robotic Systems LLC owns and protects the "mjbots" and "moteus" trademarks in many jurisdictions.

If you want to use these names in your project or product, please read the [Trademark Policy](https://mjbots.com/trademark-policy)
