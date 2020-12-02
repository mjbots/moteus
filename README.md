# moteus brushless servo #

This contains full designs for the moteus brushless servo actuator,
including firmware and PCBs.

**WARNING**: This is not just a software project.  It includes designs
for moderately high power electronics.  It has not yet burned down my
(or anyone's that I know of) house, but there are no guarantees.

 * travis-ci [![Build Status](https://travis-ci.org/mjbots/moteus.svg?branch=main)](https://travis-ci.org/mjbots/moteus)

# LICENSE #

All files contained in this repository, unless otherwise noted, are
available under an Apache 2.0 License:
https://www.apache.org/licenses/LICENSE-2.0

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
