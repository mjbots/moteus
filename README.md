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
* docs/ - documentation

# Documentation #

* [Getting Started](docs/getting_started.md)
* [Reference](docs/reference.md)
