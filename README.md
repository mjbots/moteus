# moteus brushless servo #

This contains full designs for the moteus brushless servo actuator,
including firmware and PCBs, along with additional mechanical and
electrical designs allowing one to combine the actuators into a
quadraped robot.

**WARNING**: This is not just a software project.  It includes designs
for moderately high power electronics.  It has not yet burned down my
(or anyone's that I know of) house, but there are no guarantees.

# LICENSE #

All files contained in this repository, unless otherwise noted, are
available under an Apache 2.0 License:
https://www.apache.org/licenses/LICENSE-2.0

# Directory structure #

* hw/ - hardware (mechanical and electrical designs)
  * controller/ - PCB design for servo controller
  * 6008_leg/ - brackets and 3d printed parts for robot leg
  * busbar/ - power distribution PCB
  * imu_junction/ - IMU and power distribution for quadruped
* mjlib/ - software libraries common with firmware and host platform
* moteus/ - firmware for brushless controller
* tools/ - bazel build configure

# Setup of an individual actuator #

## Building and flashing the moteus firmware ##

The flash target works out of the box with a stm32f3discovery based
programmer board.

```
tools/bazel test -c opt --cpu stm32f4 //moteus:flash
```

or, to flash an already built binary

```
moteus/flash.sh
```

## Initial configuration ##

First, build the configuration tool:

```
tools/bazel test //moteus:moteus_tool
```

Next, calibrate the encoder.  IMPORTANT: The servo must be able to
spin freely to complete this calibration.

```
./bazel-bin/moteus/moteus_tool --target 1 --calibrate -v
```

Test the calibration by commanding a low current position in tview:

```
d pos 0 0 5
```
