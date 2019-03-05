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

You will want to use `tview.py` to configure an RS485 id, and to
configure the motor resistance and volts/Hz.

## Calibrating the encoder ##

Using `tview.py` from the mjmech, repository, issue a `d cal 0.3`
command.  Copy the output to a text file, then execute:

```
./tools/bazel test //:host
./moteus/calibrate_encoder /path/to/cal.txt -o /tmp/path/to/result.txt
./bazel-bin/mjlib/micro/conf_write -d /dev/ttyUSB0 /tmp/path/to/result.txt -t N
```

Test the calibration by commanding a low current position in tview:

```
d pos 0 0 5
```

# Directory structure #

* hw/ - hardware (mechanical and electrical designs)
  * controller/ - PCB design for servo controller
  * 6008_leg/ - brackets and 3d printed parts for robot leg
  * busbar/ - power distribution PCB
  * imu_junction/ - IMU and power distribution for quadruped
* mjlib/ - software libraries common with firmware and host platform
* moteus/ - firmware for brushless controller
* tools/ - bazel build configure
