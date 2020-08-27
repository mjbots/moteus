# Getting started with the moteus controller #

## Dependencies ##

Some dependencies are required for the graphical debugging UI.  Instructions for ubuntu 18.04 systems:

```
sudo apt install curl python3-matplotlib python3-pyside python3-qtconsole python3-serial python3-snappy python
```

For ubuntu 20.04 systems:

```
sudo apt install libc6-dev libtinfo5 gcc curl libc6-dev python3-matplotlib python3-pyside2.qtgui python3-pyside2.qtwidgets python3-pyside2.qtsvg python3-pyside2.qtprintsupport python3-pyside2.qtuitools python3-qtconsole python3-serial python3-snappy python-is-python3
```

## Building the software ##

This will build the host and target binaries for the latest version of
the moteus controller.

```
tools/bazel test -c opt --cpu=stm32g4 //:target
tools/bazel test --cpu=k8 //:host
```

## fdcanusb ##

You can make life more convenient by installing the fdcanusb udev rule
so that it appears as `/dev/fdcanusb`.  See:
https://github.com/mjbots/fdcanusb/blob/master/70-fdcanusb.rules

## Running tview ##

tview lets you configure and inspect the state of the controller.  It
can be run like:

```
./bazel-out/k8-opt/bin/moteus/tool/tview --devices=1
```

By default, it uses `/dev/fdcanusb` to communicate with targets.

tview has three panes, left, right, and bottom.  Further, the left
pane has two tabs.

Left pane, right tab (the default) shows a hierarchical tree of all
telemetry items.

Left pane, left tab shows a hierarchical tree of all configurable
parameters.  Parameter values can be updated by double clicking on
their value and entering a new one.

The right pane shows real time plots of telemetry items.  It can be
populated with plots by right clicking on telemetry items in the
telemetry tab.

The bottom pane has a command line console which shows the commands
sent internally by tview and their responses, and provides an
interactive console to interact with the device using the diagnostic
protocol.

# Initial Configuration #

There are a few parameters you will likely want to configure early on
in your setup:

* `servopos.position_min` and `servopos.position_max` these define the bounds of motion which the controller will allow when in position control mode.  Attempting to start beyond this region will fault, and if outside the region in operation, no torque will be applied to go further outside.
* `servo.max_current_A` the maximum phase current to apply to the motor.  This can be used to limit the maximum torque that the system is capable of regardless of any command sent.
* `servo.pid_position` the PID parameters for the position control loop.
* `motor.unwrapped_position_scale` any gearbox scaling, a reducing gearbox should be configured with a number smaller than one, e.g. 0.25 for a 4x reduction gearbox.  This affects reported position, speed, and torques.
* `id.id` the CAN-FD id used by this device

A larger set of parameters is documented in the reference manual.

# Upgrading the Firmware #

## Flashing over CAN ##

```
./moteus_tool -t 1 --flash path/to/file.elf
```

## Flashing from the debug port ##

If you want to re-flash the firmware using the debugging port, a custom openocd is currently required.  Clone and install from mjbots/openocde

```
sudo apt install autotools-dev automake autogen autoconf libtool
git clone https://github.com/mjbots/openocd
cd openocd.git
./bootstrap
./configure && make && sudo make install
```

Then you can either flash using bazel:

```
tools/bazel test -c opt --cpu=stm32g4 //moteus:flash
```

Or re-flash the same file using:

```
./moteus/flash.sh
```
