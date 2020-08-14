# Getting started with the moteus controller #

## Dependencies ##

Some dependencies are required for the graphical debugging UI.  Instructions for ubuntu 18.04 systems:

```
sudo apt install curl python3-matplotlib python3-pyside python3-qtconsole python3-serial
```

For ubuntu 20.04 systems:

```
sudo apt install curl libc6-python3-matplotlib python3-pyside2.qtgui python3-qtconsole python3-serial
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
./bazel-out/k8-fastbuild/bin/moteus/tool/tview --devices=1
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
