# Client Tools

## tview usage

tview can monitor and control 1 or more devices simultaneously.  It can be started with:

```
python3 -m moteus_gui.tview --target 1[,2,3]...
```

When running, the configuration for each can be modified in the left hand tab, and live telemetry values can be displayed or plotted from the right hand tab.  diagnostic mode commands may be issued in the bottom terminal window.

A simple command and scripting language beyond the diagnostic protocol is available within the tview terminal window.

### Communicating with a specific device

If more than one device is available, commands can be sent to a specific device by prefixing the command with `ID>`.  For instance, to send a stop command to ID #2, you can do:

```
2>d stop
```

The 'A' character can be used to send to all devices simultaneously.

```
A>d stop
```

### Sending multiple commands at once

The `&&` token may be used to separate individual commands, which will be issued back to back.  For instance:

```
1>d stop && 2>d stop
```

Will send the command to both devices.

### Delays

A delay may be inserted into a sequence of commands by entering an integer number of milliseconds prefixed by the colon (':') character.  For instance:

```
d pos nan 0.5 1 s0.5 && :1000 && d stop
```

Will command a position mode, wait 1s, then command a stop.

### Waiting for a trajectory to complete

A sequence of commands can be paused until a controller has finished
its trajectory using the `?` character.

```
d pos 0 0 nan a2 && ? && d pos 1 0 nan a2
```

Will move to position 0, wait until that motion is complete, then move
to position 1, all with an acceleration limit of 2.

Specific devices can be queried by following the question mark with an
ID number.

```
2>d pos 0 0 nan a2 && 1>d pos 10 0 0 nan a2 && ?2 && 2>d stop
```

## Calibration

Assuming your controller has firmware installed already, you can
calibrate the controller using the following procedure.

```
python3 -m moteus.moteus_tool --target 1 --calibrate
```

WARNING: Any attached motor must be able to spin freely.  It will be
spun in both directions and at high speed.

After calibrating for an entirely new type of motor, you may need to
adjust PID gains before the motor will perform acceptably, and/or
configure `motor_position.rotor_to_output_ratio`.

## Setting the "zero offset"

The moteus controller can locate positions within one revolution after being power cycled, and will start with the reported position being between -0.5 and 0.5.  The physical zero position can be set using the following command:

```
python3 -m moteus.moteus_tool --target 1 --zero-offset
```

## Configuring the CAN-FD transport

`moteus_tool` and `tview` can be configured to communicate with a
moteus controller through a variety of transports.  By default, it
will attempt to autodetect either a fdcanusb or socketcan interface.

To force usage of a particular fdcanusb, you can use:

```
python3 -m moteus.moteus_tool --fdcanusb /path/to/fdcanusb
```

To force a particular python-can method, you can use:

```
python3 -m moteus.moteus_tool --can-iface socketcan --can-chan can0
```

where `--can-iface` specifies the "interface" for python-can and
`--can-chan` specifies the "channel".

Note, these are in addition to any other `moteus_tool` or `tview`
options that may be desired.
