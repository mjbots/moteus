# Getting started with the moteus controller #

Binary versions of `tview` and `moteus_tool` can be installed for most
platforms (desktop Linux, Windows, Mac) via python using the
`moteus_gui` package.

```
pip3 install moteus_gui
```

If you have a Raspberry Pi, see the [instructions here](raspberry_pi.md).

# Running tview #

tview lets you configure and inspect the state of the controller.  It
can be run like:

```
python3 -m moteus_gui.tview --devices=1
```

(Your pip may have installed a `tview` script into your path which you
could also use).

By default, it attempts to detect an attached fdcanusb to communicate
with the target.

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

# How position mode works #

The primary control mode for the moteus controller is an integrated
position/velocity controller.  The semantics of the control command
are somewhat unorthodox, so as to be easily amenable to idempotent
commands sent from a higher level controller.  Each command looks
like:

 * Position: The desired position *right now* in revolutions
 * Velocity: The rate at which the desired position changes in
   revolutions / s
 * Maximum torque: Never use more than this amount of torque when controlling
 * Feedforward torque: Give this much extra torque beyond what the
   normal control loop says
 * Stop position: If non-special, never move the desired position away
   from this target.
 * kp scale: Scale the proportional constant by this factor
 * kd scale: Scale the derivative constant by this factor

Additionally, the position may be set as a "special value" (NaN for
floating point and the debug interface, maximal negative for integer
encodings).  In that case, the position selected is "wherever you are
right now".

Some limited amount of preprogrammed constant velocity trajectories
can be emulated using an unset position and the stop position.  In
that case, the sign of the velocity command is ignored, and is instead
selected to point towards the stop position.  If that is the only
command, or that command is repeated indefinitely, it will have the
same effect of causing the controller to move to the stop position at
a constant velocity.

A pure velocity mode can be obtained by setting the kp scale to 0 (or
permanently so by configuring the kp constant to 0).

# Initial Configuration #

There are a few parameters you will likely want to configure early on
in your setup:

* `servopos.position_min` and `servopos.position_max` these define the bounds of motion which the controller will allow when in position control mode.  Attempting to start beyond this region will fault, and if outside the region in operation, no torque will be applied to go further outside.
* `servo.max_current_A` the maximum phase current to apply to the motor.  This can be used to limit the maximum torque that the system is capable of regardless of any command sent.
* `servo.pid_position` the PID parameters for the position control loop.
* `motor.unwrapped_position_scale` any gearbox scaling, a reducing gearbox should be configured with a number smaller than one, e.g. 0.25 for a 4x reduction gearbox.  This affects reported position, speed, and torques.
* `id.id` the CAN-FD id used by this device

A larger set of parameters is documented in the reference manual.

# Learning more #

The complete reference documentation can be found at:
[Reference](reference.md)
