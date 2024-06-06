# Getting started with the moteus controller #

If you are starting from a developer kit, then skip down to the section [Software](#software).

# Hardware #

## Mechanical Mounting ##

If using the default onboard encoder, the following steps must be taken:

1. A diametrically magnetized sense magnet should be attached to the rotor.  The fastening must be rigid, allowing no slip.  Adhesive is generally required, cyanoacrylate or epoxy can be used.
2. The moteus controller must be mounted so that the encoder is (a) centered laterally on the sense magnet and (b) has an air-gap of approximately 2mm.  The moteus-r4 encoder is *not* centered under the bolt pattern, check the 2D CAD for details.  This mount must be rigid and allow no slip.

## Electrical ##

Phase wires for the motor should be soldered to the pads labeled A, B, and C.  The order does not matter.  An inline connector, like an MR30, MR60, or bullet connectors can be used if repeated disconnection is desired.

*IMPORTANT NOTE ON ELECTRICAL DAMAGE*

moteus uses moderately large currents and moderately high voltages. It is important to take many factors in consideration when deploying it to avoid electrical damage to the controller.  Before soldering cables, or attaching moteus to a new motor, be sure to read and understand each of the following sections in the reference manual:

 * [Phase Wire Soldering](reference.md#phase-wire-soldering)
 * [Cable Construction](reference.md#power-cable-construction)
 * [Power Connectorization](reference.md#power-connectorization)
 * [Regenerative Braking Safety](reference.md#regenerative-braking-safety)

# Initial Parameters #

There are a few parameters you will likely want to configure early on
in your setup.

* `servopos.position_min` and `servopos.position_max` these define the bounds of motion which the controller will allow when in position control mode.  Attempting to start beyond this region will fault, and if outside the region in operation, no torque will be applied to go further outside.
* `servo.max_current_A` the maximum phase current to apply to the motor.  This can be used to limit the maximum torque that the system is capable of regardless of any command sent.
* `servo.max_velocity` limits the maximum speed the motor is permitted to achieve before no torque is produced
* `servo.default_velocity_limit` / `servo.default_accel_limit` controls how fast the motor can accelerate and spin in order to reach position and velocity targets.  Bare boards ship with these unset, while development kits ship with human-eye pleasing values.
* `servo.pid_position` the PID parameters for the position control loop, a [possible tuning procedure](reference.md#pid-tuning) can be found in the reference manual.
* `motor_position.rotor_to_output_ratio` any gearbox scaling, a reducing gearbox should be configured with a number smaller than one, e.g. 0.25 for a 4x reduction gearbox.  This affects reported position, speed, and torques.
* `id.id` the CAN-FD id used by this device

A larger set of parameters is documented in the reference manual.

# Calibration #

If you started from a bare moteus board, you will need to calibrate it for the attached motor before any control modes are possible.

```
python3 -m moteus.moteus_tool --target 1 --calibrate
```

WARNING: Any attached motor must be able to spin freely.  It will be
spun in both directions and at high speed.

# Software #

Binary versions of `tview` and `moteus_tool` can be installed for most
platforms (desktop Linux, Windows, Mac) via python using the
`moteus_gui` package.

```
pip3 install moteus_gui
```

## Raspberry Pi ##

If you have a Raspberry Pi, see the [instructions here](raspberry_pi.md).

## Linux ##

If using the fdcanusb, you will need to have udev rules set up in order for regular users to access the device.  Follow the instructions at: https://github.com/mjbots/fdcanusb/blob/master/70-fdcanusb.rules

## Windows ##

On some modern Windows installations, you need to invoke python as `python`, not `python3`.

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

 * Position: The desired position in revolutions
 * Velocity: The rate at which the desired position changes in
   revolutions / s
 * Maximum torque: Never use more than this amount of torque when controlling
 * Feedforward torque: Give this much extra torque beyond what the
   normal control loop says
 * Stop position: If non-special, never move the desired position away
   from this target.
 * kp scale: Scale the proportional constant by this factor
 * kd scale: Scale the derivative constant by this factor
 * Velocity limit override: If non-special, override the configured
   velocity limit, which constrains how quickly the target position is
   reached.
 * Acceleration limit override: If non-special, override the
   configured acceleration limit, which constrains how quickly the
   target position is reached.

Additionally, the position may be set as a "special value" (NaN for
floating point and the debug interface, maximal negative for integer
encodings).  In that case, the position selected is "wherever you are
right now".

A pure velocity mode can be obtained by setting the kp scale to 0 (or
permanently so by configuring the kp constant to 0).  In this case,
using the `servo.max_position_slip` configurable parameter may be
valuable as per the [reference manual](reference.md#velocity-control).


# Learning more #

The complete reference documentation can be found at:
[Reference](reference.md)
