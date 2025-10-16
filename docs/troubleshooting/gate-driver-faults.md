# Troubleshooting Gate Driver Faults

Certain types of electrical problems in moteus can result in the onboard MOSFET gate drivers declaring an error condition.  When that happens, moteus will report fault 33.  This document describes how to troubleshoot the issue.

## Identifying the cause

To determine *which* fault the gate driver flagged, you need to look at the `drv8323` tree in tview or with `python -m moteus.moteus_tool -t 1 --read drv8323`.  Each fault has a different boolean value associated with it, and multiple can be flagged at the same time.  From a diagnostic perspective though, they boil down into two categories:

## Recoverable faults

The only fault type that, if repeatable, is easily recoverable is the `uvlo=true` fault.  This means that the gate driver identified the input supply voltage dropped below the minimum operation threshold.  It happens when moteus attempts to draw more power than the supply or battery can provide.  Depending upon the cause, there are several potential resolutions:

**moteus needs to output more torque/power**: If the fault triggered but moteus was not yet applying sufficient torque for your application, then you must change your supply to be able to provide more power.  If the supply is a current limited lab supply, you could increase the current limit.  If it is a fixed current wall supply, you could switch to one with a higher power output.  If it is a battery, you could switch to one with a higher peak current rating.  It is also possible to occur due to poor supply wiring, i.e. if the resistance of the supply power cables is too high.

**moteus should not attempt to draw that much power**: If moteus was attempting to apply more torque or power than is desirable, there are two limits which can be adjusted.

1. `servo.max_current_A` - This will determine the maximum phase current moteus can output and will correspond to the maximum amount of torque it can output.  This does not directly correlate with supply power, as the supply power required also scales linearly with the motor velocity.  However, it is good to ensure this is not set larger than it needs to be.
2. `servo.max_power_W` - This will determine the maximum amount of power moteus can output to the motor, and thus roughly the amount of supply power it can consume.  If using a power supply with known voltage and current capability, you can set this to be some fraction of that power to leave sufficient margin.

## Unrecoverable faults

Most non-uvlo faults the gate driver reports indicate that the controller has suffered electrical damage.  Most of the reasons are laid out in the electrical section of the user guide:

- [Using moteus: Electrical](../guides/electrical-setup.md)

i.e. poor wire construction can result in micro-arcing which further causes large voltage transients either at the supply terminals, or the motor phase terminals.  Lack of inrush current management also results in large voltage transients at the supply terminals.  Poor strain relief of XT30 connectors, once again, can cause micro-arcing and large voltage transients.

In these cases, it may be possible to recover the controller by replacing components, but doing so requires moderate to advanced SMT rework capabilities that are beyond most users.  Usually the appropriate action is to identify which of the electrical installation guidelines were not adhered to, address them, then replace the controller with a new one.
