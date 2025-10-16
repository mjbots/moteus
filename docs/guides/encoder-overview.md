# Encoder Overview

This section describes the various ways moteus can be configured to use encoders to sense the rotor, output, or auxiliary devices.  If you intend to only use the onboard on-axis magnetic encoder, you can skip this section.

## Introduction

moteus uses encoders to properly control a motor. Encoders provide the feedback needed to know where the motor is and how fast it's moving. Understanding which encoder(s) to use and how to configure them is one of the key decisions you need to make when designing a system, as for many applications encoder selection is the primary determinant of performance parameters like maximum stiffness, control stability and whether homing will be necessary.

Moteus uses encoders for two distinct purposes. First, it needs **commutation** feedback - the electrical relationship between the motor's stator and rotor to apply torque correctly. Second, it needs **output** feedback - the position and velocity of your output shaft to follow commanded trajectories. A single encoder can serve both purposes, or you can use separate encoders for each.

The good news is that moteus comes with an onboard AS5047P or MA600 magnetic encoder that works out of the box for many applications. This encoder mounts on-axis with the motor. For many use cases, especially those without gear reductions, this is all you need. However, if you're using hall sensors, need higher resolution, or have a gear reducer that you want to measure the output of a reducer or other auxiliary devices, you'll want to add other encoders.

In this guide, we'll cover a few common variants and leave the rest for [the detailed encoder reference](../reference/encoders.md).

## Common Configurations

Here are the most common encoder configurations you'll encounter:

**Default configuration (onboard encoder only):** The AS5047P or MA600 onboard encoder serves as both commutation and output sensor. This works well for many applications, especially those without a reducer installed or with a reducer where either the absolute position of the output is not required or a homing step is permitted.

**Adding hall sensors:** Hall effect sensors provide robust commutation feedback and are useful in high-vibration environments or when cost is a primary concern.  When used as the only encoder, the resulting system will have poor low-velocity and position tracking performance.

**Dual encoder setups:** Using one encoder for commutation (typically the onboard or hall sensors) and a separate high-resolution encoder for output position provides the best performance for precision applications.

These two options are described in separate guides below:

- [Motor with Hall Effects](hall-effects.md) - Adding hall sensors for commutation
- [Dual Encoders](dual-encoders.md) - Using separate encoders for commutation and output

Once again, for detailed information about encoder configuration parameters, see the [Detailed Encoder Reference](../reference/encoders.md).

## Next Steps

Once the encoder has been configured, then you are ready to move on to the [motor calibration procedure](calibration.md).
