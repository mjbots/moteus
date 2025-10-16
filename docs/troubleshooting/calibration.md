# Troubleshooting Calibration

When setting up a [moteus controller](https://mjbots.com) with a new motor, you typically first run the automatic calibration sequence as [documented in the reference manual](../guides/calibration.md). When your system is working well, that should be all that is necessary to enable moteus to perform accurate FOC based torque control of the motor. Then you can continue on with the remainder of the items in the [Quick Start](../quick-start.md). What happens though when the automatic calibration doesn't work? This article describes the most common failure modes during calibration.

Unfortunately, most calibration failures present the same final symptom, an error like:

```
CAL timeout
```

or

```
encoder not an integral multiple of phase
```

So to resolve, you typically start with the first items in this article, then work your way down to the less common ones.

## Sense magnet issues

To operate properly using the onboard encoder, moteus requires that a diametrically magnetized sense magnet be rigidly affixed to the rotor, that it be centered laterally under the magnetic encoder on moteus, and that it has an appropriate air-gap. Let's look at each of those in turn.

First, you need a [diametrically magnetized magnet](https://www.kjmagnetics.com/magdir.asp). This is one where the axis of magnetization is sideways, so that the north pole points out of the curved side of the disc. This is contrast to axially magnetized magnets where the axis of magnetization points out of the flat side. There are a wide variety of possible options here. Every moteus controller sold at [mjbots.com](https://mjbots.com) includes an appropriate magnet, and [mjbots.com also sells spares](https://mjbots.com/products/d42dia). There are also suppliers with higher temperature magnets, like the [Radial Magnets, Inc. 9049 here at digikey](https://www.digikey.com/en/products/detail/radial-magnets-inc/9049/6030786). Sometimes however, you are converting an existing actuator to use moteus, and there is a sense magnet present already. The bad news is that it may or may not be usable with moteus and unless you know the specifications, it may be hard to know the optimal air gap required for it. In those cases it is often easiest to remove the existing magnet and replace it with a known quantity, especially if a reasonable air gap is hard to achieve.

Second, the magnet must be rigidly affixed to the rotor. No slipping is permitted whatsoever. Practically, that means that adhesive will be required at each junction between the magnet and the rotor. Common adhesives used are cyanoacrylate gel (superglue), and epoxy. For instance, if a 3D printed spacer is used, the magnet must be glued to the spacer, and the spacer must be glued to the rotor. When the rotor moves, the magnet must move. Press fits can be problematic unless the fit is designed with a well specified tolerance and relying on friction between a magnet and something it is attracted to will definitely not work.

Third, the magnet must be positioned laterally properly. This means that it needs to be centered under the magnetic encoder. For [moteus-n1](https://mjbots.com/products/moteus-n1) and [moteus-c1](https://mjbots.com/products/moteus-c1), that is centered under the bolt pattern. For [moteus-r4](https://mjbots.com/products/moteus-r4-11), you should [look at the 2D CAD](https://github.com/mjbots/moteus/blob/main/hw/controller/r4.5/20210124-moteus-controller-r45-mechanical.pdf) to see where to position the magnet.

The magnet also needs an appropriate air gap between the surface of the magnet and the surface of the encoder IC. For the magnets that mjbots sells, 2mm is near the optimal air gap, with a range of 1.5mm - 2.5mm being suitable. Spacing of 0.5mm - 8mm will likely work with degraded performance.

Finally, the controller needs itself to be rigidly mounted with respect to the stator and rotor. If it is allowed to move or rotate, then the sensed angle of the rotor will not match the actual electrical angle needed for commutation, as well as degrading the quality of position control at the output.

## Electrical problems

The most common electrical problem that prevents calibration is poor or non-existent phase wire soldering. The phase wires of the motor must make a good connection to the moteus phase terminals. That means that when soldering, the joint must have reached a sufficient temperature and had sufficient flux such that the solder wicked both all around the wire, and wicked in to the phase wire terminal. Cold solder joints may show solder that does not fill the joint completely and does not form a proper fillet to the surfaces it touches or may show as visual artifacts between the wire, pad, and solder. This may result in no or an intermittent connection. A video showing what an appropriate result should look like is here:

{{ youtube("mZ9w_TaWmjQ") }}

The next most common electrical problem is a motor that is fundamentally broken. That could be one which has shorted phases, inappropriate polarity of windings, or magnets with inappropriate polarity. These problems usually only appear with second hand motors, those of low quality, or hand-built motors. The easiest way to rule it out is to test with a known working motor.

## Less Common Sense Magnet Issues

I have observed a few less common problems associated with sense magnets.

In one case, the magnet was actually glued both to the rotor and to a stationary mounting bracket, preventing the rotor from moving at all.

In other cases, magnets were allowed to exceed their rated operating temperature. The magnets mjbots sells as of May 2024 are rated to 80C. If the temperature approaches that point, often above 70C, then the axis of magnetization can drift. If that is an issue, then magnets can be procured with higher rated temperatures.

Sense magnets for commutation must be rigidly affixed to the rotor, \*not\* to the output of a reducer, or any linkage that may have backlash. If the magnet is attached to a reducer, then the required angular accuracy can easily exceed what the onboard encoder is capable of measuring. Linkages with backlash have the same problem.

## Mechanical Problems

If your rotor is not free to move, or has a significant load on it, then calibration can fail. Calibration will work with many kinds of reducers attached, but eventually the friction and stiction will be more than the process can manage. In some cases you can increase the power used during the lock-in phase of calibration by passing an extra option to `moteus_tool --calibrate`:

```
--cal-motor-power 5
```

The units are watts, and 5 is the default. You can increase this, but at higher values, you may run the risk of overheating your motor during the encoder calibration process.

If a suitable power cannot be found, then it may be required to design the motor and controller as a sub-assembly which can be first calibrated and then attached to the reducer.

## The bad news

The bad news is that the error checking in the calibration process is not foolproof. i.e. it is possible for a setup that is broken to still report that calibration completed successfully. The two biggest indicators that it did not actually work are that the Kv value reported during calibration is grossly incorrect or that position or torque control do not work reliably. If either of those happen, you need to work through the above potential causes and rule them out one by one.

## The good news

The good news is that if you have a working motor, an appropriate magnet, the magnet is rigidly affixed to the rotor and positioned properly, and your phase wire soldering followed proper technique, calibration should succeed virtually 100% of the time.
