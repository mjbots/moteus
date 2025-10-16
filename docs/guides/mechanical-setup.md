# Mechanical Setup

To use the onboard on-axis encoder integrated with moteus, the controller must be mounted with care to ensure proper alignment and ongoing registration.

## Mechanical Mounting

If using the default onboard encoder, the following steps must be taken:

1. A diametrically magnetized sense magnet should be attached to the rotor. The fastening must be rigid, allowing no slip. Adhesive is generally required, cyanoacrylate or epoxy can be used.
2. The moteus controller must be mounted so that the encoder is (a) centered laterally on the sense magnet and (b) has an air-gap of approximately 2mm. This mount must be rigid and allow no slip.

!!! note "moteus-r4 mounting"
    Unlike other moteus controllers, the moteus-r4 encoder is *not* centered under the bolt pattern, check the [2D CAD for details](https://github.com/mjbots/moteus/blob/main/hw/controller/r4.5/20210124-moteus-controller-r45-mechanical.pdf).

!!! note "3D printed mounts"
    If using 3D printed mounts, care should be taken to ensure the mount does not warp.  PLA and other low temperature filaments will warp under load at room temperature, and will do so even more when exposed to thermal heating like from a motor controller.  It is recommended to use PETG, PC, or a filament designed to not warp at the operating temperature you intend.

## Next Steps

Once mechanical mounting is complete, proceed to [Electrical Setup](electrical-setup.md) to connect the motor phase wires and power.
