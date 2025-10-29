# moteus Brushless Servo Documentation

Welcome to the documentation site for the moteus brushless motor controller.

## What is moteus?

moteus controllers are high-performance modular brushless motor controllers with integrated on-axis magnetic encoders, designed for robotics applications. They feature:

- **Field Oriented Control (FOC)** for 3-phase brushless motors
- **Integrated magnetic encoder** for precise position sensing
- **High-speed CAN-FD communication** at 5Mbps
- **Multiple control modes**: Position, velocity, and torque control with acceleration and velocity limited trajectories
- **Fast control loops** running at 15-30kHz

## Hardware Variants

| Name   | Voltage Input | Peak Power     | Mass  | Dimensions   |
|--------|---------------|----------------|-------|--------------|
| r4.11  | 10-44V        | 900W @ 30V     | 14.2g | 46x53mm      |
| c1     | 10-51V        | 250W @ 28V     | 8.9g  | 38x38x9mm    |
| n1     | 10-54V        | 2kW @ 36V      | 14.6g | 46x46x8mm    |
| x1     | 10-54V        | 1.3kW @ 36V    | 23.8g | 56x56x10mm   |

Assembled and tested boards can be purchased at [mjbots.com](https://mjbots.com)

## Documentation

- [Quick Start](quick-start.md)
- [Using Moteus](guides/mechanical-setup.md)
- [Integration](integration/python.md)
- [Troubleshooting](troubleshooting/calibration.md)
- [Reference](reference/pinouts.md)

## Community & Support

- [Discord Community](https://discord.gg/W4hUpBb)
- [GitHub Repository](https://github.com/mjbots/moteus)
- [Purchase Hardware](https://mjbots.com)

## License

All files in this repository are available under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0).

!!! note "Trademark Notice"
    mjbots Robotic Systems LLC owns and protects the "mjbots" and "moteus" trademarks. Please read the [Trademark Policy](https://mjbots.com/trademark-policy) if you want to use these names in your project.
