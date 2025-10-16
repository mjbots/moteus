# Nvidia Jetson Setup

Many (maybe all) NVIDIA Jetson boards include an onboard CAN-FD controller.  However, very few NVIDIA Jetson carrier boards include a CAN-FD transceiver.  The former manages the logic level CAN-FD protocol, the latter translates to the electrical wire interface and both are required.

There are three major options for communicating with moteus controllers from an NVIDIA Jetson board:

1. **USB Adapter**: A mjcanfd-usb-1x or similar USB adapter can be used.  This is simple and relatively convenient, although is bulky and can be unreliable in systems that experience physical vibration or EMI.

2. **PCIe/m.2 Adapter**: Many Jetson boards have an accessible m.2 slot.  These can be used with adapters like the [PEAK m.2 CAN-FD adapter](https://www.peak-system.com/PCAN-M-2.473.0.html?&L=1).  Do note that some Jetson carrier boards have an m.2 adapter, but it is physically mounted in such a way that precludes using the PEAK adapter specifically because mechanical interference would result.

3. **Onboard Jetson Controller**: To use this, you either need a carrier board with a CAN-FD transceiver, or you must connect an external transceiver to the appropriate pins.

For option 2 or 3, you then configure the CAN-FD interface using the same method as for any socketcan device.  That reference is here:

- [socketcan Configuration](../platforms/socketcan.md)
