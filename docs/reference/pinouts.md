# Pinouts

## moteus-c1

<img src="../c1/moteus-c1-pinout-rendered.svg" width="100%" />

## moteus-r4

<img src="../r4/moteus-r4-pinout-rendered.svg" width="100%" />

## moteus-x1

<img src="../x1/moteus-x1-pinout-rendered.svg" width="100%" />

## moteus-n1

<img src="../n1/moteus-n1-pinout-rendered.svg" width="100%" />

## Additional Information

### CAN Termination

CAN connections should be terminated by a 120 ohm resistor at both
ends of a bus.  Some mjbots products have built in termination
resistors, such as the pi3hat.  The mjcanfd-usb-1x and fdcanusb have a
software configurable termination resistor that is by default on.
moteus controllers have no termination resistors.  For very short
runs, the system will work terminated only on one side.  However, when
runs become longer than 0.5m, you will likely need to terminate both
ends.  This can be done by crimping a 120 ohm resistor into a JST PH3
connector and connecting it to the open data connector or by
purchasing a [CAN terminator](https://mjbots.com/products/jst-ph3-can-fd-terminator).

### Recommended Mating Hardware

| Connector | Mate P/N | Terminal P/N | Pre-crimped Wire |
|-----------|----------|--------------|------------------|
| JST PH-3  | [PHR-3](https://mjbots.com/products/phr-3)  | [SPH-002T-P0.5L](https://mjbots.com/products/jst-ph-terminal) | [10cm, 30cm, 50cm PH3](https://mjbots.com/products/jst-ph3-cable) |
| JST GH-6  | [GHR-06V-S](https://mjbots.com/products/jst-gh6-housing) | [SSHL-002T-P0.2](https://www.digikey.com/en/products/detail/jst-sales-america-inc/SSHL-002T-P0-2/807828) | [28 AWG 20cm](https://mjbots.com/products/jst-gh-wire) |
| JST GH-7  | [GHR-07V-S](https://mjbots.com/products/jst-gh7-housing) | [SSHL-002T-P0.2](https://www.digikey.com/en/products/detail/jst-sales-america-inc/SSHL-002T-P0-2/807828) | [28 AWG 20cm](https://mjbots.com/products/jst-gh-wire) |
| JST GH-8  | [GHR-08V-S](https://mjbots.com/products/jst-gh8-housing) | [SSHL-002T-P0.2](https://www.digikey.com/en/products/detail/jst-sales-america-inc/SSHL-002T-P0-2/807828) | [28 AWG 20cm](https://mjbots.com/products/jst-gh-wire) |
| JST ZH-4  | [ZHR-4](https://www.digikey.com/en/products/detail/jst-sales-america-inc/ZHR-4/608643) | [SZH-002T-P0.5](https://www.digikey.com/en/products/detail/jst-sales-america-inc/SZH-002T-P0-5/527363) | |
| JST ZH-6  | [ZHR-6](https://www.digikey.com/en/products/detail/jst-sales-america-inc/ZHR-6/527361) | [SZH-002T-P0.5](https://www.digikey.com/en/products/detail/jst-sales-america-inc/SZH-002T-P0-5/527363) | [stm32 programmer](https://mjbots.com/products/stm32-programmer) |
| AMass XT30 | [XT30U-F](https://mjbots.com/products/xt30u-f) | | |

### Recommended Low Cost Manual Crimp Tools

- **JST PH**: [TU-190-08](https://www.amazon.com/TU-190-08-Terminals-Tool-Crimping-0-08-0-5sq-mm/dp/B01M25OLZY)
- **JST ZH**: [PEBA 1020M](https://www.amazon.com/PEBA-Ratcheting-Connectors-Crimping-terminals/dp/B0D7PKLBZT)
- **JST GH**: [PEBA 1020M](https://www.amazon.com/PEBA-Ratcheting-Connectors-Crimping-terminals/dp/B0D7PKLBZT)

### moteus-r4 - Pico-SPOX 6 ENC

The ENC/AUX1 connector on moteus-r4 are intended to be populated with
a Molex Pico-SPOX 6 connector PN 0874380643, or as an alternate, TE
5-1775444-6.
