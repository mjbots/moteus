# Dual Encoder Configuration

Often moteus is used in applications where a gearbox or other reducer is used between the rotor and the final plant.  In those situations it can be valuable to have a secondary encoder measuring the output of the reducer:

**Absolute positioning**: With only an encoder on the rotor, it is not possible to know absolutely where the output is.

**Improved accuracy or sensing performance**: The onboard encoder included with moteus is sufficient for commutation for nearly all motors, however some applications may require lower position or velocity sensing noise or higher resolution in order to achieve accurate very slow movements, or very accurate positioning.

This section will cover how to set up several common dual encoder configurations.

## <a name="match"></a>Important factors when configuring dual encoders

1. **Sign match**: moteus requires that the sign of all encoders match.  Those must be configured manually in `motor_position.sources.X.sign`.  To verify the signs are correct, plot `motor_position.sources.X.filtered_value` for all configured encoders, spin the system by hand, and verify that all plots move in the same direction.

2. **Offset match**: If the absolute values of multiple encoders must be related, such as for a disambiguation configuration, then the offsets must be matched so that the encoders read 0 at the same time.  These offsets must be manually configured in `motor_position.sources.X.offset`.  To verify they are correct, plot `motor_position.sources.X.filtered_value` for all configured encoders, spin the system by hand to the zero point, and verify that all encoders read 0.

## Onboard + 8x reducer + MA600 on-axis - Disambiguation

In this configuration, the rotor is sensed with the moteus onboard encoder and the output is sensed with an MA600 on-axis encoder.  Assuming low backlash, the output positioning is performed from the onboard encoder and the output encoder is only used for disambiguation.  This results in the best performance since the effective resolution of the onboard encoder is multiplied by the gear reduction and will be much better than the MA600 on the output.

The MA600 is attached to the aux2 port on one of the moteus-c1 [(c1 pinout)](../reference/pinouts.md#moteus-c1), moteus-n1 [(n1 pinout)](../reference/pinouts.md#moteus-n1), or moteus-x1 [(x1 pinout)](../reference/pinouts.md#moteus-x1).

This pinout is the one accomplished if using the cable included with the [mjbots MA600 breakout board](https://mjbots.com/products/ma600-breakout).

If starting from a factory default configuration, first, configure the
aux2 port:

```
aux2.pins.0.mode 1  # spi
aux2.pins.1.mode 1  # spi
aux2.pins.2.mode 1  # spi
aux2.pins.3.mode 2  # spi_cs
aux2.spi.mode 5     # ma600
aux2.spi.rate_hz 6000000
```

Then, configure motor_position:

```
motor_position.sources.1.aux_number 2
motor_position.sources.1.type 1
motor_position.sources.1.cpr 65536
motor_position.sources.1.reference 1  # output
motor_position.rotor_to_output_ratio 0.125
motor_position.output.reference_source 1
```

The sign and offset of the MA600 will need to be matched to the onboard encoder as per [the above section](#match).

## Onboard + 10x reducer + AksIM-2 on output

In this configuration, the onboard encoder is used to sense the rotor, and an AksIM-2 encoder from RLS is used on the output.  The AksIM-2 has much better accuracy and noise performance than the onboard on-axis encoder, and thus this configuration could be used for applications where either very accurate positioning is required, or very precise low speeds are required.

The AksIM-2 is attached to the GH-6 RS422 connector on a moteus-n1 [(n1 pinout)](../reference/pinouts.md#moteus-n1) or moteus-x1 [(x1 pinout)](../reference/pinouts.md#moteus-x1).

moteus is compatible with AksIM-2 encoders configured as follows:

- **Communication Interface**: SF - Asynchronous Serial, RS422, 5V
- **Communication Protocol Variant**: F - 1000 kbps
- **Resolution**: <= 19B, no multiturn support

If starting from a factory default configuration, first configure the aux1 port.

```
aux1.pins.4.mode 3 # uart
aux1.pins.5.mode 3 # uart
aux1.uart.mode 1 # aksim2
aux1.uart.baud_rate 1000000
aux1.uart.rs422 1
```

Then configure motor_position:

```
motor_position.rotor_to_output_ratio 0.1
motor_position.sources.1.aux_number 1
motor_position.sources.1.type 2          # uart
motor_position.sources.1.cpr 4194304     # used by moteus for all AksIM-2
motor_position.sources.1.reference 1     # output
motor_position.output_source 1
```

The sign and offset of the AksIM-2 will need to be matched to the onboard encoder as per [the above section](#match).

## Hall + 10x reducer + Off axis MA600

In this configuration, hall effect encoders are used to sense commutation and an MA600 is used in conjuction with a diametrically magnetized ring magnet for output positioning.  It is designed as one of the lowest cost options for a hollow axis actuator with a gear reduction, as the off-axis MA600 has minimally acceptable performance as does the hall effect sensors.

The MA600 is connected to AUX1 on a moteus-n1 [(n1 pinout)](../reference/pinouts.md#moteus-n1) or moteus-x1 [(x1 pinout)](../reference/pinouts.md#moteus-x1) and the hall effect sensors are connected to AUX2.

First, we will configure the hall effect sensors and calibrate them.

```
aux2.pins.0.mode 6  # hall
aux2.pins.0.pull 1  # pull_up
aux2.pins.1.mode 6  # hall
aux2.pins.1.pull 1  # pull_up
aux2.pins.2.mode 6  # hall
aux2.pins.2.pull 1  # pull_up
aux2.hall.enabled 1
motor_position.rotor_to_output_ratio 0.1
motor_position.sources.0.aux_number 2
motor_position.sources.0.type 4  # hall
```

Then, the hall effect sensors are calibrated:

```
python -m moteus.moteus_tool -t 1 --calibrate --cal-motor-poles 30
```

Now we will configure the MA600.

```
aux1.pins.0.mode 1  # spi
aux1.pins.1.mode 1  # spi
aux1.pins.2.mode 1  # spi
aux1.pins.3.mode 2  # spi_cs
aux1.spi.mode 5     # ma600
aux1.spi_rate_hz 6000000
motor_position.sources.1.aux_number 1
motor_position.sources.1.type 1  # spi
motor_position.sources.1.cpr 65536
motor_position.sources.1.reference 1  # output
```

Then, before configuring the MA600 to be used for output, match the sign of the MA600 and hall effects according to [the procedure here](#match).

For an off-axis MA600, BCT tuning should be performed, and non-linear compensation may be performed.

```
moteus$ ./utils/measure_ma732_bct.py
```

To peform non-linear compensation, you can use:

```
moteus$ ./utils/compensate_encoder.py
```

Finally, the MA600 can be configured as the output source:

```
motor_position.output.source 1
```
