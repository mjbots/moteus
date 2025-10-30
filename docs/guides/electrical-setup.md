# Electrical Setup

To use moteus without damage, proper care must be paid to the electrical connections and configuration.  This section describes how.

## Electrical Connections

**Motor**: Phase wires for the motor should be soldered to the pads labeled A, B, and C. The order does not matter. The location of the pads varies between controllers.  An inline connector, like an MR30, MR60, or bullet connectors can be used if repeated disconnection is desired.

**Power**: Power is provided via either the onboard XT30 connector, or on board-mountable moteus controllers, via the power pads that are interleaved with the A, B, and C phase pads.  The power input is *NOT* protected against reverse polarity.  Polarity should be triple checked before applying power, or irreversible damage may result.  XT30 connectors have a clear '+' and '-' marking on them which can be used to identify the positive and negative input.

If you are unsure, pinout diagrams for all controllers can be found here: [Pinout diagrams](../reference/pinouts.md)

!!! note "Hot plugging"
    It is important to *never* hot plug an XT30 connector.  The XT30 connector (and all connectors which are not anti-spark), should be connected *before* power is applied.  If you see or hear a spark, you have applied power in the wrong order.  See the below section on inrush current management.

## Soldering and cable construction techniques

moteus controllers can be damaged when improper techniques are used to attach wires to them, or improper techniques are used to construct power cables.

### Phase Wire Soldering

The connections between moteus and the motor under control must be high quality.  Poor quality solder joints can cause intermittent changes in resistance, sparking, and high voltage transients.  High quality solder joints require:

* Sufficient flux to clean oxides off of the wire and the via
* Sufficient temperature so that the entire wire and via is hot enough for the solder to wet all surfaces

For the former, it is often necessary to add copious amounts of additional flux in addition to the rosin core inside many solders.  For the latter, a high power soldering iron, with a fat tip, and what seems like a long time may be necessary.  The temperature is definitely insufficient if solder touched to any part of the via or wire does not melt and wick instantly.  Usually a small amount of solder is placed between the tip and wire to act as a heat transfer agent.  Then the iron rests on the via and wire for between 5-30s until it looks like the initial solder has wicked fully into the via.  Then additional solder can be added until it forms a clean smooth fillet around the entire via wicking up to the wire.

When complete, the back side of the hole can be examined, and if successful, solder will have wicked partially down out of the via onto the other side forming a clean, smooth fillet there as well.

A demonstration of this can be found in the following video: [https://www.youtube.com/watch?v=mZ9w_TaWmjQ](https://www.youtube.com/watch?v=mZ9w_TaWmjQ)

{{ youtube("mZ9w_TaWmjQ") }}

### Power Cable Construction

When constructing power cables using XT30 connectors, it is critical that solder joints be sound, otherwise intermittent connectivity can result.  This can cause sparking and high voltage transients.  The soldering principles are the same as in the phase wire soldering section above.

A demonstration of XT30 soldering can be found in the following video:
[https://www.youtube.com/watch?v=f6WtDFWuxuQ](https://www.youtube.com/watch?v=f6WtDFWuxuQ)

{{ youtube("f6WtDFWuxuQ") }}

### Power Connectorization

For moteus to operate without damage, the XT30 connectors used to transmit power must make a solid connection that is non-intermittent.  As with poor soldering, an intermittent connection can cause inductive spikes, which will destroy components on the controller.

When connectors are functional, moderate insertion force should be required and the connectors should not "wiggle" much after insertion.

The XT30 is not rated for any significant amount of mechanical force when mated.  For any application where cables may flex, strain relief should be used such that no force is applied to the connector.  The connector may be damaged if a system goes "out of control" even in one instance, although milder mechanical stress may cause accelerated fatigue and failure.

If sparks are observed, that is *definitely* a problem and the system should be powered off until the connectors can be replaced and mitigations made for what led to that event.

Genuine AMASS connectors are rated for 1000 insertions assuming no other mechanical damage, however off-brand connectors may have worse tolerances and may not make a reliable connection for even one insertion.


### Long daisy chains

When connecting more than 3 moteus controllers or servos in a system on a single power or CAN chain, additional factors should be considered.

1. It is recommended to use T-spliced power instead of using the daisy chained power connectors for chains greater than 3 units.  Small connector mis-seating or micro-arcing will cause magnified effects with long chains of controllers.

2. CAN bus electrical performance can be a limiting factor.  To operate at the default 5Mbps, you will need to ensure that CAN wires are twisted, all crimps are high quality, and that termination is installed.  Split termination may be required, where two 60 ohm resistors are connected in series with a small filter capacitor connected between the center tap of the resistors and ground.  A 4.7nF capacitor is recommended.  If that is insufficient, BRS can be disabled to operate only at 1Mbps.  With `moteus_tool` or `tview`, the `--can-disable-brs` flag can be used.

mjbots sells a CAN terminator that can be used for this purpose: [https://mjbots.com/products/jst-ph3-can-fd-terminator](https://mjbots.com/products/jst-ph3-can-fd-terminator)

## Inrush current management

moteus controllers have large DC bulk capacitance with low ESR, this is how they can achieve their small form factor.  As a result, care must be taken when applying power.  If a connector is "hot-plugged" extremely large transient currents can develop which may damage the moteus or other components in the system.  These challenges are magnified when more than one moteus controller is attached to the same power bus.

Potential solutions:

**Power Supply**: If a wall power supply is used, it can be relied upon to provide a "soft start" functionality.  Ensure that all moteus controllers are attached to the power supply *before* enabling the supply by turning it on or plugging it into a wall.

**Anti-spark connectors**: There exist anti-spark connectors like the XT90-S, which have a power resistor built in which makes contact before main power is applied.  These can be used, but have several drawbacks:

1. The proper order of connection must always be obeyed.  Unless the XT90-S is connected *last*, then inrush transients may result.
2. If the power rating on the built-in resistor is not respected, it will silently fail, resulting in no pre-charge protection at all.

**Active pre-charge circuits**: mjbots sells the [mjpower-ss](https://mjbots.com/products/mjpower-ss) and [power_dist](https://mjbots.com/products/mjbots-power-dist-r4-5b), both of which have the primary function of gradually precharging capacitive loads.  Other active pre-charge systems may also be viable.

## Regenerative braking safety

moteus can be commanded to sharply decelerate loads, either directly in response to commands, or due to external disturbances.  When braking a load, moteus by default applies the generated power to the input DC bus.

If there is nowhere for this power to go, this can cause problems.  The voltage can increase without bound, which in mild cases will cause the CAN transceiver on all devices connected to the bus to fail, and in severe cases can explode the main FETs or other components on the board.

Here's what you should know about the facilities moteus has to deal with this, and what you can do to make your design safer.

### Flux braking

The feature within moteus itself to deal with this is "flux braking".  The flux braking implementation will dissipate extra power in the windings of the motor when the bus voltage gets above a certain threshold.  This is controlled by the `servo.flux_brake_margin_voltage` and `servo.flux_brake_resistance_ohm` parameters documented above.

### Design considerations for regenerative braking

The following design considerations can be used to minimize the risk of damage to hardware in the event of overvoltage.  These are not a substitute for validation in progressively more demanding situations, but they can help you start off in a good place.

- *Tightly scope the over-voltage fault / flux braking*: The configuration parameter `servo.max_voltage` can be lowered for all devices on the bus.  This will both cause a fault if the voltage exceeds this value and in conjuction with `servo.flux_brake_margin_voltage`, select the point at which moteus will attempt to dissipate energy to prevent an overvoltage scenario.  It is recommended to set this to no less than 5V above the maximum expected supply voltage.

- *Power from a battery, not a PSU*: When not charged, batteries are capable of sinking current to minimize over-voltage transients.  However, if the battery is fully charged, most battery management systems drastically reduce the allowable charging current.  Thus, a battery is only useful as a mitigation if it is never charged above say 75 or 80% state of charge.

- *Decrease overall system voltage*: If you run the moteus controller with say a 10S battery, the peak input voltage can be as high as 42V.  That does not leave very much margin for regenerative loads.  For applications that experience sharp regenerative loads and do not have a battery capable of charging always attached, it is recommended not to exceed 8S (33.6V peak).

- *Use a supply which can sink as well as source*: Powering from an inexpensive lab supply is the most dangerous, as they typically have no ability to sink current, only source it.  A "two quadrant" supply is the necessary device.


## Next Steps

After completing electrical connections, proceed to [software installation and configuration](software-installation.md) to begin using your moteus controller.
