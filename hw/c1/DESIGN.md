moteus-c1 is a small form factor controller with the flexibility of
the moteus-n1, but designed for lower power and current applications.

# r1.1 fixes #

 * U2 3.3V regulator has enable pin undriven
 * Switched U2 to part TLV70233PDBVR which is SOT23-5 and has better
   thermal properties
 * removed gate drive resistors and discharge diodes
 * moved C7 away from AUX1/AUX2 pins
 * switch can terminator (R6) to 0603 and move further away from power
   pad
 * rev silk version to r1.1
 * version divider pulled low
 * revved TVS to the 48V version, so 3V lower than before... that will
   make the rated max voltage be 51V instead of 54V
 * added unpopulated resistor and capacitor between power ground and a
   mounting hole

# TODO #

# r1.0 ERRATA #
 * C7 blocks access to the through hole pins (they barely fit, but
   could use a bit more margin)
 * can termination resistor pad is too close to power pin of XT30
 * high side FET trace to DRV is too small compared to gate drive
   trace and should be swapped around to be a pair with gate drive
 * board version divider should never be floating, needs to be
   pulled low for next rev
 * remove gate drive resistors and reverse diodes
 * 3.3V regulator should be the SOT-23 package that has better
   thermals
 * need to fully lay out what the family pullup/pulldowns are so
   that I can be sure they can all be distinguished


r1.1 MAYBE
 * possible remove all the gate drive resistors and protection diodes?
 * maybe lower max voltage to 51V and drop the TVS diode by one step
