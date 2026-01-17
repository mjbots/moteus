# Troubleshooting Overvoltage Fault 34

moteus brushless motor controllers both apply power to motors and, depending upon the mode and external system state, can act as a generator, often termed regenerative braking.  When acting as a generator energy is emitted to the input DC bus -- this causes the voltage to rise at a rate depending upon the characteristics of the supply.  If the voltage rises far enough and for a sufficient time to trigger it, [flux braking](../guides/electrical-setup.md#flux-braking) may cause some energy to be expended in the windings of the motor.  If the regenerative braking energy increases rapidly, or is larger than the configured flux braking can dissipate, the voltage can rise above the configured fault voltage in `servo.max_voltage`, which will trigger a fault.

If you are seeing fault 34 in your application, there are several largely independent methods that can be attempted to resolve the problem.

## Configuring acceleration limits

For flux braking to operate, the rate at which regenerative power increases must be limited.  The easiest way to do so is to use acceleration limits that are as low as your application can tolerate.  The default acceleration limit can be set with the configurable parameter:

```
servo.default_accel_limit
```

Or the limit can be sent with individual position mode commands.

If you don't know what acceleration is appropriate, you can either:

- use back of the envelope calculations of the form, "I know the system needs to change from velocity X Hz to velocity Y Hz in Z s, thus the acceleration is (Y - X) / Z"
- select a large acceleration limit that is unlikely to cause problems, say 10000Hz/s.  Then lower it progressively until your application has issues.  Select a final value a healthy margin above where the issues started.


## Lower the impedance of your input supply to regenerative power

By lowering the impedance of your input supply, you decrease the rate at which the voltage rises for a given amount of regenerative energy.  There are several approaches to this.

### Increase battery current rating

If you are using batteries that are capable of charging as your power supply, you can select batteries that have a higher charging current rating.

### Add bus capacitance

If you have a mains powered supply which is incapable of accepting regenerative energy, or it is not desirable to change the battery, you can add additional capacitance to the DC input bus.  There are several considerations when doing so:

- **Inrush current management**: Per [the electrical setup guide](../guides/electrical-setup.md#inrush-current-management), whatever inrush current management solution you are using must tolerate the additional capacitance.  If you are using a mjbots power_dist or mjpower-ss, the total system capacitance must be no more than 4000uF.  mains powered supplies may be able to handle much larger bus capacitances, although their reference documentation should be checked to validate that.  In general, the larger the capacitance the better for resolving over-voltage faults, presuming you stay within the limits of your inrush current solution.

- **Capacitor rating**: The selected capacitors should have a voltage rating sufficient for the maximum voltage the bus will expect with a design margin appropriate to your application.  Additionally, the ESR (equivalent series resistance), should be low enough to control the magnitude of the voltage rise.

### Switch to a battery

If you are using a mains powered supply, it may be possible to switch to a battery solution where the battery can be charged.  When doing so, it is important to ensure that inrush current is managed appropriately.

## Configuration changes

If the above solutions were unable to resolve the problem, or the system performance is still not at the desired level, there are a few other parameters which can help some, although not to the same degree.

### `servo.flux_brake_margin_voltage`

Flux braking works by monitoring a filtered version of the system voltage and dissipating more energy in the windings as this increases.  By increasing the range of voltages where this process occurs, the system will have more time to respond as the voltage rises.  The threshold points that are relevant are:

- `servo.max_voltage - servo.flux_brake_margin_voltage`: This voltage is the point where flux braking begins.
- `servo.max_voltage`: When the voltage reaches this point, the over-voltage fault will trigger.

You can increase `servo.flux_brake_margin_voltage` until the trigger point is maybe 1V above your nominal system voltage or fully charged voltage if on a battery.  You can also increase `servo.max_voltage` and `servo.flux_brake_margin_voltage` at the same time to give even more room.

### `servo.flux_brake_resistance`

This constant determines how much energy is dissipated in the windigs for a given amount of voltage rise.  Lower values are more aggressive and will dissipate more energy, but can be less stable.  Unless you have a good mechanism for verifying stability, it is not recommended to lower this more than a factor of 2x from the default.

### `servo.pid_dq.max_desired_rate`

This configuration value selects the maximum rate at which desired current can change.  By default, it is relatively slow at 10000A/s.  However, if you have raised it previously, lowering it back down can help reduce the rate at which regenerative power is increased.  If your application can tolerate it, lowering further will increase the effectiveness of flux braking.
