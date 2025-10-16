# Hardware Specifications

## Mechanical

The current mechanical drawing for the controller can be found at:
[20210124-moteus-controller-r45-mechanical.pdf](https://github.com/mjbots/moteus/blob/main/hw/controller/r4.5/20210124-moteus-controller-r45-mechanical.pdf)

The current mechanical drawing for the qdd100 servo can be found at:
[20200315-qdd100-mechanical.pdf](https://drive.google.com/file/d/1KUQyR853e2uw8WOVrQHaeskWYHGSm3nI/view?usp=sharing)

## Power

The allowable maximum power for each moteus controller depends upon
the input voltage and PWM switching frequency.  The below table gives
the maximum allowable power at `servo.pwm_rate_hz=30000`.

| Name       | Peak power   |                | High input power |
|------------|--------------|----------------|------------------|
| moteus-r4  | <= 30V 900W  | linear derated | >= 38V 400W      |
| moteus-c1  | <= 28V 250W  | linear derated | >= 41V 150W      |
| moteus-n1  | <= 36V 2000W | linear derated | >= 44V 1000W     |

For other values of `servo.pwm_rate_hz`, the allowable maximum power
changes linearly with the PWM rate, so that at 15000, the maximum
power is half of that in the above table and at 60000 it is double
that.  Note however, that efficiency of the controller goes down
significantly at higher PWM rates.

The current power limit is reported in `servo_stats.max_power_W`.  The
controller will attempt to limit output phase current so as to stay
within this reported power limit in either direction, i.e. applying
power or regenerating energy.
