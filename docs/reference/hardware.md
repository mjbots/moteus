# Hardware Specifications

## CAD and Models

| Name      | 2D CAD    | 3D Model   |
|-----------|-----------|------------|
| moteus-c1 |[2D CAD](https://github.com/mjbots/moteus/blob/0.1-20240430/hw/c1/r1.2/20240305-moteus-c1-r1_2.pdf) | [STEP](https://github.com/mjbots/moteus/blob/0.1-20240430/hw/c1/r1.2/20240305-moteus-c1-r1_2.step) |
| moteus-r4 |[2D CAD](https://github.com/mjbots/moteus/blob/main/hw/controller/r4.5/20210124-moteus-controller-r45-mechanical.pdf) | [STEP](https://github.com/mjbots/moteus/blob/main/hw/controller/r4.5/20210124-moteus-controller-r45-mechanical.step) |
| moteus-x1 | [2D CAD](https://drive.google.com/file/d/1R7wuc7vk1khD5ZvDWPy54Bx4PgHYiUCu/view) | [STEP](https://drive.google.com/file/d/19tJa4gzy0ZBWYYsaJA-Du0PgzoDWizjL/view?usp=sharing) |
| moteus-n1 | [2D CAD](https://drive.google.com/file/d/1Ic65vT8BSeTtqz6uqd8C7l68aOMy3bMz/view?usp=share_link) | [STEP](https://drive.google.com/file/d/1wxX5G6kX6M6YZGGSceiXTOUZycxRaSIE/view?usp=share_link) |

## Electrical Limits

| Name | Min Voltage | Nominal | Abs Max Voltage | Peak Output | 3V Aux |  5V Aux | 12V Aux |
|------|-------------|---------|-----------------|-------------|--------|---------|---------|
| moteus-c1 | 10V | 24V | 51V (<= 12S) | 20A | 50mA | 100mA | N/A |
| moteus-r4 | 10V | 24V | 44V (<= 10S) | 100A | 100mA | N/A | N/A |
| moteus-x1 | 10V | 24V | 54V (<= 12S) | 120A | 100mA | 200mA | 150mA |
| moteus-n1 | 10V | 24V | 54V (<= 12S) | 100A | 100mA | 200mA | N/A |

## Power Limits

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
