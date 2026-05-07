# Optimizing for maximum duty cycle

## Background

In some cases, it is important that moteus be able to achieve the maximum possible speed on a given motor without resorting to field weakening.  The controller is limited in how much of the supply voltage it can apply to the motor.  By definition the multiple that it can supply is less than 1.  This multiple is constrained by moteus's requirement to be able to sense the current flowing to the motor in order to perform field oriented control. Two factors bound this constraint:

1. The current can only be sensed when the "low" side MOSFET is switched on.  Thus, the low-side MOSFET must be on long enough for ADC to trigger and complete its sampling process.
2. The input to the ADC is a current sense amplifier (CSA) onboad moteus.  This amplifier has a gain dependent settling time that must be respected.  This settling time must complete *before* the ADC sampling process starts.

## Tuning parameters

To maximize the available duty cycle, there are two parameters that can be controlled.

1. `servo.pwm_rate_hz`: The lower the PWM rate, the higher the available duty cycle.  Setting it to the minimum of 15000 Hz is the limit.  Lower PWM rates decrease the control update rate and decrease the peak power the board can support.

2. `drv8323.csa_gain`: The current sense amplifier gain, which can be one of [5, 10, 20, 40].  The default is 20.  Lowering this to 10 gives the maximum duty cycle benefit at the expense of increased current sense noise.  This will result in more audible noise from the system and lower available torque control bandwidth.
