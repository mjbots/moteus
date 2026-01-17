# Calibration

If you started from a bare moteus board, you will need to calibrate it for the attached motor before any control modes are possible. Calibration is required when:

- Using a new bare moteus controller for the first time
- Changing to a different motor
- Altering the mechanical registration between the controller and motor
- After certain configuration changes that affect motor characteristics

Development kits typically come pre-calibrated and do not require this step unless the motor is changed.

## Calibration Command

To calibrate your moteus controller, run the following command:

```bash
python3 -m moteus.moteus_tool --target 1 --calibrate
```

Replace `1` with the appropriate CAN-FD ID if you have configured your controller to use a different ID.

## Important Safety Warning

**WARNING: Any attached motor must be able to spin freely during calibration.**

During the calibration process:

- The motor will be spun in both directions
- The motor will reach high speeds
- Ensure no mechanical obstructions are present
- Do not apply any load to the motor shaft
- Keep hands and objects clear of the motor and any attached mechanisms

The calibration procedure measures motor electrical characteristics and encoder alignment, which requires unrestricted motor motion.

## Alternate calibration parameters

### Limited rotation

The amount of motion required for calibration can be reduced if the Kv of the motor is known in advance.  In that case, specify as an additional command line argument:

`--cal-force-kv X`

Where X is the Kv of the motor.  In this mode, moteus will only spin the rotor 360 degrees in each direction and will not perform a high speed movement.

### Very small or very large motors

During the encoder calibration process, moteus applies a constant power to the motor.  The default power, 7.5W is sufficient for many motors but not all.  If the motor is very small, it could overheat or burn if applying that much power.  If the motor is very large, 7.5W may not be enough to cause the motor to move.  This can be altered by specifying:

`--cal-motor-power X`

Where X is the power in watts to apply.  Small motors will need smaller values, large motors may need larger ones.

### Updating encoder and torque bandwidth

If your application requires changing torque or velocity very quickly there are several additional considerations.

**Torque bandwidth**: You may want to configure an alternate "bandwidth" parameter during the calibration process.  The specific value of bandwidth can be roughly used to estimate how quickly moteus can change the output torque, and how quickly it can sense velocity and position changes.  For instance, the default bandwidth is 200Hz.  This means that as a rough order of magnitude, moteus can change the torque or observe a change in velocity in 1/200 = 5ms.  Higher bandwidth values will result in the controller and motor emitting more audible noise.  Other than audible noise (which can be significant), there is not much downside to increasing the bandwidth.

`--cal-bw-hz X`

**`servo.pid_dq.max_desired_rate`**: moteus limits the rate at which the commanded phase current to the motor can change.  By default, this is set to 10000A/s.  For a moteus-r4 with a 100A current limit, that means it can take 20ms to change full range from -100A to 100A.  Many high bandwidth applications will require rates larger than this.

## Troubleshooting

If you encounter issues during calibration, such as:

- Motor not spinning
- Calibration failures or errors
- Unexpected motor behavior

Refer to the [Troubleshooting Guide](../troubleshooting/calibration.md) for common problems and solutions.

## Next Steps

After successful calibration, you can conduct the PID tuning process:

- [PID Tuning](../guides/pid-tuning.md)
