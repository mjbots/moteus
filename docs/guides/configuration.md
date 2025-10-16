# Configuration

Before using a moteus controller for the first time, it is necessary to configure several key parameters. These parameters define operational limits, control behavior, and configure the system to match your specific application.

## Essential Configuration Parameters

The following parameters should be reviewed and will likely need to configured in a customized way for nearly every application.

### Position Limits

**`servopos.position_min` and `servopos.position_max`**

These define the bounds of motion which the controller will allow when in position control mode. Attempting to start beyond this region will fault, and if outside the region during operation, no torque will be applied to move further outside these limits.  If they are unwanted, `nan` should be used to disable them.

### Current Limits

**`servo.max_current_A`**

The maximum phase current to apply to the motor. This can be used to limit the maximum torque that the system is capable of producing, regardless of any command sent. Set this based on your motor's peak current rating and thermal capabilities.

### Velocity Limits

**`servo.max_velocity`**

Limits the maximum speed the motor is permitted to achieve before no torque is produced. This acts as a safety limit to prevent excessive speeds that could damage the motor or mechanical system.

**`servo.default_velocity_limit` / `servo.default_accel_limit`**

Controls how fast the motor can accelerate and spin in order to reach position and velocity targets. Bare boards ship with these unset, while development kits ship with human-eye pleasing values. Adjust these to match your application's performance requirements.

### Gearbox Scaling

**`motor_position.rotor_to_output_ratio`**

A gearbox scaling factor. A reducing gearbox should be configured with a number smaller than one (e.g., 0.25 for a 4x reduction gearbox). This affects reported position, speed, and torques, scaling them to represent the output shaft values rather than motor shaft values.

### Device Identification

**`id.id`**

The CAN-FD ID used by this device. Each controller on the bus must have a unique ID. Valid IDs range from 1 to 126.  If you set this to a value other than 1, be aware that unless your tview and controller firmware are newer than late 2025, you will need to restart tview with:

```
python -m moteus_gui.tview -t N
```

Where N is the new ID, in order to issue a `conf write` to save the
parameters.

### PID Parameters

**`servo.pid_position`**

The PID parameters for the position control loop. It will be necessary to tune these, it isn't hard, although it must be done after calibration is performed. You'll get to the [PID Tuning guide](../guides/pid-tuning.md) after completing the calibration process.

## Saving parameters

All parameters (including the ID) are temporarily stored in volatile memory that will be discarded after a power cycle.  To save those parameters to persistent storage, it is necessary to type:

```
conf write
```

Into the diagnostic console of tview.

## Additional Configuration

A larger set of parameters is documented in the [Configuration Parameters Reference](../reference/configuration.md), covering advanced features such as:

- Motor electrical characteristics
- Encoder configuration
- Temperature limits
- Communication settings
- Advanced control parameters

## Next Steps

After configuring these essential parameters, you should familiarize yourself with the encoder system and position feedback:

- [Encoder Overview](encoder-overview.md)
