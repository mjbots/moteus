# socketcan

Many CAN-FD adapters can be used with the Linux socketcan subsystem.  This section describes how to configure them for use with moteus:

## Bit Timings

To communicate with moteus, both the bitrate *AND* sample point *AND* sjw/dsjw must be configured for the socketcan device.  The following command will work with most devices:

```
ip link set can0 up type can \
  bitrate 1000000 dbitrate 5000000 \
  sjw 10 dsjw 5 \
  sample-point 0.666 dsample-point 0.666 \
  restart-ms 1000 fd on
```

Some devices will give an error if the `sjw` or `dsjw` options are too large.  In that case, select the largest possible `sjw` or `dsjw` option.

## Incompatible adapters

CAN-FD adapters that use the slcan protocol, or that are derived from canable hardware whether or not they use the slcan or candelight firmware, are known to be currently incompatible with moteus controllers.
