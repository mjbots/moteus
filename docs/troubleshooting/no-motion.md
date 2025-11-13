# Troubleshooting No Motion

You sent a command to moteus, or ran a script, but nothing moved.  This article will describe what steps you can take to diagnose and resolve the issue.

## Using tview

If you sent a `d pos` command from tview, there are a few possibilities.

**Invalid Syntax**: If the diagnostic channel responds with an OK, then this is not the problem.  If it responds otherwise, then the error message will give you a clue as to the problem.  The syntax for the `d pos` command can be found in the [diagnostic protocol reference manual]( http://localhost:8000/mjbots/moteus/protocol/diagnostic/#d-pos).

**Position Mode Fault**: If the diagnostic channel responded OK, then to identify the problem you must look at the current reported mode and fault status.  They will be present as the first two items in the `servo_stats` telemetry channel.  Common problems when first attempting control are:

 - "39 (outside bounds)": `servopos.position_min` and `servopos.position_max` are either not configured, or the motor starting position is outside the configured bounds.  Setting either to `nan` will disable it.
 - "33 (gate driver fault)": this indicates that the onboard MOSFET gate driver reported a fault.  To know which one, you will need to expand the `drv8323` telemetry channel.  The only "easy" problem to solve is `uvlo` which indicates that moteus attempted to draw more power than your supply could provide.

Once the cause of the fault has been identified, it can be cleared by sending:

`d stop`

Then the command in question can be attempted again.

## From a script or application

If you have a python script, C++ application, or other tool which is commanding moteus and it does not move, there are a few tools you can use.

**Request and display the mode and fault**: If your application does not already request and display the mode and fault it should do so.

**Log CAN-FD data**: If the application cannot be modified or it is difficult to do so, you can either (a) if it is using the python library use the `--can-debug OUT.LOG` command flag to save a record of all frames sent and received or (b) attach a second CAN-FD adapter to the bus and use it to log data.

Once you have data, it can be examined using `utils/decode_can_frame.py` to see what moteus is reporting.

Problems may be the same as from the "Using tview" section above.

Another common problem with applications using the register protocol (what the python and C++ library does), is watchdog timeouts.  For the register protocol, by default, moteus requires commands arrive at least every 100ms or it will enter mode 11, the timeout mode.  Once in this mode, a "stop" command must be sent to clear it.

The watchdog timer configuration is described in [the reference manual](../reference/configuration.md#servodefault_timeout_s).
