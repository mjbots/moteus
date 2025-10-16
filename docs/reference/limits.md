# Application Limits

## Position

The reported and commanded position is limited to +-32768.0
revolutions

If either:

a. the position is commanded as the special value (NaN or maximally negative), or
b. the kp term is zero either through configuration or the "kp scale"

Then it is safe for the controller to "wrap around" from the maximal
possible position to the maximally negative position and vice versa.
This is useful in velocity control applications.

The reported and commanded position is internally treated as a 32 bit
floating point value when received as a command or reported to a
client.  Thus the position resolution is reduced when the magnitude of
the position is large.  Resolution equal to the full capabilities of
the onboard encoder (~0.09degree) is maintained to positions of
+-2048.0 revolutions.  At the maximum possible position, this
resolution is reduced to ~1.44degrees.  Note, that this is only for
values received from or reported to clients.  Internally, control and
PID feedback on the position works in an integral space and performs
identically throughout the available control envelope.

## Velocity

The smallest usable mechanical velocity which can be commanded is
0.0001 revolutions per second.  This corresponds to 0.036 degrees per
second.

The maximum mechanical velocity which can be commanded is 28000 rpm,
or ~467 revolutions per second before any reducers.  Note, most motors
will be incapable of this speed either mechanically or electrically.

The maximum electrical frequency is 4kHz.
