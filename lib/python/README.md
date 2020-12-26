# Python bindings for moteus brushless controller #

These bindings permit communication and control of moteus brushless
controllers.

## To use ##

```
pip3 install moteus  # or 'moteus-pi3hat' on a pi3hat
```

The following will report the status of a single controller at the
default address of 1 at 1Hz while commanding it to hold the current
position.

```
import asyncio
import math
import moteus

async def main():
    c = moteus.Controller()
    await c.set_stop()  # in case there was a fault

    while True:
        print(await c.set_position(position=math.nan, query=True))
        await asyncio.sleep(0.02)

asyncio.run(main())
```

## Theory ##

Interactions with a controller are mediated through the
`moteus.Controller` object.  When constructed with the constructor
argument `transport=None` (the default) it attempts to find some
suitable link on your host system, typically the first fdcanusb or
socketcan bus it locates.

Single controller imperative operation can be conducted by using
`await Controller.set_stop()`, `await Controller.set_position()`, and
`await Controller.query()`.

## Bus-optimized usage ##

To optimize bus usage, it is possible to command multiple controllers
simultaneously.  In this mode, a "transport" must be manually
constructed.

```
import asyncio
import math
import moteus

async def main():
    transport = moteus.Fdcanusb()
    c1 = moteus.Controller(id = 1)
    c2 = moteus.Controller(id = 2)

    while True:
        print(await transport.cycle([
          c1.make_position(position=math.nan, query=True),
          c2.make_position(position=math.nan, query=True),
        ]))

asyncio.run(main())
```

All of the "set_" methods have a "make_" variant which is suitable to
pass to a Transport's `cycle` method.

This mechanism only improves performance for non-fdcanusb links, such
as a pi3hat.

## Position mode commands ##

`Controller.set_position` and `Controller.make_position` have
arguments which exactly mirror the fields documented in
`docs/reference.md`.  Omitting them (or specifying None), results in
them being omitted from the resulting register based command.

* position
* velocity
* feedforward_torque
* kp_scale
* maximum_torque
* stop_position
* watchdog_timeout

Finally, the `query` argument controls whether information is queried
from the controller or not.

## Controlling resolution ##

The resolution of commands, and of returned query data, is controlled
by optional constructor arguments to `Controller`.  By default, the
commands are all F32, and the query requests a subset of fields as
INT16.  Here is an example of setting those.

```
pr = moteus.PositionResolution()
pr.position = moteus.INT16
pr.velocity = moteus.INT16
pr.kp_scale = moteus.F32
pr.kd_scale = moteus.F32

qr = moteus.QueryResolution()
qr.mode = mp.INT8
qr.position = mp.F32
qr.velocity = mp.F32
qr.torque = mp.F32

c = moteus.Controller(position_resolution=pr, query_resolution=qr)
```
