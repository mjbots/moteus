# Python bindings for moteus brushless controller #

This bindings permit communication and control of moteus brushless
controllers.

## To use ##

```
pip install moteus
```

The following will report the status of a single controller at the
default address of 1 at 1Hz.

```
import asyncio
import moteus

async def main():
    c = moteus.Controller()
    while True:
        print(await c.query())
        await asyncio.sleep(1.0)

asyncio.run(main())
```

## Theory ##

Interactions with a controller are mediated through the
`moteus.Controller` object.  When constructed with the constructor
argument `router=None` (the default) it attempts to find some suitable
link on your host system, typically the first fdcanusb it locates.

Single controller imperative operation can be conducted by using
`await Controller.set_stop()`, `await Controller.set_position()`, (and
optionally `await Controller.query()`.

## Bus-optimized usage ##

To optimize bus usage, it is possible to command multiple controllers
simultaneously.  In this mode, a "router" must be manually
constructed.

```
import asyncio
import moteus

async def main():
    router = moteus.Fdcanusb()
    c1 = moteus.Controller(id = 1)
    c2 = moteus.Controller(id = 2)

    while True:
        print(await router.cycle([
          c1.make_query(),
          c2.make_query(),
        ]))

asyncio.run(main())
```

All of the "set_" methods have a "make_" variant which is suitable to
pass to a Router's `cycle` method.

This mechanism only improves performance for non-fdcanusb links, such
as a pi3hat.

## Position mode commands ##

`Controller.set_position` and `Controller.make_position` have
arguments which exactly mirror the fields documented in
`docs/reference.md`.  Omitting them (or specifying None), results in
them being omitted from the resulting register based command.
