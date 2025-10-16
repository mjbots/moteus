# Python Client Library

moteus provides a python library that can be used to command and control moteus controllers using a supported CAN-FD adapter.

## Installation

To install, just use:

```
pip install moteus
```

Or add to your `requirements.txt` file, or via whatever mechanism your project uses.

## Basic Usage

There are two basic approaches to using the python library.  The first is the easiest, although does not provide optimal bus utilization if that is a design requirement.

First, construct one or more controller instances:

```python
import asyncio
import moteus

async def main():
    c1 = moteus.Controller(id=1)
    c2 = moteus.Controller(id=2)
    # ...

if __name__ == '__main__':
    asyncio.run(main())
```

Then, at regular intervals, send commands to each of the devices using the `set_` variant of each API.

```python
while True:
  c1_result = await c1.set_position(
      position=math.nan, velocity=1.0, accel_limit=0.5, query=True)
  c2_result = await c2.set_position(
      position=math.nan, velocity=0.5, accel_limit=0.25, query=True)

  c1_position = c1_result.values[moteus.Register.POSITION]
  c2_position = c2_result.values[moteus.Register.POSITION]

  print(c1_position, c2_position)

  await asyncio.sleep(0.01)
```


## `.cycle` based usage

If bus utilization or a high update rate is desired, an alternate API can be used to maximize performance.  In this API, the commands are constructed in advance using `make_` variants of commands, then submitted to the library in a group.

```python
import argparse
import asyncio
import moteus

async def main():
    parser = argparse.ArgumentParser()
    moteus.make_transport_args(parser)
    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)
    c1 = moteus.Controller(id=1, transport=transport)
    c2 = moteus.Controller(id=2, transport=transport)
    # ...

    while True:
        results = await transport.cycle([
           c1.make_position(position=math.nan, query=True)
           c2.make_position(position=math.nan, query=True),
           ])

        # Print the ID and position of all received responses.
        print(", ".join(
            f"{x.source} " +
            f"{x.values[moteus.Register.POSITION]}"
            for x in results))

        await asyncio.sleep(0.01)

if __name__ == '__main__':
    asyncio.run(main())
```

## Querying alternate registers

By default, only a limited set of registers are queried.  The full set of available registers can be found in the [register reference documentation](../protocol/registers.md).  The easiest way to query additional registers is to pass an alternate `QueryResolution` structure to the `Controller` constructor.

```python
qr = moteus.QueryResolution()
qr.power = moteus.F32
c = moteus.Controller(id=1, query_resolution=qr)
```

If the register you want is not available in the `QueryResolution` structure, it can be requested using the `_extra` method:

```python
qr = moteus.QueryResolution()
qr._extra = {
    moteus.Register.ENCODER_1_POSITION: moteus.F32,
    moteus.Register.ENCODER_1_VELOCITY: moteus.F32,
}
c = moteus.Controller(id=1, query_resolution=qr)
```

## API Reference

- [Python API Reference](../reference/python.md)

## Related Sections

- [Control Modes](../guides/control-modes.md) - Understanding different control strategies
- [Configuration Parameters](../reference/configuration.md) - Available parameters
- [Raspberry Pi Setup](../platforms/raspberry-pi.md) - Platform-specific configuration
