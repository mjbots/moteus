Hypothetical python sessions.

```

# This would connect to a single controller via some appropriate mechanism,
# (likely an attached fdcanusb or a pi3hat).

import asyncio

import moteus

async def main():
    controller = moteus.Controller()  # default id of 1

    while True:
        print(await controller.set_position(position = 0.1 * math.sin(time.time()), query=True))

asyncio.run(main())
```


```
# An all-options used variant.
import asyncio

import moteus

async def main():
    fdcanusb1 = moteus.Fdcanusb(path='/dev/fdcanusb')
    fdcanusb2 = moteus.Fdcanusb(path='/dev/ttyACM1')

    router = moteus.Router([(fdcanusb1, [1, 2]),
                            (fdcanusb2, [3, 5])])

    c1 = moteus.Controller(id=1)  # Will use a default router if any action calls are made
    c2 = moteus.Controller(id=2,
                           query_resolution={},
                           position_resolution={},
                           router=router)

    while True:
        data = await router.cycle(
          c1.make_position(position = 0.1 * math.sin(time.time()), query=True),
          c2.make_position(position = 0.1 * math.cos(time.time()), query={}),
        )
        print("1: {}, 2: {}", data[1].position, data[2].position)

        # This also works, and invokes a separate transmission cycle.
        await c2.set_position(position = 0.1 * math.cos(time.time()))
```
