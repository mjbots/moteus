#!/usr/bin/python3 -B

# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
This example commands a single servo at ID #1 using the default
transport to hold the current position indefinitely, and prints the
state of the servo to the console.
"""

import asyncio
import moteus

async def main():
    # Communicate with controller ID 1 on the default transport.
    c = moteus.Controller()

    # This returns the GPIO digital inputs as an array of bytes, one
    # for each auxiliary port.
    gpio_inputs = await c.read_gpio()

    # Expand these as individal pin values for display purposes.
    def display(aux_num = 1, value = None, count = None):
        print(f"AUX{aux_num}")
        for i in range(count):
            print(f"  Pin {i} - {value & (1 << i)}")

    display(1, gpio_inputs[0], 5)
    print()
    display(2, gpio_inputs[1], 4)


    # And to write the values to gpio pins, the
    # `(make|set)_write_gpio` function can be used.  Here, we'll set
    # all possible GPIO digital outputs to 1.
    await c.set_write_gpio(aux1=0x7f, aux2=0x7f)


    # Finally, GPIO values can be included in general query results.
    qr = moteus.QueryResolution()
    qr.aux1_gpio = moteus.INT8
    qr.aux2_gpio = moteus.INT8
    c2 = moteus.Controller(query_resolution=qr)
    result = await c2.query()

    print()
    print("From moteus.Controller.query()")
    display(1, result.values[moteus.Register.AUX1_GPIO_STATUS], 5)
    print()
    display(2, result.values[moteus.Register.AUX2_GPIO_STATUS], 4)


if __name__ == '__main__':
    asyncio.run(main())
