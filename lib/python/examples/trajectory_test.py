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

import asyncio
import moteus
import time


'''Demonstrates how to specify alternate registers to query, and how
to control the velocity and acceleration limits on a per-command basis
to create a continouous trajectory.'''


async def main():
    qr = moteus.QueryResolution()
    qr._extra = {
        moteus.Register.CONTROL_POSITION : moteus.F32,
        moteus.Register.CONTROL_VELOCITY : moteus.F32,
        moteus.Register.CONTROL_TORQUE : moteus.F32,
        moteus.Register.POSITION_ERROR : moteus.F32,
        moteus.Register.VELOCITY_ERROR : moteus.F32,
        moteus.Register.TORQUE_ERROR : moteus.F32,
        }

    c = moteus.Controller(query_resolution = qr)

    # Clear any faults.
    await c.set_stop()

    while True:
        # Our command position will alternate between 0.5 and -0.5
        # every second.
        current_command = 0.5 if (round(time.time()) % 2) else -0.5

        # The acceleration and velocity limit could be configured as
        # `servo.default_accel_limit` and
        # `servo.default_velocity_limit`.  We will override those
        # configurations here on a per-command basis to ensure that
        # the limits are always used regardless of config.
        results = await c.set_position(
            position=current_command,
            velocity=0.0,
            accel_limit=2.0,
            velocity_limit=3.0,
            query=True,
        )

        print(results)

        await asyncio.sleep(0.02)


if __name__ == '__main__':
    asyncio.run(main())
