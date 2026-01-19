#!/usr/bin/python3 -B

# Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

"""This example demonstrates using moteus.move_to() to move multiple
servos to target positions and wait for completion.

Two servos alternate between positions with a 5 second period:
  - Servo 1: alternates between 0 and 1 revolution
  - Servo 2: alternates between 0 and 3 revolutions
"""

import argparse
import asyncio
import moteus


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--id1', type=int, default=1,
        help='ID of first servo (default: 1)')
    parser.add_argument(
        '--id2', type=int, default=2,
        help='ID of second servo (default: 2)')
    parser.add_argument(
        '--period', type=float, default=5.0,
        help='Period in seconds for full cycle (default: 5.0)')
    moteus.make_transport_args(parser)
    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)

    c1 = moteus.Controller(id=args.id1, transport=transport)
    c2 = moteus.Controller(id=args.id2, transport=transport)

    # Clear any faults by sending stop commands.
    await transport.cycle([c1.make_stop(), c2.make_stop()])

    # Define the two positions for each servo.
    positions_a = {c1: 0.0, c2: 0.0}
    positions_b = {c1: 1.0, c2: 3.0}

    # Each move should take half the period.
    move_duration = args.period / 2.0

    print(f"Servo {args.id1}: alternating between 0 and 1 rev")
    print(f"Servo {args.id2}: alternating between 0 and 3 rev")
    print(f"Period: {args.period}s (each move: {move_duration}s)")
    print("Press Ctrl+C to stop")
    print()

    cycle = 0
    try:
        while True:
            # Move to position A
            print(f"Cycle {cycle}: Moving to position A "
                  f"(c1={positions_a[c1]}, c2={positions_a[c2]})")
            results = await moteus.move_to([
                (c1, positions_a[c1]),
                (c2, positions_a[c2]),
            ], duration=move_duration)

            for controller, result in results:
                pos = result.values[moteus.Register.POSITION]
                print(f"  Servo {controller.id} arrived at {pos:.3f}")

            # Move to position B
            print(f"Cycle {cycle}: Moving to position B "
                  f"(c1={positions_b[c1]}, c2={positions_b[c2]})")
            results = await moteus.move_to([
                (c1, positions_b[c1]),
                (c2, positions_b[c2]),
            ], duration=move_duration)

            for controller, result in results:
                pos = result.values[moteus.Register.POSITION]
                print(f"  Servo {controller.id} arrived at {pos:.3f}")

            cycle += 1

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        await transport.cycle([c1.make_stop(), c2.make_stop()])
        print("Servos stopped.")


if __name__ == '__main__':
    asyncio.run(main())
