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

"""Synchronized multi-joint motion using ruckig for trajectory generation.

This example demonstrates true synchronized multi-joint motion using the
ruckig library for jerk-limited trajectory generation. All joints arrive
at their targets simultaneously with smooth, coordinated motion profiles.

Each servo moves a different distance to demonstrate time synchronization:
  - Servo 1: moves 0.25 rev (1x scale)
  - Servo 2: moves 0.50 rev (2x scale)
  - Servo 3: moves 0.75 rev (3x scale)
  - etc.

Despite the different distances, ruckig ensures all servos start and finish
at the same time by computing appropriate velocity/acceleration profiles.

This is the recommended approach when you need:
- Smooth, jerk-limited motion profiles
- Precise synchronization across multiple joints
- Complex coordinated motion paths

Requirements:
    pip install ruckig

Usage:
    python3 ruckig_multiservo.py -t 1,2
    python3 ruckig_multiservo.py --targets=1,2,3 --max-velocity=0.5 --max-accel=2.0
"""

import argparse
import asyncio
import time

import moteus

try:
    from ruckig import Ruckig, InputParameter, Trajectory, Result
except ImportError:
    print("This example requires the ruckig library.")
    print("Install with: pip install ruckig")
    raise SystemExit(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Synchronized multi-joint motion using ruckig')
    parser.add_argument(
        '-t', '--targets', type=str, default='1,2',
        help='Comma-separated list of servo IDs (default: 1,2)')
    parser.add_argument(
        '--max-velocity', type=float, default=0.5,
        help='Maximum velocity in rev/s (default: 0.5)')
    parser.add_argument(
        '--max-accel', type=float, default=2.0,
        help='Maximum acceleration in rev/s^2 (default: 2.0)')
    parser.add_argument(
        '--max-jerk', type=float, default=10.0,
        help='Maximum jerk in rev/s^3 (default: 10.0)')
    parser.add_argument(
        '--cycle-time', type=float, default=0.01,
        help='Cycle time in seconds (default: 0.01)')
    parser.add_argument(
        '--base-amplitude', type=float, default=0.25,
        help='Base motion amplitude in revolutions (default: 0.25)')
    moteus.make_transport_args(parser)
    return parser.parse_args()


class SynchronizedMotion:
    """Coordinates synchronized motion of multiple servos using ruckig."""

    def __init__(self, args):
        self.args = args
        self.servo_ids = [int(x.strip()) for x in args.targets.split(',')]
        self.num_dofs = len(self.servo_ids)

        self.transport = moteus.get_singleton_transport(args)
        self.controllers = {
            servo_id: moteus.Controller(id=servo_id, transport=self.transport)
            for servo_id in self.servo_ids
        }

        self.otg = Ruckig(self.num_dofs)
        self.inp = InputParameter(self.num_dofs)
        self.trajectory = Trajectory(self.num_dofs)

        self.initial_positions = None
        self.waypoints = []
        self.current_waypoint = 0
        self.segment_start_time = 0.0

    async def initialize(self):
        """Stop servos, query positions, and set up trajectory generator."""
        self._setup_kinematic_limits()
        await self._query_initial_positions()
        self._build_waypoints()
        self._set_initial_trajectory_state()

    def _setup_kinematic_limits(self):
        # Scale velocity per DOF to make them different.
        self.inp.max_velocity = [
            self.args.max_velocity * (i + 1) for i in range(self.num_dofs)
        ]
        self.inp.max_acceleration = [self.args.max_accel] * self.num_dofs
        self.inp.max_jerk = [self.args.max_jerk] * self.num_dofs

    async def _query_initial_positions(self):
        print("Stopping servos and querying initial positions...")
        await self.transport.cycle([
            c.make_stop() for c in self.controllers.values()
        ])
        await asyncio.sleep(0.1)

        results = await self.transport.cycle([
            c.make_query() for c in self.controllers.values()
        ])

        self.initial_positions = []
        for servo_id in self.servo_ids:
            pos = 0.0
            for result in results:
                if result.id == servo_id:
                    pos = result.values.get(moteus.Register.POSITION, 0.0)
                    break
            self.initial_positions.append(pos)
            print(f"  Servo {servo_id}: position = {pos:.4f}")

    def _build_waypoints(self):
        # Each servo moves a different distance: servo i moves (i+1) *
        # base_amplitude.
        #
        # This demonstrates that ruckig synchronizes arrival times despite
        # different travel distances.

        base = self.args.base_amplitude

        scales = [i + 1 for i in range(self.num_dofs)]

        waypoint_multipliers = [1.0, 0.0, -1.0, 0.0]

        self.waypoints = []
        for mult in waypoint_multipliers:
            wp = [
                self.initial_positions[i] + mult * base * scales[i]
                for i in range(self.num_dofs)
            ]
            self.waypoints.append(wp)

        print(f"\nWaypoints (absolute positions):")
        print(f"  Each servo moves {base} * (servo_index + 1) revolutions")
        for i, wp in enumerate(self.waypoints):
            wp_str = ", ".join(f"{p:.3f}" for p in wp)
            print(f"  {i}: [{wp_str}]")

    def _set_initial_trajectory_state(self):
        self.inp.current_position = list(self.initial_positions)
        self.inp.current_velocity = [0.0] * self.num_dofs
        self.inp.current_acceleration = [0.0] * self.num_dofs

        self.current_waypoint = 0
        self.inp.target_position = self.waypoints[self.current_waypoint]
        self.inp.target_velocity = [0.0] * self.num_dofs

        result = self.otg.calculate(self.inp, self.trajectory)
        if result not in (Result.Working, Result.Finished):
            raise RuntimeError(
                f"Initial trajectory calculation failed: {result}")

    async def run(self):
        """Main control loop - stream trajectory to servos."""
        print("\nStarting synchronized motion...")
        print("Press Ctrl+C to stop")

        self.segment_start_time = time.time()
        last_print_time = 0

        try:
            while True:
                positions, velocities = self._update_trajectory()
                if positions is None:
                    break

                results = await self._send_commands(positions, velocities)
                self._print_status(results, last_print_time)
                last_print_time = self._maybe_update_print_time(last_print_time)

                await asyncio.sleep(self.args.cycle_time)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            await self.transport.cycle([
                c.make_stop() for c in self.controllers.values()
            ])
            print("Servos stopped.")

    def _update_trajectory(self):
        """Sample current trajectory at the actual elapsed wall-clock time.
        Advances to the next waypoint when the current segment is done.
        Returns (positions, velocities) or (None, None) on error."""
        elapsed = time.time() - self.segment_start_time

        if elapsed >= self.trajectory.duration:
            end_pos, end_vel, end_acc = self.trajectory.at_time(
                self.trajectory.duration)
            self.inp.current_position = list(end_pos)
            self.inp.current_velocity = list(end_vel)
            self.inp.current_acceleration = list(end_acc)

            self.current_waypoint = (
                (self.current_waypoint + 1) % len(self.waypoints))
            self.inp.target_position = self.waypoints[self.current_waypoint]

            result = self.otg.calculate(self.inp, self.trajectory)
            if result not in (Result.Working, Result.Finished):
                print(f"Ruckig error: {result}")
                return None, None

            self.segment_start_time = time.time()
            elapsed = 0.0
            print(f"Reached waypoint, moving to waypoint {self.current_waypoint}")

        positions, velocities, _ = self.trajectory.at_time(elapsed)
        return list(positions), list(velocities)

    async def _send_commands(self, positions, velocities):
        """Send position commands to all servos."""
        commands = [
            self.controllers[servo_id].make_position(
                position=positions[i],
                velocity=velocities[i],
                query=True,
            )
            for i, servo_id in enumerate(self.servo_ids)
        ]
        return await self.transport.cycle(commands)

    def _print_status(self, results, last_print_time):
        """Print servo status at ~5Hz."""
        now = time.time()
        if now - last_print_time < 0.2:
            return

        status_parts = []
        for result in results:
            pos = result.values.get(moteus.Register.POSITION, 0.0)
            vel = result.values.get(moteus.Register.VELOCITY, 0.0)
            mode = result.values.get(moteus.Register.MODE, 0)
            fault = result.values.get(moteus.Register.FAULT, 0)
            status_parts.append(
                f"ID{result.id}:p={pos:+.3f},v={vel:+.3f},"
                f"m={mode},f={fault}")
        if status_parts:
            print(" | ".join(status_parts))

    def _maybe_update_print_time(self, last_print_time):
        now = time.time()
        if now - last_print_time >= 0.2:
            return now
        return last_print_time


async def main():
    args = parse_args()

    print(f"Controlling servos: {[int(x) for x in args.targets.split(',')]}")
    print(f"Max velocity: {args.max_velocity} rev/s (scales with servo index)")
    print(f"Max acceleration: {args.max_accel} rev/s^2")
    print(f"Max jerk: {args.max_jerk} rev/s^3")
    print(f"Cycle time: {args.cycle_time} s")
    print(f"Base amplitude: {args.base_amplitude} rev")

    motion = SynchronizedMotion(args)
    await motion.initialize()
    await motion.run()


if __name__ == '__main__':
    asyncio.run(main())
