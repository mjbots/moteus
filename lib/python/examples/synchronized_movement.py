#!/usr/bin/python3

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

'''This example shows how time can be synchronized across multiple
moteus controllers.  When time is synchronized in this way, then
trajectory commands that would take the same amount of time will
complete simultaneously.'''

import argparse
import asyncio
import math
import moteus
import time


def _calculate_ms_delta(time1, time2):
    # These are returned as int32s, so they may have wrapped around.
    if time2 < 0 and time1 > 0:
        result_ms = time2 + (2**32) - time1
    else:
        result_ms = time2 - time1
    if result_ms > 100000 or time2 < time1:
        # We'll assume any difference of more than 100s is a problem
        # (or a negative difference).
        return None
    return result_ms


class ServoClock:
    '''This class can be used to keep a controller's time base
    synchronized with the host time base.

    Instantiate it, then call await ServoClock.update_second() at a
    regular rate, approximately at 1Hz.
    '''

    # This constant should be approximately 5-10x the update rate.
    # Smaller means that the device time will converge with the host
    # time more quickly, but will also be less stable.
    TIME_CONSTANT = 5.0

    # This is the approximate change in clock rate of the device for
    # each trim count change.
    TRIM_STEP_SIZE = 0.0025

    def __init__(self, controller, measure_only=False):
        self.controller = controller
        self.initial_time = None
        self.state = None
        self.device_ms_count = 0
        self.measure_only = measure_only

        # This is the currently reported time error between the host
        # and the device.
        self.time_error_s = 0.0

        self._query = self.controller.make_custom_query(
            {moteus.Register.MILLISECOND_COUNTER : moteus.INT32,
             moteus.Register.CLOCK_TRIM: moteus.INT32}
        )

    async def update_second(self):
        '''This should be called at a regular interval, no more often than
        once per second.
        '''

        old_state = self.state

        self.state = await self.controller.execute(self._query)

        now = time.time()

        ms_count_delta = 0

        if old_state is not None:
            ms_count_delta = _calculate_ms_delta(
                old_state.values[moteus.Register.MILLISECOND_COUNTER],
                self.state.values[moteus.Register.MILLISECOND_COUNTER])

            if ms_count_delta is None:
                # We became desynchronized and need to restart.
                self.device_ms_count = 0
                self.initial_time = None
            else:
                self.device_ms_count += ms_count_delta

        if self.initial_time is not None and ms_count_delta != 0:
            # We have enough information to calculate an update.

            # First, we calculate the delta between our starting time
            # and now in the host timespace.
            total_host_time = now - self.initial_time

            # And the amount of host time that has advanced since our
            # last call.  This should be approximately 1s.
            period_host_time = now - self.last_time

            # Now measure the absolute drift in seconds between the
            # device and our host clock.
            absolute_drift = self.device_ms_count / 1000 - total_host_time
            self.time_error_s = absolute_drift

            # And secondarily measure the ratio between the device
            # time and host time during the last period.
            rate_drift = (ms_count_delta / 1000) / period_host_time

            # What drift would we need to cancel out our total
            # absolute drift over the next TIME_CONSTANT seconds?
            desired_drift = 1 + -absolute_drift / self.TIME_CONSTANT

            # Figure out how much we need to change the devices clock
            # rate in order to cancel that drift, both as a floating
            # point value, then again in integer counts.
            delta_drift = desired_drift - rate_drift
            delta_drift_integral = round(delta_drift / self.TRIM_STEP_SIZE)

            # Finally, we figure out the new trim value we should ask
            # for.
            new_trim = (self.state.values[moteus.Register.CLOCK_TRIM] +
                        delta_drift_integral)

            if not self.measure_only:
                await self.controller.set_trim(trim=new_trim)

        if self.initial_time is None:
            self.initial_time = now

        self.last_time = now


class Poller:
    TIMESTAMP_S = 0.01
    CLOCK_UPDATE_S = 1.0

    def __init__(self, controllers, args):
        self.controllers = controllers
        self.last_time = time.time()
        self.servo_data = {}
        self.servo_clocks = {
            x.id: ServoClock(x, measure_only=args.no_synchronize)
            for x in controllers.values()
        }

        self.next_clock_time = self.last_time + self.CLOCK_UPDATE_S


    async def wait_for_event(self, condition, timeout=None, per_cycle=None):
        start = time.time()
        while True:
            now = time.time()
            if (now - start) > timeout:
                raise RuntimeError("timeout")

            if now > self.next_clock_time:
                self.next_clock_time += self.CLOCK_UPDATE_S
                [await x.update_second() for x in self.servo_clocks.values()]

            self.last_time += self.TIMESTAMP_S
            delta_s = self.last_time - now
            await asyncio.sleep(delta_s)

            self.servo_data = {x.id : await x.query()
                               for x in self.controllers.values()}

            if per_cycle:
                per_cycle()

            if condition():
                return



async def main():
    # For the purposes of this demonstration we will:
    #
    # * Continually sample the controllers at 100Hz.
    # * Initially command them to position 0.0 and wait for them to arrive.
    # * Command them to move to position 0.5 at a speed that will take
    #   100s to complete.
    # * Report the elapsed host time for each device to complete this
    #   trajectory.
    # * While doing the above, invoke ServoClock.update_second once
    #   per second (if synchronization is enabled)

    # Synchronization can be turned off with the command line flags
    #  --no-synchronize

    parser = argparse.ArgumentParser()

    parser.add_argument('--no-synchronize', action='store_true')
    parser.add_argument('--verbose', '-v', action='store_true')

    args = parser.parse_args()

    SERVO_IDS = [1, 2]

    qr = moteus.QueryResolution()
    qr.trajectory_complete = moteus.INT8

    controllers = {x: moteus.Controller(x, query_resolution=qr) for x in SERVO_IDS}

    poller = Poller(controllers, args)

    # Tell all the servos to go to position 0.0.
    [await servo.set_position(
        position=0.0, velocity_limit=0.5, accel_limit=5.0, watchdog_timeout=math.nan)
     for servo in controllers.values()]

    await poller.wait_for_event(
        lambda: all([x.values[moteus.Register.TRAJECTORY_COMPLETE]
                     for x in poller.servo_data.values()]),
        timeout=5.0)

    # Now all the servos are at 0.  Command them all to move to 0.5 at
    # a rate of 0.005 revolutions per second.
    [await servo.set_position(position=0.5, velocity_limit=0.005,
                              accel_limit=5.0, watchdog_timeout=math.nan)
     for servo in controllers.values()]

    start_move = time.time()
    done_time = {x:None for x in SERVO_IDS}

    def check_done_time():
        for servo, data in poller.servo_data.items():
            if done_time[servo] is None and data.values[moteus.Register.TRAJECTORY_COMPLETE]:
                done_time[servo] = time.time() - start_move

        status_line = f"{time.time()-start_move:7.3f}  "
        for id in sorted(controllers.keys()):
            status_line += (
                f"{id}:{poller.servo_data[id].values[moteus.Register.POSITION]:.4f}," +
                f"tdelta_ms={poller.servo_clocks[id].time_error_s*1000:5.1f}")
            if args.verbose and poller.servo_clocks[id].state is not None:
                status_line += f",ms={poller.servo_clocks[id].state.values[moteus.Register.MILLISECOND_COUNTER]}"
            status_line += "    "

        print(status_line, end='\r', flush=True)

    await poller.wait_for_event(
        lambda: all([x.values[moteus.Register.TRAJECTORY_COMPLETE]
                     for x in poller.servo_data.values()]),
        timeout=120.0,
        per_cycle = check_done_time,
    )

    print(f"done: {done_time}")


if __name__ == '__main__':
    asyncio.run(main())
