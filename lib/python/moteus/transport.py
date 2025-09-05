# Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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
import typing
from .transport_device import Frame, TransportDevice

class Transport:
    """This is an object which can dispatch commands to one or more
    controllers across potentially multiple CAN-FD busses.
    """

    def __init__(self, devices):
        """Initialize the Transport with one or more devices.

        Args:
          devices: Either a single TransportDevice or a list of instances.
        """
        if isinstance(devices, list):
            self._devices = devices
        else:
            self._devices = [devices]

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def close(self):
        for device in self._devices:
            device.close()

    def _get_device_for_command(self, command):
        if len(self._devices) == 1:
            return self._devices[0]

        channel = getattr(command, 'channel', None)
        if channel is not None:
            raise NotImplementedError()

        # Possibly look up by CAN ID or UUID
        raise NotImplementedError()

    def _command_to_frame(self, command) -> Frame:
        if getattr(command, 'raw', False):
            return Frame(
                arbitration_id=command.arbitration_id,
                data=command.data,
                is_extended_id=command.arbitration_id > 0x7ff,
                is_fd=True,
                bitrate_switch=True,
                channel=command.channel)

        arbitration_id = (
            command.destination |
            (0x8000 if command.reply_required else 0) |
            (command.source << 8) |
            (command.can_prefix << 16))

        return Frame(
            arbitration_id=arbitration_id,
            data=command.data,
            is_extended_id=arbitration_id > 0x7ff,
            is_fd=True,
            bitrate_switch=True)

    def _make_response_filter(self, command):
        if getattr(command, 'raw', False):
            # For raw commands, accept any response.  In practice,
            # that means that they can't be intermingled with moteus
            # responses.
            return lambda f: True

        # Filter by both the source and destination device IDs.
        return lambda f: (((f.arbitration_id >> 8) & 0x7f) == command.destination and
                          (f.arbitration_id & 0x7f) == command.source)

    async def _cycle_batch(self, commands, timeout=None):
        # Group commands by device.  This is a map from device to:
        #  (command_index, command, frame, filter)
        device_commands = {}

        for i, command in enumerate(commands):
            device = self._get_device_for_command(command)
            frame = self._command_to_frame(command)
            reply_required = command.reply_required
            response_filter = self._make_response_filter(command) if reply_required else None

            if device not in device_commands:
                device_commands[device] = []

            device_commands[device].append((i, command, frame, response_filter))

        # Perform batch transactions in parallel for all devices.
        tasks = []
        device_list = []
        for device, cmd_list in device_commands.items():
            requests = [(frame, filter) for _, _, frame, filter in cmd_list]
            tasks.append(device.transaction(requests, timeout=timeout))
            device_list.append(device)

        # Wait for all transports to complete.
        responses_list = await asyncio.gather(*tasks)
        device_responses = {}
        for device, responses in zip(device_list, responses_list):
            device_responses[device] = responses

        # Sort the results into the original command order.
        results = [None] * len(commands)
        for device, cmd_list in device_commands.items():
            device_response = device_responses[device]
            for device_idx, (orig_idx, command, _, _) in enumerate(cmd_list):
                this_response = device_response[device_idx]

                if command.reply_required and this_response:
                    if hasattr(command, 'parse'):
                        results[orig_idx] = command.parse(this_response)
                    else:
                        results[orig_idx] = this_response
        return results


    async def cycle(self, commands, timeout=None):
        return await self._cycle_batch(commands, timeout)

    async def write(self, command):
        device = self._get_device_for_command(command)
        frame = self._command_to_frame(command)
        await device.send_frame(frame)

    async def read(self, channel=None):
        if len(self._devices) == 1:
            return await self._devices[0].receive_frame()

        if channel is not None:
            raise NotImplementedError()

        raise NotImplementedError()

    async def flush_read_queue(self, timeout=0.1, channel=None):
        """Flush any pending receive frames.

        Args:
            timeout: Time in seconds to keep reading before giving up.
            channel: None = all channels
        """

        device_list = self._devices

        if channel is not None:
            raise NotImplementedError()

        start_time = asyncio.get_event_loop().time()
        tasks = []
        for device in device_list:
            async def flush_single(device):
                while True:
                    try:
                        remaining = timeout - (asyncio.get_event_loop().time() - start_time)
                        if remaining <= 0:
                            break
                        _ = await asyncio.wait_for(device.receive_frame(),
                                                   timeout=remaining)
                    except asyncio.TimeoutError:
                        break
            tasks.append(flush_single(device))

        if tasks:
            await asyncio.gather(*tasks)
