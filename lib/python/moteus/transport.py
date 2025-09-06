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
import io
import struct
import typing

from .device_info import DeviceAddress, DeviceInfo
from .protocol import Register, Mode, parse_message, Writer
from .transport_device import Frame, TransportDevice

from . import command
from . import multiplex as mp


def _make_uuid_query():
    """Create a query command to read the device UUID.

    This creates a query for UUID1-4 registers which contain the full 16-byte
    device UUID (4 bytes per register).

    Returns:
        Tuple of (query_data_bytes, expected_reply_size)
    """
    buf = io.BytesIO()
    writer = Writer(buf)

    # Create WriteCombiner for UUID registers (0x150-0x153)
    # All UUID registers are read as INT32
    uuid_combiner = mp.WriteCombiner(writer, 0x10, int(Register.UUID1), [
        mp.INT32,  # UUID1
        mp.INT32,  # UUID2
        mp.INT32,  # UUID3
        mp.INT32,  # UUID4
    ])

    expected_reply_size = 0
    for i in range(uuid_combiner.size()):
        uuid_combiner.maybe_write()

    expected_reply_size += uuid_combiner.reply_size

    return buf.getvalue(), expected_reply_size


def _extract_uuid_from_result(result):
    """Extract UUID bytes from a query result containing UUID1-4.

    Args:
        result: Query result containing UUID1-4 register values

    Returns:
        16-byte UUID as bytes, or None if UUID registers not present
    """
    if not hasattr(result, 'values') or not result.values:
        return None

    values = result.values

    # Check if all UUID registers are present
    uuid_regs = [Register.UUID1, Register.UUID2, Register.UUID3, Register.UUID4]
    if not all(reg in values for reg in uuid_regs):
        return None

    # Combine the 4 32-bit values into a 16-byte UUID
    # Each register is stored as little-endian 32-bit integer
    uuid_bytes = b''
    for reg in uuid_regs:
        # Convert signed int to unsigned int for packing
        val = int(values[reg])
        if val < 0:
            val = val + (1 << 32)  # Convert signed to unsigned 32-bit
        uuid_bytes += struct.pack('<I', val)

    return uuid_bytes


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

    async def _collect_from_device(self, start_time, timeout, device):
        local_results =  []
        while (asyncio.get_event_loop().time() - start_time) < timeout:
            try:
                remaining_time = timeout - (asyncio.get_event_loop().time() - start_time)

                if remaining_time <= 0:
                    break

                response_frame = await asyncio.wait_for(
                    device.receive_frame(),
                    timeout=remaining_time,
                )

                # Is this a response to our query?
                source_id = (response_frame.arbitration_id >> 8) & 0x7f
                dest_id = response_frame.arbitration_id & 0x7f

                if source_id == 0x7f or dest_id != 0:
                    # This wasn't addressed to us.
                    continue

                # Try to parse and extract the UUID.
                this_device = None
                try:
                    result = parse_message(response_frame)
                    uuid_bytes = _extract_uuid_from_result(result)
                    if uuid_bytes and uuid_bytes != b'\x00' * 16:
                        this_device = DeviceInfo(
                            can_id=source_id,
                            uuid=uuid_bytes,
                            transport_device=device,
                        )
                except RuntimeError:
                    pass

                if this_device is None:
                    # If parsing failed, we assume this device does
                    # not support a UUID.
                    this_device = DeviceInfo(
                        can_id=source_id,
                        transport_device=device,
                    )
                local_results.append(this_device)
            except asyncio.TimeoutError:
                break

        return local_results

    async def discover(self, can_prefix=0, timeout=0.2):
        """Discover all controllers attached to any included
        TransportDevices.

        Returns:
            A list of DeviceInfo structures containing the CAN ID,
            UUID, and TransportDevice where this device is located.

        """

        # Flush any responses that might be sitting around.
        await self.flush_read_queue()

        # Generate a UUID query.
        query_data, reply_size = _make_uuid_query()

        broadcast_cmd = command.Command()
        broadcast_cmd.destination = 0x7f  # broadcast address
        broadcast_cmd.source = 0
        broadcast_cmd.data = query_data
        broadcast_cmd.reply_required = True
        broadcast_cmd.can_prefix = can_prefix
        broadcast_cmd.raw = False
        broadcast_cmd.arbitration_id = None

        frame = self._command_to_frame(broadcast_cmd)

        for device in self._devices:
            await device.send_frame(frame)

        discovered_controllers = []
        start_time = asyncio.get_event_loop().time()

        # Collect from all transport devices in parallel.
        tasks = []
        for device in self._devices:
            tasks.append(self._collect_from_device(
                start_time, timeout, device))

        result_lists = await asyncio.gather(*tasks)
        results = []
        for single_list in result_lists:
            results.extend(single_list)

        # Now we need go through these and figure out what appropriate
        # DeviceAddress structures are for each.
        for result in results:
            # Is the CAN ID of this device unique within its
            # TransportDevice?  If so, just use that.
            proposed_can_id = result.can_id
            can_conflicts = [
                x for x in results
                if x != result and
                x.transport_device == result.transport_device and
                x.can_id == proposed_can_id]

            if len(can_conflicts) == 0:
                result.address = DeviceAddress(
                    can_id=proposed_can_id,
                    transport_device=result.transport_device)
            elif result.uuid is not None:
                # We have a UUID.  Find the shortest unique prefix.
                for prefix_len in [4, 8, 12, 16]:
                    proposed_prefix = result.uuid[:prefix_len]

                    conflicts = [x for x in results
                                 if x != result and
                                 x.uuid is not None and
                                 x.uuid[:prefix_len] == proposed_prefix]
                    if len(conflicts) == 0:
                        result.address = DeviceAddress(
                            uuid=proposed_prefix,
                            transport_device=result.transport_device)
                        break
            else:
                # We do not have a UUID or a unique CAN-ID.  That
                # means this result gets no address at all as it is
                # not addressable.
                result.address = None

        return results
