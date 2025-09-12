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
import collections
import io
import itertools
import struct
import typing

from .device_info import DeviceAddress, DeviceInfo
from .protocol import Register, Mode, parse_message, Writer, make_uuid_prefix
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
        mp.IGNORE, # UUID1_MASK
        mp.IGNORE, # UUID2_MASK
        mp.IGNORE, # UUID3_MASK
        mp.IGNORE, # UUID4_MASK
        mp.INT8,   # UUID_MASK_CAPABLE
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
        or UUID mask not capable

    """
    if not hasattr(result, 'values') or not result.values:
        return None

    values = result.values

    # Check if all UUID registers are present
    uuid_regs = [Register.UUID1, Register.UUID2, Register.UUID3, Register.UUID4, Register.UUID_MASK_CAPABLE]
    if not all(reg in values for reg in uuid_regs):
        return None

    if values[Register.UUID_MASK_CAPABLE] == 0:
        return None

    # Combine the 4 32-bit values into a 16-byte UUID
    # Each register is stored as little-endian 32-bit integer
    uuid_bytes = b''
    for reg in uuid_regs[0:4]:
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

    def __init__(self, devices, routing_table={}):
        """Initialize the Transport with one or more devices.

        Args:
          devices: Either a single TransportDevice or a list of instances.

          routing_table: A dictionary from DeviceAddress ->
            TransportDevice.  If an addressed DeviceAddress is not
            present, and multiple TransportDevices are used, then
            discovery is performed to find the correct TransportDevice
            upon first use.

        """
        if isinstance(devices, list):
            self._devices = devices
        else:
            self._devices = [devices]

        self._parent_devices = list(set(
            [d.parent() or d for d in self._devices]))

        self._routing_table = routing_table

        # This will hold frames that were received, but we could not
        # yet deliver them to the client because the calls were
        # cancelled.  They will be delivered during calls to .read(),
        # if any are made.
        self._cancelled_queue = []

        # So that the queue does not grow without bound, we limit it
        # to this many frames.
        self._cancel_queue_max_size = 100

        # This is necessary for some compatibility hacks down below.
        self._pi3hat_device = next(iter(
            [x for x in (self._devices + self._parent_devices)
             if type(x).__name__ == 'Pi3HatDevice']),
            None)

    def devices(self):
        return self._devices

    def count(self):
        return len(self._devices)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def _add_cancelled_frames(self, to_add):
        self._cancelled_queue.extend(to_add)
        while len(self._cancelled_queue) > self._cancel_queue_max_size:
            self._cancel_queue_max_size.pop(0)

    def close(self):
        for device in self._devices:
            device.close()

    async def _read_frames_until_timeout(self, device, end_time):
        results = []
        while True:
            try:
                remaining = end_time - asyncio.get_event_loop().time()

                if remaining <= 0:
                    break

                response = await asyncio.wait_for(
                    device.receive_frame(),
                    remaining)

                results.append(response)
            except asyncio.TimeoutError:
                break
        return results

    async def _get_devices_for_command(self, cmd):
        if len(self._devices) == 1:
            return self._devices

        channel = getattr(cmd, 'channel', None)
        if channel is not None:
            return [channel]

        # For legacy support, we may have a 'bus' attribute on the
        # command.
        if getattr(cmd, 'bus', None) is not None:
            return [d for d in self._devices
                    if hasattr(d, 'bus') and d.bus() == cmd.bus]

        # Make a DeviceAddress version to use everywhere here.
        cmd_destination = (
            cmd.destination
            if not isinstance(cmd.destination, int) else
            DeviceAddress(can_id=cmd.destination))

        # If this is a broadcast request, then we are going to send to
        # everything.
        if (cmd_destination.can_id == 0x7f):
            return self._devices

        # We have multiple devices, and we don't know which one has
        # this destination.  First check the cache.

        maybe_device = self._routing_table.get(cmd_destination, None)
        if maybe_device is not None:
            return [maybe_device]

        # Emit a simple time-limited request on all interfaces to see
        # where it responds and save that result.
        discover_command = command.Command()
        discover_command.destination = cmd_destination
        discover_command.source = cmd.source
        discover_command.reply_required = True
        discover_command.can_prefix = cmd.can_prefix
        discover_command.arbitration_id = None

        if (cmd_destination.can_id is None):
            # We need to request only this UUID to respond.
            discover_command.data = make_uuid_prefix(cmd_destination)

        discover_command.data += bytes([0x50])

        # Flush everything on all transports.
        await self.flush_read_queue()

        # Send the frames.
        frame = self._command_to_frame(discover_command)
        for device in self._devices:
            await device.send_frame(frame)

        # Read everything.  If the system is consistent, we should
        # only get one response across all devices.
        start_time = asyncio.get_event_loop().time()
        end_time = start_time + 0.1
        tasks = [
            self._read_frames_until_timeout(device, end_time)
            for device in self._devices
        ]
        result_lists = await asyncio.gather(*tasks)
        results = []
        for single_list in result_lists:
            results.extend(single_list)

        # Ideally, we have a single result here.
        if len(results) < 1:
            raise RuntimeError(f'{cmd_destination} not found on any CAN bus')
        if len(results) > 1:
            raise RuntimeError(f'More than one {cmd_destination} found across connected CAN busses {results}')

        self._routing_table[cmd_destination] = results[0].channel
        return [results[0].channel]

    def _make_canid(self, device_address):
        if isinstance(device_address, int):
            return device_address
        if device_address.can_id is not None:
            return device_address.can_id
        return 0x7f

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
            self._make_canid(command.destination) |
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
        return lambda f: (
            (((f.arbitration_id >> 8) & 0x7f) == self._make_canid(command.destination) or
             self._make_canid(command.destination) == 0x7f) and
            (f.arbitration_id & 0x7f) == command.source)

    async def _cycle_batch(self, commands, request_attitude,
                           read_unsolicited, force_can_check):
        # Group requests by device.  This is a map from device to:
        #  (TransportDevice.Request)
        device_requests = collections.defaultdict(list)

        for i, command in enumerate(commands):
            frame = self._command_to_frame(command)
            reply_required = command.reply_required
            response_filter = (self._make_response_filter(command)
                               if reply_required else None)

            devices = await self._get_devices_for_command(command)
            for device in devices:
                maybe_parent = device.parent()

                to_send_device = maybe_parent or device

                request = TransportDevice.Request(
                    frame=frame,
                    child_device=device if maybe_parent else None,
                    frame_filter=response_filter,
                    expected_reply_size=command.expected_reply_size)
                request.command = command

                device_requests[to_send_device].append(request)

        if read_unsolicited:
            for device in read_unsolicited:
                maybe_parent = device.parent()
                to_send_device = maybe_parent or device

                device_requests[to_send_device].append(
                    TransportDevice.Request(
                        frame=None,
                        child_device=device if maybe_parent else None,
                        frame_filter=lambda x: True))

        if request_attitude and self._pi3hat_device:
            # This is a compatibility/performance hack that only works
            # with the pi3hat.  Logically we shouldn't know anything
            # about the pi3hat here, but hey, it's necessary for
            # backward compatibility with the old Pi3HatRouter
            # interface.

            attitude_request = TransportDevice.Request(
                frame=None,
                frame_filter=lambda frame: frame.arbitration_id == -1)

            device_requests[self._pi3hat_device].append(attitude_request)

        if (force_can_check is not None
            and force_can_check != 0
            and self._pi3hat_device):
            # This is another compatibility hack for the pi3hat.
            for bus in range(1, 6):
                if ((1 << bus) & force_can_check) == 0:
                    continue

                force_request = TransportDevice.Request(
                    frame=None,
                    frame_filter=lambda x: True)

                device_requests[self._pi3hat_device].append(force_request)

        # Perform batch transactions in parallel for all devices.
        tasks = []
        device_list = []
        for device, request_list in device_requests.items():
            tasks.append(device.transaction(
                request_list,
                request_attitude=request_attitude,
                force_can_check=force_can_check))
            device_list.append(device)

        # Wait for all transports to complete.
        await asyncio.gather(*tasks)

        # We don't care as much about dropping results in the event of
        # a cancellation here in .cycle.  '.cycle' will mostly be used
        # with register mode commands which are stateless anyways.

        responses = []
        for request in itertools.chain(
                *[request_list for _, request_list in device_requests.items()]):
            for frame in request.responses:
                responses.append(
                    request.command.parse(frame)
                    if (hasattr(request, 'command') and
                        hasattr(request.command, 'parse'))
                    else frame)

        return responses

    async def cycle(self, commands,
                    request_attitude=False,
                    read_unsolicited=None,
                    force_can_check=None):
        """Issue all commands, returning any resulting frames.

        Args:

          request_attitude: This is present to allow producing IMU
            data in the same SPI cycle as CAN data with an mjbots
            pi3hat.

          read_unsolicited: An optional list of TransportDevices, for
            which any available unsolicited CAN frames should be
            returned.  If specified on a TransportDevice where moteus
            controllers are commanded, this may result in duplicate
            frame receipts.

          force_can_check: A bitfield to force a connected pi3hat to
            check specific CAN ports.  Modern code should instead use
            the read_unsolicited kwargs.

        """
        results = await self._cycle_batch(
            commands,
            request_attitude=request_attitude,
            force_can_check=force_can_check,
            read_unsolicited=read_unsolicited)
        return results

    async def write(self, command):
        devices = await self._get_devices_for_command(command)
        frame = self._command_to_frame(command)
        if len(devices) == 1:
            await devices[0].send_frame(frame)
        else:
            tasks = [asyncio.create_task(device.send_frame(frame))
                     for device in devices]
            await asyncio.gather(*tasks)

    def _add_task_results_to_cancel_queue(self, tasks):
        to_add = []
        for task in tasks:
            try:
                to_add.append(task.result())
            except:
                pass

        self._add_cancelled_frames(to_add)

    async def read(self, channel=None):
        """Wait for one or more frames to be received.

        This can be used to receive unsolicited data.

        Args:

          channel: If specified, only read from the given
            TransportDevice.
        """
        if self._cancelled_queue:
            return self._cancelled_queue.pop(0)

        if len(self._devices) == 1:
            return await self._devices[0].receive_frame()

        if channel is not None:
            assert channel in self._devices
            return await channel.receive_frame()

        try:
            # Only invoke one receive_frame per parent.
            tasks = [
                asyncio.create_task(device.receive_frame())
                for device in self._parent_devices
            ]
            done, pending = await asyncio.wait(
                tasks,
                return_when=asyncio.FIRST_COMPLETED)
        except asyncio.CancelledError:
            # NOTE: We *really* don't want to let frames get lost here
            # in .read().  Unfortunately, since cancellation
            # exceptions can be thrown from nearly everything, it is
            # really hard to ensure that any received frames are not
            # accidentally discarded.  So, we jump through many hoops,
            # including this 'except' block, the 'finally' block and
            # its sub-try/except.

            # Some of our tasks may have still completed.  We should
            # save those so that we can return them later.
            done_tasks = [x for x in tasks if x.done()]
            self._add_task_results_to_cancel_queue(done_tasks)

            raise
        finally:
            # Ensure that all our tasks are either done or cancelled.
            not_done_tasks = [x for x in tasks if not x.done()]
            _ = [x.cancel() for x in not_done_tasks]

            try:
                # We have to wait on those tasks for their
                # cancellation to become final.
                results = await asyncio.shield(
                    asyncio.gather(*not_done_tasks, return_exceptions=True))

                # We are not exiting ourselves, so limit our cancel
                # add to those things that were not done in the main
                # area.
                self._add_cancelled_frames([x for x in results
                                            if isinstance(x, Frame)])
            except asyncio.CancelledError:
                # However, while 'shield' will stop further
                # cancellations from making into our child tasks, we
                # may still be cancelled during this time.  If that
                # happens, we need to try again.
                await asyncio.shield(
                    asyncio.gather(*not_done_tasks, return_exceptions=True))

                # We are definitely going to die, so we need to get
                # *everything* from tasks.
                self._add_task_results_to_cancel_queue(tasks)

                # Now let our own cancellation make it back up.
                raise

        results = [x.result() for x in done]
        if len(results) > 1:
            self._add_cancelled_frames(results[1:])
            del results[1:]

        assert(len(results) == 1)

        return results[0]

    async def flush_read_queue(self, timeout=0.1, channel=None):
        """Flush any pending receive frames.

        Args:
            timeout: Time in seconds to keep reading before giving up.
            channel: None = all channels
        """

        device_list = self._parent_devices

        if channel is not None:
            assert channel in self._devices
            device_list = [ channel ]

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

    def _response_to_device_info(self, frame, source):
        # Is this a response to our query?
        source_id = (frame.arbitration_id >> 8) & 0x7f
        dest_id = frame.arbitration_id & 0x7f

        if source_id == 0x7f or dest_id != source:
            # This wasn't addressed to us.
            return None

        # Try to parse and extract the UUID.
        this_device = None
        try:
            result = parse_message(frame)
            uuid_bytes = _extract_uuid_from_result(result)
            if uuid_bytes:
                return DeviceInfo(
                    can_id=source_id,
                    uuid=uuid_bytes,
                    transport_device=frame.channel,
                )
        except RuntimeError:
            pass

        if this_device is None:
            # If parsing failed, we assume this device does
            # not support a UUID.
            return DeviceInfo(
                can_id=source_id,
                transport_device=frame.channel,
            )

        return None

    async def discover(self, can_prefix=0, source=0, timeout=0.2):
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
        broadcast_cmd.source = source
        broadcast_cmd.data = query_data
        broadcast_cmd.reply_required = True
        broadcast_cmd.can_prefix = can_prefix
        broadcast_cmd.raw = False
        broadcast_cmd.arbitration_id = None
        broadcast_cmd.expected_reply_size = reply_size

        frame = self._command_to_frame(broadcast_cmd)

        for device in self._devices:
            await device.send_frame(frame)

        discovered_controllers = []
        start_time = asyncio.get_event_loop().time()

        # Collect from all transport devices in parallel.
        tasks = []
        for device in self._devices:
            tasks.append(self._read_frames_until_timeout(
                device, start_time + timeout))

        result_lists = await asyncio.gather(*tasks)
        results = []
        for single_list in result_lists:
            results.extend(single_list)

        results = [a for a in
                   [self._response_to_device_info(x, source)
                    for x in results]
                   if a is not None]

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

        return sorted(results)
