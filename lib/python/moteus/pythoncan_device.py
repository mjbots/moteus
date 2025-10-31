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
import os
import sys
import time
import typing

from .transport_device import Frame, FrameFilter, TransportDevice, Subscription

can = None


def _detect_fdcanusb_serial_linux(ifname: str) -> typing.Optional[str]:
    """Get USB serial number for a socketcan interface if it's a fdcanusb.

    Args:
        ifname: Network interface name (e.g., "can0")

    Returns:
        Serial number string if this is a fdcanusb, None otherwise
    """
    try:
        # Path to USB device serial number via sysfs
        serial_path = f'/sys/class/net/{ifname}/device/../serial'
        serial_path = os.path.realpath(serial_path)

        if not os.path.exists(serial_path):
            return None

        # Verify it's actually a fdcanusb by checking VID/PID
        device_dir = os.path.dirname(serial_path)
        vid_path = os.path.join(device_dir, 'idVendor')
        pid_path = os.path.join(device_dir, 'idProduct')

        if not os.path.exists(vid_path) or not os.path.exists(pid_path):
            return None

        with open(vid_path) as f:
            vid = int(f.read().strip(), 16)
        with open(pid_path) as f:
            pid = int(f.read().strip(), 16)

        # Check if it's a fdcanusb (old or new)
        FDCANUSB_IDS = [
            (0x0483, 0x5740),  # Legacy fdcanusb
            (0x1209, 0x2323),  # New fdcanusb with gs_usb
        ]

        if (vid, pid) in FDCANUSB_IDS:
            with open(serial_path) as f:
                return f.read().strip()

    except (OSError, IOError, ValueError):
        pass

    return None


def _get_usb_device_info(device):
    """Extract identifying information from a pyusb device.

    Args:
        device: pyusb device object

    Returns:
        Dictionary with USB topology and serial number info
    """
    import usb.util

    info = {
        'bus': device.bus,
        'address': device.address,
        'vid': device.idVendor,
        'pid': device.idProduct,
        'serial': None,
        'port_path': None,
    }

    # Try to get serial number
    try:
        if device.iSerialNumber:
            info['serial'] = usb.util.get_string(device, device.iSerialNumber)
    except (ValueError, usb.core.USBError):
        pass

    # Try to get port path (USB topology)
    try:
        if hasattr(device, 'port_numbers') and device.port_numbers:
            info['port_path'] = tuple(device.port_numbers)
        elif hasattr(device, 'port_number'):
            info['port_path'] = (device.port_number,)
    except (AttributeError, usb.core.USBError):
        pass

    return info


def _match_interface_to_usb_windows(channel: str, usb_devices: typing.List[dict]) -> typing.Optional[str]:
    """Try to match a python-can interface to a USB device on Windows.

    On Windows, gs_usb interfaces might be identified by:
    - Index (e.g., 0, 1, 2)
    - Device path or identifier

    Args:
        channel: python-can channel identifier
        usb_devices: List of USB device info dictionaries

    Returns:
        Serial number if match found, None otherwise
    """
    # Strategy 1: If channel is a simple index, match by device enumeration order
    try:
        channel_idx = int(channel)
        if 0 <= channel_idx < len(usb_devices):
            return usb_devices[channel_idx]['serial']
    except (ValueError, TypeError):
        pass

    # Strategy 2: If there's only one fdcanusb device, assume it's a match
    if len(usb_devices) == 1:
        return usb_devices[0]['serial']

    # Strategy 3: Try to extract bus/device numbers from channel string
    # Some python-can backends include USB topology in the channel name
    import re
    bus_addr_match = re.search(r'bus[_\s]*(\d+)[_\s]*(?:dev|addr)[_\s]*(\d+)', channel, re.IGNORECASE)
    if bus_addr_match:
        bus_num = int(bus_addr_match.group(1))
        addr_num = int(bus_addr_match.group(2))
        for dev_info in usb_devices:
            if dev_info['bus'] == bus_num and dev_info['address'] == addr_num:
                return dev_info['serial']

    # Unable to match
    return None


def _match_interface_to_usb_mac(channel: str, usb_devices: typing.List[dict]) -> typing.Optional[str]:
    """Try to match a python-can interface to a USB device on Mac.

    On Mac, gs_usb interfaces are typically identified similar to Windows.

    Args:
        channel: python-can channel identifier
        usb_devices: List of USB device info dictionaries

    Returns:
        Serial number if match found, None otherwise
    """
    # Mac uses similar matching strategy as Windows
    return _match_interface_to_usb_windows(channel, usb_devices)


def _detect_fdcanusb_serials_windows_mac() -> typing.Dict[str, str]:
    """Get mapping of interface -> serial for fdcanusb devices on Windows/Mac.

    This function uses pyusb to enumerate USB devices and attempts to match
    them with python-can interface identifiers. The matching is best-effort
    as the correlation depends on the python-can backend and platform.

    Returns:
        Dictionary mapping interface identifier to serial number
    """
    try:
        import usb.core
        import usb.util
    except ImportError:
        # pyusb not available, return empty dict
        # Deduplication will not work without pyusb on Windows/Mac
        import logging
        logging.debug("pyusb not available - fdcanusb deduplication disabled on Windows/Mac")
        return {}

    try:
        # Find all fdcanusb devices (old and new)
        FDCANUSB_IDS = [
            (0x0483, 0x5740),  # Legacy fdcanusb
            (0x1209, 0x2323),  # New fdcanusb with gs_usb
        ]

        usb_devices = []
        for vid, pid in FDCANUSB_IDS:
            found = usb.core.find(find_all=True, idVendor=vid, idProduct=pid)
            for device in found:
                dev_info = _get_usb_device_info(device)
                if dev_info['serial']:  # Only include devices with serial numbers
                    usb_devices.append(dev_info)

        if not usb_devices:
            return {}

        # Sort by bus and address for consistent ordering
        usb_devices.sort(key=lambda d: (d['bus'], d['address']))

        import logging
        logging.debug(f"Found {len(usb_devices)} fdcanusb devices via pyusb: " +
                     ", ".join([f"{d['serial']} (bus {d['bus']} addr {d['address']})"
                               for d in usb_devices]))

        # Note: We return the device list here and do matching in enumerate_devices
        # where we have access to the actual channel names from python-can
        # Store in a global for access during enumeration
        global _cached_usb_devices
        _cached_usb_devices = usb_devices
        return {}

    except Exception as e:
        import logging
        logging.debug(f"Error detecting fdcanusb devices via pyusb: {e}")
        return {}


# Global cache for USB devices (used by Windows/Mac matching)
_cached_usb_devices = None


class PythonCanSubscription(Subscription):
    def __init__(self, device, subscription_id):
        self._device = device
        self._id = subscription_id

    def cancel(self):
        self._device._remove_subscription(self._id)


class PythonCanDevice(TransportDevice):
    '''Implements a 'Transport' on top of python-can.'''

    def __init__(self, *args, **kwargs):
        '''Nearly all arguments pass through to can.Bus'''

        super(PythonCanDevice, self).__init__(
            **{k: kwargs[k] for k in ['max_buffer_size',
                                      'padding_hex'] if k in kwargs})

        self._padding_byte = bytes.fromhex(self._padding_hex)
        self._debug_log = kwargs.pop('debug_log', None)

        # We delay importing this until we need it, as it can take a
        # while.
        global can
        if not can:
            import can
            try:
                can.rc = can.util.load_config()
            except can.CanInterfaceNotImplementedError as e:
                if 'Unknown interface type "None"' not in str(e):
                    raise

        if 'interface' not in can.rc and 'interface' not in kwargs:
            can.rc['interface'] = 'socketcan'
        if 'channel' not in can.rc and 'channel' not in kwargs:
            can.rc['channel'] = 'can0'
        if 'fd' not in can.rc:
            can.rc['fd'] = True

        self._disable_brs = False
        if 'disable_brs' in kwargs:
            self._disable_brs = kwargs['disable_brs']
            del kwargs['disable_brs']

        # Extract fdcanusb serial number before passing kwargs to can.Bus
        self._fdcanusb_serial = kwargs.pop('fdcanusb_serial', None)

        if ('timing' not in kwargs and
            kwargs.get('interface', can.rc.get('interface', None)) == 'pcan'):
            # We default to the timing that works with moteus and assume
            # an 80MHz base clock, which seems pretty typical for PCAN
            # interfaces.
            kwargs['timing'] = can.BitTimingFd.from_sample_point(
                nom_bitrate=1000000, data_bitrate=5000000,
                nom_sample_point=66, data_sample_point=66,
                f_clock=80000000)

        self._can = can.Bus(*args, **kwargs)
        self._setup = False

        self._notifier = None

        self._log_prefix = (
            getattr(self._can, 'channel', None) or
            kwargs.get('channel', None) or
            str(self._can))

    @property
    def fdcanusb_serial(self):
        """Return the USB serial number if this is a fdcanusb, None otherwise."""
        return self._fdcanusb_serial

    def __repr__(self):
        if self._fdcanusb_serial:
            return f"PythonCan('{self._log_prefix}', fdcanusb_sn='{self._fdcanusb_serial}')"
        return f"PythonCan('{self._log_prefix}')"

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    def close(self):
        if self._notifier:
            self._notifier.stop()
            self._notifier = None

        self._can.shutdown()

    def empty_bus_tx_safe(self):
        return True

    def _maybe_setup(self):
        if self._setup:
            return

        self._notifier = can.Notifier(
            self._can, [self._receive_handler],
            loop=asyncio.get_event_loop())
        self._setup = True

    async def _receive_handler(self, message):
        if message.is_error_frame:
            return

        frame = self._can_message_to_frame(message)

        if self._debug_log:
            self._write_log(f'< {frame.arbitration_id:04X} {frame.data.hex().upper()}'.encode('latin1'))

        await self._handle_received_frame(frame)

    def _frame_to_can_message(self, frame: Frame):
        dlc = self._round_up_dlc(len(frame.data))
        padding_bytes = dlc - len(frame.data)

        return can.Message(
            arbitration_id=frame.arbitration_id,
            is_extended_id=frame.is_extended_id,
            dlc=dlc,
            data=frame.data + bytes(self._padding_byte) * padding_bytes,
            is_fd=frame.is_fd,
            bitrate_switch=frame.bitrate_switch and not self._disable_brs,
        )

    def _can_message_to_frame(self, message) -> Frame:
        return Frame(
            arbitration_id=message.arbitration_id,
            data=message.data,
            dlc=message.dlc,
            is_extended_id=message.is_extended_id,
            is_fd=message.is_fd,
            bitrate_switch=getattr(message, 'bitrate_switch', False),
            channel=self,
        )

    async def send_frame(self, frame: Frame):
        self._maybe_setup()

        can_message = self._frame_to_can_message(frame)

        if self._debug_log:
            self._write_log(f'> {frame.arbitration_id:04x} {can_message.data.hex().upper()}'.encode('latin1'))

        self._can.send(self._frame_to_can_message(frame))

    async def receive_frame(self):
        self._maybe_setup()

        return await super(PythonCanDevice, self).receive_frame()

    async def transaction(
            self,
            requests: typing.List[TransportDevice.Request],
            **kwargs):
        # We do not support child devices.
        assert not any([request.child_device is not None
                        for request in requests])

        self._maybe_setup()

        def make_subscription(request):
            future = asyncio.Future()

            async def handler(frame, request=request, future=future):
                if future.done():
                    return

                request.responses.append(frame)
                future.set_result(None)

            return self._subscribe(request.frame_filter, handler), future

        subscriptions = [
            make_subscription(request)
            for request in requests
            if request.frame_filter is not None
        ]

        try:
            for request in requests:
                await self.send_frame(request.frame)

            if subscriptions:
                await asyncio.gather(*[x[1] for x in subscriptions])
        finally:
            for x in subscriptions:
                x[0].cancel()

    def _write_log(self, output: bytes):
        assert self._debug_log is not None
        self._debug_log.write(f'{time.time():.6f}/{self._log_prefix} '.encode('latin1') + output + b'\n')

    @staticmethod
    def enumerate_devices(**kwargs):
        """
        Returns:
            (list[PythonCanDevice]): list of objects for devices present
                in the current system
        """
        global can
        if not can:
            import can

        import logging

        # python-can spews a lot of logging during enumeration.
        # Squelch as much of that as we can.

        log = logging.getLogger("can")  # covers can.interface and sub-loggers
        prev_level, prev_prop = log.level, log.propagate
        log.setLevel(logging.CRITICAL)

        # Don't bubble to root handlers.
        log.propagate = False

        try:
            # Try to auto-enumerate.
            interfaces_to_check = os.getenv(
                "MJBOTS_PYTHONCAN", "socketcan,pcan,kvaser,vector").split(',')
            available_configs = can.detect_available_configs(interfaces_to_check)

            if not available_configs:
                raise RuntimeError("No python-can interfaces detected")

            def _merge_args(a, b):
                return {**a, **b}

            # Detect fdcanusb serial numbers for each interface
            result = []

            # On Windows/Mac, enumerate USB devices first (if not already done)
            global _cached_usb_devices
            if sys.platform in ('win32', 'darwin'):
                if _cached_usb_devices is None:
                    _detect_fdcanusb_serials_windows_mac()

            for config in available_configs:
                merged_config = _merge_args(kwargs, config)

                # Try to detect if this is a fdcanusb device
                fdcanusb_serial = None
                channel = config.get('channel', None)

                if channel and sys.platform.startswith('linux'):
                    # On Linux, try to get serial number from sysfs
                    fdcanusb_serial = _detect_fdcanusb_serial_linux(channel)
                elif channel and sys.platform in ('win32', 'darwin'):
                    # On Windows/Mac, use cached USB device info and try to match
                    if _cached_usb_devices:
                        if sys.platform == 'win32':
                            fdcanusb_serial = _match_interface_to_usb_windows(
                                channel, _cached_usb_devices)
                        else:  # darwin (Mac)
                            fdcanusb_serial = _match_interface_to_usb_mac(
                                channel, _cached_usb_devices)

                        if fdcanusb_serial:
                            import logging
                            logging.debug(f"Matched {channel} to fdcanusb with serial {fdcanusb_serial}")

                if fdcanusb_serial:
                    merged_config['fdcanusb_serial'] = fdcanusb_serial

                result.append(PythonCanDevice(**merged_config))

            return result
        finally:
            # Restore logging to the previous state.
            log.setLevel(prev_level)
            log.propagate = prev_prop
