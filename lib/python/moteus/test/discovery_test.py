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

"""Tests for fdcanusb device discovery and deduplication."""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
import tempfile

import moteus.fdcanusb_device as fdcanusb_device
import moteus.pythoncan_device as pythoncan_device


class FdcanusbVidPidDetectionTest(unittest.TestCase):
    """Test VID/PID detection for old and new fdcanusb devices."""

    def test_old_fdcanusb_detection(self):
        """Test that old fdcanusb (0x0483:0x5740) is detected."""
        mock_port = Mock()
        mock_port.device = '/dev/ttyACM0'
        mock_port.vid = 0x0483
        mock_port.pid = 0x5740

        with patch('serial.tools.list_ports.comports', return_value=[mock_port]):
            devices = fdcanusb_device.FdcanusbDevice.pyserial_detect_fdcanusbs()
            self.assertEqual(len(devices), 1)
            self.assertEqual(devices[0], '/dev/ttyACM0')

    def test_new_fdcanusb_detection(self):
        """Test that new fdcanusb (0x1209:0x2323) is detected."""
        mock_port = Mock()
        mock_port.device = '/dev/ttyACM1'
        mock_port.vid = 0x1209
        mock_port.pid = 0x2323

        with patch('serial.tools.list_ports.comports', return_value=[mock_port]):
            devices = fdcanusb_device.FdcanusbDevice.pyserial_detect_fdcanusbs()
            self.assertEqual(len(devices), 1)
            self.assertEqual(devices[0], '/dev/ttyACM1')

    def test_both_fdcanusb_detection(self):
        """Test that both old and new fdcanusb are detected."""
        mock_port_old = Mock()
        mock_port_old.device = '/dev/ttyACM0'
        mock_port_old.vid = 0x0483
        mock_port_old.pid = 0x5740

        mock_port_new = Mock()
        mock_port_new.device = '/dev/ttyACM1'
        mock_port_new.vid = 0x1209
        mock_port_new.pid = 0x2323

        with patch('serial.tools.list_ports.comports', return_value=[mock_port_old, mock_port_new]):
            devices = fdcanusb_device.FdcanusbDevice.pyserial_detect_fdcanusbs()
            self.assertEqual(len(devices), 2)
            self.assertIn('/dev/ttyACM0', devices)
            self.assertIn('/dev/ttyACM1', devices)

    def test_non_fdcanusb_not_detected(self):
        """Test that non-fdcanusb devices are not detected."""
        mock_port = Mock()
        mock_port.device = '/dev/ttyACM0'
        mock_port.vid = 0x1234  # Random VID
        mock_port.pid = 0x5678  # Random PID

        with patch('serial.tools.list_ports.comports', return_value=[mock_port]):
            devices = fdcanusb_device.FdcanusbDevice.pyserial_detect_fdcanusbs()
            self.assertEqual(len(devices), 0)


class FdcanusbSerialNumberTest(unittest.TestCase):
    """Test serial number detection on Linux using sysfs."""

    def setUp(self):
        """Set up test fixtures."""
        # Skip tests on non-Linux platforms
        if not sys.platform.startswith('linux'):
            self.skipTest("Serial number detection only implemented on Linux")

    def test_detect_old_fdcanusb_serial_linux(self):
        """Test detecting serial number for old fdcanusb on Linux."""
        # Create a temporary directory structure mimicking sysfs
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create mock sysfs structure
            net_dir = os.path.join(tmpdir, 'sys', 'class', 'net', 'can0')
            device_dir = os.path.join(tmpdir, 'sys', 'devices', 'usb1', '1-1')
            os.makedirs(net_dir)
            os.makedirs(device_dir)

            # Write VID/PID for old fdcanusb
            with open(os.path.join(device_dir, 'idVendor'), 'w') as f:
                f.write('0483\n')
            with open(os.path.join(device_dir, 'idProduct'), 'w') as f:
                f.write('5740\n')
            with open(os.path.join(device_dir, 'serial'), 'w') as f:
                f.write('F34E5D64\n')

            # Mock realpath to return our test directory
            def mock_realpath(path):
                if 'can0/device/../serial' in path:
                    return os.path.join(device_dir, 'serial')
                elif 'can0/device/../idVendor' in path:
                    return os.path.join(device_dir, 'idVendor')
                elif 'can0/device/../idProduct' in path:
                    return os.path.join(device_dir, 'idProduct')
                return path

            with patch('os.path.realpath', side_effect=mock_realpath):
                with patch('os.path.exists', return_value=True):
                    serial = pythoncan_device._detect_fdcanusb_serial_linux('can0')

            self.assertEqual(serial, 'F34E5D64')

    def test_detect_new_fdcanusb_serial_linux(self):
        """Test detecting serial number for new fdcanusb on Linux."""
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create mock sysfs structure
            net_dir = os.path.join(tmpdir, 'sys', 'class', 'net', 'can0')
            device_dir = os.path.join(tmpdir, 'sys', 'devices', 'usb1', '1-1')
            os.makedirs(net_dir)
            os.makedirs(device_dir)

            # Write VID/PID for new fdcanusb
            with open(os.path.join(device_dir, 'idVendor'), 'w') as f:
                f.write('1209\n')
            with open(os.path.join(device_dir, 'idProduct'), 'w') as f:
                f.write('2323\n')
            with open(os.path.join(device_dir, 'serial'), 'w') as f:
                f.write('79678ECE\n')

            def mock_realpath(path):
                if 'can0/device/../serial' in path:
                    return os.path.join(device_dir, 'serial')
                elif 'can0/device/../idVendor' in path:
                    return os.path.join(device_dir, 'idVendor')
                elif 'can0/device/../idProduct' in path:
                    return os.path.join(device_dir, 'idProduct')
                return path

            with patch('os.path.realpath', side_effect=mock_realpath):
                with patch('os.path.exists', return_value=True):
                    serial = pythoncan_device._detect_fdcanusb_serial_linux('can0')

            self.assertEqual(serial, '79678ECE')

    def test_non_fdcanusb_returns_none(self):
        """Test that non-fdcanusb devices return None."""
        with tempfile.TemporaryDirectory() as tmpdir:
            device_dir = os.path.join(tmpdir, 'sys', 'devices', 'usb1', '1-1')
            os.makedirs(device_dir)

            # Write VID/PID for non-fdcanusb device
            with open(os.path.join(device_dir, 'idVendor'), 'w') as f:
                f.write('1234\n')
            with open(os.path.join(device_dir, 'idProduct'), 'w') as f:
                f.write('5678\n')
            with open(os.path.join(device_dir, 'serial'), 'w') as f:
                f.write('ABCD1234\n')

            def mock_realpath(path):
                if 'can0/device/../serial' in path:
                    return os.path.join(device_dir, 'serial')
                elif 'can0/device/../idVendor' in path:
                    return os.path.join(device_dir, 'idVendor')
                elif 'can0/device/../idProduct' in path:
                    return os.path.join(device_dir, 'idProduct')
                return path

            with patch('os.path.realpath', side_effect=mock_realpath):
                with patch('os.path.exists', return_value=True):
                    serial = pythoncan_device._detect_fdcanusb_serial_linux('can0')

            self.assertIsNone(serial)


class DeduplicationTest(unittest.TestCase):
    """Test deduplication logic in transport factory."""

    def test_fdcanusb_serial_number_property(self):
        """Test that fdcanusb device exposes serial_number property."""
        mock_port = Mock()
        mock_port.device = '/dev/ttyACM0'
        mock_port.serial_number = 'TEST123'

        with patch('serial.tools.list_ports.comports', return_value=[mock_port]):
            with patch('moteus.fdcanusb_device.aioserial.AioSerial'):
                device = fdcanusb_device.FdcanusbDevice('/dev/ttyACM0')
                # The serial number should be detected
                self.assertIsNotNone(device.serial_number)

    def test_pythoncan_fdcanusb_serial_property(self):
        """Test that PythonCanDevice stores fdcanusb_serial."""
        # Mock python-can Bus
        with patch('moteus.pythoncan_device.can'):
            mock_bus = Mock()
            mock_bus.channel = 'can0'

            with patch('moteus.pythoncan_device.can.Bus', return_value=mock_bus):
                device = pythoncan_device.PythonCanDevice(
                    channel='can0',
                    fdcanusb_serial='TEST123'
                )

                self.assertEqual(device.fdcanusb_serial, 'TEST123')

    def test_pythoncan_repr_includes_serial(self):
        """Test that PythonCanDevice __repr__ includes fdcanusb serial."""
        with patch('moteus.pythoncan_device.can'):
            mock_bus = Mock()
            mock_bus.channel = 'can0'

            with patch('moteus.pythoncan_device.can.Bus', return_value=mock_bus):
                device = pythoncan_device.PythonCanDevice(
                    channel='can0',
                    fdcanusb_serial='TEST123'
                )

                repr_str = repr(device)
                self.assertIn('can0', repr_str)
                self.assertIn('TEST123', repr_str)
                self.assertIn('fdcanusb_sn', repr_str)


class WindowsMacMatchingTest(unittest.TestCase):
    """Test USB device matching for Windows/Mac."""

    def test_match_by_index(self):
        """Test matching when channel is a simple index."""
        usb_devices = [
            {'serial': 'DEV001', 'bus': 1, 'address': 10},
            {'serial': 'DEV002', 'bus': 1, 'address': 11},
            {'serial': 'DEV003', 'bus': 2, 'address': 5},
        ]

        # Test matching by index
        self.assertEqual(
            pythoncan_device._match_interface_to_usb_windows('0', usb_devices),
            'DEV001'
        )
        self.assertEqual(
            pythoncan_device._match_interface_to_usb_windows('1', usb_devices),
            'DEV002'
        )
        self.assertEqual(
            pythoncan_device._match_interface_to_usb_windows('2', usb_devices),
            'DEV003'
        )

    def test_match_single_device(self):
        """Test matching when only one device is present."""
        usb_devices = [
            {'serial': 'SINGLE_DEV', 'bus': 1, 'address': 10},
        ]

        # Any channel should match the single device
        self.assertEqual(
            pythoncan_device._match_interface_to_usb_windows('can0', usb_devices),
            'SINGLE_DEV'
        )
        self.assertEqual(
            pythoncan_device._match_interface_to_usb_windows('some_name', usb_devices),
            'SINGLE_DEV'
        )

    def test_match_by_bus_address(self):
        """Test matching when channel contains bus/address info."""
        usb_devices = [
            {'serial': 'DEV001', 'bus': 1, 'address': 10},
            {'serial': 'DEV002', 'bus': 2, 'address': 15},
            {'serial': 'DEV003', 'bus': 3, 'address': 20},
        ]

        # Test various bus/address string formats
        self.assertEqual(
            pythoncan_device._match_interface_to_usb_windows('bus_1_dev_10', usb_devices),
            'DEV001'
        )
        self.assertEqual(
            pythoncan_device._match_interface_to_usb_windows('bus2_addr15', usb_devices),
            'DEV002'
        )
        self.assertEqual(
            pythoncan_device._match_interface_to_usb_windows('can_bus 3 addr 20', usb_devices),
            'DEV003'
        )

    def test_match_no_match(self):
        """Test matching when no match is possible."""
        usb_devices = [
            {'serial': 'DEV001', 'bus': 1, 'address': 10},
            {'serial': 'DEV002', 'bus': 2, 'address': 15},
        ]

        # Non-existent index
        self.assertIsNone(
            pythoncan_device._match_interface_to_usb_windows('99', usb_devices)
        )

        # No bus/address match
        self.assertIsNone(
            pythoncan_device._match_interface_to_usb_windows('bus_99_dev_99', usb_devices)
        )

        # Ambiguous channel name with multiple devices
        self.assertIsNone(
            pythoncan_device._match_interface_to_usb_windows('can0', usb_devices)
        )

    def test_match_mac_uses_windows_logic(self):
        """Test that Mac matching uses same logic as Windows."""
        usb_devices = [
            {'serial': 'DEV001', 'bus': 1, 'address': 10},
        ]

        windows_result = pythoncan_device._match_interface_to_usb_windows('0', usb_devices)
        mac_result = pythoncan_device._match_interface_to_usb_mac('0', usb_devices)

        self.assertEqual(windows_result, mac_result)

    def test_get_usb_device_info(self):
        """Test extraction of USB device information."""
        # Create a mock USB device
        mock_device = Mock()
        mock_device.bus = 1
        mock_device.address = 10
        mock_device.idVendor = 0x1209
        mock_device.idProduct = 0x2323
        mock_device.iSerialNumber = 1
        mock_device.port_numbers = None  # Prevent port_numbers check

        with patch('usb.util.get_string', return_value='TEST123'):
            info = pythoncan_device._get_usb_device_info(mock_device)

        self.assertEqual(info['bus'], 1)
        self.assertEqual(info['address'], 10)
        self.assertEqual(info['vid'], 0x1209)
        self.assertEqual(info['pid'], 0x2323)
        self.assertEqual(info['serial'], 'TEST123')

    def test_get_usb_device_info_no_serial(self):
        """Test USB device info extraction when serial is unavailable."""
        mock_device = Mock()
        mock_device.bus = 1
        mock_device.address = 10
        mock_device.idVendor = 0x1209
        mock_device.idProduct = 0x2323
        mock_device.iSerialNumber = 0  # No serial number
        mock_device.port_numbers = None  # Prevent port_numbers check

        info = pythoncan_device._get_usb_device_info(mock_device)

        self.assertEqual(info['bus'], 1)
        self.assertEqual(info['address'], 10)
        self.assertIsNone(info['serial'])

    def test_detect_windows_mac_no_pyusb(self):
        """Test that missing pyusb is handled gracefully."""
        # Patch the actual import statement within the function
        with patch.dict('sys.modules', {'usb': None, 'usb.core': None, 'usb.util': None}):
            # Now the function will fail to import usb
            result = pythoncan_device._detect_fdcanusb_serials_windows_mac()
            self.assertEqual(result, {})

    def test_detect_windows_mac_with_devices(self):
        """Test USB device detection with mock pyusb."""
        # Clear cached devices
        pythoncan_device._cached_usb_devices = None

        mock_device1 = Mock()
        mock_device1.bus = 1
        mock_device1.address = 10
        mock_device1.idVendor = 0x1209
        mock_device1.idProduct = 0x2323
        mock_device1.iSerialNumber = 1
        mock_device1.port_numbers = None

        mock_device2 = Mock()
        mock_device2.bus = 2
        mock_device2.address = 15
        mock_device2.idVendor = 0x0483
        mock_device2.idProduct = 0x5740
        mock_device2.iSerialNumber = 1
        mock_device2.port_numbers = None

        # Create mock usb modules
        mock_usb_core = MagicMock()
        mock_usb_util = MagicMock()

        mock_usb_core.find.side_effect = [
            [mock_device2],  # Old fdcanusb
            [mock_device1],  # New fdcanusb
        ]
        mock_usb_util.get_string.side_effect = ['OLD_SERIAL', 'NEW_SERIAL']

        # Patch both the import and the actual module usage
        with patch.dict('sys.modules', {'usb.core': mock_usb_core, 'usb.util': mock_usb_util}):
            pythoncan_device._detect_fdcanusb_serials_windows_mac()

            # Check that devices were cached
            cached = pythoncan_device._cached_usb_devices
            self.assertIsNotNone(cached)
            self.assertEqual(len(cached), 2)

            # Should be sorted by bus, address (lower bus first, then lower address)
            self.assertEqual(cached[0]['bus'], 1)
            self.assertEqual(cached[0]['serial'], 'NEW_SERIAL')
            self.assertEqual(cached[1]['bus'], 2)
            self.assertEqual(cached[1]['serial'], 'OLD_SERIAL')


if __name__ == '__main__':
    unittest.main()
