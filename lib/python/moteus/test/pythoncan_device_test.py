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

"""Tests for the bitrates PythonCanDevice selects by default."""

import os
import unittest
from unittest.mock import Mock, patch

import can

import moteus.pythoncan_device as pythoncan_device


class DefaultBitratesTest(unittest.TestCase):
    def setUp(self):
        patchers = [
            patch.object(pythoncan_device, 'can', can),
            patch.object(can, 'rc', {'fd': True}),
        ]
        for patcher in patchers:
            patcher.start()
            self.addCleanup(patcher.stop)

    def apply(self, **kwargs):
        pythoncan_device._apply_default_bitrates(kwargs)
        return kwargs

    def test_bitrate_interfaces(self):
        # Everything not otherwise special, including third party
        # plugins, gets plain bitrates.
        for interface in ['socketcan', 'ixxat', 'etas', 'neovi', 'gs_usb',
                          'usbtingo', 'unknown_plugin']:
            self.assertEqual(
                self.apply(interface=interface),
                {'interface': interface,
                 'bitrate': 1000000,
                 'data_bitrate': 5000000},
                interface)

    def test_timing_interfaces(self):
        for interface in ['pcan', 'kvaser', 'vector', 'nixnet', 'slcan',
                          'cansub']:
            result = self.apply(interface=interface)
            self.assertEqual(set(result.keys()), {'interface', 'timing'},
                             interface)

            timing = result['timing']
            self.assertIsInstance(timing, can.BitTimingFd)
            self.assertEqual(timing.nom_bitrate, 1000000)
            self.assertEqual(timing.data_bitrate, 5000000)
            self.assertAlmostEqual(timing.nom_sample_point, 66.6, delta=5.0)
            self.assertAlmostEqual(timing.data_sample_point, 66.6, delta=5.0)

    def test_timing_interfaces_are_known(self):
        # Each interface we special case must exist, and must accept a
        # 'timing' argument.
        import ast
        import importlib.util

        for interface in pythoncan_device._FD_TIMING_F_CLOCK:
            if interface not in can.interfaces.BACKENDS:
                # A third party plugin.
                continue

            module_name, class_name = can.interfaces.BACKENDS[interface]

            # Parse instead of importing, as importing requires vendor
            # libraries to be present.
            spec = importlib.util.find_spec(module_name)
            sources = (
                [spec.origin] if not spec.submodule_search_locations else
                [os.path.join(d, f)
                 for d in spec.submodule_search_locations
                 for f in os.listdir(d) if f.endswith('.py')])

            args = None
            for source in sources:
                with open(source) as f:
                    tree = ast.parse(f.read())
                for node in ast.walk(tree):
                    if (isinstance(node, ast.ClassDef) and
                        node.name == class_name):
                        init = [x for x in node.body
                                if isinstance(x, ast.FunctionDef) and
                                x.name == '__init__'][0]
                        args = [x.arg for x in
                                init.args.args + init.args.kwonlyargs]
            self.assertIsNotNone(args, interface)
            self.assertIn('timing', args, interface)

    def test_candle(self):
        self.assertEqual(
            self.apply(interface='candle'),
            {'interface': 'candle',
             'bitrate': 1000000, 'data_bitrate': 5000000,
             'sample_point': 66, 'data_sample_point': 66})
        self.assertEqual(
            self.apply(interface='candle', fd=False),
            {'interface': 'candle', 'fd': False,
             'bitrate': 1000000, 'sample_point': 66})

        kwargs = {'interface': 'candle', 'sample_point': 80.0}
        self.assertEqual(self.apply(**kwargs), kwargs)

    def test_interface_from_rc(self):
        can.rc['interface'] = 'pcan'
        self.assertIn('timing', self.apply())

    def test_fd_disabled(self):
        self.assertEqual(
            self.apply(interface='pcan', fd=False),
            {'interface': 'pcan', 'fd': False, 'bitrate': 1000000})

        can.rc['fd'] = False
        self.assertEqual(
            self.apply(interface='cansub'),
            {'interface': 'cansub', 'bitrate': 1000000})
        self.assertEqual(
            self.apply(interface='etas'),
            {'interface': 'etas', 'bitrate': 1000000})

    def test_user_options_win(self):
        for options in [
                {'bitrate': 500000},
                {'data_bitrate': 2000000},
                {'timing': Mock()},
                {'f_clock': 80000000, 'nom_brp': 1},
                {'tseg1_abr': 6},
                {'fd_bitrate': 2000000},
        ]:
            for interface in ['pcan', 'cansub']:
                kwargs = dict(interface=interface, **options)
                self.assertEqual(self.apply(**kwargs), kwargs)

    def test_rc_options_win(self):
        # As when set in a python-can configuration file.
        can.rc['bitrate'] = 500000
        self.assertEqual(self.apply(interface='pcan'), {'interface': 'pcan'})
        self.assertEqual(self.apply(interface='cansub'),
                         {'interface': 'cansub'})

    def test_device_passes_defaults_to_bus(self):
        with patch.object(can, 'Bus') as bus:
            pythoncan_device.PythonCanDevice(
                interface='etas', channel='test')
            self.assertEqual(
                bus.call_args.kwargs,
                {'interface': 'etas', 'channel': 'test',
                 'bitrate': 1000000, 'data_bitrate': 5000000})

            pythoncan_device.PythonCanDevice(
                interface='cansub', channel='test')
            self.assertIsInstance(
                bus.call_args.kwargs['timing'], can.BitTimingFd)

            pythoncan_device.PythonCanDevice(
                interface='cansub', channel='test', bitrate=250000)
            self.assertEqual(
                bus.call_args.kwargs,
                {'interface': 'cansub', 'channel': 'test',
                 'bitrate': 250000})


if __name__ == '__main__':
    unittest.main()
