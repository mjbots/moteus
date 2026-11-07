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


import asyncio
import unittest
import unittest.mock
import warnings

from moteus import transport_factory


class _FakeEntryPoint:
    def __init__(self, name, exc):
        self.name = name
        self._exc = exc

    def load(self):
        raise self._exc


class _FakeEntryPoints(list):
    """Mimic the sliced result of importlib_metadata.entry_points()."""

    def select(self, group):
        return self


class TransportFactoryTest(unittest.TestCase):
    def setUp(self):
        # Each test gets a clean module-level cache.
        transport_factory.TRANSPORT_FACTORIES.clear()
        transport_factory._transports_initialized = False

    def tearDown(self):
        transport_factory.TRANSPORT_FACTORIES.clear()
        transport_factory._transports_initialized = False

    def _patch_entry_points(self, eps):
        return unittest.mock.patch(
            'moteus.transport_factory.importlib_metadata.entry_points',
            return_value=_FakeEntryPoints(eps))

    def test_no_plugins_returns_builtins(self):
        with self._patch_entry_points([]):
            result = transport_factory.get_transport_factories()

        names = [f.name for f in result]
        self.assertEqual(names, ['fdcanusb', 'pythoncan'])

    def test_failing_plugin_does_not_disable_builtins(self):
        # Regression test: a plugin whose load() raises must not
        # take down the always-available built-in factories, must
        # not leave the module in a half-initialised state, and must
        # surface the failure to the user via a warning.
        bad_ep = _FakeEntryPoint('broken', ImportError('plugin failed'))

        with self._patch_entry_points([bad_ep]):
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter('always')
                result = transport_factory.get_transport_factories()

        names = [f.name for f in result]
        self.assertEqual(names, ['fdcanusb', 'pythoncan'])

        warning_messages = [str(w.message) for w in caught]
        self.assertTrue(
            any('broken' in msg for msg in warning_messages),
            f'expected a warning naming the plugin, got {warning_messages!r}')

        # And a follow-up call must still return the same factories,
        # not an empty list.
        with self._patch_entry_points([]):
            second = transport_factory.get_transport_factories()
        self.assertEqual([f.name for f in second], ['fdcanusb', 'pythoncan'])

    def test_failing_plugin_does_not_block_good_plugin(self):
        # A working plugin alongside a broken one must still register.
        class _GoodFactory:
            name = 'good'
            PRIORITY = 99
            def add_args(self, parser): pass
            def is_args_set(self, args): return False
            def __call__(self, args): return []

        bad_ep = _FakeEntryPoint('broken', RuntimeError('boom'))
        good_ep = unittest.mock.Mock()
        good_ep.name = 'good'
        good_ep.load.return_value = _GoodFactory

        with self._patch_entry_points([bad_ep, good_ep]):
            with warnings.catch_warnings():
                warnings.simplefilter('ignore')
                result = transport_factory.get_transport_factories()

        names = [f.name for f in result]
        self.assertEqual(names, ['fdcanusb', 'pythoncan', 'good'])



class _FakeDevice:
    def __init__(self):
        self.closed = False

    def parent(self):
        return None

    def close(self):
        self.closed = True


class _FakeFactory:
    PRIORITY = 1
    name = 'fake'

    def __init__(self):
        self.devices = []

    def __call__(self, args):
        device = _FakeDevice()
        self.devices.append(device)
        return [device]

    def is_args_set(self, args):
        return False


class SingletonTransportTest(unittest.TestCase):
    def setUp(self):
        self.factory = _FakeFactory()
        self.patchers = [
            unittest.mock.patch(
                'moteus.transport_factory.get_transport_factories',
                return_value=[self.factory]),
            unittest.mock.patch(
                'moteus.transport_factory.check_gui_compatibility'),
        ]
        for p in self.patchers:
            p.start()
        transport_factory.GLOBAL_TRANSPORT = None
        transport_factory.GLOBAL_TRANSPORT_LOOP = None

    def tearDown(self):
        for p in self.patchers:
            p.stop()
        transport_factory.GLOBAL_TRANSPORT = None
        transport_factory.GLOBAL_TRANSPORT_LOOP = None

    def test_same_loop_reuses_transport(self):
        async def get_two():
            return (transport_factory.get_singleton_transport(),
                    transport_factory.get_singleton_transport())

        first, second = asyncio.run(get_two())
        self.assertIs(first, second)
        self.assertEqual(len(self.factory.devices), 1)

    def test_new_transport_after_event_loop_finishes(self):
        # Transport devices are tied to the event loop they were
        # created on, so a program that calls asyncio.run() more than
        # once needs a new transport each time.
        async def get():
            return transport_factory.get_singleton_transport()

        first = asyncio.run(get())
        second = asyncio.run(get())

        self.assertIsNot(first, second)
        self.assertEqual(len(self.factory.devices), 2)
        self.assertTrue(self.factory.devices[0].closed)
        self.assertFalse(self.factory.devices[1].closed)

    def test_other_running_loop_keeps_transport(self):
        # If the loop the transport was created on is still running
        # (in another thread, say), keep returning it rather than
        # closing it out from under that loop.
        import threading

        async def get():
            return transport_factory.get_singleton_transport()

        other_loop = asyncio.new_event_loop()
        thread = threading.Thread(target=other_loop.run_forever)
        thread.start()
        try:
            first = asyncio.run_coroutine_threadsafe(get(), other_loop).result()
            second = asyncio.run(get())
        finally:
            other_loop.call_soon_threadsafe(other_loop.stop)
            thread.join()
            other_loop.close()

        self.assertIs(first, second)
        self.assertFalse(self.factory.devices[0].closed)


if __name__ == '__main__':
    unittest.main()
