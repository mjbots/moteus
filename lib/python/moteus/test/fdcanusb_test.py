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
import unittest

import moteus
import moteus.fdcanusb_device as fdcanusb
from moteus.fdcanusb_device import _append_checksum
from moteus.transport_device import TransportDevice

from moteus.test.fdcanusb_test_base import FdcanusbTestBase


class FdcanusbTest(FdcanusbTestBase):

    def test_basic_construction(self):
        async def test():
            async with moteus.Fdcanusb() as dut:
                self.assertIsNotNone(dut)
        self.run_async(test())

    def test_send_basic(self):
        async def test():
            async with fdcanusb.FdcanusbDevice() as dut:
                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                self.mock_serial.queue_response(b"OK\n")

                await dut.send_frame(frame)

                expected_cmd = b"can send 0105 30313233 b\n"
                self.assertEqual(self.mock_serial.write_data, expected_cmd)

        self.run_async(test())

    def test_send_with_flags(self):
        async def test():
            async with fdcanusb.FdcanusbDevice() as dut:
                frame = fdcanusb.Frame(0x105, b'0123', is_extended_id=True,
                                       is_fd=True, bitrate_switch=True)
                self.mock_serial.queue_response(b"OK\n")
                await dut.send_frame(frame)

                flags = sorted(self.mock_serial.write_data.decode('latin1').split(' ')[-1].strip())
                self.assertEqual(flags, ['B', 'F'])
        self.run_async(test())

    def test_transaction(self):
        async def test():
            async with fdcanusb.FdcanusbDevice() as dut:
                self.mock_serial.queue_response(b"OK\n")
                self.mock_serial.queue_response(b"OK\n")

                def queue_responses(serial, data):
                    # Send the responses in a different order from the
                    # commands.  They should get sorted out when the
                    # final results come in.
                    serial.add_response(b"rcv 0186 240400 F\n")
                    serial.add_response(b"rcv 0185 240401 F\n")

                # Send our responses after the second frame is sent.
                self.mock_serial.add_write_hook(
                    b'can send 0106', queue_responses)

                requests = [
                    TransportDevice.Request(
                        frame=fdcanusb.Frame(0x105, b'4567'),
                        frame_filter=lambda f: f.arbitration_id == 0x185),
                    TransportDevice.Request(
                        frame=fdcanusb.Frame(0x106, b'5678'),
                        frame_filter=lambda f: f.arbitration_id == 0x186),
                ]

                await dut.transaction(requests)

                self.assertIsNotNone(requests[0].responses)
                self.assertIsNotNone(requests[1].responses)
                self.assertEqual(requests[0].responses[0].arbitration_id, 0x185)
                self.assertEqual(requests[1].responses[0].arbitration_id, 0x186)

        self.run_async(test())


class FdcanusbTimeoutRetryTest(FdcanusbTestBase):
    """Tests for FdcanusbDevice timeout and retry behavior."""

    def setUp(self):
        super().setUp()
        # Simulate non-fdcanusb device for timeout behavior
        self.mock_props.return_value = {'is_fdcanusb': False}

    def test_ok_timeout_triggers_retry(self):
        """Test that OK never arriving triggers retry."""
        async def test():
            async with fdcanusb.FdcanusbDevice(
                    command_timeout=0.05, max_retries=2) as dut:
                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                # First two attempts get no response, third succeeds
                self.mock_serial.inject_response_drop(2)
                self.mock_serial.queue_response(b"OK *00\n")
                self.mock_serial.queue_response(b"OK *00\n")
                self.mock_serial.queue_response(b"OK *00\n")

                await dut.send_frame(frame)

                # Should have sent 3 commands (initial + 2 retries)
                self.assertEqual(
                    self.mock_serial.write_data.count(b'can send'),
                    3)

        self.run_async(test())

    def test_ok_checksum_invalid_retry(self):
        """Test that OK with bad checksum triggers retry."""
        async def test():
            async with fdcanusb.FdcanusbDevice(
                    command_timeout=0.1, max_retries=2) as dut:
                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                # First response has bad checksum, second is valid
                self.mock_serial.queue_response(b"OK *FF\n")  # Invalid
                self.mock_serial.queue_response(b"OK *00\n")  # Valid

                await dut.send_frame(frame)

                # Should succeed after skipping invalid checksum
                self.assertIn(b'can send', self.mock_serial.write_data)

        self.run_async(test())

    def test_transaction_timeout_retry(self):
        """Test that transaction timeout triggers retry for single requests."""
        async def test():
            async with fdcanusb.FdcanusbDevice(
                    command_timeout=0.05, max_retries=2) as dut:
                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                # First attempt times out, second succeeds
                self.mock_serial.inject_response_drop(1)
                self.mock_serial.queue_response(b"OK *00\n")
                self.mock_serial.queue_response(b"OK *00\n")

                request = TransportDevice.Request(
                    frame=frame,
                    frame_filter=None)

                await dut.transaction([request])

                # Should have retried
                self.assertGreater(
                    self.mock_serial.write_data.count(b'can send'), 1)

        self.run_async(test())

    def test_max_retries_exceeded(self):
        """Test that max retries raises after exhausting retries."""
        async def test():
            async with fdcanusb.FdcanusbDevice(
                    command_timeout=0.02, max_retries=2) as dut:
                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                # Never respond - should exhaust retries and raise
                self.mock_serial.inject_response_drop(10)

                with self.assertRaises(RuntimeError):
                    await dut.send_frame(frame)

        self.run_async(test())


class FdcanusbSupportsFlashTest(FdcanusbTestBase):
    """Tests for supports_flash property."""

    def test_fdcanusb_supports_flash(self):
        """Test that fdcanusb devices support flashing."""
        async def test():
            # Default mock returns is_fdcanusb=True
            async with fdcanusb.FdcanusbDevice() as dut:
                self.assertTrue(dut.supports_flash)
        self.run_async(test())

    def test_uart_does_not_support_flash(self):
        """Test that UART devices do not support flashing."""
        async def test():
            self.mock_props.return_value = {'is_fdcanusb': False}
            async with fdcanusb.FdcanusbDevice() as dut:
                self.assertFalse(dut.supports_flash)
        self.run_async(test())


class FdcanusbChecksumTest(FdcanusbTestBase):
    """Tests for FdcanusbDevice with checksum mode enabled."""

    def setUp(self):
        super().setUp()

        # Override this to simulate non-fdcanusb device which will
        # enable checksums to start with.
        self.mock_props.return_value = {'is_fdcanusb': False}

    def test_send_with_checksum(self):
        """Test that checksums are appended when checksum mode is active."""
        async def test():
            async with fdcanusb.FdcanusbDevice() as dut:
                self.assertTrue(dut._checksum_active)

                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                # Response must also have valid checksum (CRC of "OK " is 0x00)
                self.mock_serial.queue_response(b"OK *00\n")

                await dut.send_frame(frame)

                # Verify command has checksum in format " *XX"
                cmd = self.mock_serial.write_data
                self.assertIn(b' *', cmd)
                # Verify it ends with " *XX\n" pattern
                self.assertRegex(cmd.decode('latin1'), r' \*[0-9A-Fa-f]{2}\n$')

        self.run_async(test())

    def test_receive_with_checksum(self):
        """Test that received frames with checksums are validated."""
        async def test():
            async with fdcanusb.FdcanusbDevice() as dut:
                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                self.mock_serial.queue_response(b"OK *00\n")

                def queue_rcv(serial, data):
                    # Send rcv with valid checksum
                    rcv_content = b'rcv 00000185 240401 F'
                    rcv_with_checksum = _append_checksum(rcv_content) + b'\n'
                    serial.add_response(rcv_with_checksum)

                self.mock_serial.add_write_hook(b'can send', queue_rcv)

                request = TransportDevice.Request(
                    frame=frame,
                    frame_filter=lambda f: f.arbitration_id == 0x185)

                await dut.transaction([request])

                self.assertEqual(len(request.responses), 1)
                self.assertEqual(request.responses[0].arbitration_id, 0x185)

        self.run_async(test())

    def test_checksum_error_enables_mode(self):
        """Test that receiving a checksum error enables checksum mode."""
        async def test():
            # Start with checksum mode disabled as would happen on a
            # real fdcanusb
            self.mock_props.return_value = {'is_fdcanusb': True}

            async with fdcanusb.FdcanusbDevice() as dut:
                self.assertFalse(dut._checksum_active)

                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                # The first response is a checksum error, which will
                # enable checksum mode.
                #
                # The second response must have valid checksum since
                # mode is now enabled.
                err_text = _append_checksum(b"ERR checksum") + b'\n'
                self.mock_serial.queue_response(err_text)
                self.mock_serial.queue_response(b"OK *00\n")

                await dut.send_frame(frame)

                # After a checksum error, checkum mode should be enabled
                self.assertTrue(dut._checksum_active)

        self.run_async(test())

    def test_invalid_checksum_skipped(self):
        """Test that responses with invalid checksums are skipped."""
        async def test():
            async with fdcanusb.FdcanusbDevice() as dut:
                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                # Queue invalid checksum followed by valid one
                self.mock_serial.queue_response(b"OK *FF\n")  # Invalid
                self.mock_serial.queue_response(b"OK *00\n")  # Valid

                # Should eventually succeed after skipping invalid
                await dut.send_frame(frame)

        self.run_async(test())

    def test_missing_checksum_skipped(self):
        """Test that responses without checksums are skipped when mode
        is active."""
        async def test():
            async with fdcanusb.FdcanusbDevice() as dut:
                self.assertTrue(dut._checksum_active)

                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                # Queue a response without a checksum, then one with a
                # valid checksum
                self.mock_serial.queue_response(b"OK\n")      # Missing checksum
                self.mock_serial.queue_response(b"OK *00\n")  # Valid

                await dut.send_frame(frame)

        self.run_async(test())


class FdcanusbLineOverflowTest(FdcanusbTestBase):
    """Tests for line buffer overflow protection."""

    def setUp(self):
        super().setUp()
        self.mock_props.return_value = {'is_fdcanusb': False}

    def test_line_overflow_recovery(self):
        """Test that long lines without newline are discarded."""
        async def test():
            async with fdcanusb.FdcanusbDevice(max_line_length=50) as dut:
                frame = fdcanusb.Frame(arbitration_id=0x105, data=b'0123')

                # Queue a line that's too long (no newline), then a valid response
                long_garbage = b'x' * 100
                self.mock_serial.queue_response(long_garbage)
                self.mock_serial.queue_response(b"OK *00\n")

                # Should eventually succeed after discarding overflow
                await dut.send_frame(frame)

        self.run_async(test())


if __name__ == '__main__':
    unittest.main()
