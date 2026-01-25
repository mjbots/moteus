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

"""Minimal asyncio.timeout replacement for nested timeout handling.

This module provides a timeout context manager that correctly handles
nested timeouts, unlike asyncio.wait_for which has cancellation
semantics that don't compose well.

The implementation uses Task.cancel() with a unique message to distinguish
timeout cancellation from external cancellation, similar to Python 3.11+'s
asyncio.timeout().
"""

import asyncio
import sys


# Unique sentinel to identify our timeout cancellations
_TIMEOUT_SENTINEL = "_moteus_timeout_expired"


class Timeout:
    """Async context manager for operation timeouts.

    Usage:
        async with timeout(0.5):
            await some_operation()

    Raises asyncio.TimeoutError if the deadline is exceeded.
    """

    def __init__(self, delay):
        """Create a timeout context.

        Args:
            delay: Timeout in seconds. None means no timeout.
        """
        self._delay = delay
        self._task = None
        self._timeout_handle = None
        self._cancelled_by_timeout = False

    async def __aenter__(self):
        if self._delay is None:
            return self

        self._task = asyncio.current_task()
        if self._task is None:
            raise RuntimeError("timeout() must be used inside a task")

        loop = asyncio.get_running_loop()
        self._timeout_handle = loop.call_later(
            self._delay, self._on_timeout)

        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        # Cancel the timeout callback if still pending
        if self._timeout_handle is not None:
            self._timeout_handle.cancel()
            self._timeout_handle = None

        if exc_type is asyncio.CancelledError:
            # Check if this cancellation was from our timeout
            if sys.version_info >= (3, 9):
                # On 3.9+, check the cancel message for our sentinel
                # to avoid misattributing an external cancel as a timeout.
                if (getattr(exc_val, 'args', ()) and
                    exc_val.args[0] == _TIMEOUT_SENTINEL):
                    raise asyncio.TimeoutError() from exc_val
            elif self._cancelled_by_timeout:
                # Pre-3.9, best effort using the flag
                self._cancelled_by_timeout = False
                raise asyncio.TimeoutError() from exc_val

        return False

    def _on_timeout(self):
        """Called when the timeout expires."""
        self._cancelled_by_timeout = True
        # Use cancel message if available (Python 3.9+)
        if sys.version_info >= (3, 9):
            self._task.cancel(_TIMEOUT_SENTINEL)
        else:
            self._task.cancel()


def timeout(delay):
    """Create a timeout context manager.

    Args:
        delay: Timeout in seconds. None means no timeout.

    Returns:
        Async context manager that raises asyncio.TimeoutError if
        the operation exceeds the timeout.

    Example:
        async with timeout(1.0):
            await slow_operation()
    """
    return Timeout(delay)
