#!/usr/bin/python3 -B

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

'''Interactively display and update values from an embedded device.
'''

import argparse
import ast
import asyncio
from dataclasses import dataclass
import io
import moteus
import moteus.moteus_tool
import numpy
import os
import re
import signal
import struct
import sys
import time
import traceback
import matplotlib
import matplotlib.figure

try:
    import PySide6
    from PySide6 import QtGui

    os.environ['PYSIDE_DESIGNER_PLUGINS'] = os.path.dirname(os.path.abspath(__file__))
    os.environ['QT_API'] = 'PySide6'
    from PySide6 import QtUiTools
except ImportError:
    import PySide2
    from PySide2 import QtGui
    os.environ['QT_API'] = 'pyside2'
    from PySide2 import QtUiTools


from matplotlib.backends import backend_qt5agg
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
qt_backend = matplotlib.backends.backend_qt5agg


import qtconsole
from qtconsole.history_console_widget import HistoryConsoleWidget
if getattr(qtconsole, "qt", None):
    from qtconsole.qt import QtCore, QtGui
    QtWidgets = QtGui
else:
    from qtpy import QtCore, QtGui, QtWidgets

# Why this is necessary and not just the default, I don't know, but
# otherwise we get a warning about "Qt WebEngine seems to be
# initialized from a plugin..."
QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_ShareOpenGLContexts)

if os.environ['QT_API'] == 'pyside6':
    # Something can override our request.  Put it back to the correct
    # capitalization for asyncqt.
    os.environ['QT_API'] = 'PySide6'

import asyncqt

import moteus.reader as reader
import moteus.multiplex
from moteus import Register, Mode


# Fault monitoring configuration
FAULT_POLLING_INTERVAL_MS = 500

# Fault code descriptions from docs/reference.md
FAULT_CODE_DESCRIPTIONS = {
    32: "calibration fault",
    33: "motor driver fault",
    34: "over voltage",
    35: "encoder fault",
    36: "motor not configured",
    37: "pwm cycle overrun",
    38: "over temperature",
    39: "outside limit",
    40: "under voltage",
    41: "config changed",
    42: "theta invalid",
    43: "position invalid",
    44: "driver enable fault",
    45: "stop position deprecated",
    46: "timing violation",
    47: "bemf feedforward without accel",
    48: "invalid limits",
    96: "limit: servo.max_velocity",
    97: "limit: servo.max_power_W",
    98: "limit: max system voltage",
    99: "limit: servo.max_current_A",
    100: "limit: servo.fault_temperature",
    101: "limit: servo.motor_fault_temperature",
    102: "limit: commanded max torque",
    103: "limit: servopos limit",
}


@dataclass
class FaultState:
    """Tracks fault status and user observation for a device."""
    is_faulted: bool = False
    observed: bool = True  # Default to observed (no flashing for non-faulty devices)
    current_mode: int = None  # Current mode register value
    current_fault_code: int = None  # Current fault register value


try:
    from . import version
except ImportError:
    class Version:
        VERSION = 'dev'
    version = Version()


LEFT_LEGEND_LOC = 3
RIGHT_LEGEND_LOC = 2

DEFAULT_RATE = 100
MAX_HISTORY_SIZE = 100
POLL_TIMEOUT_S = 0.1
STARTUP_TIMEOUT_S = 0.5

FORMAT_ROLE = QtCore.Qt.UserRole + 1

FMT_STANDARD = 0
FMT_HEX = 1


class CommandError(RuntimeError):
    def __init__(self, cmd, err):
        super(CommandError, self).__init__(f'CommandError: "{cmd}" => "{err}"')


def _has_nonascii(data):
    return any([ord(x) > 127 for x in data])


def calculate_optimal_uuid_prefix(full_uuid, other_uuids):
    """Pure function to find the shortest unique UUID prefix.

    Args:
        full_uuid: The full 16-byte UUID to find a prefix for
        other_uuids: List of other UUIDs to check conflicts against

    Returns:
        The shortest unique prefix (4, 8, 12, or 16 bytes)
    """
    for prefix_len in [4, 8, 12, 16]:
        proposed_prefix = full_uuid[:prefix_len]

        # Check if this prefix conflicts with any other UUID
        has_conflict = any(
            other_uuid[:prefix_len] == proposed_prefix
            for other_uuid in other_uuids
        )

        if not has_conflict:
            return proposed_prefix

    # Should never happen, but return full UUID as fallback
    return full_uuid


# TODO jpieper: Factor these out of tplot.py
def _get_data(value, name):
    fields = name.split('.')
    for field in fields:
        if isinstance(value, list):
            value = value[int(field)]
        else:
            value = getattr(value, field)
    return value


def _add_schema_item(parent, element, terminal_flags=None):
    # Cache our schema, so that we can use it for things like
    # generating better input options.
    parent.setData(1, QtCore.Qt.UserRole, element)

    if isinstance(element, reader.ObjectType):
        for field in element.fields:
            name = field.name

            item = QtWidgets.QTreeWidgetItem(parent)
            item.setText(0, name)

            _add_schema_item(item, field.type_class,
                             terminal_flags=terminal_flags)
    else:
        if terminal_flags:
            parent.setFlags(terminal_flags)

def _is_servo_stats_fault_field(item):
    """Check if the tree widget item represents a servo_stats.fault field."""
    # Check if this is a leaf node (has no children and is displaying a value)
    if item.childCount() > 0:
        return False

    # Get the field name
    field_name = item.text(0).lower()
    if field_name != "fault":
        return False

    # Check if parent is servo_stats
    parent = item.parent()
    if parent is None:
        return False

    parent_name = parent.text(0).lower()
    return parent_name == "servo_stats"

def _format_fault_code(fault_code):
    """Format a fault code with human-readable description.

    Args:
        fault_code: Integer fault code (can be None)

    Returns:
        str: Formatted fault code string, "0" for no fault, empty string for None
    """
    if fault_code is None:
        return ""

    if fault_code == 0:
        return "0"

    description = FAULT_CODE_DESCRIPTIONS.get(fault_code, "unknown fault code")
    return f"{fault_code} ({description})"

def _set_tree_widget_data(item, struct, element, terminal_flags=None):
    if (isinstance(element, reader.ObjectType) or
        isinstance(element, reader.ArrayType) or
        isinstance(element, reader.FixedArrayType)):
        if not isinstance(element, reader.ObjectType):
            for i in range(item.childCount(), len(struct)):
                subitem = QtWidgets.QTreeWidgetItem(item)
                subitem.setText(0, str(i))
                _add_schema_item(subitem, element.type_class,
                                 terminal_flags=terminal_flags)
        for i in range(item.childCount()):
            child = item.child(i)
            if isinstance(struct, list):
                field = struct[i]
                child_element = element.type_class
            else:
                name = child.text(0)
                field = getattr(struct, name)
                child_element = element.fields[i].type_class
            _set_tree_widget_data(child, field, child_element,
                                  terminal_flags=terminal_flags)
    else:
        maybe_format = item.data(1, FORMAT_ROLE)
        text = None
        if maybe_format == FMT_HEX and type(struct) == int:
            text = f"{struct:x}"
        elif _is_servo_stats_fault_field(item) and isinstance(struct, int):
            # Special formatting for servo_stats.fault field
            text = _format_fault_code(struct)
        else:
            text = repr(struct)
        item.setText(1, text)


def _console_escape(value):
    if '\x00' in value:
        return value.replace('\x00', '*')
    return value


class RecordSignal(object):
    def __init__(self):
        self._index = 0
        self._callbacks = {}

    def connect(self, handler):
        result = self._index
        self._index += 1
        self._callbacks[result] = handler

        class Connection(object):
            def __init__(self, parent, index):
                self.parent = parent
                self.index = index

            def remove(self):
                del self.parent._callbacks[self.index]

        return Connection(self, result)

    def update(self, value, now):
        for handler in self._callbacks.values():
            handler(value, now)
        return len(self._callbacks) != 0


class PlotItem(object):
    def __init__(self, axis, plot_widget, name, signal):
        self.axis = axis
        self.plot_widget = plot_widget
        self.name = name
        self.line = None
        self.xdata = []
        self.ydata = []
        self.connection = signal.connect(self._handle_update)

    def _make_line(self):
        line = matplotlib.lines.Line2D([], [])
        line.set_label(self.name)
        line.set_color(self.plot_widget.COLORS[self.plot_widget.next_color])
        self.plot_widget.next_color = (
            self.plot_widget.next_color + 1) % len(self.plot_widget.COLORS)

        self.axis.add_line(line)
        self.axis.legend(loc=self.axis.legend_loc)

        self.line = line

    def remove(self):
        self.line.remove()
        self.connection.remove()
        # NOTE jpieper: matplotlib gives us no better way to remove a
        # legend.
        if len(self.axis.lines) == 0:
            self.axis.legend_ = None
            self.axis.relim()
            self.axis.autoscale()
        else:
            self.axis.legend(loc=self.axis.legend_loc)
        self.plot_widget.canvas.draw()

    def _handle_update(self, value, now):
        if self.plot_widget.paused:
            return

        if self.line is None:
            self._make_line()

        self.xdata.append(now)
        self.ydata.append(value)

        # Remove elements from the beginning until there is at most
        # one before the window.
        oldest_time = now - self.plot_widget.history_s
        oldest_index = None
        for i in range(len(self.xdata)):
            if self.xdata[i] >= oldest_time:
                oldest_index = i - 1
                break

        if oldest_index and oldest_index > 1:
            self.xdata = self.xdata[oldest_index:]
            self.ydata = self.ydata[oldest_index:]

        self.line.set_data(self.xdata, self.ydata)

        self.axis.relim()
        self.axis.autoscale()

        self.plot_widget.data_update()


class PlotWidget(QtWidgets.QWidget):
    COLORS = 'rbgcmyk'

    def __init__(self, *args, **kwargs):
        QtWidgets.QWidget.__init__(self, *args, **kwargs)

        self.history_s = 20.0
        self.next_color = 0
        self.paused = False

        self.last_draw_time = 0.0

        self.figure = matplotlib.figure.Figure()
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumSize(10, 10)

        self.canvas.mpl_connect('key_press_event', self.handle_key_press)
        self.canvas.mpl_connect('key_release_event', self.handle_key_release)

        self.left_axis = self.figure.add_subplot(111)
        self.left_axis.grid()
        self.left_axis.fmt_xdata = lambda x: '%.3f' % x

        self.left_axis.legend_loc = LEFT_LEGEND_LOC

        self.right_axis = None

        self.toolbar = qt_backend.NavigationToolbar2QT(self.canvas, self)
        self.pause_action = QtWidgets.QAction(u'Pause', self)
        self.pause_action.setCheckable(True)
        self.pause_action.toggled.connect(self._handle_pause)
        self.toolbar.addAction(self.pause_action)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.toolbar, 0)
        layout.addWidget(self.canvas, 1)

        self.canvas.setFocusPolicy(QtCore.Qt.ClickFocus)

    def _handle_pause(self, value):
        self.paused = value

    def add_plot(self, name, signal, axis_number):
        axis = self.left_axis
        if axis_number == 1:
            if self.right_axis is None:
                self.right_axis = self.left_axis.twinx()
                self.right_axis.legend_loc = RIGHT_LEGEND_LOC
            axis = self.right_axis
        item = PlotItem(axis, self, name, signal)
        return item

    def remove_plot(self, item):
        item.remove()

    def data_update(self):
        now = time.time()
        elapsed = now - self.last_draw_time
        if elapsed > 0.1:
            self.last_draw_time = now
            self.canvas.draw()

    def _get_axes_keys(self):
        result = []
        result.append(('1', self.left_axis))
        if self.right_axis:
            result.append(('2', self.right_axis))
        return result

    def handle_key_press(self, event):
        if event.key not in ['1', '2']:
            return
        for key, axis in self._get_axes_keys():
            if key == event.key:
                axis.set_navigate(True)
            else:
                axis.set_navigate(False)

    def handle_key_release(self, event):
        if event.key not in ['1', '2']:
            return
        for key, axis in self._get_axes_keys():
            axis.set_navigate(True)


class SizedTreeWidget(QtWidgets.QTreeWidget):
    def __init__(self, parent=None):
        QtWidgets.QTreeWidget.__init__(self, parent)
        self.setColumnCount(2)
        self.headerItem().setText(0, 'Name')
        self.headerItem().setText(1, 'Value')

    def sizeHint(self):
        return QtCore.QSize(350, 500)


class TviewConsoleWidget(HistoryConsoleWidget):
    line_input = QtCore.Signal(str)

    def __init__(self, *args, **kw):
        super(TviewConsoleWidget, self).__init__(*args, **kw)

        self.execute_on_complete_input = False
        self._prompt = '>>> '
        self.clear()

        # The bionic version of ConsoleWidget seems to get the cursor
        # position screwed up after a clear.  Let's just fix it up
        # here.
        self._append_before_prompt_cursor.setPosition(0)

    def sizeHint(self):
        return QtCore.QSize(600, 200)

    def add_text(self, data):
        assert data.endswith('\n') or data.endswith('\r')
        self._append_plain_text(_console_escape(data), before_prompt=True)
        self._control.moveCursor(QtGui.QTextCursor.End)

    def _handle_timeout(self):
        self._append_plain_text('%s\r\n' % time.time(),
                                before_prompt=True)
        self._control.moveCursor(QtGui.QTextCursor.End)

    def _is_complete(self, source, interactive):
        return True, False

    def _execute(self, source, hidden):
        self.line_input.emit(source)
        self._show_prompt(self._prompt)
        return True


class TviewPythonConsole(HistoryConsoleWidget):
    def __init__(self, parent=None, get_controller=None):
        super().__init__(parent)

        self.execute_on_complete_input = False
        self._prompt = '>>> '
        self.clear()

        self._append_before_prompt_cursor.setPosition(0)

        # Track currently running async task for cancellation
        self._current_future = None

        # Create custom print function for this console
        def custom_print(*args, sep=' ', end='\n', file=None, flush=False):
            """Custom print function that outputs to the Python console widget."""
            output = io.StringIO()
            print(*args, sep=sep, end=end, file=output)
            self._append_plain_text(output.getvalue())

        self.namespace = {
            'transport': None,
            'controller': None,
            'get_controller': get_controller,
            'asyncio': asyncio,
            'moteus': moteus,
            'time': time,
            'numpy': numpy,
            'print': custom_print,  # Custom print for console output
        }

        # Install event filter to catch key events
        self._control.installEventFilter(self)

        # Also try using a QShortcut for Ctrl+C
        try:
            try:
                from PySide6.QtGui import QShortcut, QKeySequence
            except ImportError:
                from PySide6.QtWidgets import QShortcut
                from PySide6.QtGui import QKeySequence
            # PySide6 style
            self._interrupt_shortcut = QShortcut(QKeySequence("Ctrl+C"), self._control)
            self._interrupt_shortcut.activated.connect(self._handle_interrupt)
        except ImportError:
            try:
                from PySide2.QtWidgets import QShortcut
                from PySide2.QtGui import QKeySequence
                # PySide2 requires a lambda wrapper for the callable
                self._interrupt_shortcut = QShortcut(QKeySequence("Ctrl+C"), self._control, lambda: self._handle_interrupt())
            except Exception as e:
                print(f"Warning: Could not set up Ctrl+C shortcut: {e}")
                self._interrupt_shortcut = None

        for line in """
# Python REPL for moteus control
# Available: controller, get_controller(id), transport, asyncio, moteus
# Use 'await' for async operations
# Press Ctrl+C to interrupt long-running operations
""".split('\n'):
            self._append_plain_text(line + '\n')

        self._show_prompt(self._prompt)

    def sizeHint(self):
        return QtCore.QSize(600, 200)

    def _handle_interrupt(self):
        """Handle interrupt signal from QShortcut."""
        if self._current_future and not self._current_future.done():
            cancelled = self._current_future.cancel()
            if cancelled:
                self._append_plain_text('\nKeyboardInterrupt\n')
                self._show_prompt(self._prompt)
            self._current_future = None

    def eventFilter(self, obj, event):
        """Event filter to catch key events before they're processed."""
        if event.type() == QtCore.QEvent.KeyPress:
            # Check for Ctrl+C
            if (event.key() == QtCore.Qt.Key_C and
                event.modifiers() == QtCore.Qt.ControlModifier):
                if self._current_future and not self._current_future.done():
                    cancelled = self._current_future.cancel()
                    if cancelled:
                        self._append_plain_text('\nKeyboardInterrupt\n')
                        self._show_prompt(self._prompt)
                    self._current_future = None
                    return True  # Event handled, don't propagate
        return super().eventFilter(obj, event)

    def keyPressEvent(self, event):
        """Handle key press events, including Ctrl+C for interruption."""
        # Check for Ctrl+C
        if (event.key() == QtCore.Qt.Key_C and
            event.modifiers() == QtCore.Qt.ControlModifier):
            if self._current_future and not self._current_future.done():
                cancelled = self._current_future.cancel()
                if cancelled:
                    self._append_plain_text('\nKeyboardInterrupt\n')
                    self._show_prompt(self._prompt)
                self._current_future = None
                return

        # Pass other key events to parent
        super().keyPressEvent(event)

    def _is_complete(self, source, interactive):
        return True, False

    def _has_await_expression(self, source):
        """Check if source code contains await expressions using AST parsing."""
        try:
            tree = ast.parse(source)
            for node in ast.walk(tree):
                if isinstance(node, ast.Await):
                    return True
            return False
        except SyntaxError:
            # If it doesn't parse, we'll handle the error later
            return False

    def _execute(self, source, hidden):
        if not source.strip():
            self._show_prompt(self._prompt)
            return True

        loop = asyncio.get_event_loop()

        try:
            # Check if this is an async expression using AST analysis.
            if self._has_await_expression(source):
                # Determine if this is an expression or statement using AST
                # We can't use compile() because await expressions fail in eval mode
                is_expression = False
                try:
                    tree = ast.parse(source)
                    # Check if it's a single expression statement
                    if (len(tree.body) == 1 and
                        isinstance(tree.body[0], ast.Expr)):
                        is_expression = True
                except SyntaxError:
                    pass

                # Wrap in an async function appropriately
                if is_expression:
                    # For expressions, we can return the value
                    indented = '\n'.join('    ' + line for line in source.split('\n'))
                    wrapped = f"async def _async_exec():\n    return (\n{indented}\n    )"
                else:
                    # For statements, just execute them
                    indented = '\n'.join('    ' + line for line in source.split('\n'))
                    wrapped = f"async def _async_exec():\n{indented}\n    return None"

                exec(wrapped, self.namespace)
                # Now, run the function we just evaluated.
                # Use create_task for better cancellation support
                self._current_future = asyncio.create_task(self.namespace['_async_exec']())

                def done_callback(future):
                    try:
                        result = future.result()
                        if result is not None:
                            self._append_plain_text(repr(result) + '\n')
                    except asyncio.CancelledError:
                        # Don't show anything extra - already handled in keyPressEvent
                        pass
                    except Exception as e:
                        self._append_plain_text(f'Error: {type(e).__name__}: {e}\n')
                    finally:
                        # Clear the current future reference
                        if self._current_future is future:
                            self._current_future = None

                    # Only show prompt if not cancelled (already shown in keyPressEvent)
                    if not future.cancelled():
                        self._show_prompt(self._prompt)

                self._current_future.add_done_callback(done_callback)
            else:
                # Try as expression first, then as a statement.
                try:
                    result = eval(source, self.namespace)
                    if result is not None:
                        self._append_plain_text(repr(result) + '\n')
                except SyntaxError:
                    # Try as a statement.
                    exec(source, self.namespace)

                self._show_prompt(self._prompt)
        except Exception as e:
            self._append_plain_text(f'Error: {type(e).__name__}: {e}\n')
            self._show_prompt(self._prompt)

        return True


class TviewTabbedConsole(QtWidgets.QTabWidget):
    def __init__(self, parent=None, get_controller=None):
        super().__init__(parent)

        self.diagnostic_console = TviewConsoleWidget()
        self.diagnostic_console.ansi_codes = False

        self.addTab(self.diagnostic_console, "Diagnostic")

        self.python_console = TviewPythonConsole(
            get_controller=get_controller)
        self.addTab(self.python_console, "Python")

    def sizeHint(self):
        return QtCore.QSize(600, 200)

class Record:
    def __init__(self, archive):
        self.archive = archive
        self.tree_item = None
        self.signals = {}
        self.history = []

    def get_signal(self, name):
        if name not in self.signals:
            self.signals[name] = RecordSignal()

        return self.signals[name]

    def update(self, struct, now):
        count = 0
        self.history.append(struct)
        if len(self.history) > MAX_HISTORY_SIZE:
            self.history = self.history[1:]

        for key, signal in self.signals.items():
            if key.startswith('__STDDEV_'):
                remaining = key.split('__STDDEV_')[1]
                values = [_get_data(x, remaining) for x in self.history]
                value = numpy.std(values)
            elif key.startswith('__MEAN_'):
                remaining = key.split('__MEAN_')[1]
                values = [_get_data(x, remaining) for x in self.history]
                value = numpy.mean(values)
            else:
                value = _get_data(struct, key)
            if signal.update(value, now):
                count += 1
        return count != 0


class NoEditDelegate(QtWidgets.QStyledItemDelegate):
    def __init__(self, parent=None):
        QtWidgets.QStyledItemDelegate.__init__(self, parent=parent)

    def createEditor(self, parent, option, index):
        return None


class EditDelegate(QtWidgets.QStyledItemDelegate):
    def __init__(self, parent=None):
        QtWidgets.QStyledItemDelegate.__init__(self, parent=parent)

    def createEditor(self, parent, option, index):
        maybe_schema = index.data(QtCore.Qt.UserRole)

        if (maybe_schema is not None and
            (isinstance(maybe_schema, reader.EnumType) or
             isinstance(maybe_schema, reader.BooleanType))):
            editor = QtWidgets.QComboBox(parent)

            if isinstance(maybe_schema, reader.EnumType):
                options = list(maybe_schema.enum_class)
                options_text = [repr(x) for x in options]
                editor.setEditable(True)
            elif isinstance(maybe_schema, reader.BooleanType):
                options_text = ['False', 'True']
                editor.activated.connect(self.commitAndCloseEditor)

            editor.insertItems(0, options_text)

            return editor
        else:
            return super(EditDelegate, self).createEditor(parent, option, index)


    def commitAndCloseEditor(self):
        editor = self.sender()

        self.commitData.emit(editor)
        self.closeEditor.emit(editor)


def _get_item_name(item):
    name = item.text(0)
    while item.parent() and item.parent().parent():
        name = item.parent().text(0) + '.' + name
        item = item.parent()

    return name


def _get_item_root(item):
    while item.parent().parent():
        item = item.parent()
    return item.text(0)


class DeviceStream:
    def __init__(self, transport, controller):
        self._write_data = b''
        self._read_data = b''
        self.transport = transport
        self.controller = controller

        self._read_condition = asyncio.Condition()

        self.emit_count = 0
        self.poll_count = 0

    def update_controller(self, controller):
        self.controller = controller

    def ignore_all(self):
        self._read_data = b''

    def write(self, data):
        self._write_data += data

    async def poll(self):
        self.poll_count += 1
        await self.transport.write(self.controller.make_diagnostic_read())

    async def maybe_emit_one(self):
        if len(self._write_data) == 0:
            return

        self.emit_count += 1

        max_send = self.controller.max_diagnostic_write
        to_write, self._write_data = (
            self._write_data[0:max_send], self._write_data[max_send:])
        await self.transport.write(self.controller.make_diagnostic_write(to_write))

    async def process_message(self, message):
        data = message.data

        if len(data) < 3:
            return False

        if data[0] != 0x41:
            return False
        if data[1] != 1:
            return False
        if data[2] > 61:
            return False
        datalen = data[2]
        if datalen > (len(data) - 3):
            return False

        self._read_data += data[3:3+datalen]

        async with self._read_condition:
            self._read_condition.notify_all()

        return datalen > 0

    def _read_maybe_empty_line(self):
        first_newline = min((self._read_data.find(c) for c in b'\r\n'
                             if c in self._read_data), default=None)
        if first_newline is None:
            return
        to_return, self._read_data = (
            self._read_data[0:first_newline+1],
            self._read_data[first_newline+1:])
        return to_return

    async def readline(self):
        while True:
            maybe_line = self._read_maybe_empty_line()
            if maybe_line:
                maybe_line = maybe_line.rstrip()
                if len(maybe_line) > 0:
                    return maybe_line
            async with self._read_condition:
                await self._read_condition.wait()

    async def resynchronize(self):
        while True:
            oldlen = len(self._read_data)
            async with self._read_condition:
                await self._read_condition.wait()
            newlen = len(self._read_data)
            if newlen == oldlen:
                self._read_data = b''
                return

    async def read_sized_block(self):
        while True:
            if len(self._read_data) >= 5:
                size = struct.unpack('<I', self._read_data[1:5])[0]
                if size > 2 ** 24:
                    return False

                if len(self._read_data) >= (5 + size):
                    block = self._read_data[5:5+size]
                    self._read_data = self._read_data[5+size:]
                    return block

            async with self._read_condition:
                await self._read_condition.wait()


class Device:
    STATE_LINE = 0
    STATE_CONFIG = 1
    STATE_TELEMETRY = 2
    STATE_SCHEMA = 3
    STATE_DATA = 4

    def __init__(self, address,
                 source_can_id,
                 python_source_can_id,
                 transport, console, prefix,
                 config_tree_item, data_tree_item,
                 can_prefix, main_window, can_id):
        self.error_count = 0
        self.poll_count = 0
        self.poll_lock = asyncio.Lock()  # Lock for poll-address change synchronization

        self.address = address
        self.source_can_id = source_can_id
        self.python_source_can_id = python_source_can_id
        self._can_prefix = can_prefix

        # We keep around an estimate of the current CAN ID to enable
        # user friendly commands.
        self.can_id = can_id

        self.controller = moteus.Controller(
            address,
            source_can_id=source_can_id,
            can_prefix=can_prefix)
        self._transport = transport
        self._stream = DeviceStream(transport, self.controller)
        self._main_window = main_window

        self._console = console
        self._prefix = prefix
        self._config_tree_item = config_tree_item
        self._data_tree_item = data_tree_item

        self._telemetry_records = {}
        self._schema_name = None
        self._config_tree_items = {}
        self._config_callback = None

        self._events = {}
        self._data_update_time = {}
        self._data = {}

        self._updating_config = False

        # Fault monitoring state
        self.fault_state = FaultState()

    async def start(self):
        # Stop the spew.
        self.write('\r\ntel stop\r\n'.encode('latin1'))

        # Make sure we've actually had a chance to write and poll.
        while self._stream.poll_count < 5 or self._stream.emit_count < 1:
            await asyncio.sleep(0.2)

        self._stream.ignore_all()

        # Make sure we have a UUID based address available in case we
        # need it later.
        if isinstance(self.address, int) or self.address.can_id is not None:
            self.uuid_address = await self._get_uuid_address()
        else:
            self.uuid_address = self.address

        # Save the full UUID for later.
        self.full_uuid = (self.uuid_address.uuid
                          if self.uuid_address
                          else None)

        # Are we able to be addressed by UUID?
        has_uuid_capability = self.uuid_address is not None

        # Register UUID query completion with main window
        await self._main_window.register_uuid_query_complete(
            self, has_uuid_capability)

        await self.update_config()
        await self.update_telemetry()

        await self.run()

    async def _get_uuid_address(self):
        try:
            to_query = {
                moteus.Register.UUID1 : moteus.INT32,
                moteus.Register.UUID2 : moteus.INT32,
                moteus.Register.UUID3 : moteus.INT32,
                moteus.Register.UUID4 : moteus.INT32,
                moteus.Register.UUID_MASK_CAPABLE : moteus.INT32,
            }
            result = await self.controller.custom_query(to_query)


            if result.values.get(moteus.Register.UUID_MASK_CAPABLE, None) is None:
                return None

            # We'll just use the full 16 byte UUID in this case for
            # now.  Eventually maybe we could find an appropriate
            # shorter prefix.
            uuid_bytes = struct.pack(
                '<iiii',
                *[result.values[reg] for reg in [
                    moteus.Register.UUID1,
                    moteus.Register.UUID2,
                    moteus.Register.UUID3,
                    moteus.Register.UUID4]])

            return moteus.DeviceAddress(
                uuid=uuid_bytes,
                transport_device=self.address.transport_device
                if isinstance(self.address, moteus.DeviceAddress)
                else None)
        except Exception as e:
            print(f"UUID query failed: {e}")
            return None

    def _update_tree_items(self, tree_key):
        """Update tree items with the new tree key."""
        self._config_tree_item.setText(0, tree_key)
        self._data_tree_item.setText(0, tree_key)
        self._prefix = f'{tree_key}>'

    async def _handle_id_change(self):
        if self.uuid_address is None:
            # We don't have a UUID to work with, so this controller
            # may become not addressable.
            print(f"WARNING: controller {self.address} may now be unreachable")
            return

        if self.uuid_address == self.address:
            # We are already using UUID based addressing, so nothing
            # to do.
            return

        # Wait for all devices to complete UUID queries if not done
        if not self._main_window.uuid_query_event.is_set():
            print("Waiting for all devices to complete UUID queries...")
            await self._main_window.uuid_query_event.wait()

        # Check if all devices on this transport support UUID
        transport_device = self._main_window._get_transport_device(self.address)

        if not self._main_window.can_use_uuid_on_transport(transport_device):
            print(f"WARNING: Not all devices on transport {transport_device} support UUID addressing")
            print(f"Device {self.address} will remain on CAN ID addressing and may become unreachable")
            return

        # Acquire lock to ensure no poll is in progress when changing address
        async with self.poll_lock:
            await asyncio.sleep(0.1)

            # Calculate optimal UUID prefix.
            optimal_uuid = self.uuid_address  # Default to full UUID

            other_uuids = self._main_window.get_other_device_uuids(self, transport_device)

            optimal_prefix = calculate_optimal_uuid_prefix(self.full_uuid, other_uuids)

            # Create new address with optimal prefix
            optimal_uuid = moteus.DeviceAddress(
                uuid=optimal_prefix,
                transport_device=self.address.transport_device
                if isinstance(self.address, moteus.DeviceAddress)
                else None)

            print(f"Switching device {self.address} to UUID addressing: {optimal_prefix.hex()}")

            # Now perform the state change (while holding lock)
            self.address = optimal_uuid

            self.controller = moteus.Controller(
                self.address,
                source_can_id=self.source_can_id,
                can_prefix=self._can_prefix)

            self._stream.update_controller(self.controller)

        # Update tree items (can be done outside lock)
        if self._main_window:
            new_tree_key = self._main_window._calculate_tree_key(
                self.address, self._transport)
            self._update_tree_items(new_tree_key)

    async def update_config(self):
        self._updating_config = True

        try:
            # Clear out our config tree.
            self._config_tree_item.takeChildren()
            self._config_tree_items = {}

            # Try doing it the "new" way first.
            try:
                await self.schema_update_config()
                self._schema_config = True
                return
            except CommandError:
                # This means the controller we're working with doesn't
                # support the schema based config.
                self._schema_config = False
                pass

            configs = await self.command('conf enumerate')
            for config in configs.split('\n'):
                if config.strip() == '':
                    continue
                self.add_config_line(config)
        finally:
            self._updating_config = False

    async def schema_update_config(self):
        elements = [x.strip() for x in
                    (await self.command('conf list')).split('\n')
                    if x.strip() != '']
        for element in elements:
            self.write_line(f'conf schema {element}\r\n')
            schema = await self.read_schema(element)
            self.write_line(f'conf data {element}\r\n')
            data = await self.read_data(element)

            archive = reader.Type.from_binary(io.BytesIO(schema), name=element)
            item = QtWidgets.QTreeWidgetItem(self._config_tree_item)
            item.setText(0, element)

            flags = (QtCore.Qt.ItemIsEditable |
                     QtCore.Qt.ItemIsSelectable |
                     QtCore.Qt.ItemIsEnabled)

            _add_schema_item(item, archive, terminal_flags=flags)
            self._config_tree_items[element] = item
            data_struct = archive.read(reader.Stream(io.BytesIO(data)))
            _set_tree_widget_data(item, data_struct, archive, terminal_flags=flags)

            # Try to grab our current can_id.
            if (element == 'id' and
                getattr(data_struct, 'id', None) is not None
                and self.can_id is None):

                self.can_id = data_struct.id

    async def update_telemetry(self):
        self._data_tree_item.takeChildren()
        self._telemetry_records = {}

        channels = await self.command('tel list')
        for name in channels.split('\n'):
            if name.strip() == '':
                continue

            self.write_line(f'tel schema {name}\r\n')
            schema = await self.read_schema(name)

            archive = reader.Type.from_binary(io.BytesIO(schema), name=name)

            record = Record(archive)
            self._telemetry_records[name] = record
            record.tree_item = self._add_schema_to_tree(name, archive, record)

            self._add_text('<schema name=%s>\n' % name)

    async def run(self):
        while True:
            line = await self.readline()
            if _has_nonascii(line):
                # We need to try and resynchronize.  Skip to a '\r\n'
                # followed by at least 3 ASCII characters.
                await self._stream.resynchronize()
            if line.startswith('emit '):
                try:
                    await self.do_data(line.split(' ')[1])
                except Exception as e:
                    if (hasattr(self._stream.transport, '_debug_log') and
                        self._stream.transport._debug_log):
                        self._stream.transport._debug_log.write(
                            f"Error reading data: {e}".encode('latin1'))
                    print("Error reading data:", str(e))
                    # Just keep going and try to read more.


    async def read_schema(self, name):
        while True:
            line = await self.readline()
            if line.startswith('ERR'):
                raise CommandError('', line)
            if not (line == f'schema {name}' or line == f'schema {name}'):
                continue
            break
        schema = await self.read_sized_block()
        return schema

    async def read_schema(self, name):
        while True:
            line = await self.readline()
            if line.startswith('ERR'):
                raise CommandError('', line)
            if not (line == f'schema {name}' or line == f'cschema {name}'):
                continue
            break
        schema = await self.read_sized_block()
        return schema

    async def read_data(self, name):
        while True:
            line = await self.readline()
            if not line == f'cdata {name}':
                continue
            if line.startswith('ERR'):
                raise CommandError('', line)
            break
        return await self.read_sized_block()

    async def do_data(self, name):
        now = time.time()

        data = await self.read_sized_block()
        if not data:
            return

        if name not in self._telemetry_records:
            return

        record = self._telemetry_records[name]
        if record:
            struct = record.archive.read(reader.Stream(io.BytesIO(data)))
            record.update(struct, now)
            _set_tree_widget_data(record.tree_item, struct, record.archive)

            self._data[name] = struct
            if name not in self._events:
                self._events[name] = asyncio.Event()
            self._events[name].set()
            self._data_update_time[name] = time.time()

    async def wait_for_data(self, name):
        if name not in self._events:
            self._events[name] = asyncio.Event()

        await self._events[name].wait()
        self._events[name].clear()
        return self._data[name]

    async def ensure_record_active(self, name):
        now = time.time()
        if (now - self._data_update_time.get(name, 0.0)) > 0.2:
            print(f"trying to enable {name}")
            self.write_line(f'tel rate {name} 100\r\n')

    async def read_sized_block(self):
        return await self._stream.read_sized_block()

    async def process_message(self, message):
        any_data_read = await self._stream.process_message(message)

        return any_data_read

    async def emit_any_writes(self):
        await self._stream.maybe_emit_one()

    async def poll(self):
        await self._stream.poll()

    def write(self, data):
        self._stream.write(data)

        line = data.decode('latin1')

        # For some commands, we need to take special actions.
        if line.startswith('conf set id.id '):
            # Extract the new CAN ID from the command
            try:
                new_id_str = line.split('conf set id.id ')[1].strip()
                new_can_id = int(new_id_str)
                # Update our current CAN ID for future matching
                self.can_id = new_can_id
            except (IndexError, ValueError):
                # Invalid command format, ignore
                pass

            asyncio.create_task(self._handle_id_change())
        elif line.startswith('conf default') or line.startswith('conf load'):
            # Eventually it would be nice to reload the configuration
            # here so the UI stays consistent.  For now, we'll satisfy
            # ourselves with trying to switch to UUID based operation
            # so that we don't lose communication.

            # Reloading configuration is complicated, as we need to
            # dispense with the OK that the above commands would
            # create, but the current layering doesn't make that easy.
            asyncio.create_task(self._handle_id_change())

    def config_item_changed(self, name, value, schema):
        if self._updating_config:
            return
        if isinstance(schema, reader.EnumType) and ':' in value:
            int_val = value.rsplit(':', 1)[-1].strip(' >')
            value = int_val
        if isinstance(schema, reader.BooleanType) and value.lower() in ['true', 'false']:
            value = 1 if (value.lower() == 'true') else 0
        self.write_line('conf set %s %s\r\n' % (name, value))

    async def readline(self):
        result = (await self._stream.readline()).decode('latin1')
        if not result.startswith('emit '):
            self._add_text(result + '\n')
        return result

    async def command(self, message):
        self.write_line(message + '\r\n')
        result = io.StringIO()

        # First, read until we get something that is not an 'emit'
        # line.
        while True:
            line = await self.readline()
            if line.startswith('emit ') or line.startswith('schema '):
                continue
            break

        now = time.time()
        while True:
            if line.startswith('ERR'):
                raise CommandError(message, line)
            if line.startswith('OK'):
                return result.getvalue()

            result.write(line + '\n')
            line = await self.readline()
            end = time.time()
            now = end

    def add_config_line(self, line):
        # Add it into our tree view.
        key, value = line.split(' ', 1)
        name, rest = key.split('.', 1)
        if name not in self._config_tree_items:
            item = QtWidgets.QTreeWidgetItem(self._config_tree_item)
            item.setText(0, name)
            self._config_tree_items[name] = item

        def add_config(item, key, value):
            if key == '':
                item.setText(1, value)
                item.setFlags(QtCore.Qt.ItemFlags(
                    QtCore.Qt.ItemIsEditable |
                    QtCore.Qt.ItemIsSelectable |
                    QtCore.Qt.ItemIsEnabled))
                return

            fields = key.split('.', 1)
            this_field = fields[0]
            next_key = ''
            if len(fields) > 1:
                next_key = fields[1]

            child = None
            # See if we already have an appropriate child.
            for i in range(item.childCount()):
                if item.child(i).text(0) == this_field:
                    child = item.child(i)
                    break
            if child is None:
                child = QtWidgets.QTreeWidgetItem(item)
                child.setText(0, this_field)
            add_config(child, next_key, value)

        add_config(self._config_tree_items[name], rest, value)

    def _add_text(self, line):
        self._console.add_text(self._prefix + line)
        if (hasattr(self._stream.transport, '_debug_log') and
            self._stream.transport._debug_log):
            self._stream.transport._debug_log.write(
                f"{time.time()} : {line}".encode('latin1'))

    def write_line(self, line):
        self._add_text(line)
        self.write(line.encode('latin1'))

    class Schema:
        def __init__(self, name, parent, record):
            self._name = name
            self._parent = parent
            self.record = record

        def expand(self):
            self._parent.write_line('tel fmt %s 0\r\n' % self._name)
            self._parent.write_line('tel rate %s %d\r\n' %
                                    (self._name, DEFAULT_RATE))

        def collapse(self):
            self._parent.write_line('tel rate %s 0\r\n' % self._name)


    def _add_schema_to_tree(self, name, schema_data, record):
        item = QtWidgets.QTreeWidgetItem(self._data_tree_item)
        item.setText(0, name)

        schema = Device.Schema(name, self, record)
        item.setData(0, QtCore.Qt.UserRole, schema)

        _add_schema_item(item, schema_data)
        return item

    async def check_fault_status(self):
        """Check current fault status and return is_faulted, mode, fault_code)."""
        result = await self.controller.custom_query({
            Register.MODE: moteus.multiplex.INT8,
            Register.FAULT: moteus.multiplex.INT8,
        })

        mode = result.values.get(Register.MODE, None)
        fault_code = result.values.get(Register.FAULT, None)
        is_faulted = (mode == Mode.FAULT) if mode is not None else False

        return is_faulted, mode, fault_code

    def update_fault_state(self, is_faulted):
        """Update the fault state based on current status."""
        previous_faulted = self.fault_state.is_faulted
        fault_detected = False

        if is_faulted and not previous_faulted:
            # New fault detected - needs observation
            self.fault_state.is_faulted = True
            self.fault_state.observed = False
            fault_detected = True
        elif not is_faulted and previous_faulted:
            # Fault cleared - mark as observed since it's no longer present
            self.fault_state.is_faulted = False
            self.fault_state.observed = True

        return fault_detected

    def mark_fault_observed(self):
        """Mark the current fault as observed by the user."""
        if self.fault_state.is_faulted:
            self.fault_state.observed = True

    def has_unobserved_fault(self):
        """Check if device has a fault that hasn't been observed."""
        return self.fault_state.is_faulted and not self.fault_state.observed

    async def check_and_update_fault_state(self):
        """Check fault status and update state. Returns (fault_detected, fault_cleared)."""
        # Check current fault status
        is_faulted, mode, fault_code = await self.check_fault_status()

        # Store previous state to detect fault clearing
        prev_faulted = self.fault_state.is_faulted

        # Store current fault information for status bar
        self.fault_state.current_mode = mode
        self.fault_state.current_fault_code = fault_code

        # Update fault state and check if new fault detected
        fault_detected = self.update_fault_state(is_faulted)
        fault_cleared = prev_faulted and not self.fault_state.is_faulted

        return fault_detected, fault_cleared


class TviewMainWindow():
    def __init__(self, options, parent=None):
        self.options = options
        self.port = None
        self.devices = []
        self.default_rate = 100

        self.user_task = None

        # UUID coordination infrastructure
        self.uuid_query_event = asyncio.Event()
        self.uuid_query_count = 0
        self.expected_device_count = 0
        self.device_uuid_support = {}
        self.uuid_query_lock = asyncio.Lock()

        # Fault monitoring infrastructure
        self.fault_monitoring_task = None
        self.fault_flash_timer = None
        self.original_tab_color = None  # Store original tab color
        self.fault_flash_state = False

        current_script_dir = os.path.dirname(os.path.abspath(__file__))
        uifilename = os.path.join(current_script_dir, "tview_main_window.ui")

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(uifilename)
        uifile.open(QtCore.QFile.ReadOnly)
        self.ui = loader.load(uifile, parent)
        uifile.close()

        self.ui.configTreeWidget = SizedTreeWidget()
        self.ui.configDock.setWidget(self.ui.configTreeWidget)

        self.ui.telemetryTreeWidget = SizedTreeWidget()
        self.ui.telemetryDock.setWidget(self.ui.telemetryTreeWidget)

        self.ui.telemetryTreeWidget.itemExpanded.connect(
            self._handle_tree_expanded)
        self.ui.telemetryTreeWidget.itemCollapsed.connect(
            self._handle_tree_collapsed)
        self.ui.telemetryTreeWidget.setContextMenuPolicy(
            QtCore.Qt.CustomContextMenu)
        self.ui.telemetryTreeWidget.customContextMenuRequested.connect(
            self._handle_telemetry_context_menu)

        # Track clicks for fault observation
        self.ui.telemetryTreeWidget.itemClicked.connect(
            self._handle_telemetry_item_clicked)

        self.ui.configTreeWidget.setItemDelegateForColumn(
            0, NoEditDelegate(self.ui))
        self.ui.configTreeWidget.setItemDelegateForColumn(
            1, EditDelegate(self.ui))

        self.ui.configTreeWidget.itemExpanded.connect(
            self._handle_config_expanded)
        self.ui.configTreeWidget.itemChanged.connect(
            self._handle_config_item_changed)

        self.ui.plotItemRemoveButton.clicked.connect(
            self._handle_plot_item_remove)

        self.tabbed_console = TviewTabbedConsole(get_controller=self._python_get_controller)
        self.ui.consoleDock.setWidget(self.tabbed_console)

        self.console = self.tabbed_console.diagnostic_console
        self.console.line_input.connect(self._handle_user_input)

        self.python_console = self.tabbed_console.python_console

        self.ui.tabifyDockWidget(self.ui.configDock, self.ui.telemetryDock)

        layout = QtWidgets.QVBoxLayout(self.ui.plotHolderWidget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.ui.plotHolderWidget.setLayout(layout)
        self.ui.plotWidget = PlotWidget(self.ui.plotHolderWidget)
        layout.addWidget(self.ui.plotWidget)

        def update_plotwidget(value):
            self.ui.plotWidget.history_s = value
        self.ui.historySpin.valueChanged.connect(update_plotwidget)

        QtCore.QTimer.singleShot(0, self._handle_startup)

    def show(self):
        self.ui.show()

    def _make_transport(self):
        # Get a transport as configured.
        return moteus.get_singleton_transport(self.options)

    def _open(self):
        self.transport = self._make_transport()
        asyncio.create_task(self._run_transport())

        asyncio.create_task(self._populate_devices())

    def _calculate_tree_key(self, device_address, transport):
        """Calculate the tree key for a device based on its address."""
        needs_suffix = (transport.count() > 1 and
                        not isinstance(device_address, int) and
                        hasattr(device_address, 'transport_device') and
                        device_address.transport_device)

        suffix_str = f'/{device_address.transport_device}' if needs_suffix else ''

        tree_key = (
            str(device_address) if isinstance(device_address, int)
            else f'{device_address.can_id}{suffix_str}' if hasattr(device_address, 'can_id') and device_address.can_id
            else f'{device_address.uuid.hex()}{suffix_str}')

        return tree_key

    def _init_uuid_coordination(self, device_count):
        """Initialize UUID query coordination for a set of devices"""
        self.expected_device_count = device_count

    async def register_uuid_query_complete(self, device, has_uuid):
        """Called by each device when UUID query completes"""
        async with self.uuid_query_lock:
            # Track device UUID capability
            transport_device = self._get_transport_device(device.address)
            if transport_device not in self.device_uuid_support:
                self.device_uuid_support[transport_device] = []
            self.device_uuid_support[transport_device].append((device, has_uuid))

            # Update counter
            self.uuid_query_count += 1

            # Signal if all complete
            if self.uuid_query_count >= self.expected_device_count:
                self.uuid_query_event.set()

    def _get_transport_device(self, address):
        """Extract transport device from an address"""
        if isinstance(address, int):
            return None  # Default transport
        elif hasattr(address, 'transport_device'):
            return address.transport_device
        return None

    def can_use_uuid_on_transport(self, transport_device):
        """Check if all devices on a transport support UUID"""
        if transport_device not in self.device_uuid_support:
            return False

        devices_on_transport = self.device_uuid_support[transport_device]
        return all(has_uuid for _, has_uuid in devices_on_transport)

    def get_other_device_uuids(self, device, transport_device):
        """Get the UUIDs of other devices on the same transport.

        Args:
            device: The device to exclude from the list
            transport_device: The transport to query

        Returns:
            List of full UUIDs from other devices on this transport
        """
        other_uuids = []
        devices_on_transport = self.device_uuid_support.get(transport_device, [])

        for other_device, has_uuid in devices_on_transport:
            if other_device != device and has_uuid:
                # Check if device has stored full UUID
                if hasattr(other_device, 'full_uuid'):
                    other_uuids.append(other_device.full_uuid)

        return other_uuids

    def is_can_id_unique(self, can_id):
        """Check if a CAN ID is unique across all devices in the system."""
        matching_devices = [d for d in self.devices
                            if d.can_id == can_id]
        return len(matching_devices) == 1

    async def _populate_devices(self):
        self.python_console.namespace['transport'] = self.transport

        self.devices = []

        targets = moteus.moteus_tool.expand_targets(self.options.devices)
        if not targets:
            discovered = await self.transport.discover(
                can_prefix=self.options.can_prefix, source=0x7e)
            not_addressable = [x for x in discovered if x.address is None]

            if len(not_addressable) > 0:
                print("No target specified, and one or more devices are not addressable", file=sys.stderr)
                print(file=sys.stderr)
                for x in not_addressable:
                    print(f' * {x}', file=sys.stderr)
                sys.exit(1)

            targets = [x.address for x in discovered]

        self.ui.configTreeWidget.clear()
        self.ui.telemetryTreeWidget.clear()

        # Initialize UUID coordination for all devices
        self._init_uuid_coordination(len(targets))

        device_count = 0
        source_can_id = 0x7d

        for device_address in targets:
            # Extract current CAN ID from the target specification
            current_can_id = None
            if isinstance(device_address, int):
                # Direct integer CAN ID specification
                current_can_id = device_address
            elif getattr(device_address, 'can_id', None) is not None:
                # DeviceAddress with CAN ID
                current_can_id = device_address.can_id

            # UUID-only addresses will have current_can_id = None

            tree_key = self._calculate_tree_key(device_address, self.transport)

            config_item = QtWidgets.QTreeWidgetItem()

            config_item.setText(0, tree_key)
            self.ui.configTreeWidget.addTopLevelItem(config_item)

            data_item = QtWidgets.QTreeWidgetItem()
            data_item.setText(0, tree_key)
            self.ui.telemetryTreeWidget.addTopLevelItem(data_item)

            python_source_can_id = source_can_id - 1

            device = Device(device_address,
                            source_can_id,
                            python_source_can_id,
                            self.transport,
                            self.console, '{}>'.format(tree_key),
                            config_item,
                            data_item,
                            self.options.can_prefix,
                            self,
                            current_can_id)

            source_can_id -= 2

            config_item.setData(0, QtCore.Qt.UserRole, device)
            data_item.setData(0, QtCore.Qt.UserRole, device)
            asyncio.create_task(device.start())

            self.devices.append(device)

        # Start fault monitoring after all devices are created
        if self.devices and not self.fault_monitoring_task:
            self.fault_monitoring_task = asyncio.create_task(self._monitor_device_faults())

        if self.devices:
            self.python_console.namespace['controller'] = self._python_get_controller(self.devices[0].address)

    def _python_get_controller(self, name_or_address):
        def get_device():
            # Is this an address that matches one of our devices
            # exactly?
            maybe_device_by_address = [
                device for device in self.devices
                if (device.address == name_or_address
                    or (isinstance(device.address, int) and
                        isinstance(name_or_address, int) and
                        device.address == name_or_address)
                    or (not isinstance(device.address, int) and
                        device.address.can_id is not None and
                        isinstance(name_or_address, int) and
                        device.address.can_id == name_or_address))
            ]
            if maybe_device_by_address:
                return maybe_device_by_address[0]

            # Can we look it up by name?
            if isinstance(name_or_address, str):
                maybe_devices = [x for x in self.devices
                                 if self._match(x, name_or_address)]
                if maybe_devices:
                    return maybe_devices[0]

            return None

        device = get_device()
        if device:
            return moteus.Controller(
                device.address,
                source_can_id=device.python_source_can_id,
                can_prefix=self.options.can_prefix)

        # It doesn't appear to match one of our existing devices.
        # Just try to make a new instance assuming it is address-like.
        return moteus.Controller(
            name_or_address,
            can_prefix=self.options.can_prefix)

    def _handle_startup(self):
        self.console._control.setFocus()
        self._open()

    async def _dispatch_until(self, predicate):
        while True:
            message = await self.transport.read()
            if message is None:
                continue
            source_id = (message.arbitration_id >> 8) & 0x7f
            dest_id = (message.arbitration_id & 0x7f)
            any_data_read = False
            for device in self.devices:
                if ((device.address.transport_device is None or
                     device.address.transport_device == message.channel) and
                    device.source_can_id == dest_id and
                    (device.address.can_id is None or
                     device.address.can_id == source_id)):
                    any_data_read = await device.process_message(message)
                    break
            if predicate(message):
                return any_data_read

    async def _run_transport(self):
        any_data_read = False
        while True:
            # We only sleep if no devices had anything to report the last cycle.
            if not any_data_read:
                await asyncio.sleep(0.01)

            any_data_read = await self._run_transport_iteration()

    async def _run_transport_iteration(self):
        any_data_read = False

        # First, do writes from all devices.  This ensures that the
        # writes will go out at approximately the same time.
        for device in self.devices:
            await device.emit_any_writes()

        # Then poll for new data.  Back off from unresponsive devices
        # so that they don't disrupt everything.
        for device in self.devices:
            if device.poll_count:
                device.poll_count -= 1
                continue

            # Acquire lock for the entire poll-response cycle
            async with device.poll_lock:
                await device.poll()

                try:
                    this_data_read = await asyncio.wait_for(
                        self._dispatch_until(
                            lambda x: (
                                (x.arbitration_id & 0x7f) == device.source_can_id and
                                (device.address.can_id is None or
                                 (x.arbitration_id >> 8) & 0x7f == device.address.can_id))),
                        timeout = POLL_TIMEOUT_S)

                    device.error_count = 0
                    device.poll_count = 0

                    if this_data_read:
                        any_data_read = True
                except asyncio.TimeoutError:
                    # Mark this device as error-full, which will then
                    # result in backoff in polling.
                    device.error_count = min(1000, device.error_count + 1)
                    device.poll_count = device.error_count

        return any_data_read

    def make_writer(self, devices, line):
        def write():
            for device in devices:
                device.write((line + '\n').encode('latin1'))

        return write

    def _handle_user_input(self, line):
        if self.user_task is not None:
            # We have an outstanding one, so cancel it.
            self.user_task.cancel()
            self.user_task = None

        self.user_task = asyncio.create_task(
            self._run_user_command_line(line))

    async def _run_user_command_line(self, line):
        try:
            for command in [x.strip() for x in line.split('&&')]:
                await self._run_user_command(command)
        except Exception as e:
            print("Error:", str(e))

            # Otherwise ignore problems so that tview keeps running.

    def _match(self, device, s):
        # Try to parse as integer for CAN ID matching
        try:
            target_id = int(s)
            if target_id < 1 or target_id > 126:
                # These are not valid IDs.
                target_id = None
        except:
            target_id = None

        # Check current address CAN ID.
        if device.address.can_id is not None and target_id is not None:
            if target_id == device.address.can_id:
                return True

        # Check UUID addressing.
        if device.address.uuid is not None:
            if s.upper() == device.address.uuid.hex().upper():
                return True

        # Check tracked CAN ID if it's unique in the system
        if (target_id is not None and
            getattr(device, 'can_id', None) is not None and
            target_id == device.can_id and
            self.is_can_id_unique(target_id)):
            return True

        return False

    async def _wait_user_query(self, maybe_id):
        devices = [self.devices[0]]

        if maybe_id:
            devices = [x for x in self.devices if
                       self._match(x, maybe_id)]

        record = 'servo_stats'

        if len(devices) == 0:
            # Nothing to wait on, so return immediately
            return

        for d in devices:
            await d.ensure_record_active(record)
            await d.wait_for_data(record)
            await d.wait_for_data(record)

        while True:
            # Now look for at least to have trajectory_done == True
            for d in devices:
                servo_stats = await d.wait_for_data(record)
                if getattr(servo_stats, 'trajectory_done', False):
                    return

    async def _run_user_command(self, command):
        delay_re = re.search(r"^:(\d+)$", command)
        device_re = re.search(r"^(A|\d+|[a-fA-F0-9]{8,32})>\s*(.*)$", command)
        traj_re = re.search(r"^(\?(\d+|[a-fA-F0-9]{8,32})?)$", command)

        devices = [self.devices[0]]

        if traj_re:
            await self._wait_user_query(traj_re.group(2))
            return
        if delay_re:
            await asyncio.sleep(int(delay_re.group(1)) / 1000.0)
            return
        elif device_re:
            command = device_re.group(2)
            if device_re.group(1) == 'A':
                devices = self.devices
            else:
                devices = [x for x in self.devices
                           if self._match(x, device_re.group(1))]

        for device in devices:
            device.write((command + '\n').encode('latin1'))

    def _handle_tree_expanded(self, item):
        self.ui.telemetryTreeWidget.resizeColumnToContents(0)
        user_data = item.data(0, QtCore.Qt.UserRole)
        if (user_data and
            hasattr(user_data, 'expand') and
            callable(user_data.expand)):
            user_data.expand()

        # Mark fault as observed if expanding servo_stats while
        # telemetry is visible

        if (self.ui.telemetryDock.isVisible() and
            item.text(0).lower() == "servo_stats"):
            device = self._find_device_from_tree_item(item)
            if device and device.has_unobserved_fault():
                self._mark_fault_observed(device)

    def _handle_tree_collapsed(self, item):
        user_data = item.data(0, QtCore.Qt.UserRole)
        if user_data and hasattr(user_data, 'collapse') and callable(user_data.collapse):
            user_data.collapse()

    def _handle_telemetry_context_menu(self, pos):
        item = self.ui.telemetryTreeWidget.itemAt(pos)
        if item.childCount() > 0:
            return

        menu = QtWidgets.QMenu(self.ui)
        left_action = menu.addAction('Plot Left')
        right_action = menu.addAction('Plot Right')
        left_std_action = menu.addAction('Plot StdDev Left')
        right_std_action = menu.addAction('Plot StdDev Right')
        left_mean_action = menu.addAction('Plot Mean Left')
        right_mean_action = menu.addAction('Plot Mean Right')

        plot_actions = [
            left_action,
            right_action,
            left_std_action,
            right_std_action,
            left_mean_action,
            right_mean_action,
        ]

        right_actions = [right_action, right_std_action, right_mean_action]
        std_actions = [left_std_action, right_std_action]
        mean_actions = [left_mean_action, right_mean_action]

        menu.addSeparator()
        copy_name = menu.addAction('Copy Name')
        copy_value = menu.addAction('Copy Value')

        menu.addSeparator()
        fmt_standard_action = menu.addAction('Standard Format')
        fmt_hex_action = menu.addAction('Hex Format')

        requested = menu.exec_(self.ui.telemetryTreeWidget.mapToGlobal(pos))

        if requested in plot_actions:
            top = item
            while top.parent().parent():
                top = top.parent()

            schema = top.data(0, QtCore.Qt.UserRole)
            record = schema.record

            name = _get_item_name(item)
            root = _get_item_root(item)

            leaf = name.split('.', 1)[1]
            axis = 0
            if requested in right_actions:
                axis = 1

            if requested in std_actions:
                leaf = '__STDDEV_' + leaf
                name = 'stddev ' + name

            if requested in mean_actions:
                leaf = '__MEAN_' + leaf
                name = 'mean ' + name

            plot_item = self.ui.plotWidget.add_plot(
                name, record.get_signal(leaf), axis)
            self.ui.plotItemCombo.addItem(name, plot_item)
        elif requested == copy_name:
            QtWidgets.QApplication.clipboard().setText(item.text(0))
        elif requested == copy_value:
            QtWidgets.QApplication.clipboard().setText(item.text(1))
        elif requested == fmt_standard_action:
            item.setData(1, FORMAT_ROLE, FMT_STANDARD)
        elif requested == fmt_hex_action:
            item.setData(1, FORMAT_ROLE, FMT_HEX)
        else:
            # The user cancelled.
            pass

    def _handle_config_expanded(self, item):
        self.ui.configTreeWidget.resizeColumnToContents(0)

    def _handle_config_item_changed(self, item, column):
        if not item.parent():
            return

        top = item
        while top.parent():
            top = top.parent()

        device = top.data(0, QtCore.Qt.UserRole)
        device.config_item_changed(_get_item_name(item), item.text(1),
                                   item.data(1, QtCore.Qt.UserRole))

    def _handle_plot_item_remove(self):
        index = self.ui.plotItemCombo.currentIndex()

        if index < 0:
            return

        item = self.ui.plotItemCombo.itemData(index)
        self.ui.plotWidget.remove_plot(item)
        self.ui.plotItemCombo.removeItem(index)

    # Fault Monitoring System
    async def _monitor_device_faults(self):
        """Continuously monitor devices for fault conditions."""
        # Wait for devices to initialize

        # Start monitoring immediately - devices can handle queries during initialization

        while True:
            try:
                await asyncio.sleep(FAULT_POLLING_INTERVAL_MS / 1000.0)

                fault_detected = False
                for device in self.devices:
                    # Check and update fault state for this device
                    device_fault_detected, device_fault_cleared = await device.check_and_update_fault_state()
                    if device_fault_detected:
                        fault_detected = True
                    if device_fault_cleared:
                        # Clear highlighting for this device when fault is cleared
                        self._clear_device_highlighting(device)

                # Start/stop flashing based on unobserved faults
                if fault_detected:
                    self._start_fault_flashing()
                elif self._all_faults_observed():
                    self._stop_fault_flashing()

                # Update status bar with current fault information
                self._update_fault_status_bar()

            except Exception as e:
                print(f"Error in fault monitoring: {e}")
                await asyncio.sleep(1.0)

    def _all_faults_observed(self):
        """Check if all current faults have been visually observed."""
        return all(not device.has_unobserved_fault() for device in self.devices)

    def _start_fault_flashing(self):
        """Start visual fault indicators."""
        if self.fault_flash_timer is None:
            self.fault_flash_timer = QtCore.QTimer()
            self.fault_flash_timer.timeout.connect(self._toggle_fault_flash)
            self.fault_flash_timer.start(500)
            self._toggle_fault_flash()

    def _stop_fault_flashing(self):
        """Stop visual fault indicators."""
        if self.fault_flash_timer is not None:
            self.fault_flash_timer.stop()
            self.fault_flash_timer = None
            self._clear_fault_highlighting()

    def _toggle_fault_flash(self):
        """Toggle fault visual indicators."""
        self.fault_flash_state = not self.fault_flash_state
        color = "#FF4444" if self.fault_flash_state else "#FFA500"

        for device in self.devices:
            if device.has_unobserved_fault():
                self._highlight_device_fault(device, color)

    def _highlight_device_fault(self, device, color):
        """Apply fault highlighting to UI elements."""
        self._highlight_telemetry_tab(color)
        self._highlight_device_tree_items(device, color)
        self._highlight_servo_stats(device, color)

    def _highlight_telemetry_tab(self, color):
        """Flash telemetry tab text to indicate fault."""
        tab_bars = self.ui.findChildren(QtWidgets.QTabBar)
        for tab_bar in tab_bars:
            for i in range(tab_bar.count()):
                if "telemetry" in tab_bar.tabText(i).lower():
                    # Store original color on first modification
                    if self.original_tab_color is None:
                        # Get the actual default color from the palette since tabTextColor
                        # returns invalid QColor() for tabs that haven't been modified
                        self.original_tab_color = tab_bar.palette().color(QtGui.QPalette.WindowText)

                    # This is the best visual indicator I've managed
                    # to find for tab text.
                    if self.fault_flash_state:
                        tab_bar.setTabTextColor(i, QtGui.QColor("#FF0000"))
                    else:
                        tab_bar.setTabTextColor(i, QtGui.QColor("#FF8800"))
                    return

    def _highlight_device_tree_items(self, device, color):
        """Highlight device tree items in telemetry view."""
        # Only highlight if device has unobserved fault
        if hasattr(device, '_data_tree_item') and device.has_unobserved_fault():
            brush = QtGui.QBrush(QtGui.QColor(color))
            device._data_tree_item.setBackground(0, brush)
            device._data_tree_item.setBackground(1, brush)

    def _highlight_servo_stats(self, device, color):
        """Highlight servo_stats entry for device."""
        # Only highlight if device has unobserved fault
        if hasattr(device, '_data_tree_item') and device.has_unobserved_fault():
            for i in range(device._data_tree_item.childCount()):
                child = device._data_tree_item.child(i)
                if child.text(0) == 'servo_stats':
                    brush = QtGui.QBrush(QtGui.QColor(color))
                    child.setBackground(0, brush)
                    child.setBackground(1, brush)
                    break

    def _clear_fault_highlighting(self):
        """Clear all fault highlighting."""
        # Clear telemetry tab color
        tab_bars = self.ui.findChildren(QtWidgets.QTabBar)
        for tab_bar in tab_bars:
            for i in range(tab_bar.count()):
                if "telemetry" in tab_bar.tabText(i).lower():
                    # Reset the color using the stored original
                    if self.original_tab_color and self.original_tab_color.isValid():
                        tab_bar.setTabTextColor(i, self.original_tab_color)
                    break  # Found and processed the telemetry tab

        # Clear tree highlighting for all devices
        for device in self.devices:
            self._clear_device_highlighting(device)

    def _clear_device_highlighting(self, device):
        """Clear fault highlighting for a specific device."""
        if hasattr(device, '_data_tree_item'):
            device._data_tree_item.setBackground(0, QtGui.QBrush())
            device._data_tree_item.setBackground(1, QtGui.QBrush())
            # Clear servo_stats
            for i in range(device._data_tree_item.childCount()):
                child = device._data_tree_item.child(i)
                if child.text(0) == 'servo_stats':
                    child.setBackground(0, QtGui.QBrush())
                    child.setBackground(1, QtGui.QBrush())

    # Observation Tracking
    def _handle_telemetry_item_clicked(self, item, column):
        """Handle clicks on telemetry items for fault observation."""
        if not self.ui.telemetryDock.isVisible():
            return

        # We only consider a fault observed if the user expanded or
        # clicked on the 'servo_stats' entry, or the 'fault' or 'mode'
        # elements of 'servo_stats.
        item_text = item.text(0).lower()
        if item_text == "servo_stats":
            # Only count servo_stats click as observation if it's already expanded
            if item.isExpanded():
                device = self._find_device_from_tree_item(item)
            else:
                return  # Don't count clicks on collapsed servo_stats
        elif item_text in ["mode", "fault"] and item.parent() and item.parent().text(0).lower() == "servo_stats":
            device = self._find_device_from_tree_item(item.parent().parent())
        else:
            return

        if device and device.has_unobserved_fault():
            self._mark_fault_observed(device)

    def _find_device_from_tree_item(self, item):
        """Find device associated with tree item."""
        if not item:
            return None

        # Traverse up to top-level item
        current = item
        while current.parent():
            current = current.parent()

        # Check if top-level item has device data
        device_data = current.data(0, QtCore.Qt.UserRole)
        if isinstance(device_data, Device):
            return device_data

        # Fallback: search by tree item reference
        for device in self.devices:
            if hasattr(device, '_data_tree_item') and device._data_tree_item == current:
                return device

        return None

    def _mark_fault_observed(self, device):
        """Mark device fault as visually observed."""
        device.mark_fault_observed()

        # Clear highlighting for this specific device immediately
        self._clear_device_highlighting(device)

        # Check if all faults are now observed and stop global flashing if so
        if self._all_faults_observed():
            self._stop_fault_flashing()

        # Update status bar to reflect observed fault
        self._update_fault_status_bar()

    def _update_fault_status_bar(self):
        """Update the status bar with current fault information."""
        # Collect all faulted devices
        faulted_devices = []
        for device in self.devices:
            if device.fault_state.is_faulted:
                faulted_devices.append(device)

        if not faulted_devices:
            # No faults - clear status bar
            self.ui.statusbar.clearMessage()
            return

        FULL_LIST_COUNT = 2

        if len(faulted_devices) == 1:
            # Single fault - show device and fault code with description
            device = faulted_devices[0]
            fault_code = device.fault_state.current_fault_code
            fault_text = _format_fault_code(fault_code)
            if fault_text:
                message = f"Fault: {device.address} {fault_text}"
            else:
                message = f"Fault: {device.address}"
        elif len(faulted_devices) <= FULL_LIST_COUNT:
            # Multiple faults - show compact list with descriptions
            fault_strs = []
            for device in faulted_devices:
                fault_code = device.fault_state.current_fault_code
                fault_text = _format_fault_code(fault_code)
                if fault_text:
                    fault_strs.append(f"{device.address} {fault_text}")
                else:
                    fault_strs.append(f"{device.address}")
            message = f"Faults: {', '.join(fault_strs)}"
        else:
            # Many faults - show count with tooltip
            message = f"Faults: {len(faulted_devices)} devices - hover for details"

            # Create tooltip with detailed fault information including descriptions
            tooltip_lines = []
            for device in faulted_devices:
                fault_code = device.fault_state.current_fault_code
                fault_text = _format_fault_code(fault_code)
                if fault_text:
                    tooltip_lines.append(f"{device.address} {fault_text}")
                else:
                    tooltip_lines.append(f"{device.address}")
            tooltip = "\n".join(tooltip_lines)
            self.ui.statusbar.setToolTip(tooltip)

        # Display the message
        self.ui.statusbar.showMessage(message)

        # Clear tooltip if not many faults
        if len(faulted_devices) <= FULL_LIST_COUNT:
            self.ui.statusbar.setToolTip("")


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('--version', action='store_true')

    # These two commands are aliases.
    parser.add_argument('-d', '--devices', '-t', '--target',
                        action='append', type=str, default=[])
    parser.add_argument('--can-prefix', type=int, default=0)

    moteus.make_transport_args(parser)

    args = parser.parse_args()

    if args.version:
        print(f"tview version '{version.VERSION}'")
        sys.exit(0)

    app = QtWidgets.QApplication(sys.argv)
    loop = asyncqt.QEventLoop(app)
    asyncio.set_event_loop(loop)

    # Currently there are many things that can barf on exit, let's
    # just ignore all of them because, hey, we're about to exit!
    app.aboutToQuit.connect(lambda: os._exit(0))

    tv = TviewMainWindow(args)
    tv.show()

    app.exec_()


if __name__ == '__main__':
    main()
