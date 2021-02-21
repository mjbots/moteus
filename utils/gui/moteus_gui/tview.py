#!/usr/bin/python3 -B

# Copyright 2015-2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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
import asyncio
import io
import moteus
import moteus.moteus_tool
import numpy
import os
import re
import struct
import sys
import time
import traceback
import matplotlib
import matplotlib.figure

import PySide2

os.environ['QT_API'] = 'pyside2'

from matplotlib.backends import backend_qt5agg
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
qt_backend = matplotlib.backends.backend_qt5agg

from PySide2 import QtUiTools

import qtconsole
from qtconsole.history_console_widget import HistoryConsoleWidget
if getattr(qtconsole, "qt", None):
    from qtconsole.qt import QtCore, QtGui
    QtWidgets = QtGui
else:
    from qtpy import QtCore, QtGui, QtWidgets

import asyncqt

import moteus.reader as reader


LEFT_LEGEND_LOC = 3
RIGHT_LEGEND_LOC = 2

DEFAULT_RATE = 100
MAX_HISTORY_SIZE = 100
MAX_SEND = 61
POLL_TIMEOUT_S = 0.1
STARTUP_TIMEOUT_S = 0.5

# TODO jpieper: Factor these out of tplot.py
def _get_data(value, name):
    fields = name.split('.')
    for field in fields:
        if isinstance(value, list):
            value = value[int(field)]
        else:
            value = getattr(value, field)
    return value


def _set_tree_widget_data(item, struct,
                          getter=lambda x, y: getattr(x, y),
                          required_size=0):
    if item.childCount() < required_size:
        for i in range(item.childCount(), required_size):
            subitem = QtWidgets.QTreeWidgetItem(item)
            subitem.setText(0, str(i))
    for i in range(item.childCount()):
        child = item.child(i)
        name = child.text(0)

        field = getter(struct, name)
        if isinstance(field, tuple) and child.childCount() > 0:
            _set_tree_widget_data(child, field)
        elif isinstance(field, list):
            _set_tree_widget_data(child, field,
                                  getter=lambda x, y: x[int(y)],
                                  required_size=len(field))
        else:
            child.setText(1, repr(field))


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

    def update(self, value):
        for handler in self._callbacks.values():
            handler(value)
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
        else:
            self.axis.legend(loc=self.axis.legend_loc)
        self.plot_widget.canvas.draw()

    def _handle_update(self, value):
        if self.plot_widget.paused:
            return

        if self.line is None:
            self._make_line()

        now = time.time()
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

    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)

        self.history_s = 20.0
        self.next_color = 0
        self.paused = False

        self.last_draw_time = 0.0

        self.figure = matplotlib.figure.Figure()
        self.canvas = FigureCanvas(self.figure)

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

    def update(self, struct):
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
            if signal.update(value):
                count += 1
        return count != 0


class NoEditDelegate(QtWidgets.QStyledItemDelegate):
    def __init__(self, parent=None):
        QtWidgets.QStyledItemDelegate.__init__(self, parent=parent)

    def createEditor(self, parent, option, index):
        return None


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

    def ignore_all(self):
        self._read_data = b''

    def write(self, data):
        self._write_data += data

    async def maybe_emit_one(self):
        if len(self._write_data) == 0:
            return

        to_write, self._write_data = (
            self._write_data[0:MAX_SEND], self._write_data[MAX_SEND:])
        await self.transport.write(self.controller.make_diagnostic_write(to_write))

    def process_message(self, message):
        data = message.data

        if len(data) < 3:
            return False

        if data[0] != 0x41:
            return False
        if data[1] != 1:
            return False
        if data[2] > MAX_SEND:
            return False
        datalen = data[2]
        if datalen > (len(data) - 3):
            return False

        self._read_data += data[3:3+datalen]
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

    def read_line(self):
        while True:
            maybe_line = self._read_maybe_empty_line()
            if maybe_line is None:
                return
            maybe_line = maybe_line.rstrip()
            if len(maybe_line) == 0:
                continue
            return maybe_line

    def read_sized_block(self):
        if len(self._read_data) < 5:
            return

        size = struct.unpack('<I', self._read_data[1:5])[0]
        if size > 2 ** 24:
            return False

        if len(self._read_data) < 5 + size:
            return

        block = self._read_data[5:5+size]
        self._read_data = self._read_data[5+size:]
        return block


class Device:
    STATE_LINE = 0
    STATE_CONFIG = 1
    STATE_TELEMETRY = 2
    STATE_SCHEMA = 3
    STATE_DATA = 4

    def __init__(self, number, transport, console, prefix,
                 config_tree_item, data_tree_item):
        self.error_count = 0
        self.poll_count = 0

        self.number = number
        self.controller = moteus.Controller(number)
        self._transport = transport
        self._stream = DeviceStream(transport, self.controller)

        self._console = console
        self._prefix = prefix
        self._config_tree_item = config_tree_item
        self._data_tree_item = data_tree_item

        self._serial_state = self.STATE_LINE
        self._telemetry_records = {}
        self._schema_name = None
        self._config_tree_items = {}
        self._config_callback = None

        self._start_time = None

    async def start(self):
        # Stop the spew.
        self.write('\r\ntel stop\r\n'.encode('latin1'))

        # We want to wait a little bit, discard everything we have
        # received, and then initialize the device.
        self._start_time = time.time()

    def _setup_device(self, callback):
        # When we start, get a listing of all configuration options
        # and all available telemetry channels.
        def after_config():
            self.update_telemetry(callback)
        self.update_config(after_config)

    def process_message(self, message):
        now = time.time()
        if self._start_time and (now - self._start_time < STARTUP_TIMEOUT_S):
            return False

        any_data_read = self._stream.process_message(message)

        while True:
            old_len = len(self._stream._read_data)
            try:
                self._handle_serial_data()
            except Exception as e:
                traceback.print_exc()
                print("Error parsing:", e)
            if len(self._stream._read_data) == old_len:
                break

        return any_data_read

    async def emit_any_writes(self):
        await self._stream.maybe_emit_one()

    async def poll(self):
        await self._transport.write(self.controller.make_diagnostic_read())

        if self._start_time is not None:
            now = time.time()
            if now - self._start_time > STARTUP_TIMEOUT_S:
                self._stream.ignore_all()
                self._setup_device(None)
                self._start_time = None

    def write(self, data):
        self._stream.write(data)

    def config_item_changed(self, name, value):
        if self._serial_state == self.STATE_CONFIG:
            return

        self.write_line('conf set %s %s\r\n' % (name, value))

    def _handle_serial_data(self):
        if self._serial_state == self.STATE_LINE:
            self._handle_serial_line()
        elif self._serial_state == self.STATE_CONFIG:
            self._handle_config()
        elif self._serial_state == self.STATE_TELEMETRY:
            self._handle_telemetry()
        elif self._serial_state == self.STATE_SCHEMA:
            self._handle_schema()
        elif self._serial_state == self.STATE_DATA:
            self._handle_data()
        else:
            assert False

    def _handle_serial_line(self):
        line = self._get_serial_line()
        if line is None:
            return

        line = line.decode('latin1')

        display = True
        if line == '':
            display = False

        if line.startswith('schema '):
            self._serial_state = self.STATE_SCHEMA
            self._schema_name = line.split(' ', 1)[1].strip()
        elif line.startswith('emit '):
            self._serial_state = self.STATE_DATA
            self._schema_name = line.split(' ', 1)[1].strip()
            display = False

        if display:
            self._console.add_text(self._prefix + line + '\n')

    def _get_serial_line(self):
        result = self._stream.read_line()
        return result

    def update_config(self, callback):
        # Clear out our config tree.
        self._config_tree_item.takeChildren()
        self._config_tree_items = {}

        self._config_callback = callback
        self.write_line('conf enumerate\r\n')

        # TODO jpieper: In the current protocol this is racy, as there
        # is no header on the config enumeration.  I should probably
        # add one.
        self._serial_state = self.STATE_CONFIG

    def _handle_config(self):
        line = self._get_serial_line()
        if not line:
            return

        line = line.decode('latin1')
        self._console.add_text(self._prefix + line + '\n')

        if line.startswith('OK'):
            # We're done with config now.
            self._serial_state = self.STATE_LINE
            cbk, self._config_callback = self._config_callback, None
            if cbk:
                cbk()
        else:
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
                        int(QtCore.Qt.ItemIsEditable) |
                        int(QtCore.Qt.ItemIsSelectable) |
                        int(QtCore.Qt.ItemIsEnabled)))
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

            # TODO(jpieper)
            # self.ui.configTreeWidget.resizeColumnToContents(0)

    def update_telemetry(self, callback):
        self._data_tree_item.takeChildren()
        self._telemetry_records = {}

        self._telemetry_callback = callback
        self.write_line('tel list\r\n')

        self._serial_state = self.STATE_TELEMETRY

    def write_line(self, line):
        self._console.add_text(self._prefix + line)
        self.write(line.encode('latin1'))

    def _handle_telemetry(self):
        line = self._get_serial_line()
        if not line:
            return

        line = line.decode('latin1')
        self._console.add_text(self._prefix + line + '\n')

        if line.startswith('OK'):
            # Now we need to start getting schemas.
            self._serial_state = self.STATE_LINE
            self._update_schema()
        else:
            name = line.strip()
            self._telemetry_records[name] = None

    def _update_schema(self):
        # Find a channel we don't have a schema for and request it.
        for name in self._telemetry_records.keys():
            if self._telemetry_records[name] is None:
                self.write_line('tel schema %s\r\n' % name)
                self._serial_state = self.STATE_LINE
                return

        self._serial_state = self.STATE_LINE
        # Guess we are done.  Update our tree view.

        # TODO(jpieper)
        # self.ui.telemetryTreeWidget.resizeColumnToContents(0)

        cbk, self._telemetry_callback = self._telemetry_callback, None
        if cbk:
            cbk()

    def _handle_schema(self):
        schema = self._handle_sized_block()
        if not schema:
            return

        name, self._schema_name = self._schema_name, None

        if name in self._telemetry_records:
            if self._telemetry_records[name]:
                return

        archive = reader.Type.from_binary(io.BytesIO(schema), name=name)

        record = Record(archive)
        self._telemetry_records[name] = record
        record.tree_item = self._add_schema_to_tree(name, archive, record)

        self._console.add_text(self._prefix + '<schema name=%s>\n' % name)

        # Now look to see if there are any more we should request.
        self._update_schema()

    def _handle_data(self):
        data = self._handle_sized_block()
        if not data:
            return

        name, self._schema_name = self._schema_name, None

        if name not in self._telemetry_records:
            return

        record = self._telemetry_records[name]
        if record:
            struct = record.archive.read(reader.Stream(io.BytesIO(data)))
            record.update(struct)
            _set_tree_widget_data(record.tree_item, struct)

        self._serial_state = self.STATE_LINE

    def _handle_sized_block(self):
        block = self._stream.read_sized_block()

        if block is None:
            return

        if block == False:
            print('Invalid schema size, skipping whatever we were doing.')
            self._serial_state = self.STATE_LINE
            return

        return block

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

        def add_item(parent, element):
            if isinstance(element, reader.ObjectType):
                for field in element.fields:
                    name = field.name

                    item = QtWidgets.QTreeWidgetItem(parent)
                    item.setText(0, name)

                    add_item(item, field.type_class)

        add_item(item, schema_data)
        return item


class TviewMainWindow():
    def __init__(self, options, parent=None):
        self.options = options
        self.port = None
        self.devices = []
        self.default_rate = 100

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

        self.ui.configTreeWidget.setItemDelegateForColumn(
            0, NoEditDelegate(self.ui))
        self.ui.configTreeWidget.itemExpanded.connect(
            self._handle_config_expanded)
        self.ui.configTreeWidget.itemChanged.connect(
            self._handle_config_item_changed)

        self.ui.plotItemRemoveButton.clicked.connect(
            self._handle_plot_item_remove)

        self.console = TviewConsoleWidget()
        self.console.ansi_codes = False
        self.console.line_input.connect(self._handle_user_input)
        self.ui.consoleDock.setWidget(self.console)

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

        self.devices = []
        self.ui.configTreeWidget.clear()
        self.ui.telemetryTreeWidget.clear()

        for device_id in moteus.moteus_tool.expand_targets(
                self.options.devices or ['1']):
            config_item = QtWidgets.QTreeWidgetItem()
            config_item.setText(0, str(device_id))
            self.ui.configTreeWidget.addTopLevelItem(config_item)

            data_item = QtWidgets.QTreeWidgetItem()
            data_item.setText(0, str(device_id))
            self.ui.telemetryTreeWidget.addTopLevelItem(data_item)

            device = Device(device_id, self.transport,
                            self.console, '{}>'.format(device_id),
                            config_item,
                            data_item)

            config_item.setData(0, QtCore.Qt.UserRole, device)
            asyncio.create_task(device.start())

            self.devices.append(device)

    def _handle_startup(self):
        self.console._control.setFocus()
        self._open()

    async def _dispatch_until(self, predicate):
        while True:
            message = await self.transport.read()
            if message is None:
                continue
            source_id = (message.arbitration_id >> 8) & 0xff
            any_data_read = False
            for device in self.devices:
                if device.number == source_id:
                    any_data_read = device.process_message(message)
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

            await device.poll()

            try:
                this_data_read = await asyncio.wait_for(
                    self._dispatch_until(
                        lambda x: (x.arbitration_id >> 8) & 0xff == device.number),
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
        device_lines = [x.strip() for x in line.split('&&')]
        now = time.time()
        current_delay_ms = 0
        for line in device_lines:
            delay_re = re.search(r"^:(\d+)$", line)
            device_re = re.search(r"^(A|\d+)>(.*)$", line)
            if delay_re:
                current_delay_ms += int(delay_re.group(1))
                continue
            elif device_re:
                if device_re.group(1) == 'A':
                    device_nums = [x.number for x in self.devices]
                else:
                    device_nums = [int(device_re.group(1))]
                line = device_re.group(2)
            else:
                device_nums = [self.devices[0].number]
            devices = [x for x in self.devices if x.number in device_nums]
            writer = self.make_writer(devices, line)

            if current_delay_ms > 0:
                QtCore.QTimer.singleShot(current_delay_ms, writer)
            else:
                writer()

    def _handle_tree_expanded(self, item):
        self.ui.telemetryTreeWidget.resizeColumnToContents(0)
        user_data = item.data(0, QtCore.Qt.UserRole)
        if user_data:
            user_data.expand()

    def _handle_tree_collapsed(self, item):
        user_data = item.data(0, QtCore.Qt.UserRole)
        if user_data:
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
        device.config_item_changed(_get_item_name(item), item.text(1))

    def _handle_plot_item_remove(self):
        index = self.ui.plotItemCombo.currentIndex()

        if index < 0:
            return

        item = self.ui.plotItemCombo.itemData(index)
        self.ui.plotWidget.remove_plot(item)
        self.ui.plotItemCombo.removeItem(index)


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    # These two commands are aliases.
    parser.add_argument('-d', '--devices', '-t', '--target',
                        action='append', type=str, default=[])

    parser.add_argument('--max-receive-bytes', default=48, type=int)

    moteus.make_transport_args(parser)

    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    loop = asyncqt.QEventLoop(app)
    asyncio.set_event_loop(loop)

    # To work around https://bugreports.qt.io/browse/PYSIDE-88
    app.aboutToQuit.connect(lambda: os._exit(0))

    tv = TviewMainWindow(args)
    tv.show()

    app.exec_()


if __name__ == '__main__':
    main()
