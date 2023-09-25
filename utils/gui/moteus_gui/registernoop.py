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


# This file exists merely to shut up PySide6's warning about
# PYSIDE_DESIGNER_PLUGINS

import PySide6
from PySide6 import QtDesigner

class NoopPlugin(QtDesigner.QDesignerCustomWidgetInterface):
    def name(self):
        return 'noop'

QtDesigner.QPyDesignerCustomWidgetCollection.addCustomWidget(NoopPlugin())
