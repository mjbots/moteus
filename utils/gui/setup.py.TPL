# -*- python -*-

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

import setuptools
import pathlib

here = pathlib.Path(__file__).parent.resolve()

# Get the long description from the README file
long_description = (here / 'README.md').read_text(encoding='utf-8')

setuptools.setup(
    name = 'moteus-gui',
    version = {{VERSION}},
    description = 'moteus brushless controller graphical user interfaces',
    long_description = long_description,
    long_description_content_type = 'text/markdown',
    url = 'https://github.com/mjbots/moteus',
    author = 'mjbots Robotic Systems',
    author_email = 'info@mjbots.com',
    classifiers = [
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
    ],
    keywords='moteus',
    packages = [
        'moteus_gui',
    ],
    include_package_data = True,
    package_data = {
        'moteus_gui' : ['tview_main_window.ui'],
    },
    entry_points = {
        'console_scripts': ['tview=moteus_gui.tview:main'],
    },
    python_requires = '>=3.7, <4',
    install_requires = [
        'asyncqt>=0.8',
        'moteus>=0.3.26',
        'matplotlib>=3.5',
        # For some reason, matplotlib can barf without this, but
        # doesn't actually list it as a dependency on Windows.
        'msvc-runtime;platform_system=="Windows"',
        'numpy',
        'PySide2;platform_system=="Linux"',
        'PySide6>=6.2;platform_system!="Linux"',
        'qtconsole>=0.5.2',
        'qtpy>=2.0.1',
    ],
)
