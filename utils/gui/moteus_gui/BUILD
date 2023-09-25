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

package(default_visibility = ["//visibility:public"])

load("//tools/workspace:template_file.bzl", "template_file")
load("//tools:python.bzl", "VERSION")

template_file(
    name = "version.py",
    src = "version.py.TPL",
    substitutions = {
        "{{VERSION}}": '"{}"'.format(VERSION),
    },
)

filegroup(
    name = "datafiles",
    srcs = [
        "__init__.py",
        "version.py",

        "tview_main_window.ui",
    ]
)

py_library(
    name = "tview",
    srcs = [
        "tview.py",
    ],
    deps = [
        "//lib/python/moteus",
    ],
    data = [
        ":datafiles",
    ],
)
