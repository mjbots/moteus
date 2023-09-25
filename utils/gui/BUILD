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

test_suite(
    name = "host",
    tests = [
        ":bdist_test",
    ],
)

template_file(
    name = "setup.py",
    src = "setup.py.TPL",
    substitutions = {
        "{{VERSION}}" : '"{}"'.format(VERSION),
    },
)

genrule(
    name = "bdist_wheel",
    srcs = ["setup.py", "README.md", "//utils/gui/moteus_gui:tview",
            # We separately list :datafiles because bazel does not have
            # any way for py_library/binary rules to carry around non-.py
            # dependencies for "compile-time" use.  This lets us get
            # .ui files too.
            "//utils/gui/moteus_gui:datafiles",
            "//utils/gui/moteus_gui:version.py",
    ],
    outs = [
        "moteus_gui-{ver}-py3-none-any.whl".format(ver=VERSION),
        "moteus-gui-{ver}.tar.gz".format(ver=VERSION),
    ],
    cmd = (
        "mkdir -p $$(dirname $(location moteus-gui-{ver}.tar.gz)) && " +
        "RELOUT=$$(realpath --relative-to $$(dirname $(location README.md)) " +
        "          $$(dirname $(location moteus-gui-{ver}.tar.gz))) && " +
        "SRCROOT=$$(dirname $(location README.md)) && " +
        "ln -s $$(realpath $(location setup.py)) $$SRCROOT/setup.py && " +
        "ln -s $$(realpath $(location //utils/gui/moteus_gui:version.py)) $$SRCROOT/moteus_gui/version.py && " +
        "cd $$SRCROOT && " +
        "python3 setup.py sdist --dist-dir $$RELOUT 1>/dev/null && " +
        "python3 setup.py bdist_wheel --dist-dir $$RELOUT 1> /dev/null " +
        ""
    ).format(ver=VERSION),
)

sh_test(
    name = "bdist_test",
    data = [":bdist_wheel"],
    srcs = ["test/bdist_test.sh"],
)
