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
        "//lib/python/moteus:test",
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
    srcs = [
        "setup.py",
        "README.md",
        "//lib/python/moteus",
        "//lib/python/moteus:version.py",
    ],
    outs = [
        "moteus-{ver}-py3-none-any.whl".format(ver=VERSION),
        "moteus-{ver}.tar.gz".format(ver=VERSION),
    ],
    cmd = (
        "mkdir -p $$(dirname $(location moteus-{ver}.tar.gz)) && " +
        "RELOUT=$$(realpath --relative-to $$(dirname $(location README.md)) " +
        "          $$(dirname $(location moteus-{ver}.tar.gz))) && " +
        "SRCROOT=$$(dirname $(location README.md)) && " +
        "ln -s $$(realpath $(location setup.py)) $$SRCROOT/setup.py && " +
        "ln -s $$(realpath $(location //lib/python/moteus:version.py)) $$SRCROOT/moteus/version.py && " +
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
