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

load("//tools:python.bzl", "VERSION")
load("//tools/workspace:template_file.bzl", "template_file")

template_file(
    name = "version.py",
    src = "version.py.TPL",
    substitutions = {
        "{{VERSION}}": '"{}"'.format(VERSION),
    },
)

filegroup(
    name = "libfiles",
    srcs = [
        "aioserial.py",
        "aiostream.py",
        "calibrate_encoder.py",
        "command.py",
        "export.py",
        "fdcanusb.py",
        "moteus.py",
        "moteus_tool.py",
        "multiplex.py",
        "posix_aioserial.py",
        "pythoncan.py",
        "reader.py",
        "regression.py",
        "router.py",
        "transport.py",
        "version.py",
        "win32_aioserial.py",
    ]
)

py_library(
    name = "moteus",
    srcs = [
        "__init__.py",
        ":libfiles",
    ],
    imports = [
        "..",
    ],
)

py_binary(
    name = "manual_calibrate_encoder",
    srcs = [
        "manual_calibrate_encoder.py",
    ],
    deps = [
        ":moteus",
    ],
)

py_test(
    name = "calibrate_encoder_test",
    srcs = ["test/calibrate_encoder_test.py"],
    deps = ["moteus"],
    # Just so it is built.
    data = [":manual_calibrate_encoder"],
)

py_test(
    name = "multiplex_test",
    srcs = ["test/multiplex_test.py"],
    deps = [":moteus"],
)

py_test(
    name = "moteus_test",
    srcs = ["test/moteus_test.py"],
    deps = [":moteus"],
)

py_test(
    name = "reader_test",
    srcs = ["test/reader_test.py"],
    deps = [":moteus"],
)

py_test(
    name = "regression_test",
    srcs = ["test/regression_test.py"],
    deps = [":moteus"],
)

py_test(
    name = "router_test",
    srcs = ["test/router_test.py"],
    deps = [":moteus"],
)

test_suite(
    name = "test",
    tests = [
        ":calibrate_encoder_test",
        ":moteus_test",
        ":multiplex_test",
        ":reader_test",
        ":regression_test",
        ":router_test",
    ],
)
