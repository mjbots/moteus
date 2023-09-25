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

py_binary(
    name = "calibrate_encoder",
    srcs = ["calibrate_encoder.py"],
)

py_binary(
    name = "tview",
    srcs = ["tview.py"],
    main = "tview.py",
    deps = [
        "//utils/gui/moteus_gui:tview"
    ],
)

py_binary(
    name = "moteus_tool",
    srcs = ["moteus_tool.py"],
    deps = [
        "//lib/python/moteus",
    ],
)

cc_test(
    name = "test",
    srcs = [
        "test/dummy_test.cc",
        "test/test_main.cc",
    ],
    deps = [
        "@boost//:test",
    ],
    data = [
        # Just so they are built.
        ":calibrate_encoder",
        ":dynamometer_drive",
        ":dyno_static_torque_ripple",
        ":firmware_validate",
        ":moteus_tool",
        ":tview",
    ],
    size = "small",
)

cc_binary(
    name = "dynamometer_drive",
    srcs = [
        "dynamometer_drive.cc",
        "line_reader.h",
        "moteus_subset.h",
        "run_for.h",
    ],
    deps = [
        "//fw:common",
        "@com_github_mjbots_mjlib//mjlib/base:clipp",
        "@com_github_mjbots_mjlib//mjlib/base:clipp_archive",
        "@com_github_mjbots_mjlib//mjlib/base:fail",
        "@com_github_mjbots_mjlib//mjlib/base:visitor",
        "@com_github_mjbots_mjlib//mjlib/io:now",
        "@com_github_mjbots_mjlib//mjlib/io:selector",
        "@com_github_mjbots_mjlib//mjlib/io:stream_factory",
        "@com_github_mjbots_mjlib//mjlib/multiplex:asio_client",
        "@com_github_mjbots_mjlib//mjlib/multiplex:stream_asio_client_builder",
        "@com_github_mjbots_mjlib//mjlib/telemetry:binary_schema_parser",
        "@com_github_mjbots_mjlib//mjlib/telemetry:binary_write_archive",
        "@com_github_mjbots_mjlib//mjlib/telemetry:file_writer",
        "@com_github_mjbots_mjlib//mjlib/telemetry:mapped_binary_reader",
        "@fmt",
    ],
)

py_binary(
    name = "dyno_static_torque_ripple",
    srcs = ["dyno_static_torque_ripple.py"],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/telemetry:py_file_reader",
    ],
)

py_binary(
    name = "firmware_validate",
    srcs = ["firmware_validate.py"],
    deps = [
        "@bazel_tools//tools/python/runfiles",
    ],
    data = [
        ":dynamometer_drive",
        ":dyno_static_torque_ripple",
    ],
)

test_suite(
    name = "host",
    tests = [
        "test",
        "//utils/gui:host",
    ],
)
