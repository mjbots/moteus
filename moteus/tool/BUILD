# -*- python -*-

# Copyright 2018-2020 Josh Pieper, jjp@pobox.com.
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
        "@com_github_mjbots_mjlib//mjlib/telemetry:py_reader",
        "@bazel_tools//tools/python/runfiles",
    ],
    data = ["tview_main_window.ui"],
    imports = [
        "../../../mjlib",
    ],
)

cc_library(
    name = "moteus_tool_lib",
    hdrs = ["moteus_tool.h"],
    srcs = [
        "calibrate.h",
        "calibrate.cc",
        "line_reader.h",
        "moteus_subset.h",
        "moteus_tool.cc",
        "run_for.h",
    ],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/base:clipp",
        "@com_github_mjbots_mjlib//mjlib/base:inplace_function",
        "@com_github_mjbots_mjlib//mjlib/base:json5_read_archive",
        "@com_github_mjbots_mjlib//mjlib/base:json5_write_archive",
        "@com_github_mjbots_mjlib//mjlib/base:program_options_archive",
        "@com_github_mjbots_mjlib//mjlib/base:time_conversions",
        "@com_github_mjbots_mjlib//mjlib/io:async_sequence",
        "@com_github_mjbots_mjlib//mjlib/io:debug_time",
        "@com_github_mjbots_mjlib//mjlib/io:now",
        "@com_github_mjbots_mjlib//mjlib/io:selector",
        "@com_github_mjbots_mjlib//mjlib/io:stream_copy",
        "@com_github_mjbots_mjlib//mjlib/io:stream_factory",
        "@com_github_mjbots_mjlib//mjlib/multiplex:stream_asio_client_builder",
        "@com_github_mjbots_mjlib//mjlib/telemetry:binary_schema_parser",
        "@com_github_mjbots_mjlib//mjlib/telemetry:mapped_binary_reader",
        "@boost//:filesystem",
        "@boost//:system",
        "@com_github_serge1_elfio//:elfio",
        "@eigen",
        "@fmt",
    ],
)

cc_test(
    name = "test",
    srcs = [
        "test/calibrate_test.cc",
        "test/test_main.cc",
    ],
    deps = [
        ":moteus_tool_lib",
        "@boost//:test",
    ],
    data = [
        # Just so they are built.
        ":moteus_tool",
        ":dynamometer_drive",
        ":dyno_static_torque_ripple",
        ":firmware_validate",
    ],
    size = "small",
)

cc_binary(
    name = "moteus_tool",
    srcs = ["moteus_tool_main.cc"],
    deps = [":moteus_tool_lib"],
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
        "//moteus:common",
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
        ":moteus_tool",
        ":dynamometer_drive",
        ":dyno_static_torque_ripple",
    ],
)
