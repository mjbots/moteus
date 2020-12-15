# -*- python -*-

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

filegroup(
    name = "libfiles",
    srcs = [
        "aioserial.py",
        "command.py",
        "fdcanusb.py",
        "moteus.py",
        "multiplex.py",
        "pythoncan.py",
        "router.py",
        "transport.py",
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
    name = "router_test",
    srcs = ["test/router_test.py"],
    deps = [":moteus"],
)

test_suite(
    name = "test",
    tests = [
        ":moteus_test",
        ":multiplex_test",
        ":router_test",
    ],
)
