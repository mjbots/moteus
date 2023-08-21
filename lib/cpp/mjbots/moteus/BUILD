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

filegroup(
    name = "src",
    srcs = [
        "moteus_multiplex.h",
        "moteus_optional.h",
        "moteus_protocol.h",
        "moteus_tokenizer.h",
        "moteus_transport.h",
        "moteus.h",
    ],
)

cc_library(
    name = "moteus",
    hdrs = [":src"],
)

cc_test(
    name = "test",
    srcs = [
        "test/moteus_test.cc",
        "test/moteus_protocol_test.cc",
        "test/test_main.cc",
    ],
    copts = [
        "-Ilib/cpp",
    ],
    deps = [
        ":moteus",
        "@boost//:test",
    ],
    size = "small",
)
