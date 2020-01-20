# -*- python -*-

# Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

test_suite(
    name = "host",
    tests = [
        "//moteus:host",
    ],
)

filegroup(
    name = "target",
    srcs = [
        "//moteus:moteus",
        "//moteus:power_dist",
        "//moteus:can_bootloader",
    ],
)

filegroup(
    name = "f4_target",
    srcs = [
        "//moteus:f4_moteus",
        "//moteus:imu_junction_dumb",
        "//moteus:uart_bootloader",
    ],
)

exports_files(["tsconfig.json"])
