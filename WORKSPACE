# -*- python -*-

# Copyright 2018 Josh Pieper, jjp@pobox.com.
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

workspace(name = "com_github_mjbots_moteus")

BAZEL_VERSION = "0.19.0"
BAZEL_VERSION_SHA = "001371df816005e907670f483f810ee935176bdc811a5ffa2314c59766b68ec4"

load("//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()

load("@com_github_mjbots_rules_mbed//:rules.bzl", mbed_register = "mbed_register")

mbed_register(
    config = {
        "mbed_target": "targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F446xE/TARGET_NUCLEO_F446ZE",
        "mbed_config": {
            "MBED_CONF_RTOS_PRESENT": "0",

            # Start our code at sector 4, leaving sectors 0, 1, 2, and
            # 3 for application data storage.
            "MBED_APP_START": "0x8010000",
            "MBED_APP_SIZE":  "0x0070000",
        },
    }
)

load("@com_github_mjbots_bazel_deps//tools/workspace:default.bzl",
     bazel_deps_add = "add_default_repositories")
bazel_deps_add()
