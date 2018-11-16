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
load("@com_github_mjbots_rules_mbed//tools/workspace/mbed:repository.bzl", "mbed_repository")

mbed_register(
    config = {
        "mbed_target": "targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F411xE/TARGET_NUCLEO_F466ZE",
    }
)

mbed_repository(
    name = "com_github_ARMmbed_mbed-os-nucleo411",
    target = "targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F411xE/TARGET_NUCLEO_F411RE",
)


load("@com_github_mjbots_bazel_deps//tools/workspace:default.bzl",
     bazel_deps_add = "add_default_repositories")
bazel_deps_add()
