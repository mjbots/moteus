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

workspace(name = "com_github_mjbots_moteus",
          managed_directories = {
              "@npm": ["node_modules"],
          })

BAZEL_VERSION = "1.2.0"
BAZEL_VERSION_SHA = "34eb178663a9f9c6765db72dd58958cdac64c5f08dd51e5b67e28d466079bd1e"

load("//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()

# Do the Javascript part of our initialization.
load("//tools/workspace:npm_stage1.bzl", "setup_npm_stage1")
setup_npm_stage1()

load("//tools/workspace:npm_stage2.bzl", "setup_npm_stage2")
setup_npm_stage2()

load("//tools/workspace:npm_stage3.bzl", "setup_npm_stage3")
setup_npm_stage3()


# Now rules_mbed
load("@com_github_mjbots_rules_mbed//:rules.bzl", mbed_register = "mbed_register")
load("@com_github_mjbots_rules_mbed//tools/workspace/mbed:repository.bzl", "mbed_repository")

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

mbed_repository(
    name = "com_github_ARMmbed_mbed-os-bootloader",
    target = "targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F446xE/TARGET_NUCLEO_F446ZE",
    config = {
        "mbed_target": "targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F446xE/TARGET_NUCLEO_F446ZE",

        "MBED_CONF_RTOS_PRESENT": "0",

        # The application location and offset are directly
        # configured in the custom linker script, not here.

        # We need to use as few bytes as possible.
        "NDEBUG": "1",
    }
)

mbed_repository(
    name = "com_github_ARMmbed_mbed-g4-bootloader",
    target = "targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE",
    config = {
        "mbed_target": "targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE",

        "MBED_CONF_RTOS_PRESENT": "0",

        # We need to use as few bytes as possible.
        "NDEBUG": "1",
    },
)


mbed_repository(
    name = "com_github_ARMmbed_mbed-g4",
    target = "targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE",
    config = {
        "mbed_target": "targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE",

        "MBED_CONF_RTOS_PRESENT": "0",

        # Start our code at sector a ways in to leave room for the
        # isrs, persistent storage, and the bootloader.

        "MBED_APP_START": "0x8010000",
        "MBED_APP_SIZE":  "0x0070000",
    },
)


# And finally, bazel_deps
load("@com_github_mjbots_bazel_deps//tools/workspace:default.bzl",
     bazel_deps_add = "add_default_repositories")
bazel_deps_add()
