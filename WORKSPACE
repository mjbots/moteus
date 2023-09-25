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

workspace(name = "com_github_mjbots_moteus")

BAZEL_VERSION = "5.4.1"
BAZEL_VERSION_SHA = "5d90515f84b5ee1fd6ec22ee9e83103e77ed1a907ee5eec198fef3a5b45abf13"

load("//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()

# Do mjlib.
load("@com_github_mjbots_mjlib//tools/workspace:default.bzl",
     mjlib_add = "add_default_repositories")

mjlib_add()

# Now bazel-toolchain
load("@com_github_mjbots_bazel_toolchain//toolchain:deps.bzl", "bazel_toolchain_dependencies")
bazel_toolchain_dependencies()

load("@com_github_mjbots_bazel_toolchain//toolchain:rules.bzl", "llvm_toolchain")
llvm_toolchain(
    name = "llvm_toolchain",
    llvm_version = "10.0.0",
    urls = {
        "windows" : ["https://github.com/mjbots/bazel-toolchain/releases/download/0.5.6-mj20201011/LLVM-10.0.0-win64.tar.xz"],
    },
    sha256 = {
        "windows" : "2851441d3993c032f98124a05e2aeb43010b7a85f0f7441103d36ae8d00afc18",
    },
    strip_prefix = {
        "windows" : "LLVM",
    }
)

load("@llvm_toolchain//:toolchains.bzl", "llvm_register_toolchains")
llvm_register_toolchains()


# Now rules_mbed
load("@com_github_mjbots_rules_mbed//:rules.bzl", mbed_register = "mbed_register")
load("@com_github_mjbots_rules_mbed//tools/workspace/mbed:repository.bzl", "mbed_repository")

mbed_register(
    config = {
        "mbed_target": "targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F446xE/TARGET_NUCLEO_F446ZE",
        "mbed_config" : {},
    },
)

mbed_repository(
    name = "com_github_ARMmbed_mbed-g4-bootloader",
    target = "targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE",
    config = {
        "mbed_target": "targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE",

        "MBED_CONF_RTOS_PRESENT": "0",
        "MBED_CONF_TARGET_LSE_AVAILABLE": "0",

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
        "MBED_CONF_TARGET_LSE_AVAILABLE": "0",

        # Start our code at sector a ways in to leave room for the
        # isrs, persistent storage, and the bootloader.

        "MBED_APP_START": "0x8010000",
        "MBED_APP_SIZE":  "0x0070000",

        "MBED_US_TIMER_TIM": "TIM15",
        "MBED_US_TIMER_TIM_USCORE": "TIM15_",
        "MBED_US_TIMER_USCORE_TIM": "_TIM15",
        "TIM_MST_IRQ": "TIM1_BRK_TIM15_IRQn",
        "TIM_MST_BIT_WIDTH": "16",
    },
)

# And finally, bazel_deps
load("@com_github_mjbots_bazel_deps//tools/workspace:default.bzl",
     bazel_deps_add = "add_default_repositories")
bazel_deps_add()
