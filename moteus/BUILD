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

package(default_visibility = ["//visibility:public"])
load("@com_github_ARMmbed_mbed-os//:rules.bzl", "mbed_binary")

mbed_binary(
    name = "moteus",
    srcs = [
        "assert.cc",
        "error.h",
        "error.cc",
        "irq_callback_table.h",
        "irq_callback_table.cc",
        "stm32f446_async_uart.h",
        "stm32f446_async_uart.cc",
        "stm32f446_bldc_foc.h",
        "stm32f446_bldc_foc.cc",
        "main.cc",
    ],
    deps = [
        "//mjlib/base:assert",
        "//mjlib/base:system_error",
        "//mjlib/micro:async_exclusive",
        "//mjlib/micro:async_stream",
        "//mjlib/micro:command_manager",
        "//mjlib/micro:pool_ptr",
        "//mjlib/micro:static_function",
    ],
)
