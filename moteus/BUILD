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

cc_library(
    name = "common",
    hdrs = [
        "foc.h",
        "math.h",
    ],
)

mbed_binary(
    name = "moteus",
    srcs = [
        "as5047.h",
        "assert.cc",
        "bldc_foc.h",
        "bldc_foc.cc",
        "board_debug.h",
        "board_debug.cc",
        "drv8323.h",
        "drv8323.cc",
        "error.h",
        "error.cc",
        "irq_callback_table.h",
        "irq_callback_table.cc",
        "position_sensor.h",
        "stm32f446_async_uart.h",
        "stm32f446_async_uart.cc",
        "stm32_flash.h",
        "stm32_flash.cc",
        "system_info.h",
        "system_info.cc",
        "main.cc",
    ],
    deps = [
        ":common",
        "//mjlib/base:assert",
        "//mjlib/base:system_error",
        "//mjlib/micro:async_exclusive",
        "//mjlib/micro:async_stream",
        "//mjlib/micro:command_manager",
        "//mjlib/micro:persistent_config",
        "//mjlib/micro:pool_ptr",
        "//mjlib/micro:static_function",
        "//mjlib/micro:telemetry_manager",
    ],
    linker_script = "stm32f446.ld",
)

genrule(
    name = "bin",
    srcs = ["moteus.elf"],
    outs = [
        "moteus.08000000.bin",
        "moteus.08010000.bin",
    ],
    cmd = ("$(OBJCOPY) -Obinary -j .text -j .ARM.extab -j .ARM.exidx -j .data -j .bss $< $(location moteus.08010000.bin) && " +
           "$(OBJCOPY) -Obinary -j .isr_vector $< $(location moteus.08000000.bin)"),
    output_to_bindir = True,
)

OCD = (
    "openocd " +
    "-f /usr/share/openocd/scripts/interface/stlink-v2.cfg " +
    "-f /usr/share/openocd/scripts/target/stm32f4x_stlink.cfg "
)

genrule(
    name = "flash",
    tags = ["manual"],
    srcs = ["moteus.08000000.bin", "moteus.08010000.bin"],
    outs = ["flash.stamp"],
    cmd = (OCD + " -c init -c \"reset_config none separate; program $(location moteus.08000000.bin) verify 0x8000000; program $(location moteus.08010000.bin) verify reset exit 0x08010000\" && touch $@"),
)

cc_test(
    name = "test",
    srcs = [
        "test/foc_test.cc",
        "test/test_main.cc",
    ],
    deps = [
        ":common",
        "@boost//:test",
    ],
)
