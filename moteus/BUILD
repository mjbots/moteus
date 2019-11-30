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
load("@com_github_ARMmbed_mbed-os//:rules.bzl", "mbed_binary")
load("@com_github_ARMmbed_mbed-g4//:rules.bzl", g4_mbed_binary = "mbed_binary")
load("@com_github_ARMmbed_mbed-os-bootloader//:rules.bzl", bootloader_mbed_binary = "mbed_binary")
load("@com_github_ARMmbed_mbed-g4-bootloader//:rules.bzl", g4_bootloader_mbed_binary = "mbed_binary")

cc_library(
    name = "common",
    hdrs = [
        "foc.h",
        "math.h",
    ],
    srcs = [
        "foc.cc",
    ],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/micro:atomic_event_queue",
    ],
)

bootloader_mbed_binary(
    name = "imu_junction_dumb",
    srcs = [
        "imu_junction_dumb.cc",
        "imu_junction_hw.h",
        "millisecond_timer.h",
        "stm32_serial.cc",
        "stm32_serial.h",
    ],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/base:assert",
    ],
)

g4_bootloader_mbed_binary(
    name = "power_dist",
    srcs = [
        "millisecond_timer.h",
        "power_dist.cc",
        "power_dist_hw.h",
    ],
)

bootloader_mbed_binary(
    name = "bootloader",
    srcs = [
        "bootloader.h",
        "bootloader.cc",
    ],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/base:buffer_stream",
        "@com_github_mjbots_mjlib//mjlib/base:tokenizer",
        "@com_github_mjbots_mjlib//mjlib/multiplex:format",
        "@com_github_mjbots_mjlib//mjlib/multiplex:stream",
        "@boost",
    ],
    linker_script = "stm32f446_bootloader.ld",
    enable_wrappers = False,
    linkopts = [
        "-Wl,--wrap,atexit",
    ],
)

MOTEUS_SOURCES = [
    "as5047.h",
    "moteus_assert.cc",
    "bldc_servo.h",
    "bldc_servo.cc",
    "board_debug.h",
    "board_debug.cc",
    "bootloader.h",
    "drv8323.h",
    "drv8323.cc",
    "error.h",
    "error.cc",
    "hardfault.s",
    "firmware_info.h",
    "firmware_info.cc",
    "irq_callback_table.h",
    "irq_callback_table.cc",
    "moteus_controller.h",
    "moteus_controller.cc",
    "moteus_hardfault_handler.c",
    "moteus_hw.h",
    "millisecond_timer.h",
    "motor_driver.h",
    "position_sensor.h",
    "stm32f446_spi.h",
    "stm32_flash.h",
    "stm32_flash.cc",
    "stm32_serial.h",
    "stm32_serial.cc",
    "stm32.h",
    "system_info.h",
    "system_info.cc",
    "moteus.cc",
]

MOTEUS_DEPS = [
    ":common",
    "@com_github_mjbots_mjlib//mjlib/base:assert",
    "@com_github_mjbots_mjlib//mjlib/base:inplace_function",
    "@com_github_mjbots_mjlib//mjlib/base:pid",
    "@com_github_mjbots_mjlib//mjlib/base:windowed_average",
    "@com_github_mjbots_mjlib//mjlib/micro:async_exclusive",
    "@com_github_mjbots_mjlib//mjlib/micro:async_stream",
    "@com_github_mjbots_mjlib//mjlib/micro:command_manager",
    "@com_github_mjbots_mjlib//mjlib/micro:error_code",
    "@com_github_mjbots_mjlib//mjlib/micro:persistent_config",
    "@com_github_mjbots_mjlib//mjlib/micro:pool_ptr",
    "@com_github_mjbots_mjlib//mjlib/micro:telemetry_manager",
    "@com_github_mjbots_mjlib//mjlib/multiplex:micro_server",
]

mbed_binary(
    name = "f4_moteus",
    srcs = MOTEUS_SOURCES + [
        "stm32f446_async_uart.h",
        "stm32f446_async_uart.cc",
    ],
    deps = MOTEUS_DEPS,
    linker_script = "stm32f446.ld",
)

g4_mbed_binary(
    name = "moteus",
    srcs = MOTEUS_SOURCES + [
        "stm32g4_async_uart.h",
        "stm32g4_async_uart.cc",
    ],
    deps = MOTEUS_DEPS,
    linker_script = "stm32g474.ld",
)

genrule(
    name = "bin",
    srcs = ["moteus.elf", "bootloader.elf"],
    outs = [
        "moteus.08000000.bin",
        "bootloader.0800c000.bin",
        "moteus.08010000.bin",
    ],
    cmd = ("$(OBJCOPY) -Obinary -j .text -j .ARM.extab -j .ARM.exidx -j .data -j .bss $(location moteus.elf) $(location moteus.08010000.bin) && " +
           "$(OBJCOPY) -Obinary -j .text -j .ARM.extab -j .ARM.exidx -j .data -j .bss $(location bootloader.elf) $(location bootloader.0800c000.bin) && " +
           "$(OBJCOPY) -Obinary -j .isr_vector $(location moteus.elf) $(location moteus.08000000.bin)"),
    toolchains = [
        "@bazel_tools//tools/cpp:current_cc_toolchain",
    ],
    output_to_bindir = True,
)

OCD = (
    "openocd " +
    "-f interface/stlink.cfg " +
    "-f target/stm32f4x.cfg "
)

OCD_G4 = (
    "openocd " +
    "-f interface/stlink.cfg " +
    "-f target/stm32g4x.cfg "
)

genrule(
    name = "flash",
    tags = ["manual"],
    srcs = ["moteus.08000000.bin", "moteus.08010000.bin", "bootloader.0800c000.bin"],
    outs = ["flash.stamp"],
    cmd = (OCD + " -c init -c \"reset_config none separate; program $(location moteus.08000000.bin) verify 0x8000000; program $(location bootloader.0800c000.bin) verify 0x800c000; program $(location moteus.08010000.bin) verify reset exit 0x08010000\" && touch $@"),
)

genrule(
    name = "power_dist_flash",
    tags = ["manual"],
    srcs = ["power_dist.bin"],
    outs = ["power_dist.stamp"],
    cmd = (OCD_G4 + "-c init -c \"reset_config none separate; program $(location power_dist.bin) verify 0x8000000 reset exit 0x8000000\" && touch $@"),
)

genrule(
    name = "imu_junction_dumb_flash",
    tags = ["manual"],
    srcs = ["imu_junction_dumb.bin"],
    outs = ["imu_junction_dumb.stamp"],
    cmd = (OCD + "-c init -c \"reset_config none separate; program $(location imu_junction_dumb.bin) verify 0x8000000 reset exit 0x8000000\" && touch $@"),
)

genrule(
    name = "imu_junction_bin",
    srcs = ["imu_junction.elf"],
    outs = [
        "imu_junction.08000000.bin",
        "imu_junction.08010000.bin",
    ],
    cmd = ("$(OBJCOPY) -Obinary -j .text -j .ARM.extab -j .ARM.exidx -j .data -j .bss $< $(location imu_junction.08010000.bin) && " +
           "$(OBJCOPY) -Obinary -j .isr_vector $< $(location imu_junction.08000000.bin)"),
    toolchains = [
        "@bazel_tools//tools/cpp:current_cc_toolchain",
    ],
    output_to_bindir = True,
)

genrule(
    name = "imu_junction_flash",
    tags = ["manual"],
    srcs = ["imu_junction.08000000.bin", "imu_junction.08010000.bin"],
    outs = ["imu_junction_flash.stamp"],
    cmd = (OCD + " -c init -c \"reset_config none separate; program $(location imu_junction.08000000.bin) verify 0x8000000; program $(location imu_junction.08010000.bin) verify reset exit 0x08010000\" && touch $@"),
)

cc_test(
    name = "test",
    srcs = [
        "test/foc_test.cc",
        "test/test_main.cc",
    ],
    data = [
        "//moteus/tool:moteus_tool",
    ],
    deps = [
        ":common",
        "@boost//:test",
    ],
)

py_binary(
    name = "jump_test",
    srcs = ["jump_test.py"],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/multiplex:py_multiplex_protocol",
        "@com_github_mjbots_mjlib//mjlib/multiplex:aioserial",
    ],
)

py_binary(
    name = "bridge_test",
    srcs = ["bridge_test.py"],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/multiplex:py_multiplex_protocol",
        "@com_github_mjbots_mjlib//mjlib/multiplex:aioserial",
    ],
)

# A dummy target so that running all host tests will result in all our
# host binaries being built.
py_test(
    name = "dummy_host_test",
    srcs = [
        "test/dummy_host_test.py",
    ],
    deps = [
        "bridge_test",
        "jump_test",
        "//moteus/tool:py_moteus_tool",
    ],
)


test_suite(
    name = "host",
    tests = [
        "test",
        "dummy_host_test",
    ],
)
