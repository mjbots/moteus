# -*- python -*-

# Copyright 2018-2020 Josh Pieper, jjp@pobox.com.
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

COPTS = [
    "-Werror",
    "-Wdouble-promotion",
]

cc_library(
    name = "common",
    hdrs = [
        "foc.h",
        "math.h",
        "torque_model.h",
    ],
    srcs = [
        "foc.cc",
    ],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/micro:atomic_event_queue",
    ],
    copts = COPTS,
)

cc_library(
    name = "git_info",
    hdrs = ["git_info.h"],
    srcs = ["git_info.cc"],
    linkstamp = "git_info_linkstamp.cc",
    deps = [
        "@com_github_mjbots_mjlib//mjlib/base:visitor",
    ],
    copts = COPTS,
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
    copts = COPTS,
)

g4_bootloader_mbed_binary(
    name = "power_dist",
    srcs = [
        "fdcan.cc",
        "fdcan.h",
        "millisecond_timer.h",
        "power_dist.cc",
        "power_dist_hw.h",
    ],
    deps = [
        ":git_info",
        "@com_github_mjbots_mjlib//mjlib/base:string_span",
    ],
    copts = COPTS,
)

bootloader_mbed_binary(
    name = "uart_bootloader",
    srcs = [
        "bootloader.h",
        "uart_bootloader.cc",
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
    copts = COPTS,
)

g4_bootloader_mbed_binary(
    name = "can_bootloader",
    srcs = [
        "bootloader.h",
        "can_bootloader.cc",
        "stm32g4xx_fdcan_typedefs.h",
    ],
    deps = [
        ":git_info",
        "@com_github_mjbots_mjlib//mjlib/base:buffer_stream",
        "@com_github_mjbots_mjlib//mjlib/base:tokenizer",
        "@com_github_mjbots_mjlib//mjlib/multiplex:format",
        "@com_github_mjbots_mjlib//mjlib/multiplex:stream",
        "@boost",
    ],
    linker_script = "stm32g474_bootloader.ld",
    enable_wrappers = False,
    copts = COPTS,
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
    "moteus_controller.h",
    "moteus_controller.cc",
    "moteus_hardfault_handler.c",
    "moteus_hw.h",
    "millisecond_timer.h",
    "motor_driver.h",
    "pid.h",
    "stm32_spi.h",
    "stm32_serial.h",
    "stm32_serial.cc",
    "stm32.h",
    "system_info.h",
    "system_info.cc",
    "moteus.cc",
]

MOTEUS_DEPS = [
    ":common",
    ":git_info",
    "@com_github_mjbots_mjlib//mjlib/base:assert",
    "@com_github_mjbots_mjlib//mjlib/base:inplace_function",
    "@com_github_mjbots_mjlib//mjlib/base:limit",
    "@com_github_mjbots_mjlib//mjlib/base:windowed_average",
    "@com_github_mjbots_mjlib//mjlib/micro:async_exclusive",
    "@com_github_mjbots_mjlib//mjlib/micro:async_stream",
    "@com_github_mjbots_mjlib//mjlib/micro:callback_table",
    "@com_github_mjbots_mjlib//mjlib/micro:command_manager",
    "@com_github_mjbots_mjlib//mjlib/micro:error_code",
    "@com_github_mjbots_mjlib//mjlib/micro:persistent_config",
    "@com_github_mjbots_mjlib//mjlib/micro:pool_ptr",
    "@com_github_mjbots_mjlib//mjlib/micro:telemetry_manager",
    "@com_github_mjbots_mjlib//mjlib/multiplex:micro_server",
    "@com_github_mjbots_mjlib//mjlib/multiplex:micro_stream_datagram",
]

mbed_binary(
    name = "f4_moteus",
    srcs = MOTEUS_SOURCES + [
        "stm32f446_async_uart.h",
        "stm32f446_async_uart.cc",
        "stm32f4_flash.h",
        "stm32f4_flash.cc",
    ],
    deps = MOTEUS_DEPS,
    linker_script = "stm32f446.ld",
    copts = COPTS,
    features = ["nomaybe_uninitialized"],
)

g4_mbed_binary(
    name = "moteus",
    srcs = MOTEUS_SOURCES + [
        "stm32g4_async_uart.h",
        "stm32g4_async_uart.cc",
        "stm32g4_flash.h",
        "fdcan.h",
        "fdcan.cc",
        "fdcan_micro_server.h",
    ],
    deps = MOTEUS_DEPS,
    linker_script = "stm32g474.ld",
    features = ["speedopt"],
    copts = COPTS,
)

genrule(
    name = "bin",
    srcs = ["moteus.elf", "can_bootloader.elf"],
    outs = [
        "moteus.08000000.bin",
        "moteus.0800c000.bin",
        "moteus.08010000.bin",
    ],
    cmd = ("$(OBJCOPY) -Obinary -j .text -j .ARM.extab -j .ARM.exidx -j .data -j .ccmram -j .bss $(location moteus.elf) $(location moteus.08010000.bin) && " +
           "$(OBJCOPY) -Obinary -j .text -j .ARM.extab -j .ARM.exidx -j .data -j .bss $(location can_bootloader.elf) $(location moteus.0800c000.bin) && " +
           "$(OBJCOPY) -Obinary -j .isr_vector $(location moteus.elf) $(location moteus.08000000.bin)"),
    toolchains = [
        "@bazel_tools//tools/cpp:current_cc_toolchain",
    ],
    output_to_bindir = True,
)

genrule(
    name = "f4_bin",
    srcs = ["f4_moteus.elf", "uart_bootloader.elf"],
    outs = [
        "f4_moteus.08000000.bin",
        "f4_moteus.0800c000.bin",
        "f4_moteus.08010000.bin",
    ],
    cmd = ("$(OBJCOPY) -Obinary -j .text -j .ARM.extab -j .ARM.exidx -j .data -j .bss $(location f4_moteus.elf) $(location f4_moteus.08010000.bin) && " +
           "$(OBJCOPY) -Obinary -j .text -j .ARM.extab -j .ARM.exidx -j .data -j .bss $(location uart_bootloader.elf) $(location f4_moteus.0800c000.bin) && " +
           "$(OBJCOPY) -Obinary -j .isr_vector $(location f4_moteus.elf) $(location f4_moteus.08000000.bin)"),
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
    name = "f4_flash",
    tags = ["manual"],
    srcs = ["f4_moteus.08000000.bin", "f4_moteus.08010000.bin",
            "f4_moteus.0800c000.bin",
    ],
    outs = ["f4_flash.stamp"],
    cmd = (OCD + " -c init -c \"reset_config none separate; program $(location f4_moteus.08000000.bin) verify 0x8000000; program $(location f4_moteus.0800c000.bin) verify 0x800c000; program $(location f4_moteus.08010000.bin) verify reset exit 0x08010000\" && touch $@"),
)

genrule(
    name = "flash",
    tags = ["manual"],
    srcs = ["moteus.08000000.bin", "moteus.0800c000.bin", "moteus.08010000.bin"],
    outs = ["flash.stamp"],
    cmd = (OCD_G4 + " -c init -c \"reset_config none separate; program $(location moteus.08000000.bin) verify 0x8000000; program $(location moteus.0800c000.bin) verify 0x800c000; program $(location moteus.08010000.bin) verify reset exit 0x08010000\" && touch $@"),
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
        "test/math_test.cc",
        "test/torque_model_test.cc",
        "test/test_main.cc",
    ],
    data = [
        "//moteus/tool:moteus_tool",
        ":multiplex_tool",
        "//moteus/tool:tview",
    ],
    deps = [
        ":common",
        "@boost//:test",
        "@fmt",
    ],
    size = "small",
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
    name = "throw_test",
    srcs = ["throw_test.py"],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/multiplex:py_multiplex_protocol",
        "@com_github_mjbots_mjlib//mjlib/multiplex:aioserial",
    ],
)

py_binary(
    name = "fdcanusb_register_example",
    srcs = ["fdcanusb_register_example.py"],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/multiplex:py_multiplex_protocol",
        "@com_github_mjbots_mjlib//mjlib/multiplex:aioserial",
    ],
)

cc_binary(
    name = "multiplex_tool",
    srcs = ["multiplex_tool_main.cc"],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/multiplex:libmultiplex_tool",
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
        "fdcanusb_register_example",
        "jump_test",
        "throw_test",
        "//moteus/tool:calibrate_encoder",
        "//moteus/tool:tview",
    ],
    size = "small",
)


test_suite(
    name = "host",
    tests = [
        "test",
        "dummy_host_test",
        "//moteus/tool:test",
    ],
)
