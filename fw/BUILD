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
load("@com_github_ARMmbed_mbed-g4//:rules.bzl", g4_mbed_binary = "mbed_binary")
load("@com_github_ARMmbed_mbed-g4-bootloader//:rules.bzl", g4_bootloader_mbed_binary = "mbed_binary")

COPTS = [
    "-Werror",
    "-Wdouble-promotion",
    "-gdwarf-4",
]

cc_library(
    name = "common",
    hdrs = [
        "bldc_servo_position.h",
        "bldc_servo_structs.h",
        "aux_common.h",
        "ccm.h",
        "error.h",
        "foc.h",
        "math.h",
        "measured_hw_rev.h",
        "motor_position.h",
        "pid.h",
        "simple_pi.h",
        "torque_model.h",
        "stm32_i2c_timing.h",
    ],
    srcs = [
        "foc.cc",
    ],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/base:limit",
        "@com_github_mjbots_mjlib//mjlib/base:visitor",
        "@com_github_mjbots_mjlib//mjlib/micro:atomic_event_queue",
        "@com_github_mjbots_mjlib//mjlib/micro:error_code",
        "@com_github_mjbots_mjlib//mjlib/micro:persistent_config",
        "@com_github_mjbots_mjlib//mjlib/micro:telemetry_manager",
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
    "aksim2.h",
    "as5047.h",
    "aux_adc.h",
    "aux_mbed.h",
    "aux_port.h",
    "moteus_assert.cc",
    "bldc_servo.h",
    "bldc_servo.cc",
    "board_debug.h",
    "board_debug.cc",
    "bootloader.h",
    "clock_manager.h",
    "cui_amt21.h",
    "drv8323.h",
    "drv8323.cc",
    "error.cc",
    "hardfault.s",
    "firmware_info.h",
    "firmware_info.cc",
    "ic_pz.h",
    "ma732.h",
    "moteus_controller.h",
    "moteus_controller.cc",
    "moteus_hardfault_handler.c",
    "moteus_hw.h",
    "moteus_hw.cc",
    "millisecond_timer.h",
    "motor_driver.h",
    "stm32_dma.h",
    "stm32g4_adc.h",
    "stm32g4_adc.cc",
    "stm32g4_dma_uart.h",
    "stm32_bitbang_spi.h",
    "stm32_gpio_interrupt_in.h",
    "stm32_gpio_interrupt_in.cc",
    "stm32_i2c.h",
    "stm32_spi.h",
    "stm32_serial.h",
    "stm32_serial.cc",
    "stm32.h",
    "system_info.h",
    "system_info.cc",
    "uuid.h",
    "uuid.cc",
    "moteus.cc",
]

MOTEUS_DEPS = [
    ":common",
    ":git_info",
    "@com_github_mjbots_mjlib//mjlib/base:assert",
    "@com_github_mjbots_mjlib//mjlib/base:inplace_function",
    "@com_github_mjbots_mjlib//mjlib/base:windowed_average",
    "@com_github_mjbots_mjlib//mjlib/micro:async_exclusive",
    "@com_github_mjbots_mjlib//mjlib/micro:async_stream",
    "@com_github_mjbots_mjlib//mjlib/micro:callback_table",
    "@com_github_mjbots_mjlib//mjlib/micro:command_manager",
    "@com_github_mjbots_mjlib//mjlib/micro:persistent_config",
    "@com_github_mjbots_mjlib//mjlib/micro:pool_ptr",
    "@com_github_mjbots_mjlib//mjlib/micro:telemetry_manager",
    "@com_github_mjbots_mjlib//mjlib/multiplex:micro_server",
    "@com_github_mjbots_mjlib//mjlib/multiplex:micro_stream_datagram",
]

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

OCD_G4 = (
    "openocd " +
    "-f interface/stlink.cfg " +
    "-f target/stm32g4x.cfg "
)

genrule(
    name = "flash",
    tags = ["manual"],
    srcs = ["moteus.08000000.bin", "moteus.0800c000.bin", "moteus.08010000.bin"],
    outs = ["flash.stamp"],
    cmd = (OCD_G4 + " -c init -c \"reset_config none separate; program $(location moteus.08000000.bin) verify 0x8000000; program $(location moteus.0800c000.bin) verify 0x800c000; program $(location moteus.08010000.bin) verify reset exit 0x08010000\" && touch $@"),
)

cc_test(
    name = "test",
    srcs = [
        "test/bldc_servo_position_test.cc",
        "test/foc_test.cc",
        "test/math_test.cc",
        "test/motor_position_test.cc",
        "test/stm32_i2c_timing_test.cc",
        "test/torque_model_test.cc",
        "test/test_main.cc",
    ],
    data = [
        ":multiplex_tool",
    ],
    deps = [
        ":common",
        "@boost//:test",
        "@fmt",
        "@com_github_mjbots_mjlib//mjlib/micro:test_fixtures",
    ],
    size = "small",
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
    ],
    size = "small",
)


test_suite(
    name = "host",
    tests = [
        "test",
        "dummy_host_test",
    ],
)
