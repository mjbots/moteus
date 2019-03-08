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

cc_library(
    name = "common",
    hdrs = [
        "atomic_event_queue.h",
        "foc.h",
        "math.h",
    ],
    srcs = [
        "foc.cc",
    ],
    deps = [
        "//mjlib/micro:static_function",
    ],
)

mbed_binary(
    name = "imu_junction",
    srcs = [
        "imu_junction_assert.cc",
        "error.h",
        "error.cc",
        "hardfault.s",
        "imu_junction_hardfault_handler.c",
        "imu_junction_hw.h",
        "irq_callback_table.h",
        "irq_callback_table.cc",
        "millisecond_timer.h",
        "stm32f446_async_uart.h",
        "stm32f446_async_uart.cc",
        "stm32_flash.h",
        "stm32_flash.cc",
        "stm32_serial.h",
        "stm32_serial.cc",
        "stream_writer.h",
        "system_info.h",
        "system_info.cc",
        "imu_junction.cc",
    ],
    deps = [
        ":common",
        "//mjlib/base:assert",
        "//mjlib/base:system_error",
        "//mjlib/micro:async_exclusive",
        "//mjlib/micro:async_stream",
        "//mjlib/micro:command_manager",
        "//mjlib/micro:multiplex_protocol",
        "//mjlib/micro:persistent_config",
        "//mjlib/micro:pool_ptr",
        "//mjlib/micro:static_function",
        "//mjlib/micro:telemetry_manager",
    ],
    linker_script = "stm32f446.ld",
)

mbed_binary(
    name = "moteus",
    srcs = [
        "as5047.h",
        "moteus_assert.cc",
        "bldc_servo.h",
        "bldc_servo.cc",
        "board_debug.h",
        "board_debug.cc",
        "drv8323.h",
        "drv8323.cc",
        "error.h",
        "error.cc",
        "hardfault.s",
        "moteus_hardfault_handler.c",
        "moteus_hw.h",
        "irq_callback_table.h",
        "irq_callback_table.cc",
        "millisecond_timer.h",
        "motor_driver.h",
        "position_sensor.h",
        "stm32f446_async_uart.h",
        "stm32f446_async_uart.cc",
        "stm32_flash.h",
        "stm32_flash.cc",
        "stm32_serial.h",
        "stm32_serial.cc",
        "system_info.h",
        "system_info.cc",
        "moteus.cc",
    ],
    deps = [
        ":common",
        "//mjlib/base:assert",
        "//mjlib/base:pid",
        "//mjlib/base:system_error",
        "//mjlib/base:windowed_average",
        "//mjlib/micro:async_exclusive",
        "//mjlib/micro:async_stream",
        "//mjlib/micro:command_manager",
        "//mjlib/micro:multiplex_protocol",
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

genrule(
    name = "imu_junction_bin",
    srcs = ["imu_junction.elf"],
    outs = [
        "imu_junction.08000000.bin",
        "imu_junction.08010000.bin",
    ],
    cmd = ("$(OBJCOPY) -Obinary -j .text -j .ARM.extab -j .ARM.exidx -j .data -j .bss $< $(location imu_junction.08010000.bin) && " +
           "$(OBJCOPY) -Obinary -j .isr_vector $< $(location imu_junction.08000000.bin)"),
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
        "test/atomic_event_queue_test.cc",
        "test/foc_test.cc",
        "test/test_main.cc",
    ],
    deps = [
        ":common",
        "@boost//:test",
    ],
)

py_library(
    name = "calibrate_encoder",
    srcs = ["calibrate_encoder.py"],
)

py_binary(
    name = "jump_test",
    srcs = ["jump_test.py"],
    deps = [
        "//mjlib/micro:py_multiplex_protocol",
        "//mjlib/micro:aioserial",
    ],
)

py_binary(
    name = "moteus_tool",
    srcs = ["moteus_tool.py"],
    deps = [
        ":calibrate_encoder",
        "//mjlib/micro:py_multiplex_protocol",
        "//mjlib/micro:aioserial",
    ],
)

py_binary(
    name = "bridge_test",
    srcs = ["bridge_test.py"],
    deps = [
        "//mjlib/micro:py_multiplex_protocol",
        "//mjlib/micro:aioserial",
    ],
)
