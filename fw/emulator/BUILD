# -*- python -*-

# Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

load("@rules_rust//rust:defs.bzl", "rust_binary", "rust_library", "rust_test")
load(":firmware.bzl", "stm32g4_firmware")

package(default_visibility = ["//visibility:public"])

# A C interface to the motor model the firmware simulation tests use.
cc_library(
    name = "motor_sim",
    srcs = ["motor_sim.cc"],
    deps = ["//fw:motor_simulator"],
)

rust_library(
    name = "moteus_emulator",
    srcs = glob(["src/**/*.rs"], exclude = ["src/main.rs"]),
    edition = "2021",
    rustc_flags = ["-D", "warnings"],
    deps = [
        ":motor_sim",
        "//fw/emulator/zmu_cortex_m",
        "//lib/rust/moteus",
    ],
)

rust_test(
    name = "moteus_emulator_test",
    crate = ":moteus_emulator",
    edition = "2021",
    size = "small",
)

rust_binary(
    name = "emulator",
    srcs = ["src/main.rs"],
    edition = "2021",
    rustc_flags = ["-D", "warnings"],
    deps = [
        ":moteus_emulator",
        "//fw/emulator/zmu_cortex_m",
    ],
)

# The firmware image, built for the target no matter what platform
# the rest of the build is for.
stm32g4_firmware(
    name = "firmware",
    srcs = ["//fw:moteus.elf"],
)

rust_test(
    name = "boot_test",
    srcs = ["tests/boot_test.rs"],
    edition = "2021",
    rustc_flags = ["-D", "warnings"],
    data = [":firmware"],
    env = {"MOTEUS_ELF": "$(rootpath :firmware)"},
    deps = [":moteus_emulator"],
    size = "small",
)

BOARDS = [
    "r4",
    "n1",
    "c1",
    "x1",
    "r5",
]

[rust_test(
    name = "modes_test_" + board,
    srcs = ["tests/modes_test.rs"],
    crate_name = "modes_test",
    edition = "2021",
    rustc_flags = ["-D", "warnings"],
    data = [":firmware"],
    env = {
        "MOTEUS_BOARD": board,
        "MOTEUS_ELF": "$(rootpath :firmware)",
    },
    deps = [
        ":moteus_emulator",
        "//lib/rust/moteus",
    ],
    # These take around 30s, but several times that on a machine
    # which is also busy building.  Remain "small" so that they are
    # run by default.
    size = "small",
    timeout = "moderate",
) for board in BOARDS]

rust_test(
    name = "cpu_test",
    srcs = ["tests/cpu_test.rs"],
    edition = "2021",
    rustc_flags = ["-D", "warnings"],
    data = [
        "tests/flow_vectors.txt",
        "tests/fp_vectors.txt",
        "tests/int_vectors.txt",
    ],
    env = {
        "FLOW_VECTORS": "$(rootpath tests/flow_vectors.txt)",
        "FP_VECTORS": "$(rootpath tests/fp_vectors.txt)",
        "INT_VECTORS": "$(rootpath tests/int_vectors.txt)",
    },
    deps = [
        ":moteus_emulator",
        "//fw/emulator/zmu_cortex_m",
    ],
    size = "small",
)

rust_test(
    name = "calibration_test",
    srcs = ["tests/calibration_test.rs"],
    edition = "2021",
    rustc_flags = ["-D", "warnings"],
    data = [
        "calibration/benchmarks.txt",
        "calibration/hardware_cycles_x1.txt",
    ],
    env = {
        "BENCHMARKS": "$(rootpath calibration/benchmarks.txt)",
        "HARDWARE_CYCLES": "$(rootpath calibration/hardware_cycles_x1.txt)",
    },
    deps = [
        ":moteus_emulator",
        "//fw/emulator/zmu_cortex_m",
    ],
    size = "small",
)

test_suite(
    name = "host",
    tests = [
        ":boot_test",
        ":calibration_test",
        ":cpu_test",
    ] + [
        ":modes_test_" + board
        for board in BOARDS
    ] + [
        ":moteus_emulator_test",
    ],
)
