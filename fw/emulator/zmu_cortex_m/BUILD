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

load("@rules_rust//cargo:defs.bzl", "cargo_build_script")
load("@rules_rust//rust:defs.bzl", "rust_library")

package(default_visibility = ["//fw/emulator:__subpackages__"])

# Cortex-M4F: ARMv7E-M with the single precision FPv4 extension.
FEATURES = [
    "armv7em",
    "has-dsp-ext",
    "has-fp",
    "vfp-register-bank-d16",
    "fpv4-sp-d16",
]

# This generates the instruction decoders.
cargo_build_script(
    name = "build_script",
    srcs = ["build.rs"],
    crate_features = FEATURES,
    edition = "2024",
)

rust_library(
    name = "zmu_cortex_m",
    srcs = glob(["src/**/*.rs"]),
    crate_features = FEATURES,
    edition = "2024",
    deps = [":build_script"],
)
