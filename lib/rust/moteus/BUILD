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

load("@rules_rust//rust:defs.bzl", "rust_library", "rust_test")

package(default_visibility = ["//visibility:public"])

rust_library(
    name = "moteus",
    srcs = glob(["src/**/*.rs"]),
    crate_name = "moteus",
    edition = "2021",
    rustc_flags = ["-D", "warnings"],
    proc_macro_deps = [
        "//lib/rust/moteus-derive",
    ],
    deps = [
        "//lib/rust/moteus-protocol",
    ],
)

# Library with clap support for CLI applications
rust_library(
    name = "moteus_clap",
    srcs = glob(["src/**/*.rs"]),
    crate_name = "moteus",
    edition = "2021",
    rustc_flags = [
        "--cfg=feature=\"clap\"",
        "-D", "warnings",
    ],
    proc_macro_deps = [
        "//lib/rust/moteus-derive",
    ],
    deps = [
        "//lib/rust/moteus-protocol",
        "@crate_index//:clap",
    ],
)

rust_test(
    name = "moteus-test",
    crate = ":moteus",
    edition = "2021",
    size = "small",
)
