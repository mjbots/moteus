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

load("@rules_rust//rust:defs.bzl", "rust_clippy", "rust_doc", "rust_library", "rust_test")

package(default_visibility = ["//visibility:public"])

rust_library(
    name = "moteus-protocol",
    srcs = glob(["src/**/*.rs"]),
    crate_name = "moteus_protocol",
    edition = "2021",
    crate_features = ["std"],
    rustc_flags = ["-D", "warnings"],
    deps = [
        "@crate_index//:num_enum",
    ],
    proc_macro_deps = [
        "//lib/rust/moteus-derive",
    ],
)

rust_library(
    name = "moteus-protocol-nostd",
    srcs = glob(["src/**/*.rs"]),
    crate_name = "moteus_protocol",
    edition = "2021",
    crate_features = [],
    rustc_flags = ["-D", "warnings"],
    deps = [
        "@crate_index//:num_enum",
    ],
    proc_macro_deps = [
        "//lib/rust/moteus-derive",
    ],
)

rust_test(
    name = "moteus-protocol-test",
    crate = ":moteus-protocol",
    edition = "2021",
    size = "small",
)

rust_clippy(
    name = "moteus-protocol-clippy",
    deps = [":moteus-protocol"],
)

rust_doc(
    name = "moteus-protocol-doc",
    crate = ":moteus-protocol",
)
