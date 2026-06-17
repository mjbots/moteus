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

# Rust client libraries for moteus
# See moteus-protocol/ for the no_std protocol crate
# See moteus/ for the full client library with transports

load("@rules_rust//rust:defs.bzl", "rust_binary", "rustfmt_test")
load(":examples.bzl", "MOTEUS_RUST_EXAMPLES")

test_suite(
    name = "host",
    tests = [
        "//lib/rust/moteus-protocol:moteus-protocol-test",
        "//lib/rust/moteus-protocol:moteus-protocol-doc-test",
        "//lib/rust/moteus:moteus-test",
        "//lib/rust/moteus:moteus-doc-test",
        "//lib/rust/moteus:moteus-doc-test-all-features",
        "//lib/rust:clippy",
        "//lib/rust:examples",
        "//lib/rust:rustdoc",
        "//lib/rust:fmt",
    ],
)

# Clippy lint checks for all Rust crates.
# Wrapped in sh_test so it runs as part of the test suite.
sh_test(
    name = "clippy",
    srcs = ["clippy_test.sh"],
    data = [
        "//lib/rust/moteus-derive:moteus-derive-clippy",
        "//lib/rust/moteus-protocol:moteus-protocol-clippy",
        "//lib/rust/moteus:moteus-clippy",
    ],
    size = "small",
)

# Formatting check using the bazel Rust toolchain's rustfmt.
rustfmt_test(
    name = "fmt",
    targets = [
        "//lib/rust/moteus-derive",
        "//lib/rust/moteus-protocol",
        "//lib/rust/moteus:moteus",
    ],
    size = "small",
)

# Rustdoc lint checks (warnings as errors).
sh_test(
    name = "rustdoc",
    srcs = ["clippy_test.sh"],
    data = [
        "//lib/rust/moteus-protocol:moteus-protocol-doc",
        "//lib/rust/moteus:moteus-doc",
    ],
    size = "small",
)

# Ensure all example binaries build.
# Wrapped in sh_test so they build as part of the test suite.
sh_test(
    name = "examples",
    srcs = ["clippy_test.sh"],
    data = [":" + name for name in MOTEUS_RUST_EXAMPLES] + [
        ":protocol_only",
    ],
    size = "small",
)

# The transport-using examples are thin wrappers over
# moteus::examples::<name>::run; one rust_binary per entry in the shared
# MOTEUS_RUST_EXAMPLES list.
[
    rust_binary(
        name = name,
        srcs = ["//lib/rust/moteus:examples/" + name + ".rs"],
        edition = "2021",
        deps = ["//lib/rust/moteus:moteus_examples"],
    )
    for name in MOTEUS_RUST_EXAMPLES
]

# Protocol-only usage (no transport); not part of the reusable set.
rust_binary(
    name = "protocol_only",
    srcs = ["//lib/rust/moteus:examples/protocol_only.rs"],
    edition = "2021",
    deps = ["//lib/rust/moteus-protocol"],
)
