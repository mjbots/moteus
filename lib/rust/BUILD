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

load("@rules_rust//rust:defs.bzl", "rust_binary")

test_suite(
    name = "host",
    tests = [
        "//lib/rust/moteus-protocol:moteus-protocol-test",
        "//lib/rust/moteus:moteus-test",
    ],
)

# Example: Simple single-servo control
rust_binary(
    name = "simple",
    srcs = ["examples/simple.rs"],
    edition = "2021",
    deps = [
        "//lib/rust/moteus:moteus_clap_tokio",
        "@crate_index//:clap",
        "@crate_index//:tokio",
    ],
)

# Example: Protocol-only usage (no transport)
rust_binary(
    name = "protocol_only",
    srcs = ["examples/protocol_only.rs"],
    edition = "2021",
    deps = ["//lib/rust/moteus-protocol"],
)

# Example: Multi-servo control with discovery
rust_binary(
    name = "multiservo",
    srcs = ["examples/multiservo.rs"],
    edition = "2021",
    deps = [
        "//lib/rust/moteus:moteus_clap_tokio",
        "@crate_index//:clap",
        "@crate_index//:tokio",
    ],
)

# Example: Discover all connected controllers
rust_binary(
    name = "discover",
    srcs = ["examples/discover.rs"],
    edition = "2021",
    deps = [
        "//lib/rust/moteus:moteus_clap",
        "@crate_index//:clap",
    ],
)

# Example: Bandwidth test for position/query loop
rust_binary(
    name = "bandwidth_test",
    srcs = ["examples/bandwidth_test.rs"],
    edition = "2021",
    deps = [
        "//lib/rust/moteus:moteus_clap_tokio",
        "@crate_index//:clap",
        "@crate_index//:tokio",
    ],
)

# Example: Diagnostic protocol for configuration and debugging
rust_binary(
    name = "diagnostic_protocol",
    srcs = ["examples/diagnostic_protocol.rs"],
    edition = "2021",
    deps = [
        "//lib/rust/moteus:moteus_clap_tokio",
        "@crate_index//:clap",
        "@crate_index//:tokio",
    ],
)

# Example: Coordinated multi-servo move_to
rust_binary(
    name = "move_to",
    srcs = ["examples/move_to.rs"],
    edition = "2021",
    deps = [
        "//lib/rust/moteus:moteus_clap_tokio",
        "@crate_index//:clap",
        "@crate_index//:tokio",
    ],
)

# Example: GPIO reading and writing
rust_binary(
    name = "gpio",
    srcs = ["examples/gpio.rs"],
    edition = "2021",
    deps = [
        "//lib/rust/moteus:moteus_clap_tokio",
        "@crate_index//:clap",
        "@crate_index//:tokio",
    ],
)
