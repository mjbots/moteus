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

load("@rules_rust//crate_universe:defs.bzl", "crate")

def moteus_crate_packages():
    """The third-party crate specifications for the moteus rust targets.

    Exported (like MOTEUS_RUST_EXAMPLES in examples.bzl) so that
    downstream workspaces building the moteus rust targets from source
    -- e.g. mjbots/pi3hat -- can construct their crates_repository
    from these specs instead of hand-mirroring them.
    """
    return {
        # clap and clap_derive are pinned to exact versions: newer
        # clap_derive (4.6+) requires Cargo's edition2024 feature,
        # which is unavailable on the pinned Rust toolchain (1.82).
        "clap": crate.spec(
            version = "=4.5.54",
            features = ["derive"],
        ),
        "clap_derive": crate.spec(
            version = "=4.5.49",
        ),
        "proc-macro2": crate.spec(
            version = "1",
        ),
        "quote": crate.spec(
            version = "1",
        ),
        "syn": crate.spec(
            version = "2",
            features = ["full"],
        ),
        # default_features = False omits the libudev feature, which would
        # otherwise require the system libudev library at build time.  This
        # matches how mio-serial/tokio-serial already resolve serialport.
        "serialport": crate.spec(
            version = "4",
            default_features = False,
        ),
        "tokio": crate.spec(
            version = "1.0",
            features = ["net", "io-util", "time", "rt", "rt-multi-thread", "macros", "sync"],
        ),
        "tokio-serial": crate.spec(
            version = "5.4",
        ),
        "num_enum": crate.spec(
            version = "0.7",
            default_features = False,
        ),
    }
