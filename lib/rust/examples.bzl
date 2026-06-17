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

"""The set of reusable moteus Rust examples.

Each name corresponds to:
  * a `moteus::examples::<name>::run(register_transports)` function, and
  * a thin `//lib/rust:<name>` binary that calls it with a no-op hook.

The body of each example lives in the library (behind the `examples`
feature), so out-of-tree transports can rebuild the same example against
their own transport by providing a different registration hook.  This
list is loadable from other repositories (e.g. pi3hat) so they can
generate one such binary per example automatically.

`protocol_only` is intentionally absent: it uses no transport, so there
is nothing for an alternate transport to do.
"""

MOTEUS_RUST_EXAMPLES = [
    "bandwidth_test",
    "diagnostic_protocol",
    "discover",
    "gpio",
    "move_to",
    "multiservo",
    "simple",
]
