#!/bin/bash

# Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

set -euo pipefail

# Type-check the moteus python package.
#
# Everything mypy needs -- the source modules, the bazel-generated
# version.py, and py.typed -- is staged in this test's runfiles as a
# data dependency, so we run mypy against that runfiles copy.  We never
# touch the source tree: an earlier version of this script reached into
# the working copy (via the execroot's 'lib' symlink) to drop a stub
# version.py, and an interrupted run would leave that stub behind.

if [[ -z "${TEST_SRCDIR:-}" ]]; then
    echo "ERROR: TEST_SRCDIR is not set; run this via 'bazel test'." >&2
    exit 1
fi

MOTEUS_DIR="$TEST_SRCDIR/${TEST_WORKSPACE:-com_github_mjbots_moteus}/lib/python/moteus"

if [[ ! -f "$MOTEUS_DIR/version.py" ]]; then
    echo "ERROR: generated version.py not found in runfiles:" >&2
    echo "  $MOTEUS_DIR/version.py" >&2
    ls -la "$MOTEUS_DIR" >&2 2>/dev/null || true
    exit 1
fi

echo "Running: mypy $MOTEUS_DIR --ignore-missing-imports"
# Keep the cache inside the test's private tmp dir so nothing is written
# next to the (symlinked) sources.
exec mypy "$MOTEUS_DIR" \
    --ignore-missing-imports \
    --cache-dir "${TEST_TMPDIR:-/tmp}/mypy_cache"
