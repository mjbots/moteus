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

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def rules_rust_repository(name = "rules_rust"):
    """Add rules_rust to the workspace."""

    if native.existing_rule(name):
        return

    http_archive(
        name = name,
        sha256 = "c8aa806cf6066679ac23463241ee80ad692265dad0465f51111cbbe30b890352",
        urls = ["https://github.com/bazelbuild/rules_rust/releases/download/0.68.1/rules_rust-0.68.1.tar.gz"],
    )
