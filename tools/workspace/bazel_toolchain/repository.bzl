# -*- python -*-

# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

load("//tools/workspace:github_archive.bzl", "github_archive")

def bazel_toolchain_repository():
    github_archive(
        name = "com_github_mjbots_bazel_toolchain",
        repo = "mjbots/bazel-toolchain",
        commit = "47efd434976067fec4689a3e5cca9859b82b08ec",
        sha256 = "86428e992ad64e74bca6e388e4dc2846dee8fd8060b1b5cd8dfdc33631ef9103",
    )
