# -*- python -*-

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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
        commit = "2df5a44b3ab9942cde8166fecb05945b568aa214",
        sha256 = "a09d625bd071e79ca9702277ab94a31e0f0929036fe5a0d472addb14183a1a61",
    )
