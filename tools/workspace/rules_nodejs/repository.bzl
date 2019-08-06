# -*- python -*-

# Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

def rules_nodejs_repository():
    github_archive(
        name = "build_bazel_rules_nodejs",
        repo = "bazelbuild/rules_nodejs",
        commit = "7df4228602cf4216515c052532206dc5461e372e",
        sha256 = "d598a5e1b2f3ade81535ab0ac966dcc42f48ccccfddf67a81d59160f1cc6f09b",
        patches = ["@com_github_mjbots_moteus//tools/workspace/rules_nodejs:rules_nodejs.patch"],
        patch_args = ["-p1"],
    )
