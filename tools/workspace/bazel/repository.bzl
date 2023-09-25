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

def bazel_repository():
    github_archive(
        name = "bazel_skylib",
        repo = "bazelbuild/bazel-skylib",
        commit = "d2cf1cc2bcd1e879743faf5216c4887b994705af",
        sha256 = "4f5657797eb215b4281a05eee830b22289d2ef7571ad8ace6c8da4db76f47b7e",
    )
