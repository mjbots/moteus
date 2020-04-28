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

def bazel_deps_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/bazel_deps",
        commit = "1c14421dedf8af570ec19edc503413d68201ea9a",
        sha256 = "8b2f5fec8214a8c54f47aff472669a547e6e604de854f017db3b797e1e014fe5",
    )
