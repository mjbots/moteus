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


def elfio_repository():
    github_archive(
        name = "com_github_serge1_elfio",
        repo = "serge1/ELFIO",
        commit = "580da2467b3d7da4c817d45a99a367e4b0d6d326",
        sha256 = "53cfc3b703b785c0b1c071e7beeeb7cf963f2f15984a54d6ff247112e5ee88a3",
        build_file = Label("//tools/workspace/elfio:package.BUILD"),
    )
