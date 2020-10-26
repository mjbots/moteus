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

def rules_wix_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/rules_wix",
        commit = "8678d0e2c714d5dc0ab92b3acbf667771694f9fa",
        sha256 = "d9679e7b879371764fd1d4b9b634c7c4b62c755de7477998b237396b245dd2ab",
    )
