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

def mjlib_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/mjlib",
        commit = "3e4b6c7c60be0b57732fb8603da62bca8843e711",
        sha256 = "faf463e0777df217ef9a900bcf6709a668db2a6202d4cb1213933847ff7b9a59",
    )
