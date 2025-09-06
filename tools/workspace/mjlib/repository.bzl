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

def mjlib_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/mjlib",
        commit = "48bcba0e57fa673ac97191bdc60169178d12eca5",
        sha256 = "f1648de4d6669721e6c7d83878f7f1bb740f26995de8ad88153f78b70db15a08",
    )
