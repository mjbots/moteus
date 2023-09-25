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

def rules_mbed_repository():
    github_archive(
        name = "com_github_mjbots_rules_mbed",
        repo = "mjbots/rules_mbed",
        commit = "6a276d2ad4e2fe8a81438af4854d8d09d31b980b",
        sha256 = "ef880dc1e8069aed2413b2761274e2b462cedb184d7740798a5b7a5adacc7217",
    )
