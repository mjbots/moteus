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

def rules_mbed_repository():
    github_archive(
        name = "com_github_mjbots_rules_mbed",
        repo = "mjbots/rules_mbed",
        commit = "a05091e0f4abd7aa75271f582306abef506f9028",
        sha256 = "43e26e90c46517b2db91fdfb353d89963697fd682c75861a9bd54d0d2dc84568",
    )
