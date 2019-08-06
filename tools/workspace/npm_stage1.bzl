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

load("@build_bazel_rules_nodejs//:defs.bzl", "node_repositories", "yarn_install")

# Because bazel's starlark as of 0.28.1 doesn't allow a WORKSPACE
# evaluated function to have load calls anywhere but at the top, we
# have to split our npm initialization into multiple stages. :( Anyone
# who calls us will just have to invoke all of them in order to get
# the proper initialization.
def setup_npm_stage1():
    node_repositories(
        node_version = "10.16.0",
        yarn_version = "1.13.0",
    )

    yarn_install(
        name = "npm",
        package_json = "//:package.json",
        yarn_lock = "//:yarn.lock",
    )
