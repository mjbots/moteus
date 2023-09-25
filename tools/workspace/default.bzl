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

load("//tools/workspace/bazel_deps:repository.bzl", "bazel_deps_repository")
load("//tools/workspace/bazel_toolchain:repository.bzl", "bazel_toolchain_repository")
load("//tools/workspace/bazel:repository.bzl", "bazel_repository")
load("//tools/workspace/elfio:repository.bzl", "elfio_repository")
load("//tools/workspace/mjlib:repository.bzl", "mjlib_repository")
load("//tools/workspace/rules_mbed:repository.bzl", "rules_mbed_repository")

def add_default_repositories(excludes = []):
    if not native.existing_rule("com_github_mjbots_rules_mbed"):
        rules_mbed_repository()
    if not native.existing_rule("com_github_mjbots_bazel_toolchain"):
        bazel_toolchain_repository()
    if not native.existing_rule("bazel"):
        bazel_repository()
    if not native.existing_rule("com_github_mjbots_bazel_deps"):
        bazel_deps_repository(name = "com_github_mjbots_bazel_deps")
    if not native.existing_rule("elfio"):
        elfio_repository()
    if not native.existing_rule("com_github_mjbots_mjlib"):
        mjlib_repository(name = "com_github_mjbots_mjlib")
