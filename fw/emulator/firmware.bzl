# -*- python -*-

# Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

def _stm32g4_transition_impl(settings, attr):
    return {
        "//command_line_option:platforms": "@com_github_mjbots_rules_mbed//:stm32g4",
        # Flags intended for host builds must not leak into the
        # firmware build.
        "//command_line_option:copt": [],
        "//command_line_option:linkopt": [],
    }

_stm32g4_transition = transition(
    implementation = _stm32g4_transition_impl,
    inputs = [],
    outputs = [
        "//command_line_option:platforms",
        "//command_line_option:copt",
        "//command_line_option:linkopt",
    ],
)

def _stm32g4_firmware_impl(ctx):
    files = depset(transitive = [
        src[DefaultInfo].files
        for src in ctx.attr.srcs
    ])
    return [DefaultInfo(
        files = files,
        runfiles = ctx.runfiles(transitive_files = files),
    )]

stm32g4_firmware = rule(
    implementation = _stm32g4_firmware_impl,
    doc = """Builds 'srcs' for the stm32g4 regardless of the current
    platform, so that host tests can depend upon firmware images.""",
    attrs = {
        "srcs": attr.label_list(
            cfg = _stm32g4_transition,
            allow_files = True,
        ),
        "_allowlist_function_transition": attr.label(
            default = "@bazel_tools//tools/allowlists/function_transition_allowlist",
        ),
    },
)
