# -*- python -*-

# Copyright 2018 Josh Pieper, jjp@pobox.com.
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

def _dictify(data):
    result = dict([x.split('=', 1) for x in data])
    return result

def _template_file_impl(ctx):
    out = ctx.actions.declare_file(ctx.label.name)

    substitutions = dict(ctx.attr.substitutions)
    substitutions.update(_dictify(ctx.attr.substitution_list))

    ctx.actions.expand_template(
        template = ctx.files.src[0],
        output = out,
        substitutions = substitutions,
        is_executable = ctx.attr.is_executable)
    return [DefaultInfo(
        files = depset([out]),
        data_runfiles = ctx.runfiles(files = [out]),
    )]


template_file = rule(
    attrs = {
        "src": attr.label(allow_files = True),
        "is_executable": attr.bool(default = False),
        "substitutions": attr.string_dict(),
        "substitution_list": attr.string_list(),
    },
    output_to_genfiles = True,
    implementation = _template_file_impl,
)
