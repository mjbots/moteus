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

def _cargo_lock_copy_impl(repository_ctx):
    repository_ctx.file("BUILD", 'exports_files(["Cargo.lock"])\n')
    repository_ctx.file(
        "Cargo.lock",
        repository_ctx.read(repository_ctx.attr.src),
    )

cargo_lock_copy = repository_rule(
    implementation = _cargo_lock_copy_impl,
    attrs = {
        "src": attr.label(
            allow_single_file = True,
            mandatory = True,
        ),
    },
    doc = """A copy of a Cargo.lock, for crates_repository to use.

    When repinning, crates_repository writes its cargo_lockfile back
    with what it resolved.  That is not the lockfile of the cargo
    workspace: it lists a placeholder crate in place of the workspace
    members, and is in whatever format the bazel toolchain's cargo
    uses, which older versions of cargo cannot parse.  Given this copy
    instead, a repin starts from the versions cargo has locked, and
    the file cargo uses is left alone.
    """,
)
