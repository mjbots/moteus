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

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def github_archive(name, repo, commit, local_override=None,
                   sha256=None, **kwargs):
    """Like 'http_archive', but for github repositories.

    If 'local_override' is set, then reference a local repository at
    the given path instead of github.com.
    """

    if local_override:
        native.local_repository(
            name = name,
            path = local_override,
        )
    else:
        http_archive(
            name = name,
            url = "https://github.com/{repo}/archive/{commit}.zip".format(
                repo=repo, commit=commit),
            strip_prefix = "{}-{}".format(repo.rsplit('/', 1)[-1], commit),
            sha256 = sha256 or "0000000000000000000000000000000000000000000000000000000000000000",
            **kwargs)
