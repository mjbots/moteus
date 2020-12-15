# -*- python -*-

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

package(default_visibility = ["//visibility:public"])

test_suite(
    name = "host",
    tests = [
        "//lib/python/moteus:test",
        ":bdist_test",
    ],
)

# It would be nice to pull this from the python source.  As it is, the
# below genrule will fail if they don't match, so at least there is
# some verification.
VER="0.2.0"

genrule(
    name = "bdist_wheel",
    srcs = ["setup.py", "README.md", "//lib/python/moteus"],
    outs = [
        "moteus-{ver}-py3-none-any.whl".format(ver=VER),
        "moteus-{ver}.tar.gz".format(ver=VER),
    ],
    cmd = (
        "mkdir -p $$(dirname $(location moteus-{ver}.tar.gz)) && " +
        "RELOUT=$$(realpath --relative-to $$(dirname $(location setup.py)) " +
        "          $$(dirname $(location moteus-{ver}.tar.gz))) && " +
        "cd $$(dirname $(location setup.py)) && " +
        "python3 setup.py sdist --dist-dir $$RELOUT 1>/dev/null && " +
        "python3 setup.py bdist_wheel --dist-dir $$RELOUT 1> /dev/null " +
        ""
    ).format(ver=VER),
)

sh_test(
    name = "bdist_test",
    data = [":bdist_wheel"],
    srcs = ["test/bdist_test.sh"],
)
