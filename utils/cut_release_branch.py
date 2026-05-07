#!/usr/bin/python3 -B

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

"""Create a maintenance branch from a firmware MAJOR or MINOR release tag.

Usage:
    utils/cut_release_branch.py firmware/v1.0.0

Result: creates a release/firmware-1.0.x branch tracking the tag.
Bug-fix releases (1.0.1, 1.0.2, ...) are then cut from that branch.
"""

import argparse
import re
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('tag', metavar='firmware/vX.Y.Z')
    args = parser.parse_args()

    m = re.match(r'^firmware/v(\d+)\.(\d+)\.\d+(-.*)?$', args.tag)
    if not m:
        sys.exit(f'expected firmware/vX.Y.Z[-pre], got: {args.tag}')

    branch = f'release/firmware-{m.group(1)}.{m.group(2)}.x'

    if subprocess.run(['git', 'show-ref', '--verify', '--quiet',
                       f'refs/heads/{branch}']).returncode == 0:
        sys.exit(f'branch {branch} already exists locally')

    subprocess.run(['git', 'branch', branch, args.tag], check=True)
    print(f'Created branch: {branch}')
    print(f'Push it with:   git push origin {branch}')


if __name__ == '__main__':
    main()
