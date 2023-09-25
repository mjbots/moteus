#!/usr/bin/python3

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

import datetime
import os
import subprocess
import sys
import argparse


def run(cmd):
    print('>{}'.format(cmd))
    subprocess.check_call(cmd, shell=True)

def main():
    parser = argparse.ArgumentParser(
        description=f'{__file__} is used to build the firmware and create a flashable .elf artifact.')
    parser.add_argument('outdir', nargs='?', default='release', help='Target directory for the build artifacts.')
    args = parser.parse_args()

    outdir = args.outdir
    os.makedirs(outdir, exist_ok=True)

    # Make sure git is clean first.
    dirty = (subprocess.run("git diff-index --quiet HEAD --",
                            shell=True).returncode != 0)
    if dirty:
        raise RuntimeError("git is dirty, cannot release!")
    git_hash = subprocess.check_output(
        "git rev-parse HEAD", shell=True).decode('utf8').strip()

    datestr = datetime.datetime.now().strftime('%Y%m%d')

    print("Details:")
    print(" Date:", datestr)
    print(" Output Directory:", outdir)
    print(" Git Hash:", git_hash)
    print("Building...")
    print()

    run('tools/bazel clean --expunge')
    run('tools/bazel build --config=target -c opt //:target')

    run(f'cp bazel-bin/fw/moteus.elf {outdir}/{datestr}-moteus-{git_hash}.elf')
    run(f'cp bazel-bin/fw/can_bootloader.elf {outdir}/{datestr}-bootloader-{git_hash}.elf')

    print()
    print('DONE')

if __name__ == '__main__':
    main()
