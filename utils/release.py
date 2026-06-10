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

"""Cut a release tag for one component.

Usage:
    utils/release.py <component> <bump|version>

Components:
    firmware  - validates fw/moteus_hw.h ABI matches and tags firmware/vX.Y.Z
    python    - bumps tools/python.bzl and tags python/vX.Y.Z
    cpp       - bumps CMakeLists.txt and tags cpp/vX.Y.Z
    rust      - bumps lib/rust/Cargo.toml + Cargo.lock and tags rust/vX.Y.Z

Bump types:
    major | minor | patch
    rc              iterate a pre-release suffix (1.0.0-rc.1 -> 1.0.0-rc.2)
    <explicit>      e.g. 1.0.0-rc.1, 1.1.0

When current is a pre-release, `patch` graduates it (1.0.0-rc.1 -> 1.0.0).

Firmware, cpp, and rust use semver pre-release suffixes (1.0.0-rc.1).  The
python package must use normalized PEP 440 forms instead (1.0.0rc1, 1.0.0b1,
1.0.0.dev1); a non-normalized python version is rejected because PyPI and
the bazel wheel build name artifacts by the normalized form.

The rust release covers three crates (moteus, moteus-derive,
moteus-protocol) that share a single workspace version and ship in
lockstep, like the two python packages.

For firmware, MOTEUS_FIRMWARE_VERSION is bumped during feature work
(alongside SUPPORTED_ABI_VERSION in moteus_tool.py and any matching
upgrade rules).  This script never modifies it; it only validates that
its M.m does not exceed the M.m of the chosen release.
"""

import argparse
import pathlib
import re
import subprocess
import sys

from encode_firmware_version import decode, encode

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent


def git(*args, capture=False, check=True):
    return subprocess.run(['git', *args], cwd=REPO_ROOT, check=check,
                          capture_output=capture, text=True)


def read_firmware_constant():
    text = (REPO_ROOT / 'fw' / 'moteus_hw.h').read_text()
    m = re.search(r'^#define MOTEUS_FIRMWARE_VERSION (0x[0-9a-fA-F]+)$',
                  text, re.MULTILINE)
    if not m:
        sys.exit('could not find MOTEUS_FIRMWARE_VERSION in fw/moteus_hw.h')
    packed = m.group(1)
    return packed, decode(int(packed, 16))


def latest_firmware_tag():
    r = git('describe', '--tags', '--abbrev=0', '--match=firmware/v*',
            capture=True, check=False)
    return r.stdout.strip() if r.returncode == 0 else None


def read_version(path, regex):
    text = (REPO_ROOT / path).read_text()
    m = re.search(regex, text, re.MULTILINE)
    if not m:
        sys.exit(f'could not find version in {path}')
    return m.group(1)


def write_version(path, regex, replacement):
    full = REPO_ROOT / path
    full.write_text(re.sub(regex, replacement, full.read_text(),
                           count=1, flags=re.MULTILINE))


def compute_new_version(current, bump):
    m = re.match(r'^(\d+)\.(\d+)\.(\d+)(.*)$', current)
    if not m:
        sys.exit(f'could not parse current version: {current!r}')
    major, minor, patch = int(m.group(1)), int(m.group(2)), int(m.group(3))
    suffix = m.group(4)

    if bump == 'major':
        return f'{major+1}.0.0'
    if bump == 'minor':
        return f'{major}.{minor+1}.0'
    if bump == 'patch':
        # Graduate a pre-release: 1.0.0-rc.1 + patch -> 1.0.0.
        # Otherwise bump the patch byte normally.
        return f'{major}.{minor}.{patch}' if suffix else f'{major}.{minor}.{patch+1}'
    if bump == 'rc':
        # Iterate a pre-release: increment the trailing number in the suffix.
        # Works for both semver (1.0.0-rc.1) and PEP 440 (1.0.0rc1) forms.
        if not suffix:
            sys.exit(f'current {current!r} has no pre-release suffix; '
                     f'use an explicit version to start one (e.g. '
                     f'{major}.{minor}.{patch+1}-rc.1, or '
                     f'{major}.{minor}.{patch+1}rc1 for python)')
        m2 = re.match(r'^(.*?)(\d+)$', suffix)
        if not m2:
            sys.exit(f'pre-release {suffix!r} has no numeric tail to bump')
        return f'{major}.{minor}.{patch}{m2.group(1)}{int(m2.group(2))+1}'
    # Explicit version.  Accepts both semver pre-release (1.0.0-rc.1)
    # and PEP 440 (1.0.0rc1, 1.0.0.dev1) forms.  The firmware constant
    # only encodes M.m.p so any suffix is dropped there; the python
    # package needs PEP 440 to be uploadable to PyPI.
    if re.match(r'^\d+\.\d+\.\d+([.-]?[A-Za-z0-9.+-]*)?$', bump):
        return bump
    sys.exit(f'unknown bump or invalid version: {bump!r}')


def validate_firmware_abi(constant_packed, constant_version, new_version):
    new_packed = f'0x{encode(new_version):06x}'
    # The constant tracks the firmware ABI version (only bumps when
    # the protocol surface actually changes).  The release version may
    # move ahead of it on non-ABI-affecting MINOR/PATCH releases.
    # The invariant is: ABI's M.m must be <= release's M.m.
    constant_mm = int(constant_packed, 16) & 0xffff00
    new_mm = int(new_packed, 16) & 0xffff00
    if constant_mm > new_mm:
        sys.exit(
            f'\nERROR: firmware ABI ({constant_packed} = {constant_version}) '
            f'is newer than\n'
            f'the requested release version ({new_version}).\n\n'
            f"The ABI version (M.m) cannot exceed the release version's M.m.  "
            f'Either:\n\n'
            f'  - Pick a release version whose M.m is at least '
            f"{constant_version}'s M.m\n"
            f'    (e.g. utils/release.py firmware minor or major).\n\n'
            f'  - Or roll back fw/moteus_hw.h if the ABI bump in the source '
            f'tree was\n'
            f'    a mistake.'
        )


def validate_python_version(version):
    """Reject python versions that are not in normalized PEP 440 form.

    `python3 -m build` (and PyPI) name the wheel and sdist using the
    PEP 440 *normalized* version, but the genrule in lib/python/BUILD
    declares its output filenames straight from the raw VERSION string.
    If VERSION is a non-normalized spelling such as the semver-style
    '1.0.0-rc1', the declared outputs ('moteus-1.0.0-rc1-*') never match
    what build produces ('moteus-1.0.0rc1-*') and `tools/bazel test
    //:host` fails with "declared output ... was not created by genrule".

    Firmware and cpp keep using semver, so this check is python-only.
    """
    try:
        from packaging.version import Version, InvalidVersion
    except ImportError:
        sys.exit('python releases need the `packaging` module to validate '
                 'the version (apt install python3-packaging)')
    try:
        normalized = str(Version(version))
    except InvalidVersion:
        sys.exit(f'{version!r} is not a valid PEP 440 version')
    if normalized != version:
        sys.exit(
            f'\nERROR: python version {version!r} is not normalized PEP 440.'
            f'\n\nPyPI and the bazel wheel build name artifacts by the '
            f'normalized form, so\nthis spelling would break the build.  Use '
            f'{normalized!r} instead:\n\n'
            f'    utils/release.py python {normalized}\n')


def validate_rust_version(version):
    """Reject rust versions that are not plain semver.

    crates.io requires semver, so PEP 440 spellings like '1.0.0rc1'
    (which compute_new_version accepts for python's sake) must be
    rejected here.  Build metadata ('+...') is also rejected because
    the version is written verbatim into the inter-crate dependency
    requirements in lib/rust/Cargo.toml, where it would be meaningless.
    """
    if not re.match(r'^\d+\.\d+\.\d+'
                    r'(-[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?$', version):
        sys.exit(
            f'\nERROR: rust version {version!r} is not plain semver.'
            f'\n\ncrates.io requires semver; use e.g. 1.0.0 or 1.0.0-rc.1 '
            f'(not the PEP 440\nforms used for python releases).\n')


def update_rust_versions(new):
    """Write the new version everywhere the rust workspace records it.

    The three crates inherit workspace.package.version from
    lib/rust/Cargo.toml and ship in lockstep.  The inter-crate
    dependency requirements in [workspace.dependencies] are kept equal
    to the full version so a regex bump can never leave them stale.
    Cargo.lock records each workspace member's version; updating it
    here keeps `cargo --locked` and the bazel build consistent without
    needing a cargo invocation at release time.
    """
    def sub_checked(path, regex, replacement, count):
        full = REPO_ROOT / path
        text, n = re.subn(regex, replacement, full.read_text(),
                          flags=re.MULTILINE)
        if n != count:
            sys.exit(f'expected {count} match(es) of {regex!r} in {path}, '
                     f'found {n}')
        full.write_text(text)

    sub_checked('lib/rust/Cargo.toml',
                r'^version = "[^"]+"', f'version = "{new}"', count=1)
    for crate in ('moteus-derive', 'moteus-protocol'):
        sub_checked(
            'lib/rust/Cargo.toml',
            rf'^({crate} = {{ path = "{crate}", version = ")[^"]+(" }})$',
            rf'\g<1>{new}\g<2>', count=1)
    for crate in ('moteus', 'moteus-derive', 'moteus-protocol'):
        sub_checked('lib/rust/Cargo.lock',
                    rf'^(name = "{crate}"\nversion = ")[^"]+(")$',
                    rf'\g<1>{new}\g<2>', count=1)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('component',
                        choices=['firmware', 'python', 'cpp', 'rust'])
    parser.add_argument('bump',
                        help='major | minor | patch | rc | <explicit version>')
    args = parser.parse_args()

    version_file = None  # firmware does not modify a file
    constant_packed = constant_version = None

    if args.component == 'firmware':
        constant_packed, constant_version = read_firmware_constant()
        latest = latest_firmware_tag()
        current = (latest.removeprefix('firmware/v') if latest
                   else constant_version)
        tag_prefix = 'firmware/v'
    elif args.component == 'python':
        version_file = 'tools/python.bzl'
        current = read_version(version_file, r'^VERSION = "([^"]+)"')
        tag_prefix = 'python/v'
    elif args.component == 'rust':
        version_file = 'lib/rust/Cargo.toml'
        current = read_version(version_file, r'^version = "([^"]+)"')
        tag_prefix = 'rust/v'
    else:  # cpp
        version_file = 'CMakeLists.txt'
        current = read_version(version_file,
                               r'^project\(moteus VERSION ([^)]+)\)')
        tag_prefix = 'cpp/v'

    print(f'Current {args.component} version: {current}')
    new = compute_new_version(current, args.bump)
    print(f'New     {args.component} version: {new}')

    if args.component == 'python':
        validate_python_version(new)
    elif args.component == 'rust':
        validate_rust_version(new)

    tag = f'{tag_prefix}{new}'
    if git('rev-parse', '-q', '--verify', f'refs/tags/{tag}',
           capture=True, check=False).returncode == 0:
        sys.exit(f'tag {tag} already exists')

    commit_files = [version_file] if version_file else []
    if args.component == 'firmware':
        validate_firmware_abi(constant_packed, constant_version, new)
    elif args.component == 'python':
        write_version(version_file, r'^VERSION = "[^"]+"',
                      f'VERSION = "{new}"')
    elif args.component == 'rust':
        update_rust_versions(new)
        commit_files.append('lib/rust/Cargo.lock')
    else:  # cpp
        # CMake project(... VERSION X.Y.Z) does not allow pre-release
        # identifiers; strip anything after '-' for the file value but
        # keep the full string for the tag.
        cmake_value = new.split('-', 1)[0]
        write_version(version_file,
                      r'^(project\(moteus VERSION )[^)]+\)',
                      rf'\g<1>{cmake_value})')

    if commit_files and git('diff', '--quiet', '--', *commit_files,
                            check=False).returncode != 0:
        git('add', *commit_files)
        git('commit', '-m', f'Release {args.component} {new}')
    elif not commit_files:
        print('(no version-file change to commit; tagging current HEAD)')

    git('tag', '-a', tag, '-m', f'{args.component} {new}')

    print(f'\nCreated tag: {tag}\n')
    print('Next steps:')
    print('  git push origin HEAD')
    print(f'  git push origin {tag}\n')
    print(f'After the tag is pushed, the build-{args.component}-release '
          f'workflow will')
    print('produce artifacts and attach them to a GitHub Release.\n')
    print('For a major or minor firmware release, also cut a maintenance '
          'branch:')
    print(f'  utils/cut_release_branch.py {tag}')


if __name__ == '__main__':
    main()
