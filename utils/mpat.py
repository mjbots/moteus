#!/usr/bin/python3

# Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

"""Wrapper for the moteus Performance Analysis Tool CLI.

This script locates the JavaScript CLI and its dependencies and invokes
Node.js to run the analysis. It works both under Bazel (using runfiles)
and standalone (using paths relative to the script).
"""

import os
import subprocess
import sys

# Try to import Bazel runfiles, but don't fail if unavailable
try:
    from bazel_tools.tools.python.runfiles import runfiles
    HAVE_RUNFILES = True
except ImportError:
    HAVE_RUNFILES = False


def find_files_bazel():
    """Locate files using Bazel runfiles."""
    r = runfiles.Create()
    cli_path = r.Rlocation("com_github_mjbots_moteus/utils/mpat_cli.mjs")
    core_path = r.Rlocation("com_github_mjbots_moteus/utils/mpat_core.mjs")
    return cli_path, core_path


def find_files_standalone():
    """Locate files relative to this script's directory."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cli_path = os.path.join(script_dir, "mpat_cli.mjs")
    core_path = os.path.join(script_dir, "mpat_core.mjs")
    return cli_path, core_path


def extract_core_to_temp(script_dir):
    """Extract mpat_core.mjs to a temp directory, return path or None."""
    import tempfile

    extract_script = os.path.join(script_dir, "extract_mpat_core.sh")
    mpat_html = os.path.join(script_dir, "..", "docs", "mpat.html")

    if not os.path.exists(extract_script) or not os.path.exists(mpat_html):
        return None

    # Use a stable temp directory so we don't re-extract every time
    tmpdir = os.path.join(tempfile.gettempdir(), "mpat_cache")
    os.makedirs(tmpdir, exist_ok=True)
    output_path = os.path.join(tmpdir, "mpat_core.mjs")

    # Check if we need to re-extract (source newer than cached)
    if os.path.exists(output_path):
        src_mtime = os.path.getmtime(mpat_html)
        dst_mtime = os.path.getmtime(output_path)
        if dst_mtime >= src_mtime:
            return output_path

    print("Extracting mpat_core.mjs...", file=sys.stderr)
    result = subprocess.run(
        ["bash", extract_script, mpat_html, output_path],
        cwd=script_dir
    )
    if result.returncode == 0 and os.path.exists(output_path):
        return output_path
    return None


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Try Bazel runfiles first, fall back to standalone paths
    if HAVE_RUNFILES:
        try:
            cli_path, core_path = find_files_bazel()
            if cli_path and os.path.exists(cli_path):
                # Bazel mode
                pass
            else:
                # Runfiles available but files not found, try standalone
                cli_path, core_path = find_files_standalone()
        except Exception:
            cli_path, core_path = find_files_standalone()
    else:
        cli_path, core_path = find_files_standalone()

    # Validate CLI exists
    if not cli_path or not os.path.exists(cli_path):
        print("Error: Could not find mpat_cli.mjs", file=sys.stderr)
        sys.exit(1)

    # If core doesn't exist, try to extract it to temp (standalone mode only)
    if not core_path or not os.path.exists(core_path):
        core_path = extract_core_to_temp(script_dir)
        if not core_path:
            print("Error: Could not find or extract mpat_core.mjs", file=sys.stderr)
            sys.exit(1)

    # The CLI imports mpat_core.mjs from the same directory.
    # Always copy both to temp to ensure they're colocated.
    import tempfile
    import shutil

    tmpdir = tempfile.mkdtemp(prefix="mpat_")
    tmp_cli = os.path.join(tmpdir, "mpat_cli.mjs")
    tmp_core = os.path.join(tmpdir, "mpat_core.mjs")
    shutil.copy(cli_path, tmp_cli)
    shutil.copy(core_path, tmp_core)

    # Run Node.js with the CLI, passing through all arguments
    result = subprocess.run(
        ["node", tmp_cli] + sys.argv[1:],
        cwd=tmpdir
    )

    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
