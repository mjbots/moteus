# Releasing moteus

Each shipped artifact (firmware, Python client, C++ client, Rust client)
carries its own [semver](https://semver.org/) version and its own release
cadence. Tags are prefixed by component:

| Component | Version source                          | Tag prefix       |
|-----------|-----------------------------------------|------------------|
| firmware  | `fw/moteus_hw.h` (`MOTEUS_FIRMWARE_VERSION`) | `firmware/v...` |
| python    | `tools/python.bzl` (`VERSION`)          | `python/v...`    |
| cpp       | `CMakeLists.txt` (`project(... VERSION ...)`) | `cpp/v...`  |
| rust      | `lib/rust/Cargo.toml` (`[workspace.package] version`) | `rust/v...` |

Firmware, cpp, and rust pre-releases use semver suffixes (`1.0.0-rc.1`,
`1.0.0-beta.1`); the python package uses normalized PEP 440 forms instead
(`1.0.0rc1`, see [Python version strings](#python-version-strings)). For
firmware, the on-wire version (register 0x101) is the packed
`MAJOR.MINOR.PATCH` only,
so a `-rc.N` build of `1.0.0` reports exactly the same value over CAN as the
final `1.0.0`. They are ABI-equivalent on purpose; the build identity comes
from `fw/git_info.h` (commit SHA + dirty flag + timestamp).

## Firmware ABI version vs release version

`MOTEUS_FIRMWARE_VERSION` in `fw/moteus_hw.h` is the **firmware ABI
version** — what register 0x101 reports over CAN. It is independent of
the firmware release version (the `firmware/vX.Y.Z` git tag).

The ABI constant only bumps when the calibration process changes, or
configuration fields either change interpretation or require
conversion across upgrades.

By convention every ABI bump on this project has been
backwards-compatible in one direction: a newer client can talk to any
older firmware, but an older client cannot calibrate against newer
firmware and may or may not be able to successfully command a newer
firmware. So an ABI bump maps to a **MINOR** semver step. A real ABI
break that prevents new clients from talking to old firmware would be
MAJOR — that has not happened yet.

`SUPPORTED_ABI_VERSION` in `lib/python/moteus/moteus_tool.py` is the
highest firmware ABI that `moteus_tool --calibrate` knows how to handle.
In the source tree it tracks `MOTEUS_FIRMWARE_VERSION` exactly: when
firmware feature work bumps the ABI constant, the same commit (or
adjacent commit) bumps `SUPPORTED_ABI_VERSION` to match and adds the
new `FirmwareUpgrade.fix_config` rule.

The helper bumps both at once:

```bash
python3 utils/encode_firmware_version.py update \
    --firmware 1.1.0 --supported-abi 1.1.0
git commit -am "Bump firmware ABI to 1.1.0 for new <feature>"
```

`utils/release.py firmware <bump>` never modifies these constants. It
**validates** that `MOTEUS_FIRMWARE_VERSION`'s M.m is not greater than
the M.m of the release being tagged. The release's M.m may legitimately
move ahead of the ABI's M.m (a non-ABI-affecting MINOR release), but
the reverse — ABI ahead of release — would mean the firmware is
shipping with a newer ABI than its version number claims.

## When you do (and don't) need a Python release

The on-wire compatibility check that protects users is:

> published-moteus_tool `SUPPORTED_ABI_VERSION` ≥ firmware's reported
> ABI (register 0x101)

So:

- **Firmware patch release, no ABI change**: no Python release needed.
- **Firmware MINOR release, no ABI change**: no Python release needed.
- **Firmware MINOR release, ABI changed**: Python release needed so the
  published wheel knows about the new ABI. Treat it as a MINOR Python
  bump (it adds a capability — the tool now accepts firmwares it used
  to refuse).
- **Firmware MAJOR release**: Python release needed (still MINOR on the
  Python side; Python's own major version is a separate concern that
  moves only on Python API breaks).

Until the next Python release ships, users running `pip install moteus`
with the new firmware will hit the friendly "moteus_tool needs to be
upgraded" error from `SUPPORTED_ABI_VERSION` — which is correct
behavior, not a regression.

Python (and C++/Rust) releases are also of course necessary if the
functionality of the relevant library is improved or bugs are fixed.
The Rust client does not perform calibration, so it has no
`SUPPORTED_ABI_VERSION` concern — like C++, it only needs a release
when its own functionality changes.

## Cutting a release

Run the helper from the repo root, then push:

```bash
# patch bump (1.0.0 -> 1.0.1)
utils/release.py firmware patch

# minor bump
utils/release.py python minor

# start a pre-release cycle (explicit)
utils/release.py firmware 1.1.0-rc.1

# iterate a pre-release (1.1.0-rc.1 -> 1.1.0-rc.2)
utils/release.py firmware rc

# graduate a pre-release to its final release (1.1.0-rc.2 -> 1.1.0)
utils/release.py firmware patch
```

The `patch` bump is overloaded for convenience: when current is already a
pre-release, it strips the suffix instead of incrementing the patch byte
(matches npm/cargo conventions). `minor` and `major` always advance
M.m.p as you'd expect, regardless of any pre-release suffix.

The script:

1. Determines the current release version (from the latest `firmware/v*`
   tag for firmware, or from the version file for python/cpp).
2. Computes the new version (or accepts an explicit one).
3. For firmware: validates `MOTEUS_FIRMWARE_VERSION` matches the new
   release's M.m.p; refuses to tag otherwise.
4. For python/cpp/rust: updates the version file(s) in place. For rust
   this is `lib/rust/Cargo.toml` (the workspace version plus the
   inter-crate dependency requirements) and the matching package
   entries in `lib/rust/Cargo.lock`.
5. Commits (if the files changed) and creates the annotated tag.
6. Prints the `git push` commands.

Inspect the diff and tag before pushing.

```bash
git push origin HEAD
git push origin firmware/v1.0.1
```

The matching `build-<component>-release` workflow then builds the
artifact and creates a GitHub Release **as a draft**.  Drafts are not
visible to users browsing the Releases page or to `pip`/`FetchContent`
consumers — they exist only on the maintainer's view of the page.

You can also kick off a release from the GitHub UI via the
[`Release`](.github/workflows/release.yml) workflow — it runs the same
script in CI, useful when releasing from somewhere without your local
toolchain.

## Promoting a draft release to public

After the build workflow finishes, validate the draft locally:

1. Go to the repo's Releases page → the draft will show under "Draft"
   at the top.  Download the attached artifacts.
2. For firmware: flash on representative hardware and run your
   validation suite.  For python: `pip install` the wheel into a fresh
   venv and exercise `moteus_tool` against a device.  For cpp: build a
   downstream project against the tag.  For rust: build a downstream
   project against the tag (`moteus = { git = ..., tag = "rust/vX.Y.Z" }`)
   and exercise it against a device.
3. When satisfied, publish.  Either click "Publish release" in the UI,
   or:

   ```bash
   gh release edit firmware/v1.0.0 --draft=false
   ```

If something is wrong, abort instead of publishing:

```bash
gh release delete firmware/v1.0.0 --yes
git push --delete origin firmware/v1.0.0
git tag -d firmware/v1.0.0
# ...fix the issue, then re-tag
```

For python, **promoting the draft is what triggers the PyPI publish**.
The `publish-python-pypi.yml` workflow listens for the `release:
published` event, downloads the wheel and sdist attached to the
release, and uploads them to PyPI via OIDC. Do not click "Publish
release" on a python draft until you intend the wheel to land on
PyPI — PyPI is append-only, you cannot un-publish.

Likewise for rust, **promoting the draft is what triggers the
crates.io publish**. The `publish-rust-crates.yml` workflow checks out
the release tag and runs `cargo publish --workspace` via OIDC trusted
publishing. crates.io is also append-only — a published version can be
yanked but never deleted or reused.

## Cutting a maintenance branch

After every firmware MAJOR or MINOR release, cut a maintenance branch so
that future bugfixes can be backported to it without dragging in unrelated
`main` work:

```bash
utils/cut_release_branch.py firmware/v1.0.0
git push origin release/firmware-1.0.x
```

For a 1.0.1 patch release: land the fix on `main`, then:

```bash
git checkout release/firmware-1.0.x
git cherry-pick <sha>
utils/release.py firmware patch
git push origin HEAD release/firmware-1.0.x
git push origin firmware/v1.0.1
```

## Updating SUPPORTED_ABI_VERSION

`SUPPORTED_ABI_VERSION` in `lib/python/moteus/moteus_tool.py` is bumped
together with `MOTEUS_FIRMWARE_VERSION` during the feature work that
introduces the ABI change (see "Firmware ABI semantics" above). It is
not a release-time concern; by the time you run `utils/release.py
firmware ...`, the constants should already match.

If they have somehow drifted, fix them in a single source commit before
tagging:

```bash
python3 utils/encode_firmware_version.py update \
    --firmware <version> --supported-abi <version>
git commit -am "Sync firmware ABI version constants"
```

The published moteus_tool wheel doesn't pick up the new
`SUPPORTED_ABI_VERSION` until the next `python/v*` release. Until then,
users running `pip install moteus` will hit the friendly
"moteus_tool needs to be upgraded" error when calibrating against the
newer firmware. That's the intended behavior.

## What each workflow does

- `release.yml` — manual `workflow_dispatch` to bump and tag from CI.
- `build-firmware-release.yml` — fires on `firmware/v*`. Builds via
  `tools/bazel build --config=target -c opt //:target` and attaches
  the two flashable ELFs (`moteus-fw-<version>+g<sha>.elf` and
  `moteus-bl-<version>+g<sha>.elf`) to the Release. The `+g<sha>`
  portion is semver build metadata — the 10-char git short SHA of
  the tagged commit, ignored for version precedence.
- `build-python-release.yml` — fires on `python/v*` tag push. Builds
  both `//lib/python:bdist_wheel` (the `moteus` package) and
  `//utils/gui:bdist_wheel` (the `moteus_gui` package) and attaches
  the wheels + sdists for both to a draft Release. The two packages
  share the version from `tools/python.bzl` and ship in lockstep.
- `publish-python-pypi.yml` — fires when a `python/v*` Release is
  promoted out of draft. Downloads the attached wheels + sdists and
  uploads them to PyPI via OIDC trusted publishing (one upload covers
  both packages). See [Publishing to PyPI](#publishing-to-pypi) below.
- `build-cpp-release.yml` — fires on `cpp/v*`. Creates a Release page;
  GitHub auto-attaches whole-repo source archives. The C++ client is
  header-only and is normally consumed via `FetchContent` pointing at the
  tag.
- `build-rust-release.yml` — fires on `rust/v*` tag push. Runs
  `cargo package --workspace` and attaches the three `.crate` files
  (`moteus`, `moteus-derive`, `moteus-protocol`) to a draft Release.
  The three crates share the version from `lib/rust/Cargo.toml` and
  ship in lockstep.
- `publish-rust-crates.yml` — fires when a `rust/v*` Release is
  promoted out of draft. Checks out the tag and runs
  `cargo publish --workspace`, which publishes the three crates to
  crates.io in dependency order via OIDC trusted publishing. See
  [Publishing to crates.io](#publishing-to-cratesio) below.

## Python version strings

PyPI requires [PEP 440](https://peps.python.org/pep-0440/) versions, which
are *not* identical to semver. Use the PEP 440 forms when releasing the
Python package:

| Kind          | Semver        | PEP 440 (use this for python) |
|---------------|---------------|-------------------------------|
| Release       | `1.0.0`       | `1.0.0`                       |
| Release cand. | `1.0.0-rc.1`  | `1.0.0rc1`                    |
| Beta          | `1.0.0-beta.1`| `1.0.0b1`                     |
| Alpha         | `1.0.0-alpha.1`| `1.0.0a1`                    |
| Dev           | `1.0.0-dev.1` | `1.0.0.dev1`                  |
| Post          | (n/a)         | `1.0.0.post1`                 |

The firmware, C++, and Rust tags continue to use semver
(`firmware/v1.0.0-rc.1`, `rust/v1.0.0-rc.1` — crates.io requires it).
They're independent components and free to use different conventions.

```bash
# python pre-release
utils/release.py python 1.0.0rc1
# tag: python/v1.0.0rc1

# firmware pre-release (semver)
utils/release.py firmware 1.0.0-rc.1
# tag: firmware/v1.0.0-rc.1
```

## Publishing to PyPI

PyPI publishing uses [trusted publishing](https://docs.pypi.org/trusted-publishers/)
via OIDC, so no API token is stored in this repo.

A python release covers two PyPI projects — `moteus` (the client
library) and `moteus_gui` (the GUI tools, which depends on `moteus`).
They share a single version from `tools/python.bzl` and ship together.

**One-time setup** (do this for *both* projects):

1. Sign in to PyPI as a maintainer of the project.
2. Project settings → Publishing → Add a new pending publisher (or
   trusted publisher if the project exists). Configure:
   - Owner: `mjbots`
   - Repository name: `moteus`
   - Workflow name: `publish-python-pypi.yml`
   - Environment name: `pypi`
3. Repeat for the second project — both `moteus` and `moteus_gui`
   need the same trusted-publisher entry. A single workflow run
   uploads to both, but each PyPI project authenticates the OIDC
   token independently.
4. In this GitHub repo: Settings → Environments → New environment
   named `pypi`. The environment exists so PyPI's trusted-publishing
   config has something to bind to; no required reviewers are needed
   because the "Publish release" action on the draft is the gate.

After setup, the python release flow is:

1. Push a `python/v*` tag → `build-python-release.yml` builds both
   wheels + sdists (`moteus-X.Y.Z*` and `moteus_gui-X.Y.Z*`) and
   creates a draft GitHub Release with all four files attached.
2. Validate locally (`pip install` both wheels into a fresh venv,
   exercise `moteus_tool` against a device, run `tview`).
3. Publish the draft via the GitHub UI or
   `gh release edit python/vX.Y.Z --draft=false`.
4. Promoting the draft fires `publish-python-pypi.yml`, which
   downloads the attached wheels + sdists and uploads them to PyPI
   via OIDC. PyPI authenticates each upload against the matching
   project's trusted-publisher config.

A pre-release version (anything PEP 440 considers pre-release, e.g.
`1.0.0rc1`) goes to PyPI but `pip install moteus` won't pick it up
unless the user passes `--pre`. So pre-releases are safe to publish.

PyPI is append-only — you can yank a release but cannot delete it or reuse
the version number. The environment-gate is the safety net.

## Publishing to crates.io

crates.io publishing uses
[trusted publishing](https://crates.io/docs/trusted-publishing) via
OIDC, so no API token is stored in this repo — the same model as PyPI.

A rust release covers three crates — `moteus` (the client library),
`moteus-protocol` (the no_std protocol types), and `moteus-derive`
(the proc macros). They share a single version from
`lib/rust/Cargo.toml` and ship together; `cargo publish --workspace`
publishes them in dependency order automatically.

**Bootstrap**: crates.io only allows trusted publishing to be
configured on a crate that already exists, so the *very first* publish
of each crate must be done manually by a maintainer:

```bash
cd lib/rust
cargo publish --workspace --locked   # uses your `cargo login` token
```

**One-time setup** (after the first publish, for *each* of the three
crates):

1. Sign in to crates.io as an owner of the crate.
2. Crate → Settings → Trusted Publishing → Add. Configure:
   - Repository owner: `mjbots`
   - Repository name: `moteus`
   - Workflow filename: `publish-rust-crates.yml`
   - Environment: `crates-io`
3. In this GitHub repo: Settings → Environments → New environment
   named `crates-io` (analogous to the `pypi` environment; the
   "Publish release" action on the draft is the gate).

After setup, the rust release flow is:

1. Push a `rust/v*` tag → `build-rust-release.yml` packages the three
   crates and creates a draft GitHub Release with the `.crate` files
   attached.
2. Validate locally (point a downstream project's `Cargo.toml` at the
   tag, exercise it against a device).
3. Publish the draft via the GitHub UI or
   `gh release edit rust/vX.Y.Z --draft=false`.
4. Promoting the draft fires `publish-rust-crates.yml`, which checks
   out the tag, exchanges an OIDC token with crates.io, and runs
   `cargo publish --workspace --locked`.

A pre-release version (`1.0.0-rc.1`) goes to crates.io but cargo will
not select it for a normal requirement like `moteus = "1.0"`; users
must opt in with an explicit pre-release requirement
(`moteus = "1.0.0-rc.1"`). So pre-releases are safe to publish.

crates.io is append-only — you can yank a version but cannot delete it
or reuse the version number. The environment-gate is the safety net.
