# Changelog

This file tracks notable changes per shipped component. Each component is
released independently; see [RELEASING.md](RELEASING.md) for the process.

The format is loosely based on [Keep a Changelog](https://keepachangelog.com/).

Entries are added under a component's `### Unreleased` heading by the
change that introduces them; cutting a release only renames that
heading to the version and date.

## firmware

### Unreleased

(no entries yet)

### 1.1.1 - 2026-08-04

- Change the default gate drive strength for moteus-n1 to the values which it was qualified at.  When the moteus-c1 was introduced, the moteus-n1 was accidentally set to more aggressive gate drive strength values.  This could result in increased gate drive failure rates with poor phase wire soldering or input voltages above 36V.
- Improve hall effect performance
  - Reduce velocity ripple by not starting the velocity decay process until *after* the expected transition
  - Improve velocity estimation when hall rising and falling times are mismatched
- Fix AksIM-2 wrapping on startup
- CUI AMT21: Actually operate the RS485 bus in half-duplex mode.  Previously, transmit was always asserted, which would occasionally sometimes work, but resulted in two masters on the bus at the same time.
- Improve I2C aux port robustness
  - Reset peripherals that are unresponsive
  - Enable the STM32 analog filter
  - Disable the STM32 own address matching feature
- Fix the location of the default UART port for moteus-c1 and moteus-n1.  The location of the UART port by default for all controllers now matches the documentation:
  - moteus-r4: AUX2/ABS
  - moteus-n1: AUX1
  - moteus-c1: AUX2
  - moteus-x1: AUX1

### 1.1.0 - 2026-08-04

Erroneous release with no changelog

### 1.0.0 - 2026-05-28

#### Firmware improvements

- Use a CSA gain based kCurrentSampleTime - this reduces the effective modulation depth in all configurations, more with the default CSA gain, but is actually correct and does not result in spurious current sense noise at high duty cycles no matter what CSA gain is configured
- Actually count errors if hall sensors flip more than one bit at a time
- Fix the used bitrate for I2C2, which affects I2C on aux2.  Previously, actual operation was at half the configured bitrate.
- When targetting a near-zero speed with an acceleration limit, in some cases we could oscillate.  Fix this to reach the target without oscillation.
- Force a hall sensor update on boot, if we otherwise got unlucky, a hall configuration could report no encoder configured
- For hall sensors, if our "slow velocity" heuristic coasts to the end of a sector, but then no more hall updates arrive, gradually recenter back to the center of the sector

#### Other firmware behavior changes

- Fix various undefined behaviors and assertions in the firmware that could be triggered by misconfiguration.

#### Utilities and examples

- `utils/measure_ma732_bct.py` is more likely to work if Y axis trimming is required.
- `utils/measure_inertia.py` has improved accuracy
- Various compensation scripts could fail if encoder values were unfortunately situated relative to the pi/-pi boundary
- `utils/plot_highrate.py` accepts hex / oct / bin literals for --emit-debug
- `lib/python/examples/ruckig_multiservo.py` both works properly on Windows and reports the mode and fault for each device

#### Release engineering

- First semver release, ABI bumped to 0x010000 (1.0.0), although no functional ABI changes were made.
- Implement a ubsan build, verify it in CI.
- Support building on Ubuntu 26.04

## python

### Unreleased

- moteus_tool now reports hall effect performance in the calibration report

### 1.1.2 - 2026-08-04

- Improve hall effect calibration
  - Fine calibration of hall transition point within the electrical cycle
  - Sweep through 3 electrical cycles in forward and reverse direction
  - If a hall sensor transition happens to be right on a sample point and oscillates, do something reasonable
- Support firmware ABI 1.1.0

### 1.1.1 - 2026-08-04

Unreleased version

### 1.1.0 - 2026-08-04

Erroneous release with no changelog

### 1.0.0 - 2026-05-28

#### Improvements

- `moteus_tool --calibrate` for controllers configured for hall commutation now calibrate a fine position for each hall transition, which improves performance and consistency for hall based configurations

#### Fixes

- `moteus.move_to` now respects the `velocity_limit` argument across all devices
- `moteus.move_to` works properly when exiting the stopped state
- Don't inject `await` when it isn't needed, rending some python mode definitions possible that previously would have triggered syntax errors
- Fix the order of D and Q axes for current mode requests

## cpp

### Unreleased

(no entries yet)

#### 1.0.0 - 2026-05-28

* First semver based release.

## rust

The `moteus`, `moteus-protocol`, and `moteus-derive` crates share one
version and are released together.

### Unreleased

#### Improvements

- The examples accept transport-specific arguments and report those in
  help messages.
- New `transport::args::parse_with_transport_args::<T>()`: parses a
  clap derive struct together with every registered transport
  argument.
- The `bandwidth_test` example uses the synchronous transport.
- docs.rs renders the feature-gated API (async transports, clap
  helpers).
- The bazel crate specifications are exported from
  `lib/rust/crates.bzl` for downstream workspaces.

#### Fixes

- The crates build on their declared MSRV of 1.75 again (0.5.3 used
  `Option::is_none_or`, which needs 1.82). CI now checks the MSRV.
- `TransportOptions::from_arg_matches()` no longer panics when the
  command defines only a subset of the transport arguments.

#### Behavior changes

- `dispatch_frame` never delivers replies to requests that do not
  expect one.
- `create_transports` / `create_async_transports` actually report
  failure error messages to callers.

### 0.5.3 - 2026-06-17

- The example bodies are exposed as a library via the `examples`
  feature, so downstream binaries can reuse them with other
  transports.
- `ArgSpec::new()` lets external transport factories declare
  command-line arguments.

### 0.5.2 - 2026-06-17

- Re-release; no library changes.

### 0.5.1 - 2026-06-17

- Support out-of-crate transports: parent-only transport devices and
  `known_can_ids` routing hints.
- Support moteus connected directly over TTL-level UART.
- Examples moved into the crate.
