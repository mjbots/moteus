# Changelog

This file tracks notable changes per shipped component. Each component is
released independently; see [RELEASING.md](RELEASING.md) for the process.

The format is loosely based on [Keep a Changelog](https://keepachangelog.com/).

## firmware

### Unreleased

(no entries yet)

### 1.0.0 - YYYY-MM-DD

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
