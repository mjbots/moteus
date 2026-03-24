# moteus

Rust client library for moteus brushless motor controllers.

This crate provides a high-level API for communicating with moteus
controllers over CAN-FD. It supports multiple transport backends and
automatic device discovery.

## Crate Structure

This library is split into two crates:

- **moteus-protocol**: Low-level protocol encoding/decoding. `no_std`
  compatible for embedded use.
- **moteus**: High-level API with transport implementations for FdCanUSB
  and SocketCAN.

## Quick Start - Auto-Discovery (Recommended)

The simplest way to use moteus is with automatic transport discovery:

```rust,no_run
use moteus::{BlockingController, command::PositionCommand};

fn main() -> Result<(), moteus::Error> {
    // Auto-discovers transport on first use
    let mut ctrl = BlockingController::new(1);

    // Clear any faults
    ctrl.set_stop()?;

    // Command position mode
    loop {
        let result = ctrl.set_position(PositionCommand::new().position(f32::NAN))?;
        println!("Position: {}", result.position);
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
}
```

## Builder Pattern API

Commands are built using the builder pattern for clear, self-documenting code:

```rust
use moteus::command::PositionCommand;

// Simple position command
let cmd = PositionCommand::new().position(0.5);

// Position with velocity
let cmd = PositionCommand::new()
    .position(0.5)
    .velocity(1.0);

// Full control over all parameters
let cmd = PositionCommand::new()
    .position(0.5)
    .velocity(1.0)
    .feedforward_torque(0.1)
    .kp_scale(0.8)
    .kd_scale(1.0)
    .maximum_torque(2.0);
```

## Explicit Transport

For more control, you can specify a transport explicitly:

```rust,no_run
use moteus::{BlockingController, TransportOptions};

fn main() -> Result<(), moteus::Error> {
    let opts = TransportOptions::new()
        .socketcan_interfaces(vec!["can0"]);
    let mut ctrl = BlockingController::with_options(1, &opts);

    ctrl.set_stop()?;
    Ok(())
}
```

## Transport Options

Configure transport auto-detection with options:

```rust
use moteus::{BlockingController, TransportOptions};
use std::time::Duration;

let opts = TransportOptions::new()
    .socketcan_interfaces(vec!["can0"])
    .timeout(Duration::from_millis(200));

let mut ctrl = BlockingController::with_options(1, &opts);
```

## Low-Level Protocol Access

For embedded or custom transport use cases, the `moteus-protocol` crate
can be used directly to construct and parse CAN-FD frames:

```rust
use moteus_protocol::{CanFdFrame, command::{PositionCommand, PositionFormat}};

// Create a position command frame using builder pattern
let mut frame = CanFdFrame::new();
frame.arbitration_id = 0x8001;  // dest=1, source=0

let cmd = PositionCommand::new()
    .position(0.5)
    .velocity(1.0);
cmd.serialize(&mut frame, &PositionFormat::default());

// frame.data and frame.size now contain the encoded command
```

## Controller Configuration

Use the builder pattern to configure controller settings:

```rust
use moteus::{Controller, BlockingController, Resolution};
use moteus::query::QueryFormat;

// Configure a controller with custom settings
let ctrl = Controller::new(1)
    .source_id(0x10)
    .query_format(QueryFormat::comprehensive());

// Use with auto-discovered transport
let ctrl = BlockingController::with_controller(ctrl);
```

## Query Format Overrides

You can override the query format on a per-call basis:

```rust
use moteus::{CommandExt, command::PositionCommand};
use moteus::query::QueryFormat;

// Override query format for this specific call
let cmd = PositionCommand::new()
    .position(0.5)
    .with_query(QueryFormat::comprehensive());
```

## Resolution Configuration

Control the resolution of commands and queries to optimize bandwidth:

```rust
use moteus::{Controller, Resolution};
use moteus_protocol::query::QueryFormat;

let mut ctrl = Controller::new(1);

// Customize query resolution
ctrl.query_format.position = Resolution::Float;
ctrl.query_format.velocity = Resolution::Float;
ctrl.query_format.torque = Resolution::Int16;
```

## Features

- `tokio`: Async I/O transports and `AsyncController` (optional)
- `clap`: CLI argument parsing support (optional)

## Supported Transports

- **FdCanUSB**: USB-to-CAN adapter (serial CDC)
- **SocketCAN**: Linux kernel CAN interface

## Building with Bazel

```bash
tools/bazel build //lib/rust/moteus
tools/bazel test //lib/rust:host
```
