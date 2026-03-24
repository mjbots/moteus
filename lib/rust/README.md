# Rust bindings for moteus brushless controller

These bindings permit communication and control of moteus brushless
controllers from Rust.

## Crate Structure

This library is split into two crates:

- **moteus-protocol**: Low-level protocol encoding/decoding. `no_std`
  compatible for embedded use.
- **moteus**: High-level API with transport implementations for FdCanUSB
  and SocketCAN.

## Minimum Supported Rust Version (MSRV)

This library requires **Rust 1.75.0** or later.

## Quick Start - Auto-Discovery (Recommended)

The simplest way to use moteus is with automatic transport discovery:

```rust,no_run
use moteus::{BlockingController, command::PositionCommand};

fn main() -> Result<(), moteus::Error> {
    // Auto-discovers transport (fdcanusb, socketcan, etc.)
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
use moteus::{BlockingController, transport::socketcan::SocketCan};

fn main() -> Result<(), moteus::Error> {
    let transport = SocketCan::new("can0")?;
    let mut ctrl = BlockingController::new(1)
        .transport(transport);

    // Use the controller...
    ctrl.set_stop()?;
    Ok(())
}
```

## Transport Options

Configure transport auto-detection with options:

```rust,no_run
use moteus::{BlockingController, TransportOptions};

fn main() -> Result<(), moteus::Error> {
    let opts = TransportOptions::new()
        .socketcan_interfaces(vec!["can0"])
        .timeout(std::time::Duration::from_millis(200));

    let mut ctrl = BlockingController::with_options(1, &opts);
    ctrl.set_stop()?;
    Ok(())
}
```

## Using moteus-protocol Directly

For embedded or custom transport use cases, the `moteus-protocol` crate
can be used directly to construct and parse CAN-FD frames:

```rust
use moteus_protocol::{CanFdFrame, command::{PositionCommand, PositionFormat}};

// Create a position command frame using builder pattern
let mut frame = CanFdFrame::new();
frame.arbitration_id = 0x8001;  // dest=1, source=0
frame.destination = 1;

let cmd = PositionCommand::new()
    .position(0.5)
    .velocity(1.0);
cmd.serialize(&mut frame, &PositionFormat::default());

// frame.data and frame.size now contain the encoded command
```

## Transport Options

### SocketCAN (Linux)

```rust,no_run
use moteus::transport::socketcan::SocketCan;

let transport = SocketCan::new("can0")?;
// or with custom timeout:
let transport = SocketCan::with_timeout("can0", 200)?;
```

### FdCanUSB

The FdCanUSB transport requires a serial port implementation. The
`fdcanusb` module provides the protocol encoder/decoder:

```rust
use moteus::transport::fdcanusb::FdcanusbProtocol;
use moteus_protocol::CanFdFrame;

let frame = CanFdFrame::new();
let cmd = FdcanusbProtocol::encode_frame(&frame, false);
// Send `cmd` over serial port

// When receiving:
let line = "rcv 8001 0102030405060708 BF\n";
if let Some(frame) = FdcanusbProtocol::parse_line(line) {
    // Process received frame
}
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

## Building with Bazel

```bash
tools/bazel build //lib/rust/moteus
tools/bazel test //lib/rust:host
```

### Testing Against MSRV

To verify compatibility with the minimum supported Rust version (1.75.0):

```bash
tools/bazel test //lib/rust:host --extra_toolchains=@rust_msrv//:all
```
