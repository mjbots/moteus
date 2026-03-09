# Rust Client Library

moteus provides a Rust client library for commanding and controlling moteus controllers using a supported CAN-FD adapter. The library consists of two crates:

- **`moteus`** — High-level controller API with transport management
- **`moteus-protocol`** — Low-level `no_std` protocol encoding/decoding

## Installation

Add the moteus crate to your project:

```
cargo add moteus
```

Or add it directly to your `Cargo.toml`:

```toml
[dependencies]
moteus = "0.1"
```

For async support with tokio, enable the `tokio` feature:

```toml
[dependencies]
moteus = { version = "0.1", features = ["tokio"] }
```

For CLI argument parsing support, enable the `clap` feature:

```toml
[dependencies]
moteus = { version = "0.1", features = ["clap"] }
```

## Basic Usage — BlockingController

The simplest way to use moteus is with `BlockingController`, which provides synchronous, blocking communication:

```rust
use std::thread;
use std::time::Duration;
use moteus::{BlockingController, command::PositionCommand};

fn main() -> Result<(), moteus::Error> {
    // Auto-discovers transport (fdcanusb, socketcan, etc.)
    let mut ctrl = BlockingController::new(1)?;

    // Clear any faults
    ctrl.set_stop()?;

    // Send commands at regular intervals to prevent watchdog timeout
    loop {
        let result = ctrl.set_position(
            PositionCommand::new()
                .position(f64::NAN)
                .velocity(1.0)
                .accel_limit(0.5)
        )?;

        println!("Position: {} Velocity: {}", result.position, result.velocity);

        thread::sleep(Duration::from_millis(10));
    }
}
```

The `BlockingController` automatically discovers an available transport (fdcanusb, socketcan) on construction.

## Async Usage — AsyncController

For async applications, enable the `tokio` feature and use `AsyncController`:

```rust
use moteus::AsyncController;
use moteus::command::PositionCommand;

#[tokio::main]
async fn main() -> Result<(), moteus::Error> {
    // Auto-discovers transport (fdcanusb, socketcan, etc.)
    let mut ctrl = AsyncController::new(1).await?;

    ctrl.set_stop().await?;

    loop {
        let result = ctrl.set_position(
            PositionCommand::new()
                .position(f64::NAN)
                .velocity(0.5)
        ).await?;

        println!("Position: {}", result.position);

        tokio::time::sleep(std::time::Duration::from_millis(10)).await;
    }
}
```

## Low-Level Usage — Controller

The `Controller` type provides frame building without any transport. This is useful for custom transports, embedded systems, or when you want full control over how frames are sent and received.

```rust
use moteus::{Controller, command::PositionCommand};

let controller = Controller::new(1);

// Build a position command
let cmd = controller.make_position_command(
    &PositionCommand::new().position(0.5).velocity(1.0),
    true  // request query response
);

// Convert to wire frame
let frame = cmd.into_frame();

// Send frame via your own transport...
// frame.arbitration_id, frame.data, frame.size
```

Parse responses back into query results:

```rust
use moteus::query::QueryResult;

// After receiving a response frame from transport...
let result = controller.parse_query(&response_frame)?;
println!("Mode: {:?}, Position: {}", result.mode, result.position);
```

## Custom Query Resolution

By default, only a limited set of registers are queried. To request additional registers, customize the `QueryFormat`:

```rust
use moteus::{BlockingController, Resolution, Register};
use moteus::query::QueryFormat;

let mut query_format = QueryFormat::default();
query_format.power = Resolution::Float;

let mut ctrl = BlockingController::new(1)?;
ctrl.controller.query_format = query_format;
```

For registers not in the standard `QueryFormat`, use the `extra` mechanism:

```rust
use moteus::{BlockingController, Resolution, Register};
use moteus::query::{QueryFormat, ExtraQuery};

let mut query_format = QueryFormat::default();
query_format.extra[0] = ExtraQuery {
    register: Register::Encoder1Position,
    resolution: Resolution::Float,
};

let mut ctrl = BlockingController::new(1)?;
ctrl.controller.query_format = query_format;
```

## Transport Configuration

### Auto-Discovery Options

Configure transport auto-detection with `TransportOptions`:

```rust
use moteus::{BlockingController, TransportOptions};

let opts = TransportOptions::new()
    .socketcan_interfaces(vec!["can0"])
    .timeout_ms(200);

let mut ctrl = BlockingController::with_options(1, &opts)?;
```

### Explicit Transport

You can also specify a transport directly:

```rust
use moteus::BlockingController;
use moteus::transport::socketcan::SocketCan;

let transport = SocketCan::new("can0")?;
let mut ctrl = BlockingController::new(1)?
    .transport(transport);
```

## Diagnostics

The `DiagnosticStream` provides access to the moteus diagnostic protocol for reading and writing configuration:

```rust
use moteus::DiagnosticStream;

let mut stream = DiagnosticStream::new(1)?;

// Read a configuration value
let response = stream.command("conf get servo.max_current_A")?;
println!("{}", response);

// Write a configuration value
stream.command("conf set servo.max_current_A 30.0")?;
```

## Multi-Servo Coordinated Moves

The `move_to` function provides coordinated multi-servo motion:

```rust
use moteus::move_to::{move_to, MoveToOptions, Setpoint};

let setpoints = vec![
    Setpoint { id: 1, position: 0.5, velocity: None, max_torque: None },
    Setpoint { id: 2, position: -0.3, velocity: None, max_torque: None },
];

let opts = MoveToOptions::default();
move_to(&setpoints, &opts)?;
```

## API Reference

- [Rust API Reference](../reference/rust.md) — Curated summary of key types
- [moteus on docs.rs](https://docs.rs/moteus/) — Full auto-generated API documentation
- [moteus-protocol on docs.rs](https://docs.rs/moteus-protocol/) — Protocol crate documentation
