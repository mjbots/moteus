# Rust API Reference

The moteus Rust library is split into two crates. Full auto-generated documentation is available on docs.rs.

- [**moteus** on docs.rs](https://docs.rs/moteus/) — High-level controller API
- [**moteus-protocol** on docs.rs](https://docs.rs/moteus-protocol/) — Low-level protocol types (`no_std` compatible)

This page provides a curated summary of the key types and their roles.

## moteus crate

### Controllers

[`BlockingController`](https://docs.rs/moteus/latest/moteus/struct.BlockingController.html)
:   Synchronous controller with auto-discovered transport.
    Simplest API for single-threaded use.
    Provides `set_position()`, `set_stop()` and other blocking methods that send a command and return a `QueryResult`.

[`AsyncController`](https://docs.rs/moteus/latest/moteus/struct.AsyncController.html)
:   Async controller for use with `async`/`.await`.
    Supports both wrapped-blocking and true async (tokio feature) transports.
    Same method set as `BlockingController`, but all methods return futures.

[`Controller`](https://docs.rs/moteus/latest/moteus/struct.Controller.html)
:   Low-level frame builder.
    Produces `Command` values without any transport — use for custom transports or embedded systems.
    Provides `make_position_command()`, `make_stop()`, `parse_query()`, etc.

### Commands and Results

[`Command`](https://docs.rs/moteus/latest/moteus/struct.Command.html)
:   A routed moteus message combining destination/source/prefix with serialized multiplex protocol data.
    Produced by `Controller::make_*()` methods.
    Convert to a wire-level `CanFdFrame` via `into_frame()`.

[`command::PositionCommand`](https://docs.rs/moteus/latest/moteus/command/struct.PositionCommand.html)
:   Position mode parameters: position, velocity, feedforward torque, torque limits, PID scales, acceleration/velocity limits, and more.
    Uses a builder pattern — chain `.position()`, `.velocity()`, `.maximum_torque()`, etc.

[`command::StopCommand`](https://docs.rs/moteus/latest/moteus/command/struct.StopCommand.html)
:   Stop command that clears faults and sets the controller to stopped mode.

[`command::CurrentCommand`](https://docs.rs/moteus/latest/moteus/command/struct.CurrentCommand.html)
:   Direct current (d/q axis) control command.

[`command::VFOCCommand`](https://docs.rs/moteus/latest/moteus/command/struct.VFOCCommand.html)
:   Voltage FOC (field-oriented control) command.

[`command::StayWithinCommand`](https://docs.rs/moteus/latest/moteus/command/struct.StayWithinCommand.html)
:   Stay-within bounds command — the controller maintains position within specified limits.

[`command::ZeroVelocityCommand`](https://docs.rs/moteus/latest/moteus/command/struct.ZeroVelocityCommand.html)
:   Zero velocity mode command — the controller actively holds zero velocity.

[`query::QueryFormat`](https://docs.rs/moteus/latest/moteus/query/struct.QueryFormat.html)
:   Specifies which registers to query and at what resolution.
    Customize to request additional telemetry fields beyond the defaults (mode, position, velocity, torque).

[`query::QueryResult`](https://docs.rs/moteus/latest/moteus/query/struct.QueryResult.html)
:   Parsed response containing mode, position, velocity, torque, voltage, temperature, fault code, and other telemetry fields.

### Transport Layer

[`Transport`](https://docs.rs/moteus/latest/moteus/struct.Transport.html)
:   Manages communication channels to moteus controllers.
    Provides `cycle()` (batch send/receive), `write()` (fire-and-forget), `read()` (receive unsolicited), and `flush_read()`, matching the Python API.

[`TransportOptions`](https://docs.rs/moteus/latest/moteus/struct.TransportOptions.html)
:   Configuration for transport auto-discovery.
    Set preferred interfaces, timeouts, and other options.

[`TransportOps`](https://docs.rs/moteus/latest/moteus/trait.TransportOps.html)
:   Trait for blocking transport implementations.
    Implement this to create custom transport backends.

[`AsyncTransport`](https://docs.rs/moteus/latest/moteus/struct.AsyncTransport.html) *(requires `tokio` feature)*
:   Async transport wrapper for true non-blocking I/O with tokio.

[`AsyncTransportOptions`](https://docs.rs/moteus/latest/moteus/struct.AsyncTransportOptions.html) *(requires `tokio` feature)*
:   Configuration for async transport creation.

[`AsyncTransportOps`](https://docs.rs/moteus/latest/moteus/trait.AsyncTransportOps.html) *(requires `tokio` feature)*
:   Trait for async transport implementations.

### Transport Backends

[`transport::fdcanusb::Fdcanusb`](https://docs.rs/moteus/latest/moteus/transport/fdcanusb/struct.Fdcanusb.html)
:   FdCanUSB serial (CDC) transport.
    Auto-detected when an fdcanusb device is connected via USB.

[`transport::socketcan::SocketCan`](https://docs.rs/moteus/latest/moteus/transport/socketcan/struct.SocketCan.html)
:   Linux SocketCAN transport.
    Uses the kernel CAN interface (e.g., `can0`).

[`transport::async_fdcanusb::AsyncFdcanusb`](https://docs.rs/moteus/latest/moteus/transport/async_fdcanusb/struct.AsyncFdcanusb.html) *(requires `tokio` feature)*
:   Async FdCanUSB transport using tokio-serial.

[`transport::async_socketcan::AsyncSocketCan`](https://docs.rs/moteus/latest/moteus/transport/async_socketcan/struct.AsyncSocketCan.html) *(requires `tokio` feature)*
:   Async SocketCAN transport using tokio.

### Diagnostics and Utilities

[`DiagnosticStream`](https://docs.rs/moteus/latest/moteus/struct.DiagnosticStream.html)
:   Blocking diagnostic protocol stream for reading and writing configuration values, firmware info, and other diagnostic commands.

[`AsyncDiagnosticStream`](https://docs.rs/moteus/latest/moteus/struct.AsyncDiagnosticStream.html) *(requires `tokio` feature)*
:   Async variant of the diagnostic stream.

[`move_to()`](https://docs.rs/moteus/latest/moteus/move_to/fn.move_to.html)
:   Free function for coordinated multi-servo moves.
    Moves multiple servos to target positions simultaneously, waiting for all to arrive.

[`async_move_to()`](https://docs.rs/moteus/latest/moteus/move_to/fn.async_move_to.html) *(requires `tokio` feature)*
:   Async variant of coordinated multi-servo moves.

[`Error`](https://docs.rs/moteus/latest/moteus/enum.Error.html)
:   Error enum with variants: `Io`, `Timeout`, `NoResponse`, `Fault`, `InvalidResponse`, `NotConnected`, `DeviceNotFound`, `Protocol`.
    Implements `std::error::Error` and `From<std::io::Error>`.

### Factory and Discovery

[`get_singleton_transport()`](https://docs.rs/moteus/latest/moteus/fn.get_singleton_transport.html)
:   Returns a shared global transport instance.
    Creates the transport on first call using auto-discovery.

[`create_default_transport()`](https://docs.rs/moteus/latest/moteus/fn.create_default_transport.html)
:   Creates a new transport with default auto-discovery settings.

[`register()`](https://docs.rs/moteus/latest/moteus/fn.register.html)
:   Register a custom transport factory for auto-discovery.
    Allows third-party transport backends to participate in automatic transport selection.

[`DeviceAddress`](https://docs.rs/moteus/latest/moteus/struct.DeviceAddress.html)
:   Address a device by CAN ID, UUID, or both.
    UUID-based addressing enables automatic ID resolution on the bus.

## moteus-protocol crate

This crate is `no_std` compatible (with optional `std` feature) and provides the wire protocol types:

[`CanFdFrame`](https://docs.rs/moteus-protocol/latest/moteus_protocol/struct.CanFdFrame.html)
:   Raw CAN-FD frame with `arbitration_id`, `data`, `size`, `brs`, and `fdcan` fields.
    Contains no routing information — routing is handled by `Command` in the moteus crate.

[`Register`](https://docs.rs/moteus-protocol/latest/moteus_protocol/enum.Register.html)
:   Enum of all moteus register addresses (mode, position, velocity, torque, voltage, temperature, fault, encoder registers, etc.).

[`Resolution`](https://docs.rs/moteus-protocol/latest/moteus_protocol/enum.Resolution.html)
:   Register value resolution: `Ignore`, `Int8`, `Int16`, `Int32`, `Float`.
    Controls the precision and wire size of register values.

[`Mode`](https://docs.rs/moteus-protocol/latest/moteus_protocol/enum.Mode.html)
:   Controller operating mode enum: `Stopped`, `Fault`, `Position`, `ZeroVelocity`, `StayWithin`, `Current`, etc.

[`HomeState`](https://docs.rs/moteus-protocol/latest/moteus_protocol/enum.HomeState.html)
:   Homing state enum: `Relative`, `Rotor`, `Output`.

[`Scaling`](https://docs.rs/moteus-protocol/latest/moteus_protocol/struct.Scaling.html)
:   Register scaling helpers for encoding/decoding fixed-point values to/from wire format.

[`command::*`](https://docs.rs/moteus-protocol/latest/moteus_protocol/command/index.html)
:   Command structs and serialization for all command types.
    Each command type has a corresponding format struct that controls which fields are sent and at what resolution.

[`query::*`](https://docs.rs/moteus-protocol/latest/moteus_protocol/query/index.html)
:   Query format and result types with deserialization.
    `QueryFormat` controls which registers are requested; `QueryResult` holds the parsed response.

[`calculate_arbitration_id()`](https://docs.rs/moteus-protocol/latest/moteus_protocol/fn.calculate_arbitration_id.html)
:   Compute a CAN arbitration ID from source, destination, prefix, and reply flag.

[`parse_arbitration_id()`](https://docs.rs/moteus-protocol/latest/moteus_protocol/fn.parse_arbitration_id.html)
:   Parse source, destination, prefix, and reply flag from a CAN arbitration ID.

## Feature Flags

`tokio`
:   Enables true async transports using tokio.
    Adds `AsyncFdcanusb`, `AsyncSocketCan`, `AsyncTransport`, `AsyncDiagnosticStream`, and `async_move_to()`.

`clap`
:   Enables `add_transport_args()` for CLI argument parsing with clap.

## Integration Guide

For usage examples and tutorials, see the [Rust Client integration guide](../integration/rust.md).
