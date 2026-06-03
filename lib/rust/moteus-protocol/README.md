# moteus-protocol

Low-level CAN-FD protocol types for moteus brushless motor controllers.

This crate encodes and decodes the CAN-FD frames used to communicate with
moteus controllers. It performs no I/O of its own: you bring the CAN-FD
transport, and this crate builds the frames you send and parses the frames
you receive.

It is `no_std` compatible and requires no allocator, so it is usable on
embedded systems as well as in standard environments.

Most applications should use the higher-level
[`moteus`](https://crates.io/crates/moteus) crate instead, which builds on
this one to add blocking and async controllers, transport implementations
(fdcanusb, SocketCAN), and device discovery. Reach for `moteus-protocol`
directly when you are on an embedded target or have your own CAN-FD
transport.

## Encoding a Command

Commands use a builder pattern, and serialize into a `CanFdFrame`:

```rust
use moteus_protocol::{calculate_arbitration_id, CanFdFrame};
use moteus_protocol::command::{PositionCommand, PositionFormat};

// Address servo ID 1 from source ID 0, requesting a reply.
let mut frame = CanFdFrame::new();
frame.arbitration_id = calculate_arbitration_id(0, 1, 0, true);

let cmd = PositionCommand::new()
    .position(0.5)   // revolutions
    .velocity(1.0);  // revolutions / s
cmd.serialize(&mut frame, &PositionFormat::default());

// frame.data and frame.size now contain the encoded command, ready
// to hand to any CAN-FD transport.
```

## Requesting Telemetry

A query describes which registers the controller should report, and at
what resolution. It can be appended to the same frame as a command, or
sent on its own:

```rust
use moteus_protocol::{CanFdFrame, Resolution};
use moteus_protocol::query::QueryFormat;

let mut frame = CanFdFrame::new();

let mut query = QueryFormat::default();
query.position = Resolution::Float;  // full precision
query.velocity = Resolution::Float;

let expected_reply_size = query.serialize(&mut frame);
```

## Parsing a Reply

```rust
use moteus_protocol::{CanFdFrame, Mode};
use moteus_protocol::query::QueryResult;

// A reply frame as received from the transport.  This one reports the
// mode register as an int8 and the position register as a float.
let mut reply = CanFdFrame::new();
reply.data[..9].copy_from_slice(&[
    0x21, 0x00, 0x0A, // reply int8, register 0x000: Mode = 10 (position)
    0x2D, 0x01, // reply f32, register 0x001: Position
    0x00, 0x00, 0x00, 0x3F, // 0.5f32, little endian
]);
reply.size = 9;

let result = QueryResult::parse(&reply);
assert_eq!(result.mode, Mode::Position);
assert_eq!(result.position, 0.5);
```

## Key Types

- `CanFdFrame`: a raw CAN-FD frame (arbitration ID, payload, flags),
  independent of any particular transport.
- `command`: builder-style command types such as `PositionCommand`,
  `CurrentCommand`, `VFOCCommand`, `StayWithinCommand`, `StopCommand`,
  and `BrakeCommand`, with matching `*Format` resolution descriptions.
- `query::QueryFormat` / `query::QueryResult`: telemetry requests and
  replies.
- `Register`, `Mode`, `Resolution`: the moteus register map, operating
  modes, and wire resolutions.
- `WriteCanData`, `WriteCombiner`, `parse_frame`: multiplex primitives
  for reading and writing arbitrary registers.
- `calculate_arbitration_id` / `parse_arbitration_id`: CAN ID routing
  helpers.

## Building with Bazel

When building from the [moteus repository](https://github.com/mjbots/moteus):

```bash
tools/bazel build //lib/rust/moteus-protocol
```
