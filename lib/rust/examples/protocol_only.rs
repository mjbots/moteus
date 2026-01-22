// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! This example demonstrates how to use the low-level moteus-protocol
//! crate to construct and parse CAN-FD frames without any transport.
//!
//! This is useful for embedded systems or custom transport implementations.
//!
//! Usage:
//!   tools/bazel run //lib/rust:protocol_only

use moteus_protocol::command::{
    BrakeCommand, CurrentCommand, CurrentFormat, PositionCommand, PositionFormat, StopCommand,
};
use moteus_protocol::query::{QueryFormat, QueryResult};
use moteus_protocol::{calculate_arbitration_id, parse_arbitration_id, CanFdFrame, Resolution};

fn main() {
    println!("=== moteus-protocol Example ===\n");

    // Example 1: Create a stop command
    println!("1. Creating a stop command for servo ID 1:");
    let mut stop_frame = CanFdFrame::new();
    stop_frame.arbitration_id = calculate_arbitration_id(0, 1, 0, true);
    StopCommand::serialize(&mut stop_frame);
    print_frame(&stop_frame);

    // Example 2: Create a position command using builder pattern
    println!("\n2. Creating a position command:");
    let mut pos_frame = CanFdFrame::new();
    pos_frame.arbitration_id = calculate_arbitration_id(0, 1, 0, true);

    let pos_cmd = PositionCommand::new()
        .position(0.5) // 0.5 revolutions
        .velocity(1.0) // 1.0 rev/s
        .kp_scale(1.0)
        .kd_scale(1.0);
    let pos_format = PositionFormat::default();

    pos_cmd.serialize(&mut pos_frame, &pos_format);
    print_frame(&pos_frame);

    // Example 3: Create a query-only command
    println!("\n3. Creating a query-only command:");
    let mut query_frame = CanFdFrame::new();
    query_frame.arbitration_id = calculate_arbitration_id(0, 1, 0, true);

    let mut custom_query = QueryFormat::default();
    custom_query.position = Resolution::Float; // Request full precision
    custom_query.velocity = Resolution::Float;
    custom_query.torque = Resolution::Float;

    let reply_size = custom_query.serialize(&mut query_frame);
    print_frame(&query_frame);
    println!("  Expected reply size: {} bytes", reply_size);

    // Example 4: Parse a simulated response
    println!("\n4. Parsing a simulated response:");

    // Construct a fake response frame as if from servo ID 1
    let mut response = CanFdFrame::new();
    response.arbitration_id = calculate_arbitration_id(1, 0, 0, false);

    // Manually construct response data:
    // This would normally come from the CAN bus
    // Format: [reply header] [register] [values...]
    response.data[0] = 0x21; // Reply Int8, count=1
    response.data[1] = 0x00; // Register 0 (Mode)
    response.data[2] = 0x0A; // Mode = Position (10)
    response.data[3] = 0x2D; // Reply Float (0x2c), count=1 (0x01)
    response.data[4] = 0x01; // Register 1 (Position)
    response.data[5] = 0x00; // Position as f32 (0.5)
    response.data[6] = 0x00;
    response.data[7] = 0x00;
    response.data[8] = 0x3F;
    response.size = 9;

    let result = QueryResult::parse(&response);
    println!("  Parsed QueryResult:");
    println!("    Mode: {:?}", result.mode);
    println!("    Position: {:.4} rev", result.position);
    println!("    Velocity: {:.4} rev/s", result.velocity);
    println!("    Torque: {:.4} Nm", result.torque);

    // Example 5: Create a current (torque) command using builder pattern
    println!("\n5. Creating a current command:");
    let mut current_frame = CanFdFrame::new();
    current_frame.arbitration_id = calculate_arbitration_id(0, 1, 0, true);

    let current_cmd = CurrentCommand::new().d_current(0.0).q_current(0.5);
    current_cmd.serialize(&mut current_frame, &CurrentFormat::default());
    print_frame(&current_frame);

    // Example 6: Create a brake command
    println!("\n6. Creating a brake command:");
    let mut brake_frame = CanFdFrame::new();
    brake_frame.arbitration_id = calculate_arbitration_id(0, 1, 0, true);

    BrakeCommand::serialize(&mut brake_frame);
    print_frame(&brake_frame);
    println!("  Mode byte: {} (Brake=15)", brake_frame.data[2]);

    println!("\n=== Done ===");
}

fn print_frame(frame: &CanFdFrame) {
    let (source, destination, _prefix) = parse_arbitration_id(frame.arbitration_id);
    let reply_required = frame.arbitration_id & 0x8000 != 0;
    print!("  Arbitration ID: 0x{:04X}", frame.arbitration_id);
    println!(" (dest={}, source={})", destination, source);
    print!("  Data ({} bytes): ", frame.size);
    for i in 0..frame.size as usize {
        print!("{:02X} ", frame.data[i]);
    }
    println!();
    println!("  Reply required: {}", reply_required);
}
