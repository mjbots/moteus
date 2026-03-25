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

//! This example commands a single servo to hold the current position
//! indefinitely, and prints the state of the servo to the console.
//!
//! Usage:
//!   tools/bazel run //lib/rust:simple
//!   tools/bazel run //lib/rust:simple -- --fdcanusb /dev/ttyACM0
//!   tools/bazel run //lib/rust:simple -- --id 1
//!   tools/bazel run //lib/rust:simple -- --async

use clap::Parser;
use moteus::command::PositionCommand;
use moteus::transport::args::TransportArgs;
use moteus::{AsyncController, BlockingController};
use std::time::Duration;

/// Command a single servo to hold position.
#[derive(Parser)]
#[command(about = "Command a single servo to hold position")]
struct Args {
    /// Servo ID
    #[arg(long, default_value = "1")]
    id: u8,

    /// Use async transport instead of blocking
    #[arg(long)]
    r#async: bool,

    #[command(flatten)]
    transport: TransportArgs,
}

fn run_blocking(args: &Args) -> Result<(), moteus::Error> {
    // Create a controller with the specified ID and transport options.
    // By default, this picks an arbitrary CAN-FD transport, preferring
    // an attached fdcanusb if available.
    let mut c = BlockingController::with_options(args.id, &args.transport.clone().into());

    // Clear any faults by sending a stop command.
    c.set_stop()?;

    loop {
        // `set_position` accepts a PositionCommand built with the builder
        // pattern. If a given field is omitted, then that register is
        // omitted from the command itself, with semantics as described
        // in the reference manual.
        //
        // The return type of `set_position` is a QueryResult type which
        // contains the current state of the servo.
        let state = c.set_position(PositionCommand::new().position(f32::NAN))?;

        // Print out everything.
        println!("{:?}", state);

        // Print out just the position register.
        println!("Position: {}", state.position);

        // And a blank line so we can separate one iteration from the next.
        println!();

        // Wait 20ms between iterations. By default, when commanded
        // over CAN, there is a watchdog which requires commands to be
        // sent at least every 100ms or the controller will enter a
        // latched fault state.
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[tokio::main]
async fn run_async(args: &Args) -> Result<(), moteus::Error> {
    // Create a controller with the specified ID and transport options.
    // By default, this picks an arbitrary CAN-FD transport, preferring
    // an attached fdcanusb if available.
    let mut c =
        AsyncController::with_options(args.id, &args.transport.clone().into());

    // Clear any faults by sending a stop command.
    c.set_stop().await?;

    loop {
        // `set_position` accepts a PositionCommand built with the builder
        // pattern. If a given field is omitted, then that register is
        // omitted from the command itself, with semantics as described
        // in the reference manual.
        //
        // The return type of `set_position` is a QueryResult type which
        // contains the current state of the servo.
        let state = c.set_position(PositionCommand::new().position(f32::NAN)).await?;

        // Print out everything.
        println!("{:?}", state);

        // Print out just the position register.
        println!("Position: {}", state.position);

        // And a blank line so we can separate one iteration from the next.
        println!();

        // Wait 20ms between iterations. By default, when commanded
        // over CAN, there is a watchdog which requires commands to be
        // sent at least every 100ms or the controller will enter a
        // latched fault state.
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

fn main() -> Result<(), moteus::Error> {
    let args = Args::parse();

    if args.r#async {
        run_async(&args)
    } else {
        run_blocking(&args)
    }
}
