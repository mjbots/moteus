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

//! This example commands multiple servos connected to a system.
//!
//! It uses the `cycle()` method to optimally use bandwidth across any
//! connected CAN-FD interfaces.
//!
//! Usage:
//!   tools/bazel run //lib/rust:multiservo
//!   tools/bazel run //lib/rust:multiservo -- --fdcanusb /dev/ttyACM0
//!   tools/bazel run //lib/rust:multiservo -- --can-chan can0
//!   tools/bazel run //lib/rust:multiservo -- --async

use clap::Parser;
use moteus::command::PositionCommand;
use moteus::query::QueryResult;
use moteus::transport::args::TransportArgs;
use moteus::transport::async_transport::AsyncTransport;
use moteus::transport::singleton::get_singleton_transport;
use moteus::transport::transaction::Request;
use moteus::Controller;
use std::time::{Duration, Instant};

/// Command multiple servos with auto-discovery.
#[derive(Parser)]
#[command(about = "Command multiple servos with auto-discovery")]
struct Args {
    /// Use async transport instead of blocking
    #[arg(long)]
    r#async: bool,

    #[command(flatten)]
    transport: TransportArgs,
}

fn run_blocking(args: &Args) -> Result<(), moteus::Error> {
    // Get the singleton transport with specified options.
    // This auto-discovers fdcanusb, socketcan, etc.
    let transport = get_singleton_transport(Some(&args.transport.clone().into()))?;
    let mut transport = transport.lock().unwrap();

    // Discover all connected devices.
    let devices = transport.discover(0, 0)?;

    if devices.is_empty() {
        println!("No devices found!");
        return Ok(());
    }

    println!("Discovered {} device(s):", devices.len());
    for device in &devices {
        println!("  CAN ID: {}", device.can_id);
    }
    println!();

    // Alternatively, hard-code a list of servos by specifying CAN IDs.
    let servo_ids: Vec<u8> = devices.iter().map(|d| d.can_id).collect();

    // Create one Controller instance for each servo.
    let servos: Vec<Controller> = servo_ids.iter().map(|&id| Controller::new(id)).collect();

    // Clear any faults by sending a stop command to all servos.
    let mut stop_requests: Vec<_> = servos
        .iter()
        .map(|s| Request::new(s.make_stop(false).into_frame()))
        .collect();
    transport.cycle(&mut stop_requests)?;

    // Track start time for sinusoidal velocity.
    let start = Instant::now();

    loop {
        // The `cycle` method accepts a list of commands, each created
        // by calling one of the `make_*` methods on Controller.

        let now = start.elapsed().as_secs_f32();

        // Construct a position command for each servo with a sinusoidal
        // velocity starting from the current position.
        let mut requests: Vec<_> = servos
            .iter()
            .enumerate()
            .map(|(i, servo)| {
                let velocity = 0.1 * (now + i as f32).sin();
                Request::new(servo.make_position_command(
                    &PositionCommand::new().position(f32::NAN).velocity(velocity),
                    true, // query
                ).into_frame())
            })
            .collect();

        // By sending all commands in one go, the library can send out
        // commands and retrieve responses simultaneously from all
        // interfaces. It can also pipeline commands and responses for
        // multiple servos on the same bus.
        transport.cycle(&mut requests)?;

        // The result is response frames in each request's collector.
        // Parse them to get QueryResult structures with access to
        // individual registers. Note: not all servos may respond.
        let output: Vec<String> = requests
            .iter()
            .flat_map(|req| req.responses.peek())
            .map(|frame| {
                let result = QueryResult::parse(&frame);
                format!(
                    "({:04X} {:.4} {:.4})",
                    frame.arbitration_id, result.position, result.velocity
                )
            })
            .collect();

        println!("{}", output.join(", "));

        // Wait 20ms between iterations. By default, when commanded
        // over CAN, there is a watchdog which requires commands to be
        // sent at least every 100ms or the controller will enter a
        // latched fault state.
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[tokio::main]
async fn run_async(args: &Args) -> Result<(), moteus::Error> {
    // Create async transport with specified options.
    let opts = args.transport.clone().into();
    let mut transport = AsyncTransport::with_options(&opts).await?;

    // Discover all connected devices.
    let devices = transport.discover(0, 0).await?;

    if devices.is_empty() {
        println!("No devices found!");
        return Ok(());
    }

    println!("Discovered {} device(s):", devices.len());
    for device in &devices {
        println!("  CAN ID: {}", device.can_id);
    }
    println!();

    // Alternatively, hard-code a list of servos by specifying CAN IDs.
    let servo_ids: Vec<u8> = devices.iter().map(|d| d.can_id).collect();

    // Create one Controller instance for each servo.
    let servos: Vec<Controller> = servo_ids.iter().map(|&id| Controller::new(id)).collect();

    // Clear any faults by sending a stop command to all servos.
    let mut stop_requests: Vec<_> = servos
        .iter()
        .map(|s| Request::new(s.make_stop(false).into_frame()))
        .collect();
    transport.cycle(&mut stop_requests).await?;

    // Track start time for sinusoidal velocity.
    let start = Instant::now();

    loop {
        // The `cycle` method accepts a list of commands, each created
        // by calling one of the `make_*` methods on Controller.

        let now = start.elapsed().as_secs_f32();

        // Construct a position command for each servo with a sinusoidal
        // velocity starting from the current position.
        let mut requests: Vec<_> = servos
            .iter()
            .enumerate()
            .map(|(i, servo)| {
                let velocity = 0.1 * (now + i as f32).sin();
                Request::new(servo.make_position_command(
                    &PositionCommand::new().position(f32::NAN).velocity(velocity),
                    true, // query
                ).into_frame())
            })
            .collect();

        // By sending all commands in one go, the library can send out
        // commands and retrieve responses simultaneously from all
        // interfaces. It can also pipeline commands and responses for
        // multiple servos on the same bus.
        transport.cycle(&mut requests).await?;

        // The result is response frames in each request's collector.
        // Parse them to get QueryResult structures with access to
        // individual registers. Note: not all servos may respond.
        let output: Vec<String> = requests
            .iter()
            .flat_map(|req| req.responses.peek())
            .map(|frame| {
                let result = QueryResult::parse(&frame);
                format!(
                    "({:04X} {:.4} {:.4})",
                    frame.arbitration_id, result.position, result.velocity
                )
            })
            .collect();

        println!("{}", output.join(", "));

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
