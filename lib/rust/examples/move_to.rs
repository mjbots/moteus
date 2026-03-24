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

//! This example demonstrates using `move_to` to move multiple servos
//! to target positions and wait for completion.
//!
//! Two servos alternate between positions with a configurable period:
//!   - Servo 1: alternates between 0 and 1 revolution
//!   - Servo 2: alternates between 0 and 3 revolutions
//!
//! Usage:
//!   tools/bazel run //lib/rust:move_to
//!   tools/bazel run //lib/rust:move_to -- --fdcanusb /dev/ttyACM0
//!   tools/bazel run //lib/rust:move_to -- --id1 1 --id2 2
//!   tools/bazel run //lib/rust:move_to -- --period 3.0
//!   tools/bazel run //lib/rust:move_to -- --async

use clap::Parser;
use moteus::transport::async_transport::AsyncTransport;
use moteus::{
    async_move_to, get_singleton_transport, move_to, Controller, MoveToOptions, Request,
    TransportArgs,
};

/// Move multiple servos to target positions with coordinated timing.
#[derive(Parser)]
#[command(about = "Move multiple servos to target positions with coordinated timing")]
struct Args {
    /// ID of first servo
    #[arg(long, default_value = "1")]
    id1: u8,

    /// ID of second servo
    #[arg(long, default_value = "2")]
    id2: u8,

    /// Period in seconds for full cycle
    #[arg(long, default_value = "5.0")]
    period: f32,

    /// Use async transport instead of blocking
    #[arg(long)]
    r#async: bool,

    #[command(flatten)]
    transport: TransportArgs,
}

fn run_blocking(args: &Args) -> Result<(), moteus::Error> {
    let transport = get_singleton_transport(Some(&args.transport.clone().into()))?;
    let mut transport = transport.lock().unwrap();

    let c1 = Controller::new(args.id1);
    let c2 = Controller::new(args.id2);

    // Clear any faults by sending stop commands.
    let mut stop_requests = vec![
        Request::new(c1.make_stop(false).into_frame()),
        Request::new(c2.make_stop(false).into_frame()),
    ];
    transport.cycle(&mut stop_requests)?;

    let move_duration = args.period / 2.0;

    println!("Servo {}: alternating between 0 and 1 rev", args.id1);
    println!("Servo {}: alternating between 0 and 3 rev", args.id2);
    println!(
        "Period: {}s (each move: {}s)",
        args.period, move_duration
    );
    println!("Press Ctrl+C to stop");
    println!();

    let opts = MoveToOptions::new().duration(move_duration);

    let mut cycle = 0u64;
    loop {
        // Move to position A.
        println!("Cycle {}: Moving to position A (c1=0.0, c2=0.0)", cycle);
        let results = move_to(
            &mut *transport,
            &[(&c1, 0.0.into()), (&c2, 0.0.into())],
            &opts,
        )?;
        for r in &results {
            println!("  Servo {} arrived at {:.3}", r.id, r.result.position);
        }

        // Move to position B.
        println!("Cycle {}: Moving to position B (c1=1.0, c2=3.0)", cycle);
        let results = move_to(
            &mut *transport,
            &[(&c1, 1.0.into()), (&c2, 3.0.into())],
            &opts,
        )?;
        for r in &results {
            println!("  Servo {} arrived at {:.3}", r.id, r.result.position);
        }

        cycle += 1;
    }
}

#[tokio::main]
async fn run_async(args: &Args) -> Result<(), moteus::Error> {
    let opts = args.transport.clone().into();
    let mut transport = AsyncTransport::with_options(&opts).await?;

    let c1 = Controller::new(args.id1);
    let c2 = Controller::new(args.id2);

    // Clear any faults by sending stop commands.
    let mut stop_requests = vec![
        Request::new(c1.make_stop(false).into_frame()),
        Request::new(c2.make_stop(false).into_frame()),
    ];
    transport.cycle(&mut stop_requests).await?;

    let move_duration = args.period / 2.0;

    println!("Servo {}: alternating between 0 and 1 rev", args.id1);
    println!("Servo {}: alternating between 0 and 3 rev", args.id2);
    println!(
        "Period: {}s (each move: {}s)",
        args.period, move_duration
    );
    println!("Press Ctrl+C to stop");
    println!();

    let move_opts = MoveToOptions::new().duration(move_duration);

    let mut cycle = 0u64;
    loop {
        // Move to position A.
        println!("Cycle {}: Moving to position A (c1=0.0, c2=0.0)", cycle);
        let results = async_move_to(
            &mut transport,
            &[(&c1, 0.0.into()), (&c2, 0.0.into())],
            &move_opts,
        )
        .await?;
        for r in &results {
            println!("  Servo {} arrived at {:.3}", r.id, r.result.position);
        }

        // Move to position B.
        println!("Cycle {}: Moving to position B (c1=1.0, c2=3.0)", cycle);
        let results = async_move_to(
            &mut transport,
            &[(&c1, 1.0.into()), (&c2, 3.0.into())],
            &move_opts,
        )
        .await?;
        for r in &results {
            println!("  Servo {} arrived at {:.3}", r.id, r.result.position);
        }

        cycle += 1;
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
