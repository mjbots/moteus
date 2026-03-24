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

//! This example measures the maximal bandwidth capability of a given
//! system for a position/query loop.
//!
//! Usage:
//!   tools/bazel run //lib/rust:bandwidth_test
//!   tools/bazel run //lib/rust:bandwidth_test -- --fdcanusb /dev/ttyACM0
//!   tools/bazel run //lib/rust:bandwidth_test -- --target 1 --target 2
//!   tools/bazel run //lib/rust:bandwidth_test -- --minimal-format

use clap::Parser;
use moteus::command::{PositionCommand, PositionFormat};
use moteus::query::QueryFormat;
use moteus::transport::async_transport::AsyncTransport;
use moteus::{Controller, Request, Resolution, TransportArgs};
use std::io::Write;
use std::time::Instant;

const STATUS_PERIOD_S: f64 = 0.1;

/// Measure maximal bandwidth for a position/query loop.
#[derive(Parser)]
#[command(about = "Measure maximal bandwidth for a position/query loop")]
struct Args {
    /// Target servo IDs (can specify multiple times, or auto-discovers)
    #[arg(short = 't', long = "target", action = clap::ArgAction::Append)]
    target: Vec<u8>,

    /// Use minimal CAN frame size with lower resolution
    #[arg(long)]
    minimal_format: bool,

    #[command(flatten)]
    transport: TransportArgs,
}

#[tokio::main]
async fn main() -> Result<(), moteus::Error> {
    let args = Args::parse();

    // Create async transport with specified options.
    let opts = args.transport.into();
    let mut transport = AsyncTransport::with_options(&opts).await?;

    // Use specified targets or auto-discover.
    let targets: Vec<u8> = if args.target.is_empty() {
        let devices = transport.discover(0, 0).await?;
        devices.iter().map(|d| d.can_id).collect()
    } else {
        args.target
    };

    if targets.is_empty() {
        println!("No devices found!");
        return Ok(());
    }

    println!("Testing with {} target(s):", targets.len());
    for t in &targets {
        println!(" * {}", t);
    }
    println!();

    // Create controllers with optional minimal format.
    let controllers: Vec<Controller> = targets
        .iter()
        .map(|&id| {
            let mut ctrl = Controller::new(id);

            if args.minimal_format {
                // Use Int16 resolution for smaller frames
                let mut pos_fmt = PositionFormat::default();
                pos_fmt.position = Resolution::Int16;
                pos_fmt.velocity = Resolution::Int16;

                let mut qry_fmt = QueryFormat::default();
                qry_fmt.mode = Resolution::Int16;
                qry_fmt.position = Resolution::Int16;
                qry_fmt.velocity = Resolution::Int16;
                qry_fmt.torque = Resolution::Int16;

                ctrl = ctrl.position_format(pos_fmt).query_format(qry_fmt);
            }

            ctrl
        })
        .collect();

    let mut hz_count: u64 = 0;
    let mut status_time = Instant::now();

    loop {
        hz_count += 1;

        // Build position commands for all controllers.
        let mut requests: Vec<_> = controllers
            .iter()
            .map(|c| {
                Request::new(c.make_position_command(
                    &PositionCommand::new().position(f32::NAN).velocity(0.0),
                    true,
                ).into_frame())
            })
            .collect();

        // Send commands and receive responses.
        transport.cycle(&mut requests).await?;
        let count: usize = requests.iter().map(|r| r.responses.len()).sum();

        // Report stats periodically.
        let elapsed = status_time.elapsed().as_secs_f64();
        if elapsed > STATUS_PERIOD_S {
            let hz = hz_count as f64 / elapsed;
            print!("{:6.1}Hz  rx_count={}  \r", hz, count);
            std::io::stdout().flush().ok();
            hz_count = 0;
            status_time = Instant::now();
        }

        // Yield to allow other async tasks.
        tokio::task::yield_now().await;
    }
}
