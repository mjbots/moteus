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

//! This example discovers all connected moteus controllers and displays
//! their CAN IDs and UUIDs.
//!
//! Usage:
//!   tools/bazel run //lib/rust:discover
//!   tools/bazel run //lib/rust:discover -- --fdcanusb /dev/ttyACM0
//!   tools/bazel run //lib/rust:discover -- --can-chan can0

use clap::Parser;
use moteus::{get_singleton_transport, TransportArgs};

/// Discover all connected moteus controllers.
#[derive(Parser)]
#[command(about = "Discover all connected moteus controllers")]
struct Args {
    #[command(flatten)]
    transport: TransportArgs,
}

fn main() -> Result<(), moteus::Error> {
    let args = Args::parse();

    // Get the singleton transport with specified options.
    let transport = get_singleton_transport(Some(&args.transport.into()))?;
    let mut transport = transport.lock().unwrap();

    // Discover all connected devices.
    let devices = transport.discover(0, 0)?;

    if devices.is_empty() {
        println!("No devices found!");
        return Ok(());
    }

    for device in &devices {
        println!("{}", device);
    }

    Ok(())
}
