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

//! This example demonstrates reading and writing GPIO digital pins.
//!
//! Usage:
//!   tools/bazel run //lib/rust:gpio
//!   tools/bazel run //lib/rust:gpio -- --fdcanusb /dev/ttyACM0
//!   tools/bazel run //lib/rust:gpio -- --id 1
//!   tools/bazel run //lib/rust:gpio -- --async

use clap::Parser;
use moteus::query::QueryFormat;
use moteus::{AsyncController, BlockingController, Resolution, TransportArgs};

/// Demonstrate GPIO reading and writing.
#[derive(Parser)]
#[command(about = "Demonstrate GPIO reading and writing")]
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

/// Display GPIO pin states for an auxiliary port.
fn display_gpio(aux_num: u8, value: u8, pin_count: u8) {
    println!("AUX{}", aux_num);
    for i in 0..pin_count {
        let state = if value & (1 << i) != 0 { "HIGH" } else { "LOW" };
        println!("  Pin {} - {}", i, state);
    }
}

fn run_blocking(args: &Args) -> Result<(), moteus::Error> {
    let mut c = BlockingController::with_options(args.id, &args.transport.clone().into());

    // Read GPIO digital inputs from both AUX ports. Each value is a
    // byte where bit N represents pin N's state.
    let (aux1, aux2) = c.read_gpio()?;

    // Display the GPIO inputs as individual pin values.
    display_gpio(1, aux1, 5);
    println!();
    display_gpio(2, aux2, 4);

    // Write GPIO digital outputs (set all outputs high).
    c.set_write_gpio(Some(0x7f), Some(0x7f))?;
    println!("\nSet all GPIO outputs to HIGH");

    // GPIO values can also be included in general query results.
    let controller = moteus::Controller::new(args.id).query_format(QueryFormat {
        aux1_gpio: Resolution::Int8,
        aux2_gpio: Resolution::Int8,
        ..QueryFormat::default()
    });
    let mut c2 = BlockingController::with_controller(controller);

    // Query the controller - response includes GPIO values.
    let result = c2.query()?;

    println!("\nFrom BlockingController::query()");
    display_gpio(1, result.aux1_gpio as u8, 5);
    println!();
    display_gpio(2, result.aux2_gpio as u8, 4);

    Ok(())
}

#[tokio::main]
async fn run_async(args: &Args) -> Result<(), moteus::Error> {
    let mut c =
        AsyncController::with_options(args.id, &args.transport.clone().into());

    // Read GPIO digital inputs from both AUX ports. Each value is a
    // byte where bit N represents pin N's state.
    let (aux1, aux2) = c.read_gpio().await?;

    // Display the GPIO inputs as individual pin values.
    display_gpio(1, aux1, 5);
    println!();
    display_gpio(2, aux2, 4);

    // Write GPIO digital outputs (set all outputs high).
    c.set_write_gpio(Some(0x7f), Some(0x7f)).await?;
    println!("\nSet all GPIO outputs to HIGH");

    // GPIO values can also be included in general query results.
    let controller = moteus::Controller::new(args.id).query_format(QueryFormat {
        aux1_gpio: Resolution::Int8,
        aux2_gpio: Resolution::Int8,
        ..QueryFormat::default()
    });
    let mut c2 =
        AsyncController::with_controller(controller);

    // Query the controller - response includes GPIO values.
    let result = c2.query().await?;

    println!("\nFrom AsyncController::query()");
    display_gpio(1, result.aux1_gpio as u8, 5);
    println!();
    display_gpio(2, result.aux2_gpio as u8, 4);

    Ok(())
}

fn main() -> Result<(), moteus::Error> {
    let args = Args::parse();

    if args.r#async {
        run_async(&args)
    } else {
        run_blocking(&args)
    }
}
