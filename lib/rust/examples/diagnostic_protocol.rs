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

//! This example demonstrates the diagnostic protocol for configuration
//! and debugging.
//!
//! The diagnostic protocol should not be used during active control, but
//! is useful for setup, provisioning, and debugging.
//!
//! Usage:
//!   tools/bazel run //lib/rust:diagnostic_protocol
//!   tools/bazel run //lib/rust:diagnostic_protocol -- --fdcanusb /dev/ttyACM0
//!   tools/bazel run //lib/rust:diagnostic_protocol -- --id 1
//!   tools/bazel run //lib/rust:diagnostic_protocol -- --async

use clap::Parser;
use moteus::diagnostic::{AsyncDiagnosticStream, DiagnosticStream};
use moteus::transport::args::TransportArgs;
use moteus::{AsyncController, BlockingController};

/// Demonstrate the diagnostic protocol.
#[derive(Parser)]
#[command(about = "Demonstrate the diagnostic protocol")]
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
    let mut ctrl = BlockingController::with_options(args.id, &args.transport.clone().into());
    let mut stream = DiagnosticStream::new(&mut ctrl);

    // Applications like tview may leave the controller "spewing" on the
    // diagnostic channel (sending unsolicited data). Stop telemetry and
    // flush all data before issuing commands.
    println!("Stopping telemetry and flushing...");
    stream.write_message(b"tel stop")?;
    stream.flush_read()?;

    // Read a configuration value. The `conf get` command replies with a
    // single line, so use `command_oneline`.
    let old_kp = stream.command_oneline(b"conf get servo.pid_position.kp")?;
    let old_kp_str = String::from_utf8_lossy(&old_kp);
    let old_kp: f64 = old_kp_str.trim().parse().unwrap_or(0.0);

    let new_kp = 4.0;

    // Set a configuration value (waits for "OK").
    let cmd = format!("conf set servo.pid_position.kp {}", new_kp);
    stream.command(cmd.as_bytes())?;

    println!("Changed kp from {} to {}", old_kp, new_kp);

    Ok(())
}

#[tokio::main]
async fn run_async(args: &Args) -> Result<(), moteus::Error> {
    let mut ctrl =
        AsyncController::with_options(args.id, &args.transport.clone().into());
    let mut stream = AsyncDiagnosticStream::new(&mut ctrl);

    // Stop telemetry and flush pending data.
    println!("Stopping telemetry and flushing...");
    stream.write_message(b"tel stop").await?;
    stream.flush_read().await?;

    // Read a configuration value. The `conf get` command replies with a
    // single line, so use `command_oneline`.
    let old_kp = stream.command_oneline(b"conf get servo.pid_position.kp").await?;
    let old_kp_str = String::from_utf8_lossy(&old_kp);
    let old_kp: f64 = old_kp_str.trim().parse().unwrap_or(0.0);

    let new_kp = 4.0;

    // Set a configuration value (waits for "OK").
    let cmd = format!("conf set servo.pid_position.kp {}", new_kp);
    stream.command(cmd.as_bytes()).await?;

    println!("Changed kp from {} to {}", old_kp, new_kp);

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
