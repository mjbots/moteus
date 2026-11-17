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

use moteus_emulator::{Board, Machine};

fn load(board: Board) -> Machine {
    let path = std::env::var("MOTEUS_ELF").expect("MOTEUS_ELF must name the firmware");
    let image = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    Machine::new(&image, board).unwrap()
}

/// The firmware boots and settles into running its control interrupt.
#[test]
fn boots_and_runs_control_isr() {
    let mut machine = load(Board::R5);
    machine.run_isrs(200, 50_000_000).unwrap();

    // The first interrupts after the timer starts do less work, so
    // only look at the steady state.
    let steady = &machine.isr_samples[100..];
    let min = steady.iter().map(|s| s.instructions).min().unwrap();
    let max = steady.iter().map(|s| s.instructions).max().unwrap();

    // These are loose sanity bounds, not a performance requirement.
    assert!(min > 300, "implausibly short control interrupt: {min}");
    assert!(max < 5000, "implausibly long control interrupt: {max}");
}
