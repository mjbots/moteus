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

//! Runs the firmware in each of the modes whose control interrupt
//! execution time matters, on each supported board, driving a
//! simulated motor.

use moteus::reader::Value;
use moteus_emulator::motor::Motor;
use moteus_emulator::{Board, Machine};

/// The limit the dynamometer validation applies to the firmware's
/// reported "final_timer" in position mode, in clock cycles.  That is
/// applied here too, but a change to the firmware will be caught by
/// EXPECTED long before it gets there.
///
/// On a moteus-x1 the emulator has been within 0.3% of what the
/// controller reports, as the README describes.
const MAX_FINAL_TIMER: u32 = 3765;
const MAX_FINAL_TIMER_FIELD_WEAKENING: u32 = 3995;

/// What a calibrated controller would have configured for the
/// simulated motor, to the extent the firmware requires in order to
/// control it.
fn motor_config() -> Vec<String> {
    let motor = Motor::params();
    vec![
        format!("conf set motor.poles {}", 2 * motor.pole_pairs),
        format!("conf set motor.resistance_ohm {}", motor.resistance_ohm),
        format!("conf set motor.inductance_d_H {}", motor.inductance_h),
        format!("conf set motor.inductance_q_H {}", motor.inductance_h),
        format!("conf set motor.Kv {}", motor.kv),
        // The default limits only permit a fraction of a revolution.
        "conf set servopos.position_min nan".to_string(),
        "conf set servopos.position_max nan".to_string(),
    ]
}

const FIRST_LIMIT_CODE: u32 = 96;

/// An external torque which takes several amps to resist, while
/// staying clear of the default torque limit.
const LOAD_NM: f32 = 0.1;

/// A speed, in revolutions per second, well beyond what the simulated
/// motor can reach from the bus voltage without field weakening.
const WEAKENED_SPEED: f32 = 100.0;

/// The default position proportional gain, in Nm per revolution.
const POSITION_KP: f32 = 4.0;

const MODE_STOPPED: u32 = 0;
const MODE_POSITION: u32 = 10;

fn load(board: Board) -> Machine {
    let path = std::env::var("MOTEUS_ELF").expect("MOTEUS_ELF must name the firmware");
    let image = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut machine = Machine::new(&image, board).unwrap();
    machine.run_isrs(100, 50_000_000).unwrap();
    machine
}

/// The servo_stats telemetry channel.
struct ServoStats(Value);

impl ServoStats {
    fn read(machine: &mut Machine) -> Self {
        Self(machine.telemetry("servo_stats").unwrap())
    }

    fn get(&self, field: &str) -> f32 {
        self.0
            .get_path(field)
            .and_then(Value::as_f64)
            .unwrap_or_else(|| panic!("no number called {field} in servo_stats")) as f32
    }

    fn mode(&self) -> u32 {
        self.get("mode") as u32
    }
}

/// Run until the firmware is controlling position, which takes a few
/// milliseconds from stopped while the gate driver enables.
fn wait_for_position_mode(machine: &mut Machine) {
    for _ in 0..50 {
        machine.run_isrs(100, 50_000_000).unwrap();
        if ServoStats::read(machine).mode() == MODE_POSITION {
            return;
        }
    }
}

/// Measure the control interrupt in the current mode, returning the
/// largest "final_timer" and the firmware's telemetry.
fn measure(
    machine: &mut Machine,
    board: Board,
    scenario: &str,
    expected_mode: u32,
) -> (u32, ServoStats) {
    machine.isr_samples.clear();
    machine.run_isrs(300, 50_000_000).unwrap();
    let samples = &machine.isr_samples;
    let instructions = samples.iter().map(|s| s.instructions).max().unwrap();
    let min = samples.iter().map(|s| s.final_timer).min().unwrap();
    let max = samples.iter().map(|s| s.final_timer).max().unwrap();
    println!(
        "{:10} {scenario:10} instructions={instructions:5} final_timer={min}..{max}",
        board.name()
    );

    // Read the telemetry afterward, so that it describes the same
    // steady state which was measured.
    let stats = ServoStats::read(machine);
    // Codes from here up report which limit is active, not a fault.
    let fault = stats.get("fault") as u32;
    assert!(
        fault == 0 || fault >= FIRST_LIMIT_CODE,
        "{} {scenario}: fault {fault}",
        board.name()
    );
    assert_eq!(stats.mode(), expected_mode, "{} {scenario}", board.name());

    // What the emulator observes is what the firmware itself reports.
    let reported = stats.get("final_timer") as u32;
    assert!(
        reported + 100 >= min && reported <= max + 100,
        "firmware reports {reported}, emulator measured {min}..{max}"
    );
    (max, stats)
}

/// How often the dynamometer test's client polls for telemetry, in
/// processor cycles: every 10ms.
const POLL_CYCLES: u64 = 1_700_000;

/// The largest final_timer among the servo_stats which the firmware
/// streams, which is what the dynamometer test checks.  That is more
/// than most interrupts take, as the firmware is busy sending when
/// it takes these.
fn measure_streamed(machine: &mut Machine, board: Board, scenario: &str) -> u32 {
    machine.stream_telemetry().unwrap();
    // Long enough for several to be sent.
    machine.run_isrs_polling(4500, POLL_CYCLES).unwrap();
    let reported: Vec<u32> = machine
        .streamed_telemetry("servo_stats")
        .unwrap()
        .iter()
        .map(|stats| stats.get("final_timer").and_then(Value::as_u64).unwrap() as u32)
        .collect();
    machine.stop_telemetry().unwrap();
    assert!(reported.len() >= 2, "{scenario}: {reported:?}");
    let min = *reported.iter().min().unwrap();
    let max = *reported.iter().max().unwrap();
    println!(
        "{:10} {scenario:10} streamed final_timer={min}..{max}",
        board.name()
    );
    max
}

/// How many cycles the control interrupt takes in each state, on each
/// board, as the largest "final_timer".  Nothing here varies from one
/// run to the next, so these only change when the firmware does, or
/// the compiler, or the emulator.
///
/// When one of those is meant to change them, the test prints what to
/// put here.  When it is not, this is what notices.
const EXPECTED: &[(&str, &[(&str, u32)])] = &[
    (
        "moteus-r4",
        &[
            ("stopped", 1782),
            ("position", 3282),
            ("trajectory", 3452),
            ("trajectory, streamed", 3443),
            ("velocity", 3279),
            ("loaded", 3280),
            ("field weakening enabled", 3687),
            ("field weakening enabled, streamed", 3675),
            ("field weakening", 3521),
        ],
    ),
    (
        "moteus-n1",
        &[
            ("stopped", 1772),
            ("position", 3262),
            ("trajectory", 3428),
            ("trajectory, streamed", 3432),
            ("velocity", 3264),
            ("loaded", 3269),
            ("field weakening enabled", 3663),
            ("field weakening enabled, streamed", 3661),
            ("field weakening", 3503),
        ],
    ),
    (
        "moteus-c1",
        &[
            ("stopped", 1776),
            ("position", 3272),
            ("trajectory", 3440),
            ("trajectory, streamed", 3433),
            ("velocity", 3272),
            ("loaded", 3274),
            ("field weakening enabled", 3673),
            ("field weakening enabled, streamed", 3669),
            ("field weakening", 3511),
        ],
    ),
    (
        "moteus-x1",
        &[
            ("stopped", 1757),
            ("position", 3254),
            ("trajectory", 3432),
            ("trajectory, streamed", 3418),
            ("velocity", 3262),
            ("loaded", 3253),
            ("field weakening enabled", 3648),
            ("field weakening enabled, streamed", 3654),
            ("field weakening", 3519),
        ],
    ),
    (
        "moteus-r5",
        &[
            ("stopped", 1776),
            ("position", 3278),
            ("trajectory", 3440),
            ("trajectory, streamed", 3440),
            ("velocity", 3268),
            ("loaded", 3269),
            ("field weakening enabled", 3672),
            ("field weakening enabled, streamed", 3666),
            ("field weakening", 3538),
        ],
    ),
];

/// How far above what is expected a measurement may be, and how far
/// below.  It is being slower which matters, so there is less room
/// for that.
const ALLOWED_ABOVE: u32 = 25;
const ALLOWED_BELOW: u32 = 60;

fn check_expected(board: Board, measured: &[(&str, u32)]) {
    let listing: String = measured
        .iter()
        .map(|(scenario, value)| format!("            (\"{scenario}\", {value}),\n"))
        .collect();
    let listing = format!(
        "    (\n        \"{}\",\n        &[\n{listing}        ],\n    ),",
        board.name()
    );
    let Some((_, expected)) = EXPECTED.iter().find(|x| x.0 == board.name()) else {
        panic!("nothing is expected of {}:\n{listing}", board.name());
    };
    let mut problems = Vec::new();
    for (scenario, value) in measured {
        match expected.iter().find(|x| x.0 == *scenario) {
            Some((_, usual))
                if *value <= usual + ALLOWED_ABOVE && *value + ALLOWED_BELOW >= *usual => {}
            Some((_, usual)) => problems.push(format!("{scenario}: {value}, not {usual}")),
            None => problems.push(format!("{scenario}: {value}, with nothing expected")),
        }
    }
    assert!(
        problems.is_empty() && measured.len() == expected.len(),
        "{} does not take as long as expected:\n  {}\nIf this is intended, EXPECTED wants:\n{listing}",
        board.name(),
        problems.join("\n  ")
    );
}

fn check_board(board: Board) {
    let mut machine = load(board);
    measure(&mut machine, board, "stopped", MODE_STOPPED);
    // The occasional interrupt does much more when stopped, from
    // flash, so take what is usual.
    let mut all: Vec<u32> = machine.isr_samples.iter().map(|s| s.final_timer).collect();
    all.sort_unstable();
    let stopped = all[all.len() / 2];

    machine.attach_motor().unwrap();
    for line in motor_config() {
        machine.command(&line).unwrap();
    }

    // Hold the current position.
    machine.command("d pos nan 0 nan").unwrap();
    wait_for_position_mode(&mut machine);
    let (position, _) = measure(&mut machine, board, "position", MODE_POSITION);

    // What the dynamometer uses to exercise the slowest path: a
    // trajectory which is velocity and acceleration limited, but with
    // no torque with which to follow it.
    machine.command("d pos 10000 nan 0 a40 v100").unwrap();
    machine.run_isrs(200, 50_000_000).unwrap();
    let (trajectory, _) = measure(&mut machine, board, "trajectory", MODE_POSITION);

    // Turn at a constant speed.
    machine.command("d pos nan 2 nan").unwrap();
    machine.run_isrs(1500, 100_000_000).unwrap();
    let (velocity, stats) = measure(&mut machine, board, "velocity", MODE_POSITION);
    let measured = stats.get("velocity");
    assert!((measured - 2.0).abs() < 0.1, "velocity {measured}");

    // Hold position against an external torque, so that real current
    // flows.
    machine.command("d pos nan 0 nan").unwrap();
    machine.run_isrs(1500, 100_000_000).unwrap();
    let held_position = ServoStats::read(&mut machine).get("position");
    set_load(&machine, LOAD_NM);
    machine.run_isrs(3000, 200_000_000).unwrap();
    let (loaded, stats) = measure(&mut machine, board, "loaded", MODE_POSITION);
    let torque = stats.get("torque_Nm");
    assert!(
        (torque.abs() - LOAD_NM).abs() < 0.2 * LOAD_NM,
        "torque {torque}"
    );
    let speed = stats.get("velocity");
    assert!(speed.abs() < 0.1, "velocity {speed} under load");
    // The rotor settles where the proportional term balances the load.
    let deflection = (stats.get("position") - held_position).abs();
    let expected = LOAD_NM / POSITION_KP;
    assert!(
        (deflection - expected).abs() < 0.2 * expected,
        "deflection {deflection}"
    );
    set_load(&machine, 0.0);

    assert!(stopped < position, "{stopped} {position}");
    for (scenario, final_timer) in [
        ("position", position),
        ("trajectory", trajectory),
        ("velocity", velocity),
        ("loaded", loaded),
    ] {
        assert!(final_timer < MAX_FINAL_TIMER, "{scenario} {final_timer}");
    }

    machine.command("d stop").unwrap();
    machine.run_isrs(100, 50_000_000).unwrap();
    measure(&mut machine, board, "stopped", MODE_STOPPED);

    // The same trajectory again, as the dynamometer test sees it.
    machine.command("d pos 10000 nan 0 a40 v100").unwrap();
    wait_for_position_mode(&mut machine);
    let trajectory_streamed = measure_streamed(&mut machine, board, "trajectory");
    assert!(
        trajectory_streamed < MAX_FINAL_TIMER,
        "trajectory {trajectory_streamed}"
    );
    machine.command("d stop").unwrap();
    machine.run_isrs(100, 50_000_000).unwrap();

    // The dynamometer repeats its slowest case with field weakening
    // enabled, for which it permits more time.
    for line in [
        "conf set servo.bemf_feedforward 1.0",
        "conf set servo.bemf_feedforward_override 1",
        "conf set servo.fw.enable 1",
        "d pos 10000 nan 0 a40 v100",
    ] {
        machine.command(line).unwrap();
    }
    wait_for_position_mode(&mut machine);
    let (field_weakening, _) = measure(&mut machine, board, "fw", MODE_POSITION);
    let field_weakening_streamed = measure_streamed(&mut machine, board, "fw");
    assert!(
        field_weakening.max(field_weakening_streamed) < MAX_FINAL_TIMER_FIELD_WEAKENING,
        "{field_weakening} {field_weakening_streamed}"
    );
    assert!(trajectory < field_weakening);

    // Then run fast enough that the field actually has to be weakened:
    // without it, the motor cannot exceed its base speed.
    machine
        .command(&format!("d pos nan {WEAKENED_SPEED} nan a3000"))
        .unwrap();
    // It overshoots, and then settles at what the bus voltage and
    // the current available to weaken the field with allow, which on
    // some boards is less than was asked for.  How long the interrupt
    // takes differs until then.
    machine.run_isrs(20000, 200_000_000).unwrap();
    let (weakening, stats) = measure(&mut machine, board, "weakening", MODE_POSITION);
    let speed = stats.get("velocity");
    assert!(
        speed > 1.02 * stats.get("motor_base_velocity"),
        "{speed} is not above base speed"
    );
    let d_current = stats.get("fw.id_A");
    assert!(d_current < -1.0, "field weakening current {d_current}");
    assert!(weakening < MAX_FINAL_TIMER_FIELD_WEAKENING, "{weakening}");

    check_expected(
        board,
        &[
            ("stopped", stopped),
            ("position", position),
            ("trajectory", trajectory),
            ("trajectory, streamed", trajectory_streamed),
            ("velocity", velocity),
            ("loaded", loaded),
            ("field weakening enabled", field_weakening),
            (
                "field weakening enabled, streamed",
                field_weakening_streamed,
            ),
            ("field weakening", weakening),
        ],
    );
}

/// Apply an external torque to the simulated motor.
fn set_load(machine: &Machine, load_nm: f32) {
    machine.soc.borrow_mut().motor().unwrap().load_nm = load_nm;
}

/// Each board is its own test target, so that none takes long.
#[test]
fn board() {
    let name = std::env::var("MOTEUS_BOARD").expect("MOTEUS_BOARD must name the board");
    check_board(Board::from_name(&name).unwrap_or_else(|| panic!("unknown board {name}")));
}
