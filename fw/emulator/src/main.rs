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

//! Boots a moteus firmware image in the emulator and reports on the
//! control interrupt.
//!
//! Usage: emulator [--board=NAME] moteus.elf [ISR_COUNT [COMMAND...]]
//!
//! NAME is one of r4, n1, c1, x1 or r5.  --encoder-velocity=N spins the
//! on-board encoder at N revolutions per second.  --motor instead
//! connects a simulated motor.
//!
//! In place of an ELF file, the contents of flash as read back from a
//! controller can be given.
//!
//! Each COMMAND is a diagnostic channel line, such as "d stop", a
//! number of control interrupts to run, "load N" to apply a torque to
//! the simulated motor, "mem ADDRESS COUNT" to print memory, or
//! "send LINE" for a diagnostic channel line which will get no
//! response.  --poll-ms=N polls the diagnostic channel that often
//! while running, as a client which has asked for telemetry to be
//! streamed does.

use std::process::ExitCode;

use moteus_emulator::{Board, Machine};
use zmu_cortex_m::bus::Bus;

/// Run control interrupts, polling the diagnostic channel if asked to.
fn run(machine: &mut Machine, isrs: usize, poll_ms: Option<f64>) -> Result<(), String> {
    match poll_ms {
        // The processor runs at 170MHz.
        Some(ms) => machine.run_isrs_polling(isrs, (ms * 170_000.0) as u64),
        None => machine.run_isrs(isrs, 1_000_000_000),
    }
}

fn run_main() -> Result<(), String> {
    const USAGE: &str =
        "usage: emulator [--board=NAME] [--encoder-velocity=N] moteus.elf [ISR_COUNT [COMMAND...]]";
    let mut board = Board::R5;
    let mut args = Vec::new();
    let mut encoder_velocity = 0.0;
    let mut motor = false;
    let mut poll_ms = None;
    for arg in std::env::args().skip(1) {
        if let Some(name) = arg.strip_prefix("--board=") {
            board = Board::from_name(name).ok_or(format!("unknown board {name}"))?;
        } else if arg == "--motor" {
            motor = true;
        } else if let Some(value) = arg.strip_prefix("--poll-ms=") {
            poll_ms = Some(value.parse::<f64>().map_err(|e| format!("{value}: {e}"))?);
        } else if let Some(value) = arg.strip_prefix("--encoder-velocity=") {
            encoder_velocity = value.parse().map_err(|e| format!("{value}: {e}"))?;
        } else {
            args.push(arg);
        }
    }
    let path = args.first().ok_or(USAGE)?;
    let count: usize = match args.get(1) {
        Some(arg) => arg.parse().map_err(|e| format!("{arg}: {e}"))?,
        None => 1000,
    };

    let image = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    // Anything which is not an ELF file is taken to be the contents
    // of flash, as read back from a controller.
    let mut machine = if image.starts_with(b"\x7fELF") {
        Machine::new(&image, board)?
    } else {
        Machine::with_flash_image(&image, board)?
    };
    machine.soc.borrow_mut().trace = std::env::var_os("EMULATOR_TRACE").is_some();
    if let Ok(name) = std::env::var("EMULATOR_TRACE_FUNCTION") {
        machine.trace_function = machine.elf.functions.iter().position(|f| f.name == name);
    }
    machine
        .soc
        .borrow_mut()
        .set_encoder_velocity(encoder_velocity);
    // Let the firmware settle before profiling.
    machine.run_isrs(200, 1_000_000_000)?;
    if motor {
        machine.attach_motor()?;
    }
    if poll_ms.is_some() {
        // To read what is streamed with, asked for while the
        // firmware is sending nothing else.
        machine.schema("servo_stats")?;
    }
    for line in args.iter().skip(2) {
        println!("> {line}");
        // A bare number runs that many control interrupts, and
        // "load N" applies a torque to the simulated motor.
        if let Some(torque) = line.strip_prefix("load ") {
            let torque: f32 = torque.parse().map_err(|e| format!("{torque}: {e}"))?;
            let mut soc = machine.soc.borrow_mut();
            soc.motor().ok_or("load requires --motor")?.load_nm = torque;
            continue;
        }
        // "send LINE" does not wait for a response.
        if let Some(rest) = line.strip_prefix("send ") {
            machine.send_line(rest)?;
            continue;
        }
        // "mem ADDRESS COUNT" prints words of memory, in hex.
        if let Some(rest) = line.strip_prefix("mem ") {
            let fields: Vec<&str> = rest.split_whitespace().collect();
            let parse = |text: &str| {
                u32::from_str_radix(text.trim_start_matches("0x"), 16)
                    .map_err(|e| format!("{text}: {e}"))
            };
            let address = parse(fields.first().ok_or("mem needs an address")?)?;
            let count = match fields.get(1) {
                Some(text) => text.parse::<u32>().map_err(|e| format!("{text}: {e}"))?,
                None => 1,
            };
            let words: Vec<String> = (0..count)
                .map(|i| match machine.cpu.read32(address + 4 * i) {
                    Ok(word) => format!("{word:08x}"),
                    Err(_) => "????????".to_string(),
                })
                .collect();
            println!("{}", words.join(" "));
            continue;
        }
        match line.parse::<usize>() {
            Ok(isrs) => run(&mut machine, isrs, poll_ms)?,
            Err(_) => print!("{}", machine.command(line)?),
        }
    }
    machine.isr_samples.clear();
    machine.profile = Some(Default::default());
    let pc_profile_path = std::env::var("EMULATOR_PC_PROFILE").ok();
    if pc_profile_path.is_some() {
        machine.pc_profile = Some(Default::default());
    }
    let cycles_before = machine.cpu.cycle_count;
    machine.streamed.clear();
    run(&mut machine, count, poll_ms)?;
    let reported = if poll_ms.is_some() {
        machine
            .streamed_telemetry("servo_stats")?
            .iter()
            .map(|stats| {
                let value = stats.get("final_timer").and_then(|x| x.as_u64());
                Ok(value.ok_or("servo_stats has no final_timer")? as u32)
            })
            .collect::<Result<Vec<_>, String>>()?
    } else {
        Vec::new()
    };

    if let Some(path) = pc_profile_path {
        // One line per address: the address, and the cycles spent
        // there, preceded by the total.
        let mut lines = vec![format!("total {}", machine.cpu.cycle_count - cycles_before)];
        let mut entries: Vec<_> = machine
            .pc_profile
            .take()
            .unwrap_or_default()
            .into_iter()
            .collect();
        entries.sort();
        lines.extend(entries.iter().map(|(pc, cycles)| {
            let kind = machine.pc_kinds.get(pc).map_or("?", |k| k.as_str());
            let isr = machine.pc_profile_isr.get(pc).copied().unwrap_or(0);
            let (arrival, order) = machine.pc_arrival.get(pc).copied().unwrap_or((0, 0));
            format!(
                "{pc:08x} {cycles} {} {isr} {arrival} {order}",
                kind.replace(' ', "")
            )
        }));
        std::fs::write(&path, lines.join("\n") + "\n").map_err(|e| format!("{path}: {e}"))?;
    }

    let samples = &machine.isr_samples;
    let stats = |name: &str, values: Vec<u64>| {
        println!(
            "  {name}: min={} max={} last={}",
            values.iter().min().unwrap_or(&0),
            values.iter().max().unwrap_or(&0),
            values.last().unwrap_or(&0),
        );
    };
    println!("{} control interrupts", samples.len());
    stats(
        "instructions",
        samples.iter().map(|s| s.instructions).collect(),
    );
    stats("cycles", samples.iter().map(|s| s.cycles).collect());
    stats(
        "final_timer",
        samples.iter().map(|s| u64::from(s.final_timer)).collect(),
    );

    if poll_ms.is_some() {
        // What a client which checks every servo_stats it is sent
        // sees, which is not a fair sample of every interrupt: the
        // firmware is busy sending when it takes these.
        stats(
            "final_timer as streamed",
            reported.iter().map(|&x| u64::from(x)).collect(),
        );
        if std::env::var_os("EMULATOR_DEBUG").is_some() {
            let mut histogram = std::collections::BTreeMap::new();
            for value in &reported {
                *histogram.entry(value).or_insert(0) += 1;
            }
            println!("streamed final_timer x count: {histogram:?}");
        }
    }

    if std::env::var_os("EMULATOR_DEBUG").is_some() {
        println!(
            "ADC channels read: {:x?}",
            machine.soc.borrow().adc_channels_read
        );
        print!("{}", machine.soc.borrow().gpio_debug());
        print!("{}", machine.soc.borrow().adc_debug());
        println!("msp={:08x} psp={:08x}", machine.cpu.msp, machine.cpu.psp);
    }
    if std::env::var_os("EMULATOR_DEBUG").is_some() {
        // The distribution of final_timer, and of how long after the
        // timer event the handler was entered.
        let mut histogram = std::collections::BTreeMap::new();
        for sample in samples {
            *histogram
                .entry((sample.final_timer, sample.cycles))
                .or_insert(0) += 1;
        }
        println!("(final_timer, cycles) x count: {histogram:?}");
    }
    println!("per interrupt, by function:");
    println!("  {:>8} {:>8}", "insns", "cycles");
    let mut profile: Vec<_> = machine
        .profile
        .take()
        .unwrap_or_default()
        .into_iter()
        .collect();
    profile.sort_by_key(|(_, (_, cycles))| std::cmp::Reverse(*cycles));
    let n = samples.len().max(1) as f64;
    for (index, (instructions, cycles)) in profile.iter().take(25) {
        println!(
            "  {:8.1} {:8.1} {}",
            *instructions as f64 / n,
            *cycles as f64 / n,
            machine.elf.functions[*index].name
        );
    }
    Ok(())
}

fn main() -> ExitCode {
    match run_main() {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("{message}");
            ExitCode::FAILURE
        }
    }
}
