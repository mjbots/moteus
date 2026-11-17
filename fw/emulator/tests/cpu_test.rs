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

//! Checks the instruction set simulator against known results, for
//! the areas where it has been found wanting.

use std::cell::RefCell;
use std::rc::Rc;

use moteus_emulator::soc::{Soc, SocBus};
use moteus_emulator::Board;
use zmu_cortex_m::bus::Bus;
use zmu_cortex_m::core::register::BaseReg;
use zmu_cortex_m::executor::Executor;
use zmu_cortex_m::Processor;

/// An address in CCM RAM to execute from.
const CODE: u32 = 0x1000_0000;

const FLAG_Z: u32 = 1 << 30;

/// A processor in thread mode with the FPU enabled.
fn make_cpu() -> Processor {
    let soc = Rc::new(RefCell::new(Soc::new(Board::R5)));
    let mut cpu = Processor::new();
    cpu.device(Some(Box::new(SocBus(soc))));
    cpu.running = true;
    cpu.cpacr = 0x00f0_0000;
    // Thumb state.
    cpu.psr.value = 1 << 24;
    cpu
}

/// Execute a sequence of halfwords, one step per instruction.
/// Returns false if a fault resulted.
fn try_run(cpu: &mut Processor, halfwords: &[u16], instructions: usize) -> bool {
    for (i, hw) in halfwords.iter().enumerate() {
        cpu.write16(CODE + 2 * i as u32, *hw).unwrap();
    }
    // The code at this address just changed.
    cpu.clear_ram_instruction_cache();
    cpu.set_pc(CODE);
    for _ in 0..instructions {
        cpu.step();
    }
    cpu.take_pending_fault_trap().is_none()
}

/// Execute a sequence of halfwords, one step per instruction.
fn run(cpu: &mut Processor, halfwords: &[u16], instructions: usize) {
    assert!(try_run(cpu, halfwords, instructions));
}

/// One example of each kind of failing instruction.
fn summarize(failures: &[String]) -> String {
    let mut seen = std::collections::BTreeMap::new();
    for failure in failures {
        let name = failure.split_whitespace().next().unwrap_or("");
        let entry = seen.entry(name).or_insert((0, failure));
        entry.0 += 1;
    }
    seen.iter()
        .map(|(_, (count, example))| format!("{count:4} x {example}"))
        .collect::<Vec<_>>()
        .join("\n")
}

fn is_nan(bits: u32) -> bool {
    f32::from_bits(bits).is_nan()
}

#[test]
fn floating_point_matches_ieee754() {
    let path = std::env::var("FP_VECTORS").expect("FP_VECTORS must name the vectors");
    let vectors = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));

    let mut cpu = make_cpu();
    let mut count = 0;
    let mut failures = Vec::new();
    for line in vectors.lines().filter(|l| !l.starts_with('#')) {
        let fields: Vec<&str> = line.split_whitespace().collect();
        let hex = |i: usize| u32::from_str_radix(fields[i], 16).unwrap();
        let flags = fields[3] == "flags";

        cpu.fp_regs[0] = hex(4);
        cpu.fp_regs[1] = hex(5);
        cpu.fp_regs[2] = hex(6);
        cpu.fpscr = 0;
        run(&mut cpu, &[hex(1) as u16, hex(2) as u16], 1);

        let expected = hex(7);
        let actual = if flags {
            cpu.fpscr >> 28
        } else {
            cpu.fp_regs[2]
        };
        let matches = actual == expected || (!flags && is_nan(actual) && is_nan(expected));
        if !matches {
            failures.push(format!("{line} -> {actual:08x}"));
        }
        count += 1;
    }
    assert!(count > 1000, "only {count} vectors");
    assert!(
        failures.is_empty(),
        "{} of {count} failed, including:\n{}",
        failures.len(),
        failures[..failures.len().min(20)].join("\n")
    );
}

#[test]
fn integer_instructions_match_documented_behavior() {
    let path = std::env::var("INT_VECTORS").expect("INT_VECTORS must name the vectors");
    let vectors = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));

    let mut cpu = make_cpu();
    let mut count = 0;
    let mut failures = Vec::new();
    for line in vectors.lines().filter(|l| !l.starts_with('#')) {
        // name hw1 hw2 flags r0 r1 r2 r3 -> r2 r3 flags
        let fields: Vec<&str> = line.split_whitespace().collect();
        let hex = |i: usize| u32::from_str_radix(fields[i], 16).unwrap();

        // Thumb state, with the given NZCVQ.
        cpu.psr.value = (1 << 24) | (hex(3) << 27);
        for (i, field) in (4..8).enumerate() {
            cpu.r0_12[i] = hex(field);
        }
        let ok = if fields[2] == "----" {
            try_run(&mut cpu, &[hex(1) as u16], 1)
        } else {
            try_run(&mut cpu, &[hex(1) as u16, hex(2) as u16], 1)
        };
        if !ok {
            failures.push(format!("{line} faulted"));
            cpu = make_cpu();
            count += 1;
            continue;
        }

        let actual = (cpu.r0_12[2], cpu.r0_12[3], cpu.psr.value >> 27);
        if actual != (hex(9), hex(10), hex(11)) {
            failures.push(format!(
                "{line} got {:08x} {:08x} {:02x}",
                actual.0, actual.1, actual.2
            ));
        }
        count += 1;
    }
    assert!(count > 1000, "only {count} vectors");
    assert!(
        failures.is_empty(),
        "{} of {count} failed, including:\n{}",
        failures.len(),
        summarize(&failures)
    );
}

/// Where the flow vectors expect r4 and sp to point into.
const DATA: u32 = 0x2000_1000;
const DATA_WORDS: usize = 8;

#[test]
fn loads_stores_and_branches_match_documented_behavior() {
    let path = std::env::var("FLOW_VECTORS").expect("FLOW_VECTORS must name the vectors");
    let vectors = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));

    /// The flags, r0 to r5, sp, lr, and then memory.
    fn parse(text: &str) -> Vec<u32> {
        text.split_whitespace()
            .map(|x| u32::from_str_radix(x, 16).unwrap())
            .collect()
    }

    let mut count = 0;
    let mut failures = Vec::new();
    for line in vectors.lines().filter(|l| !l.starts_with('#')) {
        // name steps halfwords : before -> after pc
        let (head, rest) = line.split_once(" : ").unwrap();
        let (before, after) = rest.split_once(" -> ").unwrap();
        let head: Vec<&str> = head.split_whitespace().collect();
        let steps: usize = head[1].parse().unwrap();
        let code: Vec<u16> = head[2..]
            .iter()
            .map(|x| u16::from_str_radix(x, 16).unwrap())
            .collect();
        let before = parse(before);
        let (after, expected_pc) = after.rsplit_once(' ').unwrap();
        let expected = parse(after);

        let mut cpu = make_cpu();
        cpu.psr.value = (1 << 24) | (before[0] << 28);
        cpu.r0_12[..6].copy_from_slice(&before[1..7]);
        cpu.msp = before[7];
        cpu.lr = before[8];
        for (i, word) in before[9..].iter().enumerate() {
            cpu.write32(DATA + 4 * i as u32, *word).unwrap();
        }

        count += 1;
        if !try_run(&mut cpu, &code, steps) {
            failures.push(format!("{line} faulted"));
            continue;
        }

        let mut actual = vec![cpu.psr.value >> 28];
        actual.extend(&cpu.r0_12[..6]);
        actual.push(cpu.msp);
        actual.push(cpu.lr);
        for i in 0..DATA_WORDS {
            actual.push(cpu.read32(DATA + 4 * i as u32).unwrap());
        }
        let pc_ok = match expected_pc {
            "-" => true,
            offset => cpu.get_pc() == CODE + u32::from_str_radix(offset, 16).unwrap(),
        };
        if actual != expected || !pc_ok {
            let actual: Vec<String> = actual.iter().map(|x| format!("{x:08x}")).collect();
            failures.push(format!(
                "{line}\n       got {} pc {:x}",
                actual.join(" "),
                cpu.get_pc().wrapping_sub(CODE)
            ));
        }
    }
    assert!(count > 1000, "only {count} vectors");
    assert!(
        failures.is_empty(),
        "{} of {count} failed, including:\n{}",
        failures.len(),
        summarize(&failures)
    );
}

/// Floating point moves inside an IT block only happen when the
/// condition holds.
#[test]
fn conditional_floating_point_moves() {
    const IT_NE: u16 = 0xbf18;
    const IT_EQ: u16 = 0xbf08;
    // vmov.f32 s2, s0
    const VMOV_REGISTER: [u16; 2] = [0xeeb0, 0x1a40];
    // vmov.f32 s2, #1.0
    const VMOV_IMMEDIATE: [u16; 2] = [0xeeb7, 0x1a00];
    // vmov s2, r0
    const VMOV_FROM_CORE: [u16; 2] = [0xee01, 0x0a10];
    // vmov r1, s0
    const VMOV_TO_CORE: [u16; 2] = [0xee10, 0x1a10];

    for (it, executes) in [(IT_NE, false), (IT_EQ, true)] {
        for op in [VMOV_REGISTER, VMOV_IMMEDIATE, VMOV_FROM_CORE, VMOV_TO_CORE] {
            let mut cpu = make_cpu();
            // With Z set, "eq" holds and "ne" does not.
            cpu.psr.value |= FLAG_Z;
            cpu.fp_regs[0] = 0x4000_0000;
            cpu.fp_regs[2] = 0x1234_5678;
            cpu.r0_12[0] = 0x4040_0000;
            cpu.r0_12[1] = 0x8765_4321;
            run(&mut cpu, &[it, op[0], op[1]], 2);

            let changed = cpu.fp_regs[2] != 0x1234_5678 || cpu.r0_12[1] != 0x8765_4321;
            assert_eq!(changed, executes, "it={it:04x} op={op:04x?}");
        }
    }
}

/// Subtraction involving infinities, which was once confused with
/// addition, and which once never returned.
#[test]
fn subtract_infinity() {
    // vsub.f32 s2, s0, s1
    const VSUB: [u16; 2] = [0xee30, 0x1a60];
    let inf = f32::INFINITY.to_bits();
    let neg_inf = f32::NEG_INFINITY.to_bits();
    for (a, b, expected) in [
        (1.0f32.to_bits(), inf, Some(neg_inf)),
        (1.0f32.to_bits(), neg_inf, Some(inf)),
        (inf, neg_inf, Some(inf)),
        (neg_inf, inf, Some(neg_inf)),
        (inf, inf, None),
    ] {
        let mut cpu = make_cpu();
        cpu.fp_regs[0] = a;
        cpu.fp_regs[1] = b;
        run(&mut cpu, &VSUB, 1);
        match expected {
            Some(value) => assert_eq!(cpu.fp_regs[2], value, "{a:08x} - {b:08x}"),
            None => assert!(is_nan(cpu.fp_regs[2]), "{a:08x} - {b:08x}"),
        }
    }
}
