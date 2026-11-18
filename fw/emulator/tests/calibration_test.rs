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

//! Runs the timing calibration benchmarks in the emulator, and holds
//! the result to what a real STM32G474 measured for the same program.
//!
//! calibration/hardware_cycles_x1.txt was recorded from a moteus-x1
//! with calibration/run_on_hardware.py.

use moteus_emulator::{Board, Machine};
use zmu_cortex_m::bus::Bus;

// These must match calibration/make_benchmarks.py.
const CODE: u32 = 0x1000_0000;
const RESULTS: u32 = 0x2000_0400;
const END_MARKER: u32 = 0xd0e5_d0e5;

/// Returns the name and cycle count of each benchmark, less the
/// overhead of timing it.
fn run_benchmarks() -> Vec<(String, i64)> {
    let path = std::env::var("BENCHMARKS").expect("BENCHMARKS must name the programs");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));

    // Each program is the names of its results, and its bytes.
    let mut programs: Vec<(Vec<String>, Vec<u8>)> = Vec::new();
    for line in text.lines() {
        if line.starts_with("program") {
            programs.push((Vec::new(), Vec::new()));
        } else if let Some(name) = line.strip_prefix("name ") {
            programs.last_mut().unwrap().0.push(name.to_string());
        } else if let Some(hex) = line.strip_prefix("code ") {
            let code = &mut programs.last_mut().unwrap().1;
            for i in (0..hex.len()).step_by(2) {
                code.push(u8::from_str_radix(&hex[i..i + 2], 16).unwrap());
            }
        }
    }

    let mut result = Vec::new();
    for (names, program) in programs {
        let mut machine = Machine::with_ram_program(&program, CODE, Board::R5).unwrap();
        if let Err(error) = machine.run_until_spin(1_000_000) {
            // r5 points at where the next result goes.
            let index = (machine.cpu.r0_12[5].wrapping_sub(RESULTS) / 4) as usize;
            let name = names.get(index).map_or("?", |x| x.as_str());
            panic!("{name}: {error}");
        }

        let reached = (machine.cpu.r0_12[5].wrapping_sub(RESULTS) / 4) as usize;
        let mut read = |index: usize| machine.cpu.read32(RESULTS + 4 * index as u32).unwrap();
        assert_eq!(
            read(names.len()),
            END_MARKER,
            "the benchmarks stopped at {}",
            names.get(reached).map_or("?", |x| x.as_str())
        );
        // The first of each program is empty.
        let overhead = i64::from(read(0));
        for (i, name) in names.into_iter().enumerate() {
            result.push((name, i64::from(read(i)) - overhead));
        }
    }
    result
}

/// Benchmarks where the model is known to differ from hardware, as
/// (name, hardware, model).  Roughly in order, these are:
///
///  * Unaligned accesses, since the model does not know the alignment
///    of an address
///  * Exceptions reached through a handler which jumps to another
///  * Runs of stores to the RAM being executed from, which the
///    firmware does not do
///  * Long runs of loads of literals, and of floating point code
///    which does not start on a word boundary, where what the
///    prefetcher has in hand is not tracked quite right
///  * A load from SRAM2 after a narrow store to SRAM1 in the middle
///    of other things, which costs a cycle more than that pair alone
///  * Reading the CORDIC one instruction after writing to it
///
/// The table is generated from a comparison, so that a change to the
/// model which fixes or breaks any one of them is noticed.
const KNOWN_DIFFERENCES: &[(&str, i64, i64)] = &[
    ("ldr_unaligned", 3, 1),
    ("ldr_unaligned_half", 2, 1),
    ("pendsv", 34, 32),
    ("pendsv_fp_context", 34, 32),
    ("pendsv_fp_used", 71, 69),
    ("ldr_pool_bx_use", 7, 6),
    ("str_x4@ccm_alias", 8, 5),
    ("str_add_x4@ccm_alias", 12, 8),
    ("str_x4@ccm", 8, 5),
    ("str_add_x4@ccm", 12, 8),
    ("bcc_to_ldr_sram2_aligned", 5, 4),
    ("vldr_vmul_x4_sram2_unaligned", 17, 18),
    ("fp_block_x4_unaligned", 29, 32),
    ("mixed_block_x4", 35, 32),
    ("fp_block_x8_unaligned", 57, 63),
    ("mixed_block_x8", 71, 64),
    ("fp_block_x16_unaligned", 113, 124),
    ("mixed_block_x16", 143, 128),
    ("run_vstr_sram2_x12_unaligned", 18, 20),
    ("run_str.w_sram2_x12_unaligned", 18, 20),
    ("blk_fp_mid_x12_unaligned", 85, 84),
    ("blk_fp_last_x12_unaligned", 85, 75),
    ("blk_ldr_str_x12_unaligned", 36, 37),
    ("blk_vldr_vadd3_x12_unaligned", 85, 84),
    ("blk_vadd3_vstr_x12_unaligned", 72, 73),
    ("blk_fp_sram1_x12_unaligned", 73, 85),
    ("prefix_fp_7", 13, 14),
    ("prefix_fp_12", 22, 21),
    ("prefix_mixed_7", 11, 10),
    ("prefix_mixed_8", 13, 12),
    ("prefix_mixed_9", 14, 13),
    ("prefix_mixed_10", 15, 14),
    ("prefix_mixed_11", 16, 15),
    ("prefix_mixed_12", 17, 16),
    ("vcmp_not_taken_w_wide_unaligned", 11, 10),
    ("vcmp_x2_wide_unaligned", 16, 14),
    ("vcmp_x2_narrow_unaligned", 14, 13),
    ("vcmp_not_taken_vldr_x2_vmul_vfma_unaligned", 14, 15),
    ("prefix_fw_vcmp_vldr_4_unaligned", 8, 7),
    ("prefix_fw_vcmp_vldr_5_unaligned", 9, 10),
    ("prefix_fw_vcmp_vldr_6_unaligned", 11, 12),
    ("prefix_fw_vcmp_vldr_7_unaligned", 12, 13),
    ("prefix_fw_vcmp_vldr_8_unaligned", 13, 14),
    ("prefix_fw_vcmp_vldr_9_unaligned", 14, 15),
    ("prefix_fw_pointer_vldr_other_base_4_unaligned", 8, 10),
    ("prefix_fw_pointer_vldr_other_base_5_unaligned", 10, 12),
    ("prefix_fw_pointer_vldr_other_base_6_unaligned", 11, 13),
    ("prefix_fw_pointer_ldr_vldr_4_aligned", 9, 10),
    ("prefix_fw_pointer_ldr_vldr_5_aligned", 11, 12),
    ("prefix_fw_pointer_ldr_vldr_6_aligned", 12, 13),
    ("cordic_after1", 14, 16),
    ("cordic_core_after1", 14, 16),
    ("prefix_fw_taken_literal_3_aligned", 4, 3),
    ("prefix_fw_taken_literal_3_unaligned", 5, 4),
    ("prefix_fw_taken_literal_10_unaligned", 14, 15),
    ("prefix_fw_taken_literal_11_unaligned", 15, 16),
    ("prefix_fw_taken_w_literal_3_aligned", 4, 3),
    ("prefix_fw_taken_w_literal_9_aligned", 10, 11),
    ("prefix_fw_taken_w_literal_9_unaligned", 12, 13),
    ("prefix_fw_taken_w_literal_10_aligned", 11, 12),
    ("prefix_fw_taken_w_literal_10_unaligned", 13, 14),
    ("prefix_fw_vldr_literal_vabs_4_aligned", 8, 6),
    ("prefix_fw_vldr_literal_vabs_4_unaligned", 9, 7),
    ("prefix_fw_vldr_literal_vabs_5_aligned", 8, 7),
    ("prefix_fw_vldr_literal_vabs_5_unaligned", 9, 8),
    ("prefix_fw_vldr_literal_vabs_7_aligned", 9, 8),
    ("prefix_fw_vldr_literal_vabs_7_unaligned", 11, 10),
    ("prefix_fw_vldr_literal_vabs_8_aligned", 10, 9),
    ("prefix_fw_vldr_literal_vabs_8_unaligned", 12, 11),
    ("prefix_fw_vldr_literal_vabs_9_aligned", 11, 10),
    ("prefix_fw_vldr_literal_vabs_9_unaligned", 13, 12),
    ("prefix_fw_vldr_literal_vabs_10_aligned", 12, 11),
    ("prefix_fw_vldr_literal_vabs_10_unaligned", 14, 13),
    ("prefix_fw_literal_x2_vcvt_4_aligned", 7, 6),
    ("prefix_fw_literal_x2_vcvt_5_aligned", 8, 7),
    ("prefix_fw_literal_x2_vcvt_6_aligned", 9, 8),
    ("prefix_fw_add_literal_add_4_unaligned", 8, 7),
    ("prefix_fw_add_literal_add_5_unaligned", 9, 8),
    ("prefix_fw_add_literal_add_6_unaligned", 10, 9),
    ("svc_pends_pendsv", 38, 37),
    ("prefix_fw2_clamp_19_unaligned", 33, 34),
    ("prefix_fw2_clamp_20_unaligned", 35, 36),
    ("prefix_fw2_clamp_21_unaligned", 36, 37),
    ("prefix_fw2_clamp_22_unaligned", 37, 38),
    ("prefix_fw2_limit_5_aligned", 8, 7),
    ("prefix_fw2_limit_6_aligned", 10, 9),
    ("prefix_fw2_limit_6_unaligned", 10, 9),
    ("prefix_fw2_limit_7_aligned", 12, 11),
    ("prefix_fw2_limit_8_aligned", 15, 14),
    ("prefix_fw2_limit_8_unaligned", 15, 14),
    ("prefix_fw2_limit_9_aligned", 18, 17),
    ("prefix_fw2_limit_9_unaligned", 18, 17),
    ("prefix_fw2_limit_10_aligned", 20, 19),
    ("prefix_fw2_limit_10_unaligned", 20, 19),
    ("prefix_fw2_limit_11_unaligned", 22, 21),
    ("prefix_fw2_limit_12_aligned", 24, 23),
    ("prefix_fw2_limit_12_unaligned", 23, 22),
    ("prefix_fw2_limit_13_aligned", 27, 26),
    ("prefix_fw2_limit_13_unaligned", 26, 25),
    ("prefix_fw2_limit_14_aligned", 28, 27),
    ("prefix_fw2_limit_14_unaligned", 27, 26),
    ("prefix_fw2_limit_19_aligned", 33, 31),
    ("prefix_fw2_limit_19_unaligned", 31, 30),
    ("prefix_fw2_limit_20_aligned", 35, 33),
    ("prefix_fw2_limit_20_unaligned", 33, 32),
    ("prefix_fw2_limit_21_aligned", 36, 34),
    ("prefix_fw2_limit_21_unaligned", 34, 33),
    ("prefix_fw2_limit_22_aligned", 38, 35),
    ("prefix_fw2_limit_22_unaligned", 36, 34),
    ("prefix_fw2_limit_23_aligned", 39, 37),
    ("prefix_fw2_limit_23_unaligned", 37, 36),
    ("prefix_fw2_limit_27_aligned", 42, 40),
    ("prefix_fw2_limit_27_unaligned", 40, 39),
    ("prefix_fw2_limit_28_aligned", 43, 41),
    ("prefix_fw2_limit_28_unaligned", 41, 40),
    ("prefix_fw2_limit_29_aligned", 44, 42),
    ("prefix_fw2_limit_29_unaligned", 42, 41),
    ("prefix_fw2_limit_30_aligned", 45, 43),
    ("prefix_fw2_limit_30_unaligned", 43, 42),
    ("prefix_fw2_gather_9_unaligned", 16, 17),
    ("prefix_fw2_threshold_3_aligned", 5, 6),
    ("prefix_fw2_threshold_4_aligned", 7, 8),
    ("prefix_fw2_threshold_5_aligned", 10, 11),
    ("prefix_fw2_threshold_6_aligned", 12, 13),
    ("prefix_fw2_threshold_8_unaligned", 16, 15),
    ("prefix_fw2_threshold_9_unaligned", 17, 16),
    ("prefix_fw2_threshold_10_unaligned", 19, 18),
    ("prefix_fw2_threshold_11_unaligned", 20, 19),
    ("prefix_fw2_threshold_12_unaligned", 21, 20),
    ("prefix_fw2_threshold_13_unaligned", 22, 21),
    ("prefix_fw2_nan_check_11_aligned", 13, 12),
    ("prefix_fw2_nan_check_12_aligned", 15, 14),
    ("prefix_fw2_nan_check_13_aligned", 16, 15),
    ("prefix_fw2_nan_check_17_aligned", 19, 18),
    ("prefix_fw2_nan_check_23_aligned", 24, 23),
    ("prefix_fw2_nan_check_24_aligned", 27, 26),
    ("prefix_fw2_nan_check_25_aligned", 29, 28),
    ("prefix_fw2_nan_check_26_aligned", 30, 29),
    ("prefix_fw2_nan_check_27_aligned", 31, 30),
    ("prefix_fw2_call_15_aligned", 39, 38),
    ("prefix_fw2_call_15_unaligned", 40, 39),
    ("prefix_fw2_call_16_aligned", 41, 40),
    ("prefix_fw2_call_16_unaligned", 42, 41),
    ("prefix_fw2_call_17_aligned", 42, 41),
    ("prefix_fw2_call_17_unaligned", 43, 42),
    ("prefix_fw2_call_18_aligned", 43, 42),
    ("prefix_fw2_call_18_unaligned", 44, 43),
    ("call_push9_ldr_str_vldr_pop9_then_adds_aligned", 39, 38),
    ("call_push9_ldr_str_vldr_pop9_then_vldr_aligned", 41, 40),
    ("call_push9_ldr_str_vldr_pop9_then_adds_unaligned", 40, 39),
    ("call_push9_ldr_str_vldr_pop9_then_vldr_unaligned", 42, 41),
    ("ppb_ldr_str_add.w_unaligned", 8, 7),
    ("timer_interrupt_delay0", 829, 830),
    ("timer_interrupt_delay1", 830, 831),
    ("timer_interrupt_delay2", 831, 832),
    ("timer_interrupt_delay3", 832, 833),
    ("timer_interrupt_div2_delay1", 807, 808),
    ("timer_interrupt_div2_delay3", 809, 810),
];

/// Groups of benchmarks, by the start of their names, which the model
/// only approximates, with how many cycles it may be out by.
const APPROXIMATE: &[(&str, i64)] = &[
    // The bridge to the peripheral buses, above all with them at
    // half speed as the firmware has them.  What it does depends on
    // more of what came before than is modelled.
    ("apb_", 5),
    ("spi_probe", 3),
];

#[test]
fn model_matches_hardware() {
    let model = run_benchmarks();

    // For comparing by hand with calibration/compare.py.
    if let Ok(path) = std::env::var("CALIBRATION_WRITE") {
        let text: String = model
            .iter()
            .map(|(name, cycles)| format!("{name} {cycles}\n"))
            .collect();
        std::fs::write(&path, format!("# Predicted by the emulator.\n{text}")).unwrap();
    }

    let path = std::env::var("HARDWARE_CYCLES").expect("HARDWARE_CYCLES must name the file");
    let hardware: Vec<(String, i64)> = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{path}: {e}"))
        .lines()
        .filter(|l| !l.starts_with('#'))
        .map(|l| {
            let (name, cycles) = l.split_once(' ').unwrap();
            (name.to_string(), cycles.parse().unwrap())
        })
        .collect();

    assert_eq!(
        model.iter().map(|x| &x.0).collect::<Vec<_>>(),
        hardware.iter().map(|x| &x.0).collect::<Vec<_>>(),
        "the measurements are not of the current benchmarks"
    );

    let mut problems = Vec::new();
    for ((name, predicted), (_, measured)) in model.iter().zip(hardware.iter()) {
        let known = KNOWN_DIFFERENCES.iter().find(|x| x.0 == name);
        let expected = match known {
            Some((_, hardware, model)) => {
                assert_eq!(hardware, measured, "{name} is out of date");
                *model
            }
            None => *measured,
        };
        let tolerance = APPROXIMATE
            .iter()
            .find(|x| name.starts_with(x.0))
            .map_or(0, |x| x.1);
        if (*predicted - expected).abs() > tolerance {
            problems.push(format!(
                "  {name}: hardware {measured}, model {predicted}\n"
            ));
        }
    }
    assert!(
        problems.is_empty(),
        "The timing model no longer agrees with hardware:\n{}",
        problems.concat()
    );
    assert!(model.len() > 300);
}
