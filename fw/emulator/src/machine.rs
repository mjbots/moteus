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

//! Ties the simulated processor, peripherals and firmware image
//! together.

use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

use moteus::reader;
use zmu_cortex_m::bus::Bus;
use zmu_cortex_m::core::exception::{Exception, ExceptionHandling};
use zmu_cortex_m::core::fault::FaultTrapMode;
use zmu_cortex_m::core::register::BaseReg;
use zmu_cortex_m::core::reset::Reset;
use zmu_cortex_m::core::thumb::ThumbCode;
use zmu_cortex_m::executor::Executor;
use zmu_cortex_m::memory::map::MemoryMapConfig;
use zmu_cortex_m::Processor;

use crate::elf::Elf;
use crate::soc::{Board, CanFrame, Soc, SocBus};
use crate::timing::{self, DataClass, Decoded, ExceptionEvent, Executed, Kind, Timing};

const FLASH_BASE: u32 = 0x0800_0000;
const FLASH_SIZE: usize = 0x8_0000;

const PENDSV: u32 = 14;

const DEFAULT_BUS_VOLTAGE: f32 = 24.0;

/// The default CAN ID of a moteus.
const DEVICE_ID: u32 = 1;
const MULTIPLEX_CLIENT_DATA: u8 = 0x40;
const MULTIPLEX_SERVER_DATA: u8 = 0x41;
const MULTIPLEX_CLIENT_POLL: u8 = 0x42;
const MULTIPLEX_PAD: u8 = 0x50;
const DIAGNOSTIC_CHANNEL: u8 = 1;
/// How many times to ask for more of a response.
const COMMAND_POLL_LIMIT: u32 = 2000;
/// How many instructions to allow the firmware to act on a frame.
const COMMAND_STEP_LIMIT: u32 = 2_000_000;

/// Everything which the firmware sent with the given heading, such as
/// "emit servo_stats", in what was received on the diagnostic
/// channel.  Each is the heading, a line ending, a 32 bit size, and
/// that many bytes.  One which has not all arrived is left out.
fn framed<'a>(received: &'a [u8], heading: &str) -> Vec<&'a [u8]> {
    let heading = format!("{heading}\r\n").into_bytes();
    let mut result = Vec::new();
    let mut position = 0;
    while let Some(found) = received[position..]
        .windows(heading.len())
        .position(|window| window == heading)
    {
        let start = position + found + heading.len();
        let Some(size) = received.get(start..start + 4) else {
            break;
        };
        let end = start + 4 + u32::from_le_bytes(size.try_into().unwrap()) as usize;
        let Some(contents) = received.get(start + 4..end) else {
            break;
        };
        result.push(contents);
        position = end;
    }
    result
}

/// One execution of the control interrupt, measured from entry into
/// the PWM timer handler until the PendSV handler which completes
/// the work returns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IsrSample {
    pub instructions: u64,
    /// Modelled clock cycles from the first instruction of the timer
    /// handler.
    pub cycles: u64,
    /// The firmware's own measure: timer counts, which are clock
    /// cycles, from the peak of the PWM timer to the end of the
    /// control interrupt.
    pub final_timer: u32,
}

pub struct Machine {
    pub cpu: Processor,
    pub elf: Elf,
    /// Every completed control interrupt since reset.
    pub isr_samples: Vec<IsrSample>,

    /// For each function, the (instructions, cycles) spent in it
    /// within the control interrupt, when profiling is enabled.
    pub profile: Option<HashMap<usize, (u64, u64)>>,

    /// The cycles spent at each address, when enabled.  Unlike
    /// `profile` this covers everything, not only the control
    /// interrupt, and needs no symbols.
    pub pc_profile: Option<HashMap<u32, u64>>,
    /// The part of `pc_profile` spent within the control interrupt.
    pub pc_profile_isr: HashMap<u32, u64>,
    /// For each address reached within a control interrupt, the
    /// fewest cycles from the start of its handler to first arriving
    /// there, and the index of the interrupt's instruction at which
    /// that was.  This is comparable with what a debugger sees on
    /// stopping at a breakpoint there, having zeroed the cycle
    /// counter at one on the handler.
    pub pc_arrival: HashMap<u32, (u32, u64)>,
    /// What the firmware has sent on the diagnostic channel while
    /// being polled by `run_isrs_polling`.
    pub streamed: Vec<u8>,
    /// The schema of each telemetry channel which has been read.
    schemas: HashMap<String, reader::Type>,
    apb_half_speed: bool,
    arrived_this_isr: std::collections::HashSet<u32>,
    /// How each address profiled was classified for timing.
    pub pc_kinds: HashMap<u32, String>,
    /// Log every instruction executed in this function, by index
    /// into `elf.functions`, to stderr.
    pub trace_function: Option<usize>,

    pub soc: Rc<RefCell<Soc>>,
    timing: Timing,
    /// What `timing` needs to know of the instruction at each address.
    decode_cache: HashMap<u32, (Decoded, u32, u32)>,
    error_addr: Option<u32>,
    assert_addrs: Vec<u32>,
    die_addr: Option<u32>,
    last_exception: u32,
    /// The (instruction, cycle) counts at the most recent entry to the
    /// PWM timer handler which has not yet completed.
    isr_start: Option<(u64, u64)>,
}

impl Machine {
    /// Create a machine with the given ELF firmware image programmed
    /// into flash, held in reset.
    pub fn new(image: &[u8], board: Board) -> Result<Self, String> {
        let elf = Elf::parse(image)?;

        // Erased flash reads as all ones.
        let mut flash = vec![0xffu8; FLASH_SIZE];
        for segment in &elf.segments {
            let start = segment
                .paddr
                .checked_sub(FLASH_BASE)
                .ok_or("segment below flash")? as usize;
            flash
                .get_mut(start..start + segment.data.len())
                .ok_or("segment beyond flash")?
                .copy_from_slice(&segment.data);
        }

        let mut result = Self::with_flash(&flash, elf, board);
        result
            .cpu
            .reset()
            .map_err(|fault| format!("reset: {fault}"))?;
        result.cpu.running = true;
        Ok(result)
    }

    /// Create a machine with the given contents of flash, as read
    /// back from a real controller, held in reset.  With no symbols,
    /// firmware errors cannot be recognized.
    pub fn with_flash_image(image: &[u8], board: Board) -> Result<Self, String> {
        let mut flash = vec![0xffu8; FLASH_SIZE];
        flash
            .get_mut(..image.len())
            .ok_or("image larger than flash")?
            .copy_from_slice(image);
        let mut result = Self::with_flash(&flash, Elf::empty(), board);
        result
            .cpu
            .reset()
            .map_err(|fault| format!("reset: {fault}"))?;
        result.cpu.running = true;
        Ok(result)
    }

    /// Create a machine with blank flash and the given program in RAM
    /// at `address`, about to execute its first instruction.  This is
    /// how a debugger would run something without firmware.
    pub fn with_ram_program(program: &[u8], address: u32, board: Board) -> Result<Self, String> {
        let flash = vec![0xffu8; FLASH_SIZE];
        let mut result = Self::with_flash(&flash, Elf::empty(), board);
        for (i, byte) in program.iter().enumerate() {
            result
                .cpu
                .write8(address + i as u32, *byte)
                .map_err(|fault| format!("loading program: {fault}"))?;
        }
        // Reset, so that everything is in its initial state, and then
        // do what a debugger would to start the program instead of
        // what the blank flash calls for.
        result
            .cpu
            .reset()
            .map_err(|fault| format!("reset: {fault}"))?;
        result.cpu.psr.value = 1 << 24;
        result.cpu.set_pc(address);
        result.cpu.running = true;
        Ok(result)
    }

    fn with_flash(flash: &[u8], elf: Elf, board: Board) -> Self {
        let soc = Rc::new(RefCell::new(Soc::new(board)));
        soc.borrow_mut().set_bus_voltage(DEFAULT_BUS_VOLTAGE);
        let mut cpu = Processor::new();
        cpu.device(Some(Box::new(SocBus(soc.clone()))));
        cpu.memory_map(Some(MemoryMapConfig::new(FLASH_BASE, 0, FLASH_SIZE)));
        cpu.fault_trap_mode(FaultTrapMode::all());
        cpu.flash_memory(FLASH_SIZE, flash);
        cpu.cache_instructions();

        Self {
            cpu,
            error_addr: elf.function("error"),
            assert_addrs: [
                "mbed_assert_internal",
                "_ZN5mjlib4base16assertion_failedEPKcS2_i",
            ]
            .iter()
            .filter_map(|name| elf.function(name))
            .collect(),
            die_addr: elf.function("mbed_die"),
            elf,
            isr_samples: Vec::new(),
            profile: None,
            pc_profile: None,
            pc_profile_isr: HashMap::new(),
            pc_arrival: HashMap::new(),
            streamed: Vec::new(),
            schemas: HashMap::new(),
            apb_half_speed: false,
            arrived_this_isr: std::collections::HashSet::new(),
            pc_kinds: HashMap::new(),
            trace_function: None,
            soc,
            timing: Timing::new(timing::Params::default()),
            decode_cache: HashMap::new(),
            last_exception: 0,
            isr_start: None,
        }
    }

    fn read_cstr(&mut self, mut addr: u32) -> String {
        let mut result = String::new();
        while let Ok(c) = self.cpu.read8(addr) {
            if c == 0 || result.len() > 200 {
                break;
            }
            result.push(c as char);
            addr += 1;
        }
        result
    }

    /// Execute one instruction.  An error is returned if the firmware
    /// faults or reports a fatal error.
    pub fn step(&mut self) -> Result<(), String> {
        let pc = self.cpu.get_pc();
        if Some(pc) == self.error_addr {
            // This is the mbed fatal error handler.
            let message = self.read_cstr(self.cpu.r0_12[0]);
            let caller = self.elf.symbolize(self.cpu.lr & !1);
            return Err(format!("firmware error from {caller}: {message:?}"));
        }
        if self.assert_addrs.contains(&pc) {
            // These all take the expression, file and line.
            let expression = self.read_cstr(self.cpu.r0_12[0]);
            let file = self.read_cstr(self.cpu.r0_12[1]);
            let line = self.cpu.r0_12[2];
            return Err(format!(
                "firmware assertion failed: {expression:?} at {file}:{line}\n  {}",
                self.backtrace().join("\n  ")
            ));
        }
        if Some(pc) == self.die_addr {
            return Err(format!(
                "firmware called mbed_die:\n  {}",
                self.backtrace().join("\n  ")
            ));
        }

        {
            let mut soc = self.soc.borrow_mut();
            soc.cycles = self.cpu.cycle_count;
            if std::mem::take(&mut soc.code_written) {
                // Anything decoded from RAM may now be stale.
                self.cpu.clear_ram_instruction_cache();
                self.decode_cache.clear();
            }
        }
        if self.cpu.sleeping {
            self.cpu.step_sleep();
            self.cpu.cycle_count += 1;
        } else {
            let cycles_before = self.cpu.cycle_count;
            if self.pc_profile.is_some() {
                if let Some((start, start_cycles)) = self.isr_start {
                    if self.arrived_this_isr.insert(pc) {
                        let arrival = (
                            (self.cpu.cycle_count - start_cycles) as u32,
                            self.cpu.instruction_count - start,
                        );
                        let entry = self.pc_arrival.entry(pc).or_insert(arrival);
                        if arrival.0 < entry.0 {
                            *entry = arrival;
                        }
                    }
                } else {
                    self.arrived_this_isr.clear();
                }
            }
            self.step_instruction(pc);
            if let Some(profile) = self.pc_profile.as_mut() {
                let cycles = self.cpu.cycle_count - cycles_before;
                *profile.entry(pc).or_default() += cycles;
                if self.isr_start.is_some() {
                    *self.pc_profile_isr.entry(pc).or_default() += cycles;
                }
            }
            if self.isr_start.is_some() {
                if let Some(profile) = self.profile.as_mut() {
                    if let Some(index) = self.elf.function_index(pc) {
                        let entry = profile.entry(index).or_default();
                        entry.0 += 1;
                        entry.1 += self.cpu.cycle_count - cycles_before;
                    }
                }
            }
        }
        if let Some(context) = self.cpu.take_pending_fault_trap() {
            return Err(format!("{context} after {}", self.elf.symbolize(pc)));
        }

        // Deliver any interrupt which is now due before the next
        // instruction.
        let irq = {
            let mut soc = self.soc.borrow_mut();
            soc.cycles = self.cpu.cycle_count;
            soc.tick()
        };
        if let Some(n) = irq {
            self.pend_irq(n);
            let before = self.exception_number();
            self.cpu.check_exceptions();
            if self.exception_number() != before {
                let cycles = self.timing.exception_entry();
                self.add_cycles(cycles);
            }
        }

        self.track_isr();
        Ok(())
    }

    fn exception_number(&self) -> u32 {
        self.cpu.psr.value & 0x1ff
    }

    /// Advance time by an amount the simulator did not account for.
    fn add_cycles(&mut self, cycles: u32) {
        self.cpu.cycle_count += u64::from(cycles);
        if self.cpu.dwt_ctrl & 1 != 0 {
            self.cpu.dwt_cyccnt = self.cpu.dwt_cyccnt.wrapping_add(cycles);
        }
    }

    fn decode(&mut self, pc: u32) -> Option<(Decoded, u32, u32)> {
        if let Some(cached) = self.decode_cache.get(&pc) {
            return Some(*cached);
        }
        let result = self.decode_uncached(pc)?;
        self.decode_cache.insert(pc, result);
        Some(result)
    }

    fn decode_uncached(&self, pc: u32) -> Option<(Decoded, u32, u32)> {
        match self.cpu.fetch_code(pc).ok()? {
            ThumbCode::Thumb16 { opcode } => Some((timing::decode16(opcode), u32::from(opcode), 0)),
            ThumbCode::Thumb32 { opcode } => {
                let (hw1, hw2) = (opcode >> 16, opcode & 0xffff);
                Some((timing::decode32(hw1 as u16, hw2 as u16), hw1, hw2))
            }
            ThumbCode::Undefined => None,
        }
    }

    fn core_reg(&self, n: u32) -> u32 {
        match n {
            0..=12 => self.cpu.r0_12[n as usize],
            13 => self.cpu.msp,
            14 => self.cpu.lr,
            _ => 0,
        }
    }

    /// What kind of memory a load or store about to execute will
    /// access.  This goes by its base register, on the grounds that
    /// no offset is large enough to reach a different kind.
    fn data_class(&self, pc: u32, decoded: &Decoded) -> DataClass {
        let address = match decoded.base {
            timing::NO_BASE => return DataClass::None,
            15 => pc,
            n => self.core_reg(u32::from(n)),
        };
        // The CCM RAM also appears above SRAM2.
        let is_ccm = |a: u32| matches!(a >> 12, 0x10000..=0x10007 | 0x20018..=0x2001f);
        match address >> 12 {
            0x08000..=0x0807f => DataClass::Flash,
            0x40000..=0x4001f => DataClass::Apb,
            0x40020..=0x50fff => DataClass::Ahb,
            // Data and instruction fetches compete for the one memory.
            _ if is_ccm(address) && is_ccm(pc) => DataClass::CodeRam,
            0x20014..=0x20017 => DataClass::SlowRam,
            0xe0000..=0xe00ff => DataClass::Ppb,
            _ => DataClass::Ram,
        }
    }

    /// The address a load with an immediate offset reads, as near as
    /// matters for knowing which read of flash it falls in.
    fn data_address(&self, pc: u32, decoded: &Decoded, hw1: u32, hw2: u32) -> u32 {
        let base = match decoded.base {
            timing::NO_BASE => return 0,
            15 => (pc + 4) & !3,
            n => self.core_reg(u32::from(n)),
        };
        let offset = if decoded.size == 2 {
            match hw1 >> 11 {
                // A literal, and relative to the stack pointer.
                0b01001 | 0b10011 => (hw1 & 0xff) << 2,
                0b01101 => ((hw1 >> 6) & 0x1f) << 2,
                0b01111 => (hw1 >> 6) & 0x1f,
                0b10001 => ((hw1 >> 6) & 0x1f) << 1,
                _ => 0,
            }
        } else if (hw1 >> 9) & 0x7f == 0x76 {
            // VLDR, which can go either way.
            let offset = (hw2 & 0xff) << 2;
            if hw1 & 0x80 == 0 {
                return base.wrapping_sub(offset);
            }
            offset
        } else if hw1 & 0xfe80 == 0xf880 {
            hw2 & 0xfff
        } else if hw1 & 0xff7f == 0xf85f {
            // A literal, which can go either way.
            if hw1 & 0x80 == 0 {
                return base.wrapping_sub(hw2 & 0xfff);
            }
            hw2 & 0xfff
        } else {
            0
        };
        base.wrapping_add(offset)
    }

    /// Whether the instruction about to execute passes its IT block
    /// condition, if it is in one.
    fn condition_passed(&self) -> bool {
        let itstate = self.cpu.get_itstate();
        if itstate & 0xf == 0 {
            return true;
        }
        let psr = self.cpu.psr.value;
        let (n, z, c, v) = (
            psr & (1 << 31) != 0,
            psr & (1 << 30) != 0,
            psr & (1 << 29) != 0,
            psr & (1 << 28) != 0,
        );
        let cond = itstate >> 4;
        let base = match cond >> 1 {
            0 => z,
            1 => c,
            2 => n,
            3 => v,
            4 => c && !z,
            5 => n == v,
            6 => n == v && !z,
            _ => true,
        };
        if cond & 1 != 0 && cond != 0xf {
            !base
        } else {
            base
        }
    }

    /// The value an instruction is about to load into the PC, where
    /// that can be an exception return.
    fn pc_load_value(&mut self, decoded: &Decoded) -> Option<u32> {
        match decoded.kind {
            Kind::BranchRegister { rm } => Some(self.core_reg(u32::from(rm))),
            Kind::Multiple { count } if decoded.loads_pc => {
                // A pop: the PC is the last word.
                let sp = self.cpu.msp;
                self.cpu.read32(sp + 4 * (count - 1)).ok()
            }
            _ => None,
        }
    }

    /// Execute one instruction and correct the simulator's notion of
    /// how long it took.
    fn step_instruction(&mut self, pc: u32) {
        let Some((decoded, hw1, hw2)) = self.decode(pc) else {
            // This will fault.
            self.cpu.step();
            return;
        };

        let condition_passed = self.condition_passed();
        let exception_before = self.exception_number();
        let exc_return = if exception_before != 0 && condition_passed {
            self.pc_load_value(&decoded).filter(|v| *v >= 0xffff_ff00)
        } else {
            None
        };
        let regs_before = self.cpu.r0_12;
        let sp_before = (self.cpu.msp, self.cpu.psp);
        let lr_before = self.cpu.lr;
        let fp_before = self.cpu.fp_regs;
        let lazy_before = self.cpu.fpccr & 1 != 0;
        let (dividend, divisor) = (self.core_reg(hw1 & 0xf), self.core_reg(hw2 & 0xf));
        let data = self.data_class(pc, &decoded);
        let data_address = self.data_address(pc, &decoded, hw1, hw2);
        let stall_before = self.soc.borrow().stall_cycles;
        let cycles_before = self.cpu.cycle_count;
        let dwt_before = self.cpu.dwt_cyccnt;

        self.cpu.step();

        let next_pc = self.cpu.get_pc();
        let exception_after = self.exception_number();
        let event = if exc_return.is_some() {
            // If another exception was pending, the simulator has
            // already entered it.
            let vector = if exception_after != 0 {
                let vtor = self.cpu.vtor;
                self.cpu.read32(vtor + 4 * exception_after).ok()
            } else {
                None
            };
            if vector.map(|v| v & !1) == Some(next_pc) {
                ExceptionEvent::TailChain
            } else {
                // The floating point registers are only restored if
                // they were actually saved: if the lazy save is still
                // outstanding, the reserved space is just discarded.
                ExceptionEvent::Return {
                    fp_frame: exc_return.is_some_and(|v| v & 0x10 == 0) && !lazy_before,
                }
            }
        } else if exception_after != exception_before {
            ExceptionEvent::Entry
        } else {
            ExceptionEvent::None
        };

        let mut regs_written = 0u16;
        for (i, (before, after)) in regs_before.iter().zip(self.cpu.r0_12.iter()).enumerate() {
            regs_written |= u16::from(before != after) << i;
        }
        regs_written |= u16::from(sp_before != (self.cpu.msp, self.cpu.psp)) << 13;
        regs_written |= u16::from(lr_before != self.cpu.lr) << 14;
        let mut fp_written = 0u32;
        for (i, (before, after)) in fp_before.iter().zip(self.cpu.fp_regs.iter()).enumerate() {
            fp_written |= u32::from(before != after) << i;
        }

        if self.trace_function.is_some() && self.elf.function_index(pc) == self.trace_function {
            let mut changes = String::new();
            for i in 0..13 {
                if regs_written & (1 << i) != 0 {
                    changes.push_str(&format!(" r{i}={:#x}", self.cpu.r0_12[i]));
                }
            }
            for i in 0..32 {
                if fp_written & (1 << i) != 0 {
                    let bits = self.cpu.fp_regs[i];
                    changes.push_str(&format!(" s{i}={}({bits:#x})", f32::from_bits(bits)));
                }
            }
            eprintln!(
                "{} {hw1:04x} {hw2:04x}{}{changes}",
                self.elf.symbolize(pc),
                if condition_passed { "" } else { " (skipped)" }
            );
        }

        if condition_passed {
            regs_written |= if decoded.size == 2 {
                timing::writes16(hw1 as u16)
            } else {
                timing::writes32(hw1 as u16, hw2 as u16)
            };
        }
        let next_is_32bit = matches!(self.cpu.fetch_code(next_pc), Ok(ThumbCode::Thumb32 { .. }));

        let executed = Executed {
            pc,
            decoded,
            condition_passed,
            next_pc,
            next_is_32bit,
            regs_written,
            fp_written,
            dividend,
            divisor,
            data,
            data_address,
        };
        if self.pc_profile.is_some() {
            self.pc_kinds
                .entry(pc)
                .or_insert_with(|| format!("{:?}/{:?}", executed.decoded.kind, executed.data));
        }
        let mut cycles = self.timing.step(&executed, event);
        if lazy_before && self.cpu.fpccr & 1 == 0 && exc_return.is_none() {
            // The first floating point instruction of a handler saved
            // the interrupted context's registers.
            cycles += self.timing.lazy_fp_save();
        }
        // Follow what the firmware sets the clocks and the flash up
        // to do.
        self.timing.params.flash_wait_states = self.soc.borrow().flash_latency();
        let half_speed = self.soc.borrow().apb_divider() == 2;
        if half_speed != self.apb_half_speed {
            self.apb_half_speed = half_speed;
            self.timing.set_apb_half_speed(half_speed);
        }
        let stalled = (self.soc.borrow().stall_cycles - stall_before) as u32;
        self.timing.stall(stalled);
        cycles += stalled;

        // Replace the simulator's nominal cost with the modelled one.
        let nominal = (self.cpu.cycle_count - cycles_before) as u32;
        self.cpu.cycle_count = cycles_before + u64::from(cycles);
        // Unless the instruction itself wrote to the cycle counter.
        let counted = self.cpu.dwt_cyccnt == dwt_before.wrapping_add(nominal);
        if self.cpu.dwt_ctrl & 1 != 0 && counted {
            self.cpu.dwt_cyccnt = dwt_before.wrapping_add(cycles);
        }
    }

    fn pend_irq(&mut self, n: usize) {
        let mask = 1 << (n % 32);
        self.cpu.nvic_interrupt_pending[n / 32] |= mask;
        if self.cpu.nvic_interrupt_enabled[n / 32] & mask != 0 {
            self.cpu.set_exception_pending(Exception::Interrupt { n });
        }
    }

    fn track_isr(&mut self) {
        let exception = self.cpu.psr.value & 0x1ff;
        if exception == self.last_exception {
            return;
        }
        let now = (self.cpu.instruction_count, self.cpu.cycle_count);
        let pwm_exception = self.soc.borrow().pwm_irq().map(|n| n as u32 + 16);

        if Some(exception) == pwm_exception && self.isr_start.is_none() {
            self.isr_start = Some(now);
        }
        if exception == 0 && Some(self.last_exception) == pwm_exception {
            // The timer handler returned without requesting the rest
            // of the control interrupt, as it does for the update
            // event at the bottom of the count.
            self.isr_start = None;
        }
        if self.last_exception == PENDSV {
            if let Some(start) = self.isr_start.take() {
                self.isr_samples.push(IsrSample {
                    instructions: now.0 - start.0,
                    cycles: now.1 - start.1,
                    final_timer: self.soc.borrow().pwm_count_read.unwrap_or(0),
                });
            }
        }
        self.last_exception = exception;
    }

    /// Send one multiplex frame to the firmware, and wait for it to
    /// be received.
    fn can_send(&mut self, reply: bool, payload: &[u8]) -> Result<(), String> {
        // Frames are padded out to a valid CAN-FD length.
        let mut data = payload.to_vec();
        let length = [8, 12, 16, 20, 24, 32, 48, 64]
            .into_iter()
            .find(|&l| l >= data.len())
            .ok_or("frame too long")?;
        data.resize(length.max(data.len()), MULTIPLEX_PAD);
        let id = if reply { 0x8000 } else { 0 } | DEVICE_ID;
        self.soc.borrow_mut().can_send(CanFrame {
            id,
            extended: id > 0x7ff,
            data,
        });
        for _ in 0..COMMAND_STEP_LIMIT {
            if self.soc.borrow().can_rx_empty() {
                return Ok(());
            }
            self.step()?;
        }
        Err("the firmware did not receive a CAN frame".to_string())
    }

    /// Send a line on the diagnostic channel, as "tview" does, and
    /// return what the firmware says in response.
    pub fn command(&mut self, line: &str) -> Result<String, String> {
        let text = format!("{line}\n");
        for chunk in text.as_bytes().chunks(48) {
            let mut payload = vec![MULTIPLEX_CLIENT_DATA, DIAGNOSTIC_CHANNEL, chunk.len() as u8];
            payload.extend(chunk);
            self.can_send(false, &payload)?;
        }

        let mut response = String::new();
        let mut idle_polls = 0;
        for _ in 0..COMMAND_POLL_LIMIT {
            let length_before = response.len();
            self.soc.borrow_mut().can_tx.clear();
            self.can_send(true, &[MULTIPLEX_CLIENT_POLL, DIAGNOSTIC_CHANNEL, 48])?;
            for _ in 0..COMMAND_STEP_LIMIT {
                if !self.soc.borrow().can_tx.is_empty() {
                    break;
                }
                self.step()?;
            }
            let frames = std::mem::take(&mut self.soc.borrow_mut().can_tx);
            for frame in frames {
                let d = &frame.data;
                if d.len() >= 3 && d[0] == MULTIPLEX_SERVER_DATA && d[1] == DIAGNOSTIC_CHANNEL {
                    let end = (3 + d[2] as usize).min(d.len());
                    response.push_str(&String::from_utf8_lossy(&d[3..end]));
                }
            }
            // Most commands finish with "OK", but queries such as
            // "conf get" just emit a value.
            idle_polls = if response.len() == length_before {
                idle_polls + 1
            } else {
                0
            };
            if response.ends_with("OK\r\n") || (!response.is_empty() && idle_polls >= 3) {
                return Ok(response);
            }
            if let Some(error) = response.lines().find(|l| l.starts_with("ERR")) {
                return Err(format!("{line:?}: {error}"));
            }
        }
        Err(format!("{line:?}: no response, got {response:?}"))
    }

    /// Send a line on the diagnostic channel without waiting for a
    /// response, as is needed once the firmware has been asked to
    /// send telemetry of its own accord.
    pub fn send_line(&mut self, line: &str) -> Result<(), String> {
        let text = format!("{line}\n");
        for chunk in text.as_bytes().chunks(48) {
            let mut payload = vec![MULTIPLEX_CLIENT_DATA, DIAGNOSTIC_CHANNEL, chunk.len() as u8];
            payload.extend(chunk);
            self.can_send(false, &payload)?;
        }
        Ok(())
    }

    /// Ask for whatever the firmware has to send on the diagnostic
    /// channel, until it has no more, and return it.
    pub fn poll_diagnostic(&mut self) -> Result<Vec<u8>, String> {
        let mut received = Vec::new();
        for _ in 0..64 {
            self.soc.borrow_mut().can_tx.clear();
            self.can_send(true, &[MULTIPLEX_CLIENT_POLL, DIAGNOSTIC_CHANNEL, 48])?;
            for _ in 0..COMMAND_STEP_LIMIT {
                if !self.soc.borrow().can_tx.is_empty() {
                    break;
                }
                self.step()?;
            }
            let before = received.len();
            for frame in std::mem::take(&mut self.soc.borrow_mut().can_tx) {
                let d = &frame.data;
                if d.len() >= 3 && d[0] == MULTIPLEX_SERVER_DATA && d[1] == DIAGNOSTIC_CHANNEL {
                    let end = (3 + d[2] as usize).min(d.len());
                    received.extend(&d[3..end]);
                }
            }
            if received.len() == before {
                break;
            }
        }
        Ok(received)
    }

    /// Run 'count' control interrupts, polling the diagnostic channel
    /// every 'poll_cycles' as a client streaming telemetry does.
    pub fn run_isrs_polling(&mut self, count: usize, poll_cycles: u64) -> Result<(), String> {
        let target = self.isr_samples.len() + count;
        let mut next = self.cpu.cycle_count + poll_cycles;
        let limit = self.cpu.instruction_count + 400_000 * count as u64;
        while self.isr_samples.len() < target {
            if self.cpu.instruction_count >= limit {
                return Err("the control interrupt stopped running".to_string());
            }
            if self.cpu.cycle_count >= next {
                next = self.cpu.cycle_count + poll_cycles;
                let received = self.poll_diagnostic()?;
                self.streamed.extend(received);
            }
            self.step()?;
        }
        Ok(())
    }

    /// Ask the firmware to stream the telemetry which the
    /// dynamometer test does, at the rate it does.
    pub fn stream_telemetry(&mut self) -> Result<(), String> {
        // While it still answers one thing at a time.
        self.schema("servo_stats")?;
        for name in [
            "motor_position",
            "servo_stats",
            "servo_control",
            "servo_cmd",
            "git",
            "system_info",
            "firmware",
        ] {
            self.send_line(&format!("tel rate {name} 50"))?;
        }
        self.streamed.clear();
        Ok(())
    }

    /// Stop that, and discard whatever was still to come.
    pub fn stop_telemetry(&mut self) -> Result<(), String> {
        self.send_line("tel stop")?;
        for _ in 0..4 {
            self.run_isrs(20, 50_000_000)?;
            self.poll_diagnostic()?;
        }
        Ok(())
    }

    /// Send a line on the diagnostic channel which the firmware
    /// answers in binary under the given heading, as it does "tel
    /// schema" and "tel get", and return the contents of the answer.
    fn query(&mut self, line: &str, heading: &str) -> Result<Vec<u8>, String> {
        self.send_line(line)?;
        let mut received = Vec::new();
        for _ in 0..COMMAND_POLL_LIMIT {
            received.extend(self.poll_diagnostic()?);
            if let Some(contents) = framed(&received, heading).first() {
                return Ok(contents.to_vec());
            }
            if let Some(error) = received
                .split(|&byte| byte == b'\n')
                .find(|line| line.starts_with(b"ERR"))
            {
                return Err(format!("{line:?}: {}", String::from_utf8_lossy(error)));
            }
        }
        Err(format!("{line:?}: no response"))
    }

    /// How the firmware describes the named telemetry channel.  Do
    /// not ask for one which has not been asked for before while the
    /// firmware is streaming telemetry.
    pub fn schema(&mut self, name: &str) -> Result<reader::Type, String> {
        if let Some(schema) = self.schemas.get(name) {
            return Ok(schema.clone());
        }
        let contents = self.query(&format!("tel schema {name}"), &format!("schema {name}"))?;
        let schema = reader::Type::from_binary(&contents).map_err(|e| format!("{name}: {e}"))?;
        self.schemas.insert(name.to_string(), schema.clone());
        Ok(schema)
    }

    /// The current value of the named telemetry channel, such as
    /// "servo_stats".
    pub fn telemetry(&mut self, name: &str) -> Result<reader::Value, String> {
        let schema = self.schema(name)?;
        let contents = self.query(&format!("tel get {name}"), &format!("emit {name}"))?;
        schema.read(&contents).map_err(|e| format!("{name}: {e}"))
    }

    /// Each value of the named telemetry channel which the firmware
    /// streamed while `run_isrs_polling` ran.  For servo_stats this
    /// is what the dynamometer test checks, and it is not a fair
    /// sample of every interrupt: the firmware is busy sending when
    /// it takes these.
    pub fn streamed_telemetry(&mut self, name: &str) -> Result<Vec<reader::Value>, String> {
        let schema = self
            .schemas
            .get(name)
            .ok_or(format!("the schema of {name} was not read first"))?;
        framed(&self.streamed, &format!("emit {name}"))
            .into_iter()
            .map(|contents| schema.read(contents).map_err(|e| format!("{name}: {e}")))
            .collect()
    }

    /// Read a numeric configuration value from the firmware.
    pub fn config(&mut self, name: &str) -> Result<f32, String> {
        let response = self.command(&format!("conf get {name}"))?;
        response
            .trim()
            .parse()
            .map_err(|e| format!("conf get {name}: {response:?}: {e}"))
    }

    /// Connect a simulated motor to the phase outputs, current sense
    /// inputs and on-board encoder.  The firmware still has to be
    /// configured to suit it.
    pub fn attach_motor(&mut self) -> Result<(), String> {
        // The same scale the firmware applies in reverse.
        let sense_ohm = self.config("servo.current_sense_ohm")?;
        let gain = self.config("drv8323_conf.csa_gain")?;
        let amps_per_count = 3.3 / (4096.0 * sense_ohm * gain);
        self.soc.borrow_mut().attach_motor(amps_per_count);
        Ok(())
    }

    /// Run until the program reaches a branch to itself, which is how
    /// a bare metal program says it is finished.
    pub fn run_until_spin(&mut self, max_instructions: u64) -> Result<(), String> {
        const BRANCH_TO_SELF: u16 = 0xe7fe;
        let limit = self.cpu.instruction_count + max_instructions;
        while self.cpu.instruction_count < limit {
            let pc = self.cpu.get_pc();
            if let Ok(ThumbCode::Thumb16 { opcode }) = self.cpu.fetch_code(pc) {
                if opcode == BRANCH_TO_SELF {
                    return Ok(());
                }
            }
            self.step()?;
        }
        Err(format!(
            "still running after {max_instructions} instructions"
        ))
    }

    /// The functions which appear to be on the call stack, innermost
    /// first, found by scanning the stack for return addresses.
    pub fn backtrace(&mut self) -> Vec<String> {
        let mut result = vec![self.elf.symbolize(self.cpu.get_pc())];
        let sp = self.cpu.msp;
        for i in 0..512 {
            let Ok(word) = self.cpu.read32(sp + 4 * i) else {
                break;
            };
            if word & 1 != 0 && self.elf.function_index(word & !1).is_some() {
                result.push(self.elf.symbolize(word & !1));
            }
        }
        result
    }

    /// Run until `count` further control interrupts have completed,
    /// giving up after `max_instructions`.
    pub fn run_isrs(&mut self, count: usize, max_instructions: u64) -> Result<(), String> {
        let target = self.isr_samples.len() + count;
        let limit = self.cpu.instruction_count + max_instructions;
        while self.isr_samples.len() < target {
            if self.cpu.instruction_count >= limit {
                return Err(format!(
                    "only {} of {count} control interrupts completed in {max_instructions} \
                     instructions, at:\n  {}",
                    self.isr_samples.len() + count - target,
                    self.backtrace().join("\n  ")
                ));
            }
            self.step()?;
        }
        Ok(())
    }
}
