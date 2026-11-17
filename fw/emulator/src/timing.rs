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

//! A model of how many clock cycles the Cortex-M4F in an STM32G474
//! takes to execute each instruction.
//!
//! This is intentionally independent of the instruction set
//! simulator: it classifies raw opcodes itself and is told what
//! happened when each one executed.
//!
//! The starting point is the Cortex-M4 Technical Reference Manual
//! (r0p1, ARM 100166) sections 3.3 and 7.3.  That leaves a good deal
//! unsaid, so the rules here have been fitted to measurements of a
//! real STM32G474, a moteus-x1, made with the benchmarks in
//! `calibration/`.  Rules which come from those measurements rather
//! than from the manual are marked MEASURED.
//!
//! What has not been measured, and so remains as the documentation
//! describes it, is marked ASSUMPTION.  Most notably that is
//! everything about executing from flash.

const FLASH_START: u32 = 0x0800_0000;
const FLASH_END: u32 = 0x0808_0000;

/// The tunable constants of the model.
#[derive(Debug, Clone)]
pub struct Params {
    /// The pipeline refill after a taken branch, which the TRM gives
    /// as 1 to 3 cycles.  MEASURED: 1 for a direct branch or a return
    /// through the link register.
    pub refill_direct: u32,
    /// MEASURED: 2 for CBZ, CBNZ and a branch to any other register.
    pub refill_indirect: u32,
    /// MEASURED: 3 when the PC is loaded from memory.
    pub refill_load: u32,
    /// MEASURED: A 32 bit target instruction which straddles a word
    /// boundary takes a further cycle to fetch.
    pub refill_unaligned32: u32,
    /// TRM: the time from an exception being raised to its handler's
    /// first instruction.
    pub exception_entry: u32,
    /// MEASURED: entry plus a return by BX LR is 19 cycles.  That
    /// total is split here on the assumption that entry is as the
    /// TRM says.
    pub exception_return: u32,
    /// MEASURED: a return which restores floating point registers
    /// that a handler saved.  A return when no handler used the FPU,
    /// so that nothing was saved, costs the same as any other.
    pub exception_return_fp: u32,
    /// MEASURED: from one handler's return to the first instruction
    /// of another which it had made pending.
    pub tail_chain: u32,
    /// MEASURED: the deferred save of S0-S15 and FPSCR when a handler
    /// first uses the FPU.  Only its sum with the extra cost of the
    /// restore above is known, at 36 cycles.
    pub lazy_fp_save: u32,
    /// Flash wait states, which follow what the firmware sets.
    pub flash_wait_states: u32,
    /// Lines in the flash instruction and data caches.
    ///
    /// RM0440 gives the instruction cache 32.  MEASURED: the one
    /// function in flash which the control interrupt calls misses on
    /// every call, which the main loop only brings about here with
    /// fewer than that.  Something else must be filling the real one,
    /// and until what that is is known, this stands in for it.
    pub icache_lines: usize,
    pub dcache_lines: usize,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            refill_direct: 1,
            refill_indirect: 2,
            refill_load: 3,
            refill_unaligned32: 1,
            exception_entry: 12,
            exception_return: 7,
            exception_return_fp: 25,
            tail_chain: 9,
            lazy_fp_save: 18,
            flash_wait_states: 4,
            icache_lines: 16,
            dcache_lines: 8,
        }
    }
}

/// The timing relevant class of an instruction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// Anything which takes a single cycle.
    Simple,
    Nop,
    /// Data processing which may write the PC.
    DataMaybePc,
    Divide {
        signed: bool,
    },
    Load,
    /// A store with an immediate offset.
    Store,
    StoreRegOffset,
    /// LDRD, STRD and the exclusive access instructions.
    DualOrExclusive {
        cycles: u32,
    },
    /// LDM, STM, PUSH and POP of `count` registers.
    Multiple {
        count: u32,
    },
    /// B, BL and conditional branches.
    Branch,
    /// BX and BLX.
    BranchRegister {
        rm: u8,
    },
    /// CBZ and CBNZ.
    CompareBranch,
    TableBranch,
    It,
    /// MSR, MRS, CPS and the barriers.
    System {
        cycles: u32,
    },
    FpArithmetic,
    /// The multiply accumulate family.
    FpMac,
    FpDivSqrt,
    /// A unary operation or comparison.
    FpUnary {
        produces: bool,
    },
    FpLoad {
        double: bool,
    },
    FpStore {
        double: bool,
    },
    /// VLDM, VSTM, VPUSH, VPOP of `words` 32 bit words.
    FpMultiple {
        words: u32,
    },
    /// A move between the core and floating point registers.
    FpTransfer {
        cycles: u32,
        to_core: bool,
    },
}

/// The register a load or store address is based upon, where none
/// applies.
pub const NO_BASE: u8 = 0xff;

/// What can be determined about an instruction from its encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Decoded {
    pub kind: Kind,
    /// In bytes.
    pub size: u32,
    /// For loads and stores, the registers the address is formed
    /// from as a bit mask, where 13 is the stack pointer.  The PC is
    /// never included.
    pub address_regs: u16,
    /// For loads and stores, the base register, where 15 is the PC.
    pub base: u8,
    /// Whether a load or store is relative to the PC.
    pub pc_relative: bool,
    /// The instruction may write the PC other than as a branch.
    pub loads_pc: bool,
    /// A branch which is not conditional.
    pub always_branches: bool,
    /// The single precision registers read, as a bit mask.
    pub fp_reads: u32,
    /// The single precision register floating point arithmetic
    /// writes, as a bit mask.
    pub fp_writes: u32,
}

impl Decoded {
    fn new(kind: Kind, size: u32) -> Self {
        Self {
            kind,
            size,
            address_regs: 0,
            base: NO_BASE,
            pc_relative: false,
            loads_pc: false,
            always_branches: false,
            fp_reads: 0,
            fp_writes: 0,
        }
    }

    /// Record the registers an address is formed from, base first.
    fn with_address(mut self, regs: &[u32]) -> Self {
        self.base = regs[0] as u8;
        for &reg in regs {
            if reg == 15 {
                self.pc_relative = true;
            } else {
                self.address_regs |= 1 << reg;
            }
        }
        self
    }
}

/// Classify a 16 bit instruction.
pub fn decode16(hw: u16) -> Decoded {
    let hw = u32::from(hw);
    let d = |kind| Decoded::new(kind, 2);
    let rn = (hw >> 3) & 7;
    let rm = (hw >> 6) & 7;
    match hw >> 11 {
        0b01000 => match (hw >> 8) & 7 {
            // ADD and MOV with high registers can write the PC.
            0b100 | 0b110 if (hw & 0x87) == 0x87 => d(Kind::DataMaybePc),
            0b111 => d(Kind::BranchRegister {
                rm: ((hw >> 3) & 0xf) as u8,
            }),
            _ => d(Kind::Simple),
        },
        0b01001 => d(Kind::Load).with_address(&[15]),
        0b01010 | 0b01011 => {
            let kind = if (hw >> 9) & 7 < 3 {
                Kind::StoreRegOffset
            } else {
                Kind::Load
            };
            d(kind).with_address(&[rn, rm])
        }
        0b01100 | 0b01110 | 0b10000 => d(Kind::Store).with_address(&[rn]),
        0b01101 | 0b01111 | 0b10001 => d(Kind::Load).with_address(&[rn]),
        0b10010 => d(Kind::Store).with_address(&[13]),
        0b10011 => d(Kind::Load).with_address(&[13]),
        0b10110 | 0b10111 => {
            if hw & 0xf500 == 0xb100 {
                d(Kind::CompareBranch)
            } else if hw & 0xfe00 == 0xb400 {
                d(Kind::Multiple {
                    count: (hw & 0x1ff).count_ones(),
                })
                .with_address(&[13])
            } else if hw & 0xfe00 == 0xbc00 {
                let mut result = d(Kind::Multiple {
                    count: (hw & 0x1ff).count_ones(),
                })
                .with_address(&[13]);
                result.loads_pc = hw & 0x100 != 0;
                result
            } else if hw & 0xff00 == 0xbf00 && hw & 0xf != 0 {
                d(Kind::It)
            } else if hw == 0xbf00 {
                d(Kind::Nop)
            } else if hw & 0xffe8 == 0xb660 {
                // CPSID is slower than CPSIE.
                d(Kind::System {
                    cycles: if hw & 0x10 != 0 { 2 } else { 1 },
                })
            } else {
                d(Kind::Simple)
            }
        }
        0b11000 | 0b11001 => d(Kind::Multiple {
            count: (hw & 0xff).count_ones(),
        }),
        // MEASURED: SVC takes as long as a branch does to get to its
        // handler, before the exception entry itself.
        0b11011 if (hw >> 8) & 0xf == 0xf => d(Kind::System { cycles: 4 }),
        0b11010 | 0b11011 => d(Kind::Branch),
        0b11100 => {
            let mut result = d(Kind::Branch);
            result.always_branches = true;
            result
        }
        _ => d(Kind::Simple),
    }
}

fn decode_fp(hw1: u32, hw2: u32) -> Decoded {
    let mut result = decode_fp_kind(hw1, hw2);
    if matches!(
        result.kind,
        Kind::FpArithmetic | Kind::FpMac | Kind::FpDivSqrt | Kind::FpUnary { .. }
    ) {
        let vd = (((hw2 >> 12) & 0xf) << 1) | ((hw1 >> 6) & 1);
        result.fp_writes = 1 << vd;
    }
    result
}

fn decode_fp_kind(hw1: u32, hw2: u32) -> Decoded {
    let d = |kind| Decoded::new(kind, 4);
    let double = hw2 & 0x100 != 0;
    let vd = (((hw2 >> 12) & 0xf) << 1) | ((hw1 >> 6) & 1);
    let vn = ((hw1 & 0xf) << 1) | ((hw2 >> 7) & 1);
    let vm = ((hw2 & 0xf) << 1) | ((hw2 >> 5) & 1);
    let reads = |regs: &[u32]| regs.iter().fold(0u32, |mask, &r| mask | (1 << r));

    if hw1 & 0x0e00 == 0x0c00 {
        if hw1 & 0x0fe0 == 0x0c40 {
            // VMOV between two core registers and a double or a pair
            // of singles.
            return d(Kind::FpTransfer {
                cycles: 2,
                to_core: hw1 & 0x10 != 0,
            });
        }
        let load = hw1 & 0x10 != 0;
        let rn = hw1 & 0xf;
        let indexed_no_writeback = hw1 & 0x100 != 0 && hw1 & 0x20 == 0;
        if indexed_no_writeback {
            let mut result = if load {
                d(Kind::FpLoad { double })
            } else {
                d(Kind::FpStore { double })
            }
            .with_address(&[rn]);
            if !load {
                result.fp_reads = reads(&[vd]);
            }
            return result;
        }
        return d(Kind::FpMultiple { words: hw2 & 0xff }).with_address(&[rn]);
    }

    if hw2 & 0x10 != 0 {
        // VMOV to or from a core register, VMRS and VMSR.
        let to_core = hw1 & 0x10 != 0;
        let mut result = d(Kind::FpTransfer { cycles: 1, to_core });
        if to_core && hw1 & 0xe0 == 0 {
            result.fp_reads = reads(&[vn]);
        }
        return result;
    }

    // The opcode, less the D bit.
    let mut result = match (hw1 >> 4) & 0xb {
        0x0 | 0x1 | 0x9 | 0xa => {
            // TRM table 7-1 note: the addend is consumed a cycle later
            // than the operands of the multiply, so it does not count
            // as consuming the previous result.
            let mut result = d(Kind::FpMac);
            result.fp_reads = reads(&[vn, vm]);
            return result;
        }
        0x2 | 0x3 => d(Kind::FpArithmetic),
        0x8 => d(Kind::FpDivSqrt),
        _ => {
            if hw2 & 0x40 == 0 {
                // VMOV immediate.
                return d(Kind::FpUnary { produces: false });
            }
            let op = hw1 & 0xf;
            let bit7 = hw2 & 0x80 != 0;
            let mut result = match op {
                // VMOV register, VABS and VNEG.
                0 => d(Kind::FpUnary { produces: false }),
                1 if !bit7 => d(Kind::FpUnary { produces: false }),
                1 => d(Kind::FpDivSqrt),
                // VCMP reads its destination too.
                4 | 5 => {
                    let mut result = d(Kind::FpUnary { produces: false });
                    result.fp_reads = reads(&[vd]);
                    result
                }
                // The conversions.
                _ => d(Kind::FpUnary { produces: true }),
            };
            if op != 5 {
                result.fp_reads |= reads(&[vm]);
            }
            return result;
        }
    };
    result.fp_reads = reads(&[vn, vm]);
    result
}

/// Classify a 32 bit instruction, given its two halfwords.
pub fn decode32(hw1: u16, hw2: u16) -> Decoded {
    let hw1 = u32::from(hw1);
    let hw2 = u32::from(hw2);
    let d = |kind| Decoded::new(kind, 4);
    let rn = hw1 & 0xf;
    let rt = hw2 >> 12;
    let op2 = (hw1 >> 4) & 0x7f;

    if op2 & 0x40 != 0 && (hw1 >> 11) & 3 != 0b10 {
        // Coprocessor space.  Only the FPU is present.
        return if (hw2 >> 9) & 7 == 0b101 {
            decode_fp(hw1, hw2)
        } else {
            d(Kind::Simple)
        };
    }

    match (hw1 >> 11) & 3 {
        0b01 => {
            if op2 & 0x64 == 0x00 {
                let mut result = d(Kind::Multiple {
                    count: hw2.count_ones(),
                })
                .with_address(&[rn]);
                result.loads_pc = hw1 & 0x10 != 0 && hw2 & 0x8000 != 0;
                result
            } else if op2 & 0x64 == 0x04 {
                if hw1 & 0xfff0 == 0xe8d0 && hw2 & 0xffe0 == 0xf000 {
                    d(Kind::TableBranch)
                } else if hw1 & 0xffe0 == 0xe840 || (hw1 & 0xffe0 == 0xe8c0 && hw2 & 0xc0 == 0x40) {
                    d(Kind::DualOrExclusive { cycles: 2 }).with_address(&[rn])
                } else {
                    d(Kind::DualOrExclusive { cycles: 3 }).with_address(&[rn])
                }
            } else {
                d(Kind::Simple)
            }
        }
        0b10 => {
            if hw2 & 0x8000 == 0 {
                d(Kind::Simple)
            } else if hw2 & 0x5000 == 0 && hw1 & 0x0380 == 0x0380 {
                if hw1 == 0xf3af {
                    // NOP and the other hints.
                    d(if hw2 & 0xff == 0 {
                        Kind::Nop
                    } else {
                        Kind::Simple
                    })
                } else if hw1 == 0xf3bf && hw2 & 0xf0 == 0x60 {
                    // ISB flushes the pipeline.
                    d(Kind::System { cycles: 3 })
                } else {
                    // MSR, MRS, DSB and DMB.
                    d(Kind::System { cycles: 1 })
                }
            } else {
                // Only B<cond>.W, which has neither of these bits
                // set, is conditional.
                let mut result = d(Kind::Branch);
                result.always_branches = hw2 & 0x5000 != 0;
                result
            }
        }
        _ => {
            if op2 & 0x71 == 0x00 {
                // Store single.
                let reg_offset = hw1 & 0x80 == 0 && hw2 & 0x0fc0 == 0;
                if reg_offset {
                    d(Kind::StoreRegOffset).with_address(&[rn, hw2 & 0xf])
                } else {
                    d(Kind::Store).with_address(&[rn])
                }
            } else if op2 & 0x61 == 0x01 {
                // Load byte, halfword or word.
                let word = op2 & 0x06 == 0x04;
                if rt == 15 && !word {
                    // A preload hint.
                    return d(Kind::Simple);
                }
                let reg_offset = rn != 15 && hw1 & 0x80 == 0 && hw2 & 0x0fc0 == 0;
                let mut result = if reg_offset {
                    d(Kind::Load).with_address(&[rn, hw2 & 0xf])
                } else {
                    d(Kind::Load).with_address(&[rn])
                };
                result.loads_pc = rt == 15;
                result
            } else if op2 & 0x78 == 0x38 && hw1 & 0xffd0 == 0xfb90 && hw2 & 0xf0 == 0xf0 {
                d(Kind::Divide {
                    signed: hw1 & 0x20 == 0,
                })
            } else {
                // Register data processing and the multiplies.
                d(Kind::Simple)
            }
        }
    }
}

/// The core registers a 16 bit instruction writes, as a bit mask
/// where 13 is the stack pointer, other than by writing back a base
/// register.
///
/// What an instruction wrote cannot be had by comparing registers
/// before and after, since writing the value a register already held
/// has the same effect on timing as writing any other.
pub fn writes16(hw: u16) -> u16 {
    let hw = u32::from(hw);
    let low = 1 << (hw & 7);
    let mid = 1 << ((hw >> 8) & 7);
    let result: u32 = match hw >> 11 {
        // Shifts, and three register adds and subtracts.
        0b00000..=0b00011 => low,
        // MOV, ADD and SUB of an immediate, but not CMP.
        0b00100 | 0b00110 | 0b00111 => mid,
        0b01000 if hw & 0x0400 == 0 => match (hw >> 6) & 0xf {
            // TST, CMP and CMN.
            8 | 10 | 11 => 0,
            _ => low,
        },
        0b01000 => match (hw >> 8) & 3 {
            // ADD and MOV with high registers.
            0 | 2 => 1 << (((hw >> 4) & 8) | (hw & 7)),
            _ => 0,
        },
        // A literal.
        0b01001 => mid,
        0b01010 | 0b01011 => {
            if (hw >> 9) & 7 >= 3 {
                low
            } else {
                0
            }
        }
        0b01101 | 0b01111 | 0b10001 => low,
        0b10011 => mid,
        // ADR and ADD from the stack pointer.
        0b10100 | 0b10101 => mid,
        0b10110 | 0b10111 => match (hw >> 8) & 0xf {
            0b0000 => 1 << 13,
            // The extends and the byte reverses.
            0b0010 | 0b1010 => low,
            0b0100 | 0b0101 => 1 << 13,
            0b1100 | 0b1101 => (hw & 0xff) | (1 << 13),
            _ => 0,
        },
        0b11001 => hw & 0xff,
        _ => 0,
    };
    (result & 0x7fff) as u16
}

/// As `writes16`, for a 32 bit instruction.
pub fn writes32(hw1: u16, hw2: u16) -> u16 {
    let hw1 = u32::from(hw1);
    let hw2 = u32::from(hw2);
    let op2 = (hw1 >> 4) & 0x7f;
    let rt = 1 << (hw2 >> 12);
    let rd = 1 << ((hw2 >> 8) & 0xf);
    let result: u32 = if op2 & 0x40 != 0 && (hw1 >> 11) & 3 != 0b10 {
        // VMOV to a core register, and VMRS to other than the flags.
        let fp = (hw2 >> 9) & 7 == 0b101;
        if fp && hw1 & 0x0f00 == 0x0e00 && hw2 & 0x10 != 0 && hw1 & 0x10 != 0 {
            rt
        } else if fp && hw1 & 0xff30 == 0xed10 {
            // MEASURED: a load or store through r0 which follows a
            // VLDR behaves as though the VLDR had written r0, never
            // overlapping it.  No other register does this.
            1
        } else {
            0
        }
    } else {
        match (hw1 >> 11) & 3 {
            0b01 if op2 & 0x64 == 0x00 => {
                if hw1 & 0x10 != 0 {
                    hw2
                } else {
                    0
                }
            }
            0b01 if op2 & 0x64 == 0x04 => {
                if hw1 & 0x10 != 0 {
                    rt | rd
                } else {
                    0
                }
            }
            0b01 => rd,
            0b10 if hw2 & 0x8000 == 0 => rd,
            // MRS.
            0b10 if hw1 == 0xf3ef => rd,
            0b10 => 0,
            _ => {
                if op2 & 0x71 == 0x00 {
                    0
                } else if op2 & 0x61 == 0x01 {
                    rt
                } else if hw1 & 0xff80 == 0xfb80 {
                    // The long multiplies, and the divides which have
                    // no second destination.
                    rt | rd
                } else {
                    rd
                }
            }
        }
    };
    (result & 0x7fff) as u16
}

/// How many halfwords the prefetcher can hold beyond the instruction
/// executing.
const FETCH_AHEAD_MAX: i32 = 4;

/// What kind of memory a load or store accesses, to the extent that
/// it affects timing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataClass {
    /// Not a load or store.
    None,
    /// SRAM1.
    Ram,
    /// The processor's own registers, on the private peripheral bus.
    Ppb,
    /// SRAM2, which unlike SRAM1 behaves as an AHB peripheral does.
    SlowRam,
    /// The same RAM that the code is executing from, as a literal
    /// pool always is.
    CodeRam,
    Flash,
    /// A peripheral directly on an AHB bus.
    Ahb,
    /// A peripheral behind an APB bridge.
    Apb,
}

/// What is known about one instruction once it has executed.
#[derive(Debug, Clone, Copy)]
pub struct Executed {
    pub pc: u32,
    pub decoded: Decoded,
    /// False if the instruction was skipped by an IT block.
    pub condition_passed: bool,
    pub next_pc: u32,
    /// Whether the instruction at `next_pc` is 32 bits wide.
    pub next_is_32bit: bool,
    /// Core registers written, as a bit mask, where 13 is the stack
    /// pointer.
    pub regs_written: u16,
    /// Single precision registers written, as a bit mask.
    pub fp_written: u32,
    /// The operands, for divides.
    pub dividend: u32,
    pub divisor: u32,
    /// What a load or store accesses.
    pub data: DataClass,
    /// And where, if it is a load with an immediate offset.
    pub data_address: u32,
}

impl Executed {
    fn executed_kind(&self) -> Option<Kind> {
        self.condition_passed.then_some(self.decoded.kind)
    }

    /// A load of a single register, which is what can pipeline.
    fn is_single_load(&self) -> bool {
        matches!(
            self.executed_kind(),
            Some(Kind::Load | Kind::FpLoad { double: false })
        ) && !self.decoded.loads_pc
    }

    fn is_store(&self) -> bool {
        matches!(
            self.executed_kind(),
            Some(Kind::Store | Kind::StoreRegOffset | Kind::FpStore { .. })
        )
    }

    /// Whether this did nothing more than compute a value, so that
    /// its result reaches the address generation unit late.
    fn is_data_processing(&self) -> bool {
        matches!(self.executed_kind(), Some(Kind::Simple | Kind::DataMaybePc))
    }

    /// Whether a branch was taken.  One to the very next instruction
    /// can only be recognized if it is unconditional.
    fn taken(&self) -> bool {
        self.condition_passed
            && (self.decoded.always_branches
                || self.next_pc != self.pc.wrapping_add(self.decoded.size))
    }
}

/// An exception related event which accompanied an instruction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExceptionEvent {
    None,
    /// An exception was taken once the instruction completed.
    Entry,
    /// The instruction returned from an exception.
    Return {
        fp_frame: bool,
    },
    /// As for `Return`, but another exception was pending and was
    /// entered directly.
    TailChain,
}

/// A small least recently used cache of flash lines.
struct LineCache {
    lines: Vec<u32>,
    capacity: usize,
}

impl LineCache {
    /// Flash is read 64 bits at a time, and each cache line holds
    /// what one read returned.
    const WORD_SHIFT: u32 = 3;

    fn new(capacity: usize) -> Self {
        Self {
            lines: Vec::new(),
            capacity,
        }
    }

    /// Returns true on a hit.  Either way the line is then the most
    /// recently used.
    fn access(&mut self, addr: u32) -> bool {
        let line = addr >> Self::WORD_SHIFT;
        let found = self.lines.iter().position(|&l| l == line);
        if let Some(index) = found {
            self.lines.remove(index);
        } else if self.lines.len() >= self.capacity {
            self.lines.remove(0);
        }
        self.lines.push(line);
        found.is_some()
    }
}

fn in_flash(addr: u32) -> bool {
    (FLASH_START..FLASH_END).contains(&addr)
}

/// MEASURED: how long each of a run of consecutive loads from the RAM
/// that the code is executing from takes, for 16 and 32 bit
/// instructions.  The data access competes with instruction fetch
/// for the one memory, and the longer the run the more it starves
/// the pipeline.  Beyond the end of each table, the last entry
/// repeats.
const CODE_RAM_LOAD_16: [u32; 8] = [3, 1, 1, 2, 1, 2, 2, 2];
const CODE_RAM_LOAD_32: [u32; 4] = [3, 1, 2, 3];

/// Accumulates the state which carries from one instruction to the
/// next.
pub struct Timing {
    pub params: Params,
    icache: LineCache,
    dcache: LineCache,
    /// The 64 bits of flash most recently fetched from.
    fetch_word: Option<u32>,
    /// The 64 bits of flash being read ahead, and when they arrive.
    flash_read_ahead: Option<(u32, u64)>,
    /// The total number of cycles accounted for.
    now: u64,
    previous: Option<Executed>,
    /// The single precision registers holding the result of the
    /// previous instruction, if it was floating point arithmetic.
    previous_fp_result: u32,
    /// Cycles remaining until the floating point unit has finished
    /// what it was last given.
    fp_busy: u32,
    /// The registers which will hold the result of that.
    fp_busy_result: u32,
    /// How many stores to the same kind of memory have run back to
    /// back.
    store_run: usize,
    /// How many loads from the RAM being executed from have run back
    /// to back.
    code_ram_run: usize,
    /// Whether the previous instruction was a store which followed a
    /// load closely enough to cost nothing.
    store_rode_load: bool,
    /// How many halfwords the prefetcher holds from the start of the
    /// next instruction on.
    fetch_ahead: i32,
    /// How many instructions from now the prefetcher will miss a
    /// fetch, and whether it will do so even with nothing in hand.
    fetch_loss: Option<(u32, bool)>,
    /// The instruction being timed had to wait to be fetched.
    fetch_waited: bool,
    /// The next instruction was fetched in time to overlap the one
    /// before it.
    fetch_overlap: bool,
    /// The previous instruction was a load from one of the slower
    /// memories which set a lost fetch in train.
    fetch_loss_pushable: bool,
    /// If the peripheral buses run at half the speed of the processor,
    /// a time at which their clock rose.
    apb_half_speed: Option<u64>,
    /// With those buses at half speed, when the bridge to them will
    /// next let anything else use the bus.
    apb_busy_until: u64,
    /// And when a load which has to wait for what was last written
    /// through it will finish.
    apb_read_after_write: u64,
}

impl Timing {
    pub fn new(params: Params) -> Self {
        let icache = LineCache::new(params.icache_lines);
        let dcache = LineCache::new(params.dcache_lines);
        Self {
            params,
            icache,
            dcache,
            fetch_word: None,
            flash_read_ahead: None,
            now: 0,
            previous: None,
            previous_fp_result: 0,
            fp_busy: 0,
            fp_busy_result: 0,
            store_run: 0,
            code_ram_run: 0,
            store_rode_load: false,
            fetch_ahead: FETCH_AHEAD_MAX,
            fetch_loss: None,
            fetch_waited: false,
            fetch_overlap: true,
            fetch_loss_pushable: false,
            apb_half_speed: None,
            apb_busy_until: 0,
            apb_read_after_write: 0,
        }
    }

    /// Note that the peripheral buses have just been set to run at
    /// the speed of the processor, or at half of it.
    pub fn set_apb_half_speed(&mut self, half_speed: bool) {
        self.apb_half_speed = half_speed.then_some(self.now);
    }

    /// MEASURED: how long a load from a peripheral bus running at
    /// half speed takes.
    ///
    /// This, like the rest of what is here for those buses, is an
    /// approximation to what was measured.  The bridge behaves as
    /// though the phase of its clock depends on what it has been
    /// asked to do, and that is not modelled: an access is taken to
    /// finish on a cycle of fixed parity.
    fn apb_half_speed_load(&mut self, e: &Executed) -> u32 {
        let origin = self.apb_half_speed.unwrap_or(0);
        let after_load = self
            .previous_access(e)
            .is_some_and(|(store, data)| !store && data == DataClass::Apb);
        let end = if after_load {
            // It overlaps the one before.
            self.now + 4
        } else if self.now < self.apb_read_after_write {
            // It waits for what was written to get there.
            self.apb_read_after_write
        } else {
            let end = self.now + 6;
            end + ((end - origin) & 1)
        };
        self.apb_busy_until = end;
        (end - self.now) as u32
    }

    /// As `apb_half_speed_load`, for a store.  The processor only has
    /// to wait for the bridge to accept it.
    fn apb_half_speed_store(&mut self, e: &Executed) -> u32 {
        let rides = self
            .previous_access(e)
            .is_some_and(|(store, data)| !store && data == DataClass::Apb);
        let end = if rides {
            self.now
        } else {
            (self.now + 1).max(self.apb_busy_until)
        };
        let queued = self.now < self.apb_busy_until;
        // Nothing else can use the bus for a while, and a second
        // write holds it for longer.
        self.apb_busy_until = end + if queued { 6 } else { 2 };
        self.apb_read_after_write = end + if queued { 12 } else { 10 };
        (end - self.now) as u32
    }

    /// With the peripheral buses at half speed, how long an access to
    /// anything else on the system bus has to wait for the bridge to
    /// finish a write.
    fn apb_bus_wait(&self, e: &Executed) -> u32 {
        // MEASURED: this includes the private peripheral bus.
        let waits = Self::on_system_bus(e.data) || e.data == DataClass::Ppb;
        if self.apb_half_speed.is_none() || !waits || e.data == DataClass::Apb {
            return 0;
        }
        self.apb_busy_until.saturating_sub(self.now) as u32
    }

    /// The pipeline refill after a taken branch.
    fn refill(&self, e: &Executed, base: u32) -> u32 {
        let straddles = e.next_is_32bit && e.next_pc & 2 != 0;
        base + if straddles {
            self.params.refill_unaligned32
        } else {
            0
        }
    }

    /// MEASURED: A divide by zero, or of zero, takes 2 cycles, and
    /// one whose dividend has fewer significant bits than its divisor
    /// takes 3.  Otherwise it takes 4, plus one for each 4 bits or
    /// part thereof of quotient, up to the 12 the TRM gives as the
    /// maximum.
    ///
    /// For a signed divide, the bits are those of the magnitude of
    /// the dividend, but of the complement of a negative divisor.
    fn divide_cycles(e: &Executed, signed: bool) -> u32 {
        let bits = |v: u32| 32 - v.leading_zeros();
        let (dividend_bits, divisor_bits) = if signed {
            let divisor = e.divisor as i32;
            (
                bits((e.dividend as i32).unsigned_abs()),
                bits(if divisor < 0 { !e.divisor } else { e.divisor }),
            )
        } else {
            (bits(e.dividend), bits(e.divisor))
        };
        if e.divisor == 0 || e.dividend == 0 {
            2
        } else if dividend_bits < divisor_bits {
            3
        } else {
            (4 + (dividend_bits - divisor_bits + 1).div_ceil(4)).min(12)
        }
    }

    /// How long the instruction at 'pc' has to wait to be read from
    /// flash.
    ///
    /// MEASURED, on the one function in flash which the control
    /// interrupt calls: flash is read 64 bits at a time, which takes
    /// a cycle more than there are wait states.  The next 64 bits
    /// are then read ahead, but code gets through 64 bits faster than
    /// that, so that it waits at each.
    fn fetch_penalty(&mut self, pc: u32) -> u32 {
        if !in_flash(pc) {
            self.fetch_word = None;
            return 0;
        }
        let word = pc >> LineCache::WORD_SHIFT;
        if self.fetch_word.replace(word) == Some(word) {
            return 0;
        }
        let read = u64::from(self.params.flash_wait_states) + 1;
        // RM0440 3.3.4: only what was missed, which is to say what
        // was branched to, goes into the cache.
        let wait = match self.flash_read_ahead {
            Some((ahead, ready)) if ahead == word => ready.saturating_sub(self.now),
            _ if self.icache.access(pc) => 0,
            _ => read,
        };
        let arrived = self.now + wait;
        self.flash_read_ahead = Some((word + 1, arrived + read));
        wait as u32
    }

    /// TRM 3.3.2: an address formed from the result of the previous
    /// instruction stalls for a cycle, as there is no forwarding path
    /// into the address generation unit.
    fn address_stall(&self, e: &Executed) -> u32 {
        let stalls = self.previous.as_ref().is_some_and(|p| {
            p.is_data_processing() && e.decoded.address_regs & p.regs_written != 0
        });
        u32::from(stalls)
    }

    /// The previous instruction, if it was a load or store of a single
    /// register which this access can be considered to follow: that
    /// is, one whose address did not depend upon what it loaded.
    fn previous_access(&self, e: &Executed) -> Option<(bool, DataClass)> {
        if self.fetch_waited {
            return None;
        }
        let p = self.previous.as_ref()?;
        let store = p.is_store();
        if !store && !p.is_single_load() {
            return None;
        }
        if store && p.data == DataClass::Apb && self.apb_half_speed.is_some() {
            // What that costs is accounted for by `apb_bus_wait`.
            return None;
        }
        let dependent = !store && e.decoded.address_regs & p.regs_written != 0;
        (!dependent).then_some((store, p.data))
    }

    /// MEASURED: how long a load takes when nothing it can combine
    /// with came before.  TRM 3.3.2 gives 2 cycles.  SRAM2 and the AHB
    /// peripherals take one more, the RAM being executed from another
    /// to make room for the fetch it displaces, and the APB bridge
    /// three.
    fn fresh_load(data: DataClass) -> u32 {
        match data {
            DataClass::None | DataClass::Ram | DataClass::Ppb | DataClass::Flash => 2,
            DataClass::SlowRam | DataClass::Ahb | DataClass::CodeRam => 3,
            DataClass::Apb => 5,
        }
    }

    /// Whether a memory is reached over the system bus, as opposed to
    /// the private peripheral bus, or the buses dedicated to the RAM
    /// being executed from.
    fn on_system_bus(data: DataClass) -> bool {
        matches!(
            data,
            DataClass::Ram
                | DataClass::SlowRam
                | DataClass::Ahb
                | DataClass::Apb
                | DataClass::Flash
        )
    }

    /// The cycles a load of a single register takes.
    ///
    /// MEASURED: all of this comes from timing every pairing of a load
    /// or store to one kind of memory with a load from another.
    fn load_cycles(&mut self, e: &Executed) -> u32 {
        let data = e.data;
        if self.apb_half_speed.is_some() && data == DataClass::Apb {
            return self.address_stall(e) + self.apb_half_speed_load(e);
        }
        let fresh = Self::fresh_load(data);
        let mut run = 1;
        let cycles = match self.previous_access(e) {
            // TRM 3.3.2: a load can overlap the second cycle of the
            // load before it.
            Some((false, previous)) if previous == data => match data {
                DataClass::CodeRam => {
                    run = self.code_ram_run + 1;
                    let table: &[u32] = if e.decoded.size == 2 {
                        &CODE_RAM_LOAD_16
                    } else {
                        &CODE_RAM_LOAD_32
                    };
                    table[(run - 1).min(table.len() - 1)]
                }
                // The bridge still inserts its waits.
                DataClass::Apb => 3,
                _ => 1,
            },
            // After a load from elsewhere there is no overlap, but
            // if both are on the system bus, what makes the first
            // access to its slower parts take longer is avoided.
            Some((false, previous)) => {
                let saved = matches!(data, DataClass::SlowRam | DataClass::Ahb | DataClass::Apb)
                    && Self::on_system_bus(previous);
                fresh - u32::from(saved)
            }
            // After a store, the load waits for however much of the
            // write it cannot overlap.
            // Unless that store itself followed a load, in which case
            // the bus is still going.
            Some((true, previous))
                if previous == data
                    && self.store_rode_load
                    && matches!(data, DataClass::SlowRam | DataClass::Ahb) =>
            {
                fresh - 1
            }
            Some((true, previous)) => {
                // MEASURED: one more if that store itself followed a
                // load and went elsewhere.
                let rode = u32::from(self.store_rode_load && previous != data);
                let wait = rode
                    + if previous == data {
                        // The bridge is the exception.
                        if data == DataClass::Apb {
                            2
                        } else {
                            0
                        }
                    } else {
                        match previous {
                            DataClass::Ram | DataClass::Flash => {
                                u32::from(!Self::on_system_bus(data))
                            }
                            DataClass::SlowRam | DataClass::Ahb => {
                                if matches!(
                                    data,
                                    DataClass::SlowRam | DataClass::Ahb | DataClass::Apb
                                ) {
                                    1
                                } else {
                                    2
                                }
                            }
                            DataClass::Apb | DataClass::CodeRam => 2,
                            DataClass::Ppb | DataClass::None => 0,
                        }
                    };
                fresh + wait
            }
            None => {
                // One whose address the previous load supplied gets
                // the benefit of the bus already being active.
                // MEASURED: but not if that was from the same memory.
                let after_system_load = self.previous.as_ref().is_some_and(|p| {
                    p.is_single_load() && Self::on_system_bus(p.data) && p.data != data
                });
                let saved = after_system_load
                    && matches!(data, DataClass::SlowRam | DataClass::Ahb | DataClass::Apb);
                fresh - u32::from(saved)
            }
        };
        if data == DataClass::CodeRam {
            self.code_ram_run = run;
        }
        cycles + self.address_stall(e) + self.apb_bus_wait(e)
    }

    /// The cycles a store of a single register takes.
    ///
    /// TRM 3.3.2: a store is always one cycle, as the write happens
    /// during the next instruction, or two with a register offset.
    /// MEASURED: everything else, as for loads.
    fn store_cycles(&mut self, e: &Executed, reg_offset: bool) -> u32 {
        let data = e.data;
        if self.apb_half_speed.is_some() && data == DataClass::Apb {
            self.store_run = 1;
            self.store_rode_load = false;
            return self.address_stall(e) + u32::from(reg_offset) + self.apb_half_speed_store(e);
        }
        let slow = |d| matches!(d, DataClass::SlowRam | DataClass::Ahb | DataClass::CodeRam);
        let mut run = 1;
        let cycles = match self.previous_access(e) {
            // Right after a load on the same bus, the store's address
            // phase is free.
            Some((false, previous)) => {
                let same_bus = previous == data
                    || (Self::on_system_bus(previous) && Self::on_system_bus(data));
                u32::from(!same_bus)
            }
            // After another store, it waits for the first to finish.
            Some((true, previous)) if previous == data => {
                run = self.store_run + 1;
                match data {
                    DataClass::Apb => 3,
                    // Only the first of a run is slow to start.
                    _ if slow(data) && run == 2 => 2,
                    _ => 1,
                }
            }
            Some((true, previous)) => {
                1 + match (previous, data) {
                    (DataClass::Ram, DataClass::CodeRam) => 1,
                    (DataClass::Ram, _) => 0,
                    (DataClass::SlowRam, DataClass::CodeRam) => 2,
                    (DataClass::CodeRam, _) => 2,
                    (DataClass::Apb, _) => 2,
                    (_, _) => u32::from(slow(previous)),
                }
            }
            None => 1,
        };
        self.store_run = run;
        self.store_rode_load = cycles == 0;
        cycles + u32::from(reg_offset) + self.address_stall(e) + self.apb_bus_wait(e)
    }

    /// MEASURED: the extra cycle an instruction which transfers more
    /// than one word takes to start on anything but SRAM1, unless it
    /// follows a load on the same bus.
    fn first_access(&self, e: &Executed) -> u32 {
        let slow = matches!(
            e.data,
            DataClass::Ahb | DataClass::SlowRam | DataClass::CodeRam
        );
        let after_system_load = self.previous.as_ref().is_some_and(|p| {
            p.is_single_load() && Self::on_system_bus(p.data) && Self::on_system_bus(e.data)
        });
        u32::from(slow && !after_system_load)
    }

    /// MEASURED: a NOP which follows an instruction whose final cycle
    /// only waits for the bus runs during that cycle, for free.
    fn nop_is_free(&self) -> bool {
        !self.fetch_waited
            && self.previous.as_ref().is_some_and(|p| {
                p.is_single_load()
                    || matches!(
                        p.executed_kind(),
                        Some(Kind::StoreRegOffset | Kind::TableBranch)
                    )
            })
    }

    /// TRM 3.3.1: an IT can fold onto a preceding 16 bit instruction.
    /// MEASURED: it does so wherever it falls, but not after a load
    /// or store.
    fn it_is_folded(&self) -> bool {
        self.previous.as_ref().is_some_and(|p| {
            p.decoded.size == 2
                && !p.taken()
                && matches!(
                    p.decoded.kind,
                    Kind::Simple
                        | Kind::Nop
                        | Kind::DataMaybePc
                        | Kind::Branch
                        | Kind::CompareBranch
                        | Kind::It
                        | Kind::System { .. }
                )
        })
    }

    /// MEASURED: the prefetcher reads a word of instructions each
    /// cycle, while it has room for it, and can hold two beyond what
    /// is executing.  16 bit instructions and those which take more
    /// than a cycle let it get ahead; 32 bit ones only let it keep
    /// up.  Some instructions cost it a fetch, and if it was not
    /// ahead, an instruction soon after has to wait for its second
    /// half to arrive.
    ///
    /// This is the wait before 'e' can start.
    fn fetch_wait(&mut self, e: &Executed) -> u32 {
        let size = (e.decoded.size / 2) as i32;
        let mut wait = 0;
        while self.fetch_ahead < size {
            self.fetch_ahead += 2;
            wait += 1;
        }
        wait
    }

    /// Account for what the prefetcher does while 'e', which takes
    /// 'cycles', executes.
    fn fetch_during(&mut self, e: &Executed, cycles: u32) {
        if e.taken() {
            self.fetch_restart(e.next_pc, e.next_is_32bit);
            return;
        }
        // Whether a whole word beyond the one this instruction starts
        // in has been fetched.
        let word_ahead = self.fetch_ahead + i32::from(e.pc & 2 != 0) >= 4;
        self.fetch_ahead -= (e.decoded.size / 2) as i32;
        if cycles == 0 {
            // It ran alongside the instruction before it.  It still
            // counts as an instruction, but there is no cycle of its
            // own in which a fetch could be lost.
            if self.fetch_ahead + 2 <= FETCH_AHEAD_MAX {
                self.fetch_ahead += 2;
            }
            if let Some((remaining, always)) = self.fetch_loss {
                self.fetch_loss = Some((remaining.saturating_sub(1).max(1), always));
            }
            return;
        }
        // Whether this is the cycle in which a fetch is lost.
        let mut lost = false;
        let mut due = false;
        if let Some((remaining, always)) = self.fetch_loss.take() {
            if remaining > 1 {
                self.fetch_loss = Some((remaining - 1, always));
            } else {
                // One which only delays reading ahead does not happen
                // if the word in question is the one needed next.
                lost = always || word_ahead;
                // Whether that makes any difference.
                due = lost && self.fetch_ahead + 2 <= FETCH_AHEAD_MAX;
            }
        }
        let mut room = false;
        // MEASURED: a load from the RAM being executed from takes
        // the place of a fetch from it.
        let code_ram = e.data == DataClass::CodeRam && e.is_single_load();
        for cycle in 0..cycles {
            room = self.fetch_ahead + 2 <= FETCH_AHEAD_MAX;
            let blocked = code_ram && cycle + 1 == cycles;
            if room && !blocked && !(lost && cycle == 0) {
                self.fetch_ahead += 2;
            }
            if cycle == 0 {
                // For the next instruction to overlap this one, it
                // has to be there to be decoded by now.
                self.fetch_overlap = self.fetch_ahead >= if e.next_is_32bit { 2 } else { 1 };
            }
        }

        // MEASURED: a load from the slower memories which overlaps
        // the one which set a lost fetch in train puts it off.
        let slow = matches!(e.data, DataClass::SlowRam | DataClass::Ahb);
        let pushable = std::mem::take(&mut self.fetch_loss_pushable);
        if pushable && slow && e.is_single_load() && cycles == 1 {
            if let Some((remaining, always)) = self.fetch_loss {
                self.fetch_loss = Some((remaining + 1, always));
            }
        }

        // The branch target is fetched in case it is needed, in place
        // of what follows the branch, unless there was no room for
        // that anyway.
        if e.executed_kind() == Some(Kind::Branch) {
            if room {
                self.fetch_loss = Some((1, true));
            }
            return;
        }
        // Nothing else is set in train until the last has played out,
        // other than by a load from the faster memory.
        let fast_load = e.is_single_load() && e.data == DataClass::Ram;
        if self.fetch_loss.is_some() || (due && !fast_load) {
            return;
        }
        self.fetch_loss = match e.executed_kind() {
            Some(
                Kind::Load
                | Kind::Store
                | Kind::StoreRegOffset
                | Kind::DualOrExclusive { .. }
                | Kind::Multiple { .. }
                | Kind::CompareBranch
                | Kind::System { .. }
                | Kind::FpLoad { .. }
                | Kind::FpStore { .. }
                | Kind::FpMultiple { .. },
            ) => Some((2, false)),
            // Floating point which does not touch memory only does
            // this ahead of a 32 bit instruction.
            Some(
                Kind::FpArithmetic
                | Kind::FpMac
                | Kind::FpDivSqrt
                | Kind::FpUnary { .. }
                | Kind::FpTransfer { .. },
            ) if e.next_is_32bit => Some((2, false)),
            _ => None,
        };
        self.fetch_loss_pushable = slow && e.is_single_load() && self.fetch_loss.is_some();
    }

    /// The prefetcher starts again at 'pc': after a branch, or on
    /// entering or leaving an exception handler.
    fn fetch_restart(&mut self, pc: u32, is_32bit: bool) {
        // The refill has fetched the word the instruction starts in,
        // and the next if it runs on into that.
        self.fetch_ahead = match (pc & 2 != 0, is_32bit) {
            (false, _) => 2,
            (true, true) => 3,
            (true, false) => 1,
        };
        self.fetch_loss = None;
        self.fetch_loss_pushable = false;
        self.fetch_overlap = true;
    }

    /// The cycles taken by an instruction, excluding any exception
    /// entry or return.
    fn instruction_cycles(&mut self, e: &Executed) -> u32 {
        let d = &e.decoded;
        let taken = e.taken();
        let params = &self.params;
        let (direct, indirect, from_load) = (
            self.refill(e, params.refill_direct),
            self.refill(e, params.refill_indirect),
            self.refill(e, params.refill_load),
        );

        if !e.condition_passed {
            // MEASURED: a skipped instruction costs one cycle,
            // whatever it is.
            return 1;
        }

        let is_fp = matches!(
            d.kind,
            Kind::FpArithmetic
                | Kind::FpMac
                | Kind::FpDivSqrt
                | Kind::FpUnary { .. }
                | Kind::FpLoad { .. }
                | Kind::FpStore { .. }
                | Kind::FpMultiple { .. }
                | Kind::FpTransfer { .. }
        );
        // TRM table 7-1 note: integer instructions complete out of
        // order with respect to a divide or square root, but another
        // floating point instruction must wait for it.  MEASURED:
        // the same is true of a multiply accumulate.
        let mut cycles = if is_fp {
            std::mem::take(&mut self.fp_busy)
        } else {
            0
        };
        // TRM table 7-1 note: floating point arithmetic takes a cycle
        // longer if the following instruction consumes the result.
        // That cycle is accounted for here, on the consumer.
        // MEASURED: the same holds if integer instructions came
        // between, so long as the result was still on its way.
        let waited_for = if cycles > 0 { self.fp_busy_result } else { 0 };
        if d.fp_reads & (self.previous_fp_result | waited_for) != 0 {
            cycles += 1;
        }
        if is_fp {
            self.fp_busy_result = 0;
        }

        cycles += match d.kind {
            Kind::Simple => 1,
            Kind::Nop => u32::from(!self.nop_is_free()),
            Kind::DataMaybePc => 1 + if taken { indirect } else { 0 },
            Kind::Divide { signed } => Self::divide_cycles(e, signed),
            Kind::Load | Kind::FpLoad { double: false } => {
                self.load_cycles(e) + if taken { from_load } else { 0 }
            }
            Kind::Store | Kind::FpStore { double: false } => self.store_cycles(e, false),
            Kind::StoreRegOffset => self.store_cycles(e, true),
            // ASSUMPTION: doubles are as the TRM says, and do not
            // pipeline.
            Kind::FpLoad { double: true } | Kind::FpStore { double: true } => {
                3 + self.address_stall(e)
            }
            Kind::DualOrExclusive { cycles } => {
                cycles + self.address_stall(e) + self.first_access(e) + self.apb_bus_wait(e)
            }
            Kind::Multiple { count } => {
                // MEASURED: one to the stack right after a load from
                // one of the slower memories takes a cycle longer.
                let after_slow_load = self.previous.as_ref().is_some_and(|p| {
                    p.is_single_load() && matches!(p.data, DataClass::SlowRam | DataClass::Ahb)
                });
                1 + count
                    + u32::from(after_slow_load && e.data == DataClass::Ram)
                    + self.address_stall(e)
                    + self.first_access(e)
                    + self.apb_bus_wait(e)
                    + if taken { from_load } else { 0 }
            }
            Kind::Branch => 1 + if taken { direct } else { 0 },
            // MEASURED: a return through the link register is as
            // quick as a direct branch.
            Kind::BranchRegister { rm: 14 } => 1 + direct,
            Kind::BranchRegister { .. } => 1 + indirect,
            Kind::CompareBranch => 1 + if taken { indirect } else { 0 },
            // MEASURED: 9, from a table in the RAM being executed
            // from.  ASSUMPTION: that splits as a 3 cycle load of the
            // offset, 3 to apply it, and the refill.
            Kind::TableBranch => 6 + from_load,
            Kind::It => u32::from(!self.it_is_folded()),
            Kind::System { cycles } => cycles,
            Kind::FpArithmetic | Kind::FpUnary { .. } => 1,
            Kind::FpMac => {
                self.fp_busy = 2;
                1
            }
            Kind::FpDivSqrt => {
                self.fp_busy = 13;
                1
            }
            Kind::FpMultiple { words } => 1 + words + self.first_access(e) + self.apb_bus_wait(e),
            Kind::FpTransfer { cycles, .. } => cycles,
        };

        // MEASURED: a store to the private peripheral bus holds up
        // whatever follows it for a cycle, unless that is something
        // which touches memory itself, or does nothing.
        let after_ppb_store = self
            .previous
            .as_ref()
            .is_some_and(|p| p.is_store() && p.data == DataClass::Ppb);
        let waits = !matches!(
            d.kind,
            Kind::Nop
                | Kind::Load
                | Kind::Store
                | Kind::StoreRegOffset
                | Kind::DualOrExclusive { .. }
                | Kind::Multiple { .. }
                | Kind::System { .. }
                | Kind::FpLoad { .. }
                | Kind::FpStore { .. }
                | Kind::FpMultiple { .. }
        );
        if after_ppb_store && waits {
            cycles += 1;
        }

        if !is_fp {
            self.fp_busy = self.fp_busy.saturating_sub(cycles);
        }
        cycles
    }

    /// Account for one instruction, returning the cycles it took.
    pub fn step(&mut self, e: &Executed, event: ExceptionEvent) -> u32 {
        let mut cycles = self.fetch_penalty(e.pc);

        // MEASURED: a read of data from flash which is not in the
        // data cache takes as many cycles longer as there are wait
        // states.  RM0440 3.3.4: that cache holds what the last 8
        // reads which missed returned.
        let reads_flash = e.data == DataClass::Flash
            && e.condition_passed
            && matches!(
                e.decoded.kind,
                Kind::Load | Kind::FpLoad { .. } | Kind::DualOrExclusive { .. }
            );
        if reads_flash && !self.dcache.access(e.data_address) {
            cycles += self.params.flash_wait_states;
        }

        let continues_code_ram_run = e.is_single_load() && e.data == DataClass::CodeRam;
        cycles += match event {
            // MEASURED: a return by POP costs what the POP would
            // have, on top of the return itself.
            ExceptionEvent::Return { fp_frame } => {
                let base = if fp_frame {
                    self.params.exception_return_fp
                } else {
                    self.params.exception_return
                };
                base + match e.decoded.kind {
                    Kind::Multiple { count } => 1 + count,
                    _ => 0,
                }
            }
            ExceptionEvent::TailChain => self.params.tail_chain,
            ExceptionEvent::None | ExceptionEvent::Entry => {
                // An instruction which would have overlapped the load
                // before it instead loses that, if it has to wait to
                // be fetched.
                let mut wait = self.fetch_wait(e);
                let overlaps = (e.is_single_load() || e.is_store())
                    && matches!(self.previous_access(e), Some((false, _)));
                if wait > 0 && overlaps {
                    wait -= 1;
                }
                self.fetch_waited = wait > 0 || !self.fetch_overlap;
                let cycles = self.instruction_cycles(e);
                self.fetch_waited = false;
                self.fetch_during(e, cycles);
                wait + cycles
            }
        };
        if !continues_code_ram_run {
            self.code_ram_run = 0;
        }
        if !e.is_store() {
            self.store_run = 0;
            self.store_rode_load = false;
        }
        if event != ExceptionEvent::None {
            self.fetch_restart(e.next_pc, e.next_is_32bit);
        }
        if event == ExceptionEvent::Entry {
            cycles += self.params.exception_entry;
        }

        let produces = matches!(
            e.decoded.kind,
            Kind::FpArithmetic | Kind::FpMac | Kind::FpDivSqrt | Kind::FpUnary { produces: true }
        );
        self.previous_fp_result = if produces && e.condition_passed {
            e.fp_written | e.decoded.fp_writes
        } else {
            0
        };
        if self.fp_busy > 0 && self.previous_fp_result != 0 {
            self.fp_busy_result = self.previous_fp_result;
        }
        self.previous = if event == ExceptionEvent::None {
            Some(*e)
        } else {
            // Nothing pipelines across an exception boundary.
            self.fetch_word = None;
            None
        };
        self.now += u64::from(cycles);
        cycles
    }

    /// Account for an exception taken between instructions.
    pub fn exception_entry(&mut self) -> u32 {
        self.fetch_ahead = 2;
        self.fetch_loss = None;
        self.previous = None;
        self.previous_fp_result = 0;
        self.fetch_word = None;
        self.code_ram_run = 0;
        self.now += u64::from(self.params.exception_entry);
        self.params.exception_entry
    }

    /// Account for the deferred floating point state save.
    pub fn lazy_fp_save(&mut self) -> u32 {
        self.now += u64::from(self.params.lazy_fp_save);
        self.params.lazy_fp_save
    }

    /// Account for time which passed for some other reason.
    pub fn stall(&mut self, cycles: u32) {
        self.now += u64::from(cycles);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn exec(pc: u32, decoded: Decoded) -> Executed {
        Executed {
            pc,
            decoded,
            condition_passed: true,
            next_pc: pc + decoded.size,
            next_is_32bit: false,
            regs_written: 0,
            fp_written: 0,
            dividend: 0,
            divisor: 1,
            data: if decoded.base == NO_BASE {
                DataClass::None
            } else {
                DataClass::Ram
            },
            data_address: 0,
        }
    }

    const RAM: u32 = 0x1000_0000;

    #[test]
    fn classifies_integer_instructions() {
        // ldr r3, [pc, #136]
        let d = decode16(0x4b22);
        assert_eq!(d.kind, Kind::Load);
        assert!(d.pc_relative);
        assert_eq!(d.base, 15);
        // ldr r3, [r3, #0]
        let d = decode16(0x681b);
        assert_eq!((d.kind, d.address_regs, d.base), (Kind::Load, 1 << 3, 3));
        // str r3, [r1, #12]
        assert_eq!(decode16(0x60cb).kind, Kind::Store);
        // bx lr
        assert_eq!(decode16(0x4770).kind, Kind::BranchRegister { rm: 14 });
        // push {r4, r5, lr}
        assert_eq!(decode16(0xb530).kind, Kind::Multiple { count: 3 });
        // pop {r4, pc}
        let d = decode16(0xbd10);
        assert_eq!(d.kind, Kind::Multiple { count: 2 });
        assert!(d.loads_pc);
        // ite ne
        assert_eq!(decode16(0xbf14).kind, Kind::It);
        // nop
        assert_eq!(decode16(0xbf00).kind, Kind::Nop);
        // cpsid i, cpsie i
        assert_eq!(decode16(0xb672).kind, Kind::System { cycles: 2 });
        assert_eq!(decode16(0xb662).kind, Kind::System { cycles: 1 });
        // ldr.w r2, [r3, #512]
        let d = decode32(0xf8d3, 0x2200);
        assert_eq!((d.kind, d.address_regs), (Kind::Load, 1 << 3));
        // ldr.w pc, [pc]
        let d = decode32(0xf85f, 0xf000);
        assert!(d.loads_pc && d.pc_relative);
        // udiv r0, r0, r2
        assert_eq!(
            decode32(0xfbb0, 0xf0f2).kind,
            Kind::Divide { signed: false }
        );
        // uxtah r0, r1, r0
        assert_eq!(decode32(0xfa11, 0xf080).kind, Kind::Simple);
        // orr.w r3, r3, #0x5f80000
        assert_eq!(decode32(0xf043, 0x63bf).kind, Kind::Simple);
        // bl
        assert_eq!(decode32(0xf000, 0xf800).kind, Kind::Branch);
        // dmb, isb
        assert_eq!(decode32(0xf3bf, 0x8f5f).kind, Kind::System { cycles: 1 });
        assert_eq!(decode32(0xf3bf, 0x8f6f).kind, Kind::System { cycles: 3 });
        // nop.w
        assert_eq!(decode32(0xf3af, 0x8000).kind, Kind::Nop);
        // tbb [pc, r0]
        assert_eq!(decode32(0xe8df, 0xf000).kind, Kind::TableBranch);
        // strd r0, r1, [sp]
        assert_eq!(
            decode32(0xe9cd, 0x0100).kind,
            Kind::DualOrExclusive { cycles: 3 }
        );
        // stmdb sp!, {r4-r11, lr}
        assert_eq!(decode32(0xe92d, 0x4ff0).kind, Kind::Multiple { count: 9 });
    }

    #[test]
    fn classifies_fp_instructions() {
        // vfnma.f32 s14, s15, s7
        let d = decode32(0xee97, 0x7ae3);
        assert_eq!(d.kind, Kind::FpMac);
        assert_eq!(d.fp_reads, (1 << 15) | (1 << 7));
        // vmul.f32 s15, s15, s14
        let d = decode32(0xee67, 0x7a87);
        assert_eq!(d.kind, Kind::FpArithmetic);
        assert_eq!(d.fp_reads, (1 << 15) | (1 << 14));
        // vdiv.f32 s0, s1, s2
        assert_eq!(decode32(0xee80, 0x0a81).kind, Kind::FpDivSqrt);
        // vsqrt.f32 s0, s0
        assert_eq!(decode32(0xeeb1, 0x0ac0).kind, Kind::FpDivSqrt);
        // vldr s15, [r3]
        assert_eq!(
            decode32(0xedd3, 0x7a00).kind,
            Kind::FpLoad { double: false }
        );
        // vstr s15, [r3]
        assert_eq!(
            decode32(0xedc3, 0x7a00).kind,
            Kind::FpStore { double: false }
        );
        // vpush {d8-d9}
        assert_eq!(decode32(0xed2d, 0x8b04).kind, Kind::FpMultiple { words: 4 });
        // vmrs APSR_nzcv, fpscr
        assert_eq!(
            decode32(0xeef1, 0xfa10).kind,
            Kind::FpTransfer {
                cycles: 1,
                to_core: true
            }
        );
        // vcvt.f32.s32 s15, s15
        assert_eq!(
            decode32(0xeef8, 0x7ae7).kind,
            Kind::FpUnary { produces: true }
        );
        // vabs.f32 s0, s0
        assert_eq!(
            decode32(0xeeb0, 0x0ac0).kind,
            Kind::FpUnary { produces: false }
        );
        // vcmpe.f32 s15, s14
        let d = decode32(0xeef4, 0x7ac7);
        assert_eq!(d.kind, Kind::FpUnary { produces: false });
        assert_eq!(d.fp_reads, (1 << 15) | (1 << 14));
    }

    // The behavior of the model as a whole is checked against what was
    // measured on hardware by calibration_test.  These cover only what
    // that cannot.

    #[test]
    fn flash_misses_cost_wait_states() {
        let mut timing = Timing::new(Params::default());
        let nop = |pc| exec(pc, decode16(0xbf00));
        // The first fetch from flash misses, and takes a cycle more
        // than there are wait states.
        assert_eq!(timing.step(&nop(0x0801_0000), ExceptionEvent::None), 6);
        assert_eq!(timing.step(&nop(0x0801_0002), ExceptionEvent::None), 1);
        // The next 64 bits were being read ahead, but are not there
        // yet.
        assert_eq!(timing.step(&nop(0x0801_0008), ExceptionEvent::None), 4);
        // A jump elsewhere misses, but returning hits.
        assert_eq!(timing.step(&nop(0x0802_0000), ExceptionEvent::None), 6);
        assert_eq!(timing.step(&nop(0x0801_0000), ExceptionEvent::None), 1);
        // Nothing executed from RAM is penalized.
        assert_eq!(timing.step(&nop(RAM), ExceptionEvent::None), 1);
    }

    #[test]
    fn exceptions() {
        let mut timing = Timing::new(Params::default());
        let nop = exec(RAM, decode16(0xbf00));
        let bx = exec(RAM, decode16(0x4770));
        assert_eq!(timing.step(&nop, ExceptionEvent::Entry), 13);
        assert_eq!(timing.step(&bx, ExceptionEvent::TailChain), 9);
        assert_eq!(
            timing.step(&bx, ExceptionEvent::Return { fp_frame: false }),
            7
        );
        assert_eq!(
            timing.step(&bx, ExceptionEvent::Return { fp_frame: true }),
            25
        );
    }
}
