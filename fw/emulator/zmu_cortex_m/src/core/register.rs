//!
//! Cortex core register operations
//!

use crate::Processor;
use crate::ProcessorMode;
use crate::core::bits::Bits;
use crate::core::exception::ExceptionHandling;
use crate::core::fault::Fault;
use enum_set::CLike;
use std::fmt;
use std::mem;

///
/// Base register manipulation
///
pub trait BaseReg {
    ///
    /// set PC without touching the T bit
    ///
    fn branch_write_pc(&mut self, address: u32);

    ///
    /// interworking branch
    ///
    fn blx_write_pc(&mut self, address: u32);

    ///
    /// interworking branch
    ///
    fn bx_write_pc(&mut self, address: u32) -> Result<(), Fault>;

    ///
    /// alias for `bx_write_pc`
    ///
    fn load_write_pc(&mut self, address: u32) -> Result<(), Fault>;

    ///
    /// Getter for registers
    ///
    fn get_r(&self, r: Reg) -> u32;

    ///
    /// Setter for registers
    ///
    fn set_r(&mut self, r: Reg, value: u32);

    ///
    /// Setter for MSP
    ///
    fn set_msp(&mut self, value: u32);

    ///
    /// Setter for PSP
    ///
    fn set_psp(&mut self, value: u32);

    ///
    /// Getter for MSP
    fn get_msp(&self) -> u32;

    ///
    /// Getter for PSP
    ///
    fn get_psp(&self) -> u32;

    ///
    /// Increment PC by a value
    ///
    fn add_pc(&mut self, value: u32);

    ///
    /// Get current PC value
    ///
    fn get_pc(&mut self) -> u32;

    ///
    /// Set current PC value with no side effects
    ///
    fn set_pc(&mut self, value: u32);

    ///
    /// add value to register
    ///
    fn add_r(&mut self, r: Reg, value: u32);

    ///
    /// substract value from a register
    ///
    fn sub_r(&mut self, r: Reg, value: u32);
}

///
/// Functionality for operating with floating point data
///
pub trait ExtensionRegOperations {
    ///
    /// Set value of single precision floating point register
    ///
    fn set_sr(&mut self, r: SingleReg, value: u32);

    ///
    /// Set value of double precision floating point register
    ///
    fn set_dr(&mut self, r: DoubleReg, low_word: u32, high_word: u32);

    ///
    /// Get value of single precision floating point register
    ///
    fn get_sr(&mut self, r: SingleReg) -> u32;

    ///
    /// Get value of double precision floating point register
    ///
    fn get_dr(&mut self, r: DoubleReg) -> (u32, u32);
}

impl BaseReg for Processor {
    #[inline(always)]
    fn branch_write_pc(&mut self, address: u32) {
        self.set_pc(address & 0xffff_fffe);
    }

    #[inline(always)]
    fn blx_write_pc(&mut self, address: u32) {
        self.psr.set_t((address & 1) == 1);
        self.branch_write_pc(address);
    }

    #[inline(always)]
    fn bx_write_pc(&mut self, address: u32) -> Result<(), Fault> {
        if self.mode == ProcessorMode::HandlerMode && (address.get_bits(28..32) == 0b1111) {
            self.exception_return(address.get_bits(0..28))
        } else {
            self.blx_write_pc(address);
            Ok(())
        }
    }

    #[inline(always)]
    fn load_write_pc(&mut self, address: u32) -> Result<(), Fault> {
        self.bx_write_pc(address)
    }

    #[inline(always)]
    fn get_r(&self, r: Reg) -> u32 {
        let reg = r as u32;

        if reg < 13 {
            self.r0_12[reg as usize]
        } else {
            match reg {
                13 => {
                    if self.control.sp_sel {
                        self.psp
                    } else {
                        self.msp
                    }
                }
                14 => self.lr,
                15 => self.pc + 4,
                _ => unreachable!(),
            }
        }
    }

    #[inline(always)]
    fn set_r(&mut self, r: Reg, value: u32) {
        let reg = r as u32;

        if reg < 13 {
            self.r0_12[reg as usize] = value;
        } else {
            match reg {
                13 => {
                    if self.control.sp_sel {
                        self.psp = value;
                    } else {
                        self.msp = value;
                    }
                }
                14 => self.lr = value,
                15 => panic!("use branch commands instead"),
                _ => unreachable!(),
            }
        }
    }

    #[inline(always)]
    fn set_msp(&mut self, value: u32) {
        self.msp = value;
    }

    #[inline(always)]
    fn set_psp(&mut self, value: u32) {
        self.psp = value;
    }
    #[inline(always)]
    fn get_msp(&self) -> u32 {
        self.msp
    }

    #[inline(always)]
    fn get_psp(&self) -> u32 {
        self.psp
    }

    #[inline(always)]
    fn add_pc(&mut self, value: u32) {
        self.pc += value;
    }

    #[inline(always)]
    fn get_pc(&mut self) -> u32 {
        self.pc
    }

    #[inline(always)]
    fn set_pc(&mut self, value: u32) {
        self.pc = value;
    }

    //
    // Add value to register
    //
    #[inline(always)]
    fn add_r(&mut self, r: Reg, value: u32) {
        let reg = r as u32;

        if reg < 13 {
            self.r0_12[reg as usize] += value;
        } else {
            match reg {
                13 => {
                    if self.control.sp_sel {
                        self.psp += value;
                    } else {
                        self.msp += value;
                    }
                }
                14 => self.lr += value,
                15 => self.pc += value,
                _ => unreachable!(),
            }
        }
    }
    //
    // Substract value from register
    //
    #[inline(always)]
    fn sub_r(&mut self, r: Reg, value: u32) {
        let reg = r as u32;

        if reg < 13 {
            self.r0_12[reg as usize] -= value;
        } else {
            match reg {
                13 => {
                    if self.control.sp_sel {
                        self.psp -= value;
                    } else {
                        self.msp -= value;
                    }
                }
                14 => self.lr -= value,
                15 => self.pc -= value,
                _ => unreachable!(),
            }
        }
    }
}

impl ExtensionRegOperations for Processor {
    fn set_sr(&mut self, r: SingleReg, value: u32) {
        let index: usize = r.into();
        self.fp_regs[index] = value;
    }
    fn set_dr(&mut self, r: DoubleReg, low_word: u32, high_word: u32) {
        let index: usize = r.into();
        self.fp_regs[index] = low_word;
        self.fp_regs[index + 1] = high_word;
    }

    fn get_sr(&mut self, r: SingleReg) -> u32 {
        let index: usize = r.into();
        self.fp_regs[index]
    }
    fn get_dr(&mut self, r: DoubleReg) -> (u32, u32) {
        let index: usize = r.into();
        (self.fp_regs[index], self.fp_regs[index + 1])
    }
}

#[derive(Debug)]
///
/// Processor Status Registers
/// A combination of multiple sub registers: APSR, IPSR, EPSR
pub struct PSR {
    /// raw register content
    pub value: u32,
}

/// Trait for accessing the sub parts of Application Program Status Register
pub trait Apsr {
    ///
    /// Get "N"egative flag value
    ///
    fn get_n(&self) -> bool;

    ///
    /// Set "N"egative flag value
    ///
    fn set_n(&mut self, result: u32);

    ///
    /// Set N bit
    ///
    fn set_n_bit(&mut self, n: bool);

    ///
    /// Get "Z"ero flag value
    ///
    fn get_z(&self) -> bool;

    ///
    /// Set "Z"ero flag value
    ///
    fn set_z(&mut self, result: u32);

    ///
    /// Set Z bit
    ///
    fn set_z_bit(&mut self, z: bool);

    ///
    /// Get "C"arry flag value
    ///
    fn get_c(&self) -> bool;
    ///
    /// Set "C"arry flag value
    ///
    fn set_c(&mut self, c: bool);

    ///
    /// Get Overflow flag value
    ///
    fn get_v(&self) -> bool;
    ///
    /// Set Overflow flag value
    ///
    fn set_v(&mut self, v: bool);

    ///
    /// Get Saturation flag value
    ///
    fn get_q(&self) -> bool;
    ///
    /// Set Saturation flag value
    ///
    fn set_q(&mut self, q: bool);

    ///
    /// DSP extensions: set GE0 value
    ///
    fn set_ge0(&mut self, bit: bool);
    ///
    /// DSP extensions: set GE1 value
    ///
    fn set_ge1(&mut self, bit: bool);
    ///
    /// DSP extensions: set GE2 value
    ///
    fn set_ge2(&mut self, bit: bool);
    ///
    /// DSP extensions: set GE3 value
    ///
    fn set_ge3(&mut self, bit: bool);

    ///
    /// DSP extensions: get GE0 value
    ///
    fn get_ge0(&self) -> bool;
    ///
    /// DSP extensions: get GE1 value
    ///
    fn get_ge1(&self) -> bool;
    ///
    /// DSP extensions: get GE2 value
    ///
    fn get_ge2(&self) -> bool;
    ///
    /// DSP extensions: get GE3 value
    ///
    fn get_ge3(&self) -> bool;
}

/// Trait for accessing Interrupt Program Status Register subparts
pub trait Ipsr {
    /// get the exception type number of current interrupt service routine
    fn get_isr_number(&self) -> usize;
    /// set the exception type number of current interrupt service routine
    fn set_isr_number(&mut self, exception_number: usize);
}

/// Execution Program Status register
///
/// A view to PSR register containing the data.
pub trait Epsr {
    ///
    /// Set thumb state bit
    ///
    fn set_t(&mut self, t: bool);
    ///
    /// Get thumb state bit
    ///
    fn get_t(&self) -> bool;
}

impl Apsr for PSR {
    #[inline(always)]
    fn get_n(&self) -> bool {
        (self.value & 0x8000_0000) != 0
    }

    #[inline(always)]
    fn set_n(&mut self, result: u32) {
        self.value &= 0x7fff_ffff;
        self.value |= result & 0x8000_0000;
    }

    #[inline(always)]
    fn set_n_bit(&mut self, n: bool) {
        if n {
            self.value |= 0x8000_0000;
        } else {
            self.value &= !0x8000_0000;
        }
    }

    #[inline(always)]
    fn get_z(&self) -> bool {
        (self.value & 0x4000_0000) != 0
    }
    #[inline(always)]
    fn set_z(&mut self, result: u32) {
        if result == 0 {
            self.value |= 0x4000_0000;
        } else {
            self.value &= !0x4000_0000;
        }
    }
    #[inline(always)]
    fn set_z_bit(&mut self, z: bool) {
        if z {
            self.value |= 0x4000_0000;
        } else {
            self.value &= !0x4000_0000;
        }
    }

    #[inline(always)]
    fn get_c(&self) -> bool {
        (self.value & 0x2000_0000) != 0
    }
    #[inline(always)]
    fn set_c(&mut self, c: bool) {
        if c {
            self.value |= 0x2000_0000;
        } else {
            self.value &= !0x2000_0000;
        }
    }
    #[inline(always)]
    fn get_v(&self) -> bool {
        (self.value & 0x1000_0000) != 0
    }
    #[inline(always)]
    fn set_v(&mut self, v: bool) {
        if v {
            self.value |= 0x1000_0000;
        } else {
            self.value &= !0x1000_0000;
        }
    }

    #[inline(always)]
    fn get_q(&self) -> bool {
        self.value.get_bit(27)
    }
    #[inline(always)]
    fn set_q(&mut self, q: bool) {
        self.value.set_bit(27, q);
    }

    fn set_ge0(&mut self, bit: bool) {
        self.value.set_bit(16, bit);
    }
    fn set_ge1(&mut self, bit: bool) {
        self.value.set_bit(17, bit);
    }
    fn set_ge2(&mut self, bit: bool) {
        self.value.set_bit(18, bit);
    }
    fn set_ge3(&mut self, bit: bool) {
        self.value.set_bit(19, bit);
    }

    fn get_ge0(&self) -> bool {
        self.value.get_bit(16)
    }
    fn get_ge1(&self) -> bool {
        self.value.get_bit(17)
    }
    fn get_ge2(&self) -> bool {
        self.value.get_bit(18)
    }
    fn get_ge3(&self) -> bool {
        self.value.get_bit(19)
    }
}

impl Epsr for PSR {
    #[inline(always)]
    fn get_t(&self) -> bool {
        (self.value & (1 << 24)) != 0
    }
    #[inline(always)]
    fn set_t(&mut self, n: bool) {
        if n {
            self.value |= 1 << 24;
        } else {
            self.value &= !(1 << 24);
        }
    }
}

impl Ipsr for PSR {
    #[cfg(not(feature = "armv6m"))]
    #[inline(always)]
    fn get_isr_number(&self) -> usize {
        (self.value & 0x1ff) as usize
    }

    #[cfg(feature = "armv6m")]
    #[inline(always)]
    fn get_isr_number(&self) -> usize {
        (self.value & 0x3f) as usize
    }

    #[cfg(not(feature = "armv6m"))]
    #[inline(always)]
    fn set_isr_number(&mut self, exception_number: usize) {
        self.value = (self.value & !0x1ff) | ((exception_number as u32) & 0x1ff);
    }

    #[cfg(feature = "armv6m")]
    #[inline(always)]
    fn set_isr_number(&mut self, exception_number: usize) {
        self.value = (self.value & !0x3f) | ((exception_number as u32) & 0x3f);
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u32)]
///
/// Basic registers
///
pub enum Reg {
    /// General purpose register 0, also known as a1 (argument 1 register)
    R0,
    /// General purpose register 1, also known as a2 (argument 2 register)
    R1,
    /// General purpose register 2, also known as a3 (argument 3 register)
    R2,
    /// General purpose register 3, also known as a4 (argument 4 register)
    R3,
    /// General purpose register 4, also known as v1 (variable 1 register)
    R4,
    /// General purpose register 5, also known as v2 (variable 2 register)
    R5,
    /// General purpose register 6, also known as v3 (variable 3 register)
    R6,
    /// General purpose register 7, also known as v4 (variable 4 register)
    R7,
    /// General purpose register 8, also known as v5 (variable 5 register)
    R8,
    /// General purpose register 9, also known as v6 (variable 6 register)
    /// Another alias is "sb", static base, used for relocatable code base register.
    R9,
    /// General purpose register 10, also known as v7 (variable 7 register)
    R10,
    /// General purpose register 11, also known as v8 (variable 8 register)
    R11,
    /// General purpose register 12,
    /// also known as IP (Intra procedure call scratch register) or
    /// frame pointer "fp" in some calling conventions
    R12,
    ///
    /// Stack Pointer, alias for R13
    ///
    SP,
    ///
    /// Link Register, alias for R14
    ///
    LR,
    ///
    /// Program Counter, alias for R15
    ///
    PC,
}

#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u32)]
///
/// Single precision floating point registers
///
pub enum SingleReg {
    /// Extension register 0, 32 bit floating point register
    S0,
    /// Extension register 1, 32 bit floating point register
    S1,
    /// Extension register 2, 32 bit floating point register
    S2,
    /// Extension register 3, 32 bit floating point register
    S3,
    /// Extension register 4, 32 bit floating point register
    S4,
    /// Extension register 5, 32 bit floating point register
    S5,
    /// Extension register 6, 32 bit floating point register
    S6,
    /// Extension register 7, 32 bit floating point register
    S7,
    /// Extension register 8, 32 bit floating point register
    S8,
    /// Extension register 9, 32 bit floating point register
    S9,
    /// Extension register 10, 32 bit floating point register
    S10,
    /// Extension register 11, 32 bit floating point register
    S11,
    /// Extension register 12, 32 bit floating point register
    S12,
    /// Extension register 13, 32 bit floating point register
    S13,
    /// Extension register 14, 32 bit floating point register
    S14,
    /// Extension register 15, 32 bit floating point register
    S15,
    /// Extension register 16, 32 bit floating point register
    S16,
    /// Extension register 17, 32 bit floating point register
    S17,
    /// Extension register 18, 32 bit floating point register
    S18,
    /// Extension register 19, 32 bit floating point register
    S19,
    /// Extension register 20, 32 bit floating point register
    S20,
    /// Extension register 21, 32 bit floating point register
    S21,
    /// Extension register 22, 32 bit floating point register
    S22,
    /// Extension register 23, 32 bit floating point register
    S23,
    /// Extension register 24, 32 bit floating point register
    S24,
    /// Extension register 25, 32 bit floating point register
    S25,
    /// Extension register 26, 32 bit floating point register
    S26,
    /// Extension register 27, 32 bit floating point register
    S27,
    /// Extension register 28, 32 bit floating point register
    S28,
    /// Extension register 29, 32 bit floating point register
    S29,
    /// Extension register 30, 32 bit floating point register
    S30,
    /// Extension register 31, 32 bit floating point register
    S31,
}

#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u32)]
///
/// Double precision floating point registers
///
pub enum DoubleReg {
    /// Extension register 0, 64 bit floating point register
    D0,
    /// Extension register 1, 64 bit floating point register
    D1,
    /// Extension register 2, 64 bit floating point register
    D2,
    /// Extension register 3, 64 bit floating point register
    D3,
    /// Extension register 4, 64 bit floating point register
    D4,
    /// Extension register 5, 64 bit floating point register
    D5,
    /// Extension register 6, 64 bit floating point register
    D6,
    /// Extension register 7, 64 bit floating point register
    D7,
    /// Extension register 8, 64 bit floating point register
    D8,
    /// Extension register 9, 64 bit floating point register
    D9,
    /// Extension register 10, 64 bit floating point register
    D10,
    /// Extension register 11, 64 bit floating point register
    D11,
    /// Extension register 12, 64 bit floating point register
    D12,
    /// Extension register 13, 64 bit floating point register
    D13,
    /// Extension register 14, 64 bit floating point register
    D14,
    /// Extension register 15, 64 bit floating point register
    D15,
}
#[derive(Copy, Clone, PartialEq, Debug)]
///
/// Extension registers, either single or double precision floating points
///
pub enum ExtensionReg {
    /// a single precision reg
    Single {
        /// register identification
        reg: SingleReg,
    },
    /// a double precision reg
    Double {
        /// register identification
        reg: DoubleReg,
    },
}

impl ExtensionReg {
    /// extract single precision register if available
    pub fn as_single(&self) -> Option<&SingleReg> {
        match self {
            ExtensionReg::Single { reg } => Some(reg),
            ExtensionReg::Double { .. } => None,
        }
    }

    /// extract double precision register if available
    pub fn as_double(&self) -> Option<&DoubleReg> {
        match self {
            ExtensionReg::Double { reg } => Some(reg),
            ExtensionReg::Single { .. } => None,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(u32)]
#[allow(non_camel_case_types)]
/// Declarations of Special registers, of which some are overlays of same contents
pub enum SpecialReg {
    /// Application Program Status Register, status from previous instructions
    APSR,
    /// Composite of IPSR and APSR
    IAPSR,
    /// Composite of EPSR and APSR
    EAPSR,
    /// composite of all PSR registers
    XPSR,
    /// Interrupt Program Status Register
    IPSR,
    /// Execution Program Status Register
    EPSR,
    /// Interrupt status
    IEPSR,
    /// Refers to Master Stack Pointer
    MSP,
    /// Refers to Process Stack Pointer
    PSP,
    /// To mask out exceptions
    PRIMASK,
    /// The base priority register
    BASEPRI,
    /// Read: alias of basepri, Write: can raise basepri but cannot lower it
    BASEPRI_MAX,
    /// Raise priority to hardfault level
    FAULTMASK,
    /// Special purpose control Register
    CONTROL,
}

impl CLike for Reg {
    fn to_u32(&self) -> u32 {
        *self as u32
    }

    unsafe fn from_u32(v: u32) -> Self {
        unsafe { mem::transmute(v) }
    }
}

impl CLike for SingleReg {
    fn to_u32(&self) -> u32 {
        *self as u32
    }

    unsafe fn from_u32(v: u32) -> Self {
        unsafe { mem::transmute(v) }
    }
}

impl CLike for DoubleReg {
    fn to_u32(&self) -> u32 {
        *self as u32
    }

    unsafe fn from_u32(v: u32) -> Self {
        unsafe { mem::transmute(v) }
    }
}

impl Reg {
    /// convert register to numeric index value
    pub fn value(self) -> usize {
        match self {
            Self::R0 => 0,
            Self::R1 => 1,
            Self::R2 => 2,
            Self::R3 => 3,
            Self::R4 => 4,
            Self::R5 => 5,
            Self::R6 => 6,
            Self::R7 => 7,
            Self::R8 => 8,
            Self::R9 => 9,
            Self::R10 => 10,
            Self::R11 => 11,
            Self::R12 => 12,
            Self::SP => 13,
            Self::LR => 14,
            Self::PC => 15,
        }
    }

    /// convert numeric representation to register
    pub fn from_u16(n: u16) -> Option<Self> {
        match n {
            0 => Some(Self::R0),
            1 => Some(Self::R1),
            2 => Some(Self::R2),
            3 => Some(Self::R3),
            4 => Some(Self::R4),
            5 => Some(Self::R5),
            6 => Some(Self::R6),
            7 => Some(Self::R7),
            8 => Some(Self::R8),
            9 => Some(Self::R9),
            10 => Some(Self::R10),
            11 => Some(Self::R11),
            12 => Some(Self::R12),
            13 => Some(Self::SP),
            14 => Some(Self::LR),
            15 => Some(Self::PC),
            _ => None,
        }
    }
}

impl From<u8> for Reg {
    fn from(value: u8) -> Self {
        match value & 0xf {
            0 => Self::R0,
            1 => Self::R1,
            2 => Self::R2,
            3 => Self::R3,
            4 => Self::R4,
            5 => Self::R5,
            6 => Self::R6,
            7 => Self::R7,
            8 => Self::R8,
            9 => Self::R9,
            10 => Self::R10,
            11 => Self::R11,
            12 => Self::R12,
            13 => Self::SP,
            14 => Self::LR,
            15 => Self::PC,
            _ => Self::R0,
        }
    }
}

impl From<u16> for Reg {
    fn from(value: u16) -> Self {
        match value & 0xf {
            0 => Self::R0,
            1 => Self::R1,
            2 => Self::R2,
            3 => Self::R3,
            4 => Self::R4,
            5 => Self::R5,
            6 => Self::R6,
            7 => Self::R7,
            8 => Self::R8,
            9 => Self::R9,
            10 => Self::R10,
            11 => Self::R11,
            12 => Self::R12,
            13 => Self::SP,
            14 => Self::LR,
            15 => Self::PC,
            _ => Self::R0,
        }
    }
}

impl From<u32> for Reg {
    fn from(value: u32) -> Self {
        match value & 0xf {
            0 => Self::R0,
            1 => Self::R1,
            2 => Self::R2,
            3 => Self::R3,
            4 => Self::R4,
            5 => Self::R5,
            6 => Self::R6,
            7 => Self::R7,
            8 => Self::R8,
            9 => Self::R9,
            10 => Self::R10,
            11 => Self::R11,
            12 => Self::R12,
            13 => Self::SP,
            14 => Self::LR,
            15 => Self::PC,
            _ => Self::R0,
        }
    }
}

impl From<Reg> for u8 {
    fn from(value: Reg) -> Self {
        match value {
            Reg::R0 => 0,
            Reg::R1 => 1,
            Reg::R2 => 2,
            Reg::R3 => 3,
            Reg::R4 => 4,
            Reg::R5 => 5,
            Reg::R6 => 6,
            Reg::R7 => 7,
            Reg::R8 => 8,
            Reg::R9 => 9,
            Reg::R10 => 10,
            Reg::R11 => 11,
            Reg::R12 => 12,
            Reg::SP => 13,
            Reg::LR => 14,
            Reg::PC => 15,
        }
    }
}

impl From<Reg> for usize {
    fn from(value: Reg) -> Self {
        match value {
            Reg::R0 => 0,
            Reg::R1 => 1,
            Reg::R2 => 2,
            Reg::R3 => 3,
            Reg::R4 => 4,
            Reg::R5 => 5,
            Reg::R6 => 6,
            Reg::R7 => 7,
            Reg::R8 => 8,
            Reg::R9 => 9,
            Reg::R10 => 10,
            Reg::R11 => 11,
            Reg::R12 => 12,
            Reg::SP => 13,
            Reg::LR => 14,
            Reg::PC => 15,
        }
    }
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "has-fp")]
    use super::Control;
    use super::{BaseReg, Epsr};
    use crate::{Processor, ProcessorMode};

    #[test]
    fn test_bx_write_pc_even_address_remains_out_of_scope_for_invstate_todo() {
        let mut processor = Processor::new();
        processor.mode = ProcessorMode::ThreadMode;
        processor.psr.set_t(true);

        let result = processor.bx_write_pc(0x0800_0000);

        assert_eq!(result, Ok(()));
        assert_eq!(processor.get_pc(), 0x0800_0000);
        assert!(!processor.psr.get_t());
    }

    #[test]
    fn test_load_write_pc_even_address_remains_out_of_scope_for_invstate_todo() {
        let mut processor = Processor::new();
        processor.mode = ProcessorMode::ThreadMode;
        processor.psr.set_t(true);

        let result = processor.load_write_pc(0x0800_0000);

        assert_eq!(result, Ok(()));
        assert_eq!(processor.get_pc(), 0x0800_0000);
        assert!(!processor.psr.get_t());
    }

    #[test]
    #[cfg(feature = "has-fp")]
    fn test_control_to_u8_includes_fpca_bit() {
        let control = Control {
            n_priv: true,
            sp_sel: true,
            fpca: true,
        };

        assert_eq!(u8::from(control), 0b111);
    }
}

impl SpecialReg {
    /// decode 16 bit value to Special Register designator
    pub fn from_u16(n: u16) -> Option<Self> {
        match n {
            0 => Some(Self::APSR),
            1 => Some(Self::IAPSR),
            2 => Some(Self::EAPSR),
            3 => Some(Self::XPSR),
            5 => Some(Self::IPSR),
            6 => Some(Self::EPSR),
            7 => Some(Self::IEPSR),
            8 => Some(Self::MSP),
            9 => Some(Self::PSP),
            16 => Some(Self::PRIMASK),
            17 => Some(Self::BASEPRI),
            18 => Some(Self::BASEPRI_MAX),
            19 => Some(Self::FAULTMASK),
            20 => Some(Self::CONTROL),
            _ => None,
        }
    }
}

impl From<SpecialReg> for u8 {
    fn from(value: SpecialReg) -> Self {
        match value {
            SpecialReg::APSR => 0,
            SpecialReg::IAPSR => 1,
            SpecialReg::EAPSR => 2,
            SpecialReg::XPSR => 3,
            SpecialReg::IPSR => 5,
            SpecialReg::EPSR => 6,
            SpecialReg::IEPSR => 7,
            SpecialReg::MSP => 8,
            SpecialReg::PSP => 9,
            SpecialReg::PRIMASK => 16,
            SpecialReg::BASEPRI => 17,
            SpecialReg::BASEPRI_MAX => 18,
            SpecialReg::FAULTMASK => 19,
            SpecialReg::CONTROL => 20,
        }
    }
}

impl From<u8> for SpecialReg {
    fn from(value: u8) -> Self {
        match value & 0x1f {
            0 => Self::APSR,
            1 => Self::IAPSR,
            2 => Self::EAPSR,
            3 => Self::XPSR,
            5 => Self::IPSR,
            6 => Self::EPSR,
            7 => Self::IEPSR,
            8 => Self::MSP,
            9 => Self::PSP,
            16 => Self::PRIMASK,
            20 => Self::CONTROL,
            _ => Self::APSR,
        }
    }
}

impl fmt::Display for Reg {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::R0 => write!(f, "r0"),
            Self::R1 => write!(f, "r1"),
            Self::R2 => write!(f, "r2"),
            Self::R3 => write!(f, "r3"),
            Self::R4 => write!(f, "r4"),
            Self::R5 => write!(f, "r5"),
            Self::R6 => write!(f, "r6"),
            Self::R7 => write!(f, "r7"),
            Self::R8 => write!(f, "r8"),
            Self::R9 => write!(f, "r9"),
            Self::R10 => write!(f, "r10"),
            Self::R11 => write!(f, "r11"),
            Self::R12 => write!(f, "r12"),
            Self::SP => write!(f, "sp"),
            Self::LR => write!(f, "lr"),
            Self::PC => write!(f, "pc"),
        }
    }
}

impl fmt::Display for ExtensionReg {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Single { reg } => write!(f, "{reg}"),
            Self::Double { reg } => write!(f, "{reg}"),
        }
    }
}

impl fmt::Display for SingleReg {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            SingleReg::S0 => write!(f, "s0"),
            SingleReg::S1 => write!(f, "s1"),
            SingleReg::S2 => write!(f, "s2"),
            SingleReg::S3 => write!(f, "s3"),
            SingleReg::S4 => write!(f, "s4"),
            SingleReg::S5 => write!(f, "s5"),
            SingleReg::S6 => write!(f, "s6"),
            SingleReg::S7 => write!(f, "s7"),
            SingleReg::S8 => write!(f, "s8"),
            SingleReg::S9 => write!(f, "s9"),
            SingleReg::S10 => write!(f, "s10"),
            SingleReg::S11 => write!(f, "s11"),
            SingleReg::S12 => write!(f, "s12"),
            SingleReg::S13 => write!(f, "s13"),
            SingleReg::S14 => write!(f, "s14"),
            SingleReg::S15 => write!(f, "s15"),
            SingleReg::S16 => write!(f, "s16"),
            SingleReg::S17 => write!(f, "s17"),
            SingleReg::S18 => write!(f, "s18"),
            SingleReg::S19 => write!(f, "s19"),
            SingleReg::S20 => write!(f, "s20"),
            SingleReg::S21 => write!(f, "s21"),
            SingleReg::S22 => write!(f, "s22"),
            SingleReg::S23 => write!(f, "s23"),
            SingleReg::S24 => write!(f, "s24"),
            SingleReg::S25 => write!(f, "s25"),
            SingleReg::S26 => write!(f, "s26"),
            SingleReg::S27 => write!(f, "s27"),
            SingleReg::S28 => write!(f, "s28"),
            SingleReg::S29 => write!(f, "s29"),
            SingleReg::S30 => write!(f, "s30"),
            SingleReg::S31 => write!(f, "s31"),
        }
    }
}

impl fmt::Display for DoubleReg {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            DoubleReg::D0 => write!(f, "d0"),
            DoubleReg::D1 => write!(f, "d1"),
            DoubleReg::D2 => write!(f, "d2"),
            DoubleReg::D3 => write!(f, "d3"),
            DoubleReg::D4 => write!(f, "d4"),
            DoubleReg::D5 => write!(f, "d5"),
            DoubleReg::D6 => write!(f, "d6"),
            DoubleReg::D7 => write!(f, "d7"),
            DoubleReg::D8 => write!(f, "d8"),
            DoubleReg::D9 => write!(f, "d9"),
            DoubleReg::D10 => write!(f, "s10"),
            DoubleReg::D11 => write!(f, "s11"),
            DoubleReg::D12 => write!(f, "s12"),
            DoubleReg::D13 => write!(f, "s14"),
            DoubleReg::D14 => write!(f, "s15"),
            DoubleReg::D15 => write!(f, "s16"),
        }
    }
}

impl fmt::Display for SpecialReg {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::APSR => write!(f, "APSR"),
            Self::IAPSR => write!(f, "IAPSR"),
            Self::EAPSR => write!(f, "EAPSR"),
            Self::XPSR => write!(f, "XPSR"),
            Self::IPSR => write!(f, "IPSR"),
            Self::EPSR => write!(f, "EPSR"),
            Self::IEPSR => write!(f, "IEPSR"),
            Self::MSP => write!(f, "MSP"),
            Self::PSP => write!(f, "PSP"),
            Self::PRIMASK => write!(f, "PRIMASK"),
            Self::BASEPRI => write!(f, "BASEPRI"),
            Self::BASEPRI_MAX => write!(f, "BASEPRI_MAX"),
            Self::FAULTMASK => write!(f, "FAULTMASK"),
            Self::CONTROL => write!(f, "CONTROL"),
        }
    }
}

#[derive(Debug)]
/// CONTROL register parts
pub struct Control {
    /// Thread mode priviledge level
    pub n_priv: bool,
    /// selection of current active stack pointer, true = PSP, false = MSP
    pub sp_sel: bool,
    /// FPCA bit, whether fp context is active in current state
    #[cfg(feature = "has-fp")]
    pub fpca: bool,
}

impl From<Control> for u8 {
    #[cfg(feature = "has-fp")]
    fn from(control: Control) -> Self {
        Self::from(control.n_priv)
            + (Self::from(control.sp_sel) << 1)
            + (Self::from(control.fpca) << 2)
    }

    #[cfg(not(feature = "has-fp"))]
    fn from(control: Control) -> Self {
        Self::from(control.n_priv) + (Self::from(control.sp_sel) << 1)
    }
}

///
/// for indexing floating point register array
///
impl From<DoubleReg> for usize {
    fn from(value: DoubleReg) -> Self {
        match value {
            DoubleReg::D0 => 0,
            DoubleReg::D1 => 2,
            DoubleReg::D2 => 4,
            DoubleReg::D3 => 6,
            DoubleReg::D4 => 8,
            DoubleReg::D5 => 10,
            DoubleReg::D6 => 12,
            DoubleReg::D7 => 14,
            DoubleReg::D8 => 16,
            DoubleReg::D9 => 18,
            DoubleReg::D10 => 20,
            DoubleReg::D11 => 22,
            DoubleReg::D12 => 24,
            DoubleReg::D13 => 26,
            DoubleReg::D14 => 28,
            DoubleReg::D15 => 30,
        }
    }
}

///
/// for indexing floating point register array
///
impl From<SingleReg> for usize {
    fn from(value: SingleReg) -> Self {
        match value {
            SingleReg::S0 => 0,
            SingleReg::S1 => 1,
            SingleReg::S2 => 2,
            SingleReg::S3 => 3,
            SingleReg::S4 => 4,
            SingleReg::S5 => 5,
            SingleReg::S6 => 6,
            SingleReg::S7 => 7,
            SingleReg::S8 => 8,
            SingleReg::S9 => 9,
            SingleReg::S10 => 10,
            SingleReg::S11 => 11,
            SingleReg::S12 => 12,
            SingleReg::S13 => 13,
            SingleReg::S14 => 14,
            SingleReg::S15 => 15,
            SingleReg::S16 => 16,
            SingleReg::S17 => 17,
            SingleReg::S18 => 18,
            SingleReg::S19 => 19,
            SingleReg::S20 => 20,
            SingleReg::S21 => 21,
            SingleReg::S22 => 22,
            SingleReg::S23 => 23,
            SingleReg::S24 => 24,
            SingleReg::S25 => 25,
            SingleReg::S26 => 26,
            SingleReg::S27 => 27,
            SingleReg::S28 => 28,
            SingleReg::S29 => 29,
            SingleReg::S30 => 30,
            SingleReg::S31 => 31,
        }
    }
}

impl From<u8> for DoubleReg {
    fn from(value: u8) -> Self {
        match value & 0xf {
            0 => Self::D0,
            1 => Self::D1,
            2 => Self::D2,
            3 => Self::D3,
            4 => Self::D4,
            5 => Self::D5,
            6 => Self::D6,
            7 => Self::D7,
            8 => Self::D8,
            9 => Self::D9,
            10 => Self::D10,
            11 => Self::D11,
            12 => Self::D12,
            13 => Self::D13,
            14 => Self::D14,
            15 => Self::D15,
            _ => Self::D0,
        }
    }
}

impl From<u8> for SingleReg {
    fn from(value: u8) -> Self {
        match value & 0x1f {
            0 => Self::S0,
            1 => Self::S1,
            2 => Self::S2,
            3 => Self::S3,
            4 => Self::S4,
            5 => Self::S5,
            6 => Self::S6,
            7 => Self::S7,
            8 => Self::S8,
            9 => Self::S9,
            10 => Self::S10,
            11 => Self::S11,
            12 => Self::S12,
            13 => Self::S13,
            14 => Self::S14,
            15 => Self::S15,
            16 => Self::S16,
            17 => Self::S17,
            18 => Self::S18,
            19 => Self::S19,
            20 => Self::S20,
            21 => Self::S21,
            22 => Self::S22,
            23 => Self::S23,
            24 => Self::S24,
            25 => Self::S25,
            26 => Self::S26,
            27 => Self::S27,
            28 => Self::S28,
            29 => Self::S29,
            30 => Self::S30,
            31 => Self::S31,
            _ => Self::S0,
        }
    }
}
