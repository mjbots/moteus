//!
//! Representation of Cortex M Instruction set
//!

use crate::core::condition::Condition;
use crate::core::register::{DoubleReg, ExtensionReg, Reg, SingleReg};
use crate::core::thumb::ThumbCode;
use enum_set::EnumSet;
use std::fmt::Display;
use std::fmt::Formatter;

#[derive(Debug, PartialEq, Copy, Clone)]
///
/// Types of shift operations supported
pub enum SRType {
    /// logical shift left
    LSL,
    /// logical shift right
    LSR,
    /// arithmetic shift right
    ASR,
    /// rotate right
    RRX,
    /// rotate right
    ROR,
}

#[derive(Debug, PartialEq, Copy, Clone)]
/// IT instruction conditions
pub enum ITCondition {
    /// condition normal operation
    Then,
    /// condition inverted operation
    Else,
}

#[derive(PartialEq, Debug, Copy, Clone)]
///
/// Coding of imm32+carry variants for more efficient run time behaviour
///
pub enum Imm32Carry {
    /// Precalculated value carry value was not relevant
    /// for the decoding.
    NoCarry {
        /// imm32 original value
        imm32: u32,
    },
    /// Precalculated values for cases ASPR.C == 0 and ASPR.C ==1
    /// If carry was relevant for the decoding
    /// tuple of (immediate value, carry).
    Carry {
        /// Values of imm32 and carry, when carry was originally 0.
        imm32_c0: (u32, bool),
        /// Values of imm32 and carry, when carry was originally 1.
        imm32_c1: (u32, bool),
    },
}

#[derive(PartialEq, Debug, Copy, Clone)]
/// Instruction flags setting variants
pub enum SetFlags {
    /// Set Always
    True,
    /// Set Never
    False,
    /// Set when not in "IT" block
    NotInITBlock,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct CondBranchParams {
    pub cond: Condition,
    pub imm32: i32,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg3ShiftParams {
    pub rd: Reg,
    pub rn: Reg,
    pub rm: Reg,
    pub shift_t: SRType,
    pub shift_n: u8,
    pub setflags: SetFlags,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg3Params {
    pub rd: Reg,
    pub rn: Reg,
    pub rm: Reg,
    pub setflags: SetFlags,
}
#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg3NoSetFlagsParams {
    pub rd: Reg,
    pub rn: Reg,
    pub rm: Reg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg3FullParams {
    pub rt: Reg,
    pub rn: Reg,
    pub rm: Reg,
    pub shift_t: SRType,
    pub shift_n: u8,
    pub index: bool,
    pub add: bool,
    pub wback: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2FullParams {
    pub rt: Reg,
    pub rn: Reg,
    pub imm32: u32,
    pub index: bool,
    pub add: bool,
    pub wback: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct RegImm32AddParams {
    pub rt: Reg,
    pub imm32: u32,
    pub add: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2DoubleParams {
    pub rn: Reg,
    pub rt: Reg,
    pub rt2: Reg,
    pub imm32: u32,
    pub index: bool,
    pub add: bool,
    pub wback: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct MrsParams {
    pub rd: Reg,
    pub sysm: u8,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct MsrParams {
    pub rn: Reg,
    pub sysm: u8,
    pub mask: u8,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2UsizeParams {
    pub rd: Reg,
    pub rm: Reg,
    pub rotation: usize,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg3UsizeParams {
    pub rd: Reg,
    pub rm: Reg,
    pub rn: Reg,
    pub rotation: usize,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg4NoSetFlagsParams {
    pub rd: Reg,
    pub rn: Reg,
    pub rm: Reg,
    pub ra: Reg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2ShiftParams {
    pub rd: Reg,
    pub rm: Reg,
    pub shift_t: SRType,
    pub shift_n: u8,
    pub setflags: SetFlags,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2ShiftNParams {
    pub rd: Reg,
    pub rm: Reg,
    pub shift_n: u8,
    pub setflags: SetFlags,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2Params {
    pub rd: Reg,
    pub rm: Reg,
    pub setflags: SetFlags,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2ImmParams {
    pub rd: Reg,
    pub rn: Reg,
    pub imm32: u32,
    pub setflags: SetFlags,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2ImmCarryParams {
    pub rd: Reg,
    pub rn: Reg,
    pub imm32: Imm32Carry,
    pub setflags: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct RegImmCarryParams {
    pub rd: Reg,
    pub imm32: Imm32Carry,
    pub setflags: SetFlags,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct RegImmCarryNoSetFlagsParams {
    pub rn: Reg,
    pub imm32: Imm32Carry,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2ShiftNoSetFlagsParams {
    pub rn: Reg,
    pub rm: Reg,
    pub shift_t: SRType,
    pub shift_n: u8,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2RnRmParams {
    pub rn: Reg,
    pub rm: Reg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2RdRmParams {
    pub rd: Reg,
    pub rm: Reg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2RtRnParams {
    pub rt: Reg,
    pub rn: Reg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg3RdRtRnParams {
    pub rd: Reg,
    pub rt: Reg,
    pub rn: Reg,
}

impl Reg3RdRtRnParams {
    /// Check if the instruction parameters are unpredictable
    pub fn strex_is_unpredictable(&self) -> bool {
        self.rd == Reg::PC
            || self.rt == Reg::PC
            || self.rn == Reg::PC
            || self.rd == Reg::SP
            || self.rt == Reg::SP
            || self.rd == self.rn
            || self.rd == self.rt
    }
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct LoadAndStoreMultipleParams {
    pub rn: Reg,
    pub registers: EnumSet<Reg>,
    pub wback: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VAddSubParamsf64 {
    pub dd: DoubleReg,
    pub dn: DoubleReg,
    pub dm: DoubleReg,
}
#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VAddSubParamsf32 {
    pub sd: SingleReg,
    pub sn: SingleReg,
    pub sm: SingleReg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VLoadAndStoreParams {
    pub dd: ExtensionReg,
    pub rn: Reg,
    pub add: bool,
    pub imm32: u32,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VPushPopParams {
    pub single_regs: bool,
    pub single_precision_registers: EnumSet<SingleReg>,
    pub double_precision_registers: EnumSet<DoubleReg>,
    pub imm32: u32,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub enum AddressingMode {
    IncrementAfter,
    DecrementBefore,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VStoreMultipleParams32 {
    pub mode: AddressingMode,
    pub rn: Reg,
    pub write_back: bool,
    pub list: EnumSet<SingleReg>,
    pub imm32: u32,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VStoreMultipleParams64 {
    pub mode: AddressingMode,
    pub rn: Reg,
    pub write_back: bool,
    pub list: EnumSet<DoubleReg>,
    pub imm32: u32,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VMovImmParams32 {
    pub sd: SingleReg,
    pub imm32: u32,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VMovImmParams64 {
    pub dd: DoubleReg,
    pub imm64: u64,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VMovRegParamsf32 {
    pub sd: SingleReg,
    pub sm: SingleReg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VCmpParamsf32 {
    pub sd: SingleReg,
    pub sm: SingleReg,
    pub quiet_nan_exc: bool,
    pub with_zero: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VCmpParamsf64 {
    pub dd: DoubleReg,
    pub dm: DoubleReg,
    pub quiet_nan_exc: bool,
    pub with_zero: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VCVTParams {
    pub d: ExtensionReg,
    pub m: ExtensionReg,
    pub to_integer: bool,
    pub unsigned: bool,
    pub dp_operation: bool,
    pub round_zero: bool,
    pub round_nearest: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VCVTParamsF64F32 {
    pub dd: DoubleReg,
    pub sm: SingleReg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VCVTParamsF32F64 {
    pub sd: SingleReg,
    pub dm: DoubleReg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VSelParamsf32 {
    pub sd: SingleReg,
    pub sn: SingleReg,
    pub sm: SingleReg,
    pub cond: Condition,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VSelParamsf64 {
    pub dd: DoubleReg,
    pub dn: DoubleReg,
    pub dm: DoubleReg,
    pub cond: Condition,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VMovRegParamsf64 {
    pub dd: DoubleReg,
    pub dm: DoubleReg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VMovCrScalarParams {
    pub rt: Reg,
    pub dd: DoubleReg,
    pub x: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VMovCrSpParams {
    pub to_arm_register: bool,
    pub rt: Reg,
    pub sn: SingleReg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VMovCr2Sp2Params {
    pub to_arm_registers: bool,
    pub rt: Reg,
    pub rt2: Reg,
    pub sm: SingleReg,
    pub sm1: SingleReg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct VMovCr2DpParams {
    pub to_arm_registers: bool,
    pub rt: Reg,
    pub rt2: Reg,
    pub dm: DoubleReg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg3RdRtRnImm32Params {
    pub rd: Reg,
    pub rt: Reg,
    pub rn: Reg,
    pub imm32: u32,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg2RtRnImm32Params {
    pub rt: Reg,
    pub rn: Reg,
    pub imm32: u32,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct BfxParams {
    pub rd: Reg,
    pub rn: Reg,
    pub lsb: usize,
    pub widthminus1: usize,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct MovtParams {
    pub rd: Reg,
    pub imm16: u16,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct RegImmParams {
    pub r: Reg,
    pub imm32: u32,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg643232Params {
    pub rdlo: Reg,
    pub rdhi: Reg,
    pub rm: Reg,
    pub rn: Reg,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg3HighParams {
    pub rd: Reg,
    pub rn: Reg,
    pub rm: Reg,
    pub n_high: bool,
    pub m_high: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Reg4HighParams {
    pub rd: Reg,
    pub rn: Reg,
    pub rm: Reg,
    pub ra: Reg,
    pub n_high: bool,
    pub m_high: bool,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct ParamsRegImm32 {
    pub rn: Reg,
    pub imm32: u32,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct BfcParams {
    pub rd: Reg,
    pub lsbit: usize,
    pub msbit: usize,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct BfiParams {
    pub rd: Reg,
    pub rn: Reg,
    pub lsbit: usize,
    pub width: usize,
}

#[allow(missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub enum VMRSTarget {
    APSRNZCV,
    Register(Reg),
}
impl Display for VMRSTarget {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            VMRSTarget::APSRNZCV => write!(f, "APSR_nzcv"),
            VMRSTarget::Register(r) => write!(f, "{r:?}"),
        }
    }
}

#[allow(non_camel_case_types, missing_docs)]
#[derive(PartialEq, Debug, Copy, Clone)]
///
/// Instruction set
/// These "micro instructions" are produced by the decoder
/// and operated on by the executor.
/// Note that the instruction list is not 1:1 to
/// the mnemonics listed in the ref manual, instead
/// the exact variant is decoded to allow faster runtime.
pub enum Instruction {
    // --------------------------------------------
    //
    // Group: Branch instructions
    //
    // --------------------------------------------
    /// Branch to target address (on condition)
    B_t13 {
        params: CondBranchParams,
        thumb32: bool,
    },
    /// Branch to target address
    B_t24 {
        imm32: i32,
        thumb32: bool,
    },
    /// Call a subroutine
    BL {
        imm32: i32,
    },
    /// Call a subroutine, optionally change instruction set
    BLX {
        rm: Reg,
    },
    /// Change to target address, change instruction set
    BX {
        rm: Reg,
    },
    /// Compare and branch on  Zero
    CBZ {
        params: ParamsRegImm32,
    },
    /// Compare and branch on Nonzero
    CBNZ {
        params: ParamsRegImm32,
    },
    /// Table branch, byte offsets
    TBB {
        params: Reg2RnRmParams,
    },
    /// Table branch, halfword offsets
    TBH {
        params: Reg2RnRmParams,
    },

    // --------------------------------------------
    //
    // Group: Standard data-processing instructions
    //
    // --------------------------------------------
    /// Add (immediate)
    ADD_imm {
        params: Reg2ImmParams,
        thumb32: bool,
    },
    /// Add (register)
    ADD_reg {
        params: Reg3ShiftParams,
        thumb32: bool,
    },
    /// Add (register, sp)
    ADD_sp_reg {
        params: Reg2ShiftParams,
        thumb32: bool,
    },
    /// Add with Carry
    ADC_reg {
        params: Reg3ShiftParams,
        thumb32: bool,
    },
    /// Add with Carry
    ADC_imm {
        params: Reg2ImmParams,
    },

    /// Form PC-relative Address
    ADR {
        params: RegImmParams,
        thumb32: bool,
    },

    /// Bitwise AND
    AND_reg {
        params: Reg3ShiftParams,
        thumb32: bool,
    },
    /// Bitwise AND
    AND_imm {
        params: Reg2ImmCarryParams,
    },

    /// Bitwise Bit Clear
    BIC_reg {
        params: Reg3ShiftParams,
        thumb32: bool,
    },
    /// Bitwise Bit Clear
    BIC_imm {
        params: Reg2ImmCarryParams,
    },

    /// Compare Negative
    CMN_reg {
        params: Reg2ShiftNoSetFlagsParams,
        thumb32: bool,
    },
    /// Compare Negative
    CMN_imm {
        params: RegImmParams,
    },

    /// Compare
    CMP_imm {
        params: RegImmParams,
        thumb32: bool,
    },
    /// Compare
    CMP_reg {
        params: Reg2ShiftNoSetFlagsParams,
        thumb32: bool,
    },

    /// Bitwise Exclusive OR
    EOR_reg {
        params: Reg3ShiftParams,
        thumb32: bool,
    },
    /// Bitwise Exclusive OR
    EOR_imm {
        params: Reg2ImmCarryParams,
    },

    /// Copies operand to destination
    MOV_imm {
        params: RegImmCarryParams,
        thumb32: bool,
    },
    /// Copies operand to destination
    MOV_reg {
        params: Reg2Params,
        thumb32: bool,
    },

    /// Bitwise NOT
    MVN_reg {
        params: Reg2ShiftParams,
        thumb32: bool,
    },
    /// Bitwise NOT
    MVN_imm {
        params: RegImmCarryParams,
    },
    /// Bitwise OR NOT
    ORN_reg {
        params: Reg3ShiftParams,
    },

    /// Bitwise OR
    ORR_reg {
        params: Reg3ShiftParams,
        thumb32: bool,
    },
    /// Bitwise OR
    ORR_imm {
        params: Reg2ImmCarryParams,
    },

    /// Reverse subtract
    RSB_imm {
        params: Reg2ImmParams,
        thumb32: bool,
    },
    /// Reverse subtract
    RSB_reg {
        params: Reg3ShiftParams,
        thumb32: bool,
    },

    /// Subtract with Carry
    SBC_reg {
        params: Reg3ShiftParams,
        thumb32: bool,
    },
    /// Subtract with Carry
    SBC_imm {
        params: Reg2ImmParams,
    },

    /// Subtract
    SUB_imm {
        params: Reg2ImmParams,
        thumb32: bool,
    },
    /// Subtract
    SUB_reg {
        params: Reg3ShiftParams,
        thumb32: bool,
    },

    /// Test equivalence
    TEQ_reg {
        params: Reg2ShiftNoSetFlagsParams,
    },
    /// Test equivalence
    TEQ_imm {
        params: RegImmCarryNoSetFlagsParams,
    },

    /// Test
    TST_reg {
        params: Reg2ShiftNoSetFlagsParams,
        thumb32: bool,
    },
    /// Test
    TST_imm {
        params: RegImmCarryNoSetFlagsParams,
    },

    // --------------------------------------------
    //
    // Group: Shift instructions
    //
    // --------------------------------------------
    /// Arithmetic shift right
    ASR_imm {
        params: Reg2ShiftNParams,
        thumb32: bool,
    },
    /// Arithmetic shift right
    ASR_reg {
        params: Reg3Params,
        thumb32: bool,
    },
    /// Logical Shift Left (immediate)
    LSL_imm {
        params: Reg2ShiftNParams,
        thumb32: bool,
    },
    /// Logical Shift Left (register)
    LSL_reg {
        params: Reg3Params,
        thumb32: bool,
    },
    /// Logical Shift Right (immediate)
    LSR_imm {
        params: Reg2ShiftNParams,
        thumb32: bool,
    },
    /// Logical Shift Right (register)
    LSR_reg {
        params: Reg3Params,
        thumb32: bool,
    },
    /// Rotate Right (immediate)
    ROR_imm {
        params: Reg2ShiftNParams,
    },
    /// Rotate Right (register)
    ROR_reg {
        params: Reg3Params,
        thumb32: bool,
    },
    /// Rotate Right with Extend
    RRX {
        params: Reg2Params,
    },

    // --------------------------------------------
    //
    // Group: Multiply instructions
    //
    // --------------------------------------------

    //
    // Subgroup: General multiply instructions
    //
    /// Multipy and Accumulate
    MLA {
        params: Reg4NoSetFlagsParams,
    },
    /// Multipy and Subtract
    MLS {
        params: Reg4NoSetFlagsParams,
    },
    /// Multipy
    MUL {
        params: Reg3Params,
        thumb32: bool,
    },

    //
    // Subgroup: Signed multiply instructions (ArmV7-m)
    //
    /// Signed Multiply and Accumulate (Long)
    SMLAL {
        params: Reg643232Params,
    },
    /// Signed Multiply (Long)
    SMULL {
        params: Reg643232Params,
    },

    //
    // Subgroup: Signed Multiply instructions (ARMv7-M DSP extension)
    //
    /// Signed multiply: halfwords
    /// variants: SMULTT, SMULBB, SMULTB, SMULBT
    SMUL {
        params: Reg3HighParams,
    },
    /// Signed multiply and Accumulate, halfwords
    /// variants: SMLATT, SMLABB, SMLATB, SMLABT
    SMLA {
        params: Reg4HighParams,
    },

    //SMLAL second variant?

    // Signed Multiply Accumulate Long, halfwords
    // SMLABB, SMLABT, SMLATT, SMLATB

    // Signed Multiply Accumulate Dual
    // SMLAD, SMLADX

    // Signed Multiply Accumulate Long, halfwords
    // SMLALBB, SMLALBT, SMLALTT, SMLALTB

    // Signed Multiply Accumulate Long Dual
    // SMLALD, SMLALDX

    // Signed Multiply Accumulate, word by halfword
    // SMLAWB, SMLAWT

    // Signed Multiply Subtract Dual
    // SMLSD, SMLSDX

    // Signed Multiply Subtract Long Dual
    // SMLSLD, SMLSLDX

    // Signed most significant word Multiply Accumulate
    // SMMLA, SMMLAR

    // Signed most significant word Multiply Subtract
    // SMMLS, SMMLSR

    // Signed most significant Word Multiply
    // SMMUL, SMMULR

    // Signed Dual Multiply Add
    // SMUAD, SMUADX

    // Signed Multiply, halfwords
    // SMULBB, SMULBT, SMULTB, SMULTT

    // Signed Multiply, word by halfword
    // SMULWB, SMULWT

    // Signed Dual Multiply Subtract
    // SMUSD, SMUSDX

    //
    // Subgroup: Unsigned Multiply instructions (ARMv7-M base architecture)
    //
    UMLAL {
        params: Reg643232Params,
    },
    UMULL {
        params: Reg643232Params,
    },

    //
    // Subgroup: Unsigned multiply instructions (Armv7-M DSP extension)
    //
    // UMAAL

    // --------------------------------------------
    //
    // Group: Saturating instructions
    //
    // --------------------------------------------

    //
    // Subgroup: Saturating instructions (ARMv7-M base arch)
    //
    //SSAT
    //USAT

    //
    // Subgroup: Halfword saturating instructions, (ARMv7-M DSP extensions)
    //
    //USAT16
    //SSAT16

    //
    // Subgroup: Saturating addition and subtraction instructions, Armv7-M DSP extension
    //
    //QADD
    //QSUB
    //QDADD
    //QDSUB

    // --------------------------------------------
    //
    // Group: Packing and unpacking instructions
    //
    // --------------------------------------------

    //
    // Subgroup: Packing and unpacking instructions (ARMv7-M base arch)
    //
    /// Signed Extend Byte
    SXTB {
        params: Reg2UsizeParams,
        thumb32: bool,
    },
    /// Signed Extend Halfword
    SXTH {
        params: Reg2UsizeParams,
        thumb32: bool,
    },
    /// Unsigned Extend Byte
    UXTB {
        params: Reg2UsizeParams,
        thumb32: bool,
    },
    /// Unsigned Extend Halfword
    UXTH {
        params: Reg2UsizeParams,
        thumb32: bool,
    },

    //
    // Subgroup: Packing and unpacking instructions, Armv7-M DSP extension
    //
    //PKHBT, PKHTB
    //SXTAB
    //SXTAB16
    //SXTAH
    //SXTB16
    UXTAB {
        params: Reg3UsizeParams,
    },
    //UXTAB16
    //UXTAH
    //UXTB16

    // --------------------------------------------
    //
    // Group: Divide instructions
    //
    // --------------------------------------------
    /// signed divide
    SDIV {
        params: Reg3NoSetFlagsParams,
    },
    /// Unsigned divide
    UDIV {
        params: Reg3NoSetFlagsParams,
    },

    // --------------------------------------------
    //
    // Group: Parallel add / sub (DSP extension)
    //
    // --------------------------------------------
    //SADD16, QADD16, SHADD16, UADD16, UQADD16, UHADD16
    //SASX, QASX, SHASX, UASX, UQASX, UHSX
    //SSAX, QSAX, SHSAX, USAX, UQSAX, UHSAX
    //SSUB16, QSUB16, SHSUB16, USUB16, UQSUB16, UHSUB16
    //SADD8, QADD8, SHADD8, UADD8, UQADD8, UHADD8
    //SSUB8, QSUB8, SHSUB8, USUB8, UQSUB8, UHSUB8
    UADD8 {
        params: Reg3NoSetFlagsParams,
    },

    // --------------------------------------------
    //
    // Group: Miscellaneous data-processing instructions
    //
    // --------------------------------------------

    //
    // Subgroup: Miscellaneous data-processing instructions, Armv7-M base architecture
    //
    /// Bit Field Clear
    BFC {
        params: BfcParams,
    },
    /// Bit Field Insert
    BFI {
        params: BfiParams,
    },
    /// Count Leading Zeros
    CLZ {
        params: Reg2RdRmParams,
    },
    /// Move Top
    MOVT {
        params: MovtParams,
    },
    // RBIT
    /// Byte-reverse word
    REV {
        params: Reg2RdRmParams,
        thumb32: bool,
    },

    /// Byte-reverse packed half-word
    REV16 {
        params: Reg2RdRmParams,
        thumb32: bool,
    },

    /// Byte-reverse signed half-word
    REVSH {
        params: Reg2RdRmParams,
        thumb32: bool,
    },

    //SBFX - signed bit field extract
    SBFX {
        params: BfxParams,
    },
    /// Unsigned bit field extract
    UBFX {
        params: BfxParams,
    },

    //
    // Subgroup: Miscellaneous data-processing instructions, Armv7-M DSP extension
    //
    /// Select bytes using GE flags
    SEL {
        params: Reg3NoSetFlagsParams,
    },
    //USAD8
    //USADA8

    // --------------------------------------------
    //
    // Group: Status register access instructions
    //
    // --------------------------------------------
    /// Move to Register from Special Register
    MRS {
        params: MrsParams,
    },
    /// Move to Special Register from ARM Register
    MSR_reg {
        params: MsrParams,
    },
    /// Change Processor State
    CPS {
        im: bool,
        #[cfg(not(feature = "armv6m"))]
        affect_pri: bool,
        #[cfg(not(feature = "armv6m"))]
        affect_fault: bool,
    },

    // --------------------------------------------
    //
    // Group: Load and Store instructions
    //
    // --------------------------------------------
    LDR_reg {
        params: Reg3FullParams,
        thumb32: bool,
    },

    LDRB_reg {
        params: Reg3FullParams,
        thumb32: bool,
    },

    LDRH_reg {
        params: Reg3FullParams,
        thumb32: bool,
    },

    LDRSB_reg {
        params: Reg3FullParams,
        thumb32: bool,
    },

    LDRSH_reg {
        params: Reg3FullParams,
        thumb32: bool,
    },

    LDR_imm {
        params: Reg2FullParams,
        thumb32: bool,
    },
    LDRB_imm {
        params: Reg2FullParams,
        thumb32: bool,
    },
    LDRH_imm {
        params: Reg2FullParams,
        thumb32: bool,
    },
    LDRSB_imm {
        params: Reg2FullParams,
        thumb32: bool,
    },
    LDRSH_imm {
        params: Reg2FullParams,
        thumb32: bool,
    },

    LDR_lit {
        params: RegImm32AddParams,
        thumb32: bool,
    },

    STRD_imm {
        params: Reg2DoubleParams,
    },

    STR_imm {
        params: Reg2FullParams,
        thumb32: bool,
    },
    STRB_reg {
        params: Reg3FullParams,
        thumb32: bool,
    },

    STRH_reg {
        params: Reg3FullParams,
        thumb32: bool,
    },
    STR_reg {
        params: Reg3FullParams,
        thumb32: bool,
    },

    STRH_imm {
        params: Reg2FullParams,
        thumb32: bool,
    },
    STRB_imm {
        params: Reg2FullParams,
        thumb32: bool,
    },

    LDREX {
        params: Reg2RtRnImm32Params,
    },

    LDREXB {
        params: Reg2RtRnParams,
    },

    LDREXH {
        params: Reg2RtRnParams,
    },

    LDRD_imm {
        params: Reg2DoubleParams,
    },

    STREX {
        params: Reg3RdRtRnImm32Params,
    },

    STREXB {
        params: Reg3RdRtRnParams,
    },

    STREXH {
        params: Reg3RdRtRnParams,
    },

    // --------------------------------------------
    //
    // Group: Load and Store Multiple instructions
    //
    // --------------------------------------------
    LDM {
        params: LoadAndStoreMultipleParams,
        thumb32: bool,
    },
    POP {
        registers: EnumSet<Reg>,
        thumb32: bool,
    },
    PUSH {
        registers: EnumSet<Reg>,
        thumb32: bool,
    },
    STM {
        params: LoadAndStoreMultipleParams,
        thumb32: bool,
    },
    STMDB {
        params: LoadAndStoreMultipleParams,
    },

    // --------------------------------------------
    //
    // Group: Miscellaneous instructions
    //
    // --------------------------------------------
    //CLREX
    //DBG
    /// Data Memory Barrier
    DMB,
    /// Data Synchronization Barrier
    DSB,
    /// Instruction Synchronization Barrier
    ISB,

    /// If-then
    IT {
        x: Option<ITCondition>,
        y: Option<ITCondition>,
        z: Option<ITCondition>,
        firstcond: Condition,
        mask: u8,
    },
    /// No operation
    NOP {
        thumb32: bool,
    },

    /// Preload data (immediate)
    PLD_imm {
        rn: Reg,
        imm32: u32,
        add: bool,
    },
    /// Preload data (literal)
    PLD_lit {
        imm32: u32,
        add: bool,
    },
    /// Preload data (register)
    PLD_reg {
        rn: Reg,
        rm: Reg,
        shift_t: SRType,
        shift_n: u8,
    },
    /// Send event
    SEV {
        thumb32: bool,
    },
    /// Wait for Event
    WFE {
        thumb32: bool,
    },
    /// Wait for interrupt
    WFI {
        thumb32: bool,
    },
    /// Yield
    YIELD {
        thumb32: bool,
    },
    // --------------------------------------------
    //
    // Group: Exception generating instructions
    //
    // --------------------------------------------
    /// supervisor call
    SVC {
        imm32: u32,
    },
    /// Breakpoint
    BKPT {
        imm32: u32,
    },
    // --------------------------------------------
    //
    // Group: Coprocessor instructions
    //
    // --------------------------------------------
    //CDP, CDP2
    MCR {
        rt: Reg,
        coproc: u8,
        opc1: u8,
        opc2: u8,
        crn: u8,
        crm: u8,
    },
    MCR2 {
        rt: Reg,
        coproc: u8,
        opc1: u8,
        opc2: u8,
        crn: u8,
        crm: u8,
    },
    //MCRR, MCRR2
    //MRC, MRC2
    //MRRC, MRRC2
    LDC_imm {
        coproc: u8,
        imm32: u32,
        crd: u8,
        rn: Reg,
    },

    LDC2_imm {
        coproc: u8,
        imm32: u32,
        crd: u8,
        rn: Reg,
    },

    //STC, STC2
    UDF {
        imm32: u32,
        opcode: ThumbCode,
        thumb32: bool,
    },
    // --------------------------------------------
    //
    // Group: Floating-point load and store instructions
    //
    // --------------------------------------------
    /// FP Load register
    #[cfg(feature = "has-fp")]
    VLDR {
        params: VLoadAndStoreParams,
    },
    /// FP Store register
    #[cfg(feature = "has-fp")]
    VSTR {
        params: VLoadAndStoreParams,
    },
    #[cfg(feature = "has-fp")]
    VLDM_T1 {
        params: VStoreMultipleParams64,
    },
    #[cfg(feature = "has-fp")]
    VLDM_T2 {
        params: VStoreMultipleParams32,
    },
    #[cfg(feature = "has-fp")]
    VPUSH {
        params: VPushPopParams,
    },
    #[cfg(feature = "has-fp")]
    VPOP {
        params: VPushPopParams,
    },
    #[cfg(feature = "has-fp")]
    VSTM_T1 {
        params: VStoreMultipleParams64,
    },
    #[cfg(feature = "has-fp")]
    VSTM_T2 {
        params: VStoreMultipleParams32,
    },

    // --------------------------------------------
    //
    // Group: Floating-point register transfer instructions
    //
    // --------------------------------------------
    #[cfg(feature = "has-fp")]
    VMOV_imm_32 {
        params: VMovImmParams32,
    },
    #[cfg(feature = "has-fp")]
    VMOV_imm_64 {
        params: VMovImmParams64,
    },
    #[cfg(feature = "has-fp")]
    VMOV_reg_f32 {
        params: VMovRegParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VMOV_reg_f64 {
        params: VMovRegParamsf64,
    },
    #[cfg(feature = "has-fp")]
    VMOV_cr_scalar {
        params: VMovCrScalarParams,
    },
    #[cfg(feature = "has-fp")]
    VMOV_scalar_cr {
        params: VMovCrScalarParams,
    },
    #[cfg(feature = "has-fp")]
    VMOV_cr_sp {
        params: VMovCrSpParams,
    },
    #[cfg(feature = "has-fp")]
    VMOV_cr2_sp2 {
        params: VMovCr2Sp2Params,
    },
    #[cfg(feature = "has-fp")]
    VMOV_cr2_dp {
        params: VMovCr2DpParams,
    },
    #[cfg(feature = "has-fp")]
    VMRS {
        rt: VMRSTarget,
    },

    // --------------------------------------------
    //
    // Group: Floating-point data-processing instructions
    //
    // --------------------------------------------
    #[cfg(feature = "has-fp")]
    VABS_f32 {
        params: VMovRegParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VABS_f64 {
        params: VMovRegParamsf64,
    },
    #[cfg(feature = "has-fp")]
    VNEG_f32 {
        params: VMovRegParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VNEG_f64 {
        params: VMovRegParamsf64,
    },
    #[cfg(feature = "has-fp")]
    VADD_f32 {
        params: VAddSubParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VADD_f64 {
        params: VAddSubParamsf64,
    },
    #[cfg(feature = "has-fp")]
    VFMA_f32 {
        params: VAddSubParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VFMA_f64 {
        params: VAddSubParamsf64,
    },
    #[cfg(feature = "has-fp")]
    VFMS_f32 {
        params: VAddSubParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VFMS_f64 {
        params: VAddSubParamsf64,
    },
    #[cfg(feature = "has-fp")]
    VFNMS_f32 {
        params: VAddSubParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VFNMS_f64 {
        params: VAddSubParamsf64,
    },
    #[cfg(feature = "has-fp")]
    VCMP_f32 {
        params: VCmpParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VCMP_f64 {
        params: VCmpParamsf64,
    },
    #[cfg(feature = "has-fp")]
    VCVT {
        params: VCVTParams,
    },
    #[cfg(feature = "has-fp")]
    VCVT_f64_f32 {
        params: VCVTParamsF64F32,
    },
    #[cfg(feature = "has-fp")]
    VCVT_f32_f64 {
        params: VCVTParamsF32F64,
    },
    #[cfg(feature = "has-fp")]
    VSEL_f32 {
        params: VSelParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VSEL_f64 {
        params: VSelParamsf64,
    },
    #[cfg(feature = "has-fp")]
    VDIV_f32 {
        params: VAddSubParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VDIV_f64 {
        params: VAddSubParamsf64,
    },
    //VFNMA
    //VMAXNM
    //VMLA
    //VMOV
    //VMOV
    #[cfg(feature = "has-fp")]
    VMUL_f32 {
        params: VAddSubParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VMUL_f64 {
        params: VAddSubParamsf64,
    },
    #[cfg(feature = "has-fp")]
    VNMUL_f32 {
        params: VAddSubParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VNMUL_f64 {
        params: VAddSubParamsf64,
    },
    //VNMLA
    //VRINTA
    #[cfg(feature = "has-fp")]
    VRINTZ_f32 {
        params: VMovRegParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VRINTZ_f64 {
        params: VMovRegParamsf64,
    },
    //VSEL
    #[cfg(feature = "has-fp")]
    VSQRT_f32 {
        params: VMovRegParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VSQRT_f64 {
        params: VMovRegParamsf64,
    },
    //VSUB
    #[cfg(feature = "has-fp")]
    VSUB_f32 {
        params: VAddSubParamsf32,
    },
    #[cfg(feature = "has-fp")]
    VSUB_f64 {
        params: VAddSubParamsf64,
    },
}

use std::fmt;

fn format_adressing_mode(
    name: &str,
    f: &mut fmt::Formatter,
    params: Reg2FullParams,
    thumb32: bool,
) -> fmt::Result {
    if params.index {
        if params.wback {
            // Pre-indexed
            write!(
                f,
                "{}{} {}, [{} , #{}{}]!",
                name,
                if thumb32 { ".W" } else { "" },
                params.rt,
                params.rn,
                if params.add { "+" } else { "-" },
                params.imm32
            )
        } else {
            // Offset
            write!(
                f,
                "{}{} {}, [{} {{, #{}{}}}]",
                name,
                if thumb32 { ".W" } else { "" },
                params.rt,
                params.rn,
                if params.add { "+" } else { "-" },
                params.imm32
            )
        }
    } else {
        // Post-indexed
        write!(
            f,
            "{}{} {}, [{}], #{}{}",
            name,
            if thumb32 { ".W" } else { "" },
            params.rt,
            params.rn,
            if params.add { "+" } else { "-" },
            params.imm32
        )
    }
}

#[allow(clippy::too_many_arguments, clippy::fn_params_excessive_bools)]
fn format_adressing_mode2(
    name: &str,
    f: &mut fmt::Formatter,
    params: Reg2DoubleParams,
    thumb32: bool,
) -> fmt::Result {
    if params.index {
        if params.wback {
            // Pre-indexed
            write!(
                f,
                "{}{} {}, {}, [{} , #{}{}]!",
                name,
                if thumb32 { ".W" } else { "" },
                params.rt,
                params.rt2,
                params.rn,
                if params.add { "+" } else { "-" },
                params.imm32
            )
        } else {
            // Offset
            write!(
                f,
                "{}{} {}, {}, [{} {{, #{}{}}}]",
                name,
                if thumb32 { ".W" } else { "" },
                params.rt,
                params.rt2,
                params.rn,
                if params.add { "+" } else { "-" },
                params.imm32
            )
        }
    } else {
        // Post-indexed
        write!(
            f,
            "{}{} {}, {},  [{}], #{}{}",
            name,
            if thumb32 { ".W" } else { "" },
            params.rt,
            params.rt2,
            params.rn,
            if params.add { "+" } else { "-" },
            params.imm32
        )
    }
}

fn setflags_to_str(setflags: SetFlags) -> &'static str {
    match setflags {
        SetFlags::True => "s",
        SetFlags::False | SetFlags::NotInITBlock => "",
    }
}

fn fmt_add_imm(params: Reg2ImmParams, thumb32: bool) -> String {
    if params.rn == params.rd {
        format!(
            "add{}{} {}, #{}",
            if thumb32 { ".W" } else { "" },
            setflags_to_str(params.setflags),
            params.rd,
            params.imm32
        )
    } else {
        format!(
            "add{}{} {}, {}, #{}",
            if thumb32 { ".W" } else { "" },
            setflags_to_str(params.setflags),
            params.rd,
            params.rn,
            params.imm32
        )
    }
}

#[cfg(feature = "has-fp")]
fn fmt_vcvt(params: VCVTParams) -> String {
    let fpscr_rounding = params.round_nearest;
    if params.to_integer {
        if params.dp_operation {
            format!(
                "vcvt{}.{}.f64 {}, {}",
                if fpscr_rounding { "r" } else { "" },
                if params.unsigned { "u32" } else { "s32" },
                params.d,
                params.m
            )
        } else {
            format!(
                "vcvt{}.{}.f32 {}, {}",
                if fpscr_rounding { "r" } else { "" },
                if params.unsigned { "u32" } else { "s32" },
                params.d,
                params.m
            )
        }
    } else if params.dp_operation {
        format!(
            "vcvt.f64.{} {}, {}",
            if params.unsigned { "u32" } else { "s32" },
            params.d,
            params.m
        )
    } else {
        format!(
            "vcvt.f32.{} {}, {}",
            if params.unsigned { "u32" } else { "s32" },
            params.d,
            params.m
        )
    }
}

#[allow(clippy::cognitive_complexity)]
#[allow(unused_variables)]
#[allow(clippy::too_many_lines)]
impl fmt::Display for Instruction {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        // TODO: shift_t, shift_n formattings missing.
        // TODO: some of the wide instruction formattings missing.
        match *self {
            Self::ADD_imm { params, thumb32 } => write!(f, "{}", fmt_add_imm(params, thumb32)),
            Self::ADC_imm { params } => write!(
                f,
                "adc{}.W {}, {}, #{}",
                setflags_to_str(params.setflags),
                params.rd,
                params.rn,
                params.imm32
            ),
            Self::ADD_reg { params, thumb32 } => write!(
                f,
                "add{}{} {}, {}, {}{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::ADD_sp_reg { params, thumb32 } => write!(
                f,
                "add{}{} {}, SP, {}{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::ADC_reg { params, thumb32 } => write!(
                f,
                "adc{}{} {}, {}, {}{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::ADR { params, thumb32 } => write!(
                f,
                "adr{} {}, pc, 0x#{:x}",
                if thumb32 { ".W" } else { "" },
                params.r,
                params.imm32
            ),
            Self::AND_reg { params, thumb32 } => write!(
                f,
                "and{}{} {}, {}, {}{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::AND_imm { params } => write!(
                f,
                "and{}.W {},{}, #{}",
                if params.setflags { "s" } else { "" },
                params.rd,
                params.rn,
                match params.imm32 {
                    Imm32Carry::NoCarry { imm32 } => imm32,
                    Imm32Carry::Carry { imm32_c0, imm32_c1 } => imm32_c0.0,
                }
            ),

            Self::ASR_imm { params, thumb32 } => write!(
                f,
                "asr{}{} {}, {}, #{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rm,
                params.shift_n
            ),
            Self::ROR_imm { params } => write!(
                f,
                "ror{}.w {}, {}, #{}",
                setflags_to_str(params.setflags),
                params.rd,
                params.rm,
                params.shift_n
            ),
            Self::ASR_reg { params, thumb32 } => write!(
                f,
                "asr{}{} {}, {}, {}",
                if thumb32 { ".W" } else { "" },
                setflags_to_str(params.setflags),
                params.rd,
                params.rn,
                params.rm
            ),
            Self::BIC_reg { params, thumb32 } => write!(
                f,
                "bic{}{} {}, {}, {}{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::BIC_imm { params } => write!(
                f,
                "bic{} {}, {}, #{}",
                if params.setflags { "s" } else { "" },
                params.rd,
                params.rn,
                match params.imm32 {
                    Imm32Carry::NoCarry { imm32 } => imm32,
                    Imm32Carry::Carry { imm32_c0, imm32_c1 } => imm32_c0.0,
                }
            ),
            Self::TEQ_imm { params } => write!(
                f,
                "teq.w {}, #{}",
                params.rn,
                match params.imm32 {
                    Imm32Carry::NoCarry { imm32 } => imm32,
                    Imm32Carry::Carry { imm32_c0, imm32_c1 } => imm32_c0.0,
                }
            ),
            Self::B_t13 { params, thumb32 } => write!(
                f,
                "b{}{} {}",
                params.cond,
                if thumb32 { ".W" } else { "" },
                params.imm32
            ),
            Self::B_t24 { imm32, thumb32 } => {
                write!(f, "b{} {imm32}", if thumb32 { ".W" } else { "" })
            }
            Self::BL { imm32 } => write!(f, "bl 0x#{imm32:x}"),
            Self::BX { rm } => write!(f, "bx {rm}"),
            Self::BLX { rm } => write!(f, "blx {rm}"),
            Self::BKPT { imm32 } => write!(f, "bkpt #{imm32}"),

            Self::BFI { params } => write!(
                f,
                "bfi {}, {}, #{}, #{}",
                params.rd, params.rn, params.lsbit, params.width
            ),

            Self::BFC { params } => write!(
                f,
                "bfc {}, #{}, #{}",
                params.rd,
                params.lsbit,
                params.msbit - params.lsbit + 1
            ),

            Self::CMN_reg { params, thumb32 } => write!(
                f,
                "cmn{} {}, {}{}",
                if thumb32 { ".W" } else { "" },
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::CMN_imm { params } => write!(f, "cmn.W {}, #{}", params.r, params.imm32),
            Self::CBZ { params } => write!(f, "cbz {}, #{}", params.rn, params.imm32,),
            Self::CBNZ { params } => write!(f, "cbnz {}, #{}", params.rn, params.imm32,),
            Self::CLZ { params } => write!(f, "clz {},{}", params.rd, params.rm),
            Self::CMP_imm { params, thumb32 } => write!(
                f,
                "cmp{} {}, #{}",
                if thumb32 { ".W" } else { "" },
                params.r,
                params.imm32
            ),
            Self::CMP_reg { params, thumb32 } => write!(
                f,
                "cmp{} {}, {}{}",
                if thumb32 { ".W" } else { "" },
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),

            #[cfg(feature = "armv6m")]
            Self::CPS { im } => write!(f, "cps{} i", if im { "ID" } else { "IE" }),
            #[cfg(not(feature = "armv6m"))]
            Self::CPS {
                im,
                affect_pri,
                affect_fault,
            } => write!(
                f,
                "cps{} {}{}",
                if im { "ID" } else { "IE" },
                if affect_pri { "i" } else { "" },
                if affect_fault { "f" } else { "" }
            ),
            Self::DMB => write!(f, "dmb"),
            Self::DSB => write!(f, "dsb"),
            Self::EOR_reg { params, thumb32 } => write!(
                f,
                "eor{}{} {}, {}, {}{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::ISB => write!(f, "isb"),
            Self::IT {
                ref x,
                ref y,
                ref z,
                ref firstcond,
                ref mask,
            } => {
                let x_str = match x {
                    Some(c) => format!("{c}"),
                    None => String::new(),
                };
                let y_str = match y {
                    Some(c) => format!("{c}"),
                    None => String::new(),
                };
                let z_str = match z {
                    Some(c) => format!("{c}"),
                    None => String::new(),
                };
                write!(f, "it{x_str}{y_str}{z_str} {firstcond}")
            }

            Self::LDM { params, thumb32 } => write!(
                f,
                "ldm{} {}, {{{:?}}}",
                if thumb32 { ".W" } else { "" },
                params.rn,
                params.registers
            ),
            Self::LDR_reg { params, thumb32 } => write!(
                f,
                "ldr{} {}, [{}, {}]",
                if thumb32 { ".W" } else { "" },
                params.rt,
                params.rn,
                params.rm
            ),
            Self::LDR_imm { params, thumb32 } => format_adressing_mode("ldr", f, params, thumb32),
            Self::LDR_lit { params, thumb32 } => {
                if params.imm32 == 0 {
                    write!(
                        f,
                        "ldr{} {}, [pc]",
                        if thumb32 { ".W" } else { "" },
                        params.rt
                    )
                } else {
                    write!(
                        f,
                        "ldr{} {}, [pc, #{}{}]",
                        if thumb32 { ".W" } else { "" },
                        params.rt,
                        if params.add { "+" } else { "-" },
                        params.imm32
                    )
                }
            }
            Self::LDREX { params } => {
                write!(f, "ldrex {}, {}, #{}", params.rt, params.rn, params.imm32)
            }
            Self::LDREXB { params } => write!(f, "ldrexb {}, {}", params.rt, params.rn),
            Self::LDREXH { params } => write!(f, "ldrexh {}, {}", params.rt, params.rn),

            Self::LDRB_imm { params, thumb32 } => format_adressing_mode("ldrb", f, params, thumb32),
            Self::LDRB_reg { params, thumb32 } => write!(
                f,
                "ldrb{} {}, [{}, {}]",
                if thumb32 { ".W" } else { "" },
                params.rt,
                params.rn,
                params.rm
            ),
            Self::LDRH_imm { params, thumb32 } => format_adressing_mode("ldrh", f, params, thumb32),
            Self::LDRH_reg { params, thumb32 } => write!(
                f,
                "ldrh{} {}, [{}, {}]",
                if thumb32 { ".W" } else { "" },
                params.rt,
                params.rn,
                params.rm
            ),
            Self::LDRSB_reg { params, thumb32 } => {
                write!(f, "ldrsb {}, [{}, {}]", params.rt, params.rn, params.rm)
            }
            Self::LDRSH_reg { params, thumb32 } => write!(
                f,
                "ldrsh{} {}, [{}, {}]",
                if thumb32 { ".W" } else { "" },
                params.rt,
                params.rn,
                params.rm
            ),
            Self::LSL_imm { params, thumb32 } => write!(
                f,
                "lsl{}{} {}, {}, #{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rm,
                params.shift_n
            ),
            Self::LSL_reg { params, thumb32 } => write!(
                f,
                "lsl{}{} {}, {}, {}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.rm
            ),
            Self::LSR_reg { params, thumb32 } => write!(
                f,
                "lsr{}{} {}, {}, {}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.rm
            ),
            Self::LSR_imm { params, thumb32 } => write!(
                f,
                "lsr{} {}, {}, #{}",
                setflags_to_str(params.setflags),
                params.rd,
                params.rm,
                params.shift_n
            ),
            Self::MSR_reg { params } => write!(f, "msr {}, {}", params.sysm, params.rn),
            Self::MRS { params } => write!(f, "mrs {}, {}", params.rd, params.sysm),
            Self::MUL { params, thumb32 } => write!(
                f,
                "mul{} {}, {}, {}",
                setflags_to_str(params.setflags),
                params.rd,
                params.rn,
                params.rm
            ),
            Self::SMUL { params } => write!(
                f,
                "smul{}{} {}, {}, {}",
                if params.n_high { "T" } else { "B" },
                if params.m_high { "T" } else { "B" },
                params.rd,
                params.rn,
                params.rm
            ),
            Self::SMLA { params } => write!(
                f,
                "smla{}{} {}, {}, {}, {}",
                if params.n_high { "T" } else { "B" },
                if params.m_high { "T" } else { "B" },
                params.rd,
                params.rn,
                params.rm,
                params.ra
            ),
            Self::MOV_reg { params, thumb32 } => write!(
                f,
                "mov{}{} {}, {}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rm
            ),
            Self::MOV_imm { params, thumb32 } => write!(
                f,
                "mov{}{} {}, #{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                match params.imm32 {
                    Imm32Carry::NoCarry { imm32 } => imm32,
                    Imm32Carry::Carry { imm32_c0, imm32_c1 } => imm32_c0.0,
                }
            ),
            Self::MOVT { params } => write!(f, "movt {}, #{}", params.rd, params.imm16),
            Self::LDRSH_imm { params, thumb32 } => {
                format_adressing_mode("ldrsh", f, params, thumb32)
            }

            Self::LDRSB_imm { params, thumb32 } => {
                format_adressing_mode("ldrsb", f, params, thumb32)
            }
            Self::MVN_reg { params, thumb32 } => write!(
                f,
                "mvn{}{} {}, {}, {}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::MVN_imm { params } => write!(
                f,
                "mvn{} {}, #{}",
                setflags_to_str(params.setflags),
                params.rd,
                match params.imm32 {
                    Imm32Carry::NoCarry { imm32 } => imm32,
                    Imm32Carry::Carry { imm32_c0, imm32_c1 } => imm32_c0.0,
                }
            ),
            Self::NOP { .. } => write!(f, "nop"),
            Self::ORR_reg { params, thumb32 } => write!(
                f,
                "orr{}{} {}, {}, {}{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::ORR_imm { params } => write!(
                f,
                "orr{} {}, {}, #{}",
                if params.setflags { "s" } else { "" },
                params.rd,
                params.rn,
                match params.imm32 {
                    Imm32Carry::NoCarry { imm32 } => imm32,
                    Imm32Carry::Carry { imm32_c0, imm32_c1 } => imm32_c0.0,
                }
            ),
            Self::ORN_reg { params } => write!(
                f,
                "orn{}.w {}, {}, {}{}",
                setflags_to_str(params.setflags),
                params.rd,
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::EOR_imm { params } => write!(
                f,
                "eor{} {}, {}, #{}",
                if params.setflags { "s" } else { "" },
                params.rd,
                params.rn,
                match params.imm32 {
                    Imm32Carry::NoCarry { imm32 } => imm32,
                    Imm32Carry::Carry { imm32_c0, imm32_c1 } => imm32_c0.0,
                }
            ),
            Self::POP { registers, thumb32 } => {
                write!(f, "pop{} {:?}", if thumb32 { ".W" } else { "" }, registers)
            }
            Self::PUSH { thumb32, registers } => {
                write!(f, "push{} {:?}", if thumb32 { ".W" } else { "" }, registers)
            }
            Self::PLD_imm { rn, imm32, add } => {
                write!(f, "pld [{}, {}{}]", rn, if add { "+" } else { "-" }, imm32)
            }
            Self::PLD_lit { imm32, add } => {
                write!(f, "pld [PC, {}{}]", if add { "+" } else { "-" }, imm32)
            }
            Self::PLD_reg {
                rn,
                rm,
                shift_t,
                shift_n,
            } => write!(
                f,
                "pld [{}, {}, {}]",
                rn,
                rm,
                if shift_n > 0 {
                    format!(", {shift_t:?} {shift_n}")
                } else {
                    String::new()
                }
            ),

            Self::REV { params, .. } => write!(f, "rev {}, {}", params.rd, params.rm),
            Self::REV16 { params, .. } => write!(f, "rev16 {}, {}", params.rd, params.rm),
            Self::REVSH { params, .. } => write!(f, "revsh {}, {}", params.rd, params.rm),
            Self::ROR_reg { params, .. } => write!(
                f,
                "ror{} {}, {}, #{}",
                setflags_to_str(params.setflags),
                params.rd,
                params.rn,
                params.rm
            ),
            Self::RSB_imm { params, thumb32 } => write!(
                f,
                "rsb{}{} {}, {}, #{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.imm32
            ),
            Self::RRX { params } => write!(
                f,
                "mov.w{} {}, {}, rrx",
                setflags_to_str(params.setflags),
                params.rd,
                params.rm,
            ),

            Self::SBC_imm { params } => write!(
                f,
                "sbc{}.W {}, {}, #{}",
                setflags_to_str(params.setflags),
                params.rd,
                params.rn,
                params.imm32
            ),
            Self::RSB_reg { params, thumb32 } => write!(
                f,
                "rsb{}{} {}, {}, {}{}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),

            Self::SEV { .. } => write!(f, "sev"),
            Self::SBC_reg { params, thumb32 } => write!(
                f,
                "sbc{}{} {}, {}, {}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.rm
            ),
            Self::STM { params, thumb32 } => write!(
                f,
                "stm{} {}{}, {{{:?}}}",
                if thumb32 { ".W" } else { "" },
                params.rn,
                if params.wback { "!" } else { "" },
                params.registers
            ),
            Self::STMDB { params } => write!(
                f,
                "stmdb {}{}, {{{:?}}}",
                params.rn,
                if params.wback { "!" } else { "" },
                params.registers
            ),
            Self::STR_imm { params, thumb32 } => format_adressing_mode("str", f, params, thumb32),
            Self::STREX { params } => write!(
                f,
                "strex {}, {}, {}, #{}",
                params.rd, params.rt, params.rn, params.imm32
            ),
            Self::STREXB { params } => {
                write!(f, "strexb {}, {}, {}", params.rd, params.rt, params.rn)
            }
            Self::STREXH { params } => {
                write!(f, "strexh {}, {}, {} ", params.rd, params.rt, params.rn)
            }

            Self::STRD_imm { params } => format_adressing_mode2("strd", f, params, true),
            Self::LDRD_imm { params } => format_adressing_mode2("ldrd", f, params, true),
            Self::STR_reg { params, thumb32 } => {
                write!(f, "str {}, [{}, {}]", params.rt, params.rn, params.rm)
            }
            Self::STRB_imm { params, thumb32 } => format_adressing_mode("strb", f, params, thumb32),
            Self::STRB_reg { params, thumb32 } => {
                write!(f, "strb {}, [{}, {}]", params.rt, params.rn, params.rm)
            }
            Self::STRH_imm { params, thumb32 } => format_adressing_mode("strh", f, params, thumb32),
            Self::STRH_reg { params, thumb32 } => {
                write!(f, "strh {}, [{}, {}]", params.rt, params.rn, params.rm)
            }
            Self::SUB_imm { params, thumb32 } => {
                if params.rd == params.rn {
                    write!(
                        f,
                        "sub{}{} {}, #{}",
                        setflags_to_str(params.setflags),
                        if thumb32 { ".W" } else { "" },
                        params.rd,
                        params.imm32
                    )
                } else {
                    write!(
                        f,
                        "sub{}{} {}, {}, #{}",
                        setflags_to_str(params.setflags),
                        if thumb32 { ".W" } else { "" },
                        params.rd,
                        params.rn,
                        params.imm32
                    )
                }
            }
            Self::SUB_reg { params, thumb32 } => write!(
                f,
                "sub{}{} {}, {}, {}",
                setflags_to_str(params.setflags),
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rn,
                params.rm
            ),
            Self::TEQ_reg { params } => write!(
                f,
                "teq.W {}, {}, {}",
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::SVC { imm32 } => write!(f, "svc #{imm32}"),
            Self::SXTH { params, thumb32 } => write!(
                f,
                "sxth{} {}, {}{}",
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rm,
                if params.rotation > 0 {
                    format!("{}", params.rotation)
                } else {
                    String::new()
                }
            ),

            Self::SXTB { params, thumb32 } => write!(
                f,
                "sxtb{} {}, {}{}",
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rm,
                if params.rotation > 0 {
                    format!("{}", params.rotation)
                } else {
                    String::new()
                }
            ),
            Self::TBB { params } => write!(f, "tbb [{}, {}]", params.rn, params.rm),
            Self::TBH { params } => write!(f, "tbh [{}, {}, lsl #1]", params.rn, params.rm),
            Self::TST_reg { params, thumb32 } => write!(
                f,
                "tst{} {}, {}{}",
                if thumb32 { ".W" } else { "" },
                params.rn,
                params.rm,
                if params.shift_n > 0 {
                    format!(", {:?} {}", params.shift_t, params.shift_n)
                } else {
                    String::new()
                }
            ),
            Self::TST_imm { params } => write!(
                f,
                "tst {}, #{}",
                params.rn,
                match params.imm32 {
                    Imm32Carry::NoCarry { imm32 } => imm32,
                    Imm32Carry::Carry { imm32_c0, imm32_c1 } => imm32_c0.0,
                }
            ),
            Self::UDF {
                imm32, ref opcode, ..
            } => write!(f, "udf {imm32} (opcode = {opcode})"),

            Self::UADD8 { params } => {
                write!(f, "uadd8 {}, {}, {}", params.rd, params.rn, params.rm)
            }
            Self::SEL { params } => write!(f, "sel {}, {}, {}", params.rd, params.rn, params.rm),
            // ARMv7-M
            Self::UDIV { params } => write!(f, "udiv {}, {}, {}", params.rd, params.rn, params.rm),
            Self::SDIV { params } => write!(f, "sdiv {}, {}, {}", params.rd, params.rn, params.rm),
            // ARMv7-M
            Self::UMLAL { params } => write!(
                f,
                "umlal {}, {}, {}, {}",
                params.rdlo, params.rdhi, params.rn, params.rm
            ),
            // ARMv7-M
            Self::UMULL { params } => write!(
                f,
                "umull {}, {}, {}, {}",
                params.rdlo, params.rdhi, params.rn, params.rm
            ),
            Self::SMULL { params } => write!(
                f,
                "smull {}, {}, {}, {}",
                params.rdlo, params.rdhi, params.rn, params.rm
            ),
            // ARMv7-M
            Self::MLA { params } => write!(
                f,
                "mla {}, {}, {}, {}",
                params.rd, params.rn, params.rm, params.ra
            ),
            // ARMv7-M
            Self::MLS { params } => write!(
                f,
                "mls {}, {}, {}, {}",
                params.rd, params.rn, params.rm, params.ra
            ),
            // ARMv7-M
            Self::SMLAL { params } => write!(
                f,
                "smlal {}, {}, {}, {}",
                params.rdlo, params.rdhi, params.rn, params.rm
            ),
            Self::UXTB { params, thumb32 } => write!(
                f,
                "uxtb{} {}, {}{}",
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rm,
                if params.rotation > 0 {
                    format!("{}", params.rotation)
                } else {
                    String::new()
                }
            ),
            Self::UXTAB { params } => write!(
                f,
                "uxtb.w {},{},{} {}",
                params.rd,
                params.rn,
                params.rm,
                if params.rotation > 0 {
                    format!("{}", params.rotation)
                } else {
                    String::new()
                }
            ),
            Self::UXTH { params, thumb32 } => write!(
                f,
                "uxth{} {}, {}{}",
                if thumb32 { ".W" } else { "" },
                params.rd,
                params.rm,
                if params.rotation > 0 {
                    format!("{}", params.rotation)
                } else {
                    String::new()
                }
            ),
            Self::UBFX { params } => write!(
                f,
                "ubfx {}, {}, #{}, #{}",
                params.rd,
                params.rn,
                params.lsb,
                params.widthminus1 + 1
            ),
            Self::SBFX { params } => write!(
                f,
                "sbfx {}, {}, #{}, #{}",
                params.rd,
                params.rn,
                params.lsb,
                params.widthminus1 + 1
            ),
            #[cfg(feature = "has-fp")]
            Self::VLDR { params } => write!(f, "vldr {}, {}", params.dd, params.rn),
            #[cfg(feature = "has-fp")]
            Self::VSTR { params } => write!(f, "vstr {}, {}", params.dd, params.rn),
            #[cfg(feature = "has-fp")]
            Self::VLDM_T1 { params } => write!(
                f,
                "vldm{}.64, {}{} {:?}",
                if params.mode == AddressingMode::IncrementAfter {
                    "ia"
                } else {
                    "db"
                },
                params.rn,
                if params.write_back { "!" } else { "" },
                params.list
            ),
            #[cfg(feature = "has-fp")]
            Self::VLDM_T2 { params } => write!(
                f,
                "vldm{}.32, {}{} {:?}",
                if params.mode == AddressingMode::IncrementAfter {
                    "ia"
                } else {
                    "db"
                },
                params.rn,
                if params.write_back { "!" } else { "" },
                params.list
            ),
            #[cfg(feature = "has-fp")]
            Self::VSTM_T1 { params } => write!(
                f,
                "vstm{}.64, {}{} {:?}",
                if params.mode == AddressingMode::IncrementAfter {
                    "ia"
                } else {
                    "db"
                },
                params.rn,
                if params.write_back { "!" } else { "" },
                params.list
            ),
            #[cfg(feature = "has-fp")]
            Self::VSTM_T2 { params } => write!(
                f,
                "vstm{}.32, {}{} {:?}",
                if params.mode == AddressingMode::IncrementAfter {
                    "ia"
                } else {
                    "db"
                },
                params.rn,
                if params.write_back { "!" } else { "" },
                params.list
            ),
            #[cfg(feature = "has-fp")]
            Self::VPUSH { params } => write!(
                f,
                "vpush {}",
                if params.single_regs {
                    format!("{:?}", params.single_precision_registers)
                } else {
                    format!("{:?}", params.double_precision_registers)
                }
            ),
            #[cfg(feature = "has-fp")]
            Self::VPOP { params } => write!(
                f,
                "vpop {}",
                if params.single_regs {
                    format!("{:?}", params.single_precision_registers)
                } else {
                    format!("{:?}", params.double_precision_registers)
                }
            ),

            #[cfg(feature = "has-fp")]
            Self::VMOV_imm_32 { params } => write!(f, "vmov.f32 {}, #{}", params.sd, params.imm32),
            #[cfg(feature = "has-fp")]
            Self::VMOV_imm_64 { params } => write!(f, "vmov.f64 {}, #{}", params.dd, params.imm64),
            #[cfg(feature = "has-fp")]
            Self::VMOV_reg_f32 { params } => write!(f, "vmov.f32 {}, {}", params.sd, params.sm),
            #[cfg(feature = "has-fp")]
            Self::VMOV_reg_f64 { params } => write!(f, "vmov.f64 {}, {}", params.dd, params.dm),
            #[cfg(feature = "has-fp")]
            Self::VMOV_cr_scalar { params } => {
                write!(f, "vmov {}[{}], {}", params.dd, params.x, params.rt)
            }
            #[cfg(feature = "has-fp")]
            Self::VMOV_scalar_cr { params } => {
                write!(f, "vmov {}, {}[{}]", params.rt, params.dd, params.x)
            }

            #[cfg(feature = "has-fp")]
            Self::VMOV_cr_sp { params } => {
                write!(
                    f,
                    "vmov {}",
                    if params.to_arm_register {
                        format!("{}, {}", params.rt, params.sn)
                    } else {
                        format!("{}, {}", params.sn, params.rt)
                    }
                )
            }
            #[cfg(feature = "has-fp")]
            Self::VMOV_cr2_sp2 { params } => {
                write!(
                    f,
                    "vmov {}",
                    if params.to_arm_registers {
                        format!(
                            "{}, {}, {}, {}",
                            params.rt, params.rt2, params.sm, params.sm1
                        )
                    } else {
                        format!(
                            "{}, {}, {}, {}",
                            params.sm, params.sm1, params.rt, params.rt2
                        )
                    }
                )
            }
            #[cfg(feature = "has-fp")]
            Self::VMOV_cr2_dp { params } => {
                write!(
                    f,
                    "vmov {}",
                    if params.to_arm_registers {
                        format!("{}, {}, {}", params.rt, params.rt2, params.dm)
                    } else {
                        format!("{}, {}, {}", params.dm, params.rt, params.rt2)
                    }
                )
            }
            #[cfg(feature = "has-fp")]
            Self::VABS_f32 { params } => write!(f, "vabs.f32 {}, {}", params.sd, params.sm),
            #[cfg(feature = "has-fp")]
            Self::VABS_f64 { params } => write!(f, "vabs.f64 {}, {}", params.dd, params.dm),
            #[cfg(feature = "has-fp")]
            Self::VNEG_f32 { params } => write!(f, "vneg.f32 {}, {}", params.sd, params.sm),
            #[cfg(feature = "has-fp")]
            Self::VNEG_f64 { params } => write!(f, "vneg.f64 {}, {}", params.dd, params.dm),
            #[cfg(feature = "has-fp")]
            Self::VMRS { rt } => write!(f, "vmrs {rt}, fpscr"),
            #[cfg(feature = "has-fp")]
            Self::VCMP_f32 { params } => write!(
                f,
                "vcmp.f32 {}, {}",
                params.sd,
                if params.with_zero {
                    "#0".to_string()
                } else {
                    format!("{}", params.sm)
                }
            ),
            #[cfg(feature = "has-fp")]
            Self::VCMP_f64 { params } => write!(
                f,
                "vcmp.f64 {}, {}",
                params.dd,
                if params.with_zero {
                    "#0".to_string()
                } else {
                    format!("{}", params.dm)
                }
            ),

            #[cfg(feature = "has-fp")]
            Self::VADD_f32 { params } => {
                write!(f, "vadd.f32 {}, {}, {}", params.sd, params.sn, params.sm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VADD_f64 { params } => {
                write!(f, "vadd.f64 {}, {}, {}", params.dd, params.dn, params.dm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VFMA_f32 { params } => {
                write!(f, "vfma.f32 {}, {}, {}", params.sd, params.sn, params.sm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VFMA_f64 { params } => {
                write!(f, "vfma.f64 {}, {}, {}", params.dd, params.dn, params.dm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VFMS_f32 { params } => {
                write!(f, "vfms.f32 {}, {}, {}", params.sd, params.sn, params.sm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VFMS_f64 { params } => {
                write!(f, "vfms.f64 {}, {}, {}", params.dd, params.dn, params.dm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VFNMS_f32 { params } => {
                write!(f, "vfnms.f32 {}, {}, {}", params.sd, params.sn, params.sm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VFNMS_f64 { params } => {
                write!(f, "vfnms.f64 {}, {}, {}", params.dd, params.dn, params.dm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VSQRT_f32 { params } => write!(f, "vsqrt.f32 {}, {}", params.sd, params.sm),
            #[cfg(feature = "has-fp")]
            Self::VSQRT_f64 { params } => write!(f, "vsqrt.f64 {}, {}", params.dd, params.dm),
            #[cfg(feature = "has-fp")]
            Self::VRINTZ_f32 { params } => write!(f, "vrintz.f32 {}, {}", params.sd, params.sm),
            #[cfg(feature = "has-fp")]
            Self::VRINTZ_f64 { params } => write!(f, "vrintz.f64 {}, {}", params.dd, params.dm),
            #[cfg(feature = "has-fp")]
            Self::VSUB_f32 { params } => {
                write!(f, "vsub.f32 {}, {}, {}", params.sd, params.sn, params.sm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VSUB_f64 { params } => {
                write!(f, "vsub.f64 {}, {}, {}", params.dd, params.dn, params.dm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VCVT { params } => write!(f, "{}", fmt_vcvt(params)),
            #[cfg(feature = "has-fp")]
            Self::VCVT_f64_f32 { params } => write!(f, "vcvt.f64.f32 {}, {}", params.dd, params.sm),
            #[cfg(feature = "has-fp")]
            Self::VCVT_f32_f64 { params } => write!(f, "vcvt.f32.f64 {}, {}", params.sd, params.dm),
            #[cfg(feature = "has-fp")]
            Self::VSEL_f32 { params } => {
                write!(
                    f,
                    "vsel{}.f32 {}, {}, {}",
                    params.cond, params.sd, params.sn, params.sm
                )
            }
            #[cfg(feature = "has-fp")]
            Self::VSEL_f64 { params } => {
                write!(
                    f,
                    "vsel{}.f64 {}, {}, {}",
                    params.cond, params.dd, params.dn, params.dm
                )
            }
            #[cfg(feature = "has-fp")]
            Self::VDIV_f32 { params } => {
                write!(f, "vdiv.f32 {}, {}, {}", params.sd, params.sn, params.sm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VDIV_f64 { params } => {
                write!(f, "vdiv.f64 {}, {}, {}", params.dd, params.dn, params.dm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VMUL_f32 { params } => {
                write!(f, "vmul.f32 {}, {}, {}", params.sd, params.sn, params.sm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VMUL_f64 { params } => {
                write!(f, "vmul.f64 {}, {}, {}", params.dd, params.dn, params.dm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VNMUL_f32 { params } => {
                write!(f, "vnmul.f32 {}, {}, {}", params.sd, params.sn, params.sm,)
            }
            #[cfg(feature = "has-fp")]
            Self::VNMUL_f64 { params } => {
                write!(f, "vnmul.f64 {}, {}, {}", params.dd, params.dn, params.dm,)
            }

            Self::WFE { .. } => write!(f, "wfe"),
            Self::WFI { .. } => write!(f, "wfi"),
            Self::YIELD { .. } => write!(f, "yield"),
            // ARMv7-M
            Self::MCR {
                ref rt,
                ref coproc,
                ref opc1,
                ref opc2,
                ref crn,
                ref crm,
            } => write!(f, "mcr"),

            // ARMv7-M
            Self::MCR2 {
                ref rt,
                ref coproc,
                ref opc1,
                ref opc2,
                ref crn,
                ref crm,
            } => write!(f, "mcr2"),

            // ARMv7-M
            Self::LDC_imm {
                ref coproc,
                ref imm32,
                ref crd,
                ref rn,
            } => write!(f, "ldc"),

            // ARMv7-M
            Self::LDC2_imm {
                ref coproc,
                ref imm32,
                ref crd,
                ref rn,
            } => write!(f, "ldc2"),
        }
    }
}

#[allow(unused_variables)]
impl fmt::Display for ITCondition {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Then => write!(f, "t"),
            Self::Else => write!(f, "e"),
        }
    }
}

#[allow(clippy::cognitive_complexity)]
#[allow(unused_variables)]
#[allow(clippy::too_many_lines)]
/// Get the size of an instruction in bytes
pub fn instruction_size(instruction: &Instruction) -> usize {
    match instruction {
        Instruction::ADC_imm { .. } => 4,
        Instruction::ADC_reg { params, thumb32 } => isize_t(*thumb32),
        Instruction::ADD_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::ADD_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::ADD_sp_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::ADR { thumb32, .. } => isize_t(*thumb32),
        Instruction::AND_imm { .. } => 4,
        Instruction::AND_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::ASR_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::ASR_reg { thumb32, .. } => isize_t(*thumb32),

        Instruction::B_t13 { thumb32, .. } => isize_t(*thumb32),
        Instruction::B_t24 { thumb32, .. } => isize_t(*thumb32),
        Instruction::BFI { .. } => 4,
        Instruction::BFC { .. } => 4,
        Instruction::BIC_imm { .. } => 4,
        Instruction::BIC_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::BKPT { .. } => 2,
        Instruction::BL { .. } => 4,
        Instruction::BLX { .. } => 2,
        Instruction::BX { .. } => 2,

        Instruction::CBZ { .. } => 2,
        Instruction::CBNZ { .. } => 2,
        //CDP
        //CLREX
        Instruction::CLZ { .. } => 4,
        Instruction::CMN_imm { .. } => 4,
        Instruction::CMN_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::CMP_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::CMP_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::CPS { .. } => 2,

        Instruction::DMB => 4,
        Instruction::DSB => 4,

        Instruction::EOR_imm { .. } => 4,
        Instruction::EOR_reg { thumb32, .. } => isize_t(*thumb32),

        Instruction::ISB => 4,
        Instruction::IT { .. } => 2,

        Instruction::LDC_imm { .. } => 4,
        Instruction::LDC2_imm { .. } => 4,
        Instruction::LDM { thumb32, .. } => isize_t(*thumb32),
        //LDMDB
        Instruction::LDR_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::LDR_lit { thumb32, .. } => isize_t(*thumb32),
        Instruction::LDR_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::LDRB_imm { thumb32, .. } => isize_t(*thumb32),
        //LDRB_lit
        Instruction::LDRB_reg { thumb32, .. } => isize_t(*thumb32),
        //LDRBT
        Instruction::LDRD_imm { .. } => 4,
        //LDRD_lit
        Instruction::LDREX { .. } => 4,
        Instruction::LDREXB { .. } => 4,
        Instruction::LDREXH { .. } => 4,
        Instruction::LDRH_imm { thumb32, .. } => isize_t(*thumb32),
        //LDRH_lit
        Instruction::LDRH_reg { thumb32, .. } => isize_t(*thumb32),
        //LDRHT
        Instruction::LDRSB_imm { thumb32, .. } => isize_t(*thumb32),
        //LDRSB_lit
        Instruction::LDRSB_reg { thumb32, .. } => isize_t(*thumb32),
        //LDRSBT
        Instruction::LDRSH_imm { thumb32, .. } => isize_t(*thumb32),
        //LDRSH_lit
        Instruction::LDRSH_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::LSL_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::LSL_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::LSR_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::LSR_reg { thumb32, .. } => isize_t(*thumb32),

        Instruction::MCR { .. } => 4,
        Instruction::MCR2 { .. } => 4,
        Instruction::MLA { .. } => 4,
        Instruction::MLS { .. } => 4,
        Instruction::MOV_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::MOV_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::MOVT { .. } => 4,
        //MRC, MRC2
        //MRRC, MRRC2
        Instruction::MRS { .. } => 4,
        Instruction::MSR_reg { .. } => 4,
        Instruction::MUL { thumb32, .. } => isize_t(*thumb32),
        Instruction::MVN_imm { .. } => 4,
        Instruction::MVN_reg { thumb32, .. } => isize_t(*thumb32),

        Instruction::NOP { thumb32, .. } => isize_t(*thumb32),

        //ORN_imm
        Instruction::ORN_reg { .. } => 4,
        Instruction::ORR_imm { .. } => 4,
        Instruction::ORR_reg { thumb32, .. } => isize_t(*thumb32),

        //PKHBT, PKHTB
        Instruction::PLD_imm { .. } => 4,
        Instruction::PLD_lit { .. } => 4,
        Instruction::PLD_reg { .. } => 4,
        //PLI_imm,
        //PLI_reg
        Instruction::POP { thumb32, .. } => isize_t(*thumb32),
        Instruction::PUSH { thumb32, .. } => isize_t(*thumb32),

        //QADD16
        //QADD8
        //QASX
        //QSAX
        //QADD
        //QSUB
        //QDADD
        //QDSUB
        //QSUB16
        //QSUB8

        //RBIT
        Instruction::REV { thumb32, .. } => isize_t(*thumb32),
        Instruction::REV16 { thumb32, .. } => isize_t(*thumb32),
        Instruction::REVSH { thumb32, .. } => isize_t(*thumb32),
        Instruction::ROR_imm { .. } => 4,
        Instruction::ROR_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::RRX { .. } => 4,
        Instruction::RSB_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::RSB_reg { thumb32, .. } => 4,
        //SADD16
        //SADD8
        //SASX
        Instruction::SBC_imm { .. } => 4,
        Instruction::SBC_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::SBFX { .. } => 4,
        Instruction::SDIV { .. } => 4,
        Instruction::SEL { .. } => 4,
        Instruction::SEV { thumb32, .. } => isize_t(*thumb32),
        //SHADD16
        //SHADD8
        //SHASX
        //SHSAX
        //SHSUB16
        //SHSUB8
        Instruction::SMLA { .. } => 4,
        //SMLAD
        Instruction::SMLAL { .. } => 4,
        //SMLAL second variant?
        //SMLALD
        //SMLAW
        //SMLSD
        //SMLSLD
        //SMMLA
        //SMMLS
        //SMMUL
        //SMUAD
        Instruction::SMUL { .. } => 4,
        Instruction::SMULL { .. } => 4,
        //SMULW
        //SMUSD
        //SSAT
        //SSAT16
        //SSAX
        //SSUB16
        //SSUB8
        //STC, STC2
        Instruction::STM { thumb32, .. } => isize_t(*thumb32),
        Instruction::STMDB { .. } => 4,
        Instruction::STR_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::STR_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::STRB_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::STRB_reg { thumb32, .. } => isize_t(*thumb32),
        //STRBT
        Instruction::STRD_imm { .. } => 4,
        Instruction::STREX { .. } => 4,
        Instruction::STREXB { .. } => 4,
        Instruction::STREXH { .. } => 4,
        Instruction::STRH_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::STRH_reg { thumb32, .. } => isize_t(*thumb32),
        //STRHT
        //STRT
        Instruction::SUB_imm { thumb32, .. } => isize_t(*thumb32),
        Instruction::SUB_reg { thumb32, .. } => isize_t(*thumb32),
        Instruction::SVC { .. } => 2,
        //SXTAB
        //SXTAB16
        //SXTAH
        Instruction::SXTB { thumb32, .. } => isize_t(*thumb32),
        //SXTB16
        Instruction::SXTH { thumb32, .. } => isize_t(*thumb32),

        Instruction::TBB { .. } => 4,
        Instruction::TBH { .. } => 4,
        Instruction::TEQ_imm { .. } => 4,
        Instruction::TEQ_reg { .. } => 4,
        Instruction::TST_imm { .. } => 4,
        Instruction::TST_reg { thumb32, .. } => isize_t(*thumb32),

        Instruction::UADD8 { .. } => 4,
        //UADD16
        //UASX
        Instruction::UBFX { .. } => 4,
        Instruction::UDF { thumb32, .. } => isize_t(*thumb32),
        Instruction::UDIV { .. } => 4,
        //UHADD16
        //UHADD8
        //UHASX
        //UHSAX
        //UHSUB16
        //UHSUB8
        //UMAAL
        Instruction::UMLAL { .. } => 4,
        Instruction::UMULL { .. } => 4,
        //UQADD16
        //UQADD8
        //UQASX
        //UQSAX
        //UQSUB16
        //UQSUB8
        //USAD8
        //USADA8
        //USAT
        //USAT16
        //USAX
        //USUB16
        //USUB8
        Instruction::UXTAB { .. } => 4,
        //UXTAB16
        //UXTAH
        Instruction::UXTB { thumb32, .. } => isize_t(*thumb32),
        Instruction::UXTH { thumb32, .. } => isize_t(*thumb32),

        #[cfg(feature = "has-fp")]
        Instruction::VABS_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VABS_f64 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VNEG_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VNEG_f64 { .. } => 4,

        #[cfg(feature = "has-fp")]
        Instruction::VADD_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VADD_f64 { .. } => 4,

        #[cfg(feature = "has-fp")]
        Instruction::VCMP_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VCMP_f64 { .. } => 4,
        //VCVTX
        #[cfg(feature = "has-fp")]
        Instruction::VCVT { .. } => 4,
        //VCVTB
        //VCVTT
        #[cfg(feature = "has-fp")]
        Instruction::VDIV_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VDIV_f64 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VFMA_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VFMA_f64 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VFMS_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VFMS_f64 { .. } => 4,
        //VFNMA
        #[cfg(feature = "has-fp")]
        Instruction::VFNMS_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VFNMS_f64 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VLDM_T1 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VLDM_T2 { .. } => 4,
        //VMAXNM
        //VMINNM
        //VMLA
        //VMLS
        #[cfg(feature = "has-fp")]
        Instruction::VMOV_imm_32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VMOV_imm_64 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VMOV_reg_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VMOV_reg_f64 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VMOV_cr_scalar { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VMOV_scalar_cr { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VMOV_cr_sp { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VMOV_cr2_sp2 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VMOV_cr2_dp { .. } => 4,

        #[cfg(feature = "has-fp")]
        Instruction::VMRS { .. } => 4,
        //VMSR
        #[cfg(feature = "has-fp")]
        Instruction::VMUL_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VMUL_f64 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VNMUL_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VNMUL_f64 { .. } => 4,
        //VNMLA,VNMLS
        #[cfg(feature = "has-fp")]
        Instruction::VPUSH { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VPOP { .. } => 4,
        //VRINTA, VRINTN, VRINTP, VRiNTM
        //VRINTX,
        #[cfg(feature = "has-fp")]
        Instruction::VRINTZ_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VRINTZ_f64 { .. } => 4,
        //VRINTR
        #[cfg(feature = "has-fp")]
        Instruction::VSQRT_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VSQRT_f64 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VCVT_f64_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VCVT_f32_f64 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VSEL_f32 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VSEL_f64 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VSTM_T1 { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VSTM_T2 { .. } => 4,
        //VSTR
        #[cfg(feature = "has-fp")]
        Instruction::VSUB_f32 { params } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VSUB_f64 { params } => 4,
        Instruction::WFE { thumb32, .. } => isize_t(*thumb32),
        Instruction::WFI { thumb32, .. } => isize_t(*thumb32),
        Instruction::YIELD { thumb32, .. } => isize_t(*thumb32),
        #[cfg(feature = "has-fp")]
        Instruction::VLDR { .. } => 4,
        #[cfg(feature = "has-fp")]
        Instruction::VSTR { .. } => 4,
    }
}

#[inline(always)]
fn isize_t(thumb32: bool) -> usize {
    if thumb32 { 4 } else { 2 }
}
