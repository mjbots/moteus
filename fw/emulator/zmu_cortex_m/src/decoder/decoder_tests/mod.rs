use crate::core::instruction::{
    BfcParams, BfiParams, BfxParams, CondBranchParams, Imm32Carry, MovtParams, ParamsRegImm32,
    Reg2DoubleParams, Reg2FullParams, Reg2ImmCarryParams, Reg2ImmParams, Reg2Params,
    Reg2RdRmParams, Reg2RnRmParams, Reg2RtRnImm32Params, Reg2ShiftNParams,
    Reg2ShiftNoSetFlagsParams, Reg2ShiftParams, Reg2UsizeParams, Reg3FullParams,
    Reg3NoSetFlagsParams, Reg3Params, Reg3RdRtRnImm32Params, Reg3RdRtRnParams, Reg3ShiftParams,
    Reg3UsizeParams, Reg4NoSetFlagsParams, Reg643232Params, RegImm32AddParams,
    RegImmCarryNoSetFlagsParams, RegImmCarryParams, RegImmParams, SRType, SetFlags,
};
#[cfg(feature = "has-dsp-ext")]
use crate::core::instruction::{Reg3HighParams, Reg4HighParams};

#[cfg(feature = "has-fp")]
use crate::core::instruction::{
    VAddSubParamsf32, VAddSubParamsf64, VCVTParams, VCVTParamsF32F64, VCVTParamsF64F32,
    VCmpParamsf32, VLoadAndStoreParams, VMRSTarget, VMovCr2DpParams, VMovCrSpParams,
    VMovImmParams32, VMovImmParams64, VMovRegParamsf32, VMovRegParamsf64, VSelParamsf32,
};
#[cfg(feature = "has-fp")]
use crate::core::register::{DoubleReg, ExtensionReg, SingleReg};

use super::*;

use crate::core::register::Reg;

mod branch_control;
mod data_proc;
#[cfg(feature = "has-fp")]
mod floating_point;
mod fundamentals;
mod load_store;
mod multiply_divide;
mod saturation_pack_misc;
mod system_barrier;
