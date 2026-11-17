//!
//! Thumb-2 instruction set decoder
//!
//!

use crate::core::bits::Bits;
use crate::core::instruction::Instruction;

//#[cfg(test)]
//use crate::core::register::SpecialReg;

#[cfg(test)]
use crate::core::condition::Condition;
#[cfg(test)]
use crate::core::instruction::ITCondition;
mod bfc;
mod bfi;
mod cbz;
mod clrex;
mod dbg;
mod sbfx;
mod ssat;
mod ubfx;
mod usat;
mod wfe;
mod wfi;
mod yield_;

mod adc;
mod add;
mod adr;
mod and;
mod asr;

mod b;
mod bic;
mod bkpt;
mod bl;
mod blx;
mod bx;

mod clz;
mod cmn;
mod cmp;
mod cpd;
mod cps;

mod dmb;
mod dsb;

mod eor;

mod isb;
mod it;

mod ldc;
mod ldm;
mod ldr;
mod ldrb;
mod ldrh;
mod ldrsb;
mod ldrsh;
mod lsl;
mod lsr;

mod mcr;
mod mla;
mod mls;
mod mov;
mod mrs;
mod msr;
mod mul;
mod mvn;

mod nop;
mod orn;
mod orr;

mod pld;
mod pli;
mod pop;
mod push;

mod rbit;
mod rev;
mod ror;
mod rrx;
mod rsb;

mod sbc;
mod sdiv;
#[cfg(feature = "has-dsp-ext")]
mod sel;
mod sev;
#[cfg(feature = "has-dsp-ext")]
mod smla;
mod smlal;
#[cfg(feature = "has-dsp-ext")]
mod smul;
mod smull;
mod stc;
mod stm;
mod str;
mod strex;
mod sub;
mod sxt;

mod tbb;
mod tbh;
mod teq;
mod tst;

mod movt;
#[cfg(feature = "has-dsp-ext")]
mod uadd8;
mod udiv;
mod umlal;
mod umull;
mod uxt;
mod uxtab;

#[cfg(feature = "has-fp")]
mod vabs;
#[cfg(feature = "has-fp")]
mod vadd_vsub;
#[cfg(feature = "has-fp")]
mod vcmp;
#[cfg(feature = "has-fp")]
mod vcvt;
#[cfg(feature = "has-fp")]
mod vfma;
#[cfg(feature = "has-fp")]
mod vldm;
#[cfg(feature = "has-fp")]
mod vldr;
#[cfg(feature = "has-fp")]
mod vmov;
#[cfg(feature = "has-fp")]
mod vmrs;
#[cfg(feature = "has-fp")]
mod vmul_vdiv;
#[cfg(feature = "has-fp")]
mod vneg;
#[cfg(feature = "has-fp")]
mod vpop;
#[cfg(feature = "has-fp")]
mod vpush;
#[cfg(feature = "has-fp")]
mod vrint;
#[cfg(feature = "has-fp")]
mod vsel;
#[cfg(feature = "has-fp")]
mod vsqrt;
#[cfg(feature = "has-fp")]
mod vstm;
#[cfg(feature = "has-fp")]
mod vstr;

use {
    crate::decoder::str::{
        decode_STR_imm_t1, decode_STR_imm_t2, decode_STR_imm_t3, decode_STR_imm_t4,
        decode_STR_reg_t1, decode_STR_reg_t2, decode_STRB_imm_t1, decode_STRB_imm_t2,
        decode_STRB_imm_t3, decode_STRB_reg_t1, decode_STRB_reg_t2, decode_STRD_imm_t1,
        decode_STRH_imm_t1, decode_STRH_imm_t2, decode_STRH_imm_t3, decode_STRH_reg_t1,
        decode_STRH_reg_t2,
    },
    adc::{decode_ADC_imm_t1, decode_ADC_reg_t1, decode_ADC_reg_t2},
    add::{
        decode_ADD_SP_imm_t1, decode_ADD_SP_imm_t2, decode_ADD_imm_t1, decode_ADD_imm_t2,
        decode_ADD_imm_t3, decode_ADD_imm_t4, decode_ADD_reg_sp_t1, decode_ADD_reg_sp_t2,
        decode_ADD_reg_t1, decode_ADD_reg_t2, decode_ADD_reg_t3,
    },
    adr::{decode_ADR_t1, decode_ADR_t2, decode_ADR_t3},
    and::{decode_AND_imm_t1, decode_AND_reg_t1, decode_AND_reg_t2},
    asr::{decode_ASR_imm_t1, decode_ASR_imm_t2, decode_ASR_reg_t1, decode_ASR_reg_t2},
    b::{decode_B_t1_SVC_t1, decode_B_t2, decode_B_t3, decode_B_t4},
    bfc::decode_BFC_t1,
    bfi::decode_BFI_t1,
    bic::{decode_BIC_imm_t1, decode_BIC_reg_t1, decode_BIC_reg_t2},
    bkpt::decode_BKPT_t1,
    bl::decode_BL_t1,
    blx::decode_BLX_t1,
    bx::decode_BX_t1,
    cbz::decode_CBZ_t1,
    clrex::decode_CLREX_t1,
    clz::decode_CLZ_t1,
    cmn::{decode_CMN_imm_t1, decode_CMN_reg_t1, decode_CMN_reg_t2},
    cmp::{
        decode_CMP_imm_t1, decode_CMP_imm_t2, decode_CMP_reg_t1, decode_CMP_reg_t2,
        decode_CMP_reg_t3,
    },
    cpd::{decode_CDP_t1, decode_CDP2_t2},
    cps::decode_CPS_t1,
    dbg::decode_DBG_t1,
    dmb::decode_DMB_t1,
    dsb::decode_DSB_t1,
    eor::{decode_EOR_imm_t1, decode_EOR_reg_t1, decode_EOR_reg_t2},
    isb::decode_ISB_t1,
    it::decode_IT_t1,
    ldc::{decode_LDC_imm_t1, decode_LDC_lit_t1, decode_LDC2_imm_t2, decode_LDC2_lit_t2},
    ldm::{decode_LDM_t1, decode_LDM_t2, decode_LDMDB_t1},
    ldr::{
        decode_LDR_imm_t1, decode_LDR_imm_t2, decode_LDR_imm_t3, decode_LDR_imm_t4,
        decode_LDR_lit_t1, decode_LDR_lit_t2, decode_LDR_reg_t1, decode_LDR_reg_t2,
        decode_LDRBT_t1, decode_LDRD_imm_t1, decode_LDRD_lit_t1, decode_LDREX_t1, decode_LDREXB_t1,
        decode_LDREXH_t1, decode_LDRHT_t1, decode_LDRSBT_t1, decode_LDRSHT, decode_LDRT_t1,
    },
    ldrb::{
        decode_LDRB_imm_t1, decode_LDRB_imm_t2, decode_LDRB_imm_t3, decode_LDRB_lit_t1,
        decode_LDRB_reg_t1, decode_LDRB_reg_t2,
    },
    ldrh::{
        decode_LDRH_imm_t1, decode_LDRH_imm_t2, decode_LDRH_imm_t3, decode_LDRH_lit_t1,
        decode_LDRH_reg_t1, decode_LDRH_reg_t2,
    },
    ldrsb::{
        decode_LDRSB_imm_t1, decode_LDRSB_imm_t2, decode_LDRSB_lit_t1, decode_LDRSB_reg_t1,
        decode_LDRSB_reg_t2,
    },
    ldrsh::{
        decode_LDRSH_imm_t1, decode_LDRSH_imm_t2, decode_LDRSH_lit_t1, decode_LDRSH_reg_t1,
        decode_LDRSH_reg_t2,
    },
    lsl::{decode_LSL_imm_t2, decode_LSL_reg_t1, decode_LSL_reg_t2},
    lsr::{decode_LSR_imm_t1, decode_LSR_imm_t2, decode_LSR_reg_t1, decode_LSR_reg_t2},
    mcr::{
        decode_MCR_t1, decode_MCR2_t2, decode_MCRR_t1, decode_MCRR2_t2, decode_MRC_t1,
        decode_MRC2_t2,
    },
    mla::decode_MLA_t1,
    mls::decode_MLS_t1,
    mov::{
        decode_MOV_imm_t1, decode_MOV_imm_t2, decode_MOV_imm_t3, decode_MOV_reg_t1,
        decode_MOV_reg_t2_LSL_imm_t1, decode_MOV_reg_t3,
    },
    movt::decode_MOVT_t1,
    mrs::decode_MRS_t1,
    msr::decode_MSR_reg_t1,
    mul::{decode_MUL_t1, decode_MUL_t2},
    mvn::{decode_MVN_imm_t1, decode_MVN_reg_t1, decode_MVN_reg_t2},
    nop::{decode_NOP_t1, decode_NOP_t2},
    orn::{decode_ORN_imm_t1, decode_ORN_reg_t1},
    orr::{decode_ORR_imm_t1, decode_ORR_reg_t1, decode_ORR_reg_t2},
    pld::{decode_PLD_imm_t1, decode_PLD_imm_t2, decode_PLD_lit_t1, decode_PLD_reg_t1},
    pli::{decode_PLI_lit_imm_t1, decode_PLI_lit_imm_t2, decode_PLI_lit_imm_t3, decode_PLI_reg_t1},
    pop::{decode_POP_reg_t1, decode_POP_t2, decode_POP_t3},
    push::{decode_PUSH_t1, decode_PUSH_t2, decode_PUSH_t3},
    rbit::decode_RBIT_t1,
    rev::{
        decode_REV_t1, decode_REV_t2, decode_REV16_t1, decode_REV16_t2, decode_REVSH_t1,
        decode_REVSH_t2,
    },
    ror::{decode_ROR_imm_t1, decode_ROR_reg_t1, decode_ROR_reg_t2},
    rrx::decode_RRX_t1,
    rsb::{decode_RSB_imm_t1, decode_RSB_imm_t2, decode_RSB_reg_t1},
    sbc::{decode_SBC_imm_t1, decode_SBC_reg_t1, decode_SBC_reg_t2},
    sbfx::decode_SBFX_t1,
    sdiv::decode_SDIV_t1,
    sev::{decode_SEV_t1, decode_SEV_t2},
    smlal::decode_SMLAL_t1,
    smull::decode_SMULL_t1,
    ssat::decode_SSAT_t1,
    stc::{decode_STC_t1, decode_STC2_t2},
    stm::{decode_STM_t1, decode_STM_t2, decode_STMDB_t1},
    strex::{decode_STREX_t1, decode_STREXB_t1, decode_STREXH_t1},
    sub::{
        decode_SUB_SP_imm_t1, decode_SUB_SP_imm_t2, decode_SUB_SP_imm_t3, decode_SUB_imm_t1,
        decode_SUB_imm_t2, decode_SUB_imm_t3, decode_SUB_imm_t4, decode_SUB_reg_t1,
        decode_SUB_reg_t2,
    },
    sxt::{decode_SXTB_t1, decode_SXTB_t2, decode_SXTH_t1, decode_SXTH_t2},
    tbb::decode_TBB_t1,
    tbh::decode_TBH_t1,
    teq::{decode_TEQ_imm_t1, decode_TEQ_reg_t1},
    tst::{decode_TST_imm_t1, decode_TST_reg_t1, decode_TST_reg_t2},
    ubfx::decode_UBFX_t1,
    udiv::decode_UDIV_t1,
    umlal::decode_UMLAL_t1,
    umull::decode_UMULL_t1,
    usat::decode_USAT_t1,
    uxt::{decode_UXTB_t1, decode_UXTB_t2, decode_UXTH_t1, decode_UXTH_t2},
    uxtab::decode_UXTAB_t1,
    wfe::{decode_WFE_t1, decode_WFE_t2},
    wfi::{decode_WFI_t1, decode_WFI_t2},
    yield_::{decode_YIELD_t1, decode_YIELD_t2},
};

#[cfg(feature = "has-dsp-ext")]
use self::{
    sel::decode_SEL_t1, smla::decode_SMLA_t1, smul::decode_SMUL_t1, uadd8::decode_UADD8_t1,
};

use crate::Processor;
use crate::core::thumb::ThumbCode;

#[cfg(feature = "has-fp")]
use {
    vabs::decode_VABS_t1,
    vadd_vsub::{decode_VADD_t1, decode_VSUB_t1},
    vcmp::{decode_VCMP_t1, decode_VCMP_t2},
    vcvt::{decode_VCVT_ds_t1, decode_VCVT_t1},
    vfma::{decode_VFMA_t1, decode_VFMS_t1, decode_VFNMS_t1},
    vldm::{decode_VLDM_t1, decode_VLDM_t2},
    vldr::{decode_VLDR_t1, decode_VLDR_t2},
    vmov::decode_VMOV_cr_scalar,
    vmov::decode_VMOV_cr_sp,
    vmov::decode_VMOV_cr2_dp,
    vmov::decode_VMOV_cr2_sp2,
    vmov::decode_VMOV_imm,
    vmov::decode_VMOV_reg,
    vmov::decode_VMOV_scalar_cr,
    vmrs::decode_VMRS,
    vmul_vdiv::{decode_VDIV_t1, decode_VMUL_t1, decode_VNMUL_t1},
    vneg::decode_VNEG_t1,
    vpop::decode_VPOP_t1,
    vpop::decode_VPOP_t2,
    vpush::decode_VPUSH_t1,
    vpush::decode_VPUSH_t2,
    vrint::decode_VRINTZ_t1,
    vsel::decode_VSEL_t1,
    vsqrt::decode_VSQRT_t1,
    vstm::{decode_VSTM_t1, decode_VSTM_t2},
    vstr::{decode_VSTR_t1, decode_VSTR_t2},
};

///
/// Generic Thumbcode to instruction decoder trait
///
pub trait Decoder {
    ///
    /// Resolve the instruction from a thumb code
    ///
    fn decode(&self, code: ThumbCode) -> Instruction;
}

impl Decoder for Processor {
    fn decode(&self, code: ThumbCode) -> Instruction {
        match code {
            ThumbCode::Thumb32 { opcode } => decode_32(opcode),
            ThumbCode::Thumb16 { opcode } => decode_16(opcode),
            ThumbCode::Undefined => Instruction::UDF {
                imm32: 0,
                opcode: code,
                thumb32: false,
            },
        }
    }
}

/// determine if 16 bit word is start of 32 thumb value
pub fn is_thumb32(word: u16) -> bool {
    matches!(word.get_bits(11..16), 0b11101..=0b11111)
}

#[allow(non_snake_case)]
fn decode_undefined(opcode: u16) -> Instruction {
    Instruction::UDF {
        imm32: 0,
        opcode: opcode.into(),
        thumb32: false,
    }
}

#[allow(non_snake_case)]
fn decode_UDF_t2(opcode: u32) -> Instruction {
    Instruction::UDF {
        imm32: 0,
        opcode: opcode.into(),
        thumb32: true,
    }
}

include!(concat!(env!("OUT_DIR"), "/decode_16.rs"));

include!(concat!(env!("OUT_DIR"), "/decode_32.rs"));

#[cfg(test)]
mod decoder_tests;
