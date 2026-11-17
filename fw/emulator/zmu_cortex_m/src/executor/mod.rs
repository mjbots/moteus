//!
//! Functionality for running instructions on a Processor.
//!

use crate::core::bits::Bits;
use crate::core::condition::Condition;
use crate::core::exception::{Exception, ExceptionHandling};
use crate::core::fault::{Fault, FaultStatusContext, FaultTrapReason};
use crate::core::fetch::Fetch;
use crate::core::instruction::{Imm32Carry, Instruction, SetFlags, instruction_size};

use crate::core::operation::condition_test;
use crate::core::register::{Apsr, BaseReg, Ipsr};
use crate::decoder::Decoder;
use crate::memory::map::MapMemory;
use crate::peripheral::{dwt::Dwt, systick::SysTick};

use crate::{CachedInstruction, Processor};

mod branch;
mod coproc;
mod divide;
mod exception;
mod load_and_store;
mod load_and_store_multiple;
mod misc;
mod misc_data_processing;
mod multiply;
mod packing;
mod parallel_add;
mod shift;
mod signed_multiply;
mod status_register;
mod std_data_processing;

#[cfg(feature = "has-fp")]
mod fp_data_processing;
#[cfg(feature = "has-fp")]
mod fp_generic;
#[cfg(feature = "has-fp")]
mod fp_load_and_store;
#[cfg(feature = "has-fp")]
mod fp_register_transfer;

use branch::IsaBranch;
use coproc::IsaCoprocessor;
use divide::IsaDivide;
use exception::IsaException;
use load_and_store::IsaLoadAndStore;
use load_and_store_multiple::IsaLoadAndStoreMultiple;
use misc::IsaMisc;
use misc_data_processing::IsaMiscDataProcessing;
use multiply::IsaMultiply;
use packing::IsaPacking;
#[cfg(feature = "has-dsp-ext")]
use parallel_add::IsaParallelAddSub;
use shift::IsaShift;
use signed_multiply::IsaSignedMultiply;
use status_register::IsaStatusRegister;
use std_data_processing::IsaStandardDataProcessing;

#[cfg(feature = "has-fp")]
use fp_data_processing::IsaFloatingPointDataProcessing;
#[cfg(feature = "has-fp")]
pub(crate) use fp_generic::FloatingPointChecks;
#[cfg(feature = "has-fp")]
use fp_load_and_store::IsaFloatingPointLoadAndStore;
#[cfg(feature = "has-fp")]
use fp_register_transfer::IsaFloatingPointRegisterTransfer;

///
/// Stepping processor with instructions
///
pub trait Executor {
    ///
    /// Run processor forward. Simulates core + peripherals.
    ///
    fn step(&mut self);

    ///
    /// Run processor forward with core sleeping (peripherals only)
    ///
    fn step_sleep(&mut self);

    ///
    /// Execute given instruction. Returns number of clock cycles burn.
    ///
    fn execute(&mut self, instruction: &Instruction, instruction_size: usize) -> u32;
}

type ExecuteResult = Result<ExecuteSuccess, Fault>;
#[derive(PartialEq, Debug, Copy, Clone)]
/// Succesfull execution  an instruction
pub enum ExecuteSuccess {
    /// The instruction was taken normally
    Taken {
        /// Number of clock cycles taken for the operation
        cycles: u32,
    },
    /// The instruction was not taken as the condition did not pass
    NotTaken,
    /// The execution branched to a new address, pc was set accordingly
    Branched {
        /// Number of clock cycles taken for the operation
        cycles: u32,
    },
}

trait ExecutorHelper {
    fn condition_passed(&self) -> bool;
    fn condition_passed_b(&self, cond: Condition) -> bool;
    fn integer_zero_divide_trapping_enabled(&self) -> bool;
    fn set_itstate(&mut self, state: u8);
    fn it_advance(&mut self);
    fn in_it_block(&self) -> bool;
    #[allow(dead_code)]
    fn last_in_it_block(&self) -> bool;
    fn execute_internal(&mut self, instruction: &Instruction) -> ExecuteResult;
    fn update_flags_check_it_block(
        &mut self,
        setflags: SetFlags,
        result: u32,
        carry: bool,
        overflow: bool,
    );
}

#[inline(always)]
fn resolve_addressing(rn: u32, imm32: u32, add: bool, index: bool) -> (u32, u32) {
    let offset_address = if add {
        rn.wrapping_add(imm32)
    } else {
        rn.wrapping_sub(imm32)
    };
    let address = if index { offset_address } else { rn };
    (address, offset_address)
}

#[inline(always)]
fn expand_conditional_carry(imm32: &Imm32Carry, carry: bool) -> (u32, bool) {
    match imm32 {
        Imm32Carry::NoCarry { imm32 } => (*imm32, carry),
        Imm32Carry::Carry { imm32_c0, imm32_c1 } => {
            if carry {
                *imm32_c1
            } else {
                *imm32_c0
            }
        }
    }
}

#[inline(always)]
fn conditional_setflags(setflags: SetFlags, in_it_block: bool) -> bool {
    match setflags {
        SetFlags::True => true,
        SetFlags::False => false,
        SetFlags::NotInITBlock => !in_it_block,
    }
}

impl Processor {
    fn active_exception(&self) -> Option<Exception> {
        let active_exception = self.psr.get_isr_number();
        if active_exception == 0 {
            None
        } else {
            Some(Exception::from(active_exception))
        }
    }

    #[cfg(not(feature = "armv6m"))]
    fn fault_delivery_exception(&mut self, fault: Fault) -> Exception {
        let mapped_exception = fault.exception();

        let enabled = self.configurable_fault_enabled(mapped_exception);

        if enabled {
            mapped_exception
        } else {
            self.set_hfsr_forced();
            Exception::HardFault
        }
    }

    #[cfg(feature = "armv6m")]
    fn fault_delivery_exception(&mut self, fault: Fault) -> Exception {
        fault.exception()
    }

    fn queue_fault_trap(
        &mut self,
        fault: Fault,
        exception: Exception,
        pc: u32,
        active_exception: Option<Exception>,
        trap_reason: FaultTrapReason,
    ) {
        self.pending_fault_trap = Some(crate::core::fault::FaultContext {
            trap_reason,
            fault,
            exception,
            pc,
            active_exception,
        });
    }

    fn is_lockup(exception: Exception, active_exception: Option<Exception>) -> bool {
        exception == Exception::HardFault
            && matches!(
                active_exception,
                Some(Exception::HardFault | Exception::NMI)
            )
    }

    fn handle_fault(&mut self, fault: Fault, fault_pc: u32) -> u32 {
        let status = self.take_pending_fault_status();
        self.handle_fault_with_status(fault, fault_pc, status)
    }

    fn handle_fault_with_status(
        &mut self,
        fault: Fault,
        fault_pc: u32,
        status: FaultStatusContext,
    ) -> u32 {
        let exception = self.fault_delivery_exception(fault);
        let active_exception = self.active_exception();

        self.record_fault_status(fault, status);

        if Self::is_lockup(exception, active_exception) {
            self.queue_fault_trap(
                fault,
                exception,
                fault_pc,
                active_exception,
                FaultTrapReason::Lockup,
            );
            self.running = false;
            self.sleeping = false;
            return 12;
        }

        match self.exception_entry(exception, fault_pc) {
            Ok(()) => {
                if self.get_fault_trap_mode().should_trap(exception) {
                    self.queue_fault_trap(
                        fault,
                        exception,
                        fault_pc,
                        active_exception,
                        FaultTrapReason::Fault,
                    );
                }
            }
            Err(entry_fault) => {
                self.record_fault_status(entry_fault, FaultStatusContext::default());
                self.set_hfsr_forced();
                let trap_reason = if exception == Exception::HardFault
                    || Self::is_lockup(Exception::HardFault, active_exception)
                {
                    FaultTrapReason::Lockup
                } else {
                    FaultTrapReason::Fault
                };
                self.queue_fault_trap(
                    entry_fault,
                    Exception::HardFault,
                    fault_pc,
                    active_exception,
                    trap_reason,
                );
                self.running = false;
                self.sleeping = false;
            }
        }

        // TODO: proper amount of cycles calculation
        12
    }
}

impl ExecutorHelper for Processor {
    fn set_itstate(&mut self, state: u8) {
        self.itstate = state;
    }

    fn it_advance(&mut self) {
        if self.itstate != 0 {
            if self.itstate.get_bits(0..3) == 0 {
                self.itstate = 0;
            } else {
                let it = self.itstate.get_bits(0..5);
                self.itstate.set_bits(0..5, (it << 1) & 0b11111);
            }
        }
    }

    fn in_it_block(&self) -> bool {
        self.itstate.get_bits(0..4) != 0
    }

    fn last_in_it_block(&self) -> bool {
        self.itstate.get_bits(0..4) == 0b1000
    }
    fn integer_zero_divide_trapping_enabled(&self) -> bool {
        true
    }

    #[inline(always)]
    fn condition_passed(&self) -> bool {
        let itstate = self.itstate;

        if itstate == 0 {
            true
        } else {
            let cond = u16::from(itstate.get_bits(4..8));
            condition_test(
                Condition::from_u16(cond).unwrap_or(Condition::AL),
                &self.psr,
            )
        }
    }

    #[inline(always)]
    fn condition_passed_b(&self, cond: Condition) -> bool {
        condition_test(cond, &self.psr)
    }

    #[inline(always)]
    fn update_flags_check_it_block(
        &mut self,
        setflags: SetFlags,
        result: u32,
        carry: bool,
        overflow: bool,
    ) {
        if conditional_setflags(setflags, self.in_it_block()) {
            self.psr.set_n(result);
            self.psr.set_z(result);
            self.psr.set_c(carry);
            self.psr.set_v(overflow);
        }
    }

    #[allow(clippy::too_many_lines)]
    fn execute_internal(&mut self, instruction: &Instruction) -> ExecuteResult {
        match instruction {
            // --------------------------------------------
            //
            // Group: Branch instructions
            //
            // --------------------------------------------
            Instruction::B_t13 { params, .. } => self.exec_b_t13(*params),
            Instruction::B_t24 { imm32, .. } => self.exec_b_t24(*imm32),

            Instruction::BLX { rm } => self.exec_blx(*rm),
            Instruction::BX { rm } => self.exec_bx(*rm),
            Instruction::BL { imm32 } => self.exec_bl(*imm32),

            Instruction::CBZ { params } => self.exec_cbz(*params),
            Instruction::CBNZ { params } => self.exec_cbnz(*params),

            Instruction::TBB { params } => self.exec_tbb(*params),
            Instruction::TBH { params } => self.exec_tbh(*params),

            // --------------------------------------------
            //
            // Group: Standard data-processing instructions
            //
            // --------------------------------------------
            Instruction::ADD_reg { params, .. } => self.exec_add_reg(params),
            Instruction::ADD_imm { params, .. } => self.exec_add_imm(params),
            Instruction::ADD_sp_reg { params, .. } => self.exec_add_sp_reg(params),

            Instruction::ADC_reg { params, .. } => self.exec_adc_reg(params),
            Instruction::ADC_imm { params } => self.exec_adc_imm(params),

            Instruction::ADR { params, .. } => self.exec_adr(*params),

            Instruction::AND_reg { params, .. } => self.exec_and_reg(params),
            Instruction::AND_imm { params } => self.exec_and_imm(params),

            Instruction::BIC_imm { params } => self.exec_bic_imm(params),
            Instruction::BIC_reg { params, .. } => self.exec_bic_reg(params),

            Instruction::CMN_reg { params, .. } => self.exec_cmn_reg(params),
            Instruction::CMN_imm { params } => self.exec_cmn_imm(*params),

            Instruction::CMP_reg { params, .. } => self.exec_cmp_reg(params),
            Instruction::CMP_imm { params, .. } => self.exec_cmp_imm(*params),

            Instruction::EOR_reg { params, .. } => self.exec_eor_reg(params),
            Instruction::EOR_imm { params } => self.exec_eor_imm(params),

            Instruction::MOV_reg { params, .. } => self.exec_mov_reg(params),
            Instruction::MOV_imm { params, .. } => self.exec_mov_imm(params),

            Instruction::MVN_reg { params, .. } => self.exec_mvn_reg(params),
            Instruction::MVN_imm { params } => self.exec_mvn_imm(params),

            Instruction::ORN_reg { params } => self.exec_orn_reg(params),

            Instruction::ORR_imm { params } => self.exec_orr_imm(params),
            Instruction::ORR_reg { params, .. } => self.exec_orr_reg(params),

            Instruction::RSB_reg { params, .. } => self.exec_rsb_reg(params),
            Instruction::RSB_imm { params, .. } => self.exec_rsb_imm(params),

            Instruction::SBC_reg { params, .. } => self.exec_sbc_reg(params),
            Instruction::SBC_imm { params } => self.exec_sbc_imm(params),

            Instruction::SUB_reg { params, .. } => self.exec_sub_reg(params),
            Instruction::SUB_imm { params, .. } => self.exec_sub_imm(params),

            Instruction::TEQ_reg { params } => self.exec_teq_reg(params),
            Instruction::TEQ_imm { params } => self.exec_teq_imm(params),

            Instruction::TST_reg { params, .. } => self.exec_tst_reg(params),
            Instruction::TST_imm { params } => self.exec_tst_imm(params),

            // --------------------------------------------
            //
            // Group: Shift instructions
            //
            // --------------------------------------------
            Instruction::ASR_imm { params, .. } => self.exec_asr_imm(params),
            Instruction::ASR_reg { params, .. } => self.exec_asr_reg(params),

            Instruction::LSL_imm { params, .. } => self.exec_lsl_imm(params),
            Instruction::LSL_reg { params, .. } => self.exec_lsl_reg(params),

            Instruction::LSR_imm { params, .. } => self.exec_lsr_imm(params),
            Instruction::LSR_reg { params, .. } => self.exec_lsr_reg(params),

            Instruction::ROR_imm { params } => self.exec_ror_imm(params),
            Instruction::ROR_reg { params, .. } => self.exec_ror_reg(params),

            Instruction::RRX { params } => self.exec_rrx(params),
            // --------------------------------------------
            //
            // Group: Multiply instructions
            //
            // --------------------------------------------
            Instruction::MLA { params } => self.exec_mla(params),
            Instruction::MLS { params } => self.exec_mls(params),
            Instruction::MUL { params, .. } => self.exec_mul(params),
            // --------------------------------------------
            //
            // Group: Signed multiply instructions (ArmV7-m)
            //
            // --------------------------------------------
            Instruction::SMLAL { params } => self.exec_smlal(params),
            Instruction::SMULL { params } => self.exec_smull(params),

            // --------------------------------------------
            //
            // Group: Unsigned Multiply instructions (ARMv7-M base architecture)
            //
            // --------------------------------------------
            Instruction::UMLAL { params } => self.exec_umlal(params),
            Instruction::UMULL { params } => self.exec_umull(params),

            // --------------------------------------------
            //
            // Group: Signed Multiply instructions (ARMv7-M DSP extension)
            //
            // --------------------------------------------
            #[cfg(feature = "has-dsp-ext")]
            Instruction::SMUL { params } => self.exec_smul(params),
            #[cfg(not(feature = "has-dsp-ext"))]
            Instruction::SMUL { .. } => Err(Fault::UndefInstr),
            #[cfg(feature = "has-dsp-ext")]
            Instruction::SMLA { params } => self.exec_smla(params),
            #[cfg(not(feature = "has-dsp-ext"))]
            Instruction::SMLA { .. } => Err(Fault::UndefInstr),

            // --------------------------------------------
            //
            // Group: Saturating instructions (ARMv7-M base arch)
            //
            // --------------------------------------------

            // --------------------------------------------
            //
            // Group: Saturating instructions (ARMv7-M DSP extensions)
            //
            // --------------------------------------------

            // --------------------------------------------
            //
            // Group: Saturating add/sub (ARMv7-M DSP extensions)
            //
            // --------------------------------------------

            // --------------------------------------------
            //
            // Group: Packing and unpacking instructions
            //
            // --------------------------------------------
            Instruction::SXTB { params, .. } => self.exec_sxtb(params),
            Instruction::SXTH { params, .. } => self.exec_sxth(params),

            Instruction::UXTB { params, .. } => self.exec_uxtb(params),
            Instruction::UXTH { params, .. } => self.exec_uxth(params),

            // --------------------------------------------
            //
            // Group: Packing and unpacking instructions (DSP extensions)
            //
            // --------------------------------------------
            Instruction::UXTAB { params } => self.exec_uxtab(params),

            // --------------------------------------------
            //
            // Group: Divide instructions
            //
            // --------------------------------------------
            // ARMv7-M
            Instruction::SDIV { params } => self.exec_sdiv(params),
            Instruction::UDIV { params } => self.exec_udiv(params),

            // --------------------------------------------
            //
            // Group: Parallel add / sub (DSP extension)
            //
            // --------------------------------------------
            #[cfg(feature = "has-dsp-ext")]
            Instruction::UADD8 { params } => self.exec_uadd8(params),
            #[cfg(not(feature = "has-dsp-ext"))]
            Instruction::UADD8 { .. } => Err(Fault::UndefInstr),

            // --------------------------------------------
            //
            // Group:  Miscellaneous data-processing instructions
            //
            // --------------------------------------------
            Instruction::BFC { params } => self.exec_bfc(params),
            Instruction::BFI { params } => self.exec_bfi(params),

            Instruction::CLZ { params } => self.exec_clz(*params),

            Instruction::MOVT { params } => self.exec_movt(*params),

            Instruction::REV { params, .. } => self.exec_rev(*params),
            Instruction::REV16 { params, .. } => self.exec_rev16(*params),
            Instruction::REVSH { params, .. } => self.exec_revsh(*params),

            Instruction::UBFX { params } => self.exec_ubfx(params),
            Instruction::SBFX { params } => self.exec_sbfx(params),

            // --------------------------------------------
            //
            // Group:  Miscellaneous data-processing instructions (DSP extensions)
            //
            // --------------------------------------------
            #[cfg(feature = "has-dsp-ext")]
            Instruction::SEL { params } => self.exec_sel(params),
            #[cfg(not(feature = "has-dsp-ext"))]
            Instruction::SEL { .. } => Err(Fault::UndefInstr),
            // --------------------------------------------
            //
            // Group: Status register access instructions
            //
            // --------------------------------------------
            Instruction::MRS { params } => self.exec_mrs(*params),
            Instruction::MSR_reg { params } => self.exec_msr(*params),

            #[cfg(feature = "armv6m")]
            Instruction::CPS { im } => self.exec_cps(*im),

            #[cfg(not(feature = "armv6m"))]
            Instruction::CPS {
                im,
                affect_pri,
                affect_fault,
            } => self.exec_cps(*im, *affect_pri, *affect_fault),

            // --------------------------------------------
            //
            // Group:  Load and Store instructions
            //
            // --------------------------------------------
            Instruction::LDREX { params } => self.exec_ldrex(*params),
            Instruction::LDREXB { params } => self.exec_ldrexb(*params),
            Instruction::LDREXH { params } => self.exec_ldrexh(*params),

            Instruction::LDR_imm { params, .. } => self.exec_ldr_imm(params),
            Instruction::LDRB_imm { params, .. } => self.exec_ldrb_imm(params),
            Instruction::LDRH_imm { params, .. } => self.exec_ldrh_imm(params),
            Instruction::LDRSB_imm { params, .. } => self.exec_ldrsb_imm(params),
            Instruction::LDRSH_imm { params, .. } => self.exec_ldrsh_imm(params),

            Instruction::LDR_reg { params, .. } => self.exec_ldr_reg(params),
            Instruction::LDRB_reg { params, .. } => self.exec_ldrb_reg(params),
            Instruction::LDRH_reg { params, .. } => self.exec_ldrh_reg(params),
            Instruction::LDRSB_reg { params, .. } => self.exec_ldrsb_reg(params),
            Instruction::LDRSH_reg { params, .. } => self.exec_ldrsh_reg(params),

            Instruction::STR_imm { params, .. } => self.exec_str_imm(params),
            Instruction::STRB_imm { params, .. } => self.exec_strb_imm(params),
            Instruction::STRH_imm { params, .. } => self.exec_strh_imm(params),

            Instruction::STR_reg { params, .. } => self.exec_str_reg(params),
            Instruction::STRB_reg { params, .. } => self.exec_strb_reg(params),
            Instruction::STRH_reg { params, .. } => self.exec_strh_reg(params),

            Instruction::STREX { params } => self.exec_strex(*params),
            Instruction::STREXB { params } => self.exec_strexb(*params),
            Instruction::STREXH { params } => self.exec_strexh(*params),

            Instruction::STRD_imm { params } => self.exec_strd_imm(params),
            Instruction::LDRD_imm { params } => self.exec_ldrd_imm(params),

            Instruction::LDR_lit { params, .. } => self.exec_ldr_lit(params),

            // --------------------------------------------
            //
            // Group:  Load and Store Multiple instructions
            //
            // --------------------------------------------
            Instruction::STM { params, .. } => self.exec_stm(params),
            Instruction::STMDB { params } => self.exec_stmdb(params),
            Instruction::LDM { params, .. } => self.exec_ldm(params),

            Instruction::PUSH { registers, .. } => self.exec_push(*registers),
            Instruction::POP { registers, .. } => self.exec_pop(*registers),

            // --------------------------------------------
            //
            // Group: Miscellaneous
            //
            // --------------------------------------------
            Instruction::DMB => self.exec_dmb(),
            Instruction::DSB => self.exec_dsb(),
            Instruction::ISB => self.exec_isb(),

            Instruction::IT {
                firstcond, mask, ..
            } => self.exec_it(*firstcond, *mask),

            Instruction::NOP { .. } => Ok(ExecuteSuccess::Taken { cycles: 1 }),

            Instruction::PLD_imm { .. } => self.exec_pld_imm(),
            Instruction::PLD_lit { .. } => self.exec_pld_lit(),
            Instruction::PLD_reg { .. } => self.exec_pld_reg(),

            Instruction::SEV { .. } => self.exec_sev(),
            Instruction::WFE { .. } => self.exec_wfe(),
            Instruction::YIELD { .. } => self.exec_yield(),
            Instruction::WFI { .. } => self.exec_wfi(),

            // --------------------------------------------
            //
            // Group: Exception generating instructions
            //
            // --------------------------------------------
            Instruction::SVC { .. } => self.exec_svc(),
            Instruction::BKPT { imm32 } => self.exec_bkpt(*imm32),

            // --------------------------------------------
            //
            // Group: Coprocessor instructions
            //
            // --------------------------------------------
            Instruction::MCR { .. } => self.exec_mcr(),
            Instruction::MCR2 { .. } => self.exec_mcr2(),
            Instruction::LDC_imm { .. } => self.exec_ldc_imm(),
            Instruction::LDC2_imm { .. } => self.exec_ldc2_imm(),
            // --------------------------------------------
            //
            // Group: Floating-point load and store instructions
            //
            // --------------------------------------------
            #[cfg(feature = "has-fp")]
            Instruction::VLDR { params } => self.exec_vldr(params),
            #[cfg(feature = "has-fp")]
            Instruction::VSTR { params } => self.exec_vstr(params),
            #[cfg(feature = "has-fp")]
            Instruction::VLDM_T1 { params } => self.exec_vldm_t1(params),
            #[cfg(feature = "has-fp")]
            Instruction::VLDM_T2 { params } => self.exec_vldm_t2(params),
            #[cfg(feature = "has-fp")]
            Instruction::VSTM_T1 { params } => self.exec_vstm_t1(params),
            #[cfg(feature = "has-fp")]
            Instruction::VSTM_T2 { params } => self.exec_vstm_t2(params),
            #[cfg(feature = "has-fp")]
            Instruction::VPUSH { params } => self.exec_vpush(params),
            #[cfg(feature = "has-fp")]
            Instruction::VPOP { params } => self.exec_vpop(params),
            #[cfg(feature = "has-fp")]
            Instruction::VMOV_imm_32 { params } => self.exec_vmov_imm_32(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VMOV_imm_64 { params } => self.exec_vmov_imm_64(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VMOV_reg_f32 { params } => self.exec_vmov_reg_f32(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VMOV_reg_f64 { params } => self.exec_vmov_reg_f64(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VMOV_cr_scalar { .. } => unimplemented!(),
            #[cfg(feature = "has-fp")]
            Instruction::VMOV_scalar_cr { .. } => unimplemented!(),
            #[cfg(feature = "has-fp")]
            Instruction::VMOV_cr_sp { params } => self.exec_vmov_cr_sp(params),
            #[cfg(feature = "has-fp")]
            Instruction::VMOV_cr2_sp2 { .. } => unimplemented!(),
            #[cfg(feature = "has-fp")]
            Instruction::VMOV_cr2_dp { params } => self.exec_vmov_cr2_dp(params),

            // --------------------------------------------
            //
            // Group: Floating-point register transfer instructions
            //
            // --------------------------------------------
            #[cfg(feature = "has-fp")]
            Instruction::VMRS { rt } => self.exec_vmrs(*rt),

            // --------------------------------------------
            //
            // Group: Floating-point data-processing instructions
            //
            // --------------------------------------------
            #[cfg(feature = "has-fp")]
            Instruction::VABS_f32 { params } => self.exec_vabs_f32(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VABS_f64 { params } => self.exec_vabs_f64(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VNEG_f32 { params } => self.exec_vneg_f32(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VNEG_f64 { params } => self.exec_vneg_f64(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VCMP_f32 { params } => self.exec_vcmp_f32(params),
            #[cfg(feature = "has-fp")]
            Instruction::VCMP_f64 { params } => self.exec_vcmp_f64(params),
            #[cfg(feature = "has-fp")]
            Instruction::VADD_f32 { params } => self.exec_vadd_f32(params),
            #[cfg(feature = "has-fp")]
            Instruction::VADD_f64 { params } => self.exec_vadd_f64(params),
            #[cfg(feature = "has-fp")]
            Instruction::VFMA_f32 { params } => self.exec_vfma_f32(params),
            #[cfg(feature = "has-fp")]
            Instruction::VFMA_f64 { params } => self.exec_vfma_f64(params),
            #[cfg(feature = "has-fp")]
            Instruction::VFMS_f32 { params } => self.exec_vfms_f32(params),
            #[cfg(feature = "has-fp")]
            Instruction::VFMS_f64 { params } => self.exec_vfms_f64(params),
            #[cfg(feature = "has-fp")]
            Instruction::VFNMS_f32 { params } => self.exec_vfnms_f32(params),
            #[cfg(feature = "has-fp")]
            Instruction::VFNMS_f64 { params } => self.exec_vfnms_f64(params),
            #[cfg(feature = "has-fp")]
            Instruction::VDIV_f32 { params } => self.exec_vdiv_f32(params),
            #[cfg(feature = "has-fp")]
            Instruction::VDIV_f64 { params } => self.exec_vdiv_f64(params),
            #[cfg(feature = "has-fp")]
            Instruction::VMUL_f32 { params } => self.exec_vmul_f32(params),
            #[cfg(feature = "has-fp")]
            Instruction::VMUL_f64 { params } => self.exec_vmul_f64(params),
            #[cfg(feature = "has-fp")]
            Instruction::VNMUL_f32 { params } => self.exec_vnmul_f32(params),
            #[cfg(feature = "has-fp")]
            Instruction::VNMUL_f64 { params } => self.exec_vnmul_f64(params),
            #[cfg(feature = "has-fp")]
            Instruction::VSQRT_f32 { params } => self.exec_vsqrt_f32(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VSQRT_f64 { params } => self.exec_vsqrt_f64(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VRINTZ_f32 { params } => self.exec_vrintz_f32(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VRINTZ_f64 { params } => self.exec_vrintz_f64(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VSUB_f32 { params } => self.exec_vsub_f32(params),
            #[cfg(feature = "has-fp")]
            Instruction::VSUB_f64 { params } => self.exec_vsub_f64(params),
            #[cfg(feature = "has-fp")]
            Instruction::VCVT { params } => self.exec_vcvt(params),
            #[cfg(feature = "has-fp")]
            Instruction::VCVT_f64_f32 { params } => self.exec_vcvt_f64_f32(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VCVT_f32_f64 { params } => self.exec_vcvt_f32_f64(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VSEL_f32 { params } => self.exec_vsel_f32(*params),
            #[cfg(feature = "has-fp")]
            Instruction::VSEL_f64 { params } => self.exec_vsel_f64(*params),

            // --------------------------------------------
            //
            // Fallback: unknown instruction
            //
            // --------------------------------------------
            Instruction::UDF { .. } => Err(Fault::UndefInstr),
        }
    }
}

impl Executor for Processor {
    #[inline(always)]
    fn step_sleep(&mut self) {
        if (self.syst_csr & 1) != 0 {
            self.syst_step(1);
        }
        self.check_exceptions();
        if (self.dwt_ctrl & 1) != 0 {
            self.dwt_tick(1);
        }
    }

    #[inline(always)]
    fn step(&mut self) {
        let pc = self.get_pc();
        let mapped_pc = if self.mem_map.is_some() {
            (self.map_address(pc) >> 1) as usize
        } else {
            (pc >> 1) as usize
        };
        let count = match self.instruction_cache.get(mapped_pc).copied() {
            Some(CachedInstruction::Decoded {
                instruction,
                instruction_size,
            }) => self.execute(&instruction, instruction_size),
            Some(CachedInstruction::FetchFault { fault, status }) => {
                self.handle_fault_with_status(fault, pc, status)
            }
            None => match self.fetch(pc) {
                Ok(thumb) => {
                    let instruction = self.decode(thumb);
                    self.execute(&instruction, instruction_size(&instruction))
                }
                Err(fault) => self.handle_fault(fault, pc),
            },
        };
        self.cycle_count += u64::from(count);
        if (self.dwt_ctrl & 1) != 0 {
            self.dwt_tick(count);
        }
        if (self.syst_csr & 1) != 0 {
            self.syst_step(count);
        }
        if self.pending_exception_count != 0 || self.sleeping {
            self.check_exceptions();
        }
        //TODO exception entry also burns cycles that should be accounted for
        //DWT and SYST ticking
    }

    #[inline(always)]
    fn execute(&mut self, instruction: &Instruction, instruction_size: usize) -> u32 {
        self.instruction_count += 1;

        let in_it_block = self.in_it_block();

        match self.execute_internal(instruction) {
            Err(fault) => {
                let pc = self.get_pc();
                self.handle_fault(fault, pc)
            }
            Ok(ExecuteSuccess::NotTaken) => {
                self.add_pc(instruction_size as u32);
                if in_it_block {
                    self.it_advance();
                }
                1
            }
            Ok(ExecuteSuccess::Branched { cycles }) => {
                if in_it_block {
                    self.it_advance();
                }
                cycles
            }
            Ok(ExecuteSuccess::Taken { cycles }) => {
                self.add_pc(instruction_size as u32);

                if in_it_block {
                    self.it_advance();
                }
                cycles
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(not(feature = "armv6m"))]
    use crate::bus::Bus;
    use crate::core::condition::Condition;
    use crate::core::fault::{Fault, FaultTrapReason};
    use crate::core::instruction::instruction_size;
    #[cfg(not(feature = "armv6m"))]
    use crate::core::{fault::FaultTrapMode, register::Ipsr};
    use crate::core::{
        instruction::{ITCondition, SRType, SetFlags},
        register::{BaseReg, Reg},
    };

    #[cfg(not(feature = "armv6m"))]
    const CFSR_UNDEFINSTR: u32 = 1 << 16;
    #[cfg(not(feature = "armv6m"))]
    const CFSR_INVPC: u32 = 1 << 18;
    #[cfg(not(feature = "armv6m"))]
    const CFSR_IACCVIOL: u32 = 1 << 0;
    #[cfg(not(feature = "armv6m"))]
    const CFSR_DACCVIOL: u32 = 1 << 1;
    #[cfg(not(feature = "armv6m"))]
    const CFSR_MSTKERR: u32 = 1 << 4;
    #[cfg(not(feature = "armv6m"))]
    const CFSR_MMARVALID: u32 = 1 << 7;
    #[cfg(not(feature = "armv6m"))]
    const CFSR_IBUSERR: u32 = 1 << 8;
    #[cfg(not(feature = "armv6m"))]
    const CFSR_PRECISERR: u32 = 1 << 9;
    #[cfg(not(feature = "armv6m"))]
    const CFSR_BFARVALID: u32 = 1 << 15;
    #[cfg(not(feature = "armv6m"))]
    const HFSR_VECTTBL: u32 = 1 << 1;
    #[cfg(not(feature = "armv6m"))]
    const CFSR_INVSTATE: u32 = 1 << 17;
    #[cfg(not(feature = "armv6m"))]
    const HFSR_FORCED: u32 = 1 << 30;
    #[cfg(not(feature = "armv6m"))]
    const SHCSR_MEMFAULTENA: u32 = 1 << 16;
    #[cfg(not(feature = "armv6m"))]
    const SHCSR_BUSFAULTENA: u32 = 1 << 17;
    #[cfg(not(feature = "armv6m"))]
    const SHCSR_USGFAULTENA: u32 = 1 << 18;

    #[test]
    fn test_execute_internal_udf_returns_undefinstr_fault() {
        let mut core = Processor::new();

        let result = core.execute_internal(&Instruction::UDF {
            imm32: 0,
            opcode: crate::core::thumb::ThumbCode::Thumb16 { opcode: 0xde00 },
            thumb32: false,
        });

        assert_eq!(result, Err(Fault::UndefInstr));
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_execute_udf_escalates_to_hardfault_when_usagefault_disabled() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);
        core.set_pc(0x0800_0004);

        let instruction = Instruction::UDF {
            imm32: 0,
            opcode: crate::core::thumb::ThumbCode::Thumb16 { opcode: 0xde00 },
            thumb32: false,
        };

        let cycles = core.execute(&instruction, instruction_size(&instruction));

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::HardFault.into());
        assert_eq!(core.cfsr, CFSR_UNDEFINSTR);
        assert_eq!(core.hfsr, HFSR_FORCED);
        assert_eq!(core.take_pending_fault_trap(), None);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_execute_udf_enters_usagefault_when_enabled() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);
        core.set_pc(0x0800_0004);
        core.write32(0xE000_ED24, SHCSR_USGFAULTENA).unwrap();

        let instruction = Instruction::UDF {
            imm32: 0,
            opcode: crate::core::thumb::ThumbCode::Thumb16 { opcode: 0xde00 },
            thumb32: false,
        };

        let cycles = core.execute(&instruction, instruction_size(&instruction));

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::UsageFault.into());
        assert_eq!(core.cfsr, CFSR_UNDEFINSTR);
        assert_eq!(core.hfsr, 0);
        assert_eq!(core.take_pending_fault_trap(), None);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_execute_svc_enters_svcall_and_stacks_next_pc() {
        const STACK_TOP: u32 = 0x2000_0100;
        const SVC_VECTOR: u32 = 11 * 4;
        const SVC_HANDLER: u32 = 0x0800_0101;

        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(STACK_TOP);
        core.set_pc(0x0800_0044);
        core.vtor = 0;

        let mut image = vec![0_u8; 0x100];
        image[SVC_VECTOR as usize..SVC_VECTOR as usize + 4]
            .copy_from_slice(&SVC_HANDLER.to_le_bytes());
        core.flash_memory(image.len(), &image);

        let instruction = Instruction::SVC { imm32: 0 };

        let cycles = core.execute(&instruction, instruction_size(&instruction));

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::SVCall.into());
        assert_eq!(core.get_pc(), 0x0800_0100);
        assert_eq!(core.get_msp(), STACK_TOP - 0x20);
        assert_eq!(core.read32(STACK_TOP - 0x08).unwrap(), 0x0800_0046);
        assert_eq!(core.get_r(Reg::LR), 0xFFFF_FFF9);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_fault_trap_mode_matrix_for_all_mapped_faults() {
        let cases = [
            (Fault::Forced, Exception::HardFault),
            (Fault::DAccViol, Exception::MemoryManagementFault),
            (Fault::Preciserr, Exception::BusFault),
            (Fault::UndefInstr, Exception::UsageFault),
        ];

        for trap_bits in 0u8..16 {
            let mut mode = FaultTrapMode::none();
            mode.set_trap(Exception::HardFault, (trap_bits & 0b0001) != 0);
            mode.set_trap(Exception::MemoryManagementFault, (trap_bits & 0b0010) != 0);
            mode.set_trap(Exception::BusFault, (trap_bits & 0b0100) != 0);
            mode.set_trap(Exception::UsageFault, (trap_bits & 0b1000) != 0);

            for (index, (fault, exception)) in cases.iter().copied().enumerate() {
                let mut core = Processor::new();
                core.fault_trap_mode(mode);
                core.set_msp(0x2000_0100);
                core.write32(
                    0xE000_ED24,
                    SHCSR_MEMFAULTENA | SHCSR_BUSFAULTENA | SHCSR_USGFAULTENA,
                )
                .unwrap();

                let pc = 0x0800_0000 + u32::from(trap_bits) * 0x10 + u32::try_from(index).unwrap();
                let cycles = core.handle_fault(fault, pc);

                assert_eq!(cycles, 12);
                assert_eq!(core.psr.get_isr_number(), exception.into());

                let trap = core.take_pending_fault_trap();
                let should_trap = mode.should_trap(exception);

                assert_eq!(
                    trap.is_some(),
                    should_trap,
                    "trap_bits={trap_bits:04b} fault={fault:?}"
                );

                if let Some(trap) = trap {
                    assert_eq!(trap.trap_reason, FaultTrapReason::Fault);
                    assert_eq!(trap.fault, fault);
                    assert_eq!(trap.exception, exception);
                    assert_eq!(trap.pc, pc);
                }
            }
        }
    }

    #[test]
    fn test_default_fault_trap_traps_hardfault() {
        let mut core = Processor::new();
        core.set_msp(0x2000_0100);

        let cycles = core.handle_fault(Fault::Forced, 0x0800_0000);

        assert_eq!(cycles, 12);
        let trap = core
            .take_pending_fault_trap()
            .expect("hardfault trap expected");
        assert_eq!(trap.trap_reason, FaultTrapReason::Fault);
        assert_eq!(trap.fault, Fault::Forced);
        assert_eq!(trap.exception, Exception::HardFault);
        assert_eq!(trap.pc, 0x0800_0000);
    }

    #[test]
    fn test_lockup_always_traps_even_when_hardfault_trap_disabled() {
        let mut core = Processor::new();
        core.fault_trap_mode(crate::core::fault::FaultTrapMode::none());
        core.psr.set_isr_number(Exception::HardFault.into());
        core.mode = crate::ProcessorMode::HandlerMode;

        let cycles = core.handle_fault(Fault::Forced, 0x0800_0100);

        assert_eq!(cycles, 12);
        let trap = core
            .take_pending_fault_trap()
            .expect("lockup trap expected");
        assert_eq!(trap.trap_reason, FaultTrapReason::Lockup);
        assert_eq!(trap.fault, Fault::Forced);
        assert_eq!(trap.exception, Exception::HardFault);
        assert_eq!(trap.pc, 0x0800_0100);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_handle_fault_sets_cfsr_undefinstr_bit() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);
        core.write32(0xE000_ED24, SHCSR_USGFAULTENA).unwrap();

        let cycles = core.handle_fault(Fault::UndefInstr, 0x0800_0200);

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::UsageFault.into());
        assert_eq!(core.cfsr, CFSR_UNDEFINSTR);
        assert_eq!(core.hfsr, 0);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_handle_fault_sets_cfsr_invpc_bit() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);
        core.write32(0xE000_ED24, SHCSR_USGFAULTENA).unwrap();

        let cycles = core.handle_fault(Fault::InvPc, 0x0800_0210);

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::UsageFault.into());
        assert_eq!(core.cfsr, CFSR_INVPC);
        assert_eq!(core.hfsr, 0);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_invalid_exception_return_enters_usagefault_when_enabled() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);
        core.write32(0xE000_ED24, SHCSR_USGFAULTENA).unwrap();
        core.psr.set_isr_number(Exception::SysTick.into());
        core.mode = crate::ProcessorMode::HandlerMode;

        let fault = core
            .bx_write_pc(0xFFFF_FFF0)
            .expect_err("INVPC fault expected");
        let cycles = core.handle_fault(fault, 0x0800_0300);

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::UsageFault.into());
        assert_eq!(core.cfsr, CFSR_INVPC);
        assert_eq!(core.hfsr, 0);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_invalid_exception_return_escalates_to_hardfault_when_usagefault_disabled() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);
        core.psr.set_isr_number(Exception::SysTick.into());
        core.mode = crate::ProcessorMode::HandlerMode;

        let fault = core
            .bx_write_pc(0xFFFF_FFF0)
            .expect_err("INVPC fault expected");
        let cycles = core.handle_fault(fault, 0x0800_0310);

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::HardFault.into());
        assert_eq!(core.cfsr, CFSR_INVPC);
        assert_eq!(core.hfsr, HFSR_FORCED);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_invalid_exception_return_with_bad_thumb_state_enters_usagefault_when_enabled() {
        const FRAMEPTR: u32 = 0x2000_00e0;

        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(FRAMEPTR);
        core.write32(0xE000_ED24, SHCSR_USGFAULTENA).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x1c), 0).unwrap();
        core.test_set_active_exception(Exception::SysTick);

        let fault = core
            .bx_write_pc(0xFFFF_FFF9)
            .expect_err("INVSTATE fault expected");
        let cycles = core.handle_fault(fault, 0x0800_0320);

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::UsageFault.into());
        assert_eq!(core.cfsr, CFSR_INVSTATE);
        assert_eq!(core.hfsr, 0);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_invalid_exception_return_with_bad_thumb_state_escalates_to_hardfault_when_usagefault_disabled()
     {
        const FRAMEPTR: u32 = 0x2000_00e0;

        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(FRAMEPTR);
        core.write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x1c), 0).unwrap();
        core.test_set_active_exception(Exception::SysTick);

        let fault = core
            .bx_write_pc(0xFFFF_FFF9)
            .expect_err("INVSTATE fault expected");
        let cycles = core.handle_fault(fault, 0x0800_0330);

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::HardFault.into());
        assert_eq!(core.cfsr, CFSR_INVSTATE);
        assert_eq!(core.hfsr, HFSR_FORCED);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_invalid_exception_return_with_nonzero_stacked_ipsr_enters_usagefault_when_enabled() {
        const FRAMEPTR: u32 = 0x2000_00e0;

        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(FRAMEPTR);
        core.write32(0xE000_ED24, SHCSR_USGFAULTENA).unwrap();
        core.write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        core.write32(
            FRAMEPTR.wrapping_add(0x1c),
            (1 << 24) | usize::from(Exception::HardFault) as u32,
        )
        .unwrap();
        core.test_set_active_exception(Exception::SysTick);

        let fault = core
            .bx_write_pc(0xFFFF_FFF9)
            .expect_err("INVPC fault expected");
        let cycles = core.handle_fault(fault, 0x0800_0340);

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::UsageFault.into());
        assert_eq!(core.cfsr, CFSR_INVPC);
        assert_eq!(core.hfsr, 0);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_invalid_exception_return_with_nonzero_stacked_ipsr_escalates_to_hardfault_when_usagefault_disabled()
     {
        const FRAMEPTR: u32 = 0x2000_00e0;

        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(FRAMEPTR);
        core.write32(FRAMEPTR.wrapping_add(0x18), 0x0800_0001)
            .unwrap();
        core.write32(
            FRAMEPTR.wrapping_add(0x1c),
            (1 << 24) | usize::from(Exception::HardFault) as u32,
        )
        .unwrap();
        core.test_set_active_exception(Exception::SysTick);

        let fault = core
            .bx_write_pc(0xFFFF_FFF9)
            .expect_err("INVPC fault expected");
        let cycles = core.handle_fault(fault, 0x0800_0350);

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::HardFault.into());
        assert_eq!(core.cfsr, CFSR_INVPC);
        assert_eq!(core.hfsr, HFSR_FORCED);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_handle_fault_sets_cfsr_invstate_bit() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);
        core.write32(0xE000_ED24, SHCSR_USGFAULTENA).unwrap();

        let cycles = core.handle_fault(Fault::Invstate, 0x0800_0214);

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::UsageFault.into());
        assert_eq!(core.cfsr, CFSR_INVSTATE);
        assert_eq!(core.hfsr, 0);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_handle_fault_sets_hfsr_vecttbl_bit() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);

        let cycles = core.handle_fault(Fault::VectorTable, 0x0800_0220);

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::HardFault.into());
        assert_eq!(core.hfsr, HFSR_VECTTBL);
        assert_eq!(core.cfsr, 0);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_handle_fault_sets_hfsr_forced_bit_for_forced_fault() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);

        let cycles = core.handle_fault(Fault::Forced, 0x0800_0240);

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::HardFault.into());
        assert_eq!(core.hfsr, HFSR_FORCED);
        assert_eq!(core.cfsr, 0);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_handle_fault_escalates_entry_stacking_fault_to_hardfault() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0);

        let cycles = core.handle_fault(Fault::Forced, 0x0800_0250);

        assert_eq!(cycles, 12);
        let trap = core.take_pending_fault_trap().expect("fault trap expected");
        assert_eq!(trap.fault, Fault::Mstkerr);
        assert_eq!(trap.exception, Exception::HardFault);
        assert_eq!(core.cfsr, CFSR_MSTKERR);
        assert_eq!(core.hfsr, HFSR_FORCED);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_execute_str_reg_latches_mmfar_for_data_access_violation() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);
        core.write32(0xE000_ED24, SHCSR_MEMFAULTENA).unwrap();
        core.set_r(Reg::R0, 0x1234_5678);
        core.set_r(Reg::R1, 0x6000_0000);
        core.set_r(Reg::R2, 0);

        let instruction = Instruction::STR_reg {
            params: crate::core::instruction::Reg3FullParams {
                rt: Reg::R0,
                rn: Reg::R1,
                rm: Reg::R2,
                shift_t: SRType::LSL,
                shift_n: 0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: false,
        };

        let cycles = core.execute(&instruction, instruction_size(&instruction));

        assert_eq!(cycles, 12);
        assert_eq!(
            core.psr.get_isr_number(),
            Exception::MemoryManagementFault.into()
        );
        assert_eq!(core.cfsr, CFSR_DACCVIOL | CFSR_MMARVALID);
        assert_eq!(core.mmfar, 0x6000_0000);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_step_latches_mmfar_for_instruction_access_violation() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);
        core.write32(0xE000_ED24, SHCSR_MEMFAULTENA).unwrap();
        core.set_pc(0x6000_0000);

        core.step();

        assert_eq!(
            core.psr.get_isr_number(),
            Exception::MemoryManagementFault.into()
        );
        assert_eq!(core.cfsr, CFSR_IACCVIOL | CFSR_MMARVALID);
        assert_eq!(core.mmfar, 0x6000_0000);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_handle_fault_sets_bfar_for_precise_bus_fault() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);
        core.write32(0xE000_ED24, SHCSR_BUSFAULTENA).unwrap();

        let cycles = core.handle_fault_with_status(
            Fault::Preciserr,
            0x0800_0230,
            FaultStatusContext::with_fault_address(0x4000_1000),
        );

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::BusFault.into());
        assert_eq!(core.cfsr, CFSR_PRECISERR | CFSR_BFARVALID);
        assert_eq!(core.bfar, 0x4000_1000);
    }

    #[test]
    #[cfg(not(feature = "armv6m"))]
    fn test_handle_fault_without_address_does_not_set_bfarvalid_or_overwrite_bfar() {
        let mut core = Processor::new();
        core.fault_trap_mode(FaultTrapMode::none());
        core.set_msp(0x2000_0100);
        core.write32(0xE000_ED24, SHCSR_BUSFAULTENA).unwrap();
        core.bfar = 0xfeed_face;

        let cycles = core.handle_fault_with_status(
            Fault::IBusErr,
            0x0800_0240,
            FaultStatusContext::default(),
        );

        assert_eq!(cycles, 12);
        assert_eq!(core.psr.get_isr_number(), Exception::BusFault.into());
        assert_eq!(core.cfsr, CFSR_IBUSERR);
        assert_eq!(core.bfar, 0xfeed_face);
    }

    #[test]
    fn test_it_block_branch_clears_state() {
        // This test reproduces a bug: when a conditional branch inside
        // an IT block executes and branches, the IT state is not advanced,
        // leaving residual itstate that affects the next instruction.
        //
        // This is the exact sequence from hello_world-cm3.elf:
        // 3fc: subs r3, #4
        // 3fe: ittt ge
        // 400: ldrge r0, [r1, r3]  <- conditional
        // 402: strge r0, [r2, r3]  <- conditional
        // 404: bge 0x3fc            <- conditional branch that loops back
        //
        // When the branch executes, IT state should advance (clearing it since
        // it's the last instruction), but in bug case it didn't, so the SUB at
        // 0x3fc thought it's still in an IT block and didn't set flags!

        use crate::core::instruction::Reg2ImmParams;

        let mut core = Processor::new();
        core.set_r(Reg::R1, 0x2000_0000);
        core.set_r(Reg::R2, 0x2000_1000);
        core.set_r(Reg::R3, 0x0000_0008);

        // Set flags so GE is true initially (N=0, V=0)
        core.psr.set_n(0);
        core.psr.set_v(false);
        core.psr.set_z(0);
        core.psr.set_c(true);

        // Execute "ittt ge" - 3 Then instructions with GE condition
        let it_inst = Instruction::IT {
            x: Some(ITCondition::Then),
            y: Some(ITCondition::Then),
            z: None,
            firstcond: Condition::GE,
            mask: 0b0010,
        };
        core.execute(&it_inst, instruction_size(&it_inst));

        // Execute 3 conditional instructions, the last being a branch
        // Instruction 1: ldr r0, [r1, r3]
        let ldr = Instruction::LDR_reg {
            params: crate::core::instruction::Reg3FullParams {
                rt: Reg::R0,
                rn: Reg::R1,
                rm: Reg::R3,
                shift_t: SRType::LSL,
                shift_n: 0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: false,
        };
        core.execute(&ldr, instruction_size(&ldr));

        // Instruction 2: str r0, [r2, r3]
        let str = Instruction::STR_reg {
            params: crate::core::instruction::Reg3FullParams {
                rt: Reg::R0,
                rn: Reg::R2,
                rm: Reg::R3,
                shift_t: SRType::LSL,
                shift_n: 0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: false,
        };
        core.execute(&str, instruction_size(&str));

        // Instruction 3: b (conditional branch back)
        // When this executes and branches, itstate should be cleared
        let b = Instruction::B_t13 {
            params: crate::core::instruction::CondBranchParams {
                cond: Condition::GE,
                imm32: -4, // Branch back (negative offset)
            },
            thumb32: false,
        };

        // Before the branch, we should still be in IT block
        assert!(core.in_it_block(), "Should be in IT block before branch");

        core.execute(&b, instruction_size(&b));

        // After the branch executes, IT state should be cleared!
        assert!(
            !core.in_it_block(),
            "IT state should be cleared after the last instruction in IT block (the branch)"
        );

        // Now execute a SUBS instruction (SetFlags::NotInITBlock)
        // This should set flags because we're no longer in an IT block
        let sub_inst = Instruction::SUB_imm {
            params: Reg2ImmParams {
                rd: Reg::R3,
                rn: Reg::R3,
                imm32: 4,
                setflags: SetFlags::NotInITBlock,
            },
            thumb32: false,
        };

        // Clear flags first to make the test clear
        core.psr.set_n(0);
        core.psr.set_z(0);

        core.execute(&sub_inst, instruction_size(&sub_inst));

        // R3 should be 4 (8 - 4)
        assert_eq!(core.get_r(Reg::R3), 0x0000_0004);

        // Flags should have been set by SUBS (since we're not in IT block)
        // With result = 4, N should be 0 and Z should be 0
        assert!(
            !core.psr.get_n(),
            "N flag should be clear when result is positive"
        );
        assert!(
            !core.psr.get_z(),
            "Z flag should be clear when result is non-zero"
        );
    }

    #[test]
    fn test_resolve_addressing_wraps_on_add_overflow() {
        assert_eq!(resolve_addressing(u32::MAX, 4, true, true), (3, 3));
    }

    #[test]
    fn test_resolve_addressing_wraps_on_sub_underflow() {
        assert_eq!(
            resolve_addressing(1, 4, false, true),
            (u32::MAX - 2, u32::MAX - 2)
        );
    }
}
