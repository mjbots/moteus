use crate::Processor;

use crate::core::operation::{add_with_carry, shift, shift_c};

use crate::executor::{ExecuteSuccess, ExecutorHelper};

use super::{ExecuteResult, conditional_setflags, expand_conditional_carry};
use crate::core::instruction::{
    Reg2ImmCarryParams, Reg2ImmParams, Reg2Params, Reg2ShiftNoSetFlagsParams, Reg2ShiftParams,
    Reg3ShiftParams, RegImmCarryNoSetFlagsParams, RegImmCarryParams, RegImmParams,
};
use crate::core::register::{Apsr, BaseReg, Reg};

/// Standard data processing instruction support
pub trait IsaStandardDataProcessing {
    fn exec_add_imm(&mut self, params: &Reg2ImmParams) -> ExecuteResult;
    fn exec_add_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult;
    fn exec_add_sp_reg(&mut self, params: &Reg2ShiftParams) -> ExecuteResult;

    fn exec_adc_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult;
    fn exec_adc_imm(&mut self, params: &Reg2ImmParams) -> ExecuteResult;

    fn exec_adr(&mut self, params: RegImmParams) -> ExecuteResult;

    fn exec_and_imm(&mut self, params: &Reg2ImmCarryParams) -> ExecuteResult;
    fn exec_and_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult;

    fn exec_bic_imm(&mut self, params: &Reg2ImmCarryParams) -> ExecuteResult;
    fn exec_bic_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult;

    fn exec_cmn_reg(&mut self, params: &Reg2ShiftNoSetFlagsParams) -> ExecuteResult;
    fn exec_cmn_imm(&mut self, params: RegImmParams) -> ExecuteResult;

    fn exec_cmp_imm(&mut self, params: RegImmParams) -> ExecuteResult;
    fn exec_cmp_reg(&mut self, params: &Reg2ShiftNoSetFlagsParams) -> ExecuteResult;

    fn exec_eor_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult;
    fn exec_eor_imm(&mut self, params: &Reg2ImmCarryParams) -> ExecuteResult;

    fn exec_mov_reg(&mut self, params: &Reg2Params) -> ExecuteResult;
    fn exec_mov_imm(&mut self, params: &RegImmCarryParams) -> ExecuteResult;

    fn exec_mvn_reg(&mut self, params: &Reg2ShiftParams) -> ExecuteResult;
    fn exec_mvn_imm(&mut self, params: &RegImmCarryParams) -> ExecuteResult;

    fn exec_orn_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult;

    fn exec_orr_imm(&mut self, params: &Reg2ImmCarryParams) -> ExecuteResult;
    fn exec_orr_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult;

    fn exec_rsb_imm(&mut self, params: &Reg2ImmParams) -> ExecuteResult;
    fn exec_rsb_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult;

    fn exec_sbc_imm(&mut self, params: &Reg2ImmParams) -> ExecuteResult;
    fn exec_sbc_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult;

    fn exec_sub_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult;
    fn exec_sub_imm(&mut self, params: &Reg2ImmParams) -> ExecuteResult;

    fn exec_teq_reg(&mut self, params: &Reg2ShiftNoSetFlagsParams) -> ExecuteResult;
    fn exec_teq_imm(&mut self, params: &RegImmCarryNoSetFlagsParams) -> ExecuteResult;

    fn exec_tst_reg(&mut self, params: &Reg2ShiftNoSetFlagsParams) -> ExecuteResult;
    fn exec_tst_imm(&mut self, params: &RegImmCarryNoSetFlagsParams) -> ExecuteResult;
}

impl IsaStandardDataProcessing for Processor {
    #[inline(always)]
    fn exec_add_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let shifted = shift(rm, params.shift_t, params.shift_n as usize, c);
            let (result, carry, overflow) = add_with_carry(rn, shifted, false);

            if params.rd == Reg::PC {
                self.branch_write_pc(result);
                Ok(ExecuteSuccess::Branched { cycles: 3 })
            } else {
                self.update_flags_check_it_block(params.setflags, result, carry, overflow);
                self.set_r(params.rd, result);
                Ok(ExecuteSuccess::Taken { cycles: 1 })
            }
        } else {
            Ok(ExecuteSuccess::NotTaken)
        }
    }
    fn exec_add_sp_reg(&mut self, params: &Reg2ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let rn = self.get_r(Reg::SP);
            let rm = self.get_r(params.rm);

            let shifted = shift(rm, params.shift_t, params.shift_n as usize, c);
            let (result, carry, overflow) = add_with_carry(rn, shifted, false);

            if params.rd == Reg::PC {
                self.branch_write_pc(result);
                Ok(ExecuteSuccess::Branched { cycles: 3 })
            } else {
                if conditional_setflags(params.setflags, self.in_it_block()) {
                    self.psr.set_n(result);
                    self.psr.set_z(result);
                    self.psr.set_c(carry);
                    self.psr.set_v(overflow);
                }
                self.set_r(params.rd, result);
                Ok(ExecuteSuccess::Taken { cycles: 1 })
            }
        } else {
            Ok(ExecuteSuccess::NotTaken)
        }
    }

    #[inline(always)]
    fn exec_add_imm(&mut self, params: &Reg2ImmParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let (result, carry, overflow) = add_with_carry(rn, params.imm32, false);

            self.update_flags_check_it_block(params.setflags, result, carry, overflow);
            self.set_r(params.rd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_adc_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();
            let shifted = shift(rm, params.shift_t, params.shift_n as usize, c);

            let (result, carry, overflow) = add_with_carry(rn, shifted, c);
            self.set_r(params.rd, result);

            self.update_flags_check_it_block(params.setflags, result, carry, overflow);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_adc_imm(&mut self, params: &Reg2ImmParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let c = self.psr.get_c();
            let (result, carry, overflow) = add_with_carry(rn, params.imm32, c);

            self.set_r(params.rd, result);
            self.update_flags_check_it_block(params.setflags, result, carry, overflow);

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_adr(&mut self, params: RegImmParams) -> ExecuteResult {
        if self.condition_passed() {
            let result = (self.get_r(Reg::PC) & 0xffff_fffc) + params.imm32;
            self.set_r(params.r, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_and_imm(&mut self, params: &Reg2ImmCarryParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let c = self.psr.get_c();
            let (im, carry) = expand_conditional_carry(&params.imm32, c);

            let result = rn & im;

            self.set_r(params.rd, result);

            if params.setflags {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    #[inline(always)]
    fn exec_and_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);

            let (shifted, carry) = shift_c(rm, params.shift_t, params.shift_n as usize, c);

            let result = rn & shifted;

            self.set_r(params.rd, result);

            if conditional_setflags(params.setflags, self.in_it_block()) {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_bic_imm(&mut self, params: &Reg2ImmCarryParams) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let rn = self.get_r(params.rn);
            let (im, carry) = expand_conditional_carry(&params.imm32, c);

            let result = rn & (im ^ 0xffff_ffff);
            self.set_r(params.rd, result);

            if params.setflags {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_bic_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();

            let (shifted, carry) = shift_c(rm, params.shift_t, params.shift_n as usize, c);

            let result = rn & (shifted ^ 0xffff_ffff);
            self.set_r(params.rd, result);

            if conditional_setflags(params.setflags, self.in_it_block()) {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_cmn_reg(&mut self, params: &Reg2ShiftNoSetFlagsParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();

            let shifted = shift(rm, params.shift_t, params.shift_n as usize, c);
            let (result, carry, overflow) = add_with_carry(rn, shifted, false);
            self.psr.set_n(result);
            self.psr.set_z(result);
            self.psr.set_c(carry);
            self.psr.set_v(overflow);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_cmn_imm(&mut self, params: RegImmParams) -> ExecuteResult {
        if self.condition_passed() {
            let (result, carry, overflow) =
                add_with_carry(self.get_r(params.r), params.imm32, false);
            self.psr.set_n(result);
            self.psr.set_z(result);
            self.psr.set_c(carry);
            self.psr.set_v(overflow);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    #[inline(always)]
    fn exec_cmp_reg(&mut self, params: &Reg2ShiftNoSetFlagsParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();

            let shifted = shift(rm, params.shift_t, params.shift_n as usize, c);
            let (result, carry, overflow) = add_with_carry(rn, shifted ^ 0xFFFF_FFFF, true);

            self.psr.set_n(result);
            self.psr.set_z(result);
            self.psr.set_c(carry);
            self.psr.set_v(overflow);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    #[inline(always)]
    fn exec_cmp_imm(&mut self, params: RegImmParams) -> ExecuteResult {
        if self.condition_passed() {
            let (result, carry, overflow) =
                add_with_carry(self.get_r(params.r), params.imm32 ^ 0xFFFF_FFFF, true);
            self.psr.set_n(result);
            self.psr.set_z(result);
            self.psr.set_c(carry);
            self.psr.set_v(overflow);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_eor_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();

            let (shifted, carry) = shift_c(rm, params.shift_t, params.shift_n as usize, c);

            let result = rn ^ shifted;

            self.set_r(params.rd, result);

            if conditional_setflags(params.setflags, self.in_it_block()) {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }

        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_eor_imm(&mut self, params: &Reg2ImmCarryParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let c = self.psr.get_c();
            let (im, carry) = expand_conditional_carry(&params.imm32, c);

            let result = rn ^ im;

            self.set_r(params.rd, result);

            if params.setflags {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    #[inline(always)]
    fn exec_mov_reg(&mut self, params: &Reg2Params) -> ExecuteResult {
        if self.condition_passed() {
            let result = self.get_r(params.rm);

            if params.rd == Reg::PC {
                self.branch_write_pc(result);
                return Ok(ExecuteSuccess::Branched { cycles: 3 });
            } else {
                self.set_r(params.rd, result);
                if conditional_setflags(params.setflags, self.in_it_block()) {
                    self.psr.set_n(result);
                    self.psr.set_z(result);
                }
                return Ok(ExecuteSuccess::Taken { cycles: 1 });
            }
        }

        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_mov_imm(&mut self, params: &RegImmCarryParams) -> ExecuteResult {
        if self.condition_passed() {
            let (result, carry) = expand_conditional_carry(&params.imm32, self.psr.get_c());
            self.set_r(params.rd, result);
            if conditional_setflags(params.setflags, self.in_it_block()) {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_mvn_reg(&mut self, params: &Reg2ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let (shifted, carry) = shift_c(
                self.get_r(params.rm),
                params.shift_t,
                params.shift_n as usize,
                self.psr.get_c(),
            );
            let result = shifted ^ 0xFFFF_FFFF;
            self.set_r(params.rd, result);

            if conditional_setflags(params.setflags, self.in_it_block()) {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_mvn_imm(&mut self, params: &RegImmCarryParams) -> ExecuteResult {
        if self.condition_passed() {
            let (im, carry) = expand_conditional_carry(&params.imm32, self.psr.get_c());
            let result = im ^ 0xFFFF_FFFF;
            self.set_r(params.rd, result);

            if conditional_setflags(params.setflags, self.in_it_block()) {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_orn_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();

            let (shifted, carry) = shift_c(rm, params.shift_t, params.shift_n as usize, c);
            let result = rn | (shifted ^ 0xFFFF_FFFF);

            self.set_r(params.rd, result);

            if conditional_setflags(params.setflags, self.in_it_block()) {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_orr_imm(&mut self, params: &Reg2ImmCarryParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let c = self.psr.get_c();
            let (im, carry) = expand_conditional_carry(&params.imm32, c);

            let result = rn | im;

            self.set_r(params.rd, result);

            if params.setflags {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    #[inline(always)]
    fn exec_orr_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();

            let (shifted, carry) = shift_c(rm, params.shift_t, params.shift_n as usize, c);
            let result = rn | shifted;

            self.set_r(params.rd, result);

            if conditional_setflags(params.setflags, self.in_it_block()) {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_rsb_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();

            let shifted = shift(rm, params.shift_t, params.shift_n as usize, c);
            let (result, carry, overflow) = add_with_carry(rn ^ 0xFFFF_FFFF, shifted, true);

            self.set_r(params.rd, result);

            if conditional_setflags(params.setflags, self.in_it_block()) {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
                self.psr.set_v(overflow);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_rsb_imm(&mut self, params: &Reg2ImmParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let (result, carry, overflow) = add_with_carry(rn ^ 0xFFFF_FFFF, params.imm32, true);

            self.update_flags_check_it_block(params.setflags, result, carry, overflow);

            self.set_r(params.rd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_sbc_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();

            let shifted = shift(rm, params.shift_t, params.shift_n as usize, c);

            let (result, carry, overflow) =
                add_with_carry(rn, shifted ^ 0xffff_ffff, self.psr.get_c());

            self.update_flags_check_it_block(params.setflags, result, carry, overflow);

            self.set_r(params.rd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_sbc_imm(&mut self, params: &Reg2ImmParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let c = self.psr.get_c();
            let (result, carry, overflow) = add_with_carry(rn, params.imm32 ^ 0xFFFF_FFFF, c);

            self.set_r(params.rd, result);

            if conditional_setflags(params.setflags, self.in_it_block()) {
                self.psr.set_n(result);
                self.psr.set_z(result);
                self.psr.set_c(carry);
                self.psr.set_v(overflow);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_sub_reg(&mut self, params: &Reg3ShiftParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();
            let shifted = shift(rm, params.shift_t, params.shift_n as usize, c);

            let (result, carry, overflow) = add_with_carry(rn, shifted ^ 0xFFFF_FFFF, true);
            self.set_r(params.rd, result);

            self.update_flags_check_it_block(params.setflags, result, carry, overflow);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_sub_imm(&mut self, params: &Reg2ImmParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let (result, carry, overflow) = add_with_carry(rn, params.imm32 ^ 0xFFFF_FFFF, true);

            self.update_flags_check_it_block(params.setflags, result, carry, overflow);

            self.set_r(params.rd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_teq_reg(&mut self, params: &Reg2ShiftNoSetFlagsParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();

            let (shifted, carry) = shift_c(rm, params.shift_t, params.shift_n as usize, c);
            let result = rn ^ shifted;

            self.psr.set_n(result);
            self.psr.set_z(result);
            self.psr.set_c(carry);

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_teq_imm(&mut self, params: &RegImmCarryNoSetFlagsParams) -> ExecuteResult {
        if self.condition_passed() {
            let (im, carry) = expand_conditional_carry(&params.imm32, self.psr.get_c());

            let result = self.get_r(params.rn) ^ im;

            self.psr.set_n(result);
            self.psr.set_z(result);
            self.psr.set_c(carry);

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_tst_reg(&mut self, params: &Reg2ShiftNoSetFlagsParams) -> ExecuteResult {
        if self.condition_passed() {
            let (shifted, carry) = shift_c(
                self.get_r(params.rm),
                params.shift_t,
                params.shift_n as usize,
                self.psr.get_c(),
            );

            let result = self.get_r(params.rn) & shifted;

            self.psr.set_n(result);
            self.psr.set_z(result);
            self.psr.set_c(carry);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_tst_imm(&mut self, params: &RegImmCarryNoSetFlagsParams) -> ExecuteResult {
        if self.condition_passed() {
            let (im, carry) = expand_conditional_carry(&params.imm32, self.psr.get_c());

            let result = self.get_r(params.rn) & im;

            self.psr.set_n(result);
            self.psr.set_z(result);
            self.psr.set_c(carry);

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{
        instruction::{Instruction, SRType, SetFlags},
        register::Reg,
    };

    #[test]
    fn test_sub() {
        // arrange
        let mut core = Processor::new();
        core.psr.value = 0;

        //3:418415f7 4:00000418 5:80000000 6:7d17d411
        core.set_r(Reg::R3, 0x4184_15f7);
        core.set_r(Reg::R4, 0x0000_0418);
        core.psr.value = 0;

        let instruction = Instruction::SUB_reg {
            params: Reg3ShiftParams {
                rd: Reg::R6,
                rn: Reg::R4,
                rm: Reg::R3,
                setflags: SetFlags::False,
                shift_t: SRType::LSR,
                shift_n: 20,
            },
            thumb32: true,
        };

        core.execute_internal(&instruction).unwrap();

        assert_eq!(core.get_r(Reg::R6), 0);
    }
}
