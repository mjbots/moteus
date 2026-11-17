use crate::Processor;

use crate::core::operation::shift_c;
use crate::executor::{ExecuteSuccess, ExecutorHelper};

use super::{ExecuteResult, conditional_setflags};
use crate::core::instruction::{Reg2Params, Reg2ShiftNParams, Reg3Params, SRType};
use crate::core::{
    bits::Bits,
    register::{Apsr, BaseReg},
};

/// Shift operations
pub trait IsaShift {
    fn exec_asr_imm(&mut self, params: &Reg2ShiftNParams) -> ExecuteResult;
    fn exec_asr_reg(&mut self, params: &Reg3Params) -> ExecuteResult;

    fn exec_lsl_imm(&mut self, params: &Reg2ShiftNParams) -> ExecuteResult;
    fn exec_lsl_reg(&mut self, params: &Reg3Params) -> ExecuteResult;

    fn exec_lsr_imm(&mut self, params: &Reg2ShiftNParams) -> ExecuteResult;
    fn exec_lsr_reg(&mut self, params: &Reg3Params) -> ExecuteResult;

    fn exec_ror_imm(&mut self, params: &Reg2ShiftNParams) -> ExecuteResult;
    fn exec_ror_reg(&mut self, params: &Reg3Params) -> ExecuteResult;

    fn exec_rrx(&mut self, params: &Reg2Params) -> ExecuteResult;
}

impl IsaShift for Processor {
    fn exec_asr_imm(&mut self, params: &Reg2ShiftNParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();
            let (result, carry) = shift_c(rm, SRType::ASR, usize::from(params.shift_n), c);

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

    fn exec_asr_reg(&mut self, params: &Reg3Params) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let rn = self.get_r(params.rn);
            let shift_n: u32 = rm.get_bits(0..8);
            let c = self.psr.get_c();
            let (result, carry) = shift_c(rn, SRType::ASR, shift_n as usize, c);
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

    fn exec_lsl_imm(&mut self, params: &Reg2ShiftNParams) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let rm = self.get_r(params.rm);
            let (result, carry) = shift_c(rm, SRType::LSL, params.shift_n as usize, c);
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

    fn exec_lsl_reg(&mut self, params: &Reg3Params) -> ExecuteResult {
        if self.condition_passed() {
            let shift_n: u32 = self.get_r(params.rm).get_bits(0..8);
            let c = self.psr.get_c();
            let (result, carry) = shift_c(self.get_r(params.rn), SRType::LSL, shift_n as usize, c);
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

    fn exec_lsr_imm(&mut self, params: &Reg2ShiftNParams) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let rm = self.get_r(params.rm);
            let (result, carry) = shift_c(rm, SRType::LSR, usize::from(params.shift_n), c);
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

    fn exec_lsr_reg(&mut self, params: &Reg3Params) -> ExecuteResult {
        if self.condition_passed() {
            let shift_n: u32 = self.get_r(params.rm).get_bits(0..8);
            let c = self.psr.get_c();
            let (result, carry) = shift_c(self.get_r(params.rn), SRType::LSR, shift_n as usize, c);

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

    fn exec_ror_imm(&mut self, params: &Reg2ShiftNParams) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let rm = self.get_r(params.rm);
            let (result, carry) = shift_c(rm, SRType::ROR, usize::from(params.shift_n), c);

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

    fn exec_ror_reg(&mut self, params: &Reg3Params) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let shift_n = self.get_r(params.rm) & 0xff;
            let (result, carry) = shift_c(self.get_r(params.rn), SRType::ROR, shift_n as usize, c);
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

    fn exec_rrx(&mut self, params: &Reg2Params) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let (result, carry) = shift_c(self.get_r(params.rm), SRType::RRX, 1, c);
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
}
