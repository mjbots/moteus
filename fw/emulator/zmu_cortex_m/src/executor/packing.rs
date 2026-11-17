use crate::Processor;

use crate::executor::{ExecuteSuccess, ExecutorHelper};

use super::ExecuteResult;
use crate::core::instruction::{Reg2UsizeParams, Reg3UsizeParams};
use crate::core::{
    bits::Bits,
    operation::{ror, sign_extend},
    register::BaseReg,
};

/// Multiply operations
pub trait IsaPacking {
    fn exec_sxtb(&mut self, params: &Reg2UsizeParams) -> ExecuteResult;
    fn exec_sxth(&mut self, params: &Reg2UsizeParams) -> ExecuteResult;
    fn exec_uxtb(&mut self, params: &Reg2UsizeParams) -> ExecuteResult;
    fn exec_uxth(&mut self, params: &Reg2UsizeParams) -> ExecuteResult;
    fn exec_uxtab(&mut self, params: &Reg3UsizeParams) -> ExecuteResult;
}

impl IsaPacking for Processor {
    fn exec_sxtb(&mut self, params: &Reg2UsizeParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let rotated = ror(rm, params.rotation);
            let result = sign_extend(rotated.get_bits(0..8), 7, 32) as u32;
            self.set_r(params.rd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_sxth(&mut self, params: &Reg2UsizeParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let rotated = ror(rm, params.rotation);
            let result = sign_extend(rotated.get_bits(0..16), 15, 32) as u32;
            self.set_r(params.rd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_uxtb(&mut self, params: &Reg2UsizeParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let rotated = ror(rm, params.rotation);
            self.set_r(params.rd, rotated.get_bits(0..8));
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_uxth(&mut self, params: &Reg2UsizeParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let rotated = ror(rm, params.rotation);
            self.set_r(params.rd, rotated.get_bits(0..16));
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_uxtab(&mut self, params: &Reg3UsizeParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let rotated = ror(rm, params.rotation);
            let rn = self.get_r(params.rn);
            let result = rn.wrapping_add(rotated.get_bits(0..8));
            self.set_r(params.rd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
}
