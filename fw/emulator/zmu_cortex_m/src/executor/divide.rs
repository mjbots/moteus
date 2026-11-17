use crate::Processor;

use crate::executor::{ExecuteSuccess, ExecutorHelper};

use super::ExecuteResult;
use crate::core::instruction::Reg3NoSetFlagsParams;
use crate::core::{fault::Fault, register::BaseReg};

/// Divide operations
pub trait IsaDivide {
    fn exec_udiv(&mut self, params: &Reg3NoSetFlagsParams) -> ExecuteResult;
    fn exec_sdiv(&mut self, params: &Reg3NoSetFlagsParams) -> ExecuteResult;
}

impl IsaDivide for Processor {
    fn exec_sdiv(&mut self, params: &Reg3NoSetFlagsParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let result = if rm == 0 {
                if self.integer_zero_divide_trapping_enabled() {
                    return Err(Fault::DivByZero);
                }
                0
            } else {
                let rn = self.get_r(params.rn);
                (rn as i32) / (rm as i32)
            };
            self.set_r(params.rd, result as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_udiv(&mut self, params: &Reg3NoSetFlagsParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let result = if rm == 0 {
                if self.integer_zero_divide_trapping_enabled() {
                    return Err(Fault::DivByZero);
                }
                0
            } else {
                let rn = self.get_r(params.rn);
                rn / rm
            };
            self.set_r(params.rd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{instruction::Instruction, register::Reg};

    #[test]
    fn test_udiv() {
        // arrange
        let mut core = Processor::new();
        core.set_r(Reg::R0, 0x7d0);
        core.set_r(Reg::R1, 0x3);
        core.psr.value = 0;

        let instruction = Instruction::UDIV {
            params: Reg3NoSetFlagsParams {
                rd: Reg::R0,
                rn: Reg::R0,
                rm: Reg::R1,
            },
        };

        // act
        let result = core.execute_internal(&instruction);

        assert_eq!(result, Ok(ExecuteSuccess::Taken { cycles: 2 }));

        assert_eq!(core.get_r(Reg::R0), 0x29a);
        assert_eq!(core.get_r(Reg::R1), 0x3);
    }
}
