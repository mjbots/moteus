use crate::Processor;

use crate::executor::{ExecuteSuccess, ExecutorHelper};

use super::ExecuteResult;

use crate::{
    bus::Bus,
    core::{
        instruction::{CondBranchParams, ParamsRegImm32, Reg2RnRmParams},
        register::{BaseReg, Reg},
    },
};

/// Branching operations
pub trait IsaBranch {
    fn exec_bl(&mut self, imm32: i32) -> ExecuteResult;
    fn exec_bx(&mut self, rm: Reg) -> ExecuteResult;
    fn exec_blx(&mut self, rm: Reg) -> ExecuteResult;

    fn exec_b_t13(&mut self, params: CondBranchParams) -> ExecuteResult;
    fn exec_b_t24(&mut self, imm32: i32) -> ExecuteResult;

    fn exec_tbb(&mut self, params: Reg2RnRmParams) -> ExecuteResult;
    fn exec_tbh(&mut self, params: Reg2RnRmParams) -> ExecuteResult;

    fn exec_cbz(&mut self, params: ParamsRegImm32) -> ExecuteResult;
    fn exec_cbnz(&mut self, params: ParamsRegImm32) -> ExecuteResult;
}

impl IsaBranch for Processor {
    fn exec_bl(&mut self, imm32: i32) -> ExecuteResult {
        if self.condition_passed() {
            let pc = self.get_r(Reg::PC);
            self.set_r(Reg::LR, pc | 0x01);
            let target = ((pc as i32) + imm32) as u32;

            self.branch_write_pc(target);
            return Ok(ExecuteSuccess::Branched { cycles: 4 });
        }

        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_bx(&mut self, rm: Reg) -> ExecuteResult {
        if self.condition_passed() {
            let r_m = self.get_r(rm);
            self.bx_write_pc(r_m)?;
            return Ok(ExecuteSuccess::Branched { cycles: 3 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_blx(&mut self, rm: Reg) -> ExecuteResult {
        if self.condition_passed() {
            let pc = self.get_r(Reg::PC);
            let target = self.get_r(rm);
            self.set_r(Reg::LR, (((pc - 2) >> 1) << 1) | 1);
            self.blx_write_pc(target);
            return Ok(ExecuteSuccess::Branched { cycles: 3 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    #[inline(always)]
    fn exec_b_t13(&mut self, params: CondBranchParams) -> ExecuteResult {
        if self.condition_passed_b(params.cond) {
            let pc = self.get_r(Reg::PC);
            let target = ((pc as i32) + params.imm32) as u32;
            self.branch_write_pc(target);
            Ok(ExecuteSuccess::Branched { cycles: 3 })
        } else {
            Ok(ExecuteSuccess::NotTaken)
        }
    }

    fn exec_b_t24(&mut self, imm32: i32) -> ExecuteResult {
        if self.condition_passed() {
            let pc = self.get_r(Reg::PC);
            let target = ((pc as i32) + imm32) as u32;
            self.branch_write_pc(target);
            Ok(ExecuteSuccess::Branched { cycles: 3 })
        } else {
            Ok(ExecuteSuccess::NotTaken)
        }
    }

    fn exec_tbb(&mut self, params: Reg2RnRmParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let pc = self.get_r(Reg::PC);
            let halfwords = u32::from(self.read8(rn + rm)?);

            self.branch_write_pc(pc + 2 * halfwords);

            return Ok(ExecuteSuccess::Branched { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_tbh(&mut self, params: Reg2RnRmParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let rm = self.get_r(params.rm);
            let pc = self.get_r(Reg::PC);
            let halfwords = u32::from(self.read16(rn + (rm << 1))?);

            self.branch_write_pc(pc + 2 * halfwords);

            return Ok(ExecuteSuccess::Branched { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_cbz(&mut self, params: ParamsRegImm32) -> ExecuteResult {
        let rn = self.get_r(params.rn);
        if rn == 0 {
            let pc = self.get_r(Reg::PC);
            self.branch_write_pc(pc + params.imm32);
            Ok(ExecuteSuccess::Branched { cycles: 1 })
        } else {
            Ok(ExecuteSuccess::Taken { cycles: 1 })
        }
    }

    fn exec_cbnz(&mut self, params: ParamsRegImm32) -> ExecuteResult {
        let rn = self.get_r(params.rn);
        if rn == 0 {
            Ok(ExecuteSuccess::Taken { cycles: 1 })
        } else {
            let pc = self.get_r(Reg::PC);
            self.branch_write_pc(pc + params.imm32);
            Ok(ExecuteSuccess::Branched { cycles: 1 })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{condition::Condition, instruction::Instruction};

    #[test]
    fn test_b_cond() {
        // arrange
        let mut core = Processor::new();
        core.psr.value = 0;

        let instruction = Instruction::B_t13 {
            params: CondBranchParams {
                cond: Condition::EQ,
                imm32: 0,
            },
            thumb32: true,
        };

        // act
        let result = core.execute_internal(&instruction);

        assert_eq!(result, Ok(ExecuteSuccess::NotTaken));
    }
}
