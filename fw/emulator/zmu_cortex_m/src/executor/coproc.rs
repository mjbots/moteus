use crate::Processor;

use crate::executor::ExecuteSuccess;

use super::ExecuteResult;

/// Branching operations
pub trait IsaCoprocessor {
    fn exec_mcr(&self) -> ExecuteResult;
    fn exec_mcr2(&self) -> ExecuteResult;
    fn exec_ldc_imm(&self) -> ExecuteResult;
    fn exec_ldc2_imm(&self) -> ExecuteResult;
}

impl IsaCoprocessor for Processor {
    fn exec_mcr(&self) -> ExecuteResult {
        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }
    fn exec_mcr2(&self) -> ExecuteResult {
        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }
    fn exec_ldc_imm(&self) -> ExecuteResult {
        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }
    fn exec_ldc2_imm(&self) -> ExecuteResult {
        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }
}
