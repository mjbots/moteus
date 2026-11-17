use crate::Processor;

use crate::{
    core::{condition::Condition, exception::ExceptionHandling},
    executor::{ExecuteSuccess, ExecutorHelper},
};

use super::ExecuteResult;

/// Branching operations
pub trait IsaMisc {
    fn exec_dmb(&self) -> ExecuteResult;
    fn exec_dsb(&self) -> ExecuteResult;
    fn exec_isb(&self) -> ExecuteResult;
    fn exec_it(&mut self, firstcond: Condition, mask: u8) -> ExecuteResult;
    fn exec_pld_imm(&self) -> ExecuteResult;
    fn exec_pld_lit(&self) -> ExecuteResult;
    fn exec_pld_reg(&self) -> ExecuteResult;
    fn exec_sev(&self) -> ExecuteResult;
    fn exec_wfe(&self) -> ExecuteResult;
    fn exec_yield(&self) -> ExecuteResult;
    fn exec_wfi(&mut self) -> ExecuteResult;
}

impl IsaMisc for Processor {
    fn exec_dmb(&self) -> ExecuteResult {
        if self.condition_passed() {
            return Ok(ExecuteSuccess::Taken { cycles: 4 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_dsb(&self) -> ExecuteResult {
        if self.condition_passed() {
            return Ok(ExecuteSuccess::Taken { cycles: 4 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_isb(&self) -> ExecuteResult {
        if self.condition_passed() {
            return Ok(ExecuteSuccess::Taken { cycles: 4 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_it(&mut self, firstcond: Condition, mask: u8) -> ExecuteResult {
        let itstate = (((firstcond.value() as u32) << 4) + u32::from(mask)) as u8;
        self.set_itstate(itstate);
        Ok(ExecuteSuccess::Taken { cycles: 4 })
    }

    fn exec_pld_imm(&self) -> ExecuteResult {
        if self.condition_passed() {
            Ok(ExecuteSuccess::Taken { cycles: 1 })
        } else {
            Ok(ExecuteSuccess::NotTaken)
        }
    }

    fn exec_pld_lit(&self) -> ExecuteResult {
        if self.condition_passed() {
            Ok(ExecuteSuccess::Taken { cycles: 1 })
        } else {
            Ok(ExecuteSuccess::NotTaken)
        }
    }

    fn exec_pld_reg(&self) -> ExecuteResult {
        if self.condition_passed() {
            Ok(ExecuteSuccess::Taken { cycles: 1 })
        } else {
            Ok(ExecuteSuccess::NotTaken)
        }
    }

    fn exec_sev(&self) -> ExecuteResult {
        if self.condition_passed() {
            Ok(ExecuteSuccess::Taken { cycles: 1 })
        } else {
            Ok(ExecuteSuccess::NotTaken)
        }
    }

    fn exec_wfe(&self) -> ExecuteResult {
        if self.condition_passed() {
            Ok(ExecuteSuccess::Taken { cycles: 1 })
        } else {
            Ok(ExecuteSuccess::NotTaken)
        }
    }

    fn exec_yield(&self) -> ExecuteResult {
        if self.condition_passed() {
            Ok(ExecuteSuccess::Taken { cycles: 1 })
        } else {
            Ok(ExecuteSuccess::NotTaken)
        }
    }

    fn exec_wfi(&mut self) -> ExecuteResult {
        if self.condition_passed() {
            if self.get_pending_exception().is_none() && !self.has_wakeup_condition() {
                self.sleeping = true;
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
}
