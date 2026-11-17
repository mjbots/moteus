use crate::Processor;

use crate::executor::{ExecuteSuccess, ExecutorHelper};

use super::ExecuteResult;
use crate::core::instruction::Reg643232Params;
#[cfg(feature = "has-dsp-ext")]
use crate::core::instruction::{Reg3HighParams, Reg4HighParams};
#[cfg(feature = "has-dsp-ext")]
use crate::core::register::Apsr;
use crate::core::{bits::Bits, register::BaseReg};

/// Multiply operations
pub trait IsaSignedMultiply {
    fn exec_smlal(&mut self, params: &Reg643232Params) -> ExecuteResult;
    fn exec_smull(&mut self, params: &Reg643232Params) -> ExecuteResult;
    #[cfg(feature = "has-dsp-ext")]
    fn exec_smul(&mut self, params: &Reg3HighParams) -> ExecuteResult;
    #[cfg(feature = "has-dsp-ext")]
    fn exec_smla(&mut self, params: &Reg4HighParams) -> ExecuteResult;
}

impl IsaSignedMultiply for Processor {
    fn exec_smlal(&mut self, params: &Reg643232Params) -> ExecuteResult {
        if self.condition_passed() {
            let rn = i64::from(self.get_r(params.rn) as i32);
            let rm = i64::from(self.get_r(params.rm) as i32);

            let rdlo = u64::from(self.get_r(params.rdlo));
            let rdhi = u64::from(self.get_r(params.rdhi));
            let accumulator = ((rdhi << 32) | rdlo) as i64;

            let result = accumulator.wrapping_add(rn.wrapping_mul(rm));
            let result_bits = result as u64;

            self.set_r(params.rdlo, result_bits.get_bits(0..32) as u32);
            self.set_r(params.rdhi, result_bits.get_bits(32..64) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_smull(&mut self, params: &Reg643232Params) -> ExecuteResult {
        if self.condition_passed() {
            let rn = i64::from(self.get_r(params.rn));
            let rm = i64::from(self.get_r(params.rm));
            let result = rn.wrapping_mul(rm) as u64;

            self.set_r(params.rdlo, result.get_bits(0..32) as u32);
            self.set_r(params.rdhi, result.get_bits(32..64) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    #[cfg(feature = "has-dsp-ext")]
    fn exec_smul(&mut self, params: &Reg3HighParams) -> ExecuteResult {
        if self.condition_passed() {
            let operand1 = i32::from(if params.n_high {
                let op = self.get_r(params.rn).get_bits(16..32);
                op as i16
            } else {
                let op = self.get_r(params.rn).get_bits(0..16);
                op as i16
            });
            let operand2 = i32::from(if params.m_high {
                let op = self.get_r(params.rm).get_bits(16..32);
                op as i16
            } else {
                let op = self.get_r(params.rm).get_bits(0..16);
                op as i16
            });

            let result = operand1.wrapping_mul(operand2);

            self.set_r(params.rd, result as u32);

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    #[cfg(feature = "has-dsp-ext")]
    fn exec_smla(&mut self, params: &Reg4HighParams) -> ExecuteResult {
        if self.condition_passed() {
            let operand1 = i32::from(if params.n_high {
                let op: u32 = self.get_r(params.rn).get_bits(16..32);
                op as i16
            } else {
                let op: u32 = self.get_r(params.rn).get_bits(0..16);
                op as i16
            });
            let operand2 = i32::from(if params.m_high {
                let op: u32 = self.get_r(params.rm).get_bits(16..32);
                op as i16
            } else {
                let op: u32 = self.get_r(params.rm).get_bits(0..16);
                op as i16
            });

            let (mul_result, mul_overflow) = operand1.overflowing_mul(operand2);
            let (result, add_overflow) = mul_result.overflowing_add(self.get_r(params.ra) as i32);

            self.set_r(params.rd, result as u32);
            if mul_overflow || add_overflow {
                self.psr.set_q(true);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{instruction::Instruction, register::Reg};

    #[cfg(feature = "has-dsp-ext")]
    #[test]
    fn test_smlabb_no_overflow() {
        let mut core = Processor::new();
        core.psr.value = 0;

        core.set_r(Reg::R8, 0xffff_9d88);
        core.set_r(Reg::R12, 0x0012_dfc3);
        core.set_r(Reg::LR, 0xa1);

        let instruction = Instruction::SMLA {
            params: Reg4HighParams {
                rd: Reg::R12,
                rn: Reg::LR,
                rm: Reg::R8,
                ra: Reg::R12,
                n_high: false,
                m_high: false,
            },
        };

        core.execute_internal(&instruction).unwrap();

        assert_eq!(core.get_r(Reg::R12), 0xFFD4_F24B);
        assert!(!core.psr.get_q());
    }

    #[cfg(feature = "has-dsp-ext")]
    #[test]
    fn test_smla_positive_overflow() {
        let mut core = Processor::new();
        core.psr.value = 0;

        core.set_r(Reg::R0, 0x7fff_7fff);
        core.set_r(Reg::R1, 0x7fff_7fff);
        core.set_r(Reg::R2, 0x7fff_ffff);

        let instruction = Instruction::SMLA {
            params: Reg4HighParams {
                rd: Reg::R0,
                rn: Reg::R0,
                rm: Reg::R1,
                ra: Reg::R2,
                n_high: true,
                m_high: true,
            },
        };

        core.execute_internal(&instruction).unwrap();

        assert_eq!(core.get_r(Reg::R0), 0xBFFF_0000);
        assert!(core.psr.get_q());
    }

    #[cfg(feature = "has-dsp-ext")]
    #[test]
    fn test_smla_negative_overflow() {
        let mut core = Processor::new();
        core.psr.value = 0;

        core.set_r(Reg::R0, 0x8000_8000);
        core.set_r(Reg::R1, 0x8000_8000);
        core.set_r(Reg::R2, 0x4000_0000);

        let instruction = Instruction::SMLA {
            params: Reg4HighParams {
                rd: Reg::R0,
                rn: Reg::R0,
                rm: Reg::R1,
                ra: Reg::R2,
                n_high: true,
                m_high: true,
            },
        };

        core.execute_internal(&instruction).unwrap();

        assert_eq!(core.get_r(Reg::R0), 0x8000_0000);
        assert!(core.psr.get_q());
    }

    #[cfg(feature = "has-dsp-ext")]
    #[test]
    fn test_smla_low_16bits() {
        let mut core = Processor::new();
        core.psr.value = 0;

        core.set_r(Reg::R0, 0x1111_1000);
        core.set_r(Reg::R1, 0x2222_0800);
        core.set_r(Reg::R2, 0x0000_0000);

        let instruction = Instruction::SMLA {
            params: Reg4HighParams {
                rd: Reg::R3,
                rn: Reg::R0,
                rm: Reg::R1,
                ra: Reg::R2,
                n_high: false,
                m_high: false,
            },
        };

        core.execute_internal(&instruction).unwrap();

        assert_eq!(core.get_r(Reg::R3), 0x80_0000);
        assert!(!core.psr.get_q());
    }

    #[cfg(feature = "has-dsp-ext")]
    #[test]
    fn test_smla_mixed_high_low() {
        let mut core = Processor::new();
        core.psr.value = 0;

        core.set_r(Reg::R0, 0xffff_1000);
        core.set_r(Reg::R1, 0x0800_0800);
        core.set_r(Reg::R2, 0x0000_1000);

        let instruction = Instruction::SMLA {
            params: Reg4HighParams {
                rd: Reg::R3,
                rn: Reg::R0,
                rm: Reg::R1,
                ra: Reg::R2,
                n_high: true,
                m_high: false,
            },
        };

        core.execute_internal(&instruction).unwrap();

        assert_eq!(core.get_r(Reg::R3), 0x0000_0800);
        assert!(!core.psr.get_q());
    }

    #[cfg(feature = "has-dsp-ext")]
    #[test]
    fn test_smla_q_flag_persistence() {
        let mut core = Processor::new();
        core.psr.value = 0;

        core.set_r(Reg::R0, 0x7fff_7fff);
        core.set_r(Reg::R1, 0x7fff_7fff);
        core.set_r(Reg::R2, 0x7fff_ffff);

        let instruction1 = Instruction::SMLA {
            params: Reg4HighParams {
                rd: Reg::R3,
                rn: Reg::R0,
                rm: Reg::R1,
                ra: Reg::R2,
                n_high: true,
                m_high: true,
            },
        };

        core.execute_internal(&instruction1).unwrap();
        assert_eq!(core.get_r(Reg::R3), 0xBFFF_0000);
        assert!(core.psr.get_q());
        core.set_r(Reg::R4, 0x0001_0001);
        core.set_r(Reg::R5, 0x0002_0002);
        core.set_r(Reg::R6, 0x0000_0000);

        let instruction2 = Instruction::SMLA {
            params: Reg4HighParams {
                rd: Reg::R7,
                rn: Reg::R4,
                rm: Reg::R5,
                ra: Reg::R6,
                n_high: false,
                m_high: false,
            },
        };

        core.execute_internal(&instruction2).unwrap();
        assert_eq!(core.get_r(Reg::R7), 0x0000_0002);
        assert!(core.psr.get_q());
    }

    #[test]
    fn test_smlal() {
        let mut core = Processor::new();
        core.psr.value = 0;

        core.set_r(Reg::R0, 0x0000_0003);
        core.set_r(Reg::R1, 0xFFFF_FFFC);
        core.set_r(Reg::R2, 0x0000_0005);
        core.set_r(Reg::R3, 0x0000_0000);

        let instruction = Instruction::SMLAL {
            params: Reg643232Params {
                rm: Reg::R1,
                rdlo: Reg::R2,
                rdhi: Reg::R3,
                rn: Reg::R0,
            },
        };

        core.execute_internal(&instruction).unwrap();

        assert_eq!(core.get_r(Reg::R2), 0xFFFF_FFF9);
        assert_eq!(core.get_r(Reg::R3), 0xFFFF_FFFF);
    }

    #[cfg(feature = "has-dsp-ext")]
    #[test]
    fn test_smla() {
        let mut core = Processor::new();
        core.psr.value = 0;

        core.set_r(Reg::R8, 0xffff_9d88);
        core.set_r(Reg::R12, 0x0012_dfc3);
        core.set_r(Reg::LR, 0xa1);

        let instruction = Instruction::SMLA {
            params: Reg4HighParams {
                rd: Reg::R12,
                rn: Reg::LR,
                rm: Reg::R8,
                ra: Reg::R12,
                n_high: false,
                m_high: false,
            },
        };

        core.execute_internal(&instruction).unwrap();

        assert_eq!(core.get_r(Reg::R12), 0xFFD4_F24B);
    }

    #[cfg(not(feature = "has-dsp-ext"))]
    #[test]
    fn test_smla_without_dsp_is_undef() {
        use crate::core::fault::Fault;
        use crate::core::instruction::Reg4HighParams;
        let mut core = Processor::new();
        let instruction = Instruction::SMLA {
            params: Reg4HighParams {
                rd: Reg::R0,
                rn: Reg::R1,
                rm: Reg::R2,
                ra: Reg::R3,
                n_high: false,
                m_high: false,
            },
        };
        let result = core.execute_internal(&instruction);
        assert_eq!(result, Err(Fault::UndefInstr));
    }

    #[cfg(not(feature = "has-dsp-ext"))]
    #[test]
    fn test_smul_without_dsp_is_undef() {
        use crate::core::fault::Fault;
        use crate::core::instruction::Reg3HighParams;
        let mut core = Processor::new();
        let instruction = Instruction::SMUL {
            params: Reg3HighParams {
                rd: Reg::R0,
                rn: Reg::R1,
                rm: Reg::R2,
                n_high: false,
                m_high: false,
            },
        };
        let result = core.execute_internal(&instruction);
        assert_eq!(result, Err(Fault::UndefInstr));
    }
}
