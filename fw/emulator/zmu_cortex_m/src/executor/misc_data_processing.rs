use crate::Processor;

use crate::executor::{ExecuteSuccess, ExecutorHelper};

use super::ExecuteResult;

use crate::core::{
    bits::Bits,
    instruction::{BfcParams, BfiParams, BfxParams, MovtParams, Reg2RdRmParams},
    operation::sign_extend,
    register::BaseReg,
};
#[cfg(feature = "has-dsp-ext")]
use crate::core::{instruction::Reg3NoSetFlagsParams, register::Apsr};

/// Branching operations
pub trait IsaMiscDataProcessing {
    fn exec_bfc(&mut self, params: &BfcParams) -> ExecuteResult;
    fn exec_bfi(&mut self, params: &BfiParams) -> ExecuteResult;

    fn exec_movt(&mut self, params: MovtParams) -> ExecuteResult;

    fn exec_clz(&mut self, params: Reg2RdRmParams) -> ExecuteResult;
    fn exec_rev(&mut self, params: Reg2RdRmParams) -> ExecuteResult;
    fn exec_rev16(&mut self, params: Reg2RdRmParams) -> ExecuteResult;
    fn exec_revsh(&mut self, params: Reg2RdRmParams) -> ExecuteResult;
    #[cfg(feature = "has-dsp-ext")]
    fn exec_sel(&mut self, params: &Reg3NoSetFlagsParams) -> ExecuteResult;
    fn exec_ubfx(&mut self, params: &BfxParams) -> ExecuteResult;
    fn exec_sbfx(&mut self, params: &BfxParams) -> ExecuteResult;
}

impl IsaMiscDataProcessing for Processor {
    fn exec_bfc(&mut self, params: &BfcParams) -> ExecuteResult {
        if self.condition_passed() {
            if params.msbit >= params.lsbit {
                let destination_upper_range = params.msbit + 1;
                let mut result: u32 = self.get_r(params.rd);
                result.set_bits(params.lsbit..destination_upper_range, 0);
                self.set_r(params.rd, result);
            }
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_bfi(&mut self, params: &BfiParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn: u32 = self.get_r(params.rn);
            let rd = self.get_r(params.rd);

            let msbit = (params.lsbit + params.width) - 1;

            let source_upper_range = (msbit - params.lsbit) + 1;
            let destination_upper_range = msbit + 1;
            let mut result: u32 = rd;
            let value: u32 = rn.get_bits(0..source_upper_range);
            result.set_bits(params.lsbit..destination_upper_range, value);

            self.set_r(params.rd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_movt(&mut self, params: MovtParams) -> ExecuteResult {
        if self.condition_passed() {
            let mut result: u32 = self.get_r(params.rd);
            result.set_bits(16..32, (params.imm16).into());
            self.set_r(params.rd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }

        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_clz(&mut self, params: Reg2RdRmParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);

            self.set_r(params.rd, rm.leading_zeros());

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_rev(&mut self, params: Reg2RdRmParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            self.set_r(
                params.rd,
                ((rm & 0xff) << 24)
                    + ((rm & 0xff00) << 8)
                    + ((rm & 0xff_0000) >> 8)
                    + ((rm & 0xff00_0000) >> 24),
            );
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_rev16(&mut self, params: Reg2RdRmParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            self.set_r(
                params.rd,
                ((rm & 0xff) << 8)
                    + ((rm & 0xff00) >> 8)
                    + ((rm & 0xff_0000) << 8)
                    + ((rm & 0xff00_0000) >> 8),
            );
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_revsh(&mut self, params: Reg2RdRmParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            self.set_r(
                params.rd,
                ((sign_extend(rm & 0xff, 7, 24) as u32) << 8) + ((rm & 0xff00) >> 8),
            );
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    #[cfg(feature = "has-dsp-ext")]
    fn exec_sel(&mut self, params: &Reg3NoSetFlagsParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let rn = self.get_r(params.rn);

            let mut result = 0;
            result.set_bits(
                0..8,
                if self.psr.get_ge0() {
                    rn.get_bits(0..8)
                } else {
                    rm.get_bits(0..8)
                },
            );
            result.set_bits(
                8..16,
                if self.psr.get_ge1() {
                    rn.get_bits(8..16)
                } else {
                    rm.get_bits(8..16)
                },
            );
            result.set_bits(
                16..24,
                if self.psr.get_ge2() {
                    rn.get_bits(16..24)
                } else {
                    rm.get_bits(16..24)
                },
            );
            result.set_bits(
                24..32,
                if self.psr.get_ge3() {
                    rn.get_bits(24..32)
                } else {
                    rm.get_bits(24..32)
                },
            );
            self.set_r(params.rd, result);

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_ubfx(&mut self, params: &BfxParams) -> ExecuteResult {
        if self.condition_passed() {
            let msbit = params.lsb + params.widthminus1;
            if msbit <= 31 {
                let upper = msbit + 1;
                let data = self.get_r(params.rn).get_bits(params.lsb..upper);
                self.set_r(params.rd, data);
            } else {
                todo!();
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_sbfx(&mut self, params: &BfxParams) -> ExecuteResult {
        if self.condition_passed() {
            let msbit = params.lsb + params.widthminus1;
            if msbit <= 31 {
                let upper = msbit + 1;
                let data = self.get_r(params.rn).get_bits(params.lsb..upper);
                let data = sign_extend(data, msbit, 32) as i32;
                self.set_r(params.rd, data as u32);
            } else {
                todo!();
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
    #[test]
    fn test_bfi() {
        // arrange
        let mut core = Processor::new();
        core.psr.value = 0;

        core.set_r(Reg::R2, 0x1122_3344);
        core.set_r(Reg::R3, 0xaabb_ccdd);
        core.psr.value = 0;

        let instruction = Instruction::BFI {
            params: BfiParams {
                rd: Reg::R2,
                rn: Reg::R3,
                lsbit: 0,
                width: 8,
            },
        };

        core.execute_internal(&instruction).unwrap();

        assert_eq!(core.get_r(Reg::R3), 0xaabb_ccdd);
        assert_eq!(core.get_r(Reg::R2), 0x1122_33dd);
    }

    #[test]
    fn test_bfi_with_shift_8() {
        // arrange
        let mut core = Processor::new();
        core.psr.value = 0;

        core.set_r(Reg::R0, 0);
        core.set_r(Reg::R1, 0x00e0_00e4);

        let instruction = Instruction::BFI {
            params: BfiParams {
                rd: Reg::R0,
                rn: Reg::R1,
                lsbit: 8,
                width: 24,
            },
        };

        core.execute_internal(&instruction).unwrap();

        assert_eq!(core.get_r(Reg::R0), 0xe000_e400);
        assert_eq!(core.get_r(Reg::R1), 0x00e0_00e4);
    }

    #[cfg(not(feature = "has-dsp-ext"))]
    #[test]
    fn test_sel_without_dsp_is_undef() {
        use crate::core::fault::Fault;
        use crate::core::instruction::Reg3NoSetFlagsParams;
        let mut core = Processor::new();
        let instruction = Instruction::SEL {
            params: Reg3NoSetFlagsParams {
                rd: Reg::R0,
                rn: Reg::R1,
                rm: Reg::R2,
            },
        };
        let result = core.execute_internal(&instruction);
        assert_eq!(result, Err(Fault::UndefInstr));
    }
}
