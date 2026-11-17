#[cfg(feature = "has-dsp-ext")]
use crate::Processor;

#[cfg(feature = "has-dsp-ext")]
use crate::executor::{ExecuteSuccess, ExecutorHelper};

#[cfg(feature = "has-dsp-ext")]
use super::ExecuteResult;
#[cfg(feature = "has-dsp-ext")]
use crate::core::instruction::Reg3NoSetFlagsParams;
#[cfg(feature = "has-dsp-ext")]
use crate::core::{
    bits::Bits,
    register::{Apsr, BaseReg},
};

/// Parallel add/subtract operations (DSP extension)
#[cfg(feature = "has-dsp-ext")]
pub trait IsaParallelAddSub {
    fn exec_uadd8(&mut self, params: &Reg3NoSetFlagsParams) -> ExecuteResult;
}

#[cfg(feature = "has-dsp-ext")]
impl IsaParallelAddSub for Processor {
    fn exec_uadd8(&mut self, params: &Reg3NoSetFlagsParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm: u32 = self.get_r(params.rm);
            let rn: u32 = self.get_r(params.rn);

            let sum1: u32 = rn.get_bits(0..8) + rm.get_bits(0..8);
            let sum2: u32 = rn.get_bits(8..16) + rm.get_bits(8..16);
            let sum3: u32 = rn.get_bits(16..24) + rm.get_bits(16..24);
            let sum4: u32 = rn.get_bits(24..32) + rm.get_bits(24..32);

            let mut result: u32 = sum1.get_bits(0..8);
            result.set_bits(8..16, sum2.get_bits(0..8));
            result.set_bits(16..24, sum3.get_bits(0..8));
            result.set_bits(24..32, sum4.get_bits(0..8));
            self.set_r(params.rd, result);

            self.psr.set_ge0(sum1 >= 0x100);
            self.psr.set_ge1(sum2 >= 0x100);
            self.psr.set_ge2(sum3 >= 0x100);
            self.psr.set_ge3(sum4 >= 0x100);

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
}

#[cfg(all(test, not(feature = "has-dsp-ext")))]
mod tests {
    use crate::Processor;
    use crate::core::{instruction::Instruction, register::Reg};
    use crate::executor::ExecutorHelper;

    #[cfg(not(feature = "has-dsp-ext"))]
    #[test]
    fn test_uadd8_without_dsp_is_undef() {
        use crate::core::fault::Fault;
        use crate::core::instruction::Reg3NoSetFlagsParams;
        let mut core = Processor::new();
        let instruction = Instruction::UADD8 {
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
