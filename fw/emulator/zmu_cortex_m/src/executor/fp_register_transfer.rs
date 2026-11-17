use crate::core::instruction::{
    VMRSTarget, VMovCr2DpParams, VMovCrSpParams, VMovImmParams32, VMovImmParams64,
    VMovRegParamsf32, VMovRegParamsf64,
};

use crate::Processor;
use crate::core::fpregister::Fpscr;
use crate::executor::ExecuteSuccess;
use crate::executor::ExecutorHelper;
use crate::executor::fp_generic::FloatingPointChecks;

use super::ExecuteResult;
use crate::core::register::{Apsr, BaseReg, ExtensionRegOperations};

pub trait IsaFloatingPointRegisterTransfer {
    fn exec_vmov_cr_sp(&mut self, params: &VMovCrSpParams) -> ExecuteResult;
    fn exec_vmov_cr2_dp(&mut self, params: &VMovCr2DpParams) -> ExecuteResult;

    fn exec_vmov_reg_f32(&mut self, params: VMovRegParamsf32) -> ExecuteResult;
    fn exec_vmov_reg_f64(&mut self, params: VMovRegParamsf64) -> ExecuteResult;

    fn exec_vmov_imm_32(&mut self, params: VMovImmParams32) -> ExecuteResult;
    fn exec_vmov_imm_64(&mut self, params: VMovImmParams64) -> ExecuteResult;

    fn exec_vmrs(&mut self, params: VMRSTarget) -> ExecuteResult;
}

impl IsaFloatingPointRegisterTransfer for Processor {
    fn exec_vmov_cr_sp(&mut self, params: &VMovCrSpParams) -> ExecuteResult {
        self.execute_fp_check()?;
        if params.to_arm_register {
            let value = self.get_sr(params.sn);
            self.set_r(params.rt, value);
        } else {
            self.set_sr(params.sn, self.get_r(params.rt));
        }

        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }

    fn exec_vmov_cr2_dp(&mut self, params: &VMovCr2DpParams) -> ExecuteResult {
        self.execute_fp_check()?;
        if params.to_arm_registers {
            let (low, high) = self.get_dr(params.dm);
            self.set_r(params.rt, low);
            self.set_r(params.rt2, high);
        } else {
            let low = self.get_r(params.rt);
            let high = self.get_r(params.rt2);
            self.set_dr(params.dm, low, high);
        }

        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }

    fn exec_vmov_reg_f32(&mut self, params: VMovRegParamsf32) -> ExecuteResult {
        self.execute_fp_check()?;
        let value = self.get_sr(params.sm);
        self.set_sr(params.sd, value);
        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }

    fn exec_vmov_reg_f64(&mut self, params: VMovRegParamsf64) -> ExecuteResult {
        self.execute_fp_check()?;
        let (low, high) = self.get_dr(params.dm);
        self.set_dr(params.dd, low, high);
        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }

    fn exec_vmov_imm_32(&mut self, params: VMovImmParams32) -> ExecuteResult {
        self.execute_fp_check()?;
        self.set_sr(params.sd, params.imm32);
        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }

    fn exec_vmov_imm_64(&mut self, params: VMovImmParams64) -> ExecuteResult {
        self.execute_fp_check()?;
        let (low, high) = (params.imm64 as u32, (params.imm64 >> 32) as u32);
        self.set_dr(params.dd, low, high);
        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }

    fn exec_vmrs(&mut self, params: VMRSTarget) -> ExecuteResult {
        if self.condition_passed() {
            //EncodingSpecificOperations();
            self.execute_fp_check()?;
            //SerializeVFP();
            //VFPExcBarrier();

            match params {
                VMRSTarget::APSRNZCV => {
                    let n = self.fpscr.get_n();
                    let z = self.fpscr.get_z();
                    let c = self.fpscr.get_c();
                    let v = self.fpscr.get_v();

                    self.psr.set_n_bit(n);
                    self.psr.set_z_bit(z);
                    self.psr.set_c(c);
                    self.psr.set_v(v);
                }
                VMRSTarget::Register(reg) => {
                    self.set_r(reg, self.fpscr);
                }
            }
        }
        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::instruction::{
        VMRSTarget, VMovCr2DpParams, VMovCrSpParams, VMovImmParams32, VMovImmParams64,
        VMovRegParamsf32, VMovRegParamsf64,
    };
    use crate::core::register::{Apsr, DoubleReg, Reg, SingleReg};

    fn fp_test_processor() -> Processor {
        let mut processor = Processor::new();
        processor.cpacr = 0x00f0_0000;
        processor
    }

    #[test]
    fn test_vmov_cr_sp_moves_between_core_and_single_registers() {
        let mut processor = fp_test_processor();
        processor.set_r(Reg::R2, 0x1234_5678);

        processor
            .exec_vmov_cr_sp(&VMovCrSpParams {
                to_arm_register: false,
                rt: Reg::R2,
                sn: SingleReg::S4,
            })
            .unwrap();
        assert_eq!(processor.get_sr(SingleReg::S4), 0x1234_5678);

        processor.set_sr(SingleReg::S5, 0x89ab_cdef);
        processor
            .exec_vmov_cr_sp(&VMovCrSpParams {
                to_arm_register: true,
                rt: Reg::R3,
                sn: SingleReg::S5,
            })
            .unwrap();
        assert_eq!(processor.get_r(Reg::R3), 0x89ab_cdef);
    }

    #[test]
    fn test_vmov_cr2_dp_moves_between_core_and_double_registers() {
        let mut processor = fp_test_processor();
        processor.set_r(Reg::R0, 0x1111_2222);
        processor.set_r(Reg::R1, 0x3333_4444);

        processor
            .exec_vmov_cr2_dp(&VMovCr2DpParams {
                to_arm_registers: false,
                rt: Reg::R0,
                rt2: Reg::R1,
                dm: DoubleReg::D6,
            })
            .unwrap();
        assert_eq!(processor.get_dr(DoubleReg::D6), (0x1111_2222, 0x3333_4444));

        processor.set_dr(DoubleReg::D7, 0xaaaa_bbbb, 0xcccc_dddd);
        processor
            .exec_vmov_cr2_dp(&VMovCr2DpParams {
                to_arm_registers: true,
                rt: Reg::R4,
                rt2: Reg::R5,
                dm: DoubleReg::D7,
            })
            .unwrap();
        assert_eq!(processor.get_r(Reg::R4), 0xaaaa_bbbb);
        assert_eq!(processor.get_r(Reg::R5), 0xcccc_dddd);
    }

    #[test]
    fn test_vmov_reg_and_immediate_variants() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S0, 0x3f80_0000);
        processor
            .exec_vmov_reg_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();
        assert_eq!(processor.get_sr(SingleReg::S1), 0x3f80_0000);

        processor.set_dr(DoubleReg::D0, 0x5555_aaaa, 0x1234_5678);
        processor
            .exec_vmov_reg_f64(VMovRegParamsf64 {
                dd: DoubleReg::D1,
                dm: DoubleReg::D0,
            })
            .unwrap();
        assert_eq!(processor.get_dr(DoubleReg::D1), (0x5555_aaaa, 0x1234_5678));

        processor
            .exec_vmov_imm_32(VMovImmParams32 {
                sd: SingleReg::S2,
                imm32: 0x4000_0000,
            })
            .unwrap();
        assert_eq!(processor.get_sr(SingleReg::S2), 0x4000_0000);

        processor
            .exec_vmov_imm_64(VMovImmParams64 {
                dd: DoubleReg::D2,
                imm64: 0x1122_3344_5566_7788,
            })
            .unwrap();
        assert_eq!(processor.get_dr(DoubleReg::D2), (0x5566_7788, 0x1122_3344));
    }

    #[test]
    fn test_vmrs_writes_apsr_flags_and_general_register() {
        let mut processor = fp_test_processor();
        processor.fpscr.set_n(true);
        processor.fpscr.set_z(false);
        processor.fpscr.set_c(true);
        processor.fpscr.set_v(false);

        processor.exec_vmrs(VMRSTarget::APSRNZCV).unwrap();
        assert!(processor.psr.get_n());
        assert!(!processor.psr.get_z());
        assert!(processor.psr.get_c());
        assert!(!processor.psr.get_v());

        processor.fpscr = 0xabcd_1234;
        processor.exec_vmrs(VMRSTarget::Register(Reg::R7)).unwrap();
        assert_eq!(processor.get_r(Reg::R7), 0xabcd_1234);
    }
}
