use crate::Processor;
use crate::core::fpregister::Fpscr;
use crate::core::instruction::{
    VAddSubParamsf32, VAddSubParamsf64, VCVTParams, VCVTParamsF32F64, VCVTParamsF64F32,
    VCmpParamsf32, VCmpParamsf64, VMovRegParamsf32, VMovRegParamsf64, VSelParamsf32, VSelParamsf64,
};

use crate::executor::ExecuteSuccess;

use super::ExecuteResult;
use super::fp_generic::{FloatingPointChecks, FloatingPointPublicOperations};
use crate::core::register::ExtensionRegOperations;
use crate::executor::ExecutorHelper;

pub trait IsaFloatingPointDataProcessing {
    fn exec_vabs_f32(&mut self, params: VMovRegParamsf32) -> ExecuteResult;
    fn exec_vabs_f64(&mut self, params: VMovRegParamsf64) -> ExecuteResult;
    fn exec_vneg_f32(&mut self, params: VMovRegParamsf32) -> ExecuteResult;
    fn exec_vneg_f64(&mut self, params: VMovRegParamsf64) -> ExecuteResult;

    fn exec_vrintz_f32(&mut self, params: VMovRegParamsf32) -> ExecuteResult;
    fn exec_vrintz_f64(&mut self, params: VMovRegParamsf64) -> ExecuteResult;

    fn exec_vsqrt_f32(&mut self, params: VMovRegParamsf32) -> ExecuteResult;
    fn exec_vsqrt_f64(&mut self, params: VMovRegParamsf64) -> ExecuteResult;

    fn exec_vcmp_f32(&mut self, params: &VCmpParamsf32) -> ExecuteResult;
    fn exec_vcmp_f64(&mut self, params: &VCmpParamsf64) -> ExecuteResult;

    fn exec_vadd_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult;
    fn exec_vadd_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult;
    fn exec_vdiv_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult;
    fn exec_vdiv_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult;
    fn exec_vfma_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult;
    fn exec_vfma_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult;
    fn exec_vfms_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult;
    fn exec_vfms_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult;
    fn exec_vfnms_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult;
    fn exec_vfnms_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult;
    fn exec_vmul_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult;
    fn exec_vmul_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult;
    fn exec_vnmul_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult;
    fn exec_vnmul_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult;

    fn exec_vsub_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult;
    fn exec_vsub_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult;

    fn exec_vcvt(&mut self, params: &VCVTParams) -> ExecuteResult;
    fn exec_vcvt_f64_f32(&mut self, params: VCVTParamsF64F32) -> ExecuteResult;
    fn exec_vcvt_f32_f64(&mut self, params: VCVTParamsF32F64) -> ExecuteResult;
    fn exec_vsel_f32(&mut self, params: VSelParamsf32) -> ExecuteResult;
    fn exec_vsel_f64(&mut self, params: VSelParamsf64) -> ExecuteResult;
}

impl IsaFloatingPointDataProcessing for Processor {
    fn exec_vabs_f32(&mut self, params: VMovRegParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let op = self.get_sr(params.sm);
            let result = self.fp_abs::<u32>(op);
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vabs_f64(&mut self, params: VMovRegParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (lower, upper) = self.get_dr(params.dm);
            let op = u64::from(upper) << 32 | (u64::from(lower));

            let result = self.fp_abs::<u64>(op);

            let result_upper = (result >> 32) as u32;
            let result_lower = result as u32;

            self.set_dr(params.dd, result_lower, result_upper);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vneg_f32(&mut self, params: VMovRegParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let op = self.get_sr(params.sm);
            self.set_sr(params.sd, op ^ 0x8000_0000);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vneg_f64(&mut self, params: VMovRegParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (lower, upper) = self.get_dr(params.dm);
            self.set_dr(params.dd, lower, upper ^ 0x8000_0000);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vrintz_f32(&mut self, params: VMovRegParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let op = self.get_sr(params.sm);
            let result = self.fp_round_int::<u32>(op, true, false, true);
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vrintz_f64(&mut self, params: VMovRegParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (lower, upper) = self.get_dr(params.dm);
            let op = (u64::from(upper) << 32) | u64::from(lower);
            let result = self.fp_round_int::<u64>(op, true, false, true);
            self.set_dr(params.dd, result as u32, (result >> 32) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vsqrt_f32(&mut self, params: VMovRegParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let op = self.get_sr(params.sm);
            let result = self.fp_sqrt::<u32>(op, true);
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vsqrt_f64(&mut self, params: VMovRegParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (lower, upper) = self.get_dr(params.dm);
            let op = (u64::from(upper) << 32) | u64::from(lower);
            let result = self.fp_sqrt::<u64>(op, true);
            self.set_dr(params.dd, result as u32, (result >> 32) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vcmp_f32(&mut self, params: &VCmpParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let op32 = if params.with_zero {
                0
            } else {
                self.get_sr(params.sm)
            };
            let op1 = self.get_sr(params.sd);
            let (n, z, c, v) = self.fp_compare::<u32>(op1, op32, params.quiet_nan_exc, true);
            self.fpscr.set_n(n);
            self.fpscr.set_z(z);
            self.fpscr.set_c(c);
            self.fpscr.set_v(v);
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_vcmp_f64(&mut self, params: &VCmpParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let op64 = if params.with_zero {
                0u64
            } else {
                let (lower, upper) = self.get_dr(params.dm);
                u64::from(upper) << 32 | u64::from(lower)
            };
            let op1_src = self.get_dr(params.dd);
            let op1 = u64::from(op1_src.1) << 32 | u64::from(op1_src.0);
            let (n, z, c, v) = self.fp_compare::<u64>(op1, op64, params.quiet_nan_exc, true);
            self.fpscr.set_n(n);
            self.fpscr.set_z(z);
            self.fpscr.set_c(c);
            self.fpscr.set_v(v);
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vadd_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let op1 = self.get_sr(params.sn);
            let op2 = self.get_sr(params.sm);
            let result = self.fp_add::<u32>(op1, op2, true);
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_vadd_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (op1_lower, op1_upper) = self.get_dr(params.dn);
            let (op2_lower, op2_upper) = self.get_dr(params.dm);
            let op1 = u64::from(op1_upper) << 32 | u64::from(op1_lower);
            let op2 = u64::from(op2_upper) << 32 | u64::from(op2_lower);
            let result = self.fp_add::<u64>(op1, op2, true);
            let result_upper = (result >> 32) as u32;
            let result_lower = result as u32;
            self.set_dr(params.dd, result_lower, result_upper);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vdiv_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let op1 = self.get_sr(params.sn);
            let op2 = self.get_sr(params.sm);
            let result = self.fp_div::<u32>(op1, op2, true);
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vdiv_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (op1_lower, op1_upper) = self.get_dr(params.dn);
            let (op2_lower, op2_upper) = self.get_dr(params.dm);
            let op1 = u64::from(op1_upper) << 32 | u64::from(op1_lower);
            let op2 = u64::from(op2_upper) << 32 | u64::from(op2_lower);
            let result = self.fp_div::<u64>(op1, op2, true);
            let result_upper = (result >> 32) as u32;
            let result_lower = result as u32;
            self.set_dr(params.dd, result_lower, result_upper);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vfma_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let addend = self.get_sr(params.sd);
            let op1 = self.get_sr(params.sn);
            let op2 = self.get_sr(params.sm);
            let result = self.fp_mul_add::<u32>(addend, op1, op2, true);
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vfma_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (dest_low, dest_high) = self.get_dr(params.dd);
            let (op1_low, op1_high) = self.get_dr(params.dn);
            let (op2_low, op2_high) = self.get_dr(params.dm);
            let addend = (u64::from(dest_high) << 32) | u64::from(dest_low);
            let op1 = (u64::from(op1_high) << 32) | u64::from(op1_low);
            let op2 = (u64::from(op2_high) << 32) | u64::from(op2_low);
            let result = self.fp_mul_add::<u64>(addend, op1, op2, true);
            self.set_dr(params.dd, result as u32, (result >> 32) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vfms_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let addend = self.get_sr(params.sd);
            let op1 = self.get_sr(params.sn);
            let op2 = self.get_sr(params.sm) ^ 0x8000_0000;
            let result = self.fp_mul_add::<u32>(addend, op1, op2, true);
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vfms_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (dest_low, dest_high) = self.get_dr(params.dd);
            let (op1_low, op1_high) = self.get_dr(params.dn);
            let (op2_low, op2_high) = self.get_dr(params.dm);
            let addend = (u64::from(dest_high) << 32) | u64::from(dest_low);
            let op1 = (u64::from(op1_high) << 32) | u64::from(op1_low);
            let op2 = ((u64::from(op2_high) << 32) | u64::from(op2_low)) ^ 0x8000_0000_0000_0000;
            let result = self.fp_mul_add::<u64>(addend, op1, op2, true);
            self.set_dr(params.dd, result as u32, (result >> 32) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vfnms_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let addend = self.get_sr(params.sd) ^ 0x8000_0000;
            let op1 = self.get_sr(params.sn);
            let op2 = self.get_sr(params.sm);
            let result = self.fp_mul_add::<u32>(addend, op1, op2, true);
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vfnms_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (dest_low, dest_high) = self.get_dr(params.dd);
            let (op1_low, op1_high) = self.get_dr(params.dn);
            let (op2_low, op2_high) = self.get_dr(params.dm);
            let addend =
                ((u64::from(dest_high) << 32) | u64::from(dest_low)) ^ 0x8000_0000_0000_0000;
            let op1 = (u64::from(op1_high) << 32) | u64::from(op1_low);
            let op2 = (u64::from(op2_high) << 32) | u64::from(op2_low);
            let result = self.fp_mul_add::<u64>(addend, op1, op2, true);
            self.set_dr(params.dd, result as u32, (result >> 32) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vmul_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let op1 = self.get_sr(params.sn);
            let op2 = self.get_sr(params.sm);
            let result = self.fp_mul::<u32>(op1, op2, true);
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vmul_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (op1_lower, op1_upper) = self.get_dr(params.dn);
            let (op2_lower, op2_upper) = self.get_dr(params.dm);
            let op1 = u64::from(op1_upper) << 32 | u64::from(op1_lower);
            let op2 = u64::from(op2_upper) << 32 | u64::from(op2_lower);
            let result = self.fp_mul::<u64>(op1, op2, true);
            let result_upper = (result >> 32) as u32;
            let result_lower = result as u32;
            self.set_dr(params.dd, result_lower, result_upper);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vnmul_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let op1 = self.get_sr(params.sn);
            let op2 = self.get_sr(params.sm);
            let result = self.fp_mul::<u32>(op1, op2, true) ^ 0x8000_0000;
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vnmul_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (op1_lower, op1_upper) = self.get_dr(params.dn);
            let (op2_lower, op2_upper) = self.get_dr(params.dm);
            let op1 = u64::from(op1_upper) << 32 | u64::from(op1_lower);
            let op2 = u64::from(op2_upper) << 32 | u64::from(op2_lower);
            let result = self.fp_mul::<u64>(op1, op2, true) ^ 0x8000_0000_0000_0000;
            let result_upper = (result >> 32) as u32;
            let result_lower = result as u32;
            self.set_dr(params.dd, result_lower, result_upper);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vsub_f32(&mut self, params: &VAddSubParamsf32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let op1 = self.get_sr(params.sn);
            let op2 = self.get_sr(params.sm);
            let result = self.fp_sub::<u32>(op1, op2, true);
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_vsub_f64(&mut self, params: &VAddSubParamsf64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (op1_lower, op1_upper) = self.get_dr(params.dn);
            let (op2_lower, op2_upper) = self.get_dr(params.dm);
            let op1 = u64::from(op1_upper) << 32 | u64::from(op1_lower);
            let op2 = u64::from(op2_upper) << 32 | u64::from(op2_lower);
            let result = self.fp_sub::<u64>(op1, op2, true);
            let result_upper = (result >> 32) as u32;
            let result_lower = result as u32;
            self.set_dr(params.dd, result_lower, result_upper);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vcvt(&mut self, params: &VCVTParams) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;

            if params.to_integer {
                if params.dp_operation {
                    let m_reg = params
                        .m
                        .as_double()
                        .expect("Invalid register for double precision operation");
                    let d_reg = params
                        .d
                        .as_single()
                        .expect("Invalid register for single precision operation");
                    let dm = self.get_dr(*m_reg);
                    let dm_val = u64::from(dm.1) << 32 | u64::from(dm.0);
                    let result = self.fp_to_fixed::<u64, u32>(
                        dm_val,
                        0,
                        params.unsigned,
                        params.round_zero,
                        true,
                    );
                    self.set_sr(*d_reg, result);
                } else {
                    let m_reg = params
                        .m
                        .as_single()
                        .expect("Invalid register for single precision operation");
                    let d_reg = params
                        .d
                        .as_single()
                        .expect("Invalid register for single precision operation");
                    let dm = self.get_sr(*m_reg);
                    let result = self.fp_to_fixed::<u32, u32>(
                        dm,
                        0,
                        params.unsigned,
                        params.round_zero,
                        true,
                    );
                    self.set_sr(*d_reg, result);
                }
            } else if params.dp_operation {
                let m_reg = params
                    .m
                    .as_single()
                    .expect("Invalid register for single precision operation");
                let d_reg = params
                    .d
                    .as_double()
                    .expect("Invalid register for double precision operation");
                let dm = self.get_sr(*m_reg);
                let result =
                    self.fixed_to_fp::<u64, u32>(dm, 0, params.unsigned, params.round_zero, true);
                let result_upper = (result >> 32) as u32;
                let result_lower = result as u32;
                self.set_dr(*d_reg, result_lower, result_upper);
            } else {
                let m_reg = params
                    .m
                    .as_single()
                    .expect("Invalid register for single precision operation");
                let d_reg = params
                    .d
                    .as_single()
                    .expect("Invalid register for single precision operation");
                let dm = self.get_sr(*m_reg);
                let result =
                    self.fixed_to_fp::<u32, u32>(dm, 0, params.unsigned, params.round_zero, true);
                self.set_sr(*d_reg, result);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vcvt_f64_f32(&mut self, params: VCVTParamsF64F32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let source = f32::from_bits(self.get_sr(params.sm));
            let result = f64::from(source).to_bits();
            self.set_dr(params.dd, result as u32, (result >> 32) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vcvt_f32_f64(&mut self, params: VCVTParamsF32F64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;
            let (lower, upper) = self.get_dr(params.dm);
            let source = f64::from_bits((u64::from(upper) << 32) | u64::from(lower));
            let result = (source as f32).to_bits();
            self.set_sr(params.sd, result);
            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vsel_f32(&mut self, params: VSelParamsf32) -> ExecuteResult {
        self.execute_fp_check()?;
        let result = if self.condition_passed_b(params.cond) {
            self.get_sr(params.sn)
        } else {
            self.get_sr(params.sm)
        };
        self.set_sr(params.sd, result);
        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }

    fn exec_vsel_f64(&mut self, params: VSelParamsf64) -> ExecuteResult {
        self.execute_fp_check()?;
        let result = if self.condition_passed_b(params.cond) {
            self.get_dr(params.dn)
        } else {
            self.get_dr(params.dm)
        };
        self.set_dr(params.dd, result.0, result.1);
        Ok(ExecuteSuccess::Taken { cycles: 1 })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::bits::Bits;
    use crate::core::register::{Apsr, DoubleReg, ExtensionReg, SingleReg};

    fn fp_test_processor() -> Processor {
        let mut processor = Processor::new();
        processor.cpacr = 0x00f0_0000;
        processor
    }

    #[test]
    fn test_vabs_f32() {
        let mut processor = fp_test_processor();

        // -1.0
        processor.set_sr(SingleReg::S0, 0xBF80_0000);
        processor
            .exec_vabs_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();

        let result = processor.get_sr(SingleReg::S1);
        assert_eq!(result, 0x3F80_0000);
    }

    #[test]
    fn test_vabs_f64() {
        let mut processor = fp_test_processor();

        // -1.0
        processor.set_dr(DoubleReg::D0, 0x0000_0000, 0xBFF0_0000);
        processor
            .exec_vabs_f64(VMovRegParamsf64 {
                dd: DoubleReg::D1,
                dm: DoubleReg::D0,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D1);
        assert_eq!(result, (0x0000_0000, 0x3FF0_0000));
    }

    #[test]
    fn test_vneg_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S0, 1.5f32.to_bits());
        processor
            .exec_vneg_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S1), (-1.5f32).to_bits());
    }

    #[test]
    fn test_vneg_f64() {
        let mut processor = fp_test_processor();

        let bits = 1.5f64.to_bits();
        processor.set_dr(DoubleReg::D0, bits as u32, (bits >> 32) as u32);
        processor
            .exec_vneg_f64(VMovRegParamsf64 {
                dd: DoubleReg::D1,
                dm: DoubleReg::D0,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D1);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert_eq!(result_bits, (-1.5f64).to_bits());
    }

    #[test]
    fn test_vsqrt_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S0, 0x4080_0000);
        processor
            .exec_vsqrt_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();

        let result = processor.get_sr(SingleReg::S1);
        assert_eq!(result, 0x4000_0000);
    }

    #[test]
    fn test_vsqrt_f64() {
        let mut processor = fp_test_processor();

        processor.set_dr(DoubleReg::D0, 0x0000_0000, 0x4010_0000);
        processor
            .exec_vsqrt_f64(VMovRegParamsf64 {
                dd: DoubleReg::D1,
                dm: DoubleReg::D0,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D1);
        assert_eq!(result, (0x0000_0000, 0x4000_0000));
    }

    #[test]
    fn test_vrintz_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S15, 1.75f32.to_bits());
        processor
            .exec_vrintz_f32(VMovRegParamsf32 {
                sd: SingleReg::S14,
                sm: SingleReg::S15,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S14), 1.0f32.to_bits());
    }

    #[test]
    fn test_vrintz_f64() {
        let mut processor = fp_test_processor();

        let input = (-2.9f64).to_bits();
        processor.set_dr(DoubleReg::D7, input as u32, (input >> 32) as u32);
        processor
            .exec_vrintz_f64(VMovRegParamsf64 {
                dd: DoubleReg::D6,
                dm: DoubleReg::D7,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D6);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert_eq!(result_bits, (-2.0f64).to_bits());
    }

    #[test]
    fn test_vrintz_f32_ignores_fpscr_rounding_mode() {
        let mut processor = fp_test_processor();
        processor
            .fpscr
            .set_rounding_mode(crate::core::fpregister::FPSCRRounding::RoundTowardsPlusInfinity);

        processor.set_sr(SingleReg::S15, (-2.9f32).to_bits());
        processor
            .exec_vrintz_f32(VMovRegParamsf32 {
                sd: SingleReg::S14,
                sm: SingleReg::S15,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S14), (-2.0f32).to_bits());
    }

    #[test]
    fn test_vsqrt_f32_zero_and_inf() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S0, 0.0f32.to_bits());
        processor
            .exec_vsqrt_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();
        assert_eq!(processor.get_sr(SingleReg::S1), 0.0f32.to_bits());

        processor.set_sr(SingleReg::S0, f32::INFINITY.to_bits());
        processor
            .exec_vsqrt_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();
        assert_eq!(processor.get_sr(SingleReg::S1), f32::INFINITY.to_bits());
    }

    #[test]
    fn test_vsqrt_f32_negative_and_nan() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S0, (-1.0f32).to_bits());
        processor
            .exec_vsqrt_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();
        assert!(f32::from_bits(processor.get_sr(SingleReg::S1)).is_nan());

        processor.set_sr(SingleReg::S0, f32::NAN.to_bits());
        processor
            .exec_vsqrt_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();
        assert!(f32::from_bits(processor.get_sr(SingleReg::S1)).is_nan());
    }

    #[test]
    fn test_vsqrt_f32_subnormal() {
        let mut processor = fp_test_processor();

        let input = f32::from_bits(0x0000_0001);
        processor.set_sr(SingleReg::S0, input.to_bits());
        processor
            .exec_vsqrt_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();

        let expected = input.sqrt().to_bits();
        assert_eq!(processor.get_sr(SingleReg::S1), expected);
    }

    #[test]
    fn test_vsqrt_f32_negative_sets_invalid_op() {
        let mut processor = fp_test_processor();
        processor.fpscr = 0;

        processor.set_sr(SingleReg::S0, (-4.0f32).to_bits());
        processor
            .exec_vsqrt_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();

        assert!(processor.fpscr.get_bit(0));
        assert!(f32::from_bits(processor.get_sr(SingleReg::S1)).is_nan());
    }

    #[test]
    fn test_vsqrt_f32_snan_sets_invalid_op_and_quiets_nan() {
        let mut processor = fp_test_processor();
        processor.fpscr = 0;

        let snan = 0x7F80_0001u32;
        processor.set_sr(SingleReg::S0, snan);
        processor
            .exec_vsqrt_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();

        let result = processor.get_sr(SingleReg::S1);
        assert!(processor.fpscr.get_bit(0));
        assert!(result.get_bit(22));
        assert!(f32::from_bits(result).is_nan());
    }

    #[test]
    fn test_vsqrt_f32_qnan_does_not_set_invalid_op() {
        let mut processor = fp_test_processor();
        processor.fpscr = 0;

        let qnan = 0x7FC0_1234u32;
        processor.set_sr(SingleReg::S0, qnan);
        processor
            .exec_vsqrt_f32(VMovRegParamsf32 {
                sd: SingleReg::S1,
                sm: SingleReg::S0,
            })
            .unwrap();

        let result = processor.get_sr(SingleReg::S1);
        assert!(!processor.fpscr.get_bit(0));
        assert_eq!(result, qnan);
    }

    #[test]
    fn test_vsqrt_f64_zero_and_inf() {
        let mut processor = fp_test_processor();

        processor.set_dr(
            DoubleReg::D0,
            0.0f64.to_bits() as u32,
            (0.0f64.to_bits() >> 32) as u32,
        );
        processor
            .exec_vsqrt_f64(VMovRegParamsf64 {
                dd: DoubleReg::D1,
                dm: DoubleReg::D0,
            })
            .unwrap();
        let result = processor.get_dr(DoubleReg::D1);
        assert_eq!(
            (u64::from(result.1) << 32) | u64::from(result.0),
            0.0f64.to_bits()
        );

        processor.set_dr(
            DoubleReg::D0,
            f64::INFINITY.to_bits() as u32,
            (f64::INFINITY.to_bits() >> 32) as u32,
        );
        processor
            .exec_vsqrt_f64(VMovRegParamsf64 {
                dd: DoubleReg::D1,
                dm: DoubleReg::D0,
            })
            .unwrap();
        let result = processor.get_dr(DoubleReg::D1);
        assert_eq!(
            (u64::from(result.1) << 32) | u64::from(result.0),
            f64::INFINITY.to_bits()
        );
    }

    #[test]
    fn test_vsqrt_f64_negative_and_nan() {
        let mut processor = fp_test_processor();

        processor.set_dr(
            DoubleReg::D0,
            (-1.0f64).to_bits() as u32,
            ((-1.0f64).to_bits() >> 32) as u32,
        );
        processor
            .exec_vsqrt_f64(VMovRegParamsf64 {
                dd: DoubleReg::D1,
                dm: DoubleReg::D0,
            })
            .unwrap();
        let result = processor.get_dr(DoubleReg::D1);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert!(f64::from_bits(result_bits).is_nan());

        processor.set_dr(
            DoubleReg::D0,
            f64::NAN.to_bits() as u32,
            (f64::NAN.to_bits() >> 32) as u32,
        );
        processor
            .exec_vsqrt_f64(VMovRegParamsf64 {
                dd: DoubleReg::D1,
                dm: DoubleReg::D0,
            })
            .unwrap();
        let result = processor.get_dr(DoubleReg::D1);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert!(f64::from_bits(result_bits).is_nan());
    }

    #[test]
    fn test_vsqrt_f64_subnormal() {
        let mut processor = fp_test_processor();

        let input = f64::from_bits(0x0000_0000_0000_0001);
        processor.set_dr(
            DoubleReg::D0,
            input.to_bits() as u32,
            (input.to_bits() >> 32) as u32,
        );
        processor
            .exec_vsqrt_f64(VMovRegParamsf64 {
                dd: DoubleReg::D1,
                dm: DoubleReg::D0,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D1);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        let expected = input.sqrt().to_bits();
        assert_eq!(result_bits, expected);
    }

    #[test]
    fn test_vfma_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S15, 1.0f32.to_bits());
        processor.set_sr(SingleReg::S13, 2.0f32.to_bits());
        processor.set_sr(SingleReg::S14, 3.0f32.to_bits());

        processor
            .exec_vfma_f32(&VAddSubParamsf32 {
                sd: SingleReg::S15,
                sn: SingleReg::S13,
                sm: SingleReg::S14,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S15), 7.0f32.to_bits());
    }

    #[test]
    fn test_vfma_f64() {
        let mut processor = fp_test_processor();

        let addend = 1.0f64.to_bits();
        let op1 = 2.0f64.to_bits();
        let op2 = 3.0f64.to_bits();

        processor.set_dr(DoubleReg::D7, addend as u32, (addend >> 32) as u32);
        processor.set_dr(DoubleReg::D6, op1 as u32, (op1 >> 32) as u32);
        processor.set_dr(DoubleReg::D5, op2 as u32, (op2 >> 32) as u32);

        processor
            .exec_vfma_f64(&VAddSubParamsf64 {
                dd: DoubleReg::D7,
                dn: DoubleReg::D6,
                dm: DoubleReg::D5,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D7);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert_eq!(result_bits, 7.0f64.to_bits());
    }

    #[test]
    fn test_vmul_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S17, 3.0f32.to_bits());

        processor
            .exec_vmul_f32(&VAddSubParamsf32 {
                sd: SingleReg::S24,
                sn: SingleReg::S17,
                sm: SingleReg::S17,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S24), 9.0f32.to_bits());
    }

    #[test]
    fn test_vnmul_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S1, 2.0f32.to_bits());
        processor.set_sr(SingleReg::S2, 3.0f32.to_bits());

        processor
            .exec_vnmul_f32(&VAddSubParamsf32 {
                sd: SingleReg::S0,
                sn: SingleReg::S1,
                sm: SingleReg::S2,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S0), (-6.0f32).to_bits());
    }

    #[test]
    fn test_vdiv_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S24, 6.0f32.to_bits());
        processor.set_sr(SingleReg::S16, 2.0f32.to_bits());

        processor
            .exec_vdiv_f32(&VAddSubParamsf32 {
                sd: SingleReg::S26,
                sn: SingleReg::S24,
                sm: SingleReg::S16,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S26), 3.0f32.to_bits());
    }

    #[test]
    fn test_vmul_f64() {
        let mut processor = fp_test_processor();

        let op = 1.5f64.to_bits();
        processor.set_dr(DoubleReg::D9, op as u32, (op >> 32) as u32);

        processor
            .exec_vmul_f64(&VAddSubParamsf64 {
                dd: DoubleReg::D12,
                dn: DoubleReg::D9,
                dm: DoubleReg::D9,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D12);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert_eq!(result_bits, 2.25f64.to_bits());
    }

    #[test]
    fn test_vnmul_f64() {
        let mut processor = fp_test_processor();

        let op1 = 1.5f64.to_bits();
        let op2 = 2.0f64.to_bits();
        processor.set_dr(DoubleReg::D8, op1 as u32, (op1 >> 32) as u32);
        processor.set_dr(DoubleReg::D9, op2 as u32, (op2 >> 32) as u32);

        processor
            .exec_vnmul_f64(&VAddSubParamsf64 {
                dd: DoubleReg::D7,
                dn: DoubleReg::D8,
                dm: DoubleReg::D9,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D7);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert_eq!(result_bits, (-3.0f64).to_bits());
    }

    #[test]
    fn test_vdiv_f64() {
        let mut processor = fp_test_processor();

        let numerator = 9.0f64.to_bits();
        let denominator = 4.5f64.to_bits();
        processor.set_dr(DoubleReg::D12, numerator as u32, (numerator >> 32) as u32);
        processor.set_dr(
            DoubleReg::D8,
            denominator as u32,
            (denominator >> 32) as u32,
        );

        processor
            .exec_vdiv_f64(&VAddSubParamsf64 {
                dd: DoubleReg::D13,
                dn: DoubleReg::D12,
                dm: DoubleReg::D8,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D13);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert_eq!(result_bits, 2.0f64.to_bits());
    }

    #[test]
    fn test_vfms_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S15, 10.0f32.to_bits());
        processor.set_sr(SingleReg::S5, 2.0f32.to_bits());
        processor.set_sr(SingleReg::S13, 3.0f32.to_bits());

        processor
            .exec_vfms_f32(&VAddSubParamsf32 {
                sd: SingleReg::S15,
                sn: SingleReg::S5,
                sm: SingleReg::S13,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S15), 4.0f32.to_bits());
    }

    #[test]
    fn test_vfnms_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S0, 10.0f32.to_bits());
        processor.set_sr(SingleReg::S5, 2.0f32.to_bits());
        processor.set_sr(SingleReg::S14, 3.0f32.to_bits());

        processor
            .exec_vfnms_f32(&VAddSubParamsf32 {
                sd: SingleReg::S0,
                sn: SingleReg::S5,
                sm: SingleReg::S14,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S0), (-4.0f32).to_bits());
    }

    #[test]
    fn test_vcvt_f64_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S7, 1.5f32.to_bits());
        processor
            .exec_vcvt_f64_f32(VCVTParamsF64F32 {
                dd: DoubleReg::D3,
                sm: SingleReg::S7,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D3);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert_eq!(result_bits, 1.5f64.to_bits());
    }

    #[test]
    fn test_vcvt_f32_f64() {
        let mut processor = fp_test_processor();

        let input = 1.25f64.to_bits();
        processor.set_dr(DoubleReg::D7, input as u32, (input >> 32) as u32);
        processor
            .exec_vcvt_f32_f64(VCVTParamsF32F64 {
                sd: SingleReg::S14,
                dm: DoubleReg::D7,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S14), 1.25f32.to_bits());
    }

    #[test]
    fn test_vsel_f32_gt_selects_sn_when_true() {
        let mut processor = fp_test_processor();
        processor.psr.set_n(0);
        processor.psr.set_v(false);
        processor.psr.set_z(1);

        processor.set_sr(SingleReg::S14, 0x3f80_0000);
        processor.set_sr(SingleReg::S15, 0x4000_0000);

        processor
            .exec_vsel_f32(VSelParamsf32 {
                sd: SingleReg::S0,
                sn: SingleReg::S14,
                sm: SingleReg::S15,
                cond: crate::core::condition::Condition::GT,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S0), 0x3f80_0000);
    }

    #[test]
    fn test_vsel_f32_gt_selects_sm_when_false() {
        let mut processor = fp_test_processor();
        processor.psr.set_n(1);
        processor.psr.set_v(false);
        processor.psr.set_z(0);

        processor.set_sr(SingleReg::S14, 0x3f80_0000);
        processor.set_sr(SingleReg::S15, 0x4000_0000);

        processor
            .exec_vsel_f32(VSelParamsf32 {
                sd: SingleReg::S0,
                sn: SingleReg::S14,
                sm: SingleReg::S15,
                cond: crate::core::condition::Condition::GT,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S0), 0x4000_0000);
    }

    #[test]
    fn test_vcmp_f64_with_zero_sets_fpscr_flags() {
        let mut processor = fp_test_processor();

        processor.set_dr(DoubleReg::D0, 0, 0x4008_0000);
        processor
            .exec_vcmp_f64(&VCmpParamsf64 {
                dd: DoubleReg::D0,
                dm: DoubleReg::D1,
                quiet_nan_exc: false,
                with_zero: true,
            })
            .unwrap();

        assert!(!processor.fpscr.get_n());
        assert!(!processor.fpscr.get_z());
        assert!(processor.fpscr.get_c());
        assert!(!processor.fpscr.get_v());
    }

    #[test]
    fn test_vcmp_f32_with_zero_sets_fpscr_flags() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S0, (-1.0f32).to_bits());
        processor
            .exec_vcmp_f32(&VCmpParamsf32 {
                sd: SingleReg::S0,
                sm: SingleReg::S1,
                quiet_nan_exc: false,
                with_zero: true,
            })
            .unwrap();

        assert!(processor.fpscr.get_n());
        assert!(!processor.fpscr.get_z());
        assert!(!processor.fpscr.get_c());
        assert!(!processor.fpscr.get_v());
    }

    #[test]
    fn test_vadd_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S2, 1.25f32.to_bits());
        processor.set_sr(SingleReg::S3, 2.5f32.to_bits());

        processor
            .exec_vadd_f32(&VAddSubParamsf32 {
                sd: SingleReg::S4,
                sn: SingleReg::S2,
                sm: SingleReg::S3,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S4), 3.75f32.to_bits());
    }

    #[test]
    fn test_vfms_f64() {
        let mut processor = fp_test_processor();

        let addend = 10.0f64.to_bits();
        let op1 = 2.0f64.to_bits();
        let op2 = 3.0f64.to_bits();
        processor.set_dr(DoubleReg::D4, addend as u32, (addend >> 32) as u32);
        processor.set_dr(DoubleReg::D5, op1 as u32, (op1 >> 32) as u32);
        processor.set_dr(DoubleReg::D6, op2 as u32, (op2 >> 32) as u32);

        processor
            .exec_vfms_f64(&VAddSubParamsf64 {
                dd: DoubleReg::D4,
                dn: DoubleReg::D5,
                dm: DoubleReg::D6,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D4);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert_eq!(result_bits, 4.0f64.to_bits());
    }

    #[test]
    fn test_vadd_f64() {
        let mut processor = fp_test_processor();

        let lhs = 1.25f64.to_bits();
        let rhs = 2.5f64.to_bits();
        processor.set_dr(DoubleReg::D0, lhs as u32, (lhs >> 32) as u32);
        processor.set_dr(DoubleReg::D1, rhs as u32, (rhs >> 32) as u32);

        processor
            .exec_vadd_f64(&VAddSubParamsf64 {
                dd: DoubleReg::D2,
                dn: DoubleReg::D0,
                dm: DoubleReg::D1,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D2);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert_eq!(result_bits, 3.75f64.to_bits());
    }

    #[test]
    fn test_vfnms_f64() {
        let mut processor = fp_test_processor();

        let addend = 10.0f64.to_bits();
        let op1 = 2.0f64.to_bits();
        let op2 = 3.0f64.to_bits();
        processor.set_dr(DoubleReg::D0, addend as u32, (addend >> 32) as u32);
        processor.set_dr(DoubleReg::D1, op1 as u32, (op1 >> 32) as u32);
        processor.set_dr(DoubleReg::D2, op2 as u32, (op2 >> 32) as u32);

        processor
            .exec_vfnms_f64(&VAddSubParamsf64 {
                dd: DoubleReg::D0,
                dn: DoubleReg::D1,
                dm: DoubleReg::D2,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D0);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert_eq!(result_bits, (-4.0f64).to_bits());
    }

    #[test]
    fn test_vsub_f64() {
        let mut processor = fp_test_processor();

        let lhs = 7.25f64.to_bits();
        let rhs = 1.5f64.to_bits();
        processor.set_dr(DoubleReg::D10, lhs as u32, (lhs >> 32) as u32);
        processor.set_dr(DoubleReg::D11, rhs as u32, (rhs >> 32) as u32);

        processor
            .exec_vsub_f64(&VAddSubParamsf64 {
                dd: DoubleReg::D12,
                dn: DoubleReg::D10,
                dm: DoubleReg::D11,
            })
            .unwrap();

        let result = processor.get_dr(DoubleReg::D12);
        let result_bits = (u64::from(result.1) << 32) | u64::from(result.0);
        assert_eq!(result_bits, 5.75f64.to_bits());
    }

    #[test]
    fn test_vsub_f32() {
        let mut processor = fp_test_processor();

        processor.set_sr(SingleReg::S8, 7.25f32.to_bits());
        processor.set_sr(SingleReg::S9, 1.5f32.to_bits());

        processor
            .exec_vsub_f32(&VAddSubParamsf32 {
                sd: SingleReg::S10,
                sn: SingleReg::S8,
                sm: SingleReg::S9,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S10), 5.75f32.to_bits());
    }

    #[test]
    fn test_vcvt_f64_to_s32() {
        let mut processor = fp_test_processor();

        let input = 5.0f64.to_bits();
        processor.set_dr(DoubleReg::D3, input as u32, (input >> 32) as u32);

        processor
            .exec_vcvt(&VCVTParams {
                d: ExtensionReg::Single { reg: SingleReg::S8 },
                m: ExtensionReg::Double { reg: DoubleReg::D3 },
                to_integer: true,
                unsigned: false,
                dp_operation: true,
                round_zero: true,
                round_nearest: false,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S8), 5);
    }

    #[test]
    fn test_vsel_f64_selects_false_operand() {
        let mut processor = fp_test_processor();
        processor.psr.set_n(1);
        processor.psr.set_v(false);
        processor.psr.set_z(0);

        processor.set_dr(DoubleReg::D0, 0x1111_2222, 0x3333_4444);
        processor.set_dr(DoubleReg::D1, 0xaaaa_bbbb, 0xcccc_dddd);

        processor
            .exec_vsel_f64(VSelParamsf64 {
                dd: DoubleReg::D2,
                dn: DoubleReg::D0,
                dm: DoubleReg::D1,
                cond: crate::core::condition::Condition::GT,
            })
            .unwrap();

        assert_eq!(processor.get_dr(DoubleReg::D2), (0xaaaa_bbbb, 0xcccc_dddd));
    }
}
