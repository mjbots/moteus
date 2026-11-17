use crate::Processor;
use crate::core::instruction::{
    AddressingMode, VPushPopParams, VStoreMultipleParams32, VStoreMultipleParams64,
};

use super::ExecuteResult;
use crate::executor::fp_generic::FloatingPointChecks;
use crate::executor::{ExecuteSuccess, ExecutorHelper};
use crate::{
    bus::Bus,
    core::{
        instruction::VLoadAndStoreParams,
        register::{BaseReg, ExtensionReg, ExtensionRegOperations, Reg},
    },
};

/// Multiply operations
pub trait IsaFloatingPointLoadAndStore {
    fn exec_vldr(&mut self, params: &VLoadAndStoreParams) -> ExecuteResult;
    fn exec_vstr(&mut self, params: &VLoadAndStoreParams) -> ExecuteResult;
    fn exec_vldm_t1(&mut self, params: &VStoreMultipleParams64) -> ExecuteResult;
    fn exec_vldm_t2(&mut self, params: &VStoreMultipleParams32) -> ExecuteResult;
    fn exec_vpush(&mut self, params: &VPushPopParams) -> ExecuteResult;
    fn exec_vpop(&mut self, params: &VPushPopParams) -> ExecuteResult;
    fn exec_vstm_t1(&mut self, params: &VStoreMultipleParams64) -> ExecuteResult;
    fn exec_vstm_t2(&mut self, params: &VStoreMultipleParams32) -> ExecuteResult;
}

impl IsaFloatingPointLoadAndStore for Processor {
    fn exec_vldr(&mut self, params: &VLoadAndStoreParams) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;

            let base = match params.rn {
                Reg::PC => self.get_r(Reg::PC) & 0xffff_fffc,
                _ => self.get_r(params.rn),
            };

            let address = if params.add {
                base + params.imm32
            } else {
                base - params.imm32
            };
            match params.dd {
                ExtensionReg::Single { reg } => {
                    let data = self.read32(address)?;
                    self.set_sr(reg, data);
                }
                ExtensionReg::Double { reg } => {
                    let word1 = self.read32(address)?;
                    let word2 = self.read32(address + 4)?;
                    self.set_dr(reg, word1, word2);
                }
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vstr(&mut self, params: &VLoadAndStoreParams) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;

            let base = self.get_r(params.rn);

            let address = if params.add {
                base + params.imm32
            } else {
                base - params.imm32
            };
            match params.dd {
                ExtensionReg::Single { reg } => {
                    let value = self.get_sr(reg);
                    self.write32(address, value)?;
                }
                ExtensionReg::Double { reg } => {
                    let (low_word, high_word) = self.get_dr(reg);
                    self.write32(address, low_word)?;
                    self.write32(address + 4, high_word)?;
                }
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vldm_t1(&mut self, params: &VStoreMultipleParams64) -> ExecuteResult {
        if self.condition_passed() {
            let base = self.get_r(params.rn);
            let mut address = if params.mode == AddressingMode::IncrementAfter {
                base
            } else {
                base.wrapping_sub(params.imm32)
            };

            if params.write_back {
                let write_back_value = if params.mode == AddressingMode::IncrementAfter {
                    base.wrapping_add(params.imm32)
                } else {
                    base.wrapping_sub(params.imm32)
                };
                self.set_r(params.rn, write_back_value);
            }

            for reg in &params.list {
                let low_word = self.read32(address)?;
                let high_word = self.read32(address + 4)?;
                if self.big_endian() {
                    self.set_dr(reg, high_word, low_word);
                } else {
                    self.set_dr(reg, low_word, high_word);
                }
                address = address.wrapping_add(8);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vldm_t2(&mut self, params: &VStoreMultipleParams32) -> ExecuteResult {
        if self.condition_passed() {
            let base = self.get_r(params.rn);
            let mut address = if params.mode == AddressingMode::IncrementAfter {
                base
            } else {
                base.wrapping_sub(params.imm32)
            };

            if params.write_back {
                let write_back_value = if params.mode == AddressingMode::IncrementAfter {
                    base.wrapping_add(params.imm32)
                } else {
                    base.wrapping_sub(params.imm32)
                };
                self.set_r(params.rn, write_back_value);
            }

            for reg in &params.list {
                let value = self.read32(address)?;
                self.set_sr(reg, value);
                address = address.wrapping_add(4);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vpush(&mut self, params: &VPushPopParams) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;

            let sp = self.get_r(Reg::SP);
            let mut address = sp - params.imm32;
            self.set_r(Reg::SP, address);

            if params.single_regs {
                for reg in &params.single_precision_registers {
                    let value = self.get_sr(reg);
                    self.write32(address, value)?;
                    address += 4;
                }
            } else {
                for reg in &params.double_precision_registers {
                    let (low_word, high_word) = self.get_dr(reg);
                    if self.big_endian() {
                        self.write32(address, high_word)?;
                        self.write32(address + 4, low_word)?;
                    } else {
                        self.write32(address, low_word)?;
                        self.write32(address + 4, high_word)?;
                    }
                    address += 8;
                }
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vpop(&mut self, params: &VPushPopParams) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;

            let sp = self.get_r(Reg::SP);
            let mut address = sp;
            self.set_r(Reg::SP, sp + params.imm32);
            if params.single_regs {
                for reg in &params.single_precision_registers {
                    let value = self.read32(address)?;
                    self.set_sr(reg, value);
                    address += 4;
                }
            } else {
                for reg in &params.double_precision_registers {
                    let low_word = self.read32(address)?;
                    let high_word = self.read32(address + 4)?;
                    if self.big_endian() {
                        self.set_dr(reg, high_word, low_word);
                    } else {
                        self.set_dr(reg, low_word, high_word);
                    }
                    address += 8;
                }
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vstm_t1(&mut self, params: &VStoreMultipleParams64) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;

            let mut address = if params.mode == AddressingMode::IncrementAfter {
                self.get_r(params.rn)
            } else {
                self.get_r(params.rn) - params.imm32
            };

            if params.write_back {
                let write_back_value = if params.mode == AddressingMode::IncrementAfter {
                    self.get_r(params.rn) + params.imm32
                } else {
                    self.get_r(params.rn) - params.imm32
                };
                self.set_r(params.rn, write_back_value);
            }

            for reg in &params.list {
                let (low_word, high_word) = self.get_dr(reg);
                if self.big_endian() {
                    self.write32(address, high_word)?;
                    self.write32(address + 4, low_word)?;
                } else {
                    self.write32(address, low_word)?;
                    self.write32(address + 4, high_word)?;
                }
                address += 8;
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_vstm_t2(&mut self, params: &VStoreMultipleParams32) -> ExecuteResult {
        if self.condition_passed() {
            self.execute_fp_check()?;

            let mut address = if params.mode == AddressingMode::IncrementAfter {
                self.get_r(params.rn)
            } else {
                self.get_r(params.rn) - params.imm32
            };

            if params.write_back {
                let write_back_value = if params.mode == AddressingMode::IncrementAfter {
                    self.get_r(params.rn) + params.imm32
                } else {
                    self.get_r(params.rn) - params.imm32
                };
                self.set_r(params.rn, write_back_value);
            }

            for reg in &params.list {
                let value = self.get_sr(reg);
                self.write32(address, value)?;
                address += 4;
            }

            return Ok(ExecuteSuccess::Taken { cycles: 1 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::Bus;
    use crate::core::register::{DoubleReg, SingleReg};
    use enum_set::EnumSet;

    fn fp_test_processor() -> Processor {
        let mut processor = Processor::new();
        processor.cpacr = 0x00f0_0000;
        processor
    }

    #[test]
    fn test_exec_vldm_t2_updates_base_and_loads_single_registers() {
        let mut processor = fp_test_processor();
        let base = 0x2000_0100;

        processor.write32(base, 1.5f32.to_bits()).unwrap();
        processor.write32(base + 4, (-2.25f32).to_bits()).unwrap();
        processor.set_r(Reg::R3, base);

        let mut list = EnumSet::new();
        list.insert(SingleReg::S14);
        list.insert(SingleReg::S15);

        processor
            .exec_vldm_t2(&VStoreMultipleParams32 {
                mode: AddressingMode::IncrementAfter,
                rn: Reg::R3,
                write_back: true,
                list,
                imm32: 8,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S14), 1.5f32.to_bits());
        assert_eq!(processor.get_sr(SingleReg::S15), (-2.25f32).to_bits());
        assert_eq!(processor.get_r(Reg::R3), base + 8);
    }

    #[test]
    fn test_exec_vldm_t1_loads_double_registers_without_writeback() {
        let mut processor = fp_test_processor();
        let base = 0x2000_0200;
        let bits = 3.5f64.to_bits();

        processor.write32(base, bits as u32).unwrap();
        processor.write32(base + 4, (bits >> 32) as u32).unwrap();
        processor.set_r(Reg::R4, base);

        let mut list = EnumSet::new();
        list.insert(DoubleReg::D8);

        processor
            .exec_vldm_t1(&VStoreMultipleParams64 {
                mode: AddressingMode::IncrementAfter,
                rn: Reg::R4,
                write_back: false,
                list,
                imm32: 8,
            })
            .unwrap();

        assert_eq!(
            processor.get_dr(DoubleReg::D8),
            (bits as u32, (bits >> 32) as u32)
        );
        assert_eq!(processor.get_r(Reg::R4), base);
    }

    #[test]
    fn test_exec_vpop_restores_single_registers_from_current_sp() {
        let mut processor = fp_test_processor();
        let sp = 0x2000_0300;

        processor.write32(sp, 1.5f32.to_bits()).unwrap();
        processor.write32(sp + 4, (-2.25f32).to_bits()).unwrap();
        processor.set_r(Reg::SP, sp);

        let mut list = EnumSet::new();
        list.insert(SingleReg::S16);
        list.insert(SingleReg::S17);

        processor
            .exec_vpop(&VPushPopParams {
                single_regs: true,
                single_precision_registers: list,
                double_precision_registers: EnumSet::new(),
                imm32: 8,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S16), 1.5f32.to_bits());
        assert_eq!(processor.get_sr(SingleReg::S17), (-2.25f32).to_bits());
        assert_eq!(processor.get_r(Reg::SP), sp + 8);
    }

    #[test]
    fn test_exec_vpop_restores_double_registers_from_current_sp() {
        let mut processor = fp_test_processor();
        let sp = 0x2000_0400;
        let d8 = 3.5f64.to_bits();
        let d9 = (-7.25f64).to_bits();

        processor.write32(sp, d8 as u32).unwrap();
        processor.write32(sp + 4, (d8 >> 32) as u32).unwrap();
        processor.write32(sp + 8, d9 as u32).unwrap();
        processor.write32(sp + 12, (d9 >> 32) as u32).unwrap();
        processor.set_r(Reg::SP, sp);

        let mut list = EnumSet::new();
        list.insert(DoubleReg::D8);
        list.insert(DoubleReg::D9);

        processor
            .exec_vpop(&VPushPopParams {
                single_regs: false,
                single_precision_registers: EnumSet::new(),
                double_precision_registers: list,
                imm32: 16,
            })
            .unwrap();

        assert_eq!(
            processor.get_dr(DoubleReg::D8),
            (d8 as u32, (d8 >> 32) as u32)
        );
        assert_eq!(
            processor.get_dr(DoubleReg::D9),
            (d9 as u32, (d9 >> 32) as u32)
        );
        assert_eq!(processor.get_r(Reg::SP), sp + 16);
    }

    #[test]
    fn test_exec_vldr_loads_single_register() {
        let mut processor = fp_test_processor();
        processor.write32(0x2000_0500, 1.25f32.to_bits()).unwrap();
        processor.set_r(Reg::R0, 0x2000_0500);

        processor
            .exec_vldr(&VLoadAndStoreParams {
                dd: ExtensionReg::Single { reg: SingleReg::S2 },
                rn: Reg::R0,
                add: true,
                imm32: 0,
            })
            .unwrap();

        assert_eq!(processor.get_sr(SingleReg::S2), 1.25f32.to_bits());
    }

    #[test]
    fn test_exec_vstr_stores_double_register() {
        let mut processor = fp_test_processor();
        let bits = (-3.5f64).to_bits();
        processor.set_dr(DoubleReg::D3, bits as u32, (bits >> 32) as u32);
        processor.set_r(Reg::R1, 0x2000_0600);

        processor
            .exec_vstr(&VLoadAndStoreParams {
                dd: ExtensionReg::Double { reg: DoubleReg::D3 },
                rn: Reg::R1,
                add: true,
                imm32: 0,
            })
            .unwrap();

        assert_eq!(processor.read32(0x2000_0600).unwrap(), bits as u32);
        assert_eq!(processor.read32(0x2000_0604).unwrap(), (bits >> 32) as u32);
    }

    #[test]
    fn test_exec_vpush_stores_single_registers_and_updates_sp() {
        let mut processor = fp_test_processor();
        processor.set_r(Reg::SP, 0x2000_0708);
        processor.set_sr(SingleReg::S0, 1.0f32.to_bits());
        processor.set_sr(SingleReg::S1, (-2.0f32).to_bits());

        let mut list = EnumSet::new();
        list.insert(SingleReg::S0);
        list.insert(SingleReg::S1);

        processor
            .exec_vpush(&VPushPopParams {
                single_regs: true,
                single_precision_registers: list,
                double_precision_registers: EnumSet::new(),
                imm32: 8,
            })
            .unwrap();

        assert_eq!(processor.get_r(Reg::SP), 0x2000_0700);
        assert_eq!(processor.read32(0x2000_0700).unwrap(), 1.0f32.to_bits());
        assert_eq!(processor.read32(0x2000_0704).unwrap(), (-2.0f32).to_bits());
    }

    #[test]
    fn test_exec_vstm_t1_stores_double_registers() {
        let mut processor = fp_test_processor();
        let bits = 6.5f64.to_bits();
        processor.set_r(Reg::R6, 0x2000_0800);
        processor.set_dr(DoubleReg::D10, bits as u32, (bits >> 32) as u32);

        let mut list = EnumSet::new();
        list.insert(DoubleReg::D10);

        processor
            .exec_vstm_t1(&VStoreMultipleParams64 {
                mode: AddressingMode::IncrementAfter,
                rn: Reg::R6,
                write_back: true,
                list,
                imm32: 8,
            })
            .unwrap();

        assert_eq!(processor.read32(0x2000_0800).unwrap(), bits as u32);
        assert_eq!(processor.read32(0x2000_0804).unwrap(), (bits >> 32) as u32);
        assert_eq!(processor.get_r(Reg::R6), 0x2000_0808);
    }

    #[test]
    fn test_exec_vstm_t2_stores_single_registers() {
        let mut processor = fp_test_processor();
        processor.set_r(Reg::R7, 0x2000_0900);
        processor.set_sr(SingleReg::S20, 4.0f32.to_bits());
        processor.set_sr(SingleReg::S21, (-5.0f32).to_bits());

        let mut list = EnumSet::new();
        list.insert(SingleReg::S20);
        list.insert(SingleReg::S21);

        processor
            .exec_vstm_t2(&VStoreMultipleParams32 {
                mode: AddressingMode::IncrementAfter,
                rn: Reg::R7,
                write_back: true,
                list,
                imm32: 8,
            })
            .unwrap();

        assert_eq!(processor.read32(0x2000_0900).unwrap(), 4.0f32.to_bits());
        assert_eq!(processor.read32(0x2000_0904).unwrap(), (-5.0f32).to_bits());
        assert_eq!(processor.get_r(Reg::R7), 0x2000_0908);
    }
}
