use crate::Processor;

use crate::core::fault::FaultStatusContext;
use crate::executor::{ExecuteSuccess, ExecutorHelper};

use super::{ExecuteResult, resolve_addressing};

use crate::{
    bus::Bus,
    core::{
        bits::Bits,
        fault::Fault,
        instruction::{
            Reg2DoubleParams, Reg2FullParams, Reg2RtRnImm32Params, Reg2RtRnParams, Reg3FullParams,
            Reg3RdRtRnImm32Params, Reg3RdRtRnParams, RegImm32AddParams,
        },
        monitor::Monitor,
        operation::{shift, sign_extend, zero_extend, zero_extend_u16},
        register::{Apsr, BaseReg, Reg},
    },
};

impl Processor {
    fn read8_data(&mut self, address: u32) -> Result<u8, Fault> {
        self.read8(address).map_err(|fault| {
            self.fault_with_status(fault, FaultStatusContext::with_fault_address(address))
        })
    }

    fn read16_data(&mut self, address: u32) -> Result<u16, Fault> {
        self.read16(address).map_err(|fault| {
            self.fault_with_status(fault, FaultStatusContext::with_fault_address(address))
        })
    }

    fn read32_data(&mut self, address: u32) -> Result<u32, Fault> {
        self.read32(address).map_err(|fault| {
            self.fault_with_status(fault, FaultStatusContext::with_fault_address(address))
        })
    }

    fn write8_data(&mut self, address: u32, value: u8) -> Result<(), Fault> {
        self.write8(address, value).map_err(|fault| {
            self.fault_with_status(fault, FaultStatusContext::with_fault_address(address))
        })
    }

    fn write16_data(&mut self, address: u32, value: u16) -> Result<(), Fault> {
        self.write16(address, value).map_err(|fault| {
            self.fault_with_status(fault, FaultStatusContext::with_fault_address(address))
        })
    }

    fn write32_data(&mut self, address: u32, value: u32) -> Result<(), Fault> {
        self.write32(address, value).map_err(|fault| {
            self.fault_with_status(fault, FaultStatusContext::with_fault_address(address))
        })
    }
}

/// Load and Store operations
pub trait IsaLoadAndStore {
    fn exec_ldr_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult;
    fn exec_ldrb_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult;
    fn exec_ldrh_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult;
    fn exec_ldrsb_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult;
    fn exec_ldrsh_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult;

    fn exec_str_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult;
    fn exec_strb_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult;
    fn exec_strh_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult;

    fn exec_ldr_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult;
    fn exec_ldrb_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult;
    fn exec_ldrh_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult;
    fn exec_ldrsb_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult;
    fn exec_ldrsh_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult;

    fn exec_str_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult;
    fn exec_strb_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult;
    fn exec_strh_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult;

    fn exec_ldrex(&mut self, params: Reg2RtRnImm32Params) -> ExecuteResult;
    fn exec_ldrexb(&mut self, params: Reg2RtRnParams) -> ExecuteResult;
    fn exec_ldrexh(&mut self, params: Reg2RtRnParams) -> ExecuteResult;

    fn exec_strex(&mut self, params: Reg3RdRtRnImm32Params) -> ExecuteResult;
    fn exec_strexb(&mut self, params: Reg3RdRtRnParams) -> ExecuteResult;
    fn exec_strexh(&mut self, params: Reg3RdRtRnParams) -> ExecuteResult;

    fn exec_ldrd_imm(&mut self, params: &Reg2DoubleParams) -> ExecuteResult;
    fn exec_strd_imm(&mut self, params: &Reg2DoubleParams) -> ExecuteResult;

    fn exec_ldr_lit(&mut self, params: &RegImm32AddParams) -> ExecuteResult;
}

impl IsaLoadAndStore for Processor {
    fn exec_ldr_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();
            let offset = shift(rm, params.shift_t, params.shift_n as usize, c);

            let rn = self.get_r(params.rn);
            let (address, offset_address) =
                resolve_addressing(rn, offset, params.add, params.index);

            let data = self.read32_data(address)?;
            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            if params.rt == Reg::PC {
                self.load_write_pc(data)?;
                return Ok(ExecuteSuccess::Branched { cycles: 2 });
            } else {
                self.set_r(params.rt, data);
                return Ok(ExecuteSuccess::Taken { cycles: 2 });
            }
        }

        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_ldrb_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();

            let offset = shift(rm, params.shift_t, params.shift_n as usize, c);

            let (address, offset_address) =
                resolve_addressing(self.get_r(params.rn), offset, params.add, params.index);

            let data = u32::from(self.read8_data(address)?);
            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            self.set_r(params.rt, data);
            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_ldrh_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();
            let offset = shift(rm, params.shift_t, params.shift_n as usize, c);

            let (address, offset_address) =
                resolve_addressing(self.get_r(params.rn), offset, params.add, params.index);

            let data = u32::from(self.read16_data(address)?);
            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            self.set_r(params.rt, data);
            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_ldrsh_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();
            let offset = shift(rm, params.shift_t, params.shift_n as usize, c);

            let (address, offset_address) =
                resolve_addressing(self.get_r(params.rn), offset, params.add, params.index);

            let data = u32::from(self.read16_data(address)?);
            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            self.set_r(params.rt, sign_extend(data, 15, 32) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_ldrsb_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let rm = self.get_r(params.rm);
            let c = self.psr.get_c();
            let offset = shift(rm, params.shift_t, params.shift_n as usize, c);

            let (address, offset_address) =
                resolve_addressing(self.get_r(params.rn), offset, params.add, params.index);

            let data = u32::from(self.read8_data(address)?);
            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            self.set_r(params.rt, sign_extend(data, 7, 32) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_str_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let offset = shift(
                self.get_r(params.rm),
                params.shift_t,
                params.shift_n as usize,
                c,
            );
            let address = self.get_r(params.rn).wrapping_add(offset);
            let value = self.get_r(params.rt);
            self.write32_data(address, value)?;

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_strb_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let offset = shift(
                self.get_r(params.rm),
                params.shift_t,
                params.shift_n as usize,
                c,
            );
            let address = self.get_r(params.rn).wrapping_add(offset);
            let rt: u32 = self.get_r(params.rt);
            let value = rt.get_bits(0..8);
            self.write8_data(address, value as u8)?;
            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_strh_reg(&mut self, params: &Reg3FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let c = self.psr.get_c();
            let offset = shift(
                self.get_r(params.rm),
                params.shift_t,
                params.shift_n as usize,
                c,
            );
            let address = self.get_r(params.rn).wrapping_add(offset);
            let value = self.get_r(params.rt).get_bits(0..16);
            self.write16_data(address, value as u16)?;
            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    #[inline(always)]
    fn exec_ldr_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let offset_address = if params.add {
                rn.wrapping_add(params.imm32)
            } else {
                rn.wrapping_sub(params.imm32)
            };
            let address = if params.index { offset_address } else { rn };

            let data = self.read32_data(address)?;
            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            if params.rt == Reg::PC {
                self.load_write_pc(data)?;
                return Ok(ExecuteSuccess::Branched { cycles: 2 });
            } else {
                self.set_r(params.rt, data);
                return Ok(ExecuteSuccess::Taken { cycles: 2 });
            }
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    #[inline(always)]
    fn exec_ldrb_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let offset_address = if params.add {
                rn.wrapping_add(params.imm32)
            } else {
                rn.wrapping_sub(params.imm32)
            };
            let address = if params.index { offset_address } else { rn };

            let data = self.read8_data(address)?;
            self.set_r(params.rt, u32::from(data));

            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_ldrh_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let (address, offset_address) = resolve_addressing(
                self.get_r(params.rn),
                params.imm32,
                params.add,
                params.index,
            );

            let data = self.read16_data(address)?;
            if params.wback {
                self.set_r(params.rn, offset_address);
            }
            self.set_r(params.rt, u32::from(data));

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_ldrsb_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let (address, offset_address) = resolve_addressing(
                self.get_r(params.rn),
                params.imm32,
                params.add,
                params.index,
            );

            let data = self.read8_data(address)?;
            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            self.set_r(params.rt, sign_extend(data.into(), 7, 32) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    #[inline(always)]
    fn exec_ldrsh_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let offset_address = if params.add {
                rn.wrapping_add(params.imm32)
            } else {
                rn.wrapping_sub(params.imm32)
            };
            let address = if params.index { offset_address } else { rn };

            let data = self.read16_data(address)?;
            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            self.set_r(params.rt, sign_extend(u32::from(data), 15, 32) as u32);
            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    #[inline(always)]
    fn exec_str_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let rn = self.get_r(params.rn);
            let offset_address = if params.add {
                rn.wrapping_add(params.imm32)
            } else {
                rn.wrapping_sub(params.imm32)
            };
            let address = if params.index { offset_address } else { rn };

            let value = self.get_r(params.rt);
            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            self.write32_data(address, value)?;

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_strb_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let (address, offset_address) = resolve_addressing(
                self.get_r(params.rn),
                params.imm32,
                params.add,
                params.index,
            );

            let value = self.get_r(params.rt);
            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            self.write8_data(address, value.get_bits(0..8) as u8)?;

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_strh_imm(&mut self, params: &Reg2FullParams) -> ExecuteResult {
        if self.condition_passed() {
            let (address, offset_address) = resolve_addressing(
                self.get_r(params.rn),
                params.imm32,
                params.add,
                params.index,
            );

            let value = self.get_r(params.rt);
            self.write16_data(address, value.get_bits(0..16) as u16)?;

            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_ldrex(&mut self, params: Reg2RtRnImm32Params) -> ExecuteResult {
        if self.condition_passed() {
            let (address, _) = resolve_addressing(self.get_r(params.rn), params.imm32, true, true);

            self.set_exclusive_monitors(address, 4);

            let data = self.read32_data(address)?;
            self.set_r(params.rt, data);

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_ldrexb(&mut self, params: Reg2RtRnParams) -> ExecuteResult {
        if self.condition_passed() {
            let address = self.get_r(params.rn);
            self.set_exclusive_monitors(address, 1);

            let data = self.read8_data(address)?;

            let data_params = [data];
            let lengths = [32];
            self.set_r(params.rt, zero_extend(&data_params, &lengths));

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_ldrexh(&mut self, params: Reg2RtRnParams) -> ExecuteResult {
        if self.condition_passed() {
            let address = self.get_r(params.rn);
            self.set_exclusive_monitors(address, 2);

            let data = self.read16_data(address)?;

            let data_params = [data];
            let lengths = [32];
            self.set_r(params.rt, zero_extend_u16(&data_params, &lengths));

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_strex(&mut self, params: Reg3RdRtRnImm32Params) -> ExecuteResult {
        if self.condition_passed() {
            let (address, _) = resolve_addressing(self.get_r(params.rn), params.imm32, true, true);

            if self.exclusive_monitors_pass(address, 4) {
                self.write32_data(address, self.get_r(params.rt))?;
                self.set_r(params.rd, 0);
            } else {
                self.set_r(params.rd, 1);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_strexb(&mut self, params: Reg3RdRtRnParams) -> ExecuteResult {
        if self.condition_passed() {
            if params.strex_is_unpredictable() {
                return Err(Fault::UndefInstr);
            }
            let address = self.get_r(params.rn);

            if self.exclusive_monitors_pass(address, 1) {
                self.write8_data(address, self.get_r(params.rt) as u8)?;
                self.set_r(params.rd, 0);
            } else {
                self.set_r(params.rd, 1);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_strexh(&mut self, params: Reg3RdRtRnParams) -> ExecuteResult {
        if self.condition_passed() {
            if params.strex_is_unpredictable() {
                return Err(Fault::UndefInstr);
            }
            let address = self.get_r(params.rn);

            if self.exclusive_monitors_pass(address, 2) {
                self.write16_data(address, self.get_r(params.rt) as u16)?;
                self.set_r(params.rd, 0);
            } else {
                self.set_r(params.rd, 1);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
    fn exec_ldrd_imm(&mut self, params: &Reg2DoubleParams) -> ExecuteResult {
        if self.condition_passed() {
            let (address, offset_address) = resolve_addressing(
                self.get_r(params.rn),
                params.imm32,
                params.add,
                params.index,
            );

            let data = self.read32_data(address)?;
            self.set_r(params.rt, data);
            let data2 = self.read32_data(address + 4)?;
            self.set_r(params.rt2, data2);

            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_strd_imm(&mut self, params: &Reg2DoubleParams) -> ExecuteResult {
        if self.condition_passed() {
            let (address, offset_address) = resolve_addressing(
                self.get_r(params.rn),
                params.imm32,
                params.add,
                params.index,
            );

            let value1 = self.get_r(params.rt);
            self.write32_data(address, value1)?;
            let value2 = self.get_r(params.rt2);
            self.write32_data(address + 4, value2)?;

            if params.wback {
                self.set_r(params.rn, offset_address);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }

    fn exec_ldr_lit(&mut self, params: &RegImm32AddParams) -> ExecuteResult {
        if self.condition_passed() {
            let base = self.get_r(Reg::PC) & 0xffff_fffc;
            let address = if params.add {
                base + params.imm32
            } else {
                base - params.imm32
            };
            let data = self.read32_data(address)?;

            if params.rt == Reg::PC {
                self.load_write_pc(data)?;
            } else {
                self.set_r(params.rt, data);
            }

            return Ok(ExecuteSuccess::Taken { cycles: 2 });
        }
        Ok(ExecuteSuccess::NotTaken)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{fault::Fault, instruction::SRType};

    #[test]
    fn test_exec_str_reg_wraps_address_calculation_instead_of_panicking() {
        let mut core = Processor::new();

        core.set_r(Reg::R0, 0x1234_5678);
        core.set_r(Reg::R1, u32::MAX);
        core.set_r(Reg::R2, 1);

        let result = core.exec_str_reg(&Reg3FullParams {
            rt: Reg::R0,
            rn: Reg::R1,
            rm: Reg::R2,
            shift_t: SRType::LSL,
            shift_n: 0,
            index: true,
            add: true,
            wback: false,
        });

        assert_eq!(result, Err(Fault::DAccViol));
    }
}
