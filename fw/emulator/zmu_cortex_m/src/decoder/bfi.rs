use crate::core::bits::Bits;
use crate::core::instruction::{BfiParams, Instruction};
use crate::core::register::Reg;

#[allow(non_snake_case)]
pub fn decode_BFI_t1(opcode: u32) -> Instruction {
    let rn: u8 = opcode.get_bits(16..20) as u8;
    let rd: u8 = opcode.get_bits(8..12) as u8;

    let imm3: u8 = opcode.get_bits(12..15) as u8;
    let imm2: u8 = opcode.get_bits(6..8) as u8;

    let lsbit = u32::from((imm3 << 2) + imm2);
    let msbit = opcode.get_bits(0..5);

    // msbit = lsbit + width -1   <=>
    // width = msbit - lsbit + 1
    // Some opcode patterns routed here can still be invalid (`msbit < lsbit`).
    // In that case decode as UDF instead of underflowing in debug builds.
    let width = match msbit.checked_sub(lsbit) {
        Some(delta) => delta + 1,
        None => {
            return Instruction::UDF {
                imm32: 0,
                opcode: opcode.into(),
                thumb32: true,
            };
        }
    };

    Instruction::BFI {
        params: BfiParams {
            rd: Reg::from(rd),
            rn: Reg::from(rn),
            lsbit: lsbit as usize,
            width: width as usize,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_bfi_invalid_msbit_is_udf() {
        let opcode = 0x0000_70C0;
        match decode_BFI_t1(opcode) {
            Instruction::UDF { thumb32, .. } => assert!(thumb32),
            other => panic!("expected UDF for invalid BFI encoding, got {other:?}"),
        }
    }
}
