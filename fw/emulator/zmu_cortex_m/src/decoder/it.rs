use crate::core::bits::Bits;
use crate::core::condition::Condition;
use crate::core::instruction::ITCondition;
use crate::core::instruction::Instruction;

#[allow(non_snake_case)]
#[allow(clippy::too_many_lines)]
pub fn decode_IT_t1(opcode: u16) -> Instruction {
    let firstcond = opcode.get_bits(4..8);
    let mask = opcode.get_bits(0..4) as u8;

    let lsb = opcode.get_bit(4);

    let (x, y, z) = if lsb {
        match mask {
            0b1000 => (None, None, None),
            0b1100 => (Some(ITCondition::Then), None, None),
            0b0100 => (Some(ITCondition::Else), None, None),
            0b1110 => (Some(ITCondition::Then), Some(ITCondition::Then), None),
            0b0110 => (Some(ITCondition::Else), Some(ITCondition::Then), None),
            0b1010 => (Some(ITCondition::Then), Some(ITCondition::Else), None),
            0b0010 => (Some(ITCondition::Else), Some(ITCondition::Else), None),

            0b1111 => (
                Some(ITCondition::Then),
                Some(ITCondition::Then),
                Some(ITCondition::Then),
            ),
            0b0111 => (
                Some(ITCondition::Else),
                Some(ITCondition::Then),
                Some(ITCondition::Then),
            ),
            0b1011 => (
                Some(ITCondition::Then),
                Some(ITCondition::Else),
                Some(ITCondition::Then),
            ),
            0b0011 => (
                Some(ITCondition::Else),
                Some(ITCondition::Else),
                Some(ITCondition::Then),
            ),
            0b1101 => (
                Some(ITCondition::Then),
                Some(ITCondition::Then),
                Some(ITCondition::Else),
            ),
            0b0101 => (
                Some(ITCondition::Else),
                Some(ITCondition::Then),
                Some(ITCondition::Else),
            ),
            0b1001 => (
                Some(ITCondition::Then),
                Some(ITCondition::Else),
                Some(ITCondition::Else),
            ),
            0b0001 => (
                Some(ITCondition::Else),
                Some(ITCondition::Else),
                Some(ITCondition::Else),
            ),
            _ => (None, None, None),
        }
    } else {
        match mask {
            0b1000 => (None, None, None),
            0b0100 => (Some(ITCondition::Then), None, None),
            0b1100 => (Some(ITCondition::Else), None, None),
            0b0010 => (Some(ITCondition::Then), Some(ITCondition::Then), None),
            0b1010 => (Some(ITCondition::Else), Some(ITCondition::Then), None),
            0b0110 => (Some(ITCondition::Then), Some(ITCondition::Else), None),
            0b1110 => (Some(ITCondition::Else), Some(ITCondition::Else), None),

            0b0001 => (
                Some(ITCondition::Then),
                Some(ITCondition::Then),
                Some(ITCondition::Then),
            ),
            0b1001 => (
                Some(ITCondition::Else),
                Some(ITCondition::Then),
                Some(ITCondition::Then),
            ),
            0b0101 => (
                Some(ITCondition::Then),
                Some(ITCondition::Else),
                Some(ITCondition::Then),
            ),
            0b1101 => (
                Some(ITCondition::Else),
                Some(ITCondition::Else),
                Some(ITCondition::Then),
            ),
            0b0011 => (
                Some(ITCondition::Then),
                Some(ITCondition::Then),
                Some(ITCondition::Else),
            ),
            0b1011 => (
                Some(ITCondition::Else),
                Some(ITCondition::Then),
                Some(ITCondition::Else),
            ),
            0b0111 => (
                Some(ITCondition::Then),
                Some(ITCondition::Else),
                Some(ITCondition::Else),
            ),
            0b1111 => (
                Some(ITCondition::Else),
                Some(ITCondition::Else),
                Some(ITCondition::Else),
            ),
            _ => (None, None, None),
        }
    };

    Instruction::IT {
        x,
        y,
        z,
        firstcond: Condition::from_u16(firstcond).unwrap_or(Condition::AL),
        mask,
    }
}
