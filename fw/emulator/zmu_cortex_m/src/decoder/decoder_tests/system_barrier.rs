use super::*;

#[test]
fn test_decode_dmb() {
    //  f3bf 8f5f       dmb sy
    assert_eq!(decode_32(0xf3bf_8f5f), Instruction::DMB);
}

#[test]
fn test_decode_it() {
    // ITT MI
    assert_eq!(
        decode_16(0xbf44),
        Instruction::IT {
            x: Some(ITCondition::Then),
            y: None,
            z: None,
            firstcond: Condition::MI,
            mask: 0x4,
        }
    );
}

#[test]
fn test_decode_itt_cc() {
    // 0xbf3c ITTCC
    assert_eq!(
        decode_16(0xbf3c),
        Instruction::IT {
            x: Some(ITCondition::Then),
            y: None,
            z: None,
            firstcond: Condition::CC,
            mask: 0b1100,
        }
    );
}

#[test]
fn test_decode_itttt_cc() {
    // 0xbf3f ITTTT CC
    assert_eq!(
        decode_16(0xbf3f),
        Instruction::IT {
            x: Some(ITCondition::Then),
            y: Some(ITCondition::Then),
            z: Some(ITCondition::Then),
            firstcond: Condition::CC,
            mask: 0b1111,
        }
    );
}
