use super::*;

#[test]
fn test_decode_b() {
    // BEQ.N
    if let Instruction::B_t13 { params, thumb32 } = decode_16(0xd001) {
        assert_eq!(params.cond, Condition::EQ);
        assert_eq!(params.imm32, (1 << 1));
        assert!(!thumb32);
    } else {
        println!(" {}", decode_16(0xd001));
        unreachable!();
    }
    // BNE.N
    match decode_16(0xd1f8) {
        Instruction::B_t13 { params, thumb32 } => {
            assert!(params.cond == Condition::NE);
            assert!(params.imm32 == -16);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    // B.N (PC + 8)
    match decode_16(0xE004) {
        Instruction::B_t24 { imm32, thumb32 } => {
            assert!(imm32 == (4 << 1));
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_bic() {
    // BICS R2,R2,R0
    match decode_16(0x4382) {
        Instruction::BIC_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R2);
            assert!(params.rn == Reg::R2);
            assert!(params.rm == Reg::R0);
            assert!(params.setflags == SetFlags::NotInITBlock);
            assert!(!thumb32);
            assert_eq!(params.shift_t, SRType::LSL);
            assert_eq!(params.shift_n, 0);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_bic_imm_w() {
    //0xf024_00ff -> BIC.W R0, R4, #255

    assert_eq!(
        decode_32(0xf024_00ff),
        Instruction::BIC_imm {
            params: Reg2ImmCarryParams {
                rd: Reg::R0,
                rn: Reg::R4,
                imm32: Imm32Carry::Carry {
                    imm32_c0: (255, false),
                    imm32_c1: (255, true),
                },
                setflags: false,
            }
        }
    );
}

#[test]
fn test_decode_bic_reg_w() {
    //0xea23_5345 -> BIC.W R3, R3, R5, LSL #21

    assert_eq!(
        decode_32(0xea23_5345),
        Instruction::BIC_reg {
            params: Reg3ShiftParams {
                rd: Reg::R3,
                rn: Reg::R3,
                rm: Reg::R5,
                setflags: SetFlags::False,
                shift_t: SRType::LSL,
                shift_n: 21,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_bkpt() {
    // BKPT #0xab
    assert_eq!(decode_16(0xbeab), Instruction::BKPT { imm32: 0xab });
}

#[cfg(feature = "armv6m")]
#[test]
fn test_decode_cpsid() {
    // CPSID i
    assert_eq!(decode_16(0xB672), Instruction::CPS { im: true });
}

#[cfg(any(feature = "armv7m", feature = "armv7em"))]
#[test]
fn test_decode_cpsid() {
    // CPSID i
    assert_eq!(
        decode_16(0xB672),
        Instruction::CPS {
            im: true,
            affect_pri: true,
            affect_fault: false
        }
    );
}

#[test]
fn test_decode_nop() {
    // NOP
    match decode_16(0xbf00) {
        Instruction::NOP { thumb32 } => {
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_pld_reg() {
    // 0xf890_f000 pld [r0]

    assert_eq!(
        decode_32(0xf890_f000),
        Instruction::PLD_imm {
            rn: Reg::R0,
            imm32: 0,
            add: true
        }
    );
}

#[test]
#[cfg(feature = "has-dsp-ext")]
fn test_decode_sel() {
    //0xfaa4_f28c       sel     r2, r4, ip

    assert_eq!(
        decode_32(0xfaa4_f28c),
        Instruction::SEL {
            params: Reg3NoSetFlagsParams {
                rd: Reg::R2,
                rn: Reg::R4,
                rm: Reg::R12,
            }
        }
    );
}

// Without the DSP extension the SEL encoding must fall through to UDF.
#[test]
#[cfg(not(feature = "has-dsp-ext"))]
fn test_decode_sel_without_dsp_ext_is_udf() {
    // 0xfaa4_f28c  SEL R2, R4, IP  — DSP-only encoding
    match decode_32(0xfaa4_f28c) {
        Instruction::UDF { thumb32, .. } => assert!(thumb32),
        other => panic!("expected UDF for SEL encoding without DSP extension, got {other:?}"),
    }
}

#[test]
fn test_is_thumb32() {
    assert!(is_thumb32(0b1110_1000_0000_0000));
    assert!(is_thumb32(0b1111_0000_0000_0000));
    assert!(is_thumb32(0b1111_1000_0000_0000));
    assert!(!is_thumb32(0b1110_0000_0000_0000));
    assert!(is_thumb32(0b1111_1111_1111_1111));
    assert!(!is_thumb32(0b0000_0000_0000_0001));
}
