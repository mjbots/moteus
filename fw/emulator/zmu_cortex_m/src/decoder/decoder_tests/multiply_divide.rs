use super::*;

#[test]
fn test_decode_mla() {
    // MLA R1, R7, R2, R1
    assert_eq!(
        decode_32(0xfb07_1102),
        Instruction::MLA {
            params: Reg4NoSetFlagsParams {
                rd: Reg::R1,
                rn: Reg::R7,
                rm: Reg::R2,
                ra: Reg::R1,
            }
        }
    );
}

#[test]
fn test_decode_mls() {
    // 0xfb02_921a MLS R2, R2, R10, R9
    assert_eq!(
        decode_32(0xfb02_921a),
        Instruction::MLS {
            params: Reg4NoSetFlagsParams {
                rd: Reg::R2,
                rn: Reg::R2,
                rm: Reg::R10,
                ra: Reg::R9,
            }
        }
    );
}

#[test]
fn test_decode_mul() {
    // MULS R4, R0, R4
    match decode_16(0x4344) {
        Instruction::MUL { params, thumb32 } => {
            assert!(params.rd == Reg::R4);
            assert!(params.rn == Reg::R0);
            assert!(params.rm == Reg::R4);
            assert!(params.setflags == SetFlags::NotInITBlock);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_mul_w() {
    //0xfb04_f604 MUL R6, R4, R4
    assert_eq!(
        decode_32(0xfb04_f604),
        Instruction::MUL {
            params: Reg3Params {
                rd: Reg::R6,
                rn: Reg::R4,
                rm: Reg::R4,
                setflags: SetFlags::False,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_sdiv() {
    // 0xfb99_f2fa SDIV, R2, R9, R10
    assert_eq!(
        decode_32(0xfb99_f2fa),
        Instruction::SDIV {
            params: Reg3NoSetFlagsParams {
                rd: Reg::R2,
                rn: Reg::R9,
                rm: Reg::R10,
            }
        }
    );
}

#[test]
#[cfg(feature = "has-dsp-ext")]
fn test_decode_smla_bb() {
    // 0xfb15_ee0b -> SMLABB LR, R5, R11, LR
    assert_eq!(
        decode_32(0xfb15_ee0b),
        Instruction::SMLA {
            params: Reg4HighParams {
                rd: Reg::LR,
                rn: Reg::R5,
                rm: Reg::R11,
                ra: Reg::LR,
                n_high: false,
                m_high: false
            }
        }
    );
}

// Without the DSP extension the SMLA encoding must fall through to UDF.
#[test]
#[cfg(not(feature = "has-dsp-ext"))]
fn test_decode_smla_bb_without_dsp_ext_is_udf() {
    // 0xfb15_ee0b  SMLABB LR, R5, R11, LR  — DSP-only encoding
    match decode_32(0xfb15_ee0b) {
        Instruction::UDF { thumb32, .. } => assert!(thumb32),
        other => panic!("expected UDF for SMLA encoding without DSP extension, got {other:?}"),
    }
}

#[test]
#[cfg(feature = "has-dsp-ext")]
fn test_decode_smul_bb() {
    // 0xfb1e_fe08 -> SMULBB LR, LR, R8
    assert_eq!(
        decode_32(0xfb1e_fe08),
        Instruction::SMUL {
            params: Reg3HighParams {
                rd: Reg::LR,
                rn: Reg::LR,
                rm: Reg::R8,
                n_high: false,
                m_high: false
            }
        }
    );
}

// Without the DSP extension the SMUL encoding must fall through to UDF.
#[test]
#[cfg(not(feature = "has-dsp-ext"))]
fn test_decode_smul_bb_without_dsp_ext_is_udf() {
    // 0xfb1e_fe08  SMULBB LR, LR, R8  — DSP-only encoding
    match decode_32(0xfb1e_fe08) {
        Instruction::UDF { thumb32, .. } => assert!(thumb32),
        other => panic!("expected UDF for SMUL encoding without DSP extension, got {other:?}"),
    }
}

#[test]
fn test_decode_smull() {
    // fb83 320b       smull   r3, r2, r3, fp
    assert_eq!(
        decode_32(0xfb83_320b),
        Instruction::SMULL {
            params: Reg643232Params {
                rdlo: Reg::R3,
                rdhi: Reg::R2,
                rn: Reg::R3,
                rm: Reg::R11,
            }
        }
    );
}

#[test]
fn test_decode_udiv() {
    // UDIV R0, R0, R1
    assert_eq!(
        decode_32(0xfbb0_f0f1),
        Instruction::UDIV {
            params: Reg3NoSetFlagsParams {
                rd: Reg::R0,
                rn: Reg::R0,
                rm: Reg::R1,
            }
        }
    );
}

#[test]
fn test_decode_ulmull() {
    // 0xfba4_2300 -> UMULL R2, R3, R4, R0
    assert_eq!(
        decode_32(0xfba4_2300),
        Instruction::UMULL {
            params: Reg643232Params {
                rdlo: Reg::R2,
                rdhi: Reg::R3,
                rn: Reg::R4,
                rm: Reg::R0,
            }
        }
    );
}
