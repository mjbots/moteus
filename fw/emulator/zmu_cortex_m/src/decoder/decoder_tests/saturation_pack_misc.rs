use super::*;

#[test]
fn test_decode_bfc() {
    //  f36f 011f       bfc     r1, #0, #32
    assert_eq!(
        decode_32(0xf36f_011f),
        Instruction::BFC {
            params: BfcParams {
                rd: Reg::R1,
                lsbit: 0,
                msbit: 31,
            }
        }
    );
}

#[test]
fn test_decode_bfi_w() {
    // 0xf363_0407 BFI R4, R3, #0, #8
    assert_eq!(
        decode_32(0xf363_0407),
        Instruction::BFI {
            params: BfiParams {
                rd: Reg::R4,
                rn: Reg::R3,
                lsbit: 0,
                width: 8,
            }
        }
    );
}

#[test]
fn test_decode_clz_w() {
    //0xfab0_f180 -> CLZ R1, R0
    assert_eq!(
        decode_32(0xfab0_f180),
        Instruction::CLZ {
            params: Reg2RdRmParams {
                rd: Reg::R1,
                rm: Reg::R0,
            }
        }
    );
}

#[test]
fn test_decode_sbfx() {
    // SBFX    r3, r3, #0, #1
    assert_eq!(
        decode_32(0xf343_0300),
        Instruction::SBFX {
            params: BfxParams {
                rd: Reg::R3,
                rn: Reg::R3,
                lsb: 0,
                widthminus1: 0,
            }
        }
    );
}

#[test]
fn test_decode_sxth_reg() {
    // SXTH R1,R1
    match decode_16(0xb209) {
        Instruction::SXTH { params, thumb32 } => {
            assert_eq!(params.rd, Reg::R1);
            assert_eq!(params.rm, Reg::R1);
            assert!(!thumb32);
            assert_eq!(params.rotation, 0);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_sxth_w() {
    // SXTH.W R10, R10
    assert_eq!(
        decode_32(0xfa0f_fa8a),
        Instruction::SXTH {
            params: Reg2UsizeParams {
                rd: Reg::R10,
                rm: Reg::R10,
                rotation: 0
            },
            thumb32: true,
        }
    );
}

#[test]
#[cfg(feature = "has-dsp-ext")]
fn test_decode_uadd8() {
    // fa82 f24c       uadd8   r2, r2, ip
    assert_eq!(
        decode_32(0xfa82_f24c),
        Instruction::UADD8 {
            params: Reg3NoSetFlagsParams {
                rd: Reg::R2,
                rn: Reg::R2,
                rm: Reg::R12,
            }
        }
    );
}

// Without the DSP extension the UADD8 encoding is not in the decoder table;
// it must fall through to UDF so that non-DSP builds treat it as undefined.
#[test]
#[cfg(not(feature = "has-dsp-ext"))]
fn test_decode_uadd8_without_dsp_ext_is_udf() {
    // fa82 f24c  UADD8 R2, R2, IP  — DSP-only encoding
    match decode_32(0xfa82_f24c) {
        Instruction::UDF { thumb32, .. } => assert!(thumb32),
        other => panic!("expected UDF for UADD8 encoding without DSP extension, got {other:?}"),
    }
}

#[test]
fn test_decode_ubfx() {
    // UBFX R1, R0, #1, #1
    assert_eq!(
        decode_32(0xf3c0_0140),
        Instruction::UBFX {
            params: BfxParams {
                rd: Reg::R1,
                rn: Reg::R0,
                lsb: 1,
                widthminus1: 0,
            }
        }
    );
}

#[test]
fn test_decode_uxtab_() {
    //0xfa54_f480 UXTAB.W R4, R4, R0

    assert_eq!(
        decode_32(0xfa54_f480),
        Instruction::UXTAB {
            params: Reg3UsizeParams {
                rd: Reg::R4,
                rn: Reg::R4,
                rm: Reg::R0,
                rotation: 0
            }
        }
    );
}

#[test]
fn test_decode_uxtb() {
    // UXTB R1,R1
    match decode_16(0xb2c9) {
        Instruction::UXTB { params, thumb32 } => {
            assert!(params.rd == Reg::R1);
            assert!(params.rm == Reg::R1);
            assert!(!thumb32);
            assert!(params.rotation == 0);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_uxtb_w() {
    //0xfa5f_f989 UXTB.W R9, R9
    assert_eq!(
        decode_32(0xfa5f_f989),
        Instruction::UXTB {
            params: Reg2UsizeParams {
                rd: Reg::R9,
                rm: Reg::R9,
                rotation: 0,
            },
            thumb32: true,
        }
    );
}
