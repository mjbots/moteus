use super::*;

#[test]
fn test_decode_b_pl_w() {
    //0xf57f_ad69 -> BPL.W -1326
    assert_eq!(
        decode_32(0xf57f_ad69),
        Instruction::B_t13 {
            params: CondBranchParams {
                cond: Condition::PL,
                imm32: -1326,
            },
            thumb32: true
        }
    );
}

#[test]
fn test_decode_bl_t1() {
    // BL -130
    assert_eq!(decode_32(0xf7ff_ffbf), Instruction::BL { imm32: -130 });

    // BL -5694
    assert_eq!(decode_32(0xf7fe_fce1), Instruction::BL { imm32: -5694 });
}

#[test]
fn test_decode_bx() {
    //BX LR
    match decode_16(0x4770) {
        Instruction::BX { rm } => {
            assert!(rm == Reg::LR);
        }
        _ => {
            unreachable!();
        }
    }
    //BX R1
    match decode_16(0x4708) {
        Instruction::BX { rm } => {
            assert!(rm == Reg::R1);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_cbnz() {
    // bb4b            cbnz    r3, (82 offset)
    assert_eq!(
        decode_16(0xbb4b),
        Instruction::CBNZ {
            params: ParamsRegImm32 {
                rn: Reg::R3,
                imm32: 82,
            }
        }
    );
}

#[test]
fn test_decode_cbz() {
    // CBZ R1, 0x3be4 (executed on addr 0x3bc2)
    assert_eq!(
        decode_16(0xb179),
        Instruction::CBZ {
            params: ParamsRegImm32 {
                rn: Reg::R1,
                imm32: 30,
            }
        }
    );
}

#[test]
fn test_decode_rsb_imm() {
    // RSB R2, R0, #0
    assert_eq!(
        decode_16(0x4242),
        Instruction::RSB_imm {
            params: Reg2ImmParams {
                rd: Reg::R2,
                rn: Reg::R0,
                imm32: 0,
                setflags: SetFlags::NotInITBlock,
            },
            thumb32: false
        }
    );
}

#[test]
fn test_decode_rsb_reg_w() {
    //0xebc0_1046 -> RSB.W R0, R0, R6, LSL #5
    assert_eq!(
        decode_32(0xebc0_1046),
        Instruction::RSB_reg {
            params: Reg3ShiftParams {
                rd: Reg::R0,
                rn: Reg::R0,
                rm: Reg::R6,
                setflags: SetFlags::False,
                shift_t: SRType::LSL,
                shift_n: 5,
            },
            thumb32: true,
        }
    );
    //0xebd7_20c0 -> RSBS.W R0, R7, R0, LSL #11
    assert_eq!(
        decode_32(0xebd7_20c0),
        Instruction::RSB_reg {
            params: Reg3ShiftParams {
                rd: Reg::R0,
                rn: Reg::R7,
                rm: Reg::R0,
                setflags: SetFlags::True,
                shift_t: SRType::LSL,
                shift_n: 11,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_rsb_w_reg() {
    //0xf1c6_003c -> RSB.W R0, R6, #60
    assert_eq!(
        decode_32(0xf1c6_003c),
        Instruction::RSB_imm {
            params: Reg2ImmParams {
                rd: Reg::R0,
                rn: Reg::R6,
                imm32: 60,
                setflags: SetFlags::False,
            },
            thumb32: true
        }
    );
}

#[test]
fn test_decode_tbb() {
    // TBB [PC, R0]
    assert_eq!(
        decode_32(0xe8df_f000),
        Instruction::TBB {
            params: Reg2RnRmParams {
                rn: Reg::PC,
                rm: Reg::R0,
            }
        }
    );
}

#[test]
fn test_decode_tbh() {
    // e8df f013       tbh     [pc, r3, lsl #1]

    assert_eq!(
        decode_32(0xe8df_f013),
        Instruction::TBH {
            params: Reg2RnRmParams {
                rn: Reg::PC,
                rm: Reg::R3,
            }
        }
    );
}
