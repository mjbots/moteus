use super::*;

#[test]
fn test_decode_adc_imm_w() {
    // 0xf154_0401 -> ADCS.W R4, R4, #1

    assert_eq!(
        decode_32(0xf154_0401),
        Instruction::ADC_imm {
            params: Reg2ImmParams {
                rd: Reg::R4,
                rn: Reg::R4,
                setflags: SetFlags::True,
                imm32: 1
            }
        }
    );
}

#[test]
fn test_decode_adc_reg() {
    // ADCS R2,R2,R2
    match decode_16(0x4152) {
        Instruction::ADC_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R2);
            assert!(params.rm == Reg::R2);
            assert!(params.rn == Reg::R2);
            assert!(params.setflags == SetFlags::NotInITBlock);
            assert!(params.shift_t == SRType::LSL);
            assert!(params.shift_n == 0);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_adc_reg_w() {
    //0xeb50_500e -> ADCS.W R0, R0, LR, LSL #20

    assert_eq!(
        decode_32(0xeb50_500e),
        Instruction::ADC_reg {
            params: Reg3ShiftParams {
                rd: Reg::R0,
                rn: Reg::R0,
                rm: Reg::LR,
                setflags: SetFlags::True,
                shift_t: SRType::LSL,
                shift_n: 20,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_add_reg_imm() {
    // ADDS R1, R1, 24
    match decode_16(0x3118) {
        Instruction::ADD_imm { params, thumb32 } => {
            assert!(params.rn == Reg::R1);
            assert!(params.rd == Reg::R1);
            assert!(params.imm32 == 24);
            assert!(params.setflags == SetFlags::NotInITBlock);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_add_reg_pc() {
    // ADD R1,R1, PC
    assert_eq!(
        decode_16(0x4479),
        Instruction::ADD_reg {
            params: Reg3ShiftParams {
                rd: Reg::R1,
                rn: Reg::R1,
                rm: Reg::PC,
                setflags: SetFlags::False,
                shift_t: SRType::LSL,
                shift_n: 0,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_add_reg_sp() {
    // ADD R1, SP, #0xc
    match decode_16(0xa903) {
        Instruction::ADD_imm { params, thumb32 } => {
            assert!(params.rn == Reg::SP);
            assert!(params.rd == Reg::R1);
            assert!(params.imm32 == 0xc);
            assert!(params.setflags == SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_add_reg_w() {
    // 0xeb01_03ca ADD.W R3, R1, R10, LSL #3
    assert_eq!(
        decode_32(0xeb01_03ca),
        Instruction::ADD_reg {
            params: Reg3ShiftParams {
                rd: Reg::R3,
                rn: Reg::R1,
                rm: Reg::R10,
                setflags: SetFlags::False,
                shift_t: SRType::LSL,
                shift_n: 3,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_adds_w() {
    // 0xf118_0801 ADDS.W R8, R8, #1
    assert_eq!(
        decode_32(0xf118_0801),
        Instruction::ADD_imm {
            params: Reg2ImmParams {
                rn: Reg::R8,
                rd: Reg::R8,
                imm32: 1,
                setflags: SetFlags::True
            },
            thumb32: true
        }
    );
}

#[test]
fn test_decode_adr() {
    // ADR R0, PC, #(7<<2)
    match decode_16(0xa007) {
        Instruction::ADR { params, thumb32 } => {
            assert!(params.r == Reg::R0);
            assert!(params.imm32 == 7 << 2);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_and() {
    // ANDS R2,R2,R3
    match decode_16(0x401a) {
        Instruction::AND_reg { params, thumb32 } => {
            assert_eq!(params.rd, Reg::R2);
            assert_eq!(params.rn, Reg::R2);
            assert_eq!(params.rm, Reg::R3);
            assert!(!thumb32);
            assert_eq!(params.shift_t, SRType::LSL);
            assert_eq!(params.shift_n, 0);
            assert!(params.setflags == SetFlags::NotInITBlock);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_and_imm_w() {
    // 0xf01a_0c03 ANDS.W R12, R10, 3
    assert_eq!(
        decode_32(0xf01a_0c03),
        Instruction::AND_imm {
            params: Reg2ImmCarryParams {
                rd: Reg::R12,
                rn: Reg::R10,
                imm32: Imm32Carry::Carry {
                    imm32_c0: (3, false),
                    imm32_c1: (3, true),
                },
                setflags: true,
            }
        }
    );
}

#[test]
fn test_decode_and_reg_w() {
    //0xea15_5411 -> ANDS.W R4, R5, R1, LSR #20
    assert_eq!(
        decode_32(0xea15_5411),
        Instruction::AND_reg {
            params: Reg3ShiftParams {
                rd: Reg::R4,
                rn: Reg::R5,
                rm: Reg::R1,
                setflags: SetFlags::True,
                shift_t: SRType::LSR,
                shift_n: 20,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_asr_imm() {
    // ASR R2,R2,#8
    match decode_16(0x1212) {
        Instruction::ASR_imm { params, thumb32 } => {
            assert!(params.rd == Reg::R2);
            assert!(params.rm == Reg::R2);
            assert!(params.shift_n == 8);
            assert!(params.setflags == SetFlags::NotInITBlock);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_asr_w() {
    //0xEA4f_39e2 ASR.W R9, R2, #15
    assert_eq!(
        decode_32(0xea4f_39e2),
        Instruction::ASR_imm {
            params: Reg2ShiftNParams {
                rd: Reg::R9,
                rm: Reg::R2,
                shift_n: 15,
                setflags: SetFlags::False,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_asrw_reg_t2() {
    //  fa43 f305       asr.w   r3, r3, r5
    assert_eq!(
        decode_32(0xfa43_f305),
        Instruction::ASR_reg {
            params: Reg3Params {
                rd: Reg::R3,
                rn: Reg::R3,
                rm: Reg::R5,
                setflags: SetFlags::False,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_cmn() {
    // CMN R4,R5
    match decode_16(0x42ec) {
        Instruction::CMN_reg { params, thumb32 } => {
            assert!(params.rn == Reg::R4);
            assert!(params.rm == Reg::R5);
            assert!(params.shift_t == SRType::LSL);
            assert!(params.shift_n == 0);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_cmn_w_reg() {
    // CMN.W R12, R1, LSL #1
    assert_eq!(
        decode_32(0xeb1c_0f41),
        Instruction::CMN_reg {
            params: Reg2ShiftNoSetFlagsParams {
                rn: Reg::R12,
                rm: Reg::R1,
                shift_n: 1,
                shift_t: SRType::LSL,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_cmp() {
    //CMP R0, R0
    assert_eq!(
        decode_16(0x2800),
        Instruction::CMP_imm {
            params: RegImmParams {
                r: Reg::R0,
                imm32: 0,
            },
            thumb32: false,
        }
    );
    // CMP R1, R4
    match decode_16(0x42a1) {
        Instruction::CMP_reg { params, thumb32 } => {
            assert!(params.rn == Reg::R1);
            assert!(params.rm == Reg::R4);
            assert!(params.shift_t == SRType::LSL);
            assert!(params.shift_n == 0);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    // CMP R2, #0
    assert_eq!(
        decode_16(0x2a00),
        Instruction::CMP_imm {
            params: RegImmParams {
                r: Reg::R2,
                imm32: 0,
            },
            thumb32: false,
        }
    );
    // CMP LR, R4
    assert_eq!(
        decode_16(0x45A6),
        Instruction::CMP_reg {
            params: Reg2ShiftNoSetFlagsParams {
                rn: Reg::LR,
                rm: Reg::R4,
                shift_t: SRType::LSL,
                shift_n: 0,
            },
            thumb32: false
        }
    );
}

#[test]
fn test_decode_cmp_imm_w() {
    // 0xf1ba_0f00 CMP.W R10, #0
    assert_eq!(
        decode_32(0xf1ba_0f00),
        Instruction::CMP_imm {
            params: RegImmParams {
                r: Reg::R10,
                imm32: 0,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_cmp_reg_w() {
    // 0xebb7_1f46 -> CMP.W R7, R6, LSL #5
    assert_eq!(
        decode_32(0xebb7_1f46),
        Instruction::CMP_reg {
            params: Reg2ShiftNoSetFlagsParams {
                rn: Reg::R7,
                rm: Reg::R6,
                shift_t: SRType::LSL,
                shift_n: 5,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_eor_imm_w() {
    //0xf481_4120 -> EOR.W R1, R1, #40960 ; 0xa000
    assert_eq!(
        decode_32(0xf481_4120),
        Instruction::EOR_imm {
            params: Reg2ImmCarryParams {
                rd: Reg::R1,
                rn: Reg::R1,
                imm32: Imm32Carry::Carry {
                    imm32_c0: (0xa000, false),
                    imm32_c1: (0xa000, false)
                },
                setflags: false
            }
        }
    );
}

#[test]
fn test_decode_eor_reg() {
    // EOR R0, R0, R4
    match decode_16(0x4060) {
        Instruction::EOR_reg { params, thumb32 } => {
            assert_eq!(params.rd, Reg::R0);
            assert_eq!(params.rn, Reg::R0);
            assert_eq!(params.rm, Reg::R4);
            assert_eq!(params.setflags, SetFlags::NotInITBlock);
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
fn test_decode_eor_reg_w() {
    // 0xea8e_0402 EOR.W R4, LR, R2
    assert_eq!(
        decode_32(0xea8e_0402),
        Instruction::EOR_reg {
            params: Reg3ShiftParams {
                rd: Reg::R4,
                rn: Reg::LR,
                rm: Reg::R2,
                setflags: SetFlags::False,
                shift_t: SRType::LSL,
                shift_n: 0,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_lsl_2() {
    // LSL r1, r1, #31
    assert_eq!(
        decode_16(0x07c9),
        Instruction::LSL_imm {
            params: Reg2ShiftNParams {
                rd: Reg::R1,
                rm: Reg::R1,
                shift_n: 31,
                setflags: SetFlags::NotInITBlock,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_lsl_reg_t2() {
    // 0xfa0c_f505 ->     lsl.w   r5, ip, r5

    assert_eq!(
        decode_32(0xfa0c_f505),
        Instruction::LSL_reg {
            params: Reg3Params {
                rd: Reg::R5,
                rn: Reg::R12,
                rm: Reg::R5,
                setflags: SetFlags::False
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_lsl_w_imm() {
    // LSL.W R8,R8,1
    assert_eq!(
        decode_32(0xea4f_0848),
        Instruction::LSL_imm {
            params: Reg2ShiftNParams {
                rd: Reg::R8,
                rm: Reg::R8,
                shift_n: 1,
                setflags: SetFlags::False,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_lsls() {
    // LSLS R1, R4, #2
    match decode_16(0x00a1) {
        Instruction::LSL_imm { params, thumb32 } => {
            assert!(params.rd == Reg::R1);
            assert!(params.rm == Reg::R4);
            assert!(params.shift_n == 2);
            assert!(params.setflags == SetFlags::NotInITBlock);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_lsr_imm() {
    // LSRS R3, R0, #8
    match decode_16(0x0a03) {
        Instruction::LSR_imm { params, thumb32 } => {
            assert!(params.rd == Reg::R3);
            assert!(params.rm == Reg::R0);
            assert!(params.shift_n == 8);
            assert!(params.setflags == SetFlags::NotInITBlock);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_lsr_reg() {
    // LSRS R1, R1, R4
    match decode_16(0x40e1) {
        Instruction::LSR_reg { params, thumb32 } => {
            assert_eq!(params.rd, Reg::R1);
            assert_eq!(params.rn, Reg::R1);
            assert_eq!(params.rm, Reg::R4);
            assert!(params.setflags == SetFlags::NotInITBlock);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_lsr_w_imm() {
    // LSRS.W R12,R10,2
    assert_eq!(
        decode_32(0xea5f_0c9a),
        Instruction::LSR_imm {
            params: Reg2ShiftNParams {
                rd: Reg::R12,
                rm: Reg::R10,
                shift_n: 2,
                setflags: SetFlags::True,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_lsr_w_reg() {
    // 0xfa30_f009 -> LSRS.W R0, R0, R9
    assert_eq!(
        decode_32(0xfa30_f009),
        Instruction::LSR_reg {
            params: Reg3Params {
                rd: Reg::R0,
                rn: Reg::R0,
                rm: Reg::R9,
                setflags: SetFlags::True,
            },
            thumb32: true
        }
    );
}

#[test]
fn test_decode_mov() {
    match decode_16(0x4600) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R0);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4608) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R1);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4610) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R2);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4618) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R3);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4620) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R4);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4628) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R5);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4630) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R6);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4638) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R7);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4640) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R8);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4648) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R9);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4650) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R10);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4658) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R11);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4660) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::R12);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4668) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::SP);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4670) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::LR);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    match decode_16(0x4678) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R0);
            assert!(params.rm == Reg::PC);
            assert_eq!(params.setflags, SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }

    match decode_16(0x0001) {
        Instruction::MOV_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R1);
            assert!(params.rm == Reg::R0);
            assert_eq!(params.setflags, SetFlags::NotInITBlock);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
    //MOVS (mov immediate)
    assert_eq!(
        decode_16(0x2001),
        Instruction::MOV_imm {
            params: RegImmCarryParams {
                rd: Reg::R0,
                imm32: Imm32Carry::NoCarry { imm32: 1 },
                setflags: SetFlags::NotInITBlock,
            },
            thumb32: false,
        }
    );

    assert_eq!(
        decode_16(0x2101),
        Instruction::MOV_imm {
            params: RegImmCarryParams {
                rd: Reg::R1,
                imm32: Imm32Carry::NoCarry { imm32: 1 },
                setflags: SetFlags::NotInITBlock,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_mov_reg_w() {
    // MOV.W R8, R3
    assert_eq!(
        decode_32(0xea4f_0803),
        Instruction::MOV_reg {
            params: Reg2Params {
                rd: Reg::R8,
                rm: Reg::R3,
                setflags: SetFlags::False,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_mov_rxx_w() {
    //ea4f 0232       mov.w   r2, r2, rrx
    assert_eq!(
        decode_32(0xea4f_0232),
        Instruction::RRX {
            params: Reg2Params {
                rd: Reg::R2,
                rm: Reg::R2,
                setflags: SetFlags::False,
            }
        }
    );
}

#[test]
fn test_decode_mov_w() {
    // MOV.W R8, #-1
    assert_eq!(
        decode_32(0xf04f_38ff),
        Instruction::MOV_imm {
            params: RegImmCarryParams {
                rd: Reg::R8,
                imm32: Imm32Carry::Carry {
                    imm32_c0: (0xffff_ffff, false),
                    imm32_c1: (0xffff_ffff, true),
                },
                setflags: SetFlags::False,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_movt() {
    // f2c2 0100       movt    r1, #8192

    assert_eq!(
        decode_32(0xf2c2_0100),
        Instruction::MOVT {
            params: MovtParams {
                rd: Reg::R1,
                imm16: 0x2000
            }
        }
    );
}

#[test]
fn test_decode_mvn_reg_w() {
    // ea6f 5507       mvn.w   r5, r7, lsl #20

    assert_eq!(
        decode_32(0xea6f_5507),
        Instruction::MVN_reg {
            params: Reg2ShiftParams {
                rd: Reg::R5,
                rm: Reg::R7,
                setflags: SetFlags::False,
                shift_n: 20,
                shift_t: SRType::LSL,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_mvns() {
    // MVNS R5,R5
    match decode_16(0x43ed) {
        Instruction::MVN_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R5);
            assert!(params.rm == Reg::R5);
            assert!(params.setflags == SetFlags::NotInITBlock);
            assert!(params.shift_t == SRType::LSL);
            assert!(params.shift_n == 0);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_orn_reg_t2() {
    // 0xea62 0205       orn     r2, r2, r5

    assert_eq!(
        decode_32(0xea62_0205),
        Instruction::ORN_reg {
            params: Reg3ShiftParams {
                rd: Reg::R2,
                rn: Reg::R2,
                rm: Reg::R5,
                setflags: SetFlags::False,
                shift_t: SRType::LSL,
                shift_n: 0,
            }
        }
    );
}

#[test]
fn test_decode_orr() {
    // ORRS R3, R3, R1
    match decode_16(0x430b) {
        Instruction::ORR_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R3);
            assert!(params.rn == Reg::R3);
            assert!(params.rm == Reg::R1);
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
fn test_decode_orr_imm_w() {
    // 0xf040_0010
    // ORR.W R0, R0, #16
    assert_eq!(
        decode_32(0xf040_0010),
        Instruction::ORR_imm {
            params: Reg2ImmCarryParams {
                rd: Reg::R0,
                rn: Reg::R0,
                imm32: Imm32Carry::Carry {
                    imm32_c0: (16, false),
                    imm32_c1: (16, true)
                },
                setflags: false
            }
        }
    );
}

#[test]
fn test_decode_orr_reg_w() {
    // 0xea44_04c8  ORR.W R4, R4, R8, LSL #3
    assert_eq!(
        decode_32(0xea44_04c8),
        Instruction::ORR_reg {
            params: Reg3ShiftParams {
                rd: Reg::R4,
                rn: Reg::R4,
                rm: Reg::R8,
                setflags: SetFlags::False,
                shift_t: SRType::LSL,
                shift_n: 3,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_ror_imm_w() {
    // 0xea4f_74f4 -> ROR.W R4, R4, #31

    assert_eq!(
        decode_32(0xea4f_74f4),
        Instruction::ROR_imm {
            params: Reg2ShiftNParams {
                rd: Reg::R4,
                rm: Reg::R4,
                shift_n: 31,
                setflags: SetFlags::False
            }
        }
    );
}

#[test]
fn test_decode_ror_t2() {
    // ror.w   r2, fp, r0
    // Opcode: 0xfa6b_f200

    assert_eq!(
        decode_32(0xfa6b_f200),
        Instruction::ROR_reg {
            params: Reg3Params {
                rd: Reg::R2,
                rn: Reg::R11,
                rm: Reg::R0,
                setflags: SetFlags::False,
            },
            thumb32: true,
        }
    );
}
#[test]
fn test_decode_sbc() {
    // SBCS R5, R5, R3
    match decode_16(0x419d) {
        Instruction::SBC_reg { params, thumb32 } => {
            assert!(params.rd == Reg::R5);
            assert!(params.rn == Reg::R5);
            assert!(params.rm == Reg::R3);
            assert!(params.setflags == SetFlags::NotInITBlock);
            assert!(params.shift_t == SRType::LSL);
            assert!(params.shift_n == 0);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_sbc_imm_w() {
    //0xf167_0700 -> SBC.W R7, R7, #0
    assert_eq!(
        decode_32(0xf167_0700),
        Instruction::SBC_imm {
            params: Reg2ImmParams {
                rd: Reg::R7,
                rn: Reg::R7,
                setflags: SetFlags::False,
                imm32: 0
            }
        }
    );
}

#[test]
fn test_decode_sbc_reg_w() {
    //0xeb6a_0a4a -> SBC.W R10, R10, R10, LSL #1
    assert_eq!(
        decode_32(0xeb6a_0a4a),
        Instruction::SBC_reg {
            params: Reg3ShiftParams {
                rd: Reg::R10,
                rn: Reg::R10,
                rm: Reg::R10,
                shift_n: 1,
                shift_t: SRType::LSL,
                setflags: SetFlags::False,
            },
            thumb32: true
        }
    );
}

#[test]
fn test_decode_sub() {
    // SUB SP,SP, #0x8
    match decode_16(0xb082) {
        Instruction::SUB_imm { params, thumb32 } => {
            assert!(params.rd == Reg::SP);
            assert!(params.rn == Reg::SP);
            assert!(params.imm32 == 0x8);
            assert!(params.setflags == SetFlags::False);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_sub2() {
    // SUBS R2,R2,#48
    match decode_16(0x3a30) {
        Instruction::SUB_imm { params, thumb32 } => {
            assert!(params.rd == Reg::R2);
            assert!(params.rn == Reg::R2);
            assert!(params.imm32 == 48);
            assert!(params.setflags == SetFlags::NotInITBlock);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_subw_imm() {
    // SUBW SP,SP,#2084
    assert_eq!(
        decode_32(0xf6ad_0d24),
        Instruction::SUB_imm {
            params: Reg2ImmParams {
                rd: Reg::SP,
                rn: Reg::SP,
                imm32: 2084,
                setflags: SetFlags::False,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_subw_imm_t4() {
    // f2a4 4333       subw    r3, r4, #1075   ; 0x433
    assert_eq!(
        decode_32(0xf2a4_4333),
        Instruction::SUB_imm {
            params: Reg2ImmParams {
                rd: Reg::R3,
                rn: Reg::R4,
                imm32: 1075,
                setflags: SetFlags::False,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_subw_reg() {
    // 0xebb0_0b09
    // SUBS.W R11, R0, R9
    assert_eq!(
        decode_32(0xebb0_0b09),
        Instruction::SUB_reg {
            params: Reg3ShiftParams {
                rd: Reg::R11,
                rn: Reg::R0,
                rm: Reg::R9,
                setflags: SetFlags::True,
                shift_t: SRType::LSL,
                shift_n: 0,
            },
            thumb32: true,
        }
    );

    // 0xEBA4_5613
    // SUB.W R6, R4, R3, LSR #20
    assert_eq!(
        decode_32(0xEBA4_5613),
        Instruction::SUB_reg {
            params: Reg3ShiftParams {
                rd: Reg::R6,
                rn: Reg::R4,
                rm: Reg::R3,
                setflags: SetFlags::False,
                shift_t: SRType::LSR,
                shift_n: 20,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_teq_reg_w() {
    // 0xea91_0f03 -> TEQ.W R1, R3

    assert_eq!(
        decode_32(0xea91_0f03),
        Instruction::TEQ_reg {
            params: Reg2ShiftNoSetFlagsParams {
                rn: Reg::R1,
                rm: Reg::R3,
                shift_t: SRType::LSL,
                shift_n: 0
            }
        }
    );
}

#[test]
fn test_decode_teq_w() {
    //f090 0f00       teq     r0, #0
    assert_eq!(
        decode_32(0xf090_0f00),
        Instruction::TEQ_imm {
            params: RegImmCarryNoSetFlagsParams {
                rn: Reg::R0,
                imm32: Imm32Carry::Carry {
                    imm32_c0: (0, false),
                    imm32_c1: (0, true),
                },
            }
        }
    );
}

#[test]
fn test_decode_tst() {
    // TST R4, R1
    match decode_16(0x420c) {
        Instruction::TST_reg { params, thumb32 } => {
            assert!(params.rn == Reg::R4);
            assert!(params.rm == Reg::R1);
            assert!(params.shift_t == SRType::LSL);
            assert!(params.shift_n == 0);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_tst_imm_w() {
    //0xf011_3f80 -> TST.W R1, 0x8080_8080
    assert_eq!(
        decode_32(0xf011_3f80),
        Instruction::TST_imm {
            params: RegImmCarryNoSetFlagsParams {
                rn: Reg::R1,
                imm32: Imm32Carry::Carry {
                    imm32_c0: (0x8080_8080, false),
                    imm32_c1: (0x8080_8080, true),
                },
            }
        }
    );
}

#[test]
fn test_decode_tst_reg_w() {
    // 0xea18_0f03 tst.w   r8, r3

    assert_eq!(
        decode_32(0xea18_0f03),
        Instruction::TST_reg {
            params: Reg2ShiftNoSetFlagsParams {
                rn: Reg::R8,
                rm: Reg::R3,
                shift_n: 0,
                shift_t: SRType::LSL,
            },
            thumb32: true,
        }
    );
}
