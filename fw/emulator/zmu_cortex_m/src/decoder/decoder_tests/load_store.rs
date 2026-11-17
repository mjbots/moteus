use super::*;

#[test]
fn test_decode_ldm() {
    // LDM R2!, {R0, R1}
    match decode_16(0xca03) {
        Instruction::LDM { params, thumb32 } => {
            assert!(params.rn == Reg::R2);
            let elems: Vec<_> = params.registers.iter().collect();
            assert_eq!(vec![Reg::R0, Reg::R1], elems);
            assert!(params.wback);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_ldm2() {
    // LDM R1!, {R3}
    match decode_16(0xc908) {
        Instruction::LDM { params, thumb32 } => {
            assert!(params.rn == Reg::R1);
            let elems: Vec<_> = params.registers.iter().collect();
            assert_eq!(vec![Reg::R3], elems);
            assert!(params.wback);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_ldm_t1_base_in_list_disables_writeback() {
    // LDM R1!, {R0, R1}
    match decode_16(0xc903) {
        Instruction::LDM { params, thumb32 } => {
            assert!(params.rn == Reg::R1);
            let elems: Vec<_> = params.registers.iter().collect();
            assert_eq!(vec![Reg::R0, Reg::R1], elems);
            assert!(!params.wback);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_ldm3() {
    // LDM R4!, {R0-R2}
    match decode_16(0xcc07) {
        Instruction::LDM { params, thumb32 } => {
            assert!(params.rn == Reg::R4);
            let elems: Vec<_> = params.registers.iter().collect();
            assert_eq!(vec![Reg::R0, Reg::R1, Reg::R2], elems);
            assert!(params.wback);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_ldm_t2_single_register_is_udf() {
    // 0xe891_0008 -> LDM R1, {R3} (UNPREDICTABLE in ARM ARM)
    match decode_32(0xe891_0008) {
        Instruction::UDF { thumb32, .. } => assert!(thumb32),
        other => panic!("expected UDF for invalid LDM T2 single-register encoding, got {other:?}"),
    }
}

#[test]
fn test_decode_ldm_t2_no_w() {
    // 0xe891_1008 -> LDM R1, {R3, R12}

    match decode_32(0xe891_1008) {
        Instruction::LDM { params, thumb32 } => {
            assert!(params.rn == Reg::R1);
            let elems: Vec<_> = params.registers.iter().collect();
            assert_eq!(vec![Reg::R3, Reg::R12], elems);
            assert!(!params.wback);
            assert!(thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_ldm_t2_wback_with_base_in_list_is_udf() {
    // 0xe8b1_100a -> LDM R1!, {R1, R3, R12} (UNPREDICTABLE in ARM ARM)
    match decode_32(0xe8b1_100a) {
        Instruction::UDF { thumb32, .. } => assert!(thumb32),
        other => {
            panic!("expected UDF for invalid LDM T2 writeback/base-list overlap, got {other:?}")
        }
    }
}

#[test]
fn test_decode_ldm_t2_w() {
    // 0xe8b1_1008 -> LDM R1!, {R3, R12}

    match decode_32(0xe8b1_1008) {
        Instruction::LDM { params, thumb32 } => {
            assert!(params.rn == Reg::R1);
            let elems: Vec<_> = params.registers.iter().collect();
            assert_eq!(vec![Reg::R3, Reg::R12], elems);
            assert!(params.wback);
            assert!(thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_ldr() {
    // LDR.N R1, [PC, 0x1c]
    match decode_16(0x4907) {
        Instruction::LDR_lit { params, thumb32 } => {
            assert!(params.rt == Reg::R1);
            assert!(params.imm32 == (7 << 2));
            assert!(!thumb32);
            assert!(params.add);
        }
        _ => {
            unreachable!();
        }
    }
    // LDR R2, [R1]
    match decode_16(0x680a) {
        Instruction::LDR_imm { params, thumb32 } => {
            assert!(params.rn == Reg::R1);
            assert!(params.rt == Reg::R2);
            assert!(params.imm32 == 0);
            assert!(params.index);
            assert!(params.add);
            assert!(!params.wback);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_ldr_lit_w() {
    //0xf8df_90cc LDR.W R9, [PC, #0xcc]
    assert_eq!(
        decode_32(0xf8df_90cc),
        Instruction::LDR_lit {
            params: RegImm32AddParams {
                rt: Reg::R9,
                imm32: 0xcc,
                add: true,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_ldr_reg_w() {
    //0xf859_4024 LDR.W R4, [R9,R4, LSL #2]
    assert_eq!(
        decode_32(0xf859_4024),
        Instruction::LDR_reg {
            params: Reg3FullParams {
                rt: Reg::R4,
                rn: Reg::R9,
                rm: Reg::R4,
                shift_t: SRType::LSL,
                shift_n: 2,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_ldrb_imm() {
    // LDRB R0, [R0m 0]
    assert_eq!(
        decode_16(0x7800),
        Instruction::LDRB_imm {
            params: Reg2FullParams {
                rt: Reg::R0,
                rn: Reg::R0,
                imm32: 0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_ldrb_imm2() {
    // LDRB R2, [R0, 0x10]
    assert_eq!(
        decode_16(0x7c02),
        Instruction::LDRB_imm {
            params: Reg2FullParams {
                rt: Reg::R2,
                rn: Reg::R0,
                imm32: 0x10,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_ldrb_reg_w() {
    //0xf816_c004 -> LDRB.W R12, [R6, R4]

    assert_eq!(
        decode_32(0xf816_c004),
        Instruction::LDRB_reg {
            params: Reg3FullParams {
                rt: Reg::R12,
                rn: Reg::R6,
                rm: Reg::R4,
                shift_t: SRType::LSL,
                shift_n: 0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_ldrb_w() {
    // 0xf896_0020 LDRB.W R0 [R6, #0x20]
    assert_eq!(
        decode_32(0xf896_0020),
        Instruction::LDRB_imm {
            params: Reg2FullParams {
                rt: Reg::R0,
                rn: Reg::R6,
                imm32: 0x20,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_ldrd_w() {
    // 0xe9d5_0100 -> LDRD R0, R1, [R5]
    assert_eq!(
        decode_32(0xe9d5_0100),
        Instruction::LDRD_imm {
            params: Reg2DoubleParams {
                rt: Reg::R0,
                rt2: Reg::R1,
                rn: Reg::R5,
                imm32: 0,
                index: true,
                add: true,
                wback: false,
            },
        }
    );
}

#[test]
fn test_decode_ldrex() {
    //  e850 3f00       ldrex   r3, [r0]
    assert_eq!(
        decode_32(0xe850_3f00),
        Instruction::LDREX {
            params: Reg2RtRnImm32Params {
                rt: Reg::R3,
                rn: Reg::R0,
                imm32: 0,
            }
        }
    );
}

#[test]
fn test_decode_ldrh() {
    // LDRH R0,[R0, #0x38]
    match decode_16(0x8f00) {
        Instruction::LDRH_imm { params, thumb32 } => {
            assert!(params.rn == Reg::R0);
            assert!(params.rt == Reg::R0);
            assert!(params.imm32 == 0x38);
            assert!(params.index);
            assert!(params.add);
            assert!(!params.wback);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_ldrh_reg_w() {
    //0xf838_301a -> LDRH.W R3, [R8, R10, LSL #1]

    assert_eq!(
        decode_32(0xf838_301a),
        Instruction::LDRH_reg {
            params: Reg3FullParams {
                rt: Reg::R3,
                rn: Reg::R8,
                rm: Reg::R10,
                shift_t: SRType::LSL,
                shift_n: 1,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_ldrh_w() {
    //0xf834_9b02 LDRH.W R9, [R4], #0x2
    assert_eq!(
        decode_32(0xf834_9b02),
        Instruction::LDRH_imm {
            params: Reg2FullParams {
                rt: Reg::R9,
                rn: Reg::R4,
                imm32: 2,
                add: true,
                index: false,
                wback: true,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_ldrsb_imm_t2() {
    // 0xf917_0c09 -> ldrsb.w r0, [r7, #-9]
    assert_eq!(
        decode_32(0xf917_0c09),
        Instruction::LDRSB_imm {
            params: Reg2FullParams {
                rt: Reg::R0,
                rn: Reg::R7,
                imm32: 9,
                index: true,
                add: false,
                wback: false,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_ldrsb_imm_w() {
    // 0xf995_6000 -> LDRSB R6, [R5]
    assert_eq!(
        decode_32(0xf995_6000),
        Instruction::LDRSB_imm {
            params: Reg2FullParams {
                rt: Reg::R6,
                rn: Reg::R5,
                imm32: 0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_ldrsb_reg() {
    // LDRSB R4, [R4, R0]
    assert_eq!(
        decode_16(0x5624),
        Instruction::LDRSB_reg {
            params: Reg3FullParams {
                rt: Reg::R4,
                rn: Reg::R4,
                rm: Reg::R0,
                shift_t: SRType::LSL,
                shift_n: 0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_ldrsh() {
    // LDRSH R0, [R6, R0]
    assert_eq!(
        decode_16(0x5e30),
        Instruction::LDRSH_reg {
            params: Reg3FullParams {
                rt: Reg::R0,
                rn: Reg::R6,
                rm: Reg::R0,
                shift_t: SRType::LSL,
                shift_n: 0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_ldrsh_imm_w() {
    // LDRSH.W R0, [SP, #0x10]
    assert_eq!(
        decode_32(0xf9bd_0010),
        Instruction::LDRSH_imm {
            params: Reg2FullParams {
                rt: Reg::R0,
                rn: Reg::SP,
                imm32: 0x10,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_ldrsh_reg_w() {
    // LDRSH.W R0, [R0, R0, LSL #0]
    assert_eq!(
        decode_32(0xf930_0000),
        Instruction::LDRSH_reg {
            params: Reg3FullParams {
                rt: Reg::R0,
                rn: Reg::R0,
                rm: Reg::R0,
                shift_t: SRType::LSL,
                shift_n: 0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_ldrw_imm() {
    // LDR.W R1, [R0], #0x4
    assert_eq!(
        decode_32(0xf85_01b04),
        Instruction::LDR_imm {
            params: Reg2FullParams {
                rt: Reg::R1,
                rn: Reg::R0,
                imm32: 0x4,
                index: false,
                add: true,
                wback: true,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_pop() {
    // POP  {R4, LR}
    match decode_16(0xbd10) {
        Instruction::POP { registers, thumb32 } => {
            let elems: Vec<_> = registers.iter().collect();
            assert_eq!(vec![Reg::R4, Reg::PC], elems);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_pop_t3_w() {
    //0xf85d_eb04 -> LDR.W LR, [SP], #4   // POP.W LR  (pop 3)
    match decode_32(0xf85d_eb04) {
        Instruction::POP { registers, thumb32 } => {
            let elems: Vec<_> = registers.iter().collect();
            assert_eq!(vec![Reg::LR], elems);
            assert!(thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_pop_w() {
    //0xe8bd_47f0 POP.W {R4-R10, LR}
    match decode_32(0xe8bd_47f0) {
        Instruction::POP { registers, thumb32 } => {
            let elems: Vec<_> = registers.iter().collect();
            assert_eq!(
                vec![
                    Reg::R4,
                    Reg::R5,
                    Reg::R6,
                    Reg::R7,
                    Reg::R8,
                    Reg::R9,
                    Reg::R10,
                    Reg::LR
                ],
                elems
            );
            assert!(thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_push() {
    // PUSH  {R4, LR}
    match decode_16(0xb510) {
        Instruction::PUSH { registers, thumb32 } => {
            let elems: Vec<_> = registers.iter().collect();
            assert_eq!(vec![Reg::R4, Reg::LR], elems);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_pushw() {
    // PUSH.W {R4-R11, LR}
    // PUSH  {R4, LR}
    match decode_32(0xe92d_4ff0) {
        Instruction::PUSH { registers, thumb32 } => {
            let elems: Vec<_> = registers.iter().collect();
            assert_eq!(
                vec![
                    Reg::R4,
                    Reg::R5,
                    Reg::R6,
                    Reg::R7,
                    Reg::R8,
                    Reg::R9,
                    Reg::R10,
                    Reg::R11,
                    Reg::LR,
                ],
                elems
            );

            assert!(thumb32);
        }
        _ => {
            unreachable!()
        }
    }
}

#[test]
fn test_decode_stm() {
    // STM R2!, {R0, R1}
    match decode_16(0xc203) {
        Instruction::STM { params, thumb32 } => {
            assert!(params.rn == Reg::R2);
            let elems: Vec<_> = params.registers.iter().collect();
            assert_eq!(vec![Reg::R0, Reg::R1], elems);
            assert!(params.wback);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_stm2() {
    // STM R3!, {R0-R2}
    match decode_16(0xc307) {
        Instruction::STM { params, thumb32 } => {
            assert!(params.rn == Reg::R3);
            let elems: Vec<_> = params.registers.iter().collect();
            assert_eq!(vec![Reg::R0, Reg::R1, Reg::R2], elems);
            assert!(params.wback);
            assert!(!thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_stmdb_w() {
    //0xe920_003c -> STMDB R0!, {R2-R5}
    match decode_32(0xe920_003c) {
        Instruction::STMDB { params } => {
            assert!(params.rn == Reg::R0);
            let elems: Vec<_> = params.registers.iter().collect();
            assert_eq!(vec![Reg::R2, Reg::R3, Reg::R4, Reg::R5], elems);
            assert!(params.wback);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_str_imm_t4() {
    //f84d cd04       str.w   ip, [sp, #-4]!
    // => same as PUSH r12
    match decode_32(0xf84d_cd04) {
        Instruction::PUSH { registers, thumb32 } => {
            let elems: Vec<_> = registers.iter().collect();
            assert_eq!(vec![Reg::R12], elems);
            assert!(thumb32);
        }
        _ => {
            unreachable!();
        }
    }
}

#[test]
fn test_decode_str_reg() {
    // STR R0, [R1, R2]
    assert_eq!(
        decode_16(0x5088),
        Instruction::STR_reg {
            params: Reg3FullParams {
                rt: Reg::R0,
                rn: Reg::R1,
                rm: Reg::R2,
                shift_t: SRType::LSL,
                shift_n: 0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_str_reg_w() {
    // 0xf841_002a
    // STR.W R0, [R1, R10, LSL #2]
    assert_eq!(
        decode_32(0xf841_002a),
        Instruction::STR_reg {
            params: Reg3FullParams {
                rt: Reg::R0,
                rn: Reg::R1,
                rm: Reg::R10,
                shift_t: SRType::LSL,
                shift_n: 2,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_strb() {
    // STRB R0, [R1]
    assert_eq!(
        decode_16(0x7008),
        Instruction::STRB_imm {
            params: Reg2FullParams {
                rt: Reg::R0,
                rn: Reg::R1,
                imm32: 0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_strb2() {
    // STRB R2, [R0, R5]
    assert_eq!(
        decode_16(0x5542),
        Instruction::STRB_reg {
            params: Reg3FullParams {
                rt: Reg::R2,
                rn: Reg::R0,
                rm: Reg::R5,
                index: true,
                add: true,
                wback: false,
                shift_n: 0,
                shift_t: SRType::LSL,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_strb_imm_w() {
    //0xf80e_ab01 STRB.W R10, [LR], #1
    assert_eq!(
        decode_32(0xf80e_ab01),
        Instruction::STRB_imm {
            params: Reg2FullParams {
                rt: Reg::R10,
                rn: Reg::LR,
                imm32: 1,
                index: false,
                add: true,
                wback: true,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_strb_reg_w() {
    //0xf80c_e007 STRB.W LR, [R12, R7]
    assert_eq!(
        decode_32(0xf80c_e007),
        Instruction::STRB_reg {
            params: Reg3FullParams {
                rt: Reg::LR,
                rn: Reg::R12,
                rm: Reg::R7,
                index: true,
                add: true,
                wback: false,
                shift_n: 0,
                shift_t: SRType::LSL,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_strd_w() {
    // 0xe9cd_0100 -> STRD R0, R1, [SP]
    assert_eq!(
        decode_32(0xe9cd_0100),
        Instruction::STRD_imm {
            params: Reg2DoubleParams {
                rt: Reg::R0,
                rt2: Reg::R1,
                rn: Reg::SP,
                imm32: 0,
                index: true,
                add: true,
                wback: false,
            }
        }
    );
}

#[test]
fn test_decode_strex() {
    //  e840 2c00       strex   ip, r2, [r0]
    assert_eq!(
        decode_32(0xe840_2c00),
        Instruction::STREX {
            params: Reg3RdRtRnImm32Params {
                rd: Reg::R12,
                rt: Reg::R2,
                rn: Reg::R0,
                imm32: 0,
            }
        }
    );
}

#[test]
fn test_decode_strexb() {
    // 5212:       e8c1 2f43       strexb  r3, r2, [r1]
    // Opcode: 0xE8C12F43
    // Rn = R1
    // Rt = R2
    // Rd = R3
    assert_eq!(
        decode_32(0xE8C1_2F43),
        Instruction::STREXB {
            params: Reg3RdRtRnParams {
                rd: Reg::R3,
                rt: Reg::R2,
                rn: Reg::R1,
            }
        }
    );
}

#[test]
fn test_decode_strexh() {
    // strexh r3, r2, [r1]
    // Opcode: 0xE8C12F53
    assert_eq!(
        decode_32(0xE8C1_2F53),
        Instruction::STREXH {
            params: Reg3RdRtRnParams {
                rd: Reg::R3,
                rt: Reg::R2,
                rn: Reg::R1,
            }
        }
    );
}

#[test]
fn test_decode_strh_imm() {
    // STRH R0, [R1, #0x38]
    assert_eq!(
        decode_16(0x8708),
        Instruction::STRH_imm {
            params: Reg2FullParams {
                rt: Reg::R0,
                rn: Reg::R1,
                imm32: 0x38,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_strh_reg() {
    // STRH R4, [R6, R1]
    assert_eq!(
        decode_16(0x5274),
        Instruction::STRH_reg {
            params: Reg3FullParams {
                rt: Reg::R4,
                rn: Reg::R6,
                rm: Reg::R1,
                index: true,
                add: true,
                wback: false,
                shift_n: 0,
                shift_t: SRType::LSL,
            },
            thumb32: false,
        }
    );
}

#[test]
fn test_decode_strh_reg_w() {
    //  STRH.W  R12, [R6, R9, LSL #1]
    assert_eq!(
        decode_32(0xf826_c019),
        Instruction::STRH_reg {
            params: Reg3FullParams {
                rt: Reg::R12,
                rn: Reg::R6,
                rm: Reg::R9,
                index: true,
                add: true,
                wback: false,
                shift_n: 1,
                shift_t: SRType::LSL,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_strh_w() {
    // STRH.W R0, [SP, #0x10]
    assert_eq!(
        decode_32(0xf8ad_0010),
        Instruction::STRH_imm {
            params: Reg2FullParams {
                rt: Reg::R0,
                rn: Reg::SP,
                imm32: 0x10,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_strh_w_2() {
    // 0xf8a8_7000 -> STRH.W R7, [R8]
    assert_eq!(
        decode_32(0xf8a8_7000),
        Instruction::STRH_imm {
            params: Reg2FullParams {
                rt: Reg::R7,
                rn: Reg::R8,
                imm32: 0x0,
                index: true,
                add: true,
                wback: false,
            },
            thumb32: true,
        }
    );
}

#[test]
fn test_decode_strw_imm() {
    // STR.W R4, [R3], #0x4
    assert_eq!(
        decode_32(0xf843_4b04),
        Instruction::STR_imm {
            params: Reg2FullParams {
                rt: Reg::R4,
                rn: Reg::R3,
                imm32: 4,
                index: false,
                add: true,
                wback: true,
            },
            thumb32: true,
        }
    );
}
