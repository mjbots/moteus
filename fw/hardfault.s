.syntax unified
.cpu cortex-m4
.thumb_func

.global HardFault_Handler
.extern hard_fault_handler_c

HardFault_Handler:
  TST LR, #4
  ITE EQ
  MRSEQ R0, MSP
  MRSNE R0, PSP
  B hard_fault_handler_c
