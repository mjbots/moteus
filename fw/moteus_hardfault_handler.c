// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fw/stm32.h"

void hard_fault_handler_c (unsigned int * hardfault_args)
{
  volatile unsigned int stacked_r0;
  volatile unsigned int stacked_r1;
  volatile unsigned int stacked_r2;
  volatile unsigned int stacked_r3;
  volatile unsigned int stacked_r12;
  volatile unsigned int stacked_lr;
  volatile unsigned int stacked_pc;
  volatile unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  (void)stacked_r0;
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  (void)stacked_r1;
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  (void)stacked_r2;
  stacked_r3 = ((unsigned long) hardfault_args[3]);
  (void)stacked_r3;

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  (void)stacked_r12;
  stacked_lr = ((unsigned long) hardfault_args[5]);
  (void)stacked_lr;
  stacked_pc = ((unsigned long) hardfault_args[6]);
  (void)stacked_pc;
  stacked_psr = ((unsigned long) hardfault_args[7]);
  (void)stacked_psr;

  // Do our best to disable the motor driver, so we cause fewer
  // explosions!

  GPIOC->BSRR = (1 << (3 + 16));
  GPIOA->BSRR = (1 << (3 + 16));

  // TODO(jpieper): Verify that the 8323 pin assignments match these
  // hard-coded constants.

  while (1);
}
