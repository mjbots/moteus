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

#include "fw/stm32g4_adc.h"

namespace moteus {

void EnableAdc(MillisecondTimer* timer, ADC_TypeDef* adc, int prescaler, int offset) {
  // 20.4.6: ADC Deep power-down mode startup procedure
  adc->CR &= ~ADC_CR_DEEPPWD;
  adc->CR |= ADC_CR_ADVREGEN;

  // tADCREG_S = 20us per STM32G474xB datasheet
  timer->wait_us(20);

  adc->CR |= ADC_CR_ADCAL;
  while (adc->CR & ADC_CR_ADCAL);
  timer->wait_us(1);

  // 20.4.9: Software procedure to enable the ADC
  adc->ISR |= ADC_ISR_ADRDY;



  // Note: The STM32G4 has many ADCs, ADC1/2 share a prescaler as do
  // 3/4.  When the 3/4 post-prescaler clocks are aligned at some
  // phases, the ADC1/2 results can be corrupted (assuming ADC3/4 have
  // their sampling started immediately after 1/2).  To make this more
  // repeatable, we ensure that the ADC is enabled at a known phase
  // with respect to the global cycle counter as measured by the DWT.

  // To do this, we use the DWT's cycle counter to delay until it hits
  // a value that modulo the prescaler equals zero exactly.  We first
  // calculate this, then jump through a bunch of hoops to reliably
  // get the STM32G4 to have a cycle accurate time.
  const uint32_t initial_cyccnt = DWT->CYCCNT;
  const uint32_t desired_cyccnt =
      static_cast<uint32_t>(initial_cyccnt / prescaler) * prescaler +
      prescaler * 64 + offset;

  // Clear the instruction and data cache, to help get the same
  // results every time.
  FLASH->ACR &= ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
  FLASH->ACR |= FLASH_ACR_ICRST | FLASH_ACR_DCRST;
  FLASH->ACR &= ~(FLASH_ACR_ICRST | FLASH_ACR_DCRST);
  FLASH->ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN);

  uint8_t buf[16];
  uint32_t tmp;
  uint32_t loop_count;
  uint32_t remainder;
  asm volatile (
      // Align ourselves to improve robustness.
      ".balign 4;"

      // Capture the current CYCCNT, subtract it from our desired and
      // a hand-tuned extra amount.
      "ldr %[tmp], [%[cyccnt], #0];"
      "sub %[tmp], %[desired], %[tmp];"
      "sub %[tmp], %[tmp], #16;"

      // Our ultimate delay loop takes 4 cycles per iteration.
      "asr %[loop_count], %[tmp], #2;"

      // Which means we may have up to 3 extra NOPs to insert.  This
      // part of the code requires the function to be placed in CCM
      // memory, otherwise the FLASH ART destroys our ability to add
      // single cycle delays.
      "and %[remainder], %[tmp], #3;"

      "cmp %[remainder], #0;"
      "ble 10f;"
      "mov %[tmp], #0;"
      "mov %[tmp], #0;"
      "10:;"

      "cmp %[remainder], #1;"
      "ble 11f;"
      "mov %[tmp], #0;"
      "mov %[tmp], #0;"
      "11:;"

      "cmp %[remainder], #2;"
      "ble 12f;"
      "mov %[tmp], #0;"
      "mov %[tmp], #0;"
      "12:;"

      // Make sure we are at the beginning of any pre-fetch lines.
      ".balign 4;"

      // Our main loop, which takes 4 cycles.
      "1:;"
      "subs %[loop_count], %[loop_count], #1;"
      "nop;"
      "bne 1b;"

      "nop;"

      // To verify this result, use a debugger and set a breakpoint at
      // the next instruction after this inline block.  At that point,
      // compare MyDWT->CYCCNT to desired_cycnt.  Note that you cannot
      // single step or use any breakpoints at any intermediate point,
      // or the results will not be meaningful.
      : [tmp]"=&r"(tmp),
        [loop_count]"=&r"(loop_count),
        [remainder]"=&r"(remainder)
      : [cyccnt]"r"(&DWT->CYCCNT),
        [desired]"r"(desired_cyccnt),
        [buf]"r"(buf)
      :
  );

  adc->CR |= ADC_CR_ADEN;

  // This is just to force the above line to not be re-ordered.
  asm volatile (
      "nop;"
  );

  while (! (adc->ISR & ADC_ISR_ADRDY));
  adc->ISR |= ADC_ISR_ADRDY;

  adc->CFGR &= ~(ADC_CFGR_CONT);
}

#if 0
// We leave this commented out because it was hard to create, and
// might be useful for debugging again.

// Delay a cycle accurate number of counts.  This allows code to vary
// how long it takes in a cycle accurate way without changing its
// layout, which can often have large knock-on effects in other
// locations.
void CycleAccurateDelay(int counts) {
  // Clear the instruction and data cache, to help get the same
  // results every time.
  FLASH->ACR &= ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
  FLASH->ACR |= FLASH_ACR_ICRST | FLASH_ACR_DCRST;
  FLASH->ACR &= ~(FLASH_ACR_ICRST | FLASH_ACR_DCRST);
  FLASH->ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN);

  uint32_t loop_count;
  uint32_t remainder;
  asm volatile (
      // Align ourselves to improve robustness.
      ".balign 4;"

      // Our ultimate delay loop takes 4 cycles per iteration.
      "asr %[loop_count], %[counts], #2;"

      // Which means we may have up to 3 extra NOPs to insert.  This
      // part of the code requires the function to be placed in CCM
      // memory, otherwise the FLASH ART destroys our ability to add
      // single cycle delays.
      "and %[remainder], %[counts], #3;"

      "cmp %[remainder], #0;"
      "ble 10f;"
      "mov %[counts], #0;"
      "mov %[counts], #0;"
      "10:;"

      "cmp %[remainder], #1;"
      "ble 11f;"
      "mov %[counts], #0;"
      "mov %[counts], #0;"
      "11:;"

      "cmp %[remainder], #2;"
      "ble 12f;"
      "mov %[counts], #0;"
      "mov %[counts], #0;"
      "12:;"

      // Make sure we are at the beginning of any pre-fetch lines.
      ".balign 4;"

      // Our main loop, which takes 4 cycles.
      "1:;"
      "subs %[loop_count], %[loop_count], #1;"
      "nop;"
      "bne 1b;"

      "nop;"
      : [counts]"+&r"(counts),
        [loop_count]"=&r"(loop_count),
        [remainder]"=&r"(remainder)
      : [cyccnt]"r"(&DWT->CYCCNT)
      :
  );
}
#endif
}
