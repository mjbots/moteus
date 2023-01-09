// Copyright 2018-2023 Josh Pieper, jjp@pobox.com.
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

#pragma once

#include "mbed.h"

#include "fw/millisecond_timer.h"

namespace moteus {

inline void DisableAdc(ADC_TypeDef* adc) {
  if (adc->CR & ADC_CR_ADEN) {
    adc->CR |= ADC_CR_ADDIS;
    while (adc->CR & ADC_CR_ADEN);
  }
}

inline void EnableAdc(MillisecondTimer* timer, ADC_TypeDef* adc) {
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
  adc->CR |= ADC_CR_ADEN;

  while (! (adc->ISR & ADC_ISR_ADRDY));
  adc->ISR |= ADC_ISR_ADRDY;

  adc->CFGR &= ~(ADC_CFGR_CONT);
}

}
