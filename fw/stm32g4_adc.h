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

#pragma once

#include "mbed.h"

#include "fw/ccm.h"
#include "fw/millisecond_timer.h"

namespace moteus {

enum class AdcTriggerMode {
  kSoftware,      // Software trigger (manual ADSTART)
  kLptim1,        // Hardware trigger from LPTIM1_OUT
};

inline void DisableAdc(ADC_TypeDef* adc) {
  if (adc->CR & ADC_CR_ADEN) {
    adc->CR |= ADC_CR_ADDIS;
    while (adc->CR & ADC_CR_ADEN);
  }
}

// This function needs to be in CCM memory to achieve deterministic timings.
void EnableAdc(MillisecondTimer* timer, ADC_TypeDef* adc, int prescaler, int offset,
               AdcTriggerMode trigger_mode = AdcTriggerMode::kSoftware) MOTEUS_CCM_ATTRIBUTE;

// Helper to trigger all ADCs simultaneously using LPTIM1
inline void TriggerAllAdcs() {
  // Generate LPTIM_OUT pulse to trigger ADC1-ADC4 simultaneously
  // LPTIM1 is already enabled and configured during initialization
  // Wait for any previous operation to complete
  while (LPTIM1->CR & LPTIM_CR_SNGSTRT);

  // Start single pulse generation
  LPTIM1->CR |= LPTIM_CR_SNGSTRT;
}
}
