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

// Helper function to configure ADC for software trigger and start conversion
inline void ConfigureAdcForSoftwareTrigger(ADC_TypeDef* adc) {
  // Stop any ongoing conversion first
  adc->CR |= ADC_CR_ADSTP;
  while (adc->CR & ADC_CR_ADSTP);

  // Clear external trigger bits to enable software trigger mode
  adc->CFGR &= ~(ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN);
}

// Helper function to configure ADC back to LPTIM1 trigger mode
inline void ConfigureAdcForLptim1Trigger(ADC_TypeDef* adc) {
  // Stop any ongoing conversion first
  adc->CR |= ADC_CR_ADSTP;
  while (adc->CR & ADC_CR_ADSTP);

  // Configure ADC for LPTIM1 external trigger mode
  adc->CFGR = (adc->CFGR & ~(ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN)) |
              (0x1D << ADC_CFGR_EXTSEL_Pos) |  // LPTIM_OUT
              (0x01 << ADC_CFGR_EXTEN_Pos);    // Rising edge trigger

  // Start ADC to be ready for external trigger events
  adc->CR |= ADC_CR_ADSTART;
}

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
