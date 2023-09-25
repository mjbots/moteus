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

#include <atomic>

#include "mbed.h"

#include "fw/ccm.h"

namespace moteus {

class AuxADC {
 public:
  struct PinConfig {
    int adc_num = -1;
    int channel = -1;
  };

  static constexpr int kMaxAdcs = 3;
  static constexpr int kMaxPins = 5;

  struct AuxInfo {
    std::array<PinConfig, kMaxPins> config = {};
    std::array<uint16_t, kMaxPins> value = {};
    mjlib::base::inplace_function<void()> config_update;
  };

  // These structures are passed by reference to each of the AuxPort
  // instances.  They are responsible for filling in the config and
  // then invoking the "config_update" member.  Then they can read the
  // current values out of AuxInfo::value in the ISR.
  AuxInfo aux_info[2] = {};

  AuxADC() {
    for (auto& info : aux_info) {
      info.config_update = std::bind(&AuxADC::UpdateConfig, this);
    }
  }

  void ISR_StartSample() MOTEUS_CCM_ATTRIBUTE {
    if (!any_adc_.load()) { return; }

    if (adc_configs_[0].num_channels) { ADC1->CR |= ADC_CR_JADSTART; }
    if (adc_configs_[1].num_channels) { ADC2->CR |= ADC_CR_JADSTART; }
    if (adc_configs_[2].num_channels) { ADC3->CR |= ADC_CR_JADSTART; }
    static_assert(kMaxAdcs == 3);
    // if (adc_configs_[3].num_channels) { ADC4->CR |= ADC_CR_JADSTART; }
    // if (adc_configs_[4].num_channels) { ADC5->CR |= ADC_CR_JADSTART; }
  }

  void ISR_EndSample() MOTEUS_CCM_ATTRIBUTE {
    if (!any_adc_.load()) { return; }

    // Maybe wait for EOC?

    for (int adc_num = 0; adc_num < kMaxAdcs; adc_num++) {
      for (int j = 0; j < adc_configs_[adc_num].num_channels; j++) {
        const uint16_t data = (&adcs_[adc_num]->JDR1)[j];
        *adc_configs_[adc_num].result_store[j] = data;
      }
    }
  }

 private:
  void UpdateConfig() {
    any_adc_.store(false);

    // Zero out our config.
    for (int i = 0; i < kMaxAdcs; i++) {
      adcs_[i]->JSQR = 0;
      adc_configs_[i] = {};
    }

    // Zero out our status.
    for (auto& info : aux_info) {
      info.value = {};
    }

    bool new_any_adc = false;

    auto process_aux_config =
        [&](AuxInfo* info) {
          for (size_t i = 0; i < info->config.size(); i++) {
            const auto& cfg = info->config[i];
            auto* value = &info->value[i];
            if (cfg.adc_num < 0) { continue; }

            if (cfg.adc_num >= static_cast<int>(adc_configs_.size())) {
              mbed_die();
            }

            new_any_adc = true;

            auto& this_adc = adc_configs_[cfg.adc_num];
            const int pos =
                this_adc.num_channels == 0 ? ADC_JSQR_JSQ1_Pos :
                this_adc.num_channels == 1 ? ADC_JSQR_JSQ2_Pos :
                this_adc.num_channels == 2 ? ADC_JSQR_JSQ3_Pos :
                this_adc.num_channels == 3 ? ADC_JSQR_JSQ4_Pos : 0;

            adcs_[cfg.adc_num]->JSQR |= (cfg.channel << pos);
            this_adc.result_store[this_adc.num_channels] = value;
            this_adc.num_channels++;

            // Each ADC only supports at most 4 injected channels.
            if (this_adc.num_channels > 4) {
              mbed_die();
            }
          }
        };

    process_aux_config(&aux_info[0]);
    process_aux_config(&aux_info[1]);

    // Now set the injected length appropriately.

    for (int i = 0; i < kMaxAdcs; i++) {
      adcs_[i]->JSQR = adcs_[i]->JSQR |
          (((adc_configs_[i].num_channels - 1) & 0x03) << ADC_JSQR_JL_Pos);
    }

    // All ready to go!
    any_adc_.store(new_any_adc);
  }

  struct AdcConfig {
    int num_channels = 0;
    std::array<uint16_t*, kMaxPins> result_store = {};
  };

  std::array<AdcConfig, kMaxAdcs> adc_configs_ = {};

  std::atomic<bool> any_adc_{false};

  ADC_TypeDef* adcs_[5] = {
    ADC1,
    ADC2,
    ADC3,
    ADC4,
    ADC5,
  };
};

}
