// Copyright 2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "bldc_foc.h"

#include <functional>

#include "mbed.h"
#include "PeripheralPins.h"

#include "mjlib/base/assert.h"

#include "moteus/irq_callback_table.h"

// TODO
//
//  * Implement current controllers

namespace micro = mjlib::micro;

namespace moteus {

namespace {

// mbed seems to configure the Timer clock input to 90MHz.  We want
// 80kHz up/down rate for 40kHz freqency, so:
constexpr uint32_t kPwmCounts = 90000000 / 80000;

IRQn_Type FindUpdateIrq(TIM_TypeDef* timer) {
  if (timer == TIM1) {
    return TIM1_UP_TIM10_IRQn;
  } else if (timer == TIM2) {
    return TIM2_IRQn;
  } else if (timer == TIM3) {
    return TIM3_IRQn;
  } else if (timer == TIM4) {
    return TIM4_IRQn;
  } else if (timer == TIM8) {
    return TIM8_UP_TIM13_IRQn;
  } else {
    MJ_ASSERT(false);
  }
  return TIM1_UP_TIM10_IRQn;
}

volatile uint32_t* FindCcr(TIM_TypeDef* timer, PinName pin) {
  const auto function = pinmap_function(pin, PinMap_PWM);

  const auto inverted = STM_PIN_INVERTED(function);
  MJ_ASSERT(!inverted);

  const auto channel = STM_PIN_CHANNEL(function);

  switch (channel) {
    case 1: { return &timer->CCR1; }
    case 2: { return &timer->CCR2; }
    case 3: { return &timer->CCR3; }
    case 4: { return &timer->CCR4; }
  }
  MJ_ASSERT(false);
  return nullptr;
}

uint32_t FindSqr(PinName pin) {
  const auto function = pinmap_function(pin, PinMap_ADC);

  const auto channel = STM_PIN_CHANNEL(function);
  return channel;
}
}

class BldcFoc::Impl {
 public:
  Impl(micro::PersistentConfig* persistent_config,
       micro::TelemetryManager* telemetry_manager,
       const Options& options)
      : options_(options),
        pwm1_(options.pwm1),
        pwm2_(options.pwm2),
        pwm3_(options.pwm3),
        current1_(options.current1),
        current2_(options.current2),
        vsense_(options.vsense),
        debug_out_(options.debug_out) {

    persistent_config->Register("bldc", &config_,
                                std::bind(&Impl::UpdateConfig, this));
    telemetry_manager->Register("bldc", &status_);

    MJ_ASSERT(!g_impl_);
    g_impl_ = this;

    ConfigureADC();
    ConfigureTimer();
  }

  ~Impl() {
    g_impl_ = nullptr;
  }

  void Command(const CommandData& data) {
    data_ = data;

    switch (data_.mode) {
      case kDisabled: {
        // TODO(jpieper)
        break;
      }
      case kPhasePwm: {
        (*pwm1_ccr_) = static_cast<uint32_t>(data.phase_a_centipercent) * kPwmCounts / 10000;
        (*pwm2_ccr_) = static_cast<uint32_t>(data.phase_b_centipercent) * kPwmCounts / 10000;
        (*pwm3_ccr_) = static_cast<uint32_t>(data.phase_c_centipercent) * kPwmCounts / 10000;

        status_.phase_a_centipercent = data.phase_a_centipercent;
        status_.phase_b_centipercent = data.phase_b_centipercent;
        status_.phase_c_centipercent = data.phase_c_centipercent;
        break;
      }
      case kFoc: {
        break;
      }
    }
  }

  Status status() const { return status_; }

  void UpdateConfig() {
  }

 private:
  void ConfigureTimer() {
    const auto pwm1_timer = pinmap_peripheral(options_.pwm1, PinMap_PWM);
    const auto pwm2_timer = pinmap_peripheral(options_.pwm2, PinMap_PWM);
    const auto pwm3_timer = pinmap_peripheral(options_.pwm3, PinMap_PWM);

    // All three must be the same and be valid.
    MJ_ASSERT(pwm1_timer != 0 &&
                pwm1_timer == pwm2_timer &&
                pwm2_timer == pwm3_timer);
    timer_ = reinterpret_cast<TIM_TypeDef*>(pwm1_timer);


    pwm1_ccr_ = FindCcr(timer_, options_.pwm1);
    pwm2_ccr_ = FindCcr(timer_, options_.pwm2);
    pwm3_ccr_ = FindCcr(timer_, options_.pwm3);


    // Enable the update interrupt.
    timer_->DIER = TIM_DIER_UIE;

    // Enable the update interrupt.
    timer_->CR1 =
        // Center-aligned mode 2.  The counter counts up and down
        // alternatively.  Output compare interrupt flags of channels
        // configured in output are set only when the counter is
        // counting up.
        (2 << TIM_CR1_CMS_Pos) |

        // ARR register is buffered.
        TIM_CR1_ARPE;

    // Update once per up/down of the counter.
    timer_->RCR |= 0x01;

    // Set up PWM.

    timer_->PSC = 0; // No prescaler.
    timer_->ARR = kPwmCounts;

    // Configure the first three outputs with positive polarity.
    // timer_->CCER =
    //     TIM_CCER_CC1E | TIM_CCER_CC1P |
    //     TIM_CCER_CC2E | TIM_CCER_CC2P |
    //     TIM_CCER_CC3E | TIM_CCER_CC3P;

    // NOTE: We don't use IrqCallbackTable here because we need the
    // absolute minimum latency possible.
    const auto irqn = FindUpdateIrq(timer_);
    NVIC_SetVector(irqn, reinterpret_cast<uint32_t>(&Impl::GlobalInterrupt));
    NVIC_SetPriority(irqn, 2);
    NVIC_EnableIRQ(irqn);

    // Reinitialize the counter and update all registers.
    timer_->EGR |= TIM_EGR_UG;

    // Finally, enable the timer.
    timer_->CR1 |= TIM_CR1_CEN;
  }

  void ConfigureADC() {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();

    // Triple mode: Regular simultaneous mode only.
    ADC->CCR = (0x16 << ADC_CCR_MULTI_Pos);

    // Turn on all the converters.
    ADC1->CR2 = ADC_CR2_ADON;
    ADC2->CR2 = ADC_CR2_ADON;
    ADC3->CR2 = ADC_CR2_ADON;

    // We rely on the AnalogIn members to configure the pins as
    // inputs, however they won't
    ADC1->SQR3 = FindSqr(options_.current1);
    ADC2->SQR3 = FindSqr(options_.current2);
    ADC3->SQR3 = FindSqr(options_.vsense);

    MJ_ASSERT(reinterpret_cast<uint32_t>(ADC1) ==
                pinmap_peripheral(options_.current1, PinMap_ADC));
    MJ_ASSERT(reinterpret_cast<uint32_t>(ADC2) ==
                pinmap_peripheral(options_.current2, PinMap_ADC));
    MJ_ASSERT(reinterpret_cast<uint32_t>(ADC3) ==
                pinmap_peripheral(options_.vsense, PinMap_ADC));

    // Set sample times to 15 cycles across the board
    constexpr uint32_t kCycles = 0x1;  // 15 cycles
    constexpr uint32_t kAll15Cycles =
        (kCycles << 0) |
        (kCycles << 3) |
        (kCycles << 6) |
        (kCycles << 9) |
        (kCycles << 12) |
        (kCycles << 15) |
        (kCycles << 18) |
        (kCycles << 21) |
        (kCycles << 24);
    ADC1->SMPR1 = kAll15Cycles;
    ADC1->SMPR2 = kAll15Cycles;
    ADC2->SMPR1 = kAll15Cycles;
    ADC2->SMPR2 = kAll15Cycles;
    ADC3->SMPR1 = kAll15Cycles;
    ADC3->SMPR2 = kAll15Cycles;
  }

  // CALLED IN INTERRUPT CONTEXT.
  static void GlobalInterrupt() {
    g_impl_->HandleTimer();
  }

  // CALLED IN INTERRUPT CONTEXT.
  void HandleTimer() {
    debug_out_ = 1;

    if (timer_->SR & TIM_SR_UIF) {
      // Start conversion.
      ADC1->CR2 |= ADC_CR2_SWSTART;

      while ((ADC1->SR & ADC_SR_EOC) == 0);

      status_.adc1_raw = ADC1->DR;
      status_.adc2_raw = ADC2->DR;
      status_.adc3_raw = ADC3->DR;

      status_.cur1_A = status_.adc1_raw * config_.i_scale_A;
      status_.cur2_A = status_.adc2_raw * config_.i_scale_A;
      status_.bus_V = status_.adc3_raw * config_.v_scale_V;
    }

    // Reset the status register.
    timer_->SR = 0x00;

    debug_out_ = 0;
  }

  const Options options_;
  Config config_;
  TIM_TypeDef* timer_ = nullptr;
  ADC_TypeDef* const adc1_ = ADC1;
  ADC_TypeDef* const adc2_ = ADC2;
  ADC_TypeDef* const adc3_ = ADC3;

  // We create these to initialize our pins as output and PWM mode,
  // but otherwise don't use them.
  PwmOut pwm1_;
  PwmOut pwm2_;
  PwmOut pwm3_;

  volatile uint32_t* pwm1_ccr_ = nullptr;
  volatile uint32_t* pwm2_ccr_ = nullptr;
  volatile uint32_t* pwm3_ccr_ = nullptr;

  AnalogIn current1_;
  AnalogIn current2_;
  AnalogIn vsense_;

  // This is just for debugging.
  DigitalOut debug_out_;

  CommandData data_;

  Status status_;

  static Impl* g_impl_;
};

BldcFoc::Impl* BldcFoc::Impl::g_impl_ = nullptr;

BldcFoc::BldcFoc(micro::Pool* pool,
                 micro::PersistentConfig* persistent_config,
                 micro::TelemetryManager* telemetry_manager,
                 const Options& options)
    : impl_(pool, persistent_config, telemetry_manager, options) {}
BldcFoc::~BldcFoc() {}

void BldcFoc::Command(const CommandData& data) {
  impl_->Command(data);
}

BldcFoc::Status BldcFoc::status() const {
  return impl_->status();
}

}
