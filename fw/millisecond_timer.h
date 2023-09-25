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

#ifdef wait_us
#undef wait_us
#endif

namespace moteus {

class MillisecondTimer {
 public:
  MillisecondTimer() {
    TIM_MST_RCC;

    constexpr int kExtraPrescaler =
#if defined(TARGET_STM32G4)
        1
#else
#error "Unknown target"
#endif
        ;

    handle_.Instance = TIM_MST;
#if TIM_MST_BIT_WIDTH == 32
    handle_.Init.Period = 0xffffffff;
#elif TIM_MST_BIT_WIDTH == 16
    handle_.Init.Period = 0xffff;
#endif
    handle_.Init.Prescaler =
        (uint32_t)(SystemCoreClock / kExtraPrescaler /
                   1000000U) - 1;  // 1 us tick
    handle_.Init.ClockDivision = 0;
    handle_.Init.CounterMode = TIM_COUNTERMODE_UP;
    handle_.Init.RepetitionCounter = 0;

    HAL_TIM_Base_Init(&handle_);
  }

#if TIM_MST_BIT_WIDTH == 32
  using TimerType = uint32_t;
#elif TIM_MST_BIT_WIDTH == 16
  using TimerType = uint16_t;
#else
# error "unsupported timer"
#endif

  TimerType read_ms() {
    return TIM_MST->CNT / 1000;
  }

  TimerType read_us() {
    return TIM_MST->CNT;
  }

  static TimerType subtract_us(TimerType a, TimerType b) {
    return static_cast<TimerType>(a - b);
  }

  void wait_ms(uint32_t delay_ms) {
    wait_us(delay_ms * 1000);
  }

  void wait_us(uint32_t delay_us) {
    while (delay_us > 50000) {
      wait_us_helper(50000);
      delay_us -= 50000;
    }
    wait_us_helper(delay_us);
  }

  void wait_us_helper(uint32_t delay_us) {
    TimerType current = TIM_MST->CNT;
    TimerType elapsed = 0;
    while (true) {
      const TimerType next = TIM_MST->CNT;
      elapsed += static_cast<TimerType>(next - current);
      // We check delay_us + 1 since we don't know where in the
      // current microsecond we started.
      if (elapsed >= (delay_us + 1)) { return; }
      current = next;
    }
  }

 private:
  TIM_HandleTypeDef handle_ = {};
};

}
