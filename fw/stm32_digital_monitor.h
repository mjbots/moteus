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

namespace moteus {

/// Read a digital input, but without configuring it in any way.
class Stm32DigitalMonitor {
 public:
  Stm32DigitalMonitor(PinName pin) {
    const uint32_t port_index1 = STM_PORT(pin);
    GPIO_TypeDef* gpio = reinterpret_cast<GPIO_TypeDef*>([&]() {
      switch (port_index1) {
        case PortA: return GPIOA_BASE;
        case PortB: return GPIOB_BASE;
        case PortC: return GPIOC_BASE;
        case PortD: return GPIOD_BASE;
        case PortE: return GPIOE_BASE;
        case PortF: return GPIOF_BASE;
      }
      MJ_ASSERT(false);
      return GPIOA_BASE;
      }());
    reg_in_ = &gpio->IDR;
    mask_ = static_cast<uint32_t>(1 << (static_cast<uint32_t>(pin) & 0xf));
  }

  bool read() {
    return (*reg_in_ & mask_) != 0;
  }

 private:
  volatile uint32_t* reg_in_ = nullptr;
  uint32_t mask_ = 0;
};

}
