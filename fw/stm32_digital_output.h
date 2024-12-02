// Copyright 2024 mjbots Robotic Systems, LLC.  info@mjbots.com
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

/// Write a digital output pin, entirely in a header file to provide
/// the lowest possible overhead.
class Stm32DigitalOutput {
 public:
  Stm32DigitalOutput(PinName pin, int value) {
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
    reg_out_ = &gpio->ODR;
    reg_set_ = &gpio->BSRR;
    reg_clr_ = &gpio->BRR;
    const auto pin_offset = (static_cast<uint32_t>(pin) & 0xf);
    mask_ = static_cast<uint32_t>(1 << pin_offset);

    write(value);

    const auto mode_mask = 0x3 << (pin_offset * 2);
    gpio->MODER = (gpio->MODER & ~mode_mask) | (1 << (pin_offset * 2));
  }

  void set() {
    *reg_set_ = mask_;
  }

  void clear() {
    *reg_clr_ = mask_;
  }

  void write(int value) {
    if (value) {
      set();
    } else {
      clear();
    }
  }

  void operator=(int value) {
    write(value);
  }

  bool read() {
    return (*reg_out_ & mask_) != 0;
  }

 private:
  volatile uint32_t* reg_out_ = nullptr;
  volatile uint32_t* reg_set_ = nullptr;
  volatile uint32_t* reg_clr_ = nullptr;

  uint32_t mask_ = 0;
};

}
