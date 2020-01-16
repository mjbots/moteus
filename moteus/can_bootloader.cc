// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#if defined(TARGET_STM32G4)
#include "stm32g4xx.h"
#else
#error "Unsupported target"
#endif

#include <string.h>

#include <cstdint>

namespace {
void BadInterrupt() {
  while (true);
}
}

extern "C" {

extern uint8_t __bss_start__;
extern uint8_t __bss_end__;

void __attribute__((__section__(".multiplex_bootloader")))
MultiplexBootloader(uint8_t source_id,
                    USART_TypeDef* uart,
                    GPIO_TypeDef* direction_port,
                    int direction_pin) {
  // While we are bootloading, we want no interrupts whatsoever.
  __disable_irq();

  // Manually zero out our BSS.
  ::memset(&__bss_start__, 0, &__bss_end__ - &__bss_start__);

  // We don't want any handlers to go into the original application
  // code, so point everything to a noop.
  for (int i = 0; i <= 113; i++) {
    const auto irq = static_cast<IRQn_Type>(i);

    if (irq == DebugMonitor_IRQn) { continue; }
    NVIC_SetVector(irq, reinterpret_cast<uint32_t>(&BadInterrupt));
  }

  for (;;) {}
}

void abort() {
  while (true) {}
}

}

namespace mjlib {
namespace base {

void assertion_failed(const char* expression, const char* filename, int line) {
  while (true);
}

}
}
