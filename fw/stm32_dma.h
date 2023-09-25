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

class Stm32Dma {
 public:
  static uint32_t FindChannelIndex(DMA_Channel_TypeDef* channel) {
    if (channel < DMA2_Channel1) {
      return ((u32(channel) - u32(DMA1_Channel1)) /
              (u32(DMA1_Channel2) - u32(DMA1_Channel1))) << 2;
    }
    return ((u32(channel) - u32(DMA2_Channel1)) /
            (u32(DMA2_Channel2) - u32(DMA2_Channel1))) << 2;
  }

  static DMAMUX_Channel_TypeDef* SelectDmamux(DMA_Channel_TypeDef* channel) {
    const auto base = (channel < DMA2_Channel1) ? DMAMUX1_Channel0 :
#if defined (STM32G471xx) || defined (STM32G473xx) || defined (STM32G474xx) || defined (STM32G483xx) || defined (STM32G484xx)
        DMAMUX1_Channel8;
#elif defined (STM32G431xx) || defined (STM32G441xx) || defined (STM32GBK1CB)
    DMAMUX1_Channel6;
#else
    DMAMUX1_Channel7;
#endif /* STM32G4x1xx) */
    return reinterpret_cast<DMAMUX_Channel_TypeDef*>(
        u32(base) + (FindChannelIndex(channel) >> 2U) *
        (u32(DMAMUX1_Channel1) - u32(DMAMUX1_Channel0)));
  }

  template <typename T>
  static uint32_t u32(T value) {
    return reinterpret_cast<uint32_t>(value);
  }
};

}
