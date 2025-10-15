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

inline PinMode MbedMapPull(aux::Pin::Pull pull) {
  switch (pull) {
    case aux::Pin::kNone: { return PullNone; }
    case aux::Pin::kPullUp: { return PullUp; }
    case aux::Pin::kPullDown: { return PullDown; }
    case aux::Pin::kOpenDrain: { return OpenDrain; }
  }
  return PullNone;
}

/// Figure out which mbed alt pin is associated with the given STM32
/// timer.
inline PinName FindTimerAlt(PinName pin, TIM_TypeDef* timer) {
  const auto int_timer = reinterpret_cast<uint32_t>(timer);

  for (uint32_t alt : {0, 0x100, 0x200, 0x300, 0x400}) {
    const PinName mbed_pin = static_cast<PinName>(pin | alt);
    if (pinmap_find_peripheral(mbed_pin, PinMap_PWM) == int_timer) {
      return mbed_pin;
    }
  }
  return NC;
}

}
