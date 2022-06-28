// Copyright 2018-2022 Josh Pieper, jjp@pobox.com.
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

#include "fw/moteus_hw.h"

#include "mjlib/base/assert.h"

namespace moteus {

namespace {
//////////////////////////////////////////
// moteus - family 0

// The following silks correspond with the following hardware
// revisions:
//
//  "r1"           - 0
//  "r2"           - 1
//  "r3"           - 2
//  "r4.1"         - 3
//  "r4.2", "r4.3" - 4
//  "r4.4"         - 5
//  "r4.5"         - 6
//  "r4.5b"-"r4.8" - 7
//  "r4.11"        - 8

// The mapping between  and the version pins on the
// board.
constexpr int kFamily0HardwareInterlock[] = {
  -1,  // r1 (never printed for g4)
  -1,  // r2 (never printed for g4)
  -1,  // r3 (never printed for g4)
  0,   // r4.1
  0,   // r4.2/r4.3 (unfortunately, indistinguishable from the interlock)
  1,   // r4.4
  2,   // r4.5
  3,   // r4.5b-r4.8
  4,   // r4.10
};
}

FamilyAndVersion DetectMoteusFamily() {
  FamilyAndVersion result;
  result.family = 0;

  if (result.family == 0) {
    DigitalIn hwrev0(PC_6, PullUp);
    DigitalIn hwrev1(PA_15, PullUp);
    // Previously this was documented as PC_13, however we never
    // pulled it down, and decided to use PC_13 for something else.
    DigitalIn hwrev2(PA_10, PullUp);

    const uint8_t this_hw_pins =
        0x07 & (~(hwrev0.read() |
                  (hwrev1.read() << 1) |
                  (hwrev2.read() << 2)));
    const uint8_t measured_hw_rev =
        [&]() {
          int i = 0;
          for (auto rev_pins : kFamily0HardwareInterlock) {
            if (rev_pins == this_hw_pins) { return i; }
            i++;
          }
          return -1;
        }();
    result.hw_version = measured_hw_rev;
  } else if (result.family == 1) {
  } else {
    MJ_ASSERT(false);
  }

  return result;
}

namespace {
PinName unsupported() {
  mbed_die();
  return NC;
}
}

MoteusHwPins FindHardwarePins(FamilyAndVersion fv) {
  MoteusHwPins result;

  const auto hv = fv.hw_version;

  if (fv.family == 0) {
    result.vsense =
        (hv <= 4 ? PA_8 :
         hv >= 5 ? PB_12_ALT0 :
         unsupported());

    result.msense =
        (hv <= 3 ? NC :
         hv == 4 ? PB_12 :
         hv >= 5 ? PA_8 :
         unsupported());

    result.vsense_adc_scale =
        (hv <= 5 ? 0.00884f : 0.017947f);
  } else {
    mbed_die();
  }

  return result;
}

}
