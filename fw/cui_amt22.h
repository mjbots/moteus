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

#include "hal/spi_api.h"

#include "fw/ccm.h"
#include "fw/moteus_hw.h"
#include "fw/stm32_spi.h"

namespace moteus {

class CuiAmt22 {
 public:
  using Options = Stm32Spi::Options;

  CuiAmt22(const Options& options)
      : spi_([&]() {
               auto options_copy = options;
               options_copy.width = 8;
               options_copy.mode = 0;
               return options_copy;
             }()),
        cs_(std::in_place_t(), options.cs, 1) {
  }

  uint16_t Sample() MOTEUS_CCM_ATTRIBUTE {
    return 0;
  }

  void StartSample() MOTEUS_CCM_ATTRIBUTE {
    return;
  }

  uint16_t FinishSample() MOTEUS_CCM_ATTRIBUTE {
    cs_->clear();
    uint8_t a = spi_.write_no_cs(0x00);
    // ideally this would be async instead of locking up the main thread
    // but this is much simpler and it's not a very long delay so it works fine for now
    // wait(50) comes out to 3.7 us on the 'scope
    wait(50);
    uint8_t b = spi_.write_no_cs(0x00);
    cs_->set();
    return (((uint16_t) a) << 8 | ((uint16_t) b)) & 0x3fff;
  }


 private:
  // using an empty loop as a less precise delay because MillisecondTimer adds
  // a ton of jitter when the signal is viewed on the oscilloscope.
  //
  // also, using an optimize attribute ensures the compiler doesn't optimize
  // away our empty loop
  void __attribute__((optimize("O0"))) wait(int cycles) MOTEUS_CCM_ATTRIBUTE {
    for (int i = 0; i < cycles; i++) {}
  }
  Stm32Spi spi_;
  std::optional<Stm32DigitalOutput> cs_;
};

}
