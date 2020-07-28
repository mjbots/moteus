// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

#include "moteus/moteus_hw.h"
#include "moteus/stm32f446_spi.h"

namespace moteus {

class AS5047 {
 public:
  using Options = Stm32F446Spi::Options;

  AS5047(const Options& options)
      : spi_([options]() {
          // The next frequency down is only 6MHz, so we run a bit out
          // of tolerance to save a fair amount of time.
          auto copy = options;
          copy.frequency = 12000000;
          return copy;
        }()) {}

  uint16_t Sample() MOTEUS_CCM_ATTRIBUTE {
    return (spi_.write(0xffff) & 0x3fff) << 2;
  }

  void StartSample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.start_write(0xffff);
  }

  uint16_t FinishSample() MOTEUS_CCM_ATTRIBUTE {
    return (spi_.finish_write() & 0x3fff) << 2;
  }

 private:
  Stm32F446Spi spi_;
};

}
