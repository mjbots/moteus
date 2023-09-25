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

class AS5047 {
 public:
  using Options = Stm32Spi::Options;

  AS5047(const Options& options)
      : spi_(options) {
  }

  uint16_t Sample() MOTEUS_CCM_ATTRIBUTE {
    return (spi_.write(0xffff) & 0x3fff) << 2;
  }

  void StartSample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.start_write(0xffff);
  }

  uint16_t FinishSample() MOTEUS_CCM_ATTRIBUTE {
    return (spi_.finish_write() & 0x3fff);
  }

 private:
  Stm32Spi spi_;
};

}
