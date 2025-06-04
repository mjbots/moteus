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

  CuiAmt22(const Options& options, MillisecondTimer* timer)
      : spi_([&]() {
               auto options_copy = options;
               options_copy.width = 8;
               options_copy.mode = 0;
               return options_copy;
             }()),
        cs_(std::in_place_t(), options.cs, 1),
        timer_(timer) {
  }

  uint16_t Sample() MOTEUS_CCM_ATTRIBUTE {
    ISR_StartSample();
    uint32_t value = 0;
    uint32_t timeout = 2000;
    while (!ISR_Update(&value) && timeout--) {}
    return value;
  }

  void ISR_StartSample() MOTEUS_CCM_ATTRIBUTE {
    if (state_ == 4) {
      state_ = 0;
    }
  }

  // return value: boolean whether or not we finished reading the sensor and updated the value
  int ISR_Update(uint32_t* value_ptr) MOTEUS_CCM_ATTRIBUTE {
    const uint32_t now_us = timer_->read_us();                
    const uint32_t delta_us = (now_us - last_byte_finished_us_);
    uint16_t value = 0;
    switch (state_) {
      case 0:
        cs_->clear();
        if (delta_us > 3) {
          spi_.start_write_no_cs(0x00);
          state_ = 1;
        }
        return 0;
      case 1:
        buffer_[0] = spi_.finish_write_no_cs();
        last_byte_finished_us_ = now_us;
        state_ = 2;
        return 0;
      case 2:
        if (delta_us > 3) {
          spi_.start_write_no_cs(0x00);
          state_ = 3;
        }
        return 0;
      case 3:
        buffer_[1] = spi_.finish_write_no_cs();
        last_byte_finished_us_ = now_us;
        cs_->set();
        value = 
          ((static_cast<uint16_t>(buffer_[0]) << 8)
          | static_cast<uint16_t>(buffer_[1]))
          & 0x3fff;
        *value_ptr = value;
        state_ = 4;
        return 1;
      default:
        state_ = 4;
        return 0;
    }
  }

 private:
  Stm32Spi spi_;
  std::optional<Stm32DigitalOutput> cs_;
  MillisecondTimer* const timer_;
  int state_ = 4;
  uint32_t last_byte_finished_us_ = 0;
  uint8_t buffer_[2];
};

}
