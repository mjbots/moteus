// Copyright 2025 Eli Rutan.  polymetricofficial@gmail.com
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

  /// @return true if we finished reading the sensor and updated the value
  bool ISR_Update(uint32_t* value_ptr) MOTEUS_CCM_ATTRIBUTE {
    const uint32_t now_us = timer_->read_us();                
    const uint32_t delta_us = (now_us - last_byte_finished_us_);
    uint16_t value = 0;
    switch (state_) {
      case State::kClearCs: {
        if (delta_us >= AMT22_TIME_BETWEEN_READS) {
          cs_->clear();
          state_ = State::kStartFirstByte;
        }
        return false;
      }
      case State::kStartFirstByte: {
        if (delta_us >= AMT22_TIME_AFTER_CS) {
          spi_.start_byte(0x00);
          state_ = State::kFinishFirstByte;
        }
        return false;
      }
			case State::kStartSecondByte: {
        buffer_[0] = spi_.finish_byte();
      	spi_.start_byte(0x00);
      	state_ = State::kFinishSecondByte;
        return false;
      }
      case State::kFinishSecondByte: {
        buffer_[1] = spi_.finish_byte();
        last_byte_finished_us_ = now_us;
        cs_->set();
        value = 
          ((static_cast<uint16_t>(buffer_[0]) << 8)
          | static_cast<uint16_t>(buffer_[1]))
          & 0x3fff;
        *value_ptr = value;
        state_ = State::kClearCs;
        return true;
      }
			default: {
				MJ_ASSERT(false);
				return false;
			}
    }
  }

 private:
  Stm32Spi spi_;
  std::optional<Stm32DigitalOutput> cs_;
  MillisecondTimer* const timer_;

	enum class State {
	  kClearCs,
		kStartFirstByte,
		kFinishFirstByte,
		kStartSecondByte,
		kFinishSecondByte,
	};

  State state_ = State::kClearCs;
  uint32_t last_byte_finished_us_ = 0;
  uint8_t buffer_[2] = {};

  static constexpr uint32_t AMT22_TIME_BETWEEN_READS = 40;
  static constexpr uint32_t AMT22_TIME_BETWEEN_BYTES = 3;
  static constexpr uint32_t AMT22_TIME_AFTER_CS = 3;
};

}
