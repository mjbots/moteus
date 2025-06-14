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
#include "fw/aux_common.h"


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

  /// @return true if we finished reading the sensor and updated the value
  bool ISR_Update(aux::Spi::Status *status) MOTEUS_CCM_ATTRIBUTE {
    uint16_t value = 0;
    switch (state_) {
      case State::kClearCs: {
        cs_->clear();
        state_ = State::kStartFirstByte;
        return false;
      }
      case State::kStartFirstByte: {
        spi_.start_byte(0x00);
        state_ = State::kStartSecondByte;
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
        cs_->set();

        if (n_ignored_samples_ < 1) {
          n_ignored_samples_++;
          state_ = State::kClearCs;
          return false;
        }

        value =
          ((static_cast<uint16_t>(buffer_[0]) << 8)
          | static_cast<uint16_t>(buffer_[1]));

        // Check parity - from page 2 of the datasheet:
        // https://www.sameskydevices.com/product/resource/amt22.pdf
        const bool received_even_parity = value >> 14 & 1;
        const bool received_odd_parity = value >> 15 & 1;

        const bool calculated_odd_parity = !(
            (value >> 13 & 1) ^
            (value >> 11 & 1) ^
            (value >> 9  & 1) ^
            (value >> 7  & 1) ^
            (value >> 5  & 1) ^
            (value >> 3  & 1) ^
            (value >> 1  & 1)
        );
        const bool calculated_even_parity = !(
            (value >> 12 & 1) ^
            (value >> 10 & 1) ^
            (value >>  8 & 1) ^
            (value >>  6 & 1) ^
            (value >>  4 & 1) ^
            (value >>  2 & 1) ^
            (value >>  0 & 1)
        );

        if (received_odd_parity != calculated_odd_parity ||
            received_even_parity != calculated_even_parity) {
          // Parity failed, just wait for the next sample
          status->checksum_errors++;
          state_ = State::kClearCs;
          return false;
        }

        status->value = value & 0x3fff;
        status->active = true;
        status->nonce++;
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

  enum class State {
    kClearCs,
    kStartFirstByte,
    kStartSecondByte,
    kFinishSecondByte,
  };

  State state_ = State::kClearCs;
  uint8_t buffer_[2] = {};
  uint16_t n_ignored_samples_ = 0;
};

}
