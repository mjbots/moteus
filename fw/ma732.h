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

class MA732 {
 public:
  enum Model {
    kMa732,
    kMa600,
  };

  struct Options : public Stm32Spi::Options {
    uint16_t filter_us = 1024;
    uint8_t bct = 0;
    uint8_t enable_trim = 0;
    Model model = kMa732;

    Options(const Stm32Spi::Options& v) : Stm32Spi::Options(v) {}
  };

  MA732(MillisecondTimer* timer, const Options& options)
      : timer_(timer),
        spi_([&]() {
          auto copy = options;
          copy.mode = 0;
          return copy;
        }()),
        model_(options.model) {
    error_ = SetConfig(options);
  }

  uint16_t Sample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.write(0x0000);
  }

  void StartSample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.start_write(0x0000);
  }

  uint16_t FinishSample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.finish_write();
  }

  bool error() const { return error_; }

  // Return true on success.
  bool SetConfig(const Options& options) {
    switch (model_) {
      case kMa732: {
        const auto desired_filter = [&]() {
          const auto filter_us = options.filter_us;
          if (filter_us <= 64) { return 51; }
          if (filter_us <= 128) { return 68; }
          if (filter_us <= 256) { return 102; }
          if (filter_us <= 1024) { return 119; }
          if (filter_us <= 2048) { return 136; }
          if (filter_us <= 4096) { return 153; }
          if (filter_us <= 8192) { return 170; }
          if (filter_us <= 16384) { return 187; }
          return 187;
        }();

        // FW = 0x0e
        if (SetRegister(0x0e, desired_filter, 0xff)) { return true; }
        break;
      }
      case kMa600: {
        const auto desired_filter = [&]() {
          const auto filter_us = options.filter_us;
          if (filter_us == 0) { return 0; }
          if (filter_us <= 40) { return 5; }
          if (filter_us <= 80) { return 6; }
          if (filter_us <= 160) { return 7; }
          if (filter_us <= 320) { return 8; }
          if (filter_us <= 640) { return 9; }
          if (filter_us <= 1280) { return 10; }
          if (filter_us <= 2560) { return 11; }
          if (filter_us <= 5120) { return 12; }
          return 12;
        }();

        // FW = 0x0d[3:0]
        if (SetRegister(0x0d, desired_filter, 0xff)) { return true; }
        break;
      }
    }

    // BCT = 0x02
    if (SetRegister(0x02, options.bct, 0xff)) { return true; }

    // ENABLE TRIMMING = 0x03
    if (SetRegister(0x03, options.enable_trim, 0x03)) { return true; }

    return false;
  }

 private:
  bool SetRegister(uint8_t reg, uint8_t desired, uint8_t mask) {
    switch (model_) {
      case kMa732: {
        spi_.write(0x4000 | (reg << 8));

        timer_->wait_us(2);

        const auto current_value = spi_.write(0x0000) >> 8;

        timer_->wait_us(2);

        if ((current_value & mask) == (desired & mask)) { return false; }

        spi_.write(0x8000 | (reg << 8) | desired);

        // Eventually it would be nice to restructure this so as to not
        // block the main loop for 20ms every time we change the MA732
        // config.  However, given that each device only has ~1000 writes
        // available, it isn't like it is something that will be happening
        // all that often.
        timer_->wait_ms(20);

        const auto final_value = (spi_.write(0x0000) >> 8);
        return (final_value & mask) != (desired & mask);
      }
      case kMa600: {
        const uint16_t read_reg_cmd = (0xd2 << 8) | reg;
        spi_.write(read_reg_cmd);

        timer_->wait_us(2);

        const auto raw_value = spi_.write(0x0000);

        const auto current_value = raw_value & 0xff;
        timer_->wait_us(2);

        if ((current_value & mask) == (desired & mask)) { return false; }

        spi_.write(0xea54);
        timer_->wait_us(2);
        const uint16_t write_reg_cmd = (reg << 8) | desired;
        spi_.write(write_reg_cmd);
        timer_->wait_us(2);
        const auto raw_final = spi_.write(0x0000);
        timer_->wait_us(2);
        const auto final_value = raw_final & 0xff;

        return (final_value & mask) != (desired & mask);
      }
    }
    return true;
  }

  MillisecondTimer* const timer_;
  Stm32Spi spi_;
  Model model_ = kMa732;
  bool error_ = false;
};

}
