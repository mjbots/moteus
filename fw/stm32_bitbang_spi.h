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

#include "fw/millisecond_timer.h"

namespace moteus {
class Stm32BitbangSpi {
 public:
  struct Options {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName cs = NC;
    int frequency = 10000000;
    int width = 16;
    int mode = 1;
  };

  Stm32BitbangSpi(MillisecondTimer* timer,
                  const Options& options)
      : timer_(timer),
        cs_(options.cs, 1),
        mosi_(options.mosi, 0),
        miso_(options.miso),
        sck_(options.sck, 0),
        options_(options) {
    if (options.mode != 1) { mbed_die(); }

    us_delay_ = std::max(1, 500000 / options.frequency);
  }

  uint16_t write(uint16_t value) {
    cs_.write(0);
    timer_->wait_us(us_delay_);

    uint16_t result = 0;

    for (int i = options_.width; i > 0; i--) {
      mosi_.write((value & (1 << (i - 1))) ? 1 : 0);
      sck_.write(1);
      timer_->wait_us(us_delay_);

      sck_.write(0);
      result <<= 1;
      result |= miso_.read() ? 1 : 0;
      timer_->wait_us(us_delay_);
    }

    mosi_.write(0);
    cs_.write(1);

    timer_->wait_us(us_delay_);

    return result;
  }

  MillisecondTimer* const timer_;

  DigitalOut cs_;
  DigitalOut mosi_;
  DigitalIn miso_;
  DigitalOut sck_;

  const Options options_;

  uint32_t us_delay_ = 1;
};
}
