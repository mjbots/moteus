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

namespace moteus {

class Stm32F446Spi {
 public:
  struct Options {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName cs = NC;
    int frequency = 10000000;
    int width = 16;
  };

  Stm32F446Spi(const Options& options)
      : cs_(options.cs, 1) {

    spi_init(&spi_, options.mosi, options.miso, options.sck, NC);
    spi_format(&spi_, options.width, 1, 0);
    spi_frequency(&spi_, options.frequency);
  }

  uint16_t write(uint16_t value) {
    // NOTE: This seems to take around 7us (this is 28% of our full
    // cycle period).  I think that if I didn't go through the HAL, I
    // could get this down to something under 2us, which would be more
    // reasonable.

    auto* const spi = spi_.spi.handle.Instance;
    cs_ = 0;

    // This doesn't seem to be a whole lot faster than the HAL in
    // practice, but it doesn't hurt to do it ourselves and not have
    // to worry about the extra stuff the HAL does.
    while ((spi->SR & SPI_SR_BSY) != 0);
    spi->DR = value;
    spi->CR1 |= SPI_CR1_SPE;
    while ((spi->SR & SPI_SR_RXNE) == 0);
    const uint16_t result = spi->DR;
    while ((spi->SR & SPI_SR_TXE) == 0);
    while ((spi->SR & SPI_SR_BSY) != 0);
    spi->CR1 &= ~(SPI_CR1_SPE);

    cs_ = 1;
    return result;
  }

 private:
  // We don't use the mbed SPI class because we want to be invokable
  // from an ISR.
  spi_t spi_;
  DigitalOut cs_;
};

}
