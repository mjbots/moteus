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

#include <optional>

#include "mbed.h"

#include "hal/spi_api.h"

#include "mjlib/base/string_span.h"

#include "fw/ccm.h"
#include "fw/stm32_dma.h"

namespace moteus {

class Stm32Spi {
 public:
  struct Options {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName cs = NC;
    int frequency = 10000000;
    int width = 16;
    int mode = 1;
    uint16_t timeout = 20000;

    // Only necessary if DMA operations will be used.
    DMA_Channel_TypeDef* rx_dma = nullptr;
    DMA_Channel_TypeDef* tx_dma = nullptr;
  };

  Stm32Spi(const Options& options)
      : cs_(std::in_place_t(), options.cs, 1),
        options_(options) {

    spi_init(&spi_, options.mosi, options.miso, options.sck, NC);
    spi_format(&spi_, options.width, options.mode, 0);
    spi_frequency(&spi_, options.frequency);

    auto* const spi = spi_.spi.handle.Instance;
    spi->CR1 &= ~SPI_CR1_SPE;

    if (options_.rx_dma ||
        options_.tx_dma) {
      MJ_ASSERT(options_.rx_dma);
      MJ_ASSERT(options_.tx_dma);

      __HAL_RCC_DMAMUX1_CLK_ENABLE();
      __HAL_RCC_DMA1_CLK_ENABLE();
      __HAL_RCC_DMA2_CLK_ENABLE();

      dmamux_rx_ = Stm32Dma::SelectDmamux(options_.rx_dma);
      dmamux_tx_ = Stm32Dma::SelectDmamux(options_.tx_dma);

      options_.rx_dma->CCR =
          DMA_PERIPH_TO_MEMORY |
          DMA_PINC_DISABLE |
          DMA_MINC_ENABLE |
          DMA_PDATAALIGN_BYTE |
          DMA_MDATAALIGN_BYTE |
          DMA_PRIORITY_HIGH;
      options_.tx_dma->CCR =
          DMA_MEMORY_TO_PERIPH |
          DMA_PINC_DISABLE |
          DMA_MINC_ENABLE |
          DMA_PDATAALIGN_BYTE |
          DMA_MDATAALIGN_BYTE |
          DMA_PRIORITY_HIGH;
      dmamux_rx_->CCR = GetSpiRxRequest(spi) & DMAMUX_CxCR_DMAREQ_ID;
      dmamux_tx_->CCR = GetSpiTxRequest(spi) & DMAMUX_CxCR_DMAREQ_ID;

      options_.tx_dma->CPAR = u32(&spi->DR);
      options_.rx_dma->CPAR = u32(&spi->DR);
    }
  }

  void set_cs(PinName cs) {
    // The SPI class may be used from an interrupt.  If we want to
    // change the CS line, we have to make sure no one can use it
    // while it is being changed.
    __disable_irq();
    cs_.emplace(cs, 1);
    __enable_irq();
  }

  uint16_t write(uint16_t value) MOTEUS_CCM_ATTRIBUTE {
    start_write(value);
    return finish_write();
  }

  void start_write(uint16_t value) MOTEUS_CCM_ATTRIBUTE {
    auto* const spi = spi_.spi.handle.Instance;
    *cs_ = 0;

    // This doesn't seem to be a whole lot faster than the HAL in
    // practice, but it doesn't hurt to do it ourselves and not have
    // to worry about the extra stuff the HAL does.
    uint16_t timeout = options_.timeout;
    while (((spi->SR & SPI_SR_BSY) != 0) && timeout) { timeout--; }
    spi->DR = value;
    spi->CR1 |= SPI_CR1_SPE;
  }

  uint16_t finish_write() MOTEUS_CCM_ATTRIBUTE {
    auto* const spi = spi_.spi.handle.Instance;

    uint16_t timeout = options_.timeout;

    while (((spi->SR & SPI_SR_RXNE) == 0) && timeout) { timeout--; }
    const uint16_t result = spi->DR;
    while (((spi->SR & SPI_SR_TXE) == 0) && timeout) { timeout--; }
    while (((spi->SR & SPI_SR_BSY) != 0) && timeout) { timeout--; }
    spi->CR1 &= ~(SPI_CR1_SPE);

    *cs_ = 1;
    return result;
  }

  void start_dma_transfer(
      std::string_view tx_buffer,
      mjlib::base::string_span rx_buffer) MOTEUS_CCM_ATTRIBUTE {
    *cs_ = 0;

    auto* const spi = spi_.spi.handle.Instance;

    // Empty out the receive FIFO.
    while (spi->SR & SPI_SR_FRLVL_Msk) {
      (void)spi->DR;
    }

    // We should not have a transaction operating at the moment.
    // MJ_ASSERT((spi->CR2 & (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN)) == 0);
    // MJ_ASSERT(tx_buffer.size() == static_cast<size_t>(rx_buffer.size()));

    options_.rx_dma->CNDTR = tx_buffer.size();
    options_.tx_dma->CNDTR = rx_buffer.size();

    options_.rx_dma->CMAR = u32(&rx_buffer[0]);
    options_.tx_dma->CMAR = u32(&tx_buffer[0]);

    spi->CR2 |= SPI_CR2_RXDMAEN;

    options_.tx_dma->CCR |= DMA_CCR_EN;
    options_.rx_dma->CCR |= DMA_CCR_EN;

    spi->CR2 |= SPI_CR2_TXDMAEN;

    spi->CR1 |= SPI_CR1_SPE;
  }

  bool is_dma_finished() MOTEUS_CCM_ATTRIBUTE {
    auto* const spi = spi_.spi.handle.Instance;

    return
        ((spi->SR & SPI_SR_BSY) == 0) &&
        ((spi->SR & SPI_SR_FTLVL_Msk) == 0) &&
        (options_.tx_dma->CNDTR == 0) &&
        (options_.rx_dma->CNDTR == 0);
  }

  void finish_dma_transfer() MOTEUS_CCM_ATTRIBUTE {
    auto* const spi = spi_.spi.handle.Instance;

    while (!is_dma_finished());

    options_.rx_dma->CCR &= ~(DMA_CCR_EN);
    options_.tx_dma->CCR &= ~(DMA_CCR_EN);

    spi->CR1 &= ~(SPI_CR1_SPE);

    spi->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    *cs_ = 1;
  }

 private:
  static uint32_t GetSpiTxRequest(SPI_TypeDef* spi) {
    switch (u32(spi)) {
      case SPI_1: return DMA_REQUEST_SPI1_TX;
      case SPI_2: return DMA_REQUEST_SPI2_TX;
      case SPI_3: return DMA_REQUEST_SPI3_TX;
    }
    mbed_die();
  }

  static uint32_t GetSpiRxRequest(SPI_TypeDef* spi) {
    switch (u32(spi)) {
      case SPI_1: return DMA_REQUEST_SPI1_RX;
      case SPI_2: return DMA_REQUEST_SPI2_RX;
      case SPI_3: return DMA_REQUEST_SPI3_RX;
    }
    mbed_die();
  }

  template <typename T>
  static uint32_t u32(T value) {
    return reinterpret_cast<uint32_t>(value);
  }

  // We don't use the mbed SPI class because we want to be invokable
  // from an ISR.
  spi_t spi_;
  std::optional<DigitalOut> cs_;
  const Options options_;
  DMAMUX_Channel_TypeDef* dmamux_rx_ = nullptr;
  DMAMUX_Channel_TypeDef* dmamux_tx_ = nullptr;
};

}
