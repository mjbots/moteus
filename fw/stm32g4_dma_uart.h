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

#include "PinNames.h"

#include "mjlib/base/string_span.h"

#include "fw/ccm.h"
#include "fw/stm32_dma.h"
#include "fw/stm32_serial.h"

namespace moteus {

class Stm32G4DmaUart {
 public:
  struct Options {
    PinName tx = NC;
    PinName rx = NC;

    int baud_rate = 115200;

    DMA_Channel_TypeDef* rx_dma = DMA1_Channel1;
    DMA_Channel_TypeDef* tx_dma = DMA1_Channel2;

    // Optional RS485 half-duplex driver-enable (active high), owned by
    // the caller.  When set, the transmit paths assert this pin before
    // sending and it is automatically de-asserted from the USART
    // transmission-complete interrupt once the final stop bit has left
    // the shift register.  This releases the bus so a device sharing a
    // single differential pair (TX+/RX+ and TX-/RX- tied together) can
    // drive its reply.  Leave as nullptr for 4-wire RS422 or non-RS485
    // use, where the driver is enabled statically by the caller.
    DigitalOut* de = nullptr;

    // NVIC pre-emption priority for the driver-enable release interrupt.
    // The default sits below the soft-GPIO handlers (quadrature/step-dir
    // at priority 1) so those always win, but above the PendSV level
    // (priority 6) where the encoder query is issued.
    uint32_t de_irq_priority = 2;
  };

  Stm32G4DmaUart(const Options& options)
      : options_(options),
        stm32_serial_([&]() {
                        Stm32Serial::Options s_options;
                        s_options.tx = options.tx;
                        s_options.rx = options.rx;
                        s_options.baud_rate = options.baud_rate;
                        return s_options;
                      }()) {

    uart_ = stm32_serial_.uart();
    MJ_ASSERT(uart_ != nullptr);

    MJ_ASSERT(options.rx_dma);

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
    dmamux_rx_->CCR = GetUartRxRequest(uart_) & DMAMUX_CxCR_DMAREQ_ID;
    options_.rx_dma->CPAR = u32(&uart_->RDR);

    options_.tx_dma->CCR =
        DMA_MEMORY_TO_PERIPH |
        DMA_PINC_DISABLE |
        DMA_MINC_ENABLE |
        DMA_PDATAALIGN_BYTE |
        DMA_MDATAALIGN_BYTE |
        DMA_PRIORITY_HIGH;
    dmamux_tx_->CCR = GetUartTxRequest(uart_) & DMAMUX_CxCR_DMAREQ_ID;
    options_.tx_dma->CPAR = u32(&uart_->TDR);

    if (options_.de) {
      options_.de->write(0);

      const int index = GetUartIndex(uart_);
      de_entries_[index].uart = uart_;
      de_entries_[index].de = options_.de;

      const auto irqn = GetUartIrqN(uart_);
      NVIC_SetVector(irqn, reinterpret_cast<uint32_t>(&Stm32G4DmaUart::ISR_De));
      HAL_NVIC_SetPriority(irqn, options_.de_irq_priority, 0);
      NVIC_EnableIRQ(irqn);
    }
  }

  ~Stm32G4DmaUart() {
    finish_dma_read();
    finish_dma_write();

    if (options_.de) {
      const auto irqn = GetUartIrqN(uart_);
      NVIC_DisableIRQ(irqn);
      NVIC_ClearPendingIRQ(irqn);
      uart_->CR1 &= ~(USART_CR1_TCIE);
      options_.de->write(0);
      auto& entry = de_entries_[GetUartIndex(uart_)];
      entry.uart = nullptr;
      entry.de = nullptr;
    }
  }

  USART_TypeDef* uart() const {
    return uart_;
  }

  // Queue a single character to be written.  We rely on the FIFO to
  // prevent overflow.
  void write_char(uint8_t data) {
    if (options_.de) { StartDe(); }
    uart_->TDR = data;
    if (options_.de) { uart_->CR1 |= USART_CR1_TCIE; }
  }

  void start_dma_read(mjlib::base::string_span output) MOTEUS_CCM_ATTRIBUTE {
    options_.rx_dma->CNDTR = output.size();
    options_.rx_dma->CMAR = u32(&output[0]);

    options_.rx_dma->CCR |= DMA_CCR_EN;
    uart_->CR3 |= USART_CR3_DMAR;
  }

  bool is_dma_read_finished() MOTEUS_CCM_ATTRIBUTE {
    return options_.rx_dma->CNDTR == 0;
  }

  int read_bytes_remaining() MOTEUS_CCM_ATTRIBUTE {
    return options_.rx_dma->CNDTR;
  }

  // This can be used to abort a request early.
  void finish_dma_read() MOTEUS_CCM_ATTRIBUTE {
    uart_->CR3 &= ~(USART_CR3_DMAR);
    options_.rx_dma->CCR &= ~(DMA_CCR_EN);
  }

  void start_dma_write(std::string_view data) MOTEUS_CCM_ATTRIBUTE {
    if (options_.de) { StartDe(); }

    options_.tx_dma->CNDTR = data.size();
    options_.tx_dma->CMAR = u32(&data[0]);

    options_.tx_dma->CCR |= DMA_CCR_EN;
    uart_->CR3 |= USART_CR3_DMAT;

    if (options_.de) { uart_->CR1 |= USART_CR1_TCIE; }
  }

  bool is_dma_write_finished() MOTEUS_CCM_ATTRIBUTE {
    return options_.tx_dma->CNDTR == 0;
  }

  int write_bytes_remaining() MOTEUS_CCM_ATTRIBUTE {
    return options_.tx_dma->CNDTR;
  }

  // This can be used to abort a request early.
  void finish_dma_write() MOTEUS_CCM_ATTRIBUTE {
    uart_->CR3 &= ~(USART_CR3_DMAT);
    options_.tx_dma->CCR &= ~(DMA_CCR_EN);
  }

  static uint32_t GetUartRxRequest(USART_TypeDef* uart) {
    switch (u32(uart)) {
      case UART_1: return DMA_REQUEST_USART1_RX;
      case UART_2: return DMA_REQUEST_USART2_RX;
      case UART_3: return DMA_REQUEST_USART3_RX;
    }
    mbed_die();
  }

  static uint32_t GetUartTxRequest(USART_TypeDef* uart) {
    switch (u32(uart)) {
      case UART_1: return DMA_REQUEST_USART1_TX;
      case UART_2: return DMA_REQUEST_USART2_TX;
      case UART_3: return DMA_REQUEST_USART3_TX;
    }
    mbed_die();
  }

  static int GetUartIndex(USART_TypeDef* uart) {
    switch (u32(uart)) {
      case UART_1: return 0;
      case UART_2: return 1;
      case UART_3: return 2;
    }
    mbed_die();
  }

  static IRQn_Type GetUartIrqN(USART_TypeDef* uart) {
    switch (u32(uart)) {
      case UART_1: return USART1_IRQn;
      case UART_2: return USART2_IRQn;
      case UART_3: return USART3_IRQn;
    }
    mbed_die();
  }

  // Assert the RS485 driver enable and clear any stale transmission
  // complete flag so the release interrupt fires only for the byte(s)
  // we are about to send.
  void StartDe() MOTEUS_CCM_ATTRIBUTE {
    options_.de->write(1);
    uart_->ICR = USART_ICR_TCCF;
  }

  // Records which driver-enable pin to release for each managed USART.
  struct DeEntry {
    USART_TypeDef* uart = nullptr;
    DigitalOut* de = nullptr;

    DeEntry() {}
  };

  // Indexed by GetUartIndex().  There is at most one Stm32G4DmaUart per
  // USART, so a flat table indexed by peripheral is sufficient.  The
  // aggregate initializer zeroes both pointers.
  static inline DeEntry de_entries_[3] = {};

  // The transmission-complete interrupt handler, shared by all managed
  // USART vectors.  It releases the bus the instant the last stop bit
  // has cleared the shift register.
  static void ISR_De() MOTEUS_CCM_ATTRIBUTE {
    for (auto& entry : de_entries_) {
      if (entry.uart == nullptr) { continue; }
      if ((entry.uart->ISR & USART_ISR_TC) &&
          (entry.uart->CR1 & USART_CR1_TCIE)) {
        // Disarm; the next transmission re-clears TC and re-enables the
        // interrupt.  Then release the driver so the encoder can reply.
        entry.uart->CR1 &= ~(USART_CR1_TCIE);
        entry.de->write(0);
      }
    }
  }

  template <typename T>
  static uint32_t u32(T value) {
    return reinterpret_cast<uint32_t>(value);
  }

  const Options options_;
  Stm32Serial stm32_serial_;

  USART_TypeDef* uart_ = nullptr;
  DMAMUX_Channel_TypeDef* dmamux_rx_ = nullptr;
  DMAMUX_Channel_TypeDef* dmamux_tx_ = nullptr;
};

}
