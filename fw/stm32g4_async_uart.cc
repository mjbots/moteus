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

#include "fw/stm32g4_async_uart.h"

#include "mjlib/micro/atomic_event_queue.h"
#include "mjlib/micro/callback_table.h"

#include "fw/error.h"
#include "fw/stm32_serial.h"

namespace base = mjlib::base;
namespace micro = mjlib::micro;

namespace moteus {
namespace {
template <typename T>
uint32_t u32(T value) {
  return reinterpret_cast<uint32_t>(value);
}

uint32_t GetUartTxRequest(USART_TypeDef* uart) {
  switch (u32(uart)) {
    case UART_1: return DMA_REQUEST_USART1_TX;
    case UART_2: return DMA_REQUEST_USART2_TX;
    case UART_3: return DMA_REQUEST_USART3_TX;
    case UART_4: return DMA_REQUEST_UART4_TX;
#if defined (UART5)
    case UART_5: return DMA_REQUEST_UART5_TX;
#endif
  }
  mbed_die();
  return 0;
}

DMA_TypeDef* GetDmaForChannel(DMA_Channel_TypeDef* dma) {
  return (dma > DMA1_Channel8) ? DMA2 : DMA1;
}

uint32_t GetDmaSr(DMA_Channel_TypeDef* dma) {
  return GetDmaForChannel(dma)->ISR;
}

void ClearDmaFlag(DMA_Channel_TypeDef* dma, uint32_t flag) {
  GetDmaForChannel(dma)->IFCR = flag;
}

uint32_t GetUartRxRequest(USART_TypeDef* uart) {
  switch (u32(uart)) {
    case UART_1: return DMA_REQUEST_USART1_RX;
    case UART_2: return DMA_REQUEST_USART2_RX;
    case UART_3: return DMA_REQUEST_USART3_RX;
    case UART_4: return DMA_REQUEST_UART4_RX;
#if defined (UART5)
    case UART_5: return DMA_REQUEST_UART5_RX;
#endif
  }
  mbed_die();
  return 0;
}

IRQn_Type GetDmaIrq(DMA_Channel_TypeDef* dma) {
  if (dma == DMA1_Channel1) { return DMA1_Channel1_IRQn; }
  if (dma == DMA1_Channel2) { return DMA1_Channel2_IRQn; }
  if (dma == DMA1_Channel3) { return DMA1_Channel3_IRQn; }
  if (dma == DMA1_Channel4) { return DMA1_Channel4_IRQn; }
  if (dma == DMA1_Channel5) { return DMA1_Channel5_IRQn; }
  if (dma == DMA1_Channel6) { return DMA1_Channel6_IRQn; }
  if (dma == DMA1_Channel7) { return DMA1_Channel7_IRQn; }
  if (dma == DMA1_Channel8) { return DMA1_Channel8_IRQn; }

  if (dma == DMA2_Channel1) { return DMA2_Channel1_IRQn; }
  if (dma == DMA2_Channel2) { return DMA2_Channel2_IRQn; }
  if (dma == DMA2_Channel3) { return DMA2_Channel3_IRQn; }
  if (dma == DMA2_Channel4) { return DMA2_Channel4_IRQn; }
  if (dma == DMA2_Channel5) { return DMA2_Channel5_IRQn; }
  if (dma == DMA2_Channel6) { return DMA2_Channel6_IRQn; }
  if (dma == DMA2_Channel7) { return DMA2_Channel7_IRQn; }
  if (dma == DMA2_Channel8) { return DMA2_Channel8_IRQn; }

  mbed_die();
}

IRQn_Type GetUsartIrq(USART_TypeDef* uart) {
  switch(u32(uart)) {
#if defined (USART1_BASE)
    case UART_1: return USART1_IRQn;
#endif
#if defined (USART2_BASE)
    case UART_2: return USART2_IRQn;
#endif
#if defined (USART3_BASE)
    case UART_3: return USART3_IRQn;
#endif
#if defined (UART4_BASE)
    case UART_4: return UART4_IRQn;
#endif
#if defined (USART4_BASE)
    case UART_4: return USART4_IRQn;
#endif
#if defined (UART5_BASE)
    case UART_5: return UART5_IRQn;
#endif
#if defined (USART5_BASE)
    case UART_5: return USART5_IRQn;
#endif
#if defined (USART6_BASE)
    case UART_6: return USART6_IRQn;
#endif
#if defined (UART7_BASE)
    case UART_7: return UART7_IRQn;
#endif
#if defined (USART7_BASE)
    case UART_7: return USART7_IRQn;
#endif
#if defined (UART8_BASE)
    case UART_8: return UART8_IRQn;
#endif
#if defined (USART8_BASE)
    case UART_8: return USART8_IRQn;
#endif
#if defined (UART9_BASE)
    case UART_9: return UART9_IRQn;
#endif
#if defined (UART10_BASE)
    case UART_10: return UART10_IRQn;
#endif
}
  mbed_die();
  return {};
}

}

class Stm32G4AsyncUart::Impl {
 public:
  Impl(micro::Pool* pool, const Options& options)
      : options_(options),
        stm32_serial_([&]() {
           Stm32Serial::Options s_options;
           s_options.tx = options.tx;
           s_options.rx = options.rx;
           s_options.baud_rate = options.baud_rate;
           return s_options;
        }()) {
    rx_buffer_ = reinterpret_cast<volatile uint16_t*>(
        pool->Allocate(options.rx_buffer_size * sizeof(*rx_buffer_),
                       sizeof(*rx_buffer_)));

    // Our receive buffer requires that all unprocessed words be
    // 0xffff.
    for (size_t i = 0; i < options_.rx_buffer_size; i++) {
      rx_buffer_[i] = 0xffff;
    }

    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    auto* const uart = stm32_serial_.uart();
    MJ_ASSERT(uart != nullptr);

    if (options.rx != NC) {
      dma_rx_ = options.rx_dma;

      const auto channel_index = [&]() -> uint32_t {
        if (dma_rx_ < DMA2_Channel1) {
          return ((u32(dma_rx_) - u32(DMA1_Channel1)) /
                  (u32(DMA1_Channel2) - u32(DMA1_Channel1))) << 2;
        }
        return ((u32(dma_rx_) - u32(DMA2_Channel1)) /
                (u32(DMA2_Channel2) - u32(DMA2_Channel1))) << 2;
      }();

      dmamux_rx_ = [&]() {
        const auto base = (dma_rx_ < DMA2_Channel1) ? DMAMUX1_Channel0 :
       #if defined (STM32G471xx) || defined (STM32G473xx) || defined (STM32G474xx) || defined (STM32G483xx) || defined (STM32G484xx)
        DMAMUX1_Channel8;
#elif defined (STM32G431xx) || defined (STM32G441xx) || defined (STM32GBK1CB)
        DMAMUX1_Channel6;
#else
        DMAMUX1_Channel7;
#endif /* STM32G4x1xx) */
        return reinterpret_cast<DMAMUX_Channel_TypeDef*>(
            u32(base) + (channel_index >> 2U) *
            (u32(DMAMUX1_Channel1) - u32(DMAMUX1_Channel0)));
      }();

      dma_rx_->CCR =
          DMA_PERIPH_TO_MEMORY |
          DMA_PINC_DISABLE |
          DMA_MINC_ENABLE |
          DMA_PDATAALIGN_HALFWORD |
          DMA_MDATAALIGN_HALFWORD |
          DMA_CIRCULAR |
          DMA_PRIORITY_HIGH |
          DMA_CCR_TEIE |
          DMA_CCR_HTIE |
          DMA_CCR_TCIE;
      dma_rx_->CPAR = u32(&uart->RDR);
      dma_rx_->CMAR = u32(&rx_buffer_[0]);
      dma_rx_->CNDTR = options.rx_buffer_size;

      dma_rx_->CCR |= DMA_CCR_EN;
      dmamux_rx_->CCR = GetUartRxRequest(uart) & DMAMUX_CxCR_DMAREQ_ID;
      uart->CR3 |= USART_CR3_DMAR;

      dma_rx_flags_.te = DMA_ISR_TEIF1 << channel_index;
    }

    hdma_usart_tx_.Instance = options.tx_dma;
    hdma_usart_tx_.Init.Request = GetUartTxRequest(uart);
    hdma_usart_tx_.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart_tx_.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart_tx_.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart_tx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart_tx_.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart_tx_.Init.Mode = DMA_NORMAL;
    hdma_usart_tx_.Init.Priority = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hdma_usart_tx_) != HAL_OK)
    {
      mbed_die();
    }

    __HAL_LINKDMA(&uart_, hdmatx,hdma_usart_tx_);

    tx_callback_ = micro::CallbackTable::MakeFunction([this]() {
        HAL_DMA_IRQHandler(&hdma_usart_tx_);
      });

    uart_callback_ = micro::CallbackTable::MakeFunction([this]() {
        HAL_UART_IRQHandler(&uart_);
      });

    const auto tx_irq = GetDmaIrq(options.tx_dma);
    const auto usart_irq = GetUsartIrq(uart);

    NVIC_SetVector(
        tx_irq,
        u32(tx_callback_.raw_function));
    NVIC_SetVector(
        usart_irq,
        u32(uart_callback_.raw_function));

    HAL_NVIC_SetPriority(tx_irq, 2, 0);
    HAL_NVIC_EnableIRQ(tx_irq);

    HAL_NVIC_SetPriority(usart_irq, 2, 0);
    HAL_NVIC_EnableIRQ(usart_irq);
  }

  void AsyncReadSome(const base::string_span& data,
                     const micro::SizeCallback& callback) {
    MJ_ASSERT(!current_read_callback_);
    current_read_data_ = data;
    current_read_callback_ = callback;
  }

  void AsyncWriteSome(const string_view& data,
                      const micro::SizeCallback& callback) {
    MJ_ASSERT(!current_write_callback_);
    current_write_callback_ = callback;
    current_write_bytes_ = data.size();

    if (HAL_UART_Transmit_DMA(
            &uart_,
            const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())),
            data.size()) != HAL_OK) {
      mbed_die();
    }
  }

  void ProcessRead() {
    if (rx_buffer_[rx_buffer_pos_] == 0xffff && !pending_rx_error_) {
      return;
    }

    const uint16_t last_pos = (
        rx_buffer_pos_ + (options_.rx_buffer_size - 1)) %
        options_.rx_buffer_size;
    if (rx_buffer_[last_pos] != 0xffff) {
      pending_rx_error_ = errc::kUartBufferOverrunError;
    }

    ssize_t bytes_read = 0;
    for (;
         (bytes_read < current_read_data_.size() &&
          rx_buffer_[rx_buffer_pos_] != 0xffffu);
         (bytes_read++,
          (rx_buffer_pos_ = (rx_buffer_pos_ + 1) % options_.rx_buffer_size))) {
      current_read_data_.data()[bytes_read] =
          rx_buffer_[rx_buffer_pos_] & 0xff;
      rx_buffer_[rx_buffer_pos_] = 0xffff;
    }

    {
      auto copy = current_read_callback_;
      auto rx_error = pending_rx_error_;

      current_read_callback_ = {};
      pending_rx_error_ = {};

      copy(rx_error, bytes_read);
    }
  }

  void Poll() {
    // See if we had an error reading of some sort.
    const auto sr = GetDmaSr(dma_rx_);
    if (sr & dma_rx_flags_.te) {
      ClearDmaFlag(dma_rx_, dma_rx_flags_.te);

      const auto uart_isr = uart_.Instance->ISR;
      pending_rx_error_ = [&]() {
        if (uart_isr & USART_ISR_ORE) {
          return errc::kUartOverrunError;
        } else if (uart_isr & USART_ISR_FE) {
          return errc::kUartFramingError;
        } else if (uart_isr & USART_ISR_NE) {
          return errc::kUartNoiseError;
        } else if (uart_isr & USART_ISR_PE) {
          return errc::kUartParityError;
        }
        return errc::kDmaStreamTransferError;
      }();
    }

    // Handle writes if they are done.
    if (current_write_callback_) {
      if (uart_.gState == HAL_UART_STATE_READY) {
        decltype(current_write_callback_) copy;
        using std::swap;
        swap(copy, current_write_callback_);
        copy(micro::error_code(), current_write_bytes_);
      }
    }

    // Handle any read data.
    if (current_read_callback_) {
      ProcessRead();
    }
  }

  const Options options_;
  Stm32Serial stm32_serial_;
  UART_HandleTypeDef& uart_ = *stm32_serial_.huart();
  DMA_Channel_TypeDef* dma_rx_ = nullptr;
  DMAMUX_Channel_TypeDef* dmamux_rx_ = nullptr;
  DMA_HandleTypeDef hdma_usart_tx_;

  micro::SizeCallback current_write_callback_;
  ssize_t current_write_bytes_ = 0;

  micro::SizeCallback current_read_callback_;
  base::string_span current_read_data_ = {};
  micro::error_code pending_rx_error_;

  micro::CallbackTable::Callback tx_callback_;
  micro::CallbackTable::Callback uart_callback_;

  struct DmaFlags {
    uint32_t te = 0;
  };
  DmaFlags dma_rx_flags_;

  // This buffer serves as a place to store things in between calls to
  // AsyncReadSome so that there is minimal chance of data loss even
  // at high data rates.
  volatile uint16_t* rx_buffer_ = nullptr;
  uint16_t rx_buffer_pos_ = 0;
};

Stm32G4AsyncUart::Stm32G4AsyncUart(micro::Pool* pool,
                                   MillisecondTimer* timer,
                                   const Options& options)
    : impl_(pool, pool, options) {}

Stm32G4AsyncUart::~Stm32G4AsyncUart() {}

void Stm32G4AsyncUart::AsyncReadSome(const base::string_span& data,
                                     const micro::SizeCallback& callback) {
  impl_->AsyncReadSome(data, callback);
}

void Stm32G4AsyncUart::AsyncWriteSome(const string_view& data,
                                      const micro::SizeCallback& callback) {
  impl_->AsyncWriteSome(data, callback);
}

void Stm32G4AsyncUart::Poll() {
  impl_->Poll();
}

}
