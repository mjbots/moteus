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

#include "moteus/stm32f446_async_uart.h"

#include <tuple>

#include "mbed.h"
#include "serial_api_hal.h"

#include "mjlib/base/assert.h"

#include "moteus/atomic_event_queue.h"
#include "moteus/error.h"
#include "moteus/irq_callback_table.h"
#include "moteus/stm32_serial.h"

namespace base = mjlib::base;
namespace micro = mjlib::micro;

namespace moteus {

namespace {
IRQn_Type FindUartRxIrq(USART_TypeDef* uart) {
  switch(reinterpret_cast<uint32_t>(uart)) {
    case UART_1: return USART1_IRQn;
    case UART_2: return USART2_IRQn;
    case UART_3: return USART3_IRQn;
    case UART_4: return UART4_IRQn;
    case UART_5: return UART5_IRQn;
    case UART_6: return USART6_IRQn;
  }
  MJ_ASSERT(false);
  return {};
}
}

class Stm32F446AsyncUart::Impl {
 public:
  Impl(micro::Pool* pool, MillisecondTimer* timer, const Options& options)
      : timer_(timer),
        options_(options),
        stm32_serial_([&options]() {
            Stm32Serial::Options s_options;
            s_options.rx = options.rx;
            s_options.tx = options.tx;
            s_options.baud_rate = options.baud_rate;
            return s_options;
          }()),
        dir_(options.dir, 0) {
    rx_buffer_ = reinterpret_cast<volatile uint16_t*>(
        pool->Allocate(options.rx_buffer_size * sizeof(*rx_buffer_),
                       sizeof(*rx_buffer_)));

    // Our receive buffer requires that all unprocessed words be
    // 0xffff.
    for (size_t i = 0; i < options_.rx_buffer_size; i++) {
      rx_buffer_[i] = 0xffff;
    }

    // Just in case no one else has done it yet.
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    uart_ = stm32_serial_.uart();

    MJ_ASSERT(uart_ != nullptr);
    uart_rx_irq_ = FindUartRxIrq(uart_);

    // TODO(josh.pieper): For now, we will hard-code which stream to
    // use when there are multiple options.  Perhaps later, the
    // Options we get passed in could provide a requirement if
    // necessary.
    auto dma_pair = MakeDma(static_cast<UARTName>(reinterpret_cast<int>(uart_)));
    tx_dma_ = dma_pair.tx;
    rx_dma_ = dma_pair.rx;

    // Roughly follow the procedure laid out in AN4031: Using the
    // STM32F2, STM32F4, and STM32F7 Series DMA Controller, section
    // 1.2

    // TODO(jpieper): This will only function if the DMA controller is
    // in a pristine state.  You could imagine asserting that is the
    // case, or even better, getting it into an appropriate state.  We
    // won't worry about it for now.

    // TODO(jpieper): Configure the FIFO to reduce the possibility of
    // bus contention causing data loss.

    if (options.tx != NC) {
      tx_dma_.stream -> PAR = reinterpret_cast<uint32_t>(&(uart_->DR));
      tx_dma_.stream -> CR =
          tx_dma_.channel |
          DMA_SxCR_MINC |
          DMA_MEMORY_TO_PERIPH |
          DMA_SxCR_TCIE | DMA_SxCR_TEIE;

      tx_callback_ = IrqCallbackTable::MakeFunction([this]() {
          this->HandleTransmit();
        });
      NVIC_SetVector(tx_dma_.irq, reinterpret_cast<uint32_t>(tx_callback_.irq_function));
      HAL_NVIC_SetPriority(tx_dma_.irq, 5, 0);
      NVIC_EnableIRQ(tx_dma_.irq);
    }

    if (options.rx != NC) {
      rx_dma_.stream -> PAR = reinterpret_cast<uint32_t>(&(uart_->DR));
      rx_dma_.stream -> CR =
          rx_dma_.channel |
          DMA_SxCR_MINC |
          DMA_PERIPH_TO_MEMORY |
          (0x1 << DMA_SxCR_MSIZE_Pos) |  // 16-bit memory
          (0x1 << DMA_SxCR_PSIZE_Pos) |  // 16-bit peripheral
          DMA_SxCR_CIRC |
          DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE;

      rx_callback_ = IrqCallbackTable::MakeFunction([this]() {
          this->HandleReceive();
        });
      NVIC_SetVector(rx_dma_.irq, reinterpret_cast<uint32_t>(rx_callback_.irq_function));
      HAL_NVIC_SetPriority(rx_dma_.irq, 5, 0);
      NVIC_EnableIRQ(rx_dma_.irq);

      // Notify when there are idle times on the bus.
      uart_->CR1 |= USART_CR1_IDLEIE;

      uart_callback_ = IrqCallbackTable::MakeFunction([this]() {
          this->HandleUart();
        });
      NVIC_SetVector(uart_rx_irq_, reinterpret_cast<uint32_t>(uart_callback_.irq_function));
      HAL_NVIC_SetPriority(uart_rx_irq_, 5, 0);
      NVIC_EnableIRQ(uart_rx_irq_);

      // We run our receiver continuously in circular buffer mode.
      rx_dma_.stream->M0AR = reinterpret_cast<uint32_t>(rx_buffer_);

      *rx_dma_.status_clear |= rx_dma_.all_status();
      rx_dma_.stream->NDTR = options_.rx_buffer_size;
      rx_dma_.stream->CR |= DMA_SxCR_EN;

      uart_->CR3 |= USART_CR3_DMAR;
    }
  }

  void AsyncReadSome(const base::string_span& data,
                     const micro::SizeCallback& callback) {
    MJ_ASSERT(!current_read_callback_.valid());

    // All this does is set our buffer and callback.  We're always
    // reading, and that process will just look to see if we have a
    // buffer outstanding.
    current_read_data_ = data;
    current_read_callback_ = callback;

    // See if we already have data for this receiver.
    EventProcessData();
  }

  void AsyncWriteSome(const string_view& data,
                      const micro::SizeCallback& callback) {
    MJ_ASSERT(!current_write_callback_.valid());

    if (dir_.is_connected()) {
      dir_.write(1);
      if (options_.enable_delay_us) {
        timer_->wait_us(options_.enable_delay_us);
      }
    }

    current_write_callback_ = callback;
    tx_size_ = data.size();

    // AN4031, 4.2: Clear all status registers.

    *tx_dma_.status_clear |= tx_dma_.all_status();

    tx_dma_.stream->NDTR = data.size();
    tx_dma_.stream->M0AR = reinterpret_cast<uint32_t>(data.data());
    tx_dma_.stream->CR |= DMA_SxCR_EN;

    uart_ -> CR3 |= USART_CR3_DMAT;
  }

  // INVOKED FROM INTERRUPT CONTEXT
  void HandleTransmit() {
    const ssize_t amount_sent = tx_size_ - tx_dma_.stream->NDTR;
    micro::error_code error_code;

    // The enable bit should be 0 at this point.
    MJ_ASSERT((tx_dma_.stream->CR & DMA_SxCR_EN) == 0);

    // Tell the UART to stop requesting DMA.
    uart_->CR3 &= ~(USART_CR3_DMAT);

    if (*tx_dma_.status_register & tx_dma_.status_teif) {
      // We've got an error, report it.
      *tx_dma_.status_clear |= tx_dma_.status_teif;
      error_code = errc::kDmaStreamTransferError;
    } else if (*tx_dma_.status_register & tx_dma_.status_feif) {
      *tx_dma_.status_clear |= tx_dma_.status_feif;
      error_code = errc::kDmaStreamFifoError;
    } else  if (*tx_dma_.status_register & tx_dma_.status_tcif) {
      // Transmit is complete.
      *tx_dma_.status_clear |= tx_dma_.status_tcif;
      error_code = {};
    } else {
      MJ_ASSERT(false);
    }

    event_queue_.Queue([this, error_code, amount_sent]() {
        if (dir_.is_connected()) {
          if (options_.disable_delay_us) {
            timer_->wait_us(options_.disable_delay_us);
          }
          dir_.write(0);
        }

        this->EventHandleTransmit(error_code, amount_sent);
      });

    // TODO(jpieper): Verify that USART_CR3_DMAT gets cleared here on
    // its own even if we send back to back quickly.
  }

  void EventHandleTransmit(micro::error_code error_code, ssize_t amount_sent) {
    auto copy = current_write_callback_;
    current_write_callback_ = {};

    // The copy could be invalid if we got an unexpected interrupt,
    // but there is no real reason to abort in that case, so just only
    // call it if is valid.
    if (copy.valid()) {
      copy(error_code, amount_sent);
    }
  }

  // INVOKED FROM INTERRUPT CONTEXT
  void HandleReceive() {
    // All we do here is process any error flags and then request to
    // flush an outstanding buffer if we have one.

    if (*rx_dma_.status_register & rx_dma_.status_teif) {
      *rx_dma_.status_clear |= rx_dma_.status_teif;
      const auto uart_sr = uart_->SR;

      // TI's reference manual in RM0390 says that to clear these
      // flags you have to read the status register followed by
      // reading the data register.  Can you read the data register
      // while a DMA transaction is ongoing?  We've already had a
      // transfer error of some sort by the time we get here, so
      // hopefully it doesn't break too much.
      volatile uint32_t tmp = uart_->DR;
      (void)tmp;

      if (uart_sr & USART_SR_ORE) {
        pending_rx_error_ = errc::kUartOverrunError;
      } else if (uart_sr & USART_SR_FE) {
        pending_rx_error_ = errc::kUartFramingError;
      } else if (uart_sr & USART_SR_NE) {
        pending_rx_error_ = errc::kUartNoiseError;
      } else {
        pending_rx_error_ = errc::kDmaStreamTransferError;
      }
    } else if (*rx_dma_.status_register & rx_dma_.status_feif) {
      *rx_dma_.status_clear |= rx_dma_.status_feif;
      pending_rx_error_ = errc::kDmaStreamFifoError;
    } else if (*rx_dma_.status_register & rx_dma_.status_tcif) {
      *rx_dma_.status_clear |= rx_dma_.status_tcif;
    } else if (*rx_dma_.status_register & rx_dma_.status_htif) {
      // We are halfway through a transfer, which in practice just
      // means we're at a random point in whatever thing we're
      // reading.  Use this as an opportune time to report some
      // fractional results so as to minimize latency for bulk reads.
      *rx_dma_.status_clear |= rx_dma_.status_htif;
    } else {
      MJ_ASSERT(false);
    }

    EventQueue::Options options;
    options.ignore_full = true;
    event_queue_.Queue([this]() {
        this->EventProcessData();
      },
      options);
  }

  // INVOKED FROM INTERRUPT CONTEXT
  void HandleUart() {
    if (uart_->SR && USART_FLAG_IDLE) {
      // Clear the IDLE flag by reading status register, then data register.
      volatile uint32_t tmp;
      tmp = uart_->SR;
      tmp = uart_->DR;
      (void)tmp;

      EventQueue::Options options;
      // EventProcessData works just fine if one or more things are
      // dropped.
      options.ignore_full = true;
      event_queue_.Queue([this]() {
          this->EventProcessData();
        },
        options);
    }
  }

  void EventProcessData() {
    if (current_read_data_.data() == nullptr) {
      // There is no outstanding callback.
      return;
    }

    if (rx_buffer_[rx_buffer_pos_] == 0xffff && !pending_rx_error_) {
      // There are no data or errors pending.
      return;
    }

    const uint16_t last_pos = (
        rx_buffer_pos_ + (options_.rx_buffer_size - 1)) % options_.rx_buffer_size;
    if (rx_buffer_[last_pos] != 0xffff) {
      pending_rx_error_ = errc::kUartBufferOverrunError;
      // We have lost synchronization with wherever the DMA controller
      // is spewing.
      if (rx_dma_.stream->CR & DMA_SxCR_EN) {
        // Disable and return early.  The TCIF interrupt will fire,
        // which will trigger us again.
        rx_dma_.stream->CR &= ~(DMA_SxCR_EN);
        return;
      } else {
        // Just fall through, we'll re-enable ourselves at the bottom
        // and start over.
      }
    }

    ssize_t bytes_read = 0;
    for (;
         bytes_read < current_read_data_.size() && rx_buffer_[rx_buffer_pos_] != 0xffffu;
         bytes_read++, (rx_buffer_pos_ = (rx_buffer_pos_ + 1) % options_.rx_buffer_size)) {
      current_read_data_.data()[bytes_read] = rx_buffer_[rx_buffer_pos_] & 0xff;
      rx_buffer_[rx_buffer_pos_] = 0xffff;
    }

    MJ_ASSERT(current_read_callback_.valid());
    {
      // Right now, we mark that we no longer want to get any more
      // data on this callback.
      //
      // However, we enqueue the callback itself on the event queue to
      // avoid recursion.

      current_read_data_ = {};
      event_queue_.Queue([this, bytes_read]() {
          auto copy = current_read_callback_;
          auto rx_error = pending_rx_error_;

          current_read_callback_ = {};
          pending_rx_error_ = {};

          copy(rx_error, bytes_read);
       });
    }

    // If our DMA stream was disabled for some reason, start over
    // again.
    if ((rx_dma_.stream->CR & DMA_SxCR_EN) == 0) {
      for (size_t i = 0; i < options_.rx_buffer_size; i++) {
        rx_buffer_[i] = 0xffff;
      }
      rx_buffer_pos_ = 0;

      rx_dma_.stream->CR |= DMA_SxCR_EN;
      uart_->CR3 |= USART_CR3_DMAR;
    }
  }

  MillisecondTimer* const timer_;
  const Options options_;
  Stm32Serial stm32_serial_;
  DigitalOut dir_;
  USART_TypeDef* uart_ = nullptr;
  IRQn_Type uart_rx_irq_ = {};

  Dma tx_dma_;
  Dma rx_dma_;

  IrqCallbackTable::Callback tx_callback_;
  IrqCallbackTable::Callback rx_callback_;
  IrqCallbackTable::Callback uart_callback_;

  micro::SizeCallback current_read_callback_;
  base::string_span current_read_data_;
  micro::error_code pending_rx_error_;

  using EventQueue = AtomicEventQueue<24>;
  EventQueue event_queue_;

  micro::SizeCallback current_write_callback_;
  ssize_t tx_size_ = 0;

  // This buffer serves as a place to store things in between calls to
  // AsyncReadSome so that there is minimal chance of data loss even
  // at high data rates.
  volatile uint16_t* rx_buffer_ = nullptr;
  uint16_t rx_buffer_pos_ = 0;
};

Stm32F446AsyncUart::Stm32F446AsyncUart(micro::Pool* pool,
                                       MillisecondTimer* timer,
                                       const Options& options)
    : impl_(pool, pool, timer, options) {}
Stm32F446AsyncUart::~Stm32F446AsyncUart() {}

void Stm32F446AsyncUart::AsyncReadSome(const base::string_span& data,
                                       const micro::SizeCallback& callback) {
  impl_->AsyncReadSome(data, callback);
}

void Stm32F446AsyncUart::AsyncWriteSome(const string_view& data,
                                        const micro::SizeCallback& callback) {
  impl_->AsyncWriteSome(data, callback);
}

void Stm32F446AsyncUart::Poll() {
  impl_->event_queue_.Poll();
}

#define MAKE_UART(DmaNumber, StreamNumber, ChannelNumber, StatusRegister) \
  Stm32F446AsyncUart::Dma {                                             \
    DmaNumber ## _Stream ## StreamNumber,                               \
        (ChannelNumber) << DMA_SxCR_CHSEL_Pos,                          \
        & ( DmaNumber -> StatusRegister ## FCR ),                       \
        & ( DmaNumber -> StatusRegister ## SR ),                        \
        DMA_ ## StatusRegister ## SR_TCIF ## StreamNumber,                \
        DMA_ ## StatusRegister ## SR_HTIF ## StreamNumber,                \
        DMA_ ## StatusRegister ## SR_TEIF ## StreamNumber,                \
        DMA_ ## StatusRegister ## SR_DMEIF ## StreamNumber,               \
        DMA_ ## StatusRegister ## SR_FEIF ## StreamNumber,                \
        DmaNumber ## _Stream ## StreamNumber ## _IRQn,                  \
        }

Stm32F446AsyncUart::DmaPair Stm32F446AsyncUart::MakeDma(UARTName uart) {
  switch (uart) {
    case UART_1:
      return { MAKE_UART(DMA2, 7, 4, HI), MAKE_UART(DMA2, 2, 4, LI), };
    case UART_2:
      return { MAKE_UART(DMA1, 6, 4, HI), MAKE_UART(DMA1, 5, 4, HI), };
    case UART_3:
      return { MAKE_UART(DMA1, 3, 4, LI), MAKE_UART(DMA1, 1, 4, LI), };
    case UART_4:
      return { MAKE_UART(DMA1, 4, 4, HI), MAKE_UART(DMA1, 2, 4, LI), };
    case UART_5:
      return { MAKE_UART(DMA1, 7, 4, HI), MAKE_UART(DMA1, 0, 4, LI), };
    case UART_6:
      return { MAKE_UART(DMA2, 6, 5, HI), MAKE_UART(DMA2, 1, 5, LI), };
  }
  MJ_ASSERT(false);
  return {};
}

#undef MAKE_UART

}
