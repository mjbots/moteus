// Copyright 2018 Josh Pieper, jjp@pobox.com.
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

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/pool_ptr.h"

namespace moteus {

/// Presents a single USART on the STM32F446 as an AsyncStream.
class Stm32F446AsyncUart : public mjlib::micro::AsyncStream {
 public:
  struct Options {
    PinName tx = NC;
    PinName rx = NC;

    // If non-NC, will be set to 1 while transmitting and left at 0
    // otherwise.  Useful for half-duplex RS-485 connections.
    PinName dir = NC;

    // If 'dir' is set, and we are enabling, wait this long after
    // enabling before beginning transmission.
    uint8_t enable_delay_us = 0;
    uint8_t disable_delay_us = 0;

    int baud_rate = 115200;
  };

  Stm32F446AsyncUart(mjlib::micro::Pool* pool,
                     const Options&);
  ~Stm32F446AsyncUart() override;

  void AsyncReadSome(const mjlib::base::string_span&,
                     const mjlib::micro::SizeCallback&) override;
  void AsyncWriteSome(const std::string_view&,
                      const mjlib::micro::SizeCallback&) override;

  // Call frequently.
  void Poll();

  // The following helper functions are exposed for modules which want
  // to operate on serial ports in DMA mode, but don't necessarily
  // want to use interrupts or the callback abstraction.
  struct Dma {
    DMA_Stream_TypeDef* stream;
    uint32_t channel;
    volatile uint32_t* status_clear;
    volatile uint32_t* status_register;
    uint32_t status_tcif;
    uint32_t status_htif;
    uint32_t status_teif;
    uint32_t status_dmeif;
    uint32_t status_feif;
    IRQn_Type irq;

    uint32_t all_status() const {
      return status_tcif |
        status_htif |
        status_teif |
        status_dmeif |
        status_feif;
    }
  };

  struct DmaPair {
    Dma tx;
    Dma rx;
  };

  static DmaPair MakeDma(UARTName uart);

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
