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

#include "fw/aux_common.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32g4_dma_uart.h"

namespace moteus {

class Aksim2 {
 public:
  Aksim2(const aux::UartEncoder::Config& config,
         Stm32G4DmaUart* uart,
         MillisecondTimer* timer)
      : config_(config),
        uart_(uart),
        timer_(timer) {}

  void ISR_Update(aux::UartEncoder::Status* status) MOTEUS_CCM_ATTRIBUTE {
    // Now check to see if we can issue a new one.
    const uint32_t now_us = timer_->read_us();
    const uint32_t delta_us = (now_us - last_query_start_us_);

    // Do we have an outstanding query?
    if (query_outstanding_) {
      if (delta_us > static_cast<uint32_t>(2 * config_.poll_rate_us)) {
        // We timed out.
        uart_->finish_dma_read();
        query_outstanding_ = false;
      } else {
        // See if we can finish it.
        ProcessQuery(status);
      }
    }

    // We did not complete the query, so just return.
    if (query_outstanding_) { return; }

    if (delta_us < static_cast<uint32_t>(config_.poll_rate_us)) {
      // Nope, we're not ready to issue another.
      return;
    }

    last_query_start_us_ = now_us;
    query_outstanding_ = true;
    uart_->write_char('d');
    StartRead();
  }

  void ProcessQuery(aux::UartEncoder::Status* status) MOTEUS_CCM_ATTRIBUTE {
    if (uart_->read_bytes_remaining() > kResyncBytes) { return; }

    if (uart_->read_bytes_remaining() == 0) {
      // We used up our resync bytes without success.  Just try again.
      uart_->finish_dma_read();
      query_outstanding_ = false;
      return;
    }

    if (buffer_[0] != 'd') {
      // Not what we are expecting.  Just fill up our buffer until
      // the timeout.
      return;
    }

    uart_->finish_dma_read();
    query_outstanding_ = false;

    status->value =
        ((buffer_[1] << 16) |
         (buffer_[2] << 8) |
         (buffer_[3] << 0)) >> 2;
    status->aksim2_err = buffer_[3] & 0x01;
    status->aksim2_warn = buffer_[3] & 0x02;
    status->aksim2_status =
        (buffer_[4] << 8) |
        (buffer_[5] << 0);

    status->nonce++;
    status->active = true;
  }

 private:
  void StartRead() MOTEUS_CCM_ATTRIBUTE {
    uart_->start_dma_read(
        mjlib::base::string_span(reinterpret_cast<char*>(&buffer_[0]),
                                 sizeof(buffer_)));
  }

  const aux::UartEncoder::Config config_;
  Stm32G4DmaUart* const uart_;
  MillisecondTimer* const timer_;

  bool query_outstanding_ = false;

  uint32_t last_query_start_us_ = 0;

  static constexpr int kResyncBytes = 3;
  static constexpr int kMaxCount = 50;

  // The "detailed" reply has a header byte, 3 bytes of position, and
  // 2 bytes of status.
  //
  // We have 3 extra bytes so that we could eventually re-synchronize.
  uint8_t buffer_[6 + kResyncBytes] = {};
};

}
