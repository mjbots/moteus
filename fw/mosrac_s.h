// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include "fw/absolute_encoder_validator.h"
#include "fw/aux_common.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32g4_dma_uart.h"

namespace moteus {

// Driver for the Mosrac S-series absolute magnetic ring encoder over
// RS422 UART.  Polls with command 0x31 and receives a 6-byte response:
// M1 M0 A2 A1 A0 CRC (no command-byte echo).
class MosracS {
 public:
  MosracS(const aux::UartEncoder::Config& config,
          Stm32G4DmaUart* uart,
          MillisecondTimer* timer)
      : config_(config),
        uart_(uart),
        timer_(timer) {}

  void ISR_Update(aux::UartEncoder::Status* status) MOTEUS_CCM_ATTRIBUTE {
    const uint32_t now_us = timer_->read_us();
    const uint32_t delta_us = (now_us - last_query_start_us_);

    if (query_outstanding_) {
      if (delta_us > (uint32_t)(2 * config_.poll_rate_us)) {
        // We timed out.
        uart_->finish_dma_read();
        query_outstanding_ = false;
        status->active = validator_.Update(
            status->active, false, 0, timer_->ms_since_boot());
      } else {
        ProcessQuery(status);
      }
    }

    if (query_outstanding_) { return; }

    if (delta_us < (uint32_t)(config_.poll_rate_us)) {
      return;
    }

    last_query_start_us_ = now_us;
    query_outstanding_ = true;
    uart_->write_char((char)(0x31));
    StartRead();
  }

 private:
  void ProcessQuery(aux::UartEncoder::Status* status) MOTEUS_CCM_ATTRIBUTE {
    if (uart_->read_bytes_remaining() > kResyncBytes) { return; }

    if (uart_->read_bytes_remaining() == 0) {
      uart_->finish_dma_read();
      query_outstanding_ = false;
      return;
    }

    uart_->finish_dma_read();
    query_outstanding_ = false;

    if (!ValidateCrc()) {
      status->checksum_errors++;
      return;
    }

    const uint32_t angle =
        (uint32_t)(buffer_[2]) << 16 |
        (uint32_t)(buffer_[3]) << 8 |
        (uint32_t)(buffer_[4]);

    status->value = angle;
    status->nonce++;
    status->active = validator_.Update(
        status->active, true, angle, timer_->ms_since_boot());
  }

  bool ValidateCrc() const MOTEUS_CCM_ATTRIBUTE {
    uint8_t crc = 0x00;
    for (int i = 0; i < 5; i++) {
      crc ^= buffer_[i];
      for (int j = 0; j < 8; j++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x97)
                           : (uint8_t)(crc << 1);
      }
    }
    return crc == buffer_[5];
  }

  void StartRead() MOTEUS_CCM_ATTRIBUTE {
    uart_->start_dma_read(
        mjlib::base::string_span((char*)(&buffer_[0]),
                                 sizeof(buffer_)));
  }

  const aux::UartEncoder::Config config_;
  Stm32G4DmaUart* const uart_;
  MillisecondTimer* const timer_;

  bool query_outstanding_ = false;
  uint32_t last_query_start_us_ = 0;
  AbsoluteEncoderValidator validator_;

  static constexpr int kResyncBytes = 3;

  // 6 response bytes (M1 M0 A2 A1 A0 CRC) + kResyncBytes extra slots.
  uint8_t buffer_[6 + kResyncBytes] = {};
};

}

