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

#include "mjlib/multiplex/micro_datagram_server.h"

#include "fw/fdcan.h"

namespace moteus {

class FDCanMicroServer : public mjlib::multiplex::MicroDatagramServer {
 public:
  // Fields in Header::flags
  static constexpr uint32_t kBrsFlag = 0x01;
  static constexpr uint32_t kFdcanFlag = 0x02;

  FDCanMicroServer(FDCan* can) : fdcan_(can) {}

  void SetPrefix(uint32_t can_prefix) {
    can_prefix_ = can_prefix;
  }

  void AsyncRead(Header* header,
                 const mjlib::base::string_span& data,
                 const mjlib::micro::SizeCallback& callback) override {
    MJ_ASSERT(!current_read_callback_);
    current_read_callback_ = callback;
    current_read_data_ = data;
    current_read_header_ = header;
  }

  void AsyncWrite(const Header& header,
                  const std::string_view& data,
                  const Header& query_header,
                  const mjlib::micro::SizeCallback& callback) override {
    const auto actual_dlc = RoundUpDlc(data.size());
    const uint32_t id =
        ((header.source & 0xff) << 8) |
        (header.destination & 0xff) |
        (can_prefix_ << 16);

    FDCan::SendOptions send_options;
    send_options.bitrate_switch =
        (query_header.flags & kBrsFlag) ?
        FDCan::Override::kRequire : FDCan::Override::kDisable;
    send_options.fdcan_frame =
        ((query_header.flags & kFdcanFlag) == 0 && data.size() <= 8) ?
        FDCan::Override::kDisable : FDCan::Override::kRequire;

    if (actual_dlc == data.size()) {
      fdcan_->Send(id, data, send_options);
    } else {
      std::memcpy(buf_, data.data(), data.size());
      for (size_t i = data.size(); i < actual_dlc; i++) {
        buf_[i] = 0x50;
      }
      fdcan_->Send(id, std::string_view(buf_, actual_dlc), send_options);
    }

    callback(mjlib::micro::error_code(), data.size());
  }

  Properties properties() const override {
    Properties properties;
    properties.max_size = 64;
    return properties;
  }

  void Poll() {
    if (!current_read_header_) { return; }

    const auto status = fdcan_->status();
    if (status.BusOff) {
      fdcan_->RecoverBusOff();
      can_reset_count_++;
    }

    const bool got_data = fdcan_->Poll(&fdcan_header_, current_read_data_);
    if (!got_data) { return; }

    // We could check the prefix here as below:
    //
    //   const uint16_t prefix = (fdcan_header_.Identifier >> 16) & 0x1fff;
    //   if (prefix != can_prefix_) { return; }
    //
    // However, we should be excluding prefix based on the hardware
    // CAN filter, and having the check here would mask if the filter
    // wasn't working.

    current_read_header_->destination = fdcan_header_.Identifier & 0xff;
    current_read_header_->source = (fdcan_header_.Identifier >> 8) & 0xff;
    current_read_header_->size = FDCan::ParseDlc(fdcan_header_.DataLength);
    current_read_header_->flags = 0
        | ((fdcan_header_.BitRateSwitch == FDCAN_BRS_ON) ? kBrsFlag : 0)
        | ((fdcan_header_.FDFormat == FDCAN_FD_CAN) ? kFdcanFlag : 0)
        ;

    auto copy = current_read_callback_;
    auto bytes = current_read_header_->size;

    current_read_callback_ = {};
    current_read_header_ = {};
    current_read_data_ = {};

    copy(mjlib::micro::error_code(), bytes);
  }

  static size_t RoundUpDlc(size_t value) {
    if (value == 0) { return 0; }
    if (value == 1) { return 1; }
    if (value == 2) { return 2; }
    if (value == 3) { return 3; }
    if (value == 4) { return 4; }
    if (value == 5) { return 5; }
    if (value == 6) { return 6; }
    if (value == 7) { return 7; }
    if (value == 8) { return 8; }
    if (value <= 12) { return 12; }
    if (value <= 16) { return 16; }
    if (value <= 20) { return 20; }
    if (value <= 24) { return 24; }
    if (value <= 32) { return 32; }
    if (value <= 48) { return 48; }
    if (value <= 64) { return 64; }
    return 0;
  }

  uint32_t can_reset_count() const { return can_reset_count_; }

 private:
  FDCan* const fdcan_;

  mjlib::micro::SizeCallback current_read_callback_;
  Header* current_read_header_ = nullptr;
  mjlib::base::string_span current_read_data_;

  FDCAN_RxHeaderTypeDef fdcan_header_ = {};
  char buf_[64] = {};
  uint32_t can_prefix_ = 0;
  uint32_t can_reset_count_ = 0;
};

}
