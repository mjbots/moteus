// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#include "moteus/fdcan.h"

namespace moteus {

class FDCanMicroServer : public mjlib::multiplex::MicroDatagramServer {
 public:
  FDCanMicroServer(FDCan* can) : fdcan_(can) {}

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
                  const mjlib::micro::SizeCallback& callback) override {
    const uint32_t id =
        ((header.source & 0xff) << 8) | (header.destination & 0xff);
    fdcan_->Send(id, data, {});
    callback(mjlib::micro::error_code(), data.size());
  }

  void Poll() {
    if (!current_read_header_) { return; }

    const bool got_data = fdcan_->Poll(&fdcan_header_, current_read_data_);
    if (!got_data) { return; }

    current_read_header_->destination = fdcan_header_.Identifier & 0xff;
    current_read_header_->source = (fdcan_header_.Identifier >> 8) & 0xff;
    current_read_header_->size = FDCan::ParseDlc(fdcan_header_.DataLength);

    auto copy = current_read_callback_;
    auto bytes = current_read_header_->size;

    current_read_callback_ = {};
    current_read_header_ = {};
    current_read_data_ = {};

    copy(mjlib::micro::error_code(), bytes);
  }

 private:
  FDCan* const fdcan_;

  mjlib::micro::SizeCallback current_read_callback_;
  Header* current_read_header_ = nullptr;
  mjlib::base::string_span current_read_data_;

  FDCAN_RxHeaderTypeDef fdcan_header_ = {};
};

}
