// Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cctype>

#include "mjlib/base/string_span.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/multiplex/micro_datagram_server.h"

namespace moteus {

// A very small ASCII fdcanusb line protocol emulator that allows
// using the moteus Python tools and GUI over a plain UART.
//
// It implements just enough of the fdcanusb text protocol to work
// with lib/python/moteus/fdcanusb_device.py:
//
// - Host -> device:
//     "can send <hex_id> <hex_payload> [flags]\n"
//   Device replies immediately:
//     "OK\n"
//
// - Device -> host (responses):
//     "rcv <hex_id> <hex_payload> [E] [B] [F]\n"
//
// The payload is padded to the CAN-FD DLC using 0x50 bytes.
class FdcanusbAsciiMicroServer : public mjlib::multiplex::MicroDatagramServer {
 public:
  // Flags in Header::flags (mirror FDCanMicroServer)
  static constexpr uint32_t kBrsFlag = 0x01;
  static constexpr uint32_t kFdcanFlag = 0x02;

  explicit FdcanusbAsciiMicroServer(mjlib::micro::AsyncStream* stream)
      : stream_(stream) {}

  void SetPrefix(uint32_t can_prefix) { can_prefix_ = can_prefix; }

  void AsyncRead(Header* header,
                 const mjlib::base::string_span& data,
                 const mjlib::micro::SizeCallback& callback) override {
    // Complete any pending read before starting a new one
    if (current_read_callback_) {
      current_read_callback_(mjlib::micro::error_code(), 0);
    }
    current_read_header_ = header;
    current_read_data_ = data;
    current_read_callback_ = callback;
  }

  void AsyncWrite(const Header& header,
                  const std::string_view& data,
                  const Header& query_header,
                  const mjlib::micro::SizeCallback& callback) override {
    // Build CAN ID: prefix | source | destination
    const uint32_t id = (can_prefix_ << 16) | 
                        ((header.source & 0xff) << 8) | 
                        (header.destination & 0xff);

    // Determine flags
    const bool brs = (query_header.flags & kBrsFlag) != 0;
    const bool fd = !((query_header.flags & kFdcanFlag) == 0 && data.size() <= 8);
    const size_t padded_len = RoundUpDlc(data.size());

    // Format: "rcv <id> <hex_payload> E [B] [F]\r\n" into dedicated buffer
    char* p = rcv_buf_;
    p += std::sprintf(p, "rcv %x ", static_cast<unsigned int>(id));

    // Hex-encode payload with padding
    for (size_t i = 0; i < padded_len; i++) {
      const uint8_t byte = (i < data.size()) ? static_cast<uint8_t>(data[i]) : 0x50;
      p += std::sprintf(p, "%02X", static_cast<unsigned int>(byte));
    }

    // Append flags
    *p++ = ' '; *p++ = 'E';
    if (brs) { *p++ = ' '; *p++ = 'B'; }
    if (fd)  { *p++ = ' '; *p++ = 'F'; }
    *p++ = '\r'; *p++ = '\n';

    rcv_len_ = p - rcv_buf_;  // Mark as ready to send
    if (callback) { callback(mjlib::micro::error_code(), data.size()); }
  }

  Properties properties() const override {
    return Properties{.max_size = 64};
  }

  void Poll() {
    // Send pending writes (OK has priority, then rcv response)
    auto write_done = [this](const mjlib::micro::error_code&, int) { write_active_ = false; };
    
    if (!write_active_) {
      if (pending_ok_) {
        pending_ok_ = false;
        write_active_ = true;
        static constexpr char kOk[] = "OK\r\n";
        stream_->AsyncWriteSome(std::string_view(kOk, sizeof(kOk) - 1), write_done);
      } else if (rcv_len_ > 0) {
        write_active_ = true;
        stream_->AsyncWriteSome(std::string_view(rcv_buf_, rcv_len_), write_done);
        rcv_len_ = 0;
      }
    }

    // Keep async read active
    if (!read_active_) {
      read_active_ = true;
      stream_->AsyncReadSome(mjlib::base::string_span(rx_buf_),
                             [this](mjlib::micro::error_code ec, size_t size) {
        if (!ec && size > 0) { AppendRx(rx_buf_, size); }
        read_active_ = false;
      });
    }

    // Process all complete lines
    while (true) {
      const int newline = FindNewline();
      if (newline < 0) { break; }
      HandleLine(newline);
    }
  }

 private:
  static constexpr uint8_t HexToNibble(char c) {
    return (c >= '0' && c <= '9') ? (c - '0') :
           (c >= 'a' && c <= 'f') ? (c - 'a' + 10) :
           (c >= 'A' && c <= 'F') ? (c - 'A' + 10) : 0;
  }

  static constexpr size_t RoundUpDlc(size_t value) {
    return (value <= 8) ? value :
           (value <= 12) ? 12 :
           (value <= 16) ? 16 :
           (value <= 20) ? 20 :
           (value <= 24) ? 24 :
           (value <= 32) ? 32 :
           (value <= 48) ? 48 : 64;
  }

  void AppendRx(const char* data, size_t size) {
    const size_t to_copy = std::min(size, sizeof(line_buf_) - line_len_);
    std::memcpy(line_buf_ + line_len_, data, to_copy);
    line_len_ += to_copy;
    if (line_len_ >= sizeof(line_buf_)) { line_len_ = 0; }  // Reset if full
  }

  int FindNewline() const {
    for (size_t i = 0; i < line_len_; i++) {
      if (line_buf_[i] == '\n' || line_buf_[i] == '\r') {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  void HandleLine(int newline_index) {
    if (newline_index <= 0) {
      Consume(newline_index + 1);
      return;
    }

    // Copy line to temporary buffer
    char tmp[192];
    const size_t copy = std::min(static_cast<size_t>(newline_index), sizeof(tmp) - 1);
    std::memcpy(tmp, line_buf_, copy);
    tmp[copy] = 0;
    Consume(newline_index + 1);

    // Only process "can send" commands
    if (std::strncmp(tmp, "can send ", 9) != 0) { return; }

    // Parse: "can send <id> <hex> [flags]"
    const char* p = tmp + 9;
    while (*p == ' ' || *p == '\t') { p++; }

    // Parse ID
    char* endp = nullptr;
    const uint32_t id = static_cast<uint32_t>(std::strtoul(p, &endp, 16));
    if (endp == p) { return; }
    p = endp;
    while (*p == ' ' || *p == '\t') { p++; }

    // Parse hex payload
    uint8_t payload[64];
    size_t payload_len = 0;
    while (payload_len < sizeof(payload) && 
           std::isxdigit(static_cast<unsigned char>(p[0])) &&
           std::isxdigit(static_cast<unsigned char>(p[1]))) {
      payload[payload_len++] = (HexToNibble(p[0]) << 4) | HexToNibble(p[1]);
      p += 2;
    }

    // Parse optional flags (B=BRS, F=FD)
    bool brs = false;
    bool fd = (payload_len > 8);
    for (; *p; p++) {
      if (*p == 'B') brs = true;
      else if (*p == 'F') fd = true;
    }

    // Deliver to multiplex server if a read is pending
    if (current_read_callback_) {
      current_read_header_->destination = id & 0xff;
      current_read_header_->source = (id >> 8) & 0xff;
      current_read_header_->size = payload_len;
      current_read_header_->flags = (brs ? kBrsFlag : 0) | (fd ? kFdcanFlag : 0);

      const size_t to_copy = std::min(payload_len, static_cast<size_t>(current_read_data_.size()));
      std::memcpy(current_read_data_.data(), payload, to_copy);

      auto cb = current_read_callback_;
      current_read_callback_ = {};
      cb(mjlib::micro::error_code(), to_copy);
    }

    pending_ok_ = true;  // Queue "OK" acknowledgment
  }

  void Consume(size_t n) {
    if (n >= line_len_) {
      line_len_ = 0;
    } else {
      std::memmove(line_buf_, line_buf_ + n, line_len_ - n);
      line_len_ -= n;
    }
  }

  mjlib::micro::AsyncStream* const stream_;
  uint32_t can_prefix_ = 0;

  // In-flight read state for MicroDatagramServer.
  Header* current_read_header_ = nullptr;
  mjlib::base::string_span current_read_data_;
  mjlib::micro::SizeCallback current_read_callback_;

  // Serial IO buffers.
  bool read_active_ = false;
  char rx_buf_[128] = {};

  // Partial line accumulation.
  char line_buf_[256] = {};
  size_t line_len_ = 0;

  // Write management: two-slot system (OK + one rcv response).
  bool write_active_ = false;
  bool pending_ok_ = false;
  char rcv_buf_[256] = {};
  size_t rcv_len_ = 0;
};

}  // namespace moteus

