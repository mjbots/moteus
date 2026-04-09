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

#include <cstring>

#include "mjlib/base/string_span.h"
#include "mjlib/multiplex/micro_datagram_server.h"

#include "fw/stm32g4_dma_uart.h"
#include "fw/strtof.h"

namespace moteus {

/// Implements the fdcanusb ASCII protocol over a UART stream,
/// presenting it as a MicroDatagramServer interface.
///
/// Protocol format:
///   Receive: "rcv <extended_id_hex> <data_hex> <flags>\n"
///   Send: "can send <extended_id_hex> <data_hex> <flags>\r\n"
class UartFdcanusbMicroServer : public mjlib::multiplex::MicroDatagramServer {
 public:
  UartFdcanusbMicroServer(Stm32G4DmaUart* uart) : uart_(uart) {
    // Start the DMA read immediately
    StartDmaRead();
  }

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
                  const Header&,
                  const mjlib::micro::SizeCallback& callback) override {
    // Only one callback-tracked write can be pending at a time
    MJ_ASSERT(!pending_write_callback_);

    // Format: "rcv <id_hex> <data_hex>\r\n"
    // (fdcanusb protocol: device sends "rcv" to host when data is received)
    const uint32_t id =
        ((header.source & 0xff) << 8) |
        (header.destination & 0xff) |
        (can_prefix_ << 16);

    // Format into temporary buffer then append
    // "rcv " (4) + hex ID (8) + " " (1) + hex data (2*max_size) +
    // checksum " *XX" (4) + "\r\n" (2)
    static constexpr size_t kMaxDataSize = 64;
    static constexpr size_t kTempSize = 4 + 8 + 1 + 2 * kMaxDataSize + 4 + 2;
    char temp[kTempSize];
    char* ptr = temp;
    std::memcpy(ptr, "rcv ", 4);
    ptr += 4;

    // Write ID in hex (8 hex digits for 32-bit extended ID)
    ptr = WriteHex32(ptr, id);
    *ptr++ = ' ';

    // Write data bytes in hex
    for (size_t i = 0; i < data.size(); i++) {
      ptr = WriteHex8(ptr, static_cast<uint8_t>(data[i]));
    }

    if (checksum_active_) {
      ptr = AppendCrc(ptr, temp);
    }

    *ptr++ = '\r';
    *ptr++ = '\n';

    // Track callback - will fire when this buffer's DMA completes
    pending_write_callback_ = callback;
    pending_write_data_size_ = data.size();
    callback_buf_idx_ = pending_buf_idx_;

    // Append to pending buffer
    AppendToWriteBuffer(temp, ptr - temp);
  }

  Properties properties() const override {
    Properties properties;
    properties.max_size = 64;
    return properties;
  }

  void Poll() {
    // Check for completed writes
    if (dma_write_active_ && uart_->is_dma_write_finished()) {
      uart_->finish_dma_write();
      dma_write_active_ = false;

      // If this buffer had callback-tracked data, fire callback
      if (pending_write_callback_ && callback_buf_idx_ == active_buf_idx_) {
        auto cb = pending_write_callback_;
        auto sz = pending_write_data_size_;
        pending_write_callback_ = {};
        pending_write_data_size_ = 0;
        callback_buf_idx_ = -1;
        cb(mjlib::micro::error_code(), sz);
      }

      // Start any pending write
      MaybeStartPendingWrite();
    }

    // Check for received data
    ProcessDmaRx();
  }

 private:
  void StartDmaRead() {
    // Clear any pending error flags and drain RDR before starting DMA.
    // If a byte arrived while DMA was disabled, it's stuck in RDR and
    // would cause an overrun on the next byte (DMA request is edge-triggered).
    auto* const uart = uart_->uart();
    uart->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    while (uart->ISR & USART_ISR_RXNE) {
      // Read and buffer any pending bytes
      if (rx_line_len_ < sizeof(rx_buf_)) {
        rx_buf_[rx_line_len_++] = uart->RDR;
      } else {
        (void)uart->RDR;  // Discard if buffer full
      }
    }

    uart_->start_dma_read(
        mjlib::base::string_span(rx_buf_ + rx_line_len_,
                                 sizeof(rx_buf_) - rx_line_len_));
  }

  void ProcessDmaRx() {
    // rx_buf_ layout: [partial line data (rx_line_len_ bytes)][DMA write area]
    // DMA writes into rx_buf_ starting at position rx_line_len_

    const size_t dma_buf_size = sizeof(rx_buf_) - rx_line_len_;
    const size_t bytes_remaining = uart_->read_bytes_remaining();
    const size_t new_bytes = dma_buf_size - bytes_remaining;
    const size_t total_bytes = rx_line_len_ + new_bytes;

    // Scan for newline in the buffer
    size_t newline_pos = total_bytes;
    for (size_t i = 0; i < total_bytes; i++) {
      if (rx_buf_[i] == '\n' || rx_buf_[i] == '\r') {
        newline_pos = i;
        break;
      }
    }

    if (newline_pos >= total_bytes) {
      // No complete line yet.  Check for buffer overflow.  The
      // margin is just to avoid any off-by-one concerns; the DMA
      // is size-limited and cannot overfill the buffer.
      if (total_bytes >= sizeof(rx_buf_) - 2) {
        // Line too long, discard and restart
        uart_->finish_dma_read();
        rx_line_len_ = 0;
        StartDmaRead();
      }
      return;
    }

    // Found a complete line - stop DMA to get a consistent final
    // byte count.  More bytes may have arrived between the first
    // read_bytes_remaining() and here; the newline position is still
    // valid, and any extra bytes become leftovers for the next line.
    uart_->finish_dma_read();

    const size_t final_remaining = uart_->read_bytes_remaining();
    const size_t final_new_bytes = dma_buf_size - final_remaining;
    const size_t final_total = rx_line_len_ + final_new_bytes;

    // Process the line (excluding the newline character)
    if (newline_pos > 0) {
      if (!ProcessLine(std::string_view(rx_buf_, newline_pos))) {
        SendError("unknown command");
      }
    }

    // Skip consecutive newline characters (\r\n, \n\r, etc.)
    size_t skip_pos = newline_pos;
    while (skip_pos < final_total &&
           (rx_buf_[skip_pos] == '\n' || rx_buf_[skip_pos] == '\r')) {
      skip_pos++;
    }

    // Move remaining bytes (start of next line) to beginning of buffer
    rx_line_len_ = final_total - skip_pos;
    if (rx_line_len_ > 0) {
      std::memmove(rx_buf_, rx_buf_ + skip_pos, rx_line_len_);
    }

    // Restart DMA after the leftover bytes
    StartDmaRead();
  }

  // Validate and strip any checksum from the line if one is present.
  //
  // Returns true if line is valid, false if checksum error.
  //
  // On success, *line is modified to strip the checksum suffix.
  bool ValidateChecksum(std::string_view* line) {
    // Find the last '*' in the line
    size_t star_pos = std::string_view::npos;
    for (size_t i = line->size(); i > 0; i--) {
      if ((*line)[i - 1] == '*') {
        star_pos = i - 1;
        break;
      }
    }

    // No '*' found, or it's not in the right position for " *XX" at end of line
    if (star_pos == std::string_view::npos || star_pos + 3 != line->size()) {
      return !checksum_active_;
    }

    // Parse the two hex digits after '*'
    const int hi = ParseHexDigit((*line)[star_pos + 1]);
    const int lo = ParseHexDigit((*line)[star_pos + 2]);
    if (hi < 0 || lo < 0) {
      return !checksum_active_;
    }

    // Validate the checksum
    const uint8_t claimed = static_cast<uint8_t>((hi << 4) | lo);
    const uint8_t computed = ComputeCrc8(line->data(), star_pos);
    if (claimed != computed) {
      return false;
    }

    // We have at least one success, so all future interactions will
    // require checksums.
    checksum_active_ = true;

    // Now strip the checksum from the line.
    *line = line->substr(0, star_pos);
    while (!line->empty() && line->back() == ' ') {
      *line = line->substr(0, line->size() - 1);
    }
    return true;
  }

  // Returns true if command was recognized, false otherwise
  bool ProcessLine(std::string_view line) {
    if (!ValidateChecksum(&line)) {
      SendError("checksum");
      return true;
    }

    // Parse "can send <id_hex> <data_hex>"
    // (fdcanusb protocol: host sends "can send" to device)
    if (line.size() < 9) { return false; }
    if (line.substr(0, 9) != "can send ") { return false; }

    line = line.substr(9);

    // Parse extended ID (hex)
    uint32_t id = 0;
    size_t id_end = 0;
    for (; id_end < line.size() && line[id_end] != ' '; id_end++) {
      const int digit = ParseHexDigit(line[id_end]);
      if (digit < 0) {
        SendError("invalid hex in id");
        return true;  // Command was recognized, just malformed
      }
      id = (id << 4) | digit;
    }

    if (id_end >= line.size()) {
      SendError("missing data");
      return true;  // Command was recognized, just malformed
    }
    line = line.substr(id_end + 1);  // Skip space

    // Check prefix
    const uint16_t prefix = (id >> 16) & 0x1fff;
    if (prefix != can_prefix_) {
      // Respond with nothing, which is what a real device would do if
      // the prefix did not match.

      return true;  // Command was recognized, just wrong prefix
    }

    // If we have a pending read, deliver this data
    if (!current_read_callback_) {
      // No pending read - just acknowledge but don't process.  In
      // other words, drop it on the floor.
      SendOk();
      return true;
    }

    // Parse the data bytes, which will be hex pairs.
    size_t data_size = 0;
    for (size_t i = 0; i + 1 < line.size() &&
         data_size < static_cast<size_t>(current_read_data_.size()); i += 2) {
      if (line[i] == ' ' || line[i] == '\r' || line[i] == '\n') {
        break;
      }
      const int hi = ParseHexDigit(line[i]);
      const int lo = ParseHexDigit(line[i + 1]);
      if (hi < 0 || lo < 0) {
        // Some of our data was bogus, report an error and bail
        SendError("malformed data");
        return true;
      }
      current_read_data_[data_size++] = static_cast<char>((hi << 4) | lo);
    }

    // Fill in header
    current_read_header_->destination = id & 0xff;
    current_read_header_->source = (id >> 8) & 0xff;
    current_read_header_->size = data_size;
    current_read_header_->flags = 0;  // No BRS/FDCAN flags for UART

    // Send OK response (fdcanusb protocol acknowledgment)
    SendOk();

    // Deliver callback
    auto copy = current_read_callback_;
    auto bytes = data_size;

    current_read_callback_ = {};
    current_read_header_ = {};
    current_read_data_ = {};

    copy(mjlib::micro::error_code(), bytes);
    return true;
  }

  void SendOk() {
    if (checksum_active_) {
      char temp[16];
      char* ptr = temp;
      std::memcpy(ptr, "OK", 2);
      ptr += 2;
      ptr = AppendCrc(ptr, temp);
      *ptr++ = '\r';
      *ptr++ = '\n';
      AppendToWriteBuffer(temp, ptr - temp);
    } else {
      static const char ok_response[] = "OK\r\n";
      AppendToWriteBuffer(ok_response, 4);
    }
  }

  void SendError(const char* message) {
    // Format: "ERR <message>\r\n" or "ERR <message> *XX\r\n"
    char temp[64];
    char* ptr = temp;
    std::memcpy(ptr, "ERR ", 4);
    ptr += 4;
    while (*message && ptr < temp + sizeof(temp) - 8) {
      *ptr++ = *message++;
    }
    if (checksum_active_) {
      ptr = AppendCrc(ptr, temp);
    }
    *ptr++ = '\r';
    *ptr++ = '\n';

    AppendToWriteBuffer(temp, ptr - temp);
  }

  void AppendToWriteBuffer(const char* data, size_t size) {
    char* buf = write_buf_[pending_buf_idx_];
    const size_t available = sizeof(write_buf_[0]) - pending_size_;
    const size_t to_copy = std::min(size, available);
    if (to_copy > 0) {
      std::memcpy(buf + pending_size_, data, to_copy);
      pending_size_ += to_copy;
    }
    MaybeStartPendingWrite();
  }

  void MaybeStartPendingWrite() {
    if (dma_write_active_) { return; }
    if (pending_size_ == 0) { return; }

    // The pending buffer becomes the active (DMA) buffer
    active_buf_idx_ = pending_buf_idx_;
    const size_t size_to_send = pending_size_;

    // Switch to other buffer for new pending data
    pending_buf_idx_ = 1 - pending_buf_idx_;
    pending_size_ = 0;

    dma_write_active_ = true;
    uart_->start_dma_write(
        std::string_view(write_buf_[active_buf_idx_], size_to_send));
  }

  static int ParseHexDigit(char c) {
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    return -1;
  }

  static char* WriteHex32(char* ptr, uint32_t value) {
    for (int i = 7; i >= 0; i--) {
      const int digit = (value >> (i * 4)) & 0xf;
      *ptr++ = digit < 10 ? ('0' + digit) : ('a' + digit - 10);
    }
    return ptr;
  }

  static char* WriteHex8(char* ptr, uint8_t value) {
    const int hi = (value >> 4) & 0xf;
    const int lo = value & 0xf;
    *ptr++ = hi < 10 ? ('0' + hi) : ('a' + hi - 10);
    *ptr++ = lo < 10 ? ('0' + lo) : ('a' + lo - 10);
    return ptr;
  }

  static constexpr uint8_t kCrc8Table[16] = {
      0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb,
      0x5d, 0xca, 0xe4, 0x73, 0xb8, 0x2f, 0x01, 0x96,
  };

  static uint8_t ComputeCrc8(const char* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
      const uint8_t b = static_cast<uint8_t>(data[i]);
      crc = kCrc8Table[((crc >> 4) ^ (b >> 4)) & 0x0f] ^ (crc << 4);
      crc = kCrc8Table[((crc >> 4) ^ (b & 0x0f)) & 0x0f] ^ (crc << 4);
    }
    return crc;
  }

  static char* AppendCrc(char* ptr, const char* start) {
    // Add a space first, then compute the CRC over all of the content
    // including the space.
    //
    // This matches how we validate: The CRC is over everything before
    // the '*'.
    *ptr++ = ' ';
    const uint8_t crc = ComputeCrc8(start, ptr - start);
    *ptr++ = '*';
    const int hi = (crc >> 4) & 0xf;
    const int lo = crc & 0xf;
    *ptr++ = hi < 10 ? ('0' + hi) : ('A' + hi - 10);
    *ptr++ = lo < 10 ? ('0' + lo) : ('A' + lo - 10);
    return ptr;
  }

  Stm32G4DmaUart* const uart_;

  mjlib::micro::SizeCallback current_read_callback_;
  Header* current_read_header_ = nullptr;
  mjlib::base::string_span current_read_data_;

  // Callback for AsyncWrite - only one can be pending at a time
  mjlib::micro::SizeCallback pending_write_callback_;
  size_t pending_write_data_size_ = 0;
  int callback_buf_idx_ = -1;  // Which buffer contains callback-tracked data (-1 = none)

  // Double-buffered write buffers
  // Data is appended to the pending buffer. When DMA completes (or is idle),
  // the pending buffer becomes active and starts DMA, while the other buffer
  // becomes the new pending buffer.
  char write_buf_[2][256] = {};
  int pending_buf_idx_ = 0;       // Buffer we're appending to
  size_t pending_size_ = 0;       // Size of data in pending buffer
  int active_buf_idx_ = 0;        // Buffer currently being DMA'd
  bool dma_write_active_ = false; // True while DMA write is in progress

  // Receive buffer - always starts at the beginning of a line.
  // rx_line_len_ bytes of partial line data at the start, DMA writes after.
  char rx_buf_[256] = {};
  size_t rx_line_len_ = 0;

  uint32_t can_prefix_ = 0;
  bool checksum_active_ = false;
};

}
