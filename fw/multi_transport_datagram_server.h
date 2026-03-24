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

#include "mjlib/multiplex/micro_datagram_server.h"

#include "fw/error.h"
#include "fw/fdcan_micro_server.h"
#include "fw/uart_fdcanusb_micro_server.h"

namespace moteus {

/// Multiplexes multiple datagram transports (CAN-FD and UART) into a single
/// MicroDatagramServer interface.
///
/// This allows both CAN-FD and UART serial control to work in the
/// same firmware image.  Responses are routed back to whichever
/// transport the request came from.
///
/// Transport source is encoded in the upper bits of Header::flags:
///   bits 0-7: Original transport flags (BRS, FDCAN for CAN)
///   bits 8-15: Transport ID (0 = CAN-FD, 1 = UART)
class MultiTransportDatagramServer : public mjlib::multiplex::MicroDatagramServer {
 public:
  static constexpr uint32_t kTransportIdMask = 0xFF00;
  static constexpr uint32_t kTransportIdShift = 8;
  static constexpr uint32_t kTransportIdCan = 0;
  static constexpr uint32_t kTransportIdUart = 1;

  explicit MultiTransportDatagramServer(FDCanMicroServer* fdcan)
      : fdcan_(fdcan) {}

  void AsyncRead(Header* header,
                 const mjlib::base::string_span& data,
                 const mjlib::micro::SizeCallback& callback) override {
    MJ_ASSERT(!pending_read_callback_);

    pending_read_callback_ = callback;
    pending_read_header_ = header;
    pending_read_data_ = data;

    // Only start a new CAN read if one isn't already in flight.
    if (!can_read_in_flight_) {
      can_read_in_flight_ = true;
      fdcan_->AsyncRead(&can_header_, can_buffer_,
                        [this](mjlib::micro::error_code ec, size_t size) {
                          HandleCanRead(ec, size);
                        });
    }

    // Only start a new UART read if one isn't already in flight.
    if (uart_server_ && !uart_read_in_flight_) {
      uart_read_in_flight_ = true;
      uart_server_->AsyncRead(&uart_header_, uart_buffer_,
                              [this](mjlib::micro::error_code ec, size_t size) {
                                HandleUartRead(ec, size);
                              });
    }
  }

  void AsyncWrite(const Header& header,
                  const std::string_view& data,
                  const Header& query_header,
                  const mjlib::micro::SizeCallback& callback) override {
    // Determine which transport to use based on encoded transport ID.
    const uint32_t transport_id =
        (query_header.flags & kTransportIdMask) >> kTransportIdShift;

    // Create a header without our transport ID bits for the
    // underlying transport.
    Header clean_query_header = query_header;
    clean_query_header.flags &= ~kTransportIdMask;

    if (transport_id == kTransportIdUart) {
      // Store the callback so we can invoke it if UART is destroyed
      pending_uart_write_callback_ = callback;
      pending_uart_write_size_ = data.size();

      if (uart_server_) {
        uart_server_->AsyncWrite(
            header, data, clean_query_header,
            [this](mjlib::micro::error_code ec, size_t size) {
              auto cb = pending_uart_write_callback_;
              pending_uart_write_callback_ = {};
              if (cb) {
                cb(ec, size);
              }
            });
      }
    } else {
      fdcan_->AsyncWrite(header, data, clean_query_header, callback);
    }
  }

  Properties properties() const override {
    // Return CAN-FD properties (64 byte max)
    return fdcan_->properties();
  }

  void Poll() {
    fdcan_->Poll();
    if (uart_server_) {
      uart_server_->Poll();
    }
  }

  uint32_t can_reset_count() const {
    return fdcan_->can_reset_count();
  }

  void SetPrefix(uint32_t prefix) {
    current_prefix_ = prefix;  // Cache for new servers
    fdcan_->SetPrefix(prefix);
    if (uart_server_) {
      uart_server_->SetPrefix(prefix);
    }
  }

  // Set the active UART server. Handles transitions between servers,
  // cleaning up any pending operations on the old server and configuring
  // the new server with the current prefix.
  void SetActiveUartServer(UartFdcanusbMicroServer* new_server) {
    if (new_server == uart_server_) { return; }

    UartFdcanusbMicroServer* const old_server = uart_server_;
    uart_server_ = new_server;

    // Clean up operations on old server (callbacks won't fire)
    if (old_server && uart_read_in_flight_) {
      uart_read_in_flight_ = false;
    }

    // Whether or not we had an old server, this callback definitely
    // needs to be called with an error.
    if (pending_uart_write_callback_) {
      auto cb = pending_uart_write_callback_;
      pending_uart_write_callback_ = {};
      cb(errc::kUartOverrunError, 0);
    }

    // Configure new server with current prefix
    if (new_server) {
      new_server->SetPrefix(current_prefix_);
    }

    // Start read on new server if needed
    if (new_server && pending_read_callback_ && !uart_read_in_flight_) {
      uart_read_in_flight_ = true;
      new_server->AsyncRead(&uart_header_, uart_buffer_,
                            [this](mjlib::micro::error_code ec, size_t size) {
                              HandleUartRead(ec, size);
                            });
    }
  }

 private:
  void HandleCanRead(mjlib::micro::error_code ec, size_t size) {
    can_read_in_flight_ = false;

    if (!pending_read_callback_) {
      // Already delivered via other transport
      return;
    }

    if (ec || size == 0) {
      // Pass errors through so the MicroServer can re-issue its
      // AsyncRead, which restarts reads on both transports.
      auto cb = pending_read_callback_;
      pending_read_callback_ = {};
      cb(ec, size);
      return;
    }

    DeliverRead(kTransportIdCan, can_header_, can_buffer_, size);
  }

  void HandleUartRead(mjlib::micro::error_code ec, size_t size) {
    uart_read_in_flight_ = false;

    if (!pending_read_callback_) {
      // Already delivered via other transport
      return;
    }

    if (ec || size == 0) {
      auto cb = pending_read_callback_;
      pending_read_callback_ = {};
      cb(ec, size);
      return;
    }

    DeliverRead(kTransportIdUart, uart_header_, uart_buffer_, size);
  }

  void DeliverRead(uint32_t transport_id,
                   const Header& src_header,
                   const mjlib::base::string_span& src_data,
                   size_t size) {
    if (!pending_read_callback_) { return; }

    // Copy data to caller's buffer
    const size_t copy_size = std::min<size_t>(
        size,
        pending_read_data_.size());
    std::memcpy(pending_read_data_.data(), src_data.data(), copy_size);

    // Fill in header with transport ID encoded in flags
    pending_read_header_->destination = src_header.destination;
    pending_read_header_->source = src_header.source;
    pending_read_header_->size = copy_size;
    pending_read_header_->flags =
        src_header.flags | (transport_id << kTransportIdShift);

    // Deliver callback
    auto cb = pending_read_callback_;
    pending_read_callback_ = {};
    pending_read_header_ = nullptr;
    pending_read_data_ = {};

    cb(mjlib::micro::error_code(), copy_size);
  }

  FDCanMicroServer* const fdcan_;

  // Cached UART server pointer and prefix
  UartFdcanusbMicroServer* uart_server_ = nullptr;
  uint32_t current_prefix_ = 0;

  // Pending read from MicroServer
  mjlib::micro::SizeCallback pending_read_callback_;
  Header* pending_read_header_ = nullptr;
  mjlib::base::string_span pending_read_data_;

  // Per-transport read state
  Header can_header_;
  char can_buffer_data_[64] = {};
  mjlib::base::string_span can_buffer_{can_buffer_data_, sizeof(can_buffer_data_)};
  bool can_read_in_flight_ = false;

  Header uart_header_;
  char uart_buffer_data_[64] = {};
  mjlib::base::string_span uart_buffer_{uart_buffer_data_, sizeof(uart_buffer_data_)};
  bool uart_read_in_flight_ = false;

  // Pending UART write callback (stored here so we can invoke on UART destruction)
  mjlib::micro::SizeCallback pending_uart_write_callback_;
  size_t pending_uart_write_size_ = 0;
};

}
