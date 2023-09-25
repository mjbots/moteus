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

#include <array>

#include "mjlib/base/assert.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/async_types.h"

namespace moteus {

/// Supports writing to a stream where the buffer passed to AsyncWrite
/// does not need to live past the call.  It does this through an
/// internal double buffer.
template <size_t Size, size_t NumCallbacks = 14>
class StreamWriter {
 public:
  StreamWriter(mjlib::micro::AsyncWriteStream* stream)
      : stream_(stream) {}

  void AsyncWrite(const std::string_view& buffer,
                  const mjlib::micro::ErrorCallback& callback) {
    // Verify we have sufficient room.
    MJ_ASSERT((current_offset_ + buffer.size()) <= Size);

    std::memcpy(&data_in_progress_->buffer[current_offset_],
                buffer.data(), buffer.size());
    current_offset_ += buffer.size();

    const bool installed_callback = [&]() {
      for (auto& cbk_holder : data_in_progress_->callbacks) {
        if (!cbk_holder) {
          cbk_holder = callback;
          return true;
        }
      }
      return false;
    }();
    MJ_ASSERT(installed_callback);

    MaybeStartWrite();
  }

 private:
  void MaybeStartWrite() {
    if (write_outstanding_) { return; }
    if (current_offset_ == 0) { return; }

    // Swap our buffers and get ready to write.
    const size_t size_to_write = current_offset_;
    std::swap(data_in_progress_, data_writing_);
    current_offset_ = 0;

    write_outstanding_ = true;
    mjlib::micro::AsyncWrite(
        *stream_,
        std::string_view(data_writing_->buffer, size_to_write),
        std::bind(&StreamWriter::HandleWrite, this, std::placeholders::_1));
  }

  void HandleWrite(const mjlib::micro::error_code& ec) {
    write_outstanding_ = false;
    for (auto& cbk : data_writing_->callbacks) {
      if (!cbk) {
        continue;
      }
      auto copy = cbk;
      cbk = {};
      copy(ec);
    }

    MaybeStartWrite();
  }

  mjlib::micro::AsyncWriteStream* const stream_;

  struct Data {
    char buffer[Size] = {};
    std::array<mjlib::micro::ErrorCallback, NumCallbacks> callbacks;
  };

  Data data1_;
  Data data2_;
  size_t current_offset_ = 0;

  Data* data_in_progress_ = &data1_;
  Data* data_writing_ = &data2_;

  bool write_outstanding_ = false;
};

}
