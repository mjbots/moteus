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

#include <iostream>
#include <optional>
#include <string>

#include <boost/asio/awaitable.hpp>
#include <boost/asio/streambuf.hpp>

#include "mjlib/base/assert.h"
#include "mjlib/base/system_error.h"
#include "mjlib/io/async_stream.h"

namespace moteus {
namespace tool {

class LineReader {
 public:
  struct Options {
    bool verbose = false;

    Options& set_verbose(bool value) {
      verbose = value;
      return *this;
    }

    Options() {}
  };

  LineReader(mjlib::io::AsyncStream* stream, const Options& options = {})
      : stream_(stream),
        options_(options) {
  }

  ~LineReader() {
  }

  boost::asio::awaitable<std::optional<std::string>> ReadLine() {
    boost::system::error_code ec;
    co_await boost::asio::async_read_until(
        *stream_,
        read_streambuf_,
        '\n',
        boost::asio::redirect_error(boost::asio::use_awaitable, ec));
    if (ec == boost::asio::error::operation_aborted) {
      co_return std::optional<std::string>();
    } else if (ec) {
      throw mjlib::base::system_error(ec);
    }
    std::istream istr(&read_streambuf_);

    std::string result;
    std::getline(istr, result);

    if (options_.verbose) {
      fmt::print("< {}\n", result);
    }
    co_return result;
  }

  auto AtLeast(int size) {
    return [this, size](auto, auto) -> size_t  {
      return std::max<int>(0, size - this->read_streambuf_.size());
    };
  }

  boost::asio::awaitable<std::string> ReadBinaryBlob() {
    boost::system::error_code ec;
    co_await boost::asio::async_read(
        *stream_,
        read_streambuf_,
        AtLeast(4),
        boost::asio::redirect_error(boost::asio::use_awaitable, ec));

    std::istream istr(&read_streambuf_);
    uint32_t size = 0;
    istr.read(reinterpret_cast<char*>(&size), sizeof(size));
    MJ_ASSERT(istr.gcount() == sizeof(size));

    co_await boost::asio::async_read(
        *stream_,
        read_streambuf_,
        AtLeast(size),
        boost::asio::redirect_error(boost::asio::use_awaitable, ec));

    std::string result;
    result.resize(size);

    istr.read(&result[0], size);
    co_return result;
  }

 private:
  mjlib::io::AsyncStream* const stream_;
  const Options options_;

  boost::asio::streambuf read_streambuf_;
};

}
}
