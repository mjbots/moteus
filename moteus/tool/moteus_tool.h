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

#include <cstdint>
#include <memory>

#include <boost/asio/executor.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/program_options.hpp>

#include "mjlib/io/async_stream.h"
#include "mjlib/io/async_types.h"

namespace moteus {
namespace tool {

class ClientBase {
 public:
  virtual ~ClientBase() {}

  struct TunnelOptions {
    // Poll this often for data to be received.
    boost::posix_time::time_duration poll_rate =
        boost::posix_time::milliseconds(10);

    TunnelOptions() {}
  };

  /// Allocate a tunnel which can be used to send and receive serial
  /// stream data.
  virtual mjlib::io::SharedStream MakeTunnel(
      uint8_t id,
      uint32_t channel,
      const TunnelOptions& options = TunnelOptions()) = 0;
};

class ClientMaker {
 public:
  virtual void AddToProgramOptions(
      boost::program_options::options_description*) = 0;
  virtual void AsyncCreate(boost::asio::executor,
                           std::unique_ptr<ClientBase>*,
                           mjlib::io::ErrorCallback) = 0;
};

int moteus_tool_main(int argc, char** argv, ClientMaker* = nullptr);
}
}
