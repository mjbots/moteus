// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include <functional>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "mjlib/base/clipp.h"
#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/system_error.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/io/now.h"
#include "mjlib/io/selector.h"
#include "mjlib/io/stream_factory.h"
#include "mjlib/multiplex/asio_client.h"
#include "mjlib/multiplex/stream_asio_client_builder.h"

#include "utils/line_reader.h"

namespace pl = std::placeholders;

namespace {
struct Options {
  std::string type;
  int target = 1;
  mjlib::io::StreamFactory::Options stream;
};

class ClockCal {
 public:
  ClockCal(const boost::asio::any_io_executor& executor,
           mjlib::io::Selector<mjlib::multiplex::AsioClient>* selector,
           const Options& options)
      : executor_(executor),
        selector_(selector),
        options_(options),
        factory_(executor_) {
  }

  void Start() {
    if (options_.type == "stream") {
      factory_.AsyncCreate(options_.stream,
                           std::bind(&ClockCal::HandleStream, this, pl::_1, pl::_2));
    } else if (options_.type == "multiplex") {
      selector_->AsyncStart(
          std::bind(&ClockCal::HandleClient, this, pl::_1));
    } else {
      throw mjlib::base::system_error::einval("unknown type: " + options_.type);
    }
  }

 private:
  void HandleClient(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);
    HandleStream(ec, selector_->selected()->MakeTunnel(options_.target, 1));
  }

  void HandleStream(const mjlib::base::error_code& ec, mjlib::io::SharedStream stream) {
    mjlib::base::FailIf(ec);

    stream_ = stream;
    reader_.emplace(stream.get());

    boost::asio::co_spawn(
        executor_,
        std::bind(&ClockCal::Calibrate, this),
        [](std::exception_ptr ptr) {
          if (ptr) {
            std::rethrow_exception(ptr);
          }
          std::exit(0);
        });
  }

  boost::asio::awaitable<void> Write(const std::string& message) {
    co_await boost::asio::async_write(
        *stream_,
        boost::asio::buffer(message + "\n"),
        boost::asio::use_awaitable);
    co_return;
  }

  boost::asio::awaitable<void> Sleep(double seconds) {
    boost::asio::deadline_timer timer(executor_);
    timer.expires_from_now(mjlib::base::ConvertSecondsToDuration(seconds));
    co_await timer.async_wait(boost::asio::use_awaitable);
  }

  boost::asio::awaitable<void> Calibrate() {
    boost::posix_time::ptime last_time;
    int64_t last_us = 0;

    for (int i = 0; i < 5; i++) {
      co_await Write("clock us");
      const auto us = std::stoul((co_await reader_->ReadLine()).value());
      const auto timestamp = mjlib::io::Now(executor_.context());

      const double us_per_s = [&]() {
        if (last_time.is_special()) { return 1000000.0; }

        return (static_cast<double>(us - last_us) /
                mjlib::base::ConvertDurationToSeconds(timestamp - last_time));
      }();

      fmt::print("{} {} {:.0f}\n", us, timestamp, us_per_s);

      last_time = timestamp;
      last_us = us;

      co_await Sleep(1.0);
    }

    co_return;
  }

  boost::asio::any_io_executor executor_;
  mjlib::io::Selector<mjlib::multiplex::AsioClient>* const selector_;
  const Options options_;
  mjlib::io::StreamFactory factory_;

  mjlib::io::SharedStream stream_;
  std::optional<moteus::tool::LineReader> reader_;
  };
}

int main(int argc, char** argv) {
  boost::asio::io_context context;

  mjlib::io::Selector<mjlib::multiplex::AsioClient> default_client_selector{
    context.get_executor(), "client_type"};
  mjlib::multiplex::StreamAsioClientBuilder::Options default_stream_options;
  default_stream_options.stream.type = mjlib::io::StreamFactory::Type::kSerial;
  default_stream_options.stream.serial_port = "/dev/fdcanusb";
  default_stream_options.stream.serial_baud = 3000000;

  default_client_selector.Register<mjlib::multiplex::StreamAsioClientBuilder>(
      "stream", default_stream_options);
  default_client_selector.set_default("stream");

  Options options;
  auto group = clipp::group(
      clipp::option("type") & clipp::value("TYPE", options.type).doc(
          "one of [stream,multiplex]"),
      clipp::option("target", "t") & clipp::value("ID", options.target)
                            );
  group.merge(mjlib::base::ClippArchive("stream.").Accept(&options.stream).release());
  group.merge(clipp::with_prefix("client.", default_client_selector.program_options()));
  mjlib::base::ClippParse(argc, argv, group);

  ClockCal app(context.get_executor(), &default_client_selector, options);
  app.Start();
  context.run();
  return 0;
}
