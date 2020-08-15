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

/// @file
///
/// Drive a dynamometer test fixture.

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/signals2/signal.hpp>

#include <fmt/format.h>

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/visitor.h"

#include "mjlib/io/now.h"
#include "mjlib/io/selector.h"

#include "mjlib/multiplex/asio_client.h"
#include "mjlib/multiplex/stream_asio_client_builder.h"

#include "mjlib/telemetry/binary_write_archive.h"
#include "mjlib/telemetry/file_writer.h"

#include "moteus/tool/line_reader.h"
#include "moteus/tool/run_for.h"

namespace base = mjlib::base;
namespace io = mjlib::io;
namespace mp = mjlib::multiplex;

namespace moteus {
namespace tool {
namespace {

constexpr int kDebugTunnel = 1;

struct TorqueTransducer {
  boost::posix_time::ptime timestamp;

  uint32_t time_code = 0;
  double torque_Nm = 0.0;
  double temperature_C = 0.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(time_code));
    a->Visit(MJ_NVP(torque_Nm));
    a->Visit(MJ_NVP(temperature_C));
  }
};

class LogRegistrar {
 public:
  LogRegistrar(mjlib::telemetry::FileWriter* file_writer)
      : file_writer_(file_writer) {}

  template <typename T>
  void Register(const std::string& name,
                boost::signals2::signal<void (const T*)>* signal) {
    const auto identifier = file_writer_->AllocateIdentifier(name);
    file_writer_->WriteSchema(
        identifier,
        mjlib::telemetry::BinarySchemaArchive::template schema<T>());
    signal->connect(std::bind(&LogRegistrar::HandleData<T>,
                              this, identifier, std::placeholders::_1));
  }

  template <typename T>
  void HandleData(mjlib::telemetry::FileWriter::Identifier identifier,
                  const T* data) {
    if (!file_writer_->IsOpen()) { return; }

    auto buffer = file_writer_->GetBuffer();
    mjlib::telemetry::BinaryWriteArchive(*buffer).Accept(data);
    file_writer_->WriteData({}, identifier, std::move(buffer));
  }

 private:
  mjlib::telemetry::FileWriter* const file_writer_;
};

std::string StringValue(const std::string& line_in) {
  const auto line = boost::trim_copy(line_in);
  std::size_t pos = line.find(' ');
  if (pos == std::string::npos) { return ""; }
  return line.substr(pos + 1);
}

struct Options {
  int fixture_id = 32;
  int dut_id = 1;
  std::string torque_transducer;

  bool verbose = false;
  std::string log;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(fixture_id));
    a->Visit(MJ_NVP(dut_id));
    a->Visit(MJ_NVP(torque_transducer));
    a->Visit(MJ_NVP(verbose));
    a->Visit(MJ_NVP(log));
  }
};

class Controller {
 public:
  Controller(const std::string& log_prefix,
             mjlib::telemetry::FileWriter* file_writer,
             mjlib::io::SharedStream stream,
             const Options& options)
      : log_prefix_(log_prefix),
        stream_(stream),
        file_writer_(file_writer),
        options_(options) {}

  boost::asio::awaitable<void> SomethingReceived() {
    auto operation = [this](io::ErrorCallback callback) {
      BOOST_ASSERT(!received_callback_);
      received_callback_ = std::move(callback);
    };

    co_await async_initiate<
      decltype(boost::asio::use_awaitable),
      void(boost::system::error_code)>(operation, boost::asio::use_awaitable);
  }

  boost::asio::awaitable<void> Start() {
    co_await WriteMessage("tel stop");

    char buf[4096] = {};

    // Discard anything we receive for some amount of time.
    co_await RunFor(
        stream_->get_executor(),
        *stream_,
        [&]() -> boost::asio::awaitable<bool> {
          boost::system::error_code ec;
          co_await boost::asio::async_read(
              *stream_,
              boost::asio::buffer(buf),
              boost::asio::redirect_error(boost::asio::use_awaitable, ec));
          // We just ignore ec.
          co_return false;
        },
        mjlib::io::Now(stream_->get_executor().context()) +
        boost::posix_time::milliseconds(300));

    fmt::print("Done flushing\n");

    boost::asio::co_spawn(
        stream_->get_executor(),
        std::bind(&Controller::Run, this),
        [](std::exception_ptr ptr) {
          if (ptr) {
            std::rethrow_exception(ptr);
          }
        });

    std::vector<std::string> names = {
      "servo_stats",
      "servo_control",
      "servo_cmd",
      "git",
    };

    for (const auto& name : names) {
      co_await WriteMessage(fmt::format("tel schema {}", name));
      co_await WriteMessage(fmt::format("tel rate {} 50", name));

      // Now wait for this to have data.
      while (true) {
        co_await SomethingReceived();
        if (log_data_.count(name) != 0) { break; }
      }
    }

    co_return;
  }

  boost::asio::awaitable<void> WriteMessage(const std::string& message) {
    if (options_.verbose) {
      fmt::print("> {}\n", message);
    }
    co_await boost::asio::async_write(
        *stream_,
        boost::asio::buffer(message + "\n"),
        boost::asio::use_awaitable);
    co_return;
  }

  boost::asio::awaitable<void> Run() {
    fmt::print("{} Run()\n", log_prefix_);
    // Read from the stream, barfing on errors, and logging any binary
    // data that comes our way.
    while (true) {
      const auto maybe_line = co_await reader_.ReadLine();

      if (!maybe_line) {
        // We had a timeout, this should only happen here on error.
        throw mjlib::base::system_error::einval("Timeout");
      }
      const auto line = *maybe_line;
      if (boost::starts_with(line, "OK")) {
        // We ignore these.
      } else if (boost::starts_with(line, "ERR")) {
        // This causes us to terminate immediately.
        throw base::system_error::einval(line);
      } else if (boost::starts_with(line, "schema ")) {
        const auto name = StringValue(line);

        const auto ident = file_writer_->AllocateIdentifier(log_prefix_ + name);
        log_ids_[name] = ident;
        file_writer_->WriteSchema(ident, co_await reader_.ReadBinaryBlob());
      } else if (boost::starts_with(line, "emit ")) {
        const auto name = StringValue(line);
        if (log_ids_.count(name)) {
          const auto ident = log_ids_.at(name);
          log_data_.insert(name);
          file_writer_->WriteData({}, ident, co_await reader_.ReadBinaryBlob());
        }
      } else {
        fmt::print("Ignoring unknown line: {}\n", line);
      }

      if (received_callback_) {
        boost::asio::post(
            stream_->get_executor(),
            std::bind(std::move(received_callback_), mjlib::base::error_code()));
      }
    }

    co_return;
  }

  mjlib::io::AsyncStream& stream() { return *stream_; }

 private:
  const std::string log_prefix_;
  mjlib::io::SharedStream stream_;
  mjlib::telemetry::FileWriter* const file_writer_;
  const Options& options_;

  LineReader reader_{stream_.get(), [&]() {
      return LineReader::Options().set_verbose(options_.verbose);
    }()};

  std::map<std::string, uint64_t> log_ids_;
  std::set<std::string> log_data_;

  io::ErrorCallback received_callback_;
};

class Application {
 public:
  Application(boost::asio::io_context& context,
              io::Selector<mp::AsioClient>* selector,
              const Options& options)
      : executor_(context.get_executor()),
        client_selector_(selector),
        options_(options),
        file_writer_() {}

  void Start() {
    if (!options_.log.empty()) {
      file_writer_.Open(options_.log);
    }
    client_selector_->AsyncStart(
        std::bind(&Application::HandleClient, this, std::placeholders::_1));
  }

  void HandleClient(const base::error_code& ec) {
    base::FailIf(ec);

    client_ = client_selector_->selected();

    boost::asio::co_spawn(
        executor_,
        std::bind(&Application::Task, this),
        [](std::exception_ptr ptr) {
          if (ptr) {
            std::rethrow_exception(ptr);
          }
          std::exit(0);
        });
  }

  boost::asio::awaitable<void> Task() {
    co_await Init();
    co_return;
  }

  boost::asio::awaitable<void> Init() {
    // First, connect to our two devices.
    fixture_.emplace(
        "fixture_",
        &file_writer_,
        client_->MakeTunnel(options_.fixture_id, kDebugTunnel, {}),
        options_);
    dut_.emplace(
        "dut_",
        &file_writer_,
        client_->MakeTunnel(options_.dut_id, kDebugTunnel, {}),
        options_);

    const auto expiration =
        mjlib::io::Now(executor_.context()) +
        boost::posix_time::seconds(3);

    co_await RunFor(
        executor_,
        fixture_->stream(),
        [&]() -> boost::asio::awaitable<bool> {
          co_await fixture_->Start();
          co_await dut_->Start();
          co_return false;
        },
        expiration);

    // Then the torque transducer.
    log_registrar_.Register("torque", &torque_signal_);
    boost::asio::co_spawn(
        executor_,
        std::bind(&Application::ReadTorque, this),
        [](std::exception_ptr ptr) {
          if (ptr) {
            std::rethrow_exception(ptr);
          }
        });

    // Then the current sense.

    // Give things a little bit to wait.
    boost::asio::deadline_timer timer(executor_);
    timer.expires_from_now(boost::posix_time::milliseconds(500));
    co_await timer.async_wait(boost::asio::use_awaitable);

    co_return;
  }

  boost::asio::awaitable<void> ReadTorque() {
    boost::asio::serial_port port(executor_, options_.torque_transducer);

    boost::asio::streambuf streambuf;
    std::istream istr(&streambuf);
    while (true) {
      co_await boost::asio::async_read_until(
          port,
          streambuf,
          "\n",
          boost::asio::use_awaitable);

      std::string line;
      std::getline(istr, line);
      EmitTorque(line);
    }
  }

  void EmitTorque(const std::string& line) {
    std::vector<std::string> fields;
    boost::split(fields, line, boost::is_any_of(","));
    if (fields.size() < 4) { return; }
    TorqueTransducer data;
    try {
      data.timestamp = mjlib::io::Now(executor_.context());
      data.time_code = std::stol(fields.at(0));
      data.torque_Nm = std::stod(fields.at(1));
      data.temperature_C = std::stod(fields.at(3));
    } catch (std::runtime_error& e) {
      fmt::print("Ignoring torque data: '{}': {}", line, e.what());
      // Ignore.
      return;
    }
    torque_signal_(&data);
  }

  boost::asio::any_io_executor executor_;
  boost::asio::executor_work_guard<boost::asio::any_io_executor> guard_{executor_};
  io::Selector<mp::AsioClient>* const client_selector_;
  const Options& options_;

  mp::AsioClient* client_ = nullptr;

  mjlib::telemetry::FileWriter file_writer_;
  LogRegistrar log_registrar_{&file_writer_};

  std::optional<Controller> fixture_;
  std::optional<Controller> dut_;

  boost::signals2::signal<void (const TorqueTransducer*)> torque_signal_;
};

int do_main(int argc, char** argv) {
  boost::asio::io_context context;

  io::Selector<mp::AsioClient> default_client_selector{
    context.get_executor(), "client_type"};

  mp::StreamAsioClientBuilder::Options default_stream_options;
  default_stream_options.stream.type = io::StreamFactory::Type::kSerial;
  default_stream_options.stream.serial_port = "/dev/fdcanusb";
  // If the baud rate does matter, 3mbit is a good one.
  default_stream_options.stream.serial_baud = 3000000;

  default_client_selector.Register<mp::StreamAsioClientBuilder>(
      "stream", default_stream_options);
  default_client_selector.set_default("stream");

  Options options;

  auto group = mjlib::base::ClippArchive().Accept(&options).group();
  group.merge(clipp::with_prefix(
                  "client.", default_client_selector.program_options()));

  mjlib::base::ClippParse(argc, argv, group);

  Application application{context, &default_client_selector, options};
  application.Start();
  context.run();

  return 0;
}

}
}
}

int main(int argc, char** argv) {
  return ::moteus::tool::do_main(argc, argv);
}
