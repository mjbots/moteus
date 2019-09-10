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

#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/program_options.hpp>

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/inplace_function.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/io/deadline_timer.h"
#include "mjlib/io/stream_copy.h"
#include "mjlib/io/stream_factory.h"
#include "mjlib/multiplex/asio_client.h"

namespace pl = std::placeholders;
namespace po = boost::program_options;
namespace base = mjlib::base;
namespace io = mjlib::io;
namespace mp = mjlib::multiplex;

namespace moteus {

namespace {
constexpr int kDebugTunnel = 1;

struct Options {
  bool stop = false;
  bool dump_config = false;
  bool console = false;

  std::vector<std::string> targets;
  io::StreamFactory::Options stream_options;
};

std::vector<int> ExpandTargets(const std::vector<std::string>& targets) {
  std::set<int> result;

  for (const auto& item : targets) {
    std::vector<std::string> fields;
    boost::split(fields, item, boost::is_any_of(","));
    for (const auto& field : fields) {
      if (field.find('-') != std::string::npos) {
        std::vector<std::string> first_last;
        boost::split(first_last, field, boost::is_any_of("-"));
        for (int i = boost::lexical_cast<int>(first_last.at(0));
             i <= boost::lexical_cast<int>(first_last.at(1));
             i++) {
          result.insert(i);
        }
      } else {
        result.insert(boost::lexical_cast<int>(field));
      }
    }
  }

  std::vector<int> sorted;
  std::copy(result.begin(), result.end(), std::back_inserter(sorted));
  return sorted;
}

class Runner {
 public:
  Runner(boost::asio::io_service& service, const Options& options)
      : service_(service),
        targets_(ExpandTargets(options.targets)),
        options_(options) {}

  void Start() {
    factory_.AsyncCreate(
        options_.stream_options,
        std::bind(&Runner::HandleStream, this, pl::_1, pl::_2));
  }

  void HandleStream(const base::error_code& ec, io::SharedStream stream) {
    base::FailIf(ec);

    stream_ = stream;
    client_.emplace(stream_.get());
    FindTargets();
  }

  void FindTargets() {
    if (!targets_.empty()) {
      StartActions();
      return;
    }

    std::cout << "Scanning for available devices:\n";
    std::vector<int> to_scan;
    for (int i = 1; i < 16; i++) { to_scan.push_back(i); }
    FindTarget(to_scan);
  }

  void StartActions() {
    auto copy = targets_;

    // For now, we only allow a single command.
    const int command_count = [&]() {
      return
      (options_.stop ? 1 : 0) +
      (options_.dump_config ? 1 : 0) +
      (options_.console ? 1 : 0);
    }();

    if (command_count > 1) {
      std::cerr << "More than one command specified!\n";
      std::exit(1);
    }

    // Some actions can only be run with a single target.
    const bool single_target = [&]() {
      if (options_.console) { return true; }
      return false;
    }();

    if (single_target) {
      if (targets_.size() != 1) {
        std::cerr <<
            "A command which requires a single target has more or less\n";
        std::exit(1);
      }
    }

    StartRemainingActions(copy);
  }

  void StartRemainingActions(std::vector<int> remaining) {
    if (remaining.empty()) {
      // We're done.
      service_.stop();
      return;
    }

    auto this_id = remaining.back();
    remaining.pop_back();

    StartAction(this_id, remaining);
  }

  struct ActionContext {
    int id = 0;
    std::vector<int> remaining;
    io::SharedStream stream;

    std::optional<io::BidirectionalStreamCopy> copy;
    io::SharedStream stdio;

    // To be used for all reading.
    boost::asio::streambuf streambuf;
  };

  void StartAction(int id, std::vector<int> remaining) {
    std::cout << fmt::format("StartCommand {}\n", id);
    auto context = std::make_shared<ActionContext>();
    context->id = id;
    context->remaining = remaining;
    context->stream = client_->MakeTunnel(id, kDebugTunnel);

    RunOneAction(context);
  }

  void RunOneAction(std::shared_ptr<ActionContext> context) {
    std::cout << fmt::format("RunOneAction {}\n", context->id);
    if (options_.stop) {
      SendAckedCommand(context, "d stop", [this, context](auto ec, auto) {
          this->FinishAction(ec, context);
        });
    } else if (options_.console) {
      io::StreamFactory::Options stdio_options;
      stdio_options.type = io::StreamFactory::Type::kStdio;
      factory_.AsyncCreate(
          stdio_options,
          [this, context](auto ec, auto stdio_stream) {
            base::FailIf(ec);
            context->stdio = stdio_stream;
            context->copy.emplace(
                service_, context->stream.get(), stdio_stream.get(),
                [context](auto ec) {
                  base::FailIf(ec);
                });
          });
    }

  }

  void FinishAction(const base::error_code& ec,
                    std::shared_ptr<ActionContext> context) {
    std::cout << fmt::format("FinishAction {}\n", context->id);
    base::FailIf(ec);

    context->stream->cancel();
    StartRemainingActions(context->remaining);
  }

  using StringCallback =
      base::inplace_function<void (const base::error_code&, std::string)>;

  void SendAckedCommand(std::shared_ptr<ActionContext> context,
                        const std::string& message,
                        StringCallback followup) {
    std::cout << fmt::format("SendAckedCommand {}\n", message);
    SendCommand(context, message, [this, context, followup](auto ec) {
        base::FailIf(ec);
        this->WaitForAck(context, followup);
      });
  }

  void SendCommand(std::shared_ptr<ActionContext> context,
                   const std::string& message,
                   io::ErrorCallback followup) {
    std::cout << fmt::format("SendCommand {}\n", message);
    auto buf = std::make_shared<std::string>(message + "\n");
    boost::asio::async_write(
        *context->stream,
        boost::asio::buffer(*buf),
        [context, buf, followup](auto ec, auto) {
          followup(ec);
        });
  }

  struct AckContext {
    std::shared_ptr<ActionContext> context;
    StringCallback followup;
    std::string result;
  };

  void WaitForAck(std::shared_ptr<ActionContext> context,
                  StringCallback followup) {
    std::cout << fmt::format("WaitForAck\n");
    auto ack_context = std::make_shared<AckContext>();
    ack_context->context = context;
    ack_context->followup = followup;

    ReadForAck(ack_context);
  }

  void ReadForAck(std::shared_ptr<AckContext> ack_context) {
    std::cout << fmt::format("ReadForAck\n");
    // Read lines until we get an OK.
    ReadLine(ack_context->context, [this, ack_context](auto ec, auto message) {
        base::FailIf(ec);

        if (boost::starts_with(message, "OK")) {
          // We're done.
          ack_context->context->stream->get_io_service().post(
              std::bind(ack_context->followup,
                        base::error_code(), ack_context->result));
        } else {
          ack_context->result += message;
          this->ReadForAck(ack_context);
        }
      });
  }

  void ReadLine(std::shared_ptr<ActionContext> context,
                StringCallback callback) {
    std::cout << fmt::format("ReadLine\n");
    boost::asio::async_read_until(
        *context->stream,
        context->streambuf,
        "\n",
        [context, callback](auto ec, auto) {
          base::FailIf(ec);
          std::ostringstream ostr;
          ostr << &context->streambuf;

          std::cout << fmt::format("got line: {}\n", ostr.str());
          context->stream->get_io_service().post(
              std::bind(callback, base::error_code(), ostr.str()));
        });
  }

 private:
  struct FindContext {
    int id = 0;
    io::SharedStream stream;
    boost::asio::streambuf streambuf;
    bool done = false;
  };

  std::string FormatTargets() const {
    std::ostringstream ostr;
    bool first = true;
    for (const auto& target : targets_) {
      if (!first) {
        ostr << ",";
      }
      first = false;
      ostr << target;
    }
    return ostr.str();
  }

  void FindTarget(std::vector<int> remaining) {
    if (remaining.empty()) {
      std::cout << fmt::format("Auto-discovered IDs: {}\n", FormatTargets());
      StartActions();
      return;
    }

    auto this_id = remaining.back();
    remaining.pop_back();

    auto context = std::make_shared<FindContext>();

    context->id = this_id;

    // Try to talk to this particular servo.
    context->stream = client_->MakeTunnel(this_id, kDebugTunnel);

    std::cout << fmt::format("trying {}\n", this_id);

    // Send a command, then wait for a reply.
    boost::asio::async_write(
        *context->stream,
        boost::asio::buffer("tel stop\n"),
        std::bind(&Runner::HandleFindTargetWrite, this, pl::_1,
                  context, remaining));
  }

  void HandleFindTargetWrite(const base::error_code& ec,
                             std::shared_ptr<FindContext> context,
                             const std::vector<int>& remaining) {
    base::FailIf(ec);


    // Now we need to read with a timeout, which asio doesn't make all
    // that easy. :(
    boost::asio::async_read_until(
        *context->stream,
        context->streambuf,
        "\n",
        std::bind(&Runner::HandleFindTargetRead, this, pl::_1,
                  context, remaining));

    timer_.expires_from_now(boost::posix_time::milliseconds(1000));
    timer_.async_wait(
        std::bind(&Runner::HandleFindTargetTimeout, this, pl::_1,
                  context, remaining));
  }

  void HandleFindTargetRead(const base::error_code& ec,
                            std::shared_ptr<FindContext> context,
                            const std::vector<int>& remaining) {
    std::cout << fmt::format("read done: {}\n", context->id);
    if (context->done) { return; }
    base::FailIf(ec);

    timer_.cancel();
    context->done = true;  // so our timeout won't do anything

    std::istream istr(&context->streambuf);
    std::string msg;
    istr >> msg;
    if (msg.substr(0, 2) == "OK") {
      targets_.push_back(context->id);
    }

    FindTarget(remaining);
  }

  void HandleFindTargetTimeout(const base::error_code& ec,
                               std::shared_ptr<FindContext> context,
                               const std::vector<int>& remaining) {
    std::cout << fmt::format("timeout id {}\n", context->id);

    if (context->done) { return; }
    base::FailIf(ec);

    context->done = true;  // so our read can't do anything
    context->stream->cancel();

    // Just switch to working on the next one.
    FindTarget(remaining);
  }

  boost::asio::io_service& service_;
  std::vector<int> targets_;
  const Options options_;

  io::DeadlineTimer timer_{service_};
  io::StreamFactory factory_{service_};
  io::SharedStream stream_;
  io::SharedStream stdio_;
  std::optional<mp::AsioClient> client_;
};
}

int moteus_tool_main(int argc, char** argv) {
  boost::asio::io_service service;

  po::options_description desc("Allowable options");

  Options options;

  desc.add_options()
      ("help,h", "display_usage")
      ("targets,t", po::value(&options.targets),
       "destination address(es) (default: autodiscover)")

      ("stop", po::bool_switch(&options.stop),
       "command the servos to stop")
      ("console,c", po::bool_switch(&options.console),
       "create a serial console")
      ("dump-config", po::bool_switch(&options.dump_config),
       "emit all configuration to the console")
      ;

  base::ProgramOptionsArchive(&desc).Accept(&options.stream_options);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc;
    return 0;
  }

  Runner runner(service, options);
  runner.Start();
  service.run();

  return 0;
}

}
