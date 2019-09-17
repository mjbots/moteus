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

#include <algorithm>
#include <cctype>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/program_options.hpp>

#include <fmt/format.h>

#include <elfio/elfio.hpp>

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/base/system_error.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/io/async_sequence.h"
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
namespace tool {

namespace {

constexpr int kMaxFlashBlockSize = 32;

std::string Hexify(const std::string& data) {
  std::ostringstream ostr;
  for (char c : data) {
    ostr << fmt::format("{:02x}", static_cast<uint8_t>(c));
  }
  return ostr.str();
}

struct ElfMapping {
  int64_t virtual_address = 0;
  int64_t physical_address = 0;
  int64_t size = 0;
};

class ElfMappings {
 public:
  ElfMappings(const ELFIO::elfio& reader) {
    for (const auto& segment : reader.segments) {
      ElfMapping mapping;
      mapping.virtual_address = segment->get_virtual_address();
      mapping.physical_address = segment->get_physical_address();
      mapping.size = segment->get_memory_size();
      mappings_.push_back(mapping);
    }
  }

  int64_t LogicalToPhysical(int64_t address, int64_t size) {
    for (const auto& mapping : mappings_) {
      if (address >= mapping.virtual_address &&
          (address + size) <= (mapping.virtual_address + mapping.size)) {
        return address - mapping.virtual_address + mapping.physical_address;
      }
    }
    throw base::system_error::einval(
        fmt::format("no mapping for {:x}", address));
  }

  std::vector<ElfMapping> mappings_;
};


struct ElfData {
  // Blocks of data associated with a given address.
  std::map<int64_t, std::string> data;
};

ElfData ReadElf(const std::string& filename,
                std::set<std::string> sections) {
  ELFIO::elfio reader;
  if (!reader.load(filename)) {
    throw base::system_error::einval("Could not load ELF file: " + filename);
  }

  ElfMappings mappings(reader);

  ElfData result;

  for (const auto& section : reader.sections) {
    const auto name = section->get_name();
    if (sections.count(name) == 0) { continue; }

    sections.erase(name);

    auto make_data = [](auto section) {
      return (section->get_data() == nullptr) ?
        std::string(static_cast<char>(0), section->get_size()) :
        std::string(section->get_data(), section->get_size());
    };

    const auto physical_address =
        mappings.LogicalToPhysical(section->get_address(), section->get_size());
    result.data[physical_address] = make_data(section);
  }

  if (sections.size()) {
    throw base::system_error::einval("Some sections not found");
  }

  return result;
}

constexpr int kDebugTunnel = 1;

struct Options {
  bool stop = false;
  bool dump_config = false;
  bool console = false;
  std::string flash;

  bool verbose = false;
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
  Runner(boost::asio::io_context& service, const Options& options)
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
    for (int i = 1; i < 127; i++) { to_scan.push_back(i); }
    FindTarget(to_scan);
  }

  void StartActions() {
    auto copy = targets_;

    // For now, we only allow a single command.
    const int command_count = [&]() {
      return
      (options_.stop ? 1 : 0) +
      (options_.dump_config ? 1 : 0) +
      (options_.console ? 1 : 0) +
      (!options_.flash.empty() ? 1 : 0);
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
      std::cout << "All actions complete\n";
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
    if (targets_.size() > 1) {
      std::cout << fmt::format("Target: {}\n", id);
    }
    auto context = std::make_shared<ActionContext>();
    context->id = id;
    context->remaining = remaining;
    mp::AsioClient::TunnelOptions tunnel_options;

    // Older versions of the bootloader could fail during long running
    // flash operations at the default 10ms poll rate.  Set it to 100
    // for now.
    tunnel_options.poll_rate = boost::posix_time::milliseconds(100);

    context->stream = client_->MakeTunnel(id, kDebugTunnel, tunnel_options);

    RunOneAction(context);
  }

  void RunOneAction(std::shared_ptr<ActionContext> context) {
    if (options_.stop) {
      SendAckedCommand(context, "d stop", [this, context](auto ec, auto) {
          this->FinishAction(ec, context);
        });
    } else if (options_.dump_config) {
      SendAckedCommand(context, "conf enumerate",
                       [this, context](auto ec, auto result) {
                         std::cout << result;
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
    } else if (!options_.flash.empty()) {
      DoFlash(context);
    }
  }

  void FinishAction(const base::error_code& ec,
                    std::shared_ptr<ActionContext> context) {
    base::FailIf(ec);

    context->stream->cancel();
    StartRemainingActions(context->remaining);
  }

  using StringCallback =
      std::function<void (const base::error_code&, std::string)>;

  struct CommandOptions {
    double retry_timeout = 0.3;
    int max_retries = 4;
    bool allow_any_response = false;

    CommandOptions() {}

    CommandOptions& set_retry_timeout(double value) {
      retry_timeout = value;
      return *this;
    }

    CommandOptions& set_max_retries(int value) {
      max_retries = value;
      return *this;
    }

    CommandOptions& set_allow_any_response(bool value) {
      allow_any_response = value;
      return *this;
    }
  };

  struct SendAckContext {
    std::shared_ptr<ActionContext> context;
    StringCallback followup;
    std::string message;
    CommandOptions options;

    bool allow_any_response = false;
    int retry_count = 0;
    bool done = false;
  };

  void SendAckedCommand(std::shared_ptr<ActionContext> context,
                        const std::string& message,
                        StringCallback followup,
                        CommandOptions command_options = CommandOptions()) {
    auto sc = std::make_shared<SendAckContext>();
    sc->context = context;
    sc->message = message;
    sc->options = command_options;
    sc->followup = followup;

    timer_.expires_from_now(
        base::ConvertSecondsToDuration(command_options.retry_timeout));
    timer_.async_wait(std::bind(&Runner::HandleSendAckedTimeout,
                                this, pl::_1, sc));

    SendCommand(sc->context, message, [this, sc](auto ec) {
        base::FailIf(ec);
        this->WaitForAck(sc->context, [this, sc](auto ec, auto message) {
            if (ec == boost::asio::error::operation_aborted) {
              // We timed out.  Try re-sending the command.
              sc->retry_count++;
              if (sc->retry_count < sc->options.max_retries) {
                SendAckedCommand(
                    sc->context, sc->message, sc->followup, sc->options);
                return;
              }
            }

            this->timer_.cancel();
            sc->done = true;
            sc->followup(ec, message);
          },
          sc->options.allow_any_response);
      });
  }

  void HandleSendAckedTimeout(const base::error_code& ec,
                              std::shared_ptr<SendAckContext> ctx) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    if (ctx->done) { return; }

    // Cancel out our existing read.
    ctx->context->stream->cancel();
  }

  void SendCommand(std::shared_ptr<ActionContext> context,
                   const std::string& message,
                   io::ErrorCallback followup) {
    if (options_.verbose) {
      std::cout << fmt::format(">{}\n", message);
    }
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
    bool allow_any_response = false;
    std::string result;
  };

  void WaitForAck(std::shared_ptr<ActionContext> context,
                  StringCallback followup,
                  bool allow_any_response) {
    auto ack_context = std::make_shared<AckContext>();
    ack_context->context = context;
    ack_context->followup = followup;
    ack_context->allow_any_response = allow_any_response;

    ReadForAck(ack_context);
  }

  void ReadForAck(std::shared_ptr<AckContext> ack_context) {
    // Read lines until we get an OK.
    ReadLine(ack_context->context, [this, ack_context](auto ec, auto message) {
        if (ec == boost::asio::error::operation_aborted) {
          ack_context->context->stream->get_io_service().post(
              std::bind(ack_context->followup, ec, ""));
          return;
        }
        base::FailIf(ec);

        if (ack_context->allow_any_response) {
          ack_context->result += message;
        }
        if (ack_context->allow_any_response ||
            boost::starts_with(message, "OK")) {
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
    boost::asio::async_read_until(
        *context->stream,
        context->streambuf,
        "\n",
        [this, context, callback](auto ec, auto) {
          if (ec == boost::asio::error::operation_aborted) {
            // We had a timeout.
            context->stream->get_io_service().post(
                std::bind(callback, ec, ""));
            return;
          }
          base::FailIf(ec);
          std::ostringstream ostr;
          ostr << &context->streambuf;

          if (options_.verbose) {
            std::cout << fmt::format("<{}", ostr.str());
          }

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

    // Send a command, then wait for a reply.
    boost::asio::async_write(
        *context->stream,
        boost::asio::buffer(std::string_view("tel stop\n")),
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

    timer_.expires_from_now(boost::posix_time::milliseconds(10));
    timer_.async_wait(
        std::bind(&Runner::HandleFindTargetTimeout, this, pl::_1,
                  context, remaining));
  }

  void HandleFindTargetRead(const base::error_code& ec,
                            std::shared_ptr<FindContext> context,
                            const std::vector<int>& remaining) {
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
    if (context->done) { return; }
    base::FailIf(ec);

    context->done = true;  // so our read can't do anything
    context->stream->cancel();

    // Just switch to working on the next one.
    FindTarget(remaining);
  }

  void DoFlash(std::shared_ptr<ActionContext> context) {
    // First, get our two binaries.
    const auto elf =
        ReadElf(options_.flash,
                {".text", ".ARM.extab", ".ARM.exidx",
                 ".data", ".bss", ".isr_vector"});

    auto count_bytes = [](const std::vector<ElfData>& elfs) {
      int64_t result = 0;
      for (const auto& elf: elfs) {
        for (const auto& pair : elf.data) {
          result += pair.second.size();
        }
      }
      return result;
    };

    std::cout << fmt::format("Read ELF file: {} bytes\n",
                             count_bytes({elf}));

    // If we had coroutines, this would look like:
    //
    //  write("d flash")
    //  read_response("")
    //  command("unlock")
    //  write_flash(elf)
    //  command("lock")
    //  command("reset")
    //
    // However, since we don't, that means we now have a long string
    // of callbacks in our future.

    io::AsyncSequence(service_)
        .op([this, context](auto handler) {
            SendCommand(context, "d flash", handler);
          })
        .op([this, context](auto handler) {
            ReadLine(context,
                     [handler](auto ec, auto) {
                       handler(ec);
                     });
          })
        .op([this, context](auto handler) {
            SendAckedCommand(context, "unlock",
                             [handler](auto ec, auto) {
                               handler(ec);
                             });
          })
        .op([this, context, elf](auto handler) {
            WriteFlash(context, elf, handler);
          })
        .op([this, context](auto handler) {
            SendAckedCommand(context, "lock",
                             [handler](auto ec, auto) {
                               handler(ec);
                             });
          })
        .op([this, context](auto handler) {
            SendCommand(context, "reset", handler);
          })
        .Start([this, context](auto ec) {
            base::FailIf(ec);
            FinishAction({}, context);
          });
  }

  struct FlashDataBlock {
    int64_t address = -1;
    std::string data;

    FlashDataBlock(int64_t address_in, std::string data_in)
        : address(address_in), data(data_in) {}

    FlashDataBlock() {}
  };

  struct FlashContext {
    std::shared_ptr<ActionContext> context;
    ElfData elf;
    io::ErrorCallback callback;

    bool verifying = false;
    int64_t current_address = -1;

    FlashContext(std::shared_ptr<ActionContext> context_in,
                 ElfData elf_in,
                 io::ErrorCallback callback_in)
        : context(context_in),
          elf(elf_in),
          callback(callback_in) {}

    FlashDataBlock GetNextBlock() {
      // Find the next pair which contains something greater than the
      // current address.
      for (const auto& pair : elf.data) {
        if (pair.first > current_address) {
          // This is definitely it.
          return FlashDataBlock(
              pair.first, pair.second.substr(0, kMaxFlashBlockSize));
        }
        // We might be inside a block that has more data.
        const auto end_of_this_block =
            static_cast<int64_t>(pair.first + pair.second.size());
        if (end_of_this_block > current_address) {
          return FlashDataBlock(
              current_address, pair.second.substr(
                  current_address - pair.first,
                  std::min<int>(end_of_this_block - current_address,
                                kMaxFlashBlockSize)));
        }
        // Keep looking.
      }
      return FlashDataBlock();
    }

    bool AdvanceBlock() {
      auto this_block = GetNextBlock();
      current_address = this_block.address + this_block.data.size();
      return (GetNextBlock().address < 0);
    }
  };

  void WriteFlash(std::shared_ptr<ActionContext> context,
                  const ElfData& elf,
                  io::ErrorCallback callback) {
    auto flash_context = std::make_shared<FlashContext>(context, elf, callback);
    DoNextFlashOperation(flash_context);
  }

  void DoNextFlashOperation(std::shared_ptr<FlashContext> flash_context) {
    if (!flash_context->verifying) {
      auto next_block = flash_context->GetNextBlock();
      auto cmd = fmt::format("w {:x} {}", next_block.address,
                             Hexify(next_block.data));
      SendAckedCommand(
          flash_context->context,
          cmd,
          std::bind(&Runner::HandleFlashOperation, this,
                    pl::_1, flash_context),
          CommandOptions().set_retry_timeout(5.0));
    } else {
      auto expected_block = flash_context->GetNextBlock();
      SendAckedCommand(
          flash_context->context,
          fmt::format("r {:x} {:x}",
                      expected_block.address,
                      expected_block.data.size()),
          std::bind(&Runner::HandleFlashVerifyWrite, this,
                    pl::_1, pl::_2, flash_context, expected_block),
          CommandOptions().set_allow_any_response(true));
    }
  }

  void HandleFlashVerifyWrite(base::error_code ec,
                              const std::string& message,
                              std::shared_ptr<FlashContext> flash_context,
                              const FlashDataBlock& expected) {
    if (ec) {
      ec.Append("While writing verify");
      service_.post(std::bind(flash_context->callback, ec));
      return;
    }

    std::vector<std::string> fields;
    boost::split(fields, message, boost::is_any_of(" "));
    if (fields.size() != 2) {
      ec.Append(fmt::format("verify returned wrong field count {:d} != 2",
                            fields.size()));
      HandleFlashOperation(ec, flash_context);
      return;
    }

    const auto actual_address = std::stoi(fields[0], nullptr, 16);
    if (actual_address != expected.address) {
      ec.Append(fmt::format("verify returned wrong address {:x} != {:x}",
                            actual_address, expected.address));
      HandleFlashOperation(ec, flash_context);
      return;
    }

    std::transform(fields[1].begin(), fields[1].end(),
                   fields[1].begin(),
                   [](auto c) { return std::tolower(c); });
    if (Hexify(expected.data) != fields[1]) {
      ec.Append(fmt::format("verify returned wrong data at {:x}, {} != {}",
                            expected.address,
                            fields[1], Hexify(expected.data)));
      HandleFlashOperation(ec, flash_context);
      return;
    }

    HandleFlashOperation({}, flash_context);
  }

  void HandleFlashOperation(base::error_code ec,
                            std::shared_ptr<FlashContext> flash_context) {
    if (ec) {
      ec.Append(fmt::format("While {} address {:x}",
                            flash_context->verifying ? "verifying" : "flashing",
                            flash_context->current_address));
      service_.post(std::bind(flash_context->callback, ec));
      return;
    }

    const bool done = flash_context->AdvanceBlock();
    if (!options_.verbose) {
      std::cout << fmt::format(
          "flash: {:15s}  {:08x}\r",
          flash_context->verifying ? "verifying" : "flashing",
          flash_context->current_address);
      std::cout.flush();
    }
    if (!flash_context->verifying) {
      if (done) {
        flash_context->current_address = -1;
        flash_context->verifying = true;
      }
      DoNextFlashOperation(flash_context);
    } else {
      if (done) {
        if (!options_.verbose) {
          std::cout << "\n";
        }
        service_.post(std::bind(flash_context->callback, base::error_code()));
      } else {
        DoNextFlashOperation(flash_context);
      }
    }
  }

  boost::asio::io_context& service_;
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
  boost::asio::io_context service;

  po::options_description desc("Allowable options");

  Options options;

  desc.add_options()
      ("help,h", "display_usage")
      ("targets,t", po::value(&options.targets),
       "destination address(es) (default: autodiscover)")
      ("verbose,v", po::bool_switch(&options.verbose),
       "emit all commands sent to and from the device")

      ("stop", po::bool_switch(&options.stop),
       "command the servos to stop")
      ("console,c", po::bool_switch(&options.console),
       "create a serial console")
      ("dump-config", po::bool_switch(&options.dump_config),
       "emit all configuration to the console")
      ("flash", po::value(&options.flash),
       "write the given elf file to flash")
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
}
