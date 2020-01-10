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

#include "moteus/tool/moteus_tool.h"

#include <algorithm>
#include <cctype>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

#include <fmt/format.h>

#include <elfio/elfio.hpp>

#include "mjlib/base/clipp.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/system_error.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/io/async_sequence.h"
#include "mjlib/io/deadline_timer.h"
#include "mjlib/io/stream_copy.h"
#include "mjlib/io/stream_factory.h"
#include "mjlib/multiplex/stream_asio_client_builder.h"

namespace pl = std::placeholders;
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
  Runner(const boost::asio::executor& executor,
         io::Selector<mp::AsioClient>* selector,
         const Options& options)
      : executor_(executor),
        client_selector_(selector),
        targets_(ExpandTargets(options.targets)),
        options_(options) {}

  void Start() {
    client_selector_->AsyncStart(
        std::bind(&Runner::HandleClient, this, std::placeholders::_1));
  }

  void HandleClient(const base::error_code& ec) {
    base::FailIf(ec);

    client_ = client_selector_->selected();

    boost::asio::co_spawn(
        executor_,
        std::bind(&Runner::Task, this),
        [](std::exception_ptr ptr) {
          if (ptr) {
            std::rethrow_exception(ptr);
          }
          std::exit(0);
        });
  }

  boost::asio::awaitable<bool> FindTarget(int target_id) {
    auto stream = client_->MakeTunnel(target_id, kDebugTunnel);

    auto maybe_result = co_await Command(*stream, "tel stop");
    co_return (!!maybe_result && *maybe_result == "OK");
  }

  boost::asio::awaitable<std::vector<int>> FindTargets() {
    if (!targets_.empty()) {
      co_return targets_;
    }

    std::vector<int> result;
    for (int i = 1; i < 127; i++) {
      const bool found = co_await FindTarget(i);
      if (found) { result.push_back(i); }
    }

    co_return result;
  }

  boost::asio::awaitable<void> Task() {
    targets_ = co_await FindTargets();

    co_await RunActions();
  }

  boost::asio::awaitable<void> RunActions() {
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

    for (int target_id : targets_) {
      if (targets_.size() > 1) {
        std::cout << fmt::format("Target: {}\n", target_id);
      }
      co_await RunAction(target_id);
    }
  }

  boost::asio::awaitable<void> RunAction(int id) {
    mp::AsioClient::TunnelOptions tunnel_options;

    auto stream = client_->MakeTunnel(id, kDebugTunnel, tunnel_options);

    if (options_.stop) {
      co_await Command(*stream, "d stop");
    } else if (options_.dump_config) {
      const auto maybe_result = co_await Command(*stream, "conf enumerate");
      if (maybe_result) {
        std::cout << *maybe_result;
      }
    } else if (options_.console) {
      auto start_factory = [this, stream](io::ErrorCallback callback) {
        io::StreamFactory::Options stdio_options;
        stdio_options.type = io::StreamFactory::Type::kStdio;
        this->factory_.AsyncCreate(
            stdio_options,
            [this, stream, callback = std::move(callback)](
                auto ec, auto stdio_stream) mutable {
              base::FailIf(ec);
              this->copy_.emplace(
                  this->executor_,
                  stream.get(), stdio_stream.get(),
                  [callback = std::move(callback),
                   stdio_stream](auto ec) mutable {
                    if (!ec || ec == boost::asio::error::eof) {
                      callback(boost::system::error_code());
                      return;
                    }
                    base::FailIf(ec);

                  });
            });
      };

      // Coop the boost::asio machinery to turn a callback based
      // asynchronous operation into a coroutine based one.
      co_await async_initiate<
        decltype(boost::asio::use_awaitable),
        void(boost::system::error_code)>(
            start_factory,
            boost::asio::use_awaitable);

    } else if (!options_.flash.empty()) {
      co_await DoFlash(*stream);
    }
  }

 private:
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

  struct CommandOptions {
    int retry_count = 3;
    double retry_timeout = 0.3;
    int max_retries = 4;
    bool allow_any_response = false;

    CommandOptions() {}

    CommandOptions& set_retry_count(int value) {
      retry_count = value;
      return *this;
    }

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

  boost::asio::awaitable<std::optional<std::string>> Command(
      io::AsyncStream& stream,
      const std::string& message,
      CommandOptions command_options = CommandOptions()) {

    for (int i = 0; i < command_options.retry_count; i++) {
      // TODO(jpieper): Actually handle timeout.
      co_await WriteMessage(stream, message);

      const auto result = co_await ReadUntilOK(stream);
      co_return result;
    }

    co_return std::optional<std::string>();
  }

  boost::asio::awaitable<std::string> ReadUntilOK(io::AsyncStream& stream) {
    std::string result;
    while (true) {
      auto line = co_await ReadLine(stream);
      if (line.substr(0, 2) == "OK") { co_return result; }
      result += line;
    }
  }

  boost::asio::awaitable<void> WriteMessage(io::AsyncStream& stream,
                                            const std::string& message) {
    co_await boost::asio::async_write(
        stream,
        boost::asio::buffer(message + "\n"),
        boost::asio::use_awaitable);
    co_return;
  }

  boost::asio::awaitable<std::string> ReadLine(io::AsyncStream& stream) {
    boost::asio::streambuf streambuf;
    co_await boost::asio::async_read_until(
        stream,
        streambuf,
        '\n',
        boost::asio::use_awaitable);
    std::ostringstream ostr;
    ostr << &streambuf;
    co_return ostr.str();
  }

  boost::asio::awaitable<void> DoFlash(io::AsyncStream& stream) {
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

    co_await WriteMessage(stream, "d flash");
    co_await ReadLine(stream);
    co_await Command(stream, "unlock");
    co_await WriteFlash(stream, elf);
    co_await Command(stream, "lock");
    co_await Command(stream, "reset");
  }

  struct FlashDataBlock {
    int64_t address = -1;
    std::string data;

    FlashDataBlock(int64_t address_in, std::string data_in)
        : address(address_in), data(data_in) {}

    FlashDataBlock() {}
  };

  struct FlashContext {
    ElfData elf;
    int64_t current_address = -1;

    FlashContext(ElfData elf_in) : elf(elf_in) {}

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

  boost::asio::awaitable<void> WriteFlash(
      io::AsyncStream& stream, const ElfData& elf) {
    auto emit_progress = [&](const auto& ctx, const std::string& type) {
      if (!options_.verbose) {
        std::cout << fmt::format(
            "flash: {:15s}  {:08x}\r",
            type,
            ctx.current_address);
        std::cout.flush();
      }
    };

    {
      FlashContext write_ctx(elf);
      for (;;) {
        const auto next_block = write_ctx.GetNextBlock();
        const auto cmd = fmt::format(
            "w {:x} {}", next_block.address,
            Hexify(next_block.data));
        co_await Command(stream, cmd, CommandOptions().set_retry_timeout(5.0));
        emit_progress(write_ctx, "flashing");
        const bool done = write_ctx.AdvanceBlock();
        if (done) { break; }
      }
    }

    {
      FlashContext verify_ctx(elf);
      for (;;) {
        const auto expected_block = verify_ctx.GetNextBlock();
        const auto cmd =
            fmt::format("r {:x} {:x}",
                        expected_block.address,
                        expected_block.data.size());
        const auto maybe_result = co_await Command(stream, cmd);
        emit_progress(verify_ctx, "verifying");
        base::system_error::throw_if(
            !maybe_result, fmt::format("no response verifying address {:x}",
                                       expected_block.address));
        VerifyBlocks(expected_block, *maybe_result);
        const bool done = verify_ctx.AdvanceBlock();
        if (done) { break; }
      }
    }
  }

  void VerifyBlocks(const FlashDataBlock& expected,
                    const std::string& message) {
    std::vector<std::string> fields;
    boost::split(fields, message, boost::is_any_of(" "));
    base::system_error::throw_if(
        fields.size() != 2,
        fmt::format("verify returned wrong field count {:d} != 2",
                    fields.size()));

    const auto actual_address = std::stoi(fields[0], nullptr, 16);
    base::system_error::throw_if(
        actual_address != expected.address,
        fmt::format("verify returned wrong address {:x} != {:x}",
                    actual_address, expected.address));

    std::transform(fields[1].begin(), fields[1].end(),
                   fields[1].begin(),
                   [](auto c) { return std::tolower(c); });
    base::system_error::throw_if(
        Hexify(expected.data) != fields[1],
        fmt::format("verify returned wrong data at {:x}, {} != {}",
                    expected.address,
                    fields[1], Hexify(expected.data)));
  }

  boost::asio::executor executor_;
  boost::asio::executor_work_guard<boost::asio::executor> guard_{executor_};
  io::Selector<mp::AsioClient>* const client_selector_;
  std::vector<int> targets_;
  const Options options_;

  io::DeadlineTimer timer_{executor_};
  io::StreamFactory factory_{executor_};
  io::SharedStream stdio_;
  mp::AsioClient* client_ = nullptr;
  std::optional<io::BidirectionalStreamCopy> copy_;
};

}

int moteus_tool_main(boost::asio::io_context& context,
                     int argc, char** argv,
                     io::Selector<mp::AsioClient>* selector) {
  io::Selector<mp::AsioClient> default_client_selector{
    context.get_executor(), "client_type"};
  if (selector == nullptr) {
    default_client_selector.Register<mp::StreamAsioClientBuilder>("stream");
    default_client_selector.set_default("stream");
    selector = &default_client_selector;
  }

  Options options;

  auto group = clipp::group(
      clipp::repeatable(
          (clipp::option("t", "target") &
           clipp::integer("TGT", options.targets)) %
          "one or more target devices (default: autodiscover)"),
      clipp::option("v", "verbose").set(options.verbose).doc(
          "emit all commands sent to and from the device"),
      clipp::option("s", "stop").set(options.stop).doc(
          "command the servos to stop"),
      clipp::option("c", "console").set(options.console).doc(
          "create a serial console"),
      clipp::option("dump-config").set(options.dump_config).doc(
          "emit all configuration to the console"),
      clipp::option("flash") & clipp::value("file", options.flash).doc(
          "write the given elf file to flash")
  );
  group.merge(clipp::with_prefix("client.", selector->program_options()));

  mjlib::base::ClippParse(argc, argv, group);

  Runner runner(context.get_executor(), selector, options);
  runner.Start();
  context.run();

  return 0;
}

}
}
