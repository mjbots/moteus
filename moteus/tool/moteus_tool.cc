// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <fmt/format.h>

#include <Eigen/Dense>

#include <elfio/elfio.hpp>

#include "mjlib/base/clipp.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/json5_write_archive.h"
#include "mjlib/base/system_error.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/io/async_sequence.h"
#include "mjlib/io/deadline_timer.h"
#include "mjlib/io/now.h"
#include "mjlib/io/stream_copy.h"
#include "mjlib/io/stream_factory.h"
#include "mjlib/multiplex/stream_asio_client_builder.h"

#include "moteus/tool/calibrate.h"
#include "moteus/tool/run_for.h"

namespace pl = std::placeholders;
namespace fs = boost::filesystem;
namespace base = mjlib::base;
namespace io = mjlib::io;
namespace mp = mjlib::multiplex;

namespace moteus {
namespace tool {

namespace {

constexpr int kMaxFlashBlockSize = 32;
constexpr double kPi = 3.141592653589793f;

std::string GetLogDirectory() {
  // First, look for a dedicated moteus calibration directory.
  const char* moteus_cal_dir = getenv("MOTEUS_CAL_DIR");
  if (moteus_cal_dir != nullptr) {
    return moteus_cal_dir;
  }

  const char* home = getenv("HOME");
  if (home != NULL) {
    fs::path maybe_dir = fs::path(home) / "moteus-cal";
    if (fs::exists(maybe_dir)) {
      return maybe_dir.native();
    }
  }

  return ".";
}

std::string Hexify(const std::string& data) {
  std::ostringstream ostr;
  for (char c : data) {
    ostr << fmt::format("{:02x}", static_cast<uint8_t>(c));
  }
  return ostr.str();
}

std::string MakeGitHash(const std::map<std::string, std::string>& data) {
  std::string result;
  for (int i = 0; i < 20; i++) {
    result += fmt::format(
        "{:02x}", std::stoi(data.at(fmt::format("git.hash.{}", i))));
  }
  return result;
}

struct ElfMapping {
  int64_t virtual_address = 0;
  int64_t physical_address = 0;
  int64_t size = 0;
};

struct DeviceInfo {
  std::string serial_number;
  std::string model;
  std::string git_hash;
  bool git_dirty = false;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(serial_number));
    a->Visit(MJ_NVP(model));
    a->Visit(MJ_NVP(git_hash));
    a->Visit(MJ_NVP(git_dirty));
  }
};

struct CalibrationReport {
  boost::posix_time::ptime timestamp;

  DeviceInfo device_info;

  CalibrationResult calibration;

  double winding_resistance = 0.0;
  double v_per_hz = 0.0;
  double kv = 0.0;
  double unwrapped_position_scale = 0.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(device_info));
    a->Visit(MJ_NVP(calibration));
    a->Visit(MJ_NVP(winding_resistance));
    a->Visit(MJ_NVP(v_per_hz));
    a->Visit(MJ_NVP(kv));
    a->Visit(MJ_NVP(unwrapped_position_scale));
  }
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
  bool info = false;
  bool dump_config = false;
  bool console = false;
  std::string flash;

  bool verbose = false;
  std::vector<std::string> targets;

  bool calibrate = false;
  double calibration_power = 0.40;
  double calibration_speed = 1.0;
  double kv_speed = 12.0;
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

    auto maybe_result = co_await Command(
        *stream, "tel stop",
        CommandOptions().set_retry_timeout(0.02).set_retry_count(1));

    co_return !!maybe_result;
  }

  boost::asio::awaitable<std::vector<int>> FindTargets() {
    if (!targets_.empty()) {
      co_return targets_;
    }

    discovered_ = true;

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
      (options_.info ? 1 : 0) +
      (options_.dump_config ? 1 : 0) +
      (options_.console ? 1 : 0) +
      (!options_.flash.empty() ? 1 : 0) +
      (options_.calibrate ? 1 : 0)
      ;
    }();

    if (command_count > 1) {
      std::cerr << "More than one command specified!\n";
      std::exit(1);
    }

    // Some actions can only be run with a single target.
    const bool single_target = [&]() {
      if (options_.console) { return true; }
      if (options_.calibrate) { return true; }
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
      if (discovered_ || targets_.size() > 1) {
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
    } else if (options_.info) {
      co_await DoInfo(*stream);
    } else if (options_.dump_config) {
      const auto maybe_result = co_await Command(*stream, "conf enumerate");
      if (maybe_result) {
        std::cout << *maybe_result;
      }
    } else if (options_.console) {
      co_await DoConsole(stream);
    } else if (!options_.flash.empty()) {
      co_await DoFlash(*stream);
    } else if (options_.calibrate) {
      co_await DoCalibrate(*stream);
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

    if (options_.verbose) {
      std::cout << fmt::format("> {}\n", message);
    }

    for (int i = 0; i < command_options.retry_count; i++) {
      co_await WriteMessage(stream, message);

      const auto expiration = io::Now(executor_.context()) +
          base::ConvertSecondsToDuration(command_options.retry_timeout);

      const auto maybe_result = co_await RunFor(
          executor_,
          stream,
          [&]() -> boost::asio::awaitable<std::optional<std::string>> {
            if (command_options.allow_any_response) {
              co_return co_await ReadLine(stream);
            } else {
              co_return co_await ReadUntilOK(stream);
            }
          },
          expiration);

      if (!!maybe_result) {
        co_return *maybe_result;
      }
    }

    co_return std::optional<std::string>();
  }

  boost::asio::awaitable<std::optional<std::string>>
  ReadUntilOK(io::AsyncStream& stream) {
    std::string result;
    while (true) {
      auto maybe_line = co_await ReadLine(stream);
      if (!maybe_line) {
        co_return std::optional<std::string>{};
      }
      const auto line = *maybe_line;
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

  boost::asio::awaitable<std::optional<std::string>>
  ReadLine(io::AsyncStream& stream) {
    boost::system::error_code ec;
    co_await boost::asio::async_read_until(
        stream,
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
      std::cout << "< " << result << "\n";
    }
    co_return result;
  }

  boost::asio::awaitable<void> DoInfo(io::AsyncStream& stream) {
    auto device_info = co_await GetDeviceInfo(stream);

    mjlib::base::Json5WriteArchive(std::cout).Accept(&device_info);
    std::cout << "\n";
  }

  boost::asio::awaitable<void> DoConsole(io::SharedStream stream) {
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
  }

  boost::asio::awaitable<void> DoFlash(io::AsyncStream& stream) {
    // First, get our two binaries.
    const auto elf =
        ReadElf(options_.flash,
                {".text", ".ARM.extab", ".ARM.exidx",
                 ".data", ".ccmram", ".isr_vector"});

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
        const auto result =
            co_await Command(stream, cmd, CommandOptions().set_retry_timeout(5.0));
        // Not much we can do to recover from a flash operation not working.  :(
        if (!result) {
          std::cerr << "Error writing to flash.\n";
          std::exit(1);
        }
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
        const auto maybe_result = co_await Command(
            stream, cmd, CommandOptions().set_allow_any_response(true));
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

    boost::trim(fields[1]);
    std::transform(fields[1].begin(), fields[1].end(),
                   fields[1].begin(),
                   [](auto c) { return std::tolower(c); });
    base::system_error::throw_if(
        Hexify(expected.data) != fields[1],
        fmt::format("verify returned wrong data at {:x}, {} != {}",
                    expected.address,
                    fields[1], Hexify(expected.data)));
  }

  boost::asio::awaitable<DeviceInfo> GetDeviceInfo(io::AsyncStream& stream) {
    DeviceInfo result;

    const auto firmware = co_await ReadData(stream, "firmware");
    const auto git = co_await ReadData(stream, "git");

    auto get = [](const auto& map, auto key, auto dft) -> std::string {
      auto it = map.find(key);
      if (it == map.end()) { return dft; }
      return it->second;
    };

    auto verify_keys = [](const auto& data,
                          const std::vector<std::string>& keys) {
      for (const auto& key : keys) {
        mjlib::base::system_error::throw_if(
            data.count(key) == 0,
            fmt::format("did not find required firmware key: {}", key));
      }
    };

    verify_keys(
        firmware,
        {
          "firmware.serial_number.0",
              "firmware.serial_number.1",
              "firmware.serial_number.2",
              "firmware.version",
              });

    result.serial_number = fmt::format(
        "{:08x}{:08x}{:08x}",
        std::stoi(firmware.at("firmware.serial_number.0")),
        std::stoi(firmware.at("firmware.serial_number.1")),
        std::stoi(firmware.at("firmware.serial_number.2")));
    result.model =
        fmt::format("{:x}", std::stoi(get(firmware, "firmware.model", "0")));

    verify_keys(git,  []() {
        std::vector<std::string> result;
        result.push_back("git.dirty");
        for (int i = 0; i < 20; i++) {
          result.push_back(fmt::format("git.hash.{}", i));
        }
        return result;
      }());

    result.git_hash = MakeGitHash(git);
    result.git_dirty = git.at("git.dirty") != "0";

    co_return result;
  }

  boost::asio::awaitable<void> DoCalibrate(io::AsyncStream& stream) {
    std::cout << "This will move the motor, ensure it can spin freely!\n";
    co_await Sleep(2.0);

    std::string error_message;

    try {
      // The user needs to have set this beforehand.
      unwrapped_position_scale_ =
          co_await ReadConfigDouble(stream, "motor.unwrapped_position_scale");

      // We have 3 things to calibrate.
      //  1) The encoder to phase mapping
      //  2) The winding resistance
      //  3) The kV rating of the motor.

      std::cout << "Starting calibration process\n";
      co_await CheckForFault(stream);

      const auto cal_result = co_await CalibrateEncoderMapping(stream);
      co_await CheckForFault(stream);

      const auto winding_resistance =
          co_await CalibrateWindingResistance(stream);
      co_await CheckForFault(stream);

      const auto v_per_hz = co_await CalibrateKvRating(stream);
      co_await CheckForFault(stream);

      // Rezero the servo since we just spun it a lot.
      co_await Command(stream, "d rezero");

      std::cout << "Saving to persistent storage\n";

      co_await Command(stream, "conf write");

      std::cout << "Calibration complete\n";
      CalibrationReport report;

      report.timestamp = io::Now(executor_.context());

      report.device_info = co_await GetDeviceInfo(stream);

      report.calibration = cal_result;
      report.winding_resistance = winding_resistance;
      report.v_per_hz = v_per_hz;
      report.kv = kPi / (report.v_per_hz / (2 * kPi));
      report.unwrapped_position_scale = unwrapped_position_scale_;

      std::string log_filename = fmt::format(
          "moteus-cal-{}-{}.log",
          report.device_info.serial_number,
          boost::posix_time::to_iso_string(report.timestamp));

      std::cout << fmt::format("REPORT: {}\n", log_filename);
      std::cout << "------------------------\n";

      mjlib::base::Json5WriteArchive(std::cout).Accept(&report);

      std::cout << "\n";

      std::ofstream log_file(GetLogDirectory() + "/" + log_filename);
      if (!log_file.is_open()) {
        std::cerr << "Could not open log: " << log_filename << "\n";
      } else {
        mjlib::base::Json5WriteArchive(log_file).Accept(&report);
        log_file << "\n";
      }

      co_return;
    } catch (std::runtime_error& e) {
      // Eat exceptions so that we can stop.
      error_message = e.what();
    }

    // At least attempt to stop the motor.
    co_await StopIf(true, stream, error_message);
  }

  boost::asio::awaitable<CalibrationResult>
  CalibrateEncoderMapping(io::AsyncStream& stream) {
    // We start with the encoder mapping.  For that to work, we
    // first want to get it locked into zero phase.
    co_await Command(
        stream, fmt::format(
            "d pwm 0 {} s{}",
            options_.calibration_power, options_.calibration_speed));
    co_await Sleep(3.0);

    co_await(Command(stream, "d stop"));
    co_await Sleep(0.1);

    co_await WriteMessage(
        stream, fmt::format("d cal {}", options_.calibration_power));

    std::vector<std::string> lines;
    int index = 0;
    while (true) {
      const auto maybe_line = co_await ReadLine(stream);
      StopIf(!maybe_line, stream, "error reading calibration");
      const auto line = boost::trim_copy(*maybe_line);
      if (!options_.verbose) {
        std::cout << "Calibrating " << "/-\\|"[index] << "\r";
        std::cout.flush();
        index = (index + 1) % 4;
      }
      lines.push_back(line);
      if (boost::starts_with(lines.back(), "CAL done")) {
        // All good!
        break;
      }
      if (boost::starts_with(lines.back(), "CAL start")) {
        continue;
      }
      if (boost::starts_with(lines.back(), "CAL")) {
        // Some problem.
        base::system_error::throw_if(
            true, fmt::format("Error calibrating: {}", lines.back()));
      }
    }

    const auto cal_result = Calibrate(lines);

    std::cout << "\nStoring encoder config\n";
    co_await Command(
        stream, fmt::format("conf set motor.poles {}", cal_result.poles));
    co_await Command(
        stream, fmt::format("conf set motor.invert {}",
                            cal_result.invert ? 1 : 0));
    for (size_t i = 0; i < cal_result.offset.size(); i++) {
      co_await Command(
          stream, fmt::format("conf set motor.offset.{} {}",
                              i, cal_result.offset[i]));
    }

    co_return cal_result;
  }

  boost::asio::awaitable<std::map<std::string, std::string>> ReadData(
      io::AsyncStream& stream, const std::string& channel) {
    co_await Command(stream, fmt::format("tel fmt {} 1", channel));
    const auto maybe_response =
        co_await Command(stream, fmt::format("tel get {}", channel),
                         CommandOptions().set_retry_timeout(1.0));
    StopIf(!maybe_response, stream, "couldn't get telemetry");
    const auto response = *maybe_response;
    std::vector<std::string> lines;
    boost::split(lines, response, boost::is_any_of("\r\n"));

    std::map<std::string, std::string> result;
    for (const auto& line : lines) {
      std::vector<std::string> fields;
      boost::split(fields, line, boost::is_any_of(" "));
      if (fields.size() < 2) { continue; }
      result[fields.at(0)] = fields.at(1);

      if (options_.verbose) {
        std::cout << fmt::format(
            "key='{}' value='{}'\n", fields.at(0), fields.at(1));
      }
    }

    co_return result;
  }

  boost::asio::awaitable<double> FindCurrent(
      io::AsyncStream& stream, double voltage) {
    BOOST_ASSERT(voltage < 0.6);
    BOOST_ASSERT(voltage >= 0.0);

    co_await Command(stream, fmt::format("d pwm 0 {:.3f}", voltage));

    // Wait a bit for it to stabilize.
    co_await Sleep(0.3);

    // Now get the servo_stats telemetry channel to read the D and Q
    // currents.
    auto data = co_await ReadData(stream, "servo_stats");

    // Stop the current.
    co_await Command(stream, "d stop");

    // Sleep a tiny bit before returning.
    co_await Sleep(0.1);

    const auto d_cur = std::stod(data["servo_stats.d_A"]);
    const auto q_cur = std::stod(data["servo_stats.q_A"]);

    const double current_A = std::hypot(d_cur, q_cur);
    std::cout << fmt::format("{}V - {}A\n", voltage, current_A);
    co_return current_A;
  }

  double CalculateWindingResistance(
      const std::vector<double>& voltages_in,
      const std::vector<double>& currents_in) const {
    Eigen::VectorXd voltages(voltages_in.size(), 1);
    for (size_t i = 0; i < voltages_in.size(); i++) {
      voltages(i) = voltages_in[i];
    }
    Eigen::MatrixXd currents(currents_in.size(), 1);
    for (size_t i = 0; i < currents_in.size(); i++) {
      currents(i, 0) = currents_in[i];
    }

    Eigen::VectorXd solution = currents.colPivHouseholderQr().solve(voltages);
    return solution(0, 0);
  }

  boost::asio::awaitable<double> CalibrateWindingResistance(
      io::AsyncStream& stream) {
    std::cout << "Calculating winding resistance\n";

    const std::vector<double> voltages = { 0.25, 0.3, 0.35, 0.4, 0.45 };
    std::vector<double> currents;
    for (auto voltage : voltages) {
      currents.push_back(co_await FindCurrent(stream, voltage));
    }

    const auto winding_resistance =
        CalculateWindingResistance(voltages, currents);

    std::cout <<
        fmt::format("Winding resistance: {} ohm\n", winding_resistance);

    co_await Command(stream, fmt::format("conf set motor.resistance_ohm {}",
                                         winding_resistance));

    co_return winding_resistance;
  }

  boost::asio::awaitable<double> CalibrateKvRating(io::AsyncStream& stream) {
    std::cout << "Calculating kV rating\n";

    // Retrieve and then restore the position configuration.
    const double original_position_min =
        co_await ReadConfigDouble(stream, "servopos.position_min");
    const double original_position_max =
        co_await ReadConfigDouble(stream, "servopos.position_max");

    const double speed = options_.kv_speed * unwrapped_position_scale_;

    co_await Command(stream, "conf set servopos.position_min -10000");
    co_await Command(stream, "conf set servopos.position_max 10000");
    co_await Command(stream, "conf set motor.v_per_hz 0");
    co_await Command(stream, "d index 0");
    co_await Command(stream, fmt::format("d pos nan {} 5", speed));

    co_await Sleep(2.0);

    // Read a number of times to be sure we've got something real.
    Eigen::VectorXd q_Vs(80);
    for (int i = 0; i < q_Vs.size(); i++) {
      const auto servo_control = co_await ReadData(stream, "servo_control");
      co_await Sleep(0.05);
      q_Vs[i] = std::stod(servo_control.at("servo_control.q_V"));
    }

    co_await Command(stream, "d stop");
    co_await Sleep(0.5);

    if (options_.verbose) {
      std::cout << fmt::format("q_V[{}] = ", q_Vs.size());
      std::cout << q_Vs << "\n";
    }

    const double average_q_V = q_Vs.mean();

    const double v_per_hz = unwrapped_position_scale_ * average_q_V / speed;

    std::cout << fmt::format("speed={} q_V={} v_per_hz (pre-gearbox)={}\n",
                             speed, average_q_V, v_per_hz);

    co_await Command(
        stream, fmt::format("conf set motor.v_per_hz {}", v_per_hz));
    co_await Command(
        stream, fmt::format("conf set servopos.position_min {}",
                            original_position_min));
    co_await Command(
        stream, fmt::format("conf set servopos.position_max {}",
                            original_position_max));

    co_return v_per_hz;
  }

  boost::asio::awaitable<double> ReadConfigDouble(
      io::AsyncStream& stream,
      const std::string& name) {
    const auto maybe_result = co_await Command(
        stream, fmt::format("conf get {}", name),
        CommandOptions().set_allow_any_response(true));
    co_await StopIf(!maybe_result, stream,
                    fmt::format("could not retrieve {}", name));
    const auto result = std::stod(*maybe_result);
    co_return result;
  }

  boost::asio::awaitable<void> CheckForFault(io::AsyncStream& stream) {
    const auto servo_stats = co_await ReadData(stream, "servo_stats");
    if (servo_stats.at("servo_stats.mode") == "1") {
      mjlib::base::system_error::throw_if(
          true, fmt::format("Controller reported fault: {}",
                            servo_stats.at("servo_stats.fault")));
    }
    co_return;
  }

  boost::asio::awaitable<void> StopIf(
      bool condition, io::AsyncStream& stream, const std::string& message) {
    if (!condition) { co_return; }

    co_await Command(stream, "d stop");
    std::cerr << "Error\n";
    std::cerr << message << "\n";
    std::exit(1);

    co_return;
  }

  boost::asio::awaitable<void> Sleep(double seconds) {
    boost::asio::deadline_timer timer(executor_);
    timer.expires_from_now(mjlib::base::ConvertSecondsToDuration(seconds));
    co_await timer.async_wait(boost::asio::use_awaitable);
  }

  boost::asio::executor executor_;
  boost::asio::executor_work_guard<boost::asio::executor> guard_{executor_};
  io::Selector<mp::AsioClient>* const client_selector_;
  std::vector<int> targets_;
  bool discovered_ = false;
  const Options options_;

  io::DeadlineTimer timer_{executor_};
  io::StreamFactory factory_{executor_};
  io::SharedStream stdio_;
  mp::AsioClient* client_ = nullptr;
  std::optional<io::BidirectionalStreamCopy> copy_;

  boost::asio::streambuf read_streambuf_;

  double unwrapped_position_scale_ = 1.0;
};

}

int moteus_tool_main(boost::asio::io_context& context,
                     int argc, char** argv,
                     io::Selector<mp::AsioClient>* selector) {
  io::Selector<mp::AsioClient> default_client_selector{
    context.get_executor(), "client_type"};
  if (selector == nullptr) {
    // Set some convenient defaults.
    mp::StreamAsioClientBuilder::Options default_stream_options;
    default_stream_options.stream.type = io::StreamFactory::Type::kSerial;
    default_stream_options.stream.serial_port = "/dev/fdcanusb";
    // If the baud rate does matter, 3mbit is a good one.
    default_stream_options.stream.serial_baud = 3000000;

    default_client_selector.Register<mp::StreamAsioClientBuilder>(
        "stream", default_stream_options);
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
      clipp::option("i", "info").set(options.info).doc(
          "display information from the servo"),
      clipp::option("c", "console").set(options.console).doc(
          "create a serial console"),
      clipp::option("dump-config").set(options.dump_config).doc(
          "emit all configuration to the console"),
      clipp::option("flash") & clipp::value("file", options.flash).doc(
          "write the given elf file to flash"),
      clipp::option("calibrate").set(options.calibrate).doc(
          "calibrate the motor, requires full freedom of motion"),
      clipp::option("calibration_power") &
      clipp::value("V", options.calibration_power).doc(
          "voltage to use during calibration"),
      clipp::option("calibration_speed") &
      clipp::value("S", options.calibration_speed).doc(
          "speed in electrical rps"),
      clipp::option("kv_speed") &
      clipp::value("S", options.kv_speed).doc(
          "speed in mechanical rotor rps")
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
