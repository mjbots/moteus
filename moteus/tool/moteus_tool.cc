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
#include <boost/multiprecision/cpp_int.hpp>

#include <fmt/format.h>

#include <Eigen/Dense>

#include <elfio/elfio.hpp>

#include "mjlib/base/clipp.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/json5_read_archive.h"
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

double CalculateSlope(const std::vector<double>& x_in,
                      const std::vector<double>& y_in) {
  Eigen::VectorXd x(x_in.size(), 1);
  for (size_t i = 0; i < x_in.size(); i++) {
    x(i) = x_in[i];
  }
  Eigen::MatrixXd y(y_in.size(), 1);
  for (size_t i = 0; i < y_in.size(); i++) {
    y(i, 0) = y_in[i];
  }

  Eigen::VectorXd solution = y.colPivHouseholderQr().solve(x);
  return solution(0, 0);
}

double CalculateWindingResistance(
    const std::vector<double>& voltages_in,
    const std::vector<double>& currents_in) {
  return CalculateSlope(voltages_in, currents_in);
}

struct FlashDataBlock {
  int64_t address = -1;
  std::string data;

  FlashDataBlock(int64_t address_in, std::string data_in)
      : address(address_in), data(data_in) {}

  FlashDataBlock() {}
};


struct ElfMapping {
  int64_t virtual_address = 0;
  int64_t physical_address = 0;
  int64_t size = 0;
};

struct ElfData {
  // Blocks of data associated with a given address.
  std::map<int64_t, std::string> data;
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
  std::string write_config;
  bool console = false;
  std::string flash;
  bool no_restore_config = false;

  bool verbose = false;
  std::vector<std::string> targets;

  bool calibrate = false;
  bool zero_offset = false;
  double calibration_power = 0.40;
  double calibration_speed = 1.0;
  std::string calibration_raw;

  double resistance_voltage = 0.45;

  std::string restore_calibration;
};

std::string Base64SerialNumber(
    uint32_t s1,
    uint32_t s2,
    uint32_t s3) {
  using bigint = boost::multiprecision::uint128_t;

  bigint serial_num = (bigint(s1) << 64) | (bigint(s2) << 32) | bigint(s3);
  constexpr char digits[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  BOOST_ASSERT(sizeof(digits) == 65);
  char result[17] = {};
  for (int i = 0; i < 16; i++) {
    const int digit_num = (serial_num % 64).convert_to<int>();
    result[15 - i] = digits[digit_num];
    serial_num /= 64;
  }
  return std::string(result);
}

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

class Controller {
 public:
  Controller(io::AsyncStream& stream, const Options& options)
      : stream_(stream),
        options_(options) {}

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
      const std::string& message,
      CommandOptions command_options = CommandOptions()) {

    for (int i = 0; i < command_options.retry_count; i++) {
      co_await WriteMessage(message);

      const auto expiration = io::Now(stream_.get_executor().context()) +
          base::ConvertSecondsToDuration(command_options.retry_timeout);

      const auto maybe_result = co_await RunFor(
          stream_.get_executor(),
          stream_,
          [&]() -> boost::asio::awaitable<std::optional<std::string>> {
            if (command_options.allow_any_response) {
              co_return co_await ReadLine();
            } else {
              co_return co_await ReadUntilOK();
            }
          },
          expiration);

      if (!!maybe_result) {
        co_return *maybe_result;
      }
    }

    co_return std::optional<std::string>();
  }

  boost::asio::awaitable<std::optional<std::string>> ReadUntilOK() {
    std::string result;
    while (true) {
      auto maybe_line = co_await ReadLine();
      if (!maybe_line) {
        co_return std::optional<std::string>{};
      }
      const auto line = *maybe_line;
      if (line.substr(0, 2) == "OK") { co_return result; }
      result += line + "\n";
    }
  }

  boost::asio::awaitable<void> WriteMessage(const std::string& message) {
    if (options_.verbose) {
      std::cout << fmt::format("> {}\n", message);
    }

    co_await boost::asio::async_write(
        stream_,
        boost::asio::buffer(message + "\n"),
        boost::asio::use_awaitable);
    co_return;
  }

  boost::asio::awaitable<std::optional<std::string>> ReadLine() {
    boost::system::error_code ec;
    co_await boost::asio::async_read_until(
        stream_,
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

  boost::asio::awaitable<double> ReadConfigDouble(
      const std::string& name) {
    const auto maybe_result = co_await Command(
        fmt::format("conf get {}", name),
        CommandOptions().set_allow_any_response(true));
    co_await StopIf(!maybe_result, fmt::format("could not retrieve {}", name));
    try {
      const auto result = std::stod(*maybe_result);
      co_return result;
    } catch (std::invalid_argument& e) {
      throw base::system_error::einval(
          fmt::format("error reading '{}'={}: {}",
                      name, *maybe_result, e.what()));
    }
  }

  boost::asio::awaitable<void> CheckForFault() {
    const auto servo_stats = co_await ReadData("servo_stats");
    if (servo_stats.at("servo_stats.mode") == "1") {
      mjlib::base::system_error::throw_if(
          true, fmt::format("Controller reported fault: {}",
                            servo_stats.at("servo_stats.fault")));
    }
    co_return;
  }

  boost::asio::awaitable<void> StopIf(
      bool condition, const std::string& message) {
    if (!condition) { co_return; }

    co_await Command("d stop");
    std::cerr << "Error\n";
    std::cerr << message << "\n";
    std::exit(1);

    co_return;
  }

  boost::asio::awaitable<void> RestoreConfig(const std::string& old_config) {
    std::istringstream istr(old_config);
    std::ostringstream ostr;

    while (static_cast<bool>(istr)) {
      std::string line;
      std::getline(istr, line);
      boost::trim(line);
      if (line.empty()) { continue; }
      ostr << "conf set " << line << "\n";
    }
    ostr << "conf write\n";

    std::istringstream conf_write(ostr.str());
    co_await WriteConfigStream(conf_write);
  }

  boost::asio::awaitable<void> WriteConfigStream(std::istream& config_stream) {
    std::vector<std::string> errors;
    while (static_cast<bool>(config_stream)) {
      std::string line;
      std::getline(config_stream, line);
      boost::trim(line);
      if (line.empty()) { continue; }

      if (options_.verbose) {
        std::cout << ":" + line + "\n";
      }
      auto maybe_result = co_await Command(line);
      if (!maybe_result) {
        errors.push_back(line);
      }
    }

    if (errors.size()) {
      std::cout << "\nSome config could not be set:\n";
      for (const auto& line : errors) {
        std::cout << " " << line << "\n";
      }
      std::cout << "\n";
    }

    co_return;
  }

  boost::asio::awaitable<DeviceInfo> GetDeviceInfo() {
    DeviceInfo result;

    const auto firmware = co_await ReadData("firmware");
    const auto git = co_await ReadData("git");

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

    result.serial_number = Base64SerialNumber(
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

  boost::asio::awaitable<void> WriteFlash(const ElfData& elf) {
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
            co_await Command(cmd, CommandOptions().set_retry_timeout(5.0));
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
            cmd, CommandOptions().set_allow_any_response(true));
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

  boost::asio::awaitable<std::map<std::string, std::string>> ReadData(
      const std::string& channel) {
    co_await Command(fmt::format("tel fmt {} 1", channel));
    const auto maybe_response =
        co_await Command(fmt::format("tel get {}", channel),
                         CommandOptions().set_retry_timeout(1.0));
    StopIf(!maybe_response, "couldn't get telemetry");
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

  boost::asio::awaitable<double> FindSpeed(double voltage) {
    BOOST_ASSERT(voltage < 1.0);
    BOOST_ASSERT(voltage >= 0.0);

    co_await Command(fmt::format("d vdq 0 {:.3f}", voltage));

    // Wait for it to stabilize.
    co_await Sleep(1.0);

    const auto data = co_await ReadData("servo_stats");

    const auto velocity = std::stod(data.at("servo_stats.velocity"));

    std::cout << fmt::format("{}V - {}Hz\n", voltage, velocity);

    co_return velocity;
  }

  boost::asio::awaitable<double> FindCurrent(double voltage) {
    BOOST_ASSERT(voltage < 3.0);
    BOOST_ASSERT(voltage >= 0.0);

    co_await Command(fmt::format("d pwm 0 {:.3f}", voltage));

    // Wait a bit for it to stabilize.
    co_await Sleep(0.3);

    // Now get the servo_stats telemetry channel to read the D and Q
    // currents.
    const auto data = co_await ReadData("servo_stats");

    // Stop the current.
    co_await Command("d stop");

    // Sleep a tiny bit before returning.
    co_await Sleep(0.1);

    const auto d_cur = std::stod(data.at("servo_stats.d_A"));
    const auto q_cur = std::stod(data.at("servo_stats.q_A"));

    const double current_A = std::hypot(d_cur, q_cur);
    std::cout << fmt::format("{}V - {}A\n", voltage, current_A);
    co_return current_A;
  }

  boost::asio::awaitable<double> CalibrateWindingResistance() {
    std::cout << "Calculating winding resistance\n";

    const std::vector<double> ratios = { 0.5, 0.6, 0.7, 0.85, 1.0 };
    std::vector<double> voltages;
    for (const auto ratio : ratios) {
      voltages.push_back(ratio * options_.resistance_voltage);
    }
    std::vector<double> currents;
    for (auto voltage : voltages) {
      currents.push_back(co_await FindCurrent(voltage));
    }

    const auto winding_resistance =
        CalculateWindingResistance(voltages, currents);

    std::cout <<
        fmt::format("Winding resistance: {} ohm\n", winding_resistance);

    co_await Command(fmt::format("conf set motor.resistance_ohm {}",
                                 winding_resistance));

    co_return winding_resistance;
  }

  boost::asio::awaitable<double> CalibrateKvRating(
      double unwrapped_position_scale) {
    std::cout << "Calculating Kv rating\n";

    // Retrieve and then restore the position configuration.
    const double original_position_min =
        co_await ReadConfigDouble("servopos.position_min");
    const double original_position_max =
        co_await ReadConfigDouble("servopos.position_max");

    co_await Command("conf set servopos.position_min NaN");
    co_await Command("conf set servopos.position_max NaN");
    co_await Command("d index 0");

    const std::vector<double> voltages = { 0.0, 0.2, 0.4, 0.6, 0.8 };
    std::vector<double> speed_hzs;
    for (auto voltage : voltages) {
      speed_hzs.push_back(co_await FindSpeed(voltage));
    }

    co_await Command("d stop");

    co_await Sleep(0.5);

    const double geared_v_per_hz = CalculateSlope(voltages, speed_hzs);

    const double v_per_hz =
        geared_v_per_hz * unwrapped_position_scale;

    std::cout << fmt::format("v_per_hz (pre-gearbox)={}\n", v_per_hz);

    co_await Command(
        fmt::format("conf set motor.v_per_hz {}", v_per_hz));
    co_await Command(
        fmt::format("conf set servopos.position_min {}",
                    original_position_min));
    co_await Command(
        fmt::format("conf set servopos.position_max {}",
                    original_position_max));

    co_return v_per_hz;
  }

  boost::asio::awaitable<void> Sleep(double seconds) {
    boost::asio::deadline_timer timer(stream_.get_executor());
    timer.expires_from_now(mjlib::base::ConvertSecondsToDuration(seconds));
    co_await timer.async_wait(boost::asio::use_awaitable);
  }

 private:
  io::AsyncStream& stream_;
  const Options options_;
  boost::asio::streambuf read_streambuf_;
};

class Runner {
 public:
  Runner(const boost::asio::any_io_executor& executor,
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

    Controller controller(*stream, options_);
    auto maybe_result = co_await controller.Command(
        "tel stop",
        Controller::CommandOptions().set_retry_timeout(0.02).set_retry_count(1));

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
      (!options_.write_config.empty() ? 1 : 0) +
      (options_.console ? 1 : 0) +
      (!options_.flash.empty() ? 1 : 0) +
      (options_.calibrate ? 1 : 0) +
      (options_.zero_offset ? 1 : 0) +
      (!options_.restore_calibration.empty() ? 1 : 0)
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
      if (!options_.restore_calibration.empty()) { return true; }
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

    co_return;
  }

  boost::asio::awaitable<void> RunAction(int id) {
    mp::AsioClient::TunnelOptions tunnel_options;

    auto stream = client_->MakeTunnel(id, kDebugTunnel, tunnel_options);
    auto controller = Controller(*stream, options_);

    if (options_.stop) {
      co_await controller.Command("d stop");
    } else if (options_.info) {
      co_await DoInfo(&controller);
    } else if (options_.dump_config) {
      Controller::CommandOptions command_options;
      command_options.retry_timeout = 2.0;
      const auto maybe_result = co_await controller.Command(
          "conf enumerate", command_options);
      if (maybe_result) {
        std::cout << *maybe_result;
      }
    } else if (!options_.write_config.empty()) {
      co_await DoWriteConfig(&controller);
    } else if (options_.console) {
      co_await DoConsole(stream);
    } else if (!options_.flash.empty()) {
      co_await DoFlash(&controller);
    } else if (options_.calibrate) {
      co_await DoCalibrate(&controller);
    } else if (options_.zero_offset) {
      co_await DoZeroOffset(&controller);
    } else if (!options_.restore_calibration.empty()) {
      co_await DoRestoreCalibration(&controller);
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

  boost::asio::awaitable<void> DoFlash(Controller* controller) {
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

    // Read our old config.
    const auto old_config = co_await controller->Command(
        "conf enumerate",
        Controller::CommandOptions().set_retry_timeout(2.0));

    std::cout << "Captured old config\n";

    base::system_error::throw_if(
        !old_config,
        fmt::format("unable to retrieve original configuration"));

    co_await controller->WriteMessage("d flash");
    co_await controller->ReadLine();
    co_await controller->Command("unlock");
    co_await controller->WriteFlash(elf);
    co_await controller->Command("lock");
    // We don't expect to get a reply from this.
    co_await controller->WriteMessage("reset");

    // Wait a bit for the controller to restart.
    co_await controller->Sleep(1.0);

    if (!options_.no_restore_config) {
      // Now restore our old config in case the new firmware isn't able
      // to load it.
      co_await controller->RestoreConfig(*old_config);

      std::cout << "Restored config\n";
    }
  }

  boost::asio::awaitable<void> DoInfo(Controller* controller) {
    auto device_info = co_await controller->GetDeviceInfo();

    mjlib::base::Json5WriteArchive(std::cout).Accept(&device_info);
    std::cout << "\n";
  }

  boost::asio::awaitable<void> DoWriteConfig(Controller* controller) {
    std::ifstream inf(options_.write_config);
    mjlib::base::system_error::throw_if(
        !inf.is_open(), "opening file: " + options_.write_config);

    std::cout << "Writing config from: " + options_.write_config + "\n";

    co_await controller->WriteConfigStream(inf);
  }

  boost::asio::awaitable<void> DoCalibrate(Controller* controller) {
    std::cout << "This will move the motor, ensure it can spin freely!\n";
    co_await controller->Sleep(2.0);

    std::string error_message;

    try {
      // The user needs to have set this beforehand.
      const double unwrapped_position_scale =
          co_await controller->ReadConfigDouble(
              "motor.unwrapped_position_scale");

      // We have 3 things to calibrate.
      //  1) The encoder to phase mapping
      //  2) The winding resistance
      //  3) The Kv rating of the motor.

      std::cout << "Starting calibration process\n";
      co_await controller->CheckForFault();

      const auto cal_result = co_await CalibrateEncoderMapping(controller);
      co_await controller->CheckForFault();

      const auto winding_resistance =
          co_await controller->CalibrateWindingResistance();
      co_await controller->CheckForFault();

      const auto v_per_hz = co_await controller->CalibrateKvRating(
          unwrapped_position_scale);
      co_await controller->CheckForFault();

      // Rezero the servo since we just spun it a lot.
      co_await controller->Command("d rezero");

      std::cout << "Saving to persistent storage\n";

      co_await controller->Command("conf write");

      std::cout << "Calibration complete\n";
      CalibrationReport report;

      report.timestamp = io::Now(executor_.context());

      report.device_info = co_await controller->GetDeviceInfo();

      report.calibration = cal_result;
      report.winding_resistance = winding_resistance;
      report.v_per_hz = v_per_hz;

      // We measure voltage to the center, not peak-to-peak, thus the
      // extra 0.5.
      report.kv = 0.5 * 60.0 / report.v_per_hz;
      report.unwrapped_position_scale = unwrapped_position_scale;

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
    co_await controller->StopIf(true, error_message);
  }

  boost::asio::awaitable<CalibrationResult>
  CalibrateEncoderMapping(Controller* controller) {
    // We start with the encoder mapping.  For that to work, we
    // first want to get it locked into zero phase.
    co_await controller->Command(
        fmt::format(
            "d pwm 0 {}",
            options_.calibration_power));
    co_await controller->Sleep(3.0);

    co_await controller->Command("d stop");
    co_await controller->Sleep(0.1);

    co_await controller->WriteMessage(
        fmt::format(
            "d cal {} s{}",
            options_.calibration_power, options_.calibration_speed));

    std::vector<std::string> lines;
    int index = 0;
    while (true) {
      const auto maybe_line = co_await controller->ReadLine();
      controller->StopIf(!maybe_line, "error reading calibration");
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

    if (!options_.calibration_raw.empty()) {
      std::ofstream of(options_.calibration_raw);
      for (auto& line : lines) {
        of << line << "\n";
      }
    }

    const auto cal_result = Calibrate(lines);

    std::cout << "\nStoring encoder config\n";
    co_await controller->Command(
        fmt::format("conf set motor.poles {}", cal_result.poles));
    co_await controller->Command(
        fmt::format("conf set motor.invert {}",
                    cal_result.invert ? 1 : 0));
    for (size_t i = 0; i < cal_result.offset.size(); i++) {
      co_await controller->Command(
          fmt::format("conf set motor.offset.{} {}",
                      i, cal_result.offset[i]));
    }

    co_return cal_result;
  }

  boost::asio::awaitable<void> DoZeroOffset(Controller* controller) {
    const auto servo_stats = co_await controller->ReadData("servo_stats");
    const auto position_raw =
        std::stoi(servo_stats.at("servo_stats.position_raw"));
    co_await controller->Command(
        fmt::format("conf set motor.position_offset {:d}",
                    -position_raw));
    co_await controller->Command("conf write");
    co_await controller->Command("d rezero");

    co_return;
  }

  boost::asio::awaitable<void> DoRestoreCalibration(Controller* controller) {
    std::ifstream in(options_.restore_calibration);
    const auto report =
        mjlib::base::Json5ReadArchive::Read<CalibrationReport>(in);

    const auto& cal_result = report.calibration;

    // Verify that the serial number matches.
    const auto this_device_info = co_await controller->GetDeviceInfo();
    if (this_device_info.serial_number != report.device_info.serial_number) {
      throw base::system_error::einval(
          fmt::format("Serial number in calibration ({}) does not match device ({})",
                      report.device_info.serial_number,
                      this_device_info.serial_number));
    }

    co_await controller->Command(
        fmt::format("conf set motor.poles {}", cal_result.poles));
    co_await controller->Command(
        fmt::format("conf set motor.invert {}",
                    cal_result.invert ? 1 : 0));
    for (size_t i = 0; i < cal_result.offset.size(); i++) {
      co_await controller->Command(
          fmt::format("conf set motor.offset.{} {}",
                      i, cal_result.offset[i]));
    }

    co_await controller->Command(
        fmt::format("conf set motor.resistance_ohm {}",
                    report.winding_resistance));
    co_await controller->Command(
        fmt::format("conf set motor.v_per_hz {}", report.v_per_hz));

    co_await controller->Command("conf write");

    std::cout << "Calibration restored\n";

    co_return;
  }

  boost::asio::any_io_executor executor_;
  boost::asio::executor_work_guard<boost::asio::any_io_executor> guard_{executor_};
  io::Selector<mp::AsioClient>* const client_selector_;
  std::vector<int> targets_;
  bool discovered_ = false;
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
           clipp::value("TGT", options.targets)) %
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
      clipp::option("write-config") &
      clipp::value("file", options.write_config).doc(
          "write the given configuration"),
      clipp::option("flash") & clipp::value("file", options.flash).doc(
          "write the given elf file to flash"),
      clipp::option("no-restore-config").set(options.no_restore_config).doc(
          "do not restore config after flash"),
      clipp::option("calibrate").set(options.calibrate).doc(
          "calibrate the motor, requires full freedom of motion"),
      clipp::option("zero-offset").set(options.zero_offset).doc(
          "set the motor's position offset"),
      clipp::option("calibration_power") &
      clipp::value("V", options.calibration_power).doc(
          "voltage to use during calibration"),
      clipp::option("calibration_speed") &
      clipp::value("S", options.calibration_speed).doc(
          "speed in electrical rps"),
      clipp::option("calibration_raw") &
      clipp::value("FILE", options.calibration_raw).doc(
          "write raw calibration data"),
      clipp::option("resistance_voltage") &
      clipp::value("V", options.resistance_voltage).doc(
          "maximum voltage when measuring resistance"),
      clipp::option("restore-calibration") &
      clipp::value("FILE", options.restore_calibration).doc(
          "restore calibration from logged data")
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
