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

/// @file
///
/// Show how to control and monitor many servos at once.  This example
/// implements a "n-lateral" teleoperation setup, where the movement
/// of any servo will result in all of the others moving as well.
///
/// This won't perform as well as a two servo teleop scenario where
/// one servo is the sensor and the other is controlled, but it does
/// allow for interesting effects like software configurable
/// bi-directional gear reductions.

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>


#include "moteus.h"

namespace {
constexpr double kFaultVelocity = 5.0;
constexpr double kFaultPosition = 2.0;

using namespace mjbots;

struct Printer {
  Printer(char* buf, size_t capacity) : buf_(buf), capacity_(capacity) {};

  const char* buf() { return buf_; }
  size_t size() const { return pos_; }
  size_t remaining() const { return capacity_ - pos_ - 1; }

  void operator()(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    auto n = ::vsnprintf(&buf_[pos_], remaining(), fmt, ap);
    va_end(ap);
    if (n < 0) { ::abort(); }
    pos_ += n;
  };

  char* const buf_;
  size_t pos_ = 0;
  const size_t capacity_;
};

template <typename Vector, typename KeyGetter>
double Average(const Vector& vector, KeyGetter key_getter) {
  double total = 0.0;
  double count = 0;
  for (const auto& item : vector) {
    total += key_getter(item);
    count += 1;
  }
  return total / count;
}

using ServoId = std::pair<int, int>;

std::vector<std::string> Split(const std::string str) {
  std::vector<std::string> result;

  size_t start = 0;
  auto pos = str.find(',');
  while (pos != std::string::npos) {
    result.push_back(str.substr(start, pos - start + 1));
    start = pos + 1;
    pos = str.find(',', start);
  }
  result.push_back(str.substr(start));
  return result;
}

/// The commandline arguments associated with a single servo.
struct Servo {

  // The following are the configurable values for this servo.
  int id = -1;
  int bus = 1;
  double position_scale = 1.0;
  double force_scale = 1.0;


  // And now the state.
  std::optional<moteus::Controller> controller;
  double initial_position = std::numeric_limits<double>::quiet_NaN();
  moteus::Query::Result query;
  int skipped = 0;

  bool seen_this_cycle = false;

  static Servo Parse(const std::string& message) {
    auto fields = Split(message);
    Servo result;
    result.id = std::stol(fields.at(0));
    fields.erase(fields.begin());
    for (const auto& field : fields) {
      if (field.at(0) == 'b') {
        result.bus = std::stol(field.substr(1));
      } else if (field.at(0) == 'p') {
        result.position_scale = std::stod(field.substr(1));
        if (std::isfinite(result.position_scale) &&
            result.position_scale > 0.25 &&
            result.position_scale < 16.0) {
          // good
        } else {
          throw std::runtime_error("Position scale out of range: " + field);
        }
      } else if (field.at(0) == 'f') {
        result.force_scale = std::stod(field.substr(1));

        if (std::isfinite(result.force_scale) &&
            result.force_scale > 0.25 &&
            result.force_scale < 4.0) {
          // good
        } else {
          throw std::runtime_error("Force scale out of range: " + field);
        }
      } else {
        throw std::runtime_error("Unknown option: " + field);
      }
    }
    return result;
  }
};

struct Arguments {
  Arguments(const std::vector<std::string>& args_in) {
    const auto args = moteus::Controller::ProcessTransportArgs(args_in);
    for (size_t i = 0; i < args.size(); i++) {
      const auto& arg = args[i];
      if (arg == "-h" || arg == "--help") {
        help = true;
      } else if (arg == "--period-s") {
        period_s = std::stod(args.at(++i));
      } else if (arg == "-s" || arg == "--servo") {
        servos.push_back(Servo::Parse(args.at(++i)));
      } else if (arg == "--kp") {
        kp = std::stod(args.at(++i));
      } else if (arg == "--kd") {
        kd = std::stod(args.at(++i));
      } else if (arg == "--max-torque") {
        max_torque = std::stod(args.at(++i));
      } else {
        throw std::runtime_error("Unknown argument: " + arg);
      }
    }
  }

  bool help = false;
  double period_s = 0.002;
  double kp = 1.0;
  double kd = 0.01;
  double max_torque = 0.3;
  std::vector<Servo> servos;
};

void DisplayUsage() {
  std::cout << "Usage: nlateral_teleop [options]\n";
  std::cout << "\n";
  std::cout << "  -h, --help           display this usage message\n";
  std::cout << "  --period-s S         period to run control\n";
  std::cout << "  --kp XX.X            select kp value\n";
  std::cout << "  --kd XX.X            select kd value\n";
  std::cout << "  --max-torque XX.X    maximum torque to apply to a servo\n";
  std::cout << "  -s, --servo CFG      add one servo to be controlled\n";
  std::cout << "   CFG=ID[,option]...\n";
  std::cout << "    bN - bus number N\n";
  std::cout << "    pXX.X - scale position by this positive float\n";
  std::cout << "    fXX.X - scale force by this positive float\n";

  auto pad = [](const std::string str, int size) {
    if (static_cast<int>(str.size()) < size) {
      return str + std::string(size - str.size(), ' ');
    }
    return str;
  };

  const auto transport_args = moteus::Controller::cmdline_arguments();
  for (const auto& arg : transport_args) {
    std::cout << "  " << pad(arg.name, 21)  << arg.help << "\n";
  }

  std::cout << "\n";
  std::cout << "Example w/ two moteus devkit motors on ID 1 and 2:\n";
  std::cout << "  sudo ./nlateral_demo -s 1 -s 2 --period-s 0.001 --kp 1.0 --kd 0.01\n";

}

class Teleop {
 public:
  using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
  using Duration = decltype(std::chrono::microseconds(0));
  using CanFdFrame = moteus::CanFdFrame;

  Teleop(const Arguments args)
      : args_(args),
        servos_(args.servos) {
    moteus::PositionMode::Format fmt;
    fmt.position = moteus::kIgnore;
    fmt.velocity = moteus::kIgnore;
    fmt.feedforward_torque = moteus::kFloat;
    fmt.kp_scale = moteus::kInt8;
    fmt.kd_scale = moteus::kInt8;
    fmt.maximum_torque = moteus::kIgnore;
    fmt.stop_position = moteus::kIgnore;
    fmt.watchdog_timeout = moteus::kIgnore;
    for (auto& servo : servos_) {
      moteus::Controller::Options options;
      options.id = servo.id;
      options.bus = servo.bus;
      options.position_format = fmt;

      servo.controller.emplace(options);
    }

    // These things are common to all our commands.
    position_command_.kp_scale = 0.0;
    position_command_.kd_scale = 0.0;

    transport_ = moteus::Controller::RequireSingletonTransport();
  }

  void Run() {
    CommandAll([](moteus::Controller* c) { return c->MakeOutputNearest({}); });
    CommandAll([](moteus::Controller* c) { return c->MakeStop(); });

    // Capture our initial positions.
    for (auto& servo : servos_) {
      while (!std::isfinite(servo.initial_position)) {
        auto maybe_result = servo.controller->SetQuery();
        if (!maybe_result) { continue; }
        servo.initial_position = maybe_result->values.position;
        servo.query = maybe_result->values;
      }
    }

    next_status_ = std::chrono::steady_clock::now();

    while (true) {
      cycle_count_++;
      hz_count_++;

      const bool is_fault = ExecuteControl();
      if (is_fault) { break; }

      const auto now = std::chrono::steady_clock::now();
      MaybePrintStatus(now);
    }

    while (true) {
      cycle_count_++;

      CommandAll([](moteus::Controller* c) { return c->MakeBrake(); });
      const auto now = std::chrono::steady_clock::now();
      MaybePrintStatus(now);
    }
  }

  void MaybePrintStatus(const TimePoint& now) {
    if (now < next_status_) { return; }

    PrintStatus();

    next_status_ += status_period_;
  }

  bool ExecuteControl() {
    const auto average_position = Average(
        servos_,
        [&](const auto& s) {
          return (s.query.position - s.initial_position) * s.position_scale;
        });
    const auto average_velocity = Average(
        servos_,
        [&](const auto& s) {
          return s.query.velocity * s.position_scale;
        });


    send_commands_.clear();
    for (auto& servo : servos_) {
      const double pos =
          servo.query.position - servo.initial_position;

      const auto position_error = pos - average_position;
      if (std::abs(position_error) > kFaultPosition) {
        // Count this as a fault too.
        std::cout << "\n\nposition fault\n\n";
        return true;
      }
      const auto p = -args_.kp * position_error * servo.position_scale;

      const auto velocity_error = servo.query.velocity - average_velocity;
      if (std::abs(velocity_error) > kFaultVelocity) {
        // We will count this as a fault.
        std::cout << "\n\nvelocity fault\n\n";
        return true;
      }
      const auto d = -args_.kd * velocity_error * servo.position_scale;

      const auto unlimited_torque = (p + d) * servo.force_scale;
      const auto torque =
          (unlimited_torque < -args_.max_torque) ? -args_.max_torque :
          (unlimited_torque > args_.max_torque) ? args_.max_torque :
          unlimited_torque;

      position_command_.feedforward_torque = torque;

      send_commands_.push_back(servo.controller->MakePosition(position_command_));
    }

    return IssueCommand();
  }

  bool IssueCommand() {
    receive_commands_.clear();
    transport_->BlockingCycle(
        &send_commands_[0],
        send_commands_.size(),
        &receive_commands_);

    for (auto& servo : servos_) { servo.seen_this_cycle = false; }

    // File away our responses.
    for (const auto& result : receive_commands_) {
      auto* const servo = FindServo(result);
      if (!servo) { continue; }
      servo->seen_this_cycle = true;
      servo->query = moteus::Query::Parse(result.data, result.size);
    }

    for (auto& servo : servos_) {
      if (!servo.seen_this_cycle) { servo.skipped++; }
      if (servo.query.mode == moteus::Mode::kFault) {
        std::cout << "\n\nservo fault\n\n";
        return true;
      }
    }

    // If we are running on an isolcpus cpu in linux, we have to sleep
    // every now and then or linux imposes a big delay on us. This
    // would typically manifest as a every 1s blip.
    std::this_thread::sleep_for(std::chrono::microseconds(10));

    return false;
  }

  void PrintStatus() {
    const double update_hz = hz_count_ / kStatusPeriod;
    hz_count_ = 0;

    std::map<int, Servo*> results;
    for (auto& servo : servos_) {
      results[servo.id << 8 | servo.bus] = &servo;
    }
    std::string modes;
    char buf[4096] = {};
    for (const auto& result_pair : results) {
      Printer p(buf, sizeof(buf));
      const auto id = result_pair.first >> 8;
      const auto bus = result_pair.first & 0xff;
      const auto& q = result_pair.second->query;
      p("%d/%d/%2d/%7.4f/%6.3f/%d , ",
        id, bus, static_cast<int>(q.mode), q.position, q.torque,
        result_pair.second->skipped);
      modes += buf;
    }

    Printer p(buf, sizeof(buf));
    p("Cycles %6d %5.0f Hz ",
      cycle_count_, update_hz);
    std::cout << buf;
    std::cout << modes;
    std::cout << "\r";
    std::cout.flush();
  }

  Servo* FindServo(const CanFdFrame& frame) {
    for (auto& servo : servos_) {
      if (servo.id == frame.source &&
          servo.bus == (frame.bus == 0 ? 1 : frame.bus)) {
        return &servo;
      }
    }
    return nullptr;
  }

  template <typename FrameMaker>
  void CommandAll(FrameMaker fm) {
    send_commands_.clear();
    for (auto& servo : servos_) {
      send_commands_.push_back(fm(&*servo.controller));
    }
    IssueCommand();
  }

 private:
  const Arguments args_;
  std::vector<Servo> servos_;
  std::shared_ptr<moteus::Transport> transport_;
  moteus::PositionMode::Command position_command_;

  std::vector<CanFdFrame> send_commands_;
  std::vector<CanFdFrame> receive_commands_;

  const double kStatusPeriod = 0.1;
  Duration status_period_{std::chrono::milliseconds(
        static_cast<int>(kStatusPeriod * 1000))};

  TimePoint next_status_;

  uint64_t cycle_count_ = 0;
  uint64_t hz_count_ = 0;
};

}

int main(int argc, char** argv) {
  Arguments args({argv + 1, argv + argc});

  if (args.help) {
    DisplayUsage();
    return 0;
  }

  Teleop teleop(args);
  teleop.Run();
  return 0;
}
