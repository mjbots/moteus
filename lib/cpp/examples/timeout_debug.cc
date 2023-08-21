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
/// A debugging example to allow calling some API functions with a
/// variety of inputs while reporting the results and timing.

#include <cmath>
#include <iostream>
#include <map>
#include <optional>
#include <vector>

#include "moteus.h"

namespace {
using namespace mjbots;

using IdBus = std::pair<int, int>;

static double GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
      static_cast<double>(ts.tv_nsec) / 1e9;
}

void DisplayUsage() {
  std::cout << "Usage: timeout_debug [options] [cmd]...\n";
  std::cout << "\n";
  std::cout << "  -h, --help             display this usage message\n";
  std::cout << "  -s, --servo ID[,BUS]   servo to communicate with\n";

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

  std::cout << "Command format:\n";
  std::cout << " {TYPE}{ID_BUS}(:[CFQ]option1,option2,...)+\n";
  std::cout << "\n";
  std::cout << "Types:\n";
  std::cout << " q - SetQuery\n";
  std::cout << " p - SetPosition\n";
  std::cout << "\n";
  std::cout << "Details:\n";
  std::cout << " C - position command arguments\n";
  std::cout << " F - position command format\n";
  std::cout << " Q - override query format\n";
  std::cout << "   OPTION = {FLAG}[i8|i16|i32|f]\n";
  std::cout << "   m - mode\n";
  std::cout << "   p - position\n";
  std::cout << "   v - velocity\n";
  std::cout << "   t - torque\n";
  std::cout << "   q - Q current\n";
  std::cout << "   d - D current\n";
  std::cout << "   V - voltage\n";
  std::cout << "   T - temperature\n";
  std::cout << "   F - fault\n";

  std::cout << "\n";
}

std::vector<std::string> Split(const std::string& str, char delim) {
  std::vector<std::string> result;
  size_t pos = 0;
  while (true) {
    auto next = str.find(delim, pos);
    if (next == std::string::npos) {
      result.push_back(str.substr(pos));
      return result;
    } else {
      result.push_back(str.substr(pos, next - pos));
      pos = next + 1;
    }
  }
}

IdBus ParseIdBus(const std::string& str) {
  const auto fields = Split(str, ',');
  if (fields.empty()) {
    throw std::runtime_error("No id found: " + str);
  }
  const auto id = std::stol(fields.at(0));
  const auto bus =
      fields.size() > 1 ?
      std::stol(fields.at(1)) : 1;
  return {id, bus};
}

template <typename T, typename GetFlag>
std::optional<T> ParseFormat(const std::string& str,
                             GetFlag get_flag) {
  if (str.empty()) { return {}; }

  T result;

  const auto fields = Split(str.substr(1), ',');
  for (const auto& field : fields) {
    if (field.empty()) { continue; }
    moteus::Resolution* const to_set = get_flag(field.at(0), &result);
    if (to_set == nullptr) {
      throw std::runtime_error("Unknown format field: " + field);
    }

    const auto fmt = field.substr(1);
    if (fmt == "i8") {
      *to_set = moteus::kInt8;
    } else if (fmt == "i16") {
      *to_set = moteus::kInt16;
    } else if (fmt == "i32") {
      *to_set = moteus::kInt32;
    } else if (fmt == "f") {
      *to_set = moteus::kFloat;
    } else {
      throw std::runtime_error("Unknown query format resolution: " + field);
    }
  }

  return result;
}

auto PositionFlagGet = [](char c, auto* s) {
  switch (c) {
    case 'p': return &s->position;
    case 'v': return &s->velocity;
    case 'f': return &s->feedforward_torque;
    case 'k': return &s->kp_scale;
    case 'd': return &s->kd_scale;
    case 't': return &s->maximum_torque;
    case 's': return &s->stop_position;
    case 'w': return &s->watchdog_timeout;
    case 'V': return &s->velocity_limit;
    case 'A': return &s->accel_limit;
    case 'O': return &s->fixed_voltage_override;
    default: return static_cast<decltype(&s->position)>(nullptr);
  }
};

std::optional<moteus::Query::Format> ParseQuery(const std::string& str) {
  return ParseFormat<moteus::Query::Format>(
      str, [](char c, auto* r) -> moteus::Resolution*{
        switch (c) {
          case 'm': { return &r->mode; }
          case 'p': { return &r->position; }
          case 'v': { return &r->velocity; }
          case 't': { return &r->torque; }
          case 'q': { return &r->q_current; }
          case 'd': { return &r->d_current; }
          case 'a': { return &r->abs_position; }
          case 'V': { return &r->voltage; }
          case 'T': { return &r->temperature; }
          case 'F': { return &r->fault; }
          default: { return nullptr; }
        }
      });
}

std::optional<moteus::PositionMode::Format> ParsePositionFormat(const std::string& str) {
  return ParseFormat<moteus::PositionMode::Format>(
      str, [](char c, auto* r) { return PositionFlagGet(c, r); });
}

std::optional<moteus::PositionMode::Command>
ParsePositionCommand(const std::string& str) {
  if (str.empty()) { return {}; }

  moteus::PositionMode::Command result;

  const auto fields = Split(str, ',');

  for (const auto& field : fields) {
    const double value = std::stod(field.substr(1));
    double* const to_set = PositionFlagGet(field.at(0), &result);
    if (!to_set) {
      throw std::runtime_error("Unknown position command field: " + field);
    }
    *to_set = value;
  }
  return result;
}

template <typename T>
void PrintResult(double time, const T& maybe_qr) {
  char buf[1024] = {};
  ::sprintf(buf, "(%.6f) ", time);
  std::cout << "query_result " << buf;

  if (!maybe_qr) {
    std::cout << "timeout!\n";
    return;
  }

  const auto& f = maybe_qr->frame;
  std::cout << "id=" << static_cast<int>(f.source)
            << "," << "bus=" << static_cast<int>(f.bus) << " ";

  std::cout << "[";

  const auto& q = maybe_qr->values;

  std::cout << "mode=" << static_cast<int>(q.mode);

  auto emit_if_finite = [&](auto label, auto value) {
    if (!std::isfinite(value)) { return; }
    std::cout << label << value;
  };

  emit_if_finite(",pos=", q.position);
  emit_if_finite(",vel=", q.velocity);
  emit_if_finite(",torque=", q.torque);
  emit_if_finite(",q_cur=", q.q_current);
  emit_if_finite(",d_cur=", q.d_current);
  emit_if_finite(",abs=", q.abs_position);
  emit_if_finite(",mtemp=", q.motor_temperature);
  emit_if_finite(",volt=", q.voltage);
  emit_if_finite(",temp=", q.temperature);

  std::cout << "]\n";
}
}

int main(int argc, char** argv) {
  using namespace mjbots;

  const std::vector<std::string> args_in(argv, argv + argc);
  if (std::find(args_in.begin(), args_in.end(), "--help") != args_in.end() ||
      std::find(args_in.begin(), args_in.end(), "-h") != args_in.end()) {
    DisplayUsage();
    return 0;
  }

  auto args = moteus::Controller::ProcessTransportArgs(args_in);
  args.erase(args.begin());

  const auto find_arg = [&](const auto& s) {
    auto it = std::find(args.begin(), args.end(), s);
    return it != args.end() && (it + 1) != args.end();
  };

  const auto remove_arg = [&](const auto& s) {
    auto it = std::find(args.begin(), args.end(), s);
    auto result = *(it + 1);
    args.erase(it, it + 2);
    return result;
  };

  std::map<IdBus, std::shared_ptr<moteus::Controller>> servos;
  while (find_arg("-s")) {
    const IdBus id_bus = ParseIdBus(remove_arg("-s"));
    servos[id_bus] = std::make_shared<moteus::Controller>([&]() {
      moteus::Controller::Options options;
      options.id = id_bus.first;
      options.bus = id_bus.second;
      return options;
    }());
  }

  auto find_field = [&](const auto& fields, char prefix) -> std::string {
    for (const auto& field : fields) {
      if (field.empty()) { continue; }
      if (field.at(0) == prefix) { return field; }
    }
    return "";
  };

  for (const auto& arg : args) {
    if (arg.empty()) { continue; }

    const auto cmd_type = arg.at(0);
    const auto fields = Split(arg, ':');

    const auto id_bus_str = Split(fields.at(0).substr(1), ',');
    if (id_bus_str.empty()) {
      throw std::runtime_error("No ID or BUS found in: " + arg);
    }
    const int id = std::stol(id_bus_str.at(0));
    const int bus = id_bus_str.size() > 1 ?
        std::stol(id_bus_str.at(1)) : 1;
    const IdBus id_bus = { id, bus };
    const auto servo_ptr = servos.at(id_bus);

    const auto cmd_field = find_field(fields, 'C');
    const auto fmt_field = find_field(fields, 'F');
    const auto query_field = find_field(fields, 'Q');

    const auto maybe_query = ParseQuery(query_field);


    const auto start = GetNow();
    const auto query_result = [&]() {
      if (cmd_type == 'q') {
        return servo_ptr->SetQuery(
            !maybe_query ? nullptr : &*maybe_query);
      } else if (cmd_type == 's') {
        return servo_ptr->SetStop(
            !maybe_query ? nullptr : &*maybe_query);
      } else if (cmd_type == 'p') {
        const auto cmd = ParsePositionCommand(cmd_field);
        const auto maybe_fmt = ParsePositionFormat(fmt_field);

        return servo_ptr->SetPosition(
            !cmd ? moteus::PositionMode::Command() : *cmd,
            !maybe_fmt ? nullptr : &*maybe_fmt,
            !maybe_query ? nullptr : &*maybe_query);
      }
      throw std::runtime_error("Unknown command: " + std::string(cmd_type, 1));
    }();
    const auto end = GetNow();

    PrintResult(end - start, query_result);
  }
  return 0;
}
