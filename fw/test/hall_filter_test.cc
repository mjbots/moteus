// Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include "fw/motor_position.h"

#include <fstream>
#include <sstream>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "mjlib/micro/test/persistent_config_fixture.h"

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/visitor.h"

using namespace moteus;

namespace {
struct Options {
  std::string input;
  std::string output;
  double pll_filter_hz = 40.0;
  bool no_commutation = false;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(input));
    a->Visit(MJ_NVP(output));
    a->Visit(MJ_NVP(pll_filter_hz));
    a->Visit(MJ_NVP(no_commutation));
  }
};

constexpr int kRequiredHeaders = 5;

std::vector<int> ParseDebugStream(const std::string& str) {
  std::vector<int> result;
  size_t pos = 0;
  while (pos + 3 < str.size()) {
    // Skip until we see N headers in a row that are appropriately
    // spaced.
    while (true) {
      const bool appropriately_spaced = [&]() {
        for (int i = 0; i < kRequiredHeaders &&
                 (pos + i * 3 < str.size()); i++) {
          if (static_cast<uint8_t>(str[pos + i * 3]) != 0x5a) {
            return false;
          }
        }
        return true;
      }();
      if (!appropriately_spaced) {
        pos++;
      } else {
        break;
      }
    }

    result.push_back(*reinterpret_cast<const uint16_t*>(&str[pos + 1]));
    pos += 3;
  }

  return result;
}

double wrap_encoder(double i) {
  if (i >= 90) { return i - 90; }
  if (i < 0) { return i + 90; }
  return i;
}

double wrap_encoder_delta(double i) {
  if (i > 45) { return i - 90; }
  if (i < -45) { return i + 90; }
  return i;
}

struct Data {
  double time = 0.0;
  uint32_t raw_value = 0;
  uint32_t hall_count = 0;
  float compensated_value = 0.0f;
  float filtered_value = 0.0f;
  float velocity = 0.0;

  float truth_value = 0.0f;
  float truth_velocity = 0.0f;
};

struct Application {
  mjlib::micro::test::PersistentConfigFixture pcf;
  mjlib::micro::TelemetryManager telemetry_manager{
    &pcf.pool, &pcf.command_manager, &pcf.write_stream, pcf.output_buffer};
  aux::AuxStatus aux1_status;
  aux::AuxStatus aux2_status;
  aux::AuxConfig aux1_config;
  aux::AuxConfig aux2_config;

  MotorPosition dut{&pcf.persistent_config, &telemetry_manager,
                    &aux1_status, &aux2_status,
                    &aux1_config, &aux2_config};

  Options options;

  static constexpr float kDt = 1.0f / 30000.0f;

  Application(int argc, char** argv) {
    auto group = mjlib::base::ClippArchive().Accept(&options).group();
    mjlib::base::ClippParse(argc, argv, group);

    dut.SetRate(kDt);

    if (options.no_commutation) {
      dut.config()->commutation_source = 1;
    }

    dut.motor()->poles = 30;

    pcf.persistent_config.Load();
  }

  void Run() {
    std::ifstream inf(options.input);
    if (!inf.is_open()) {
      throw std::runtime_error(
          fmt::format("Could not open input: {}", options.input));
    }

    std::shared_ptr<std::ofstream> out;
    if (!options.output.empty()) {
      out = std::make_shared<std::ofstream>(options.output);
    }

    // Read the entire raw input into memory.
    std::stringstream istr;
    istr << inf.rdbuf();

    // The input is a raw dump from the debug port, with
    // `servo.emit_debug=1<<11=2048` in order get the raw value from
    // position source 0.
    std::vector<int> raw_encoder = ParseDebugStream(istr.str());

    dut.config()->sources[0].type = MotorPosition::SourceConfig::kHall;
    dut.config()->sources[0].pll_filter_hz = options.pll_filter_hz;

    pcf.persistent_config.Load();
    aux1_status.hall.active = true;
    double t = 0.0;

    std::vector<Data> data_;

    for (size_t i = 0; i < raw_encoder.size(); i++) {
      // Attempt to determine the ground truth position and velocity
      // for this point.
      const auto raw_value = raw_encoder[i];
      const auto old = aux1_status.hall.count;

      static constexpr uint8_t kHallMapping[] = {
        0,  // invalid
        0,  // 0b001 => 0
        2,  // 0b010 => 2
        1,  // 0b011 => 1
        4,  // 0b100 => 4
        5,  // 0b101 => 5
        3,  // 0b110 => 3
        0,  // invalid
      };
      aux1_status.hall.count = kHallMapping[raw_value];

      if (aux1_status.hall.count != old) {
        aux1_status.hall.nonce += 1;
      }

      dut.ISR_Update();
      const auto status = dut.status();

      Data d;
      d.time = t;
      d.raw_value = raw_value;
      d.hall_count = aux1_status.hall.count;
      d.compensated_value = status.sources[0].compensated_value;
      d.filtered_value = status.sources[0].filtered_value;
      d.velocity = status.sources[0].velocity;

      data_.push_back(d);

      t += kDt;
    }

    auto find_ground_truth = [&](size_t i) {
      const auto current_value = data_[i].compensated_value;
      size_t first = i;
      for (; first > 0 && data_[first].compensated_value == current_value; first--);
      size_t last = i;
      for (; last < data_.size() && data_[last].compensated_value == current_value; last++);

      if (last == data_.size()) {
        return std::make_pair(static_cast<double>(current_value), 0.0);
      }

      if (data_[first].compensated_value == data_[last].compensated_value) {
        // We had a back and forth.  Just assume something in the
        // middle with zero velocity.
        return std::make_pair(static_cast<double>(current_value),
                              static_cast<double>(0.0f));
      }

      const auto fraction =
          static_cast<double>(i - first) / static_cast<double>(last - first);
      const auto velocity =
          static_cast<double>(wrap_encoder_delta(data_[last].compensated_value - current_value)) /
          ((1.0/30000.0) * static_cast<double>((last - first - 1)));
      return std::make_pair(
          static_cast<double>(wrap_encoder(fraction * (wrap_encoder_delta(data_[last].compensated_value - current_value)) + current_value)),
          static_cast<double>(velocity));
    };

    double position_metric = 0.0;
    double velocity_metric = 0.0;

    double max_position_error = 0.0;
    double max_position_error_time = 0.0;
    double max_velocity_error = 0.0;
    double max_velocity_error_time = 0.0;

    for (size_t i = 0; i < data_.size(); i++) {
      const auto [pos, vel] = find_ground_truth(i);

      data_[i].truth_value = pos;
      data_[i].truth_velocity = vel;

      const double position_error = wrap_encoder_delta(data_[i].truth_value - data_[i].filtered_value);
      const double velocity_error = data_[i].truth_velocity - data_[i].velocity;

      const double this_position_metric = std::pow(position_error, 2.0);
      position_metric += this_position_metric;

      const double this_velocity_metric = std::pow(velocity_error, 2.0);
      velocity_metric += this_velocity_metric;

      const auto abs_position_error = std::abs(position_error);
      if (abs_position_error > max_position_error) {
        max_position_error = abs_position_error;
        max_position_error_time = i / 30000.0;
      }

      const auto abs_velocity_error = std::abs(velocity_error);
      if (abs_velocity_error > max_velocity_error) {
        max_velocity_error = abs_velocity_error;
        max_velocity_error_time = i / 30000.0;
      }
    }

    if (out) {
      *out << fmt::format("time,raw,count,compensated,filtered,velocity,truth_pos,truth_vel\n");

      for (const auto& d : data_) {
        *out << fmt::format(
            "{},{},{},{},{},{},{},{}\n",
            d.time, d.raw_value, d.hall_count,
            d.compensated_value,
            d.filtered_value,
            d.velocity,
            d.truth_value,
            d.truth_velocity);
      }
    }

    fmt::print("{{\n");
    fmt::print("  \"position_metric\": {},\n", position_metric / data_.size());
    fmt::print("  \"velocity_metric\": {},\n", velocity_metric / data_.size());
    fmt::print("  \"max_position_error\": {},\n", max_position_error);
    fmt::print("  \"max_position_error_time\": {},\n", max_position_error_time);
    fmt::print("  \"max_velocity_error\": {},\n", max_velocity_error);
    fmt::print("  \"max_velocity_error_time\": {}\n", max_velocity_error_time);
    fmt::print("}}\n");
  }
};
};

int main(int argc, char** argv) {
  Application app(argc, argv);

  try {
    app.Run();
    return 0;
  } catch (std::runtime_error& e) {
    fmt::print(stderr, "Error: {}\n", e.what());
    return 1;
  }

  return 2;
}
