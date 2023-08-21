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
/// A very simple two servo teleoperation demonstration.  One servo is
/// the sensor, and the other is driven.

#include <inttypes.h>
#include <stdio.h>

#include <algorithm>
#include <vector>

#include "moteus.h"

namespace {
using namespace mjbots;

// A simple way to get the current time accurately as a double.
static double GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
      static_cast<double>(ts.tv_nsec) / 1e9;
}

void DisplayUsage() {
  std::cout << "Usage: simple_teleop [options]\n";
  std::cout << "\n";
  std::cout << "  -h, --help            display this usage message\n";
  std::cout << "  --primary-id    ID    servo to sense from [default: 1]\n";
  std::cout << "  --primary-bus   BUS   bus for primary servo [default: 1]\n";
  std::cout << "  --secondary-id  ID    servo to drive [default: 2]\n";
  std::cout << "  --secondary-bus BUS   bus for secondary servo [default: 1]\n";

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
}
}

int main(int argc, char** argv) {
  int primary_id = 1;
  int primary_bus = 1;
  int secondary_id = 2;
  int secondary_bus = 1;

  std::vector<std::string> args_in(argv, argv + argc);
  if (std::find(args_in.begin(), args_in.end(), "--help") != args_in.end()) {
    DisplayUsage();
    std::exit(0);
  }

  const auto args = moteus::Controller::ProcessTransportArgs(args_in);
  auto find_int_arg = [&](const std::string& name, int def_value) -> int {
    auto it = std::find(args.begin(), args.end(), name);
    if (it != args.end() && (it + 1) != args.end()) {
      return std::stol(*(it + 1));
    }
    return def_value;
  };
  primary_id = find_int_arg("--primary-id", primary_id);
  primary_bus = find_int_arg("--primary-bus", primary_bus);
  secondary_id = find_int_arg("--secondary-id", secondary_id);
  secondary_bus = find_int_arg("--secondary-bus", secondary_bus);

  moteus::Controller primary([&]() {
    moteus::Controller::Options options;
    options.id = primary_id;
    options.bus = primary_bus;
    return options;
  }());
  moteus::Controller secondary([&]() {
    moteus::Controller::Options options;
    options.id = secondary_id;
    options.bus = secondary_bus;
    return options;
  }());

  // We initially send a stop command to each in order to clear any
  // faults.
  primary.SetStop();
  secondary.SetStop();

  // Determine the initial position for each.
  auto get_initial = [&](auto* controller) {
    while (true) {
      auto maybe_result = controller->SetQuery();
      if (maybe_result) {
        return maybe_result->values.position;
      }
    }
  };

  const double primary_initial = get_initial(&primary);
  const double secondary_initial = get_initial(&secondary);

  moteus::PositionMode::Command position_cmd;

  constexpr double kStatusPeriod = 0.1;

  uint64_t cycles = 0;
  auto next_status = GetNow() + kStatusPeriod;
  uint64_t hz_count = 0;

  int rx_timeout = 0;
  int tx_timeout = 0;

  moteus::Query::Result primary_query;
  moteus::Query::Result secondary_query;

  while (true) {
    cycles++;
    hz_count++;

    bool new_data = false;

    // First query the sensor.
    auto maybe_primary_state = primary.SetQuery();
    if (!!maybe_primary_state) {
      primary_query = maybe_primary_state->values;
      new_data = true;
    } else {
      rx_timeout++;
    }
    const auto& ps = primary_query;

    if (new_data) {
      // Our command will just be to exactly match the position and
      // velocity of the sensor servo.
      position_cmd.position = ps.position - primary_initial + secondary_initial;
      position_cmd.velocity = ps.velocity;

      // The command the driven servo.
      auto maybe_secondary_state = secondary.SetPosition(position_cmd);
      if (!!maybe_secondary_state) {
        secondary_query = maybe_secondary_state->values;
      } else {
        tx_timeout++;
      }
    } else {
      auto maybe_secondary_state = secondary.SetQuery();
      if (!!maybe_secondary_state) {
        secondary_query = maybe_secondary_state->values;
      } else {
        tx_timeout++;
      }
    }
    const auto& ss = secondary_query;

    // If we are running on an isolcpus cpu in linux, we have to sleep
    // every now and then or linux imposes a big delay on us.  This
    // would typically manifest as a every 1s blip.
    ::usleep(10);

    // And finally, print out status at a semi-regular interval.
    const auto now = GetNow();
    if (now > next_status) {
      printf("%.3f %6" PRIu64 " %6.1fHz  %d/%2d/%6.3f/%6.3f/%3d  %d/%2d/%6.3f/%6.3f/%3d    \r",
             now,
             cycles,
             hz_count / kStatusPeriod,

             primary_id,
             static_cast<int>(ps.mode),
             ps.position,
             ps.velocity,
             rx_timeout,

             secondary_id,
             static_cast<int>(ss.mode),
             ss.position,
             ss.velocity,
             tx_timeout);
      fflush(stdout);

      hz_count = 0;
      next_status += kStatusPeriod;
    }

  }

  return 0;
}
