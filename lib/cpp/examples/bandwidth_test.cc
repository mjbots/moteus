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
/// This can be used to determine rough maximal bandwidth capability
/// of a given transport for a position/query loop.

#include <stdio.h>
#include <unistd.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "moteus.h"

namespace {
// A simple way to get the current time accurately as a double.
static double GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
      static_cast<double>(ts.tv_nsec) / 1e9;
}
}

int main(int argc, char** argv) {
  using namespace mjbots;

  const std::vector<std::string> args_in(argv, argv + argc);
  auto args = moteus::Controller::ProcessTransportArgs(args_in);
  auto transport = moteus::Controller::MakeSingletonTransport({});

  // Just for some kind of "--help".
  moteus::Controller::DefaultArgProcess(argc, argv);

  args.erase(args.begin());  // our name

  // Should use use int16 position/velocity command and query and
  // disable torque query?
  const bool minimal_format = [&]() {
    auto it = std::find(args.begin(), args.end(), "--minimal-format");
    if (it != args.end()) {
      args.erase(it);
      return true;
    }
    return false;
  }();

  // This will keep track of if we had a response to a given device in
  // each cycle.
  std::map<int, bool> responses;

  // Populate our list of controllers with IDs from the command line.
  std::vector<std::shared_ptr<moteus::Controller>> controllers;
  while (!args.empty()) {
    auto id = std::stoul(args.front());
    args.erase(args.begin());
    controllers.push_back(
        std::make_shared<moteus::Controller>(
            [&]() {
              moteus::Controller::Options options;
              options.id = id;
              if (minimal_format) {
                options.position_format.position = moteus::kInt16;
                options.position_format.velocity = moteus::kInt16;
                options.query_format.position = moteus::kInt16;
                options.query_format.velocity = moteus::kInt16;
                options.query_format.torque = moteus::kIgnore;
              }
              return options;
            }()));
    responses[id] = false;
  }

  std::vector<moteus::CanFdFrame> send_frames;
  std::vector<moteus::CanFdFrame> receive_frames;

  moteus::PositionMode::Command cmd;
  cmd.position = NaN;
  cmd.velocity = 0.0;

  constexpr double kStatusPeriodS = 0.1;
  double status_time = GetNow() + kStatusPeriodS;
  int hz_count = 0;

  while (true) {
    hz_count++;
    send_frames.clear();
    receive_frames.clear();

    for (auto& c : controllers) {
      send_frames.push_back(c->MakePosition(cmd));
    }
    for (auto& pair : responses) {
      pair.second = false;
    }

    transport->BlockingCycle(&send_frames[0], send_frames.size(),
                             &receive_frames);

    for (const auto& f : receive_frames) {
      responses[f.source] = true;
    }

    const int count = [&]() {
      int sum = 0;
      for (const auto& pair : responses) {
        if (pair.second) { sum++; }
      }
      return sum;
    }();

    const auto now = GetNow();
    if (now > status_time) {
      printf("%6.1fHz  rx_count=%2d   \r",
             hz_count / kStatusPeriodS, count);
      fflush(stdout);

      hz_count = 0;
      status_time += kStatusPeriodS;
    }

    ::usleep(10);
  }
}
