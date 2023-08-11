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
/// This example shows how multiple controllers can be commanded using
/// the Cycle method.  This approach can result in lower overall
/// latency and improved performance with some transports, such as the
/// pi3hat.

#include <unistd.h>

#include <cmath>
#include <iostream>
#include <map>
#include <vector>

#include "moteus.h"

static double GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
      static_cast<double>(ts.tv_nsec) / 1e9;
}

int main(int argc, char** argv) {
  using namespace mjbots;

  moteus::Controller::DefaultArgProcess(argc, argv);

  std::map<int, std::shared_ptr<moteus::Controller>> controllers;
  for (int i = 1; i <= 2; i++) {
    moteus::Controller::Options options;
    options.id = i;

    // If the intended transport supported multiple busses, you would
    // configure them here as well.
    //
    // options.bus = foo;

    controllers[i] = std::make_shared<moteus::Controller>(options);
  }

  // Stop everything to clear faults.
  for (const auto& pair : controllers) {
    pair.second->SetStop();
  }

  auto transport = moteus::Controller::MakeSingletonTransport({});

  while (true) {
    const auto now = GetNow();
    std::vector<moteus::CanFdFrame> command_frames;


    for (const auto& pair : controllers) {
      moteus::PositionMode::Command position_command;
      position_command.position = NaN;
      position_command.velocity = 0.1 * std::sin(now + pair.first);
      command_frames.push_back(pair.second->MakePosition(position_command));
    }

    std::vector<moteus::CanFdFrame> replies;
    transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);

    char buf[4096] = {};
    std::string status_line;

    ::snprintf(buf, sizeof(buf) - 1, "%10.2f) ", now);

    status_line += buf;

    for (const auto& frame : replies) {
      const auto r = moteus::Query::Parse(frame.data, frame.size);
      ::snprintf(buf, sizeof(buf) - 1,
                 "%2d %3d p/v/t=(%7.3f,%7.3f,%7.3f)  ",
                 frame.source,
                 static_cast<int>(r.mode),
                 r.position,
                 r.velocity,
                 r.torque);
      status_line += buf;
    }
    ::printf("%s  \r", status_line.c_str());
    ::fflush(::stdout);

    // Sleep 20ms between iterations.  By default, when commanded over
    // CAN, there is a watchdog which requires commands to be sent at
    // least every 100ms or the controller will enter a latched fault
    // state.
    ::usleep(20000);
  }
}
