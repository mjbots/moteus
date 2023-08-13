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
/// Show how to send Position mode commands at a regular interval and
/// intepret telemetry data from the servo.

#include <unistd.h>

#include <iostream>

#include "moteus.h"

int main(int argc, char** argv) {
  using namespace mjbots;

  // The following DefaultArgProcess is an optional call.  If made,
  // then command line arguments will be handled which allow setting
  // and configuring the default 'transport' to be used if none is
  // specified in Controller::Options::transport.
  moteus::Controller::DefaultArgProcess(argc, argv);

  // There are many possible options to set for each controller
  // instance.  Here we re-set the ID to the default (1), just to show
  // how it is done.
  moteus::Controller::Options options;
  options.id = 1;

  moteus::Controller controller(options);

  // Command a stop to the controller in order to clear any faults.
  controller.SetStop();

  while (true) {
    moteus::PositionMode::Command cmd;

    // Here we will just command a position of NaN and a velocity of
    // 0.0.  This means "hold position wherever you are".
    cmd.position = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity = 0.0;

    const auto maybe_result = controller.SetPosition(cmd);
    if (maybe_result) {
      const auto r = maybe_result->values;
      ::printf("%3d p/v/t=(%7.3f,%7.3f,%7.3f)  v/t/f=(%5.1f,%5.1f,%3d)   \r",
             static_cast<int>(r.mode),
             r.position,
             r.velocity,
             r.torque,
             r.voltage,
             r.temperature,
             r.fault);
      ::fflush(stdout);
    }

    // Sleep 20ms between iterations.  By default, when commanded over
    // CAN, there is a watchdog which requires commands to be sent at
    // least every 100ms or the controller will enter a latched fault
    // state.
    ::usleep(20000);
  }
}
