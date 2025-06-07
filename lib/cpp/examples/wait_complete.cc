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

/// @file
///
/// This example shows how to use Controller::SetPositionWaitComplete.

#include <unistd.h>

#include <iostream>

#include "moteus.h"

int main(int argc, char** argv) {
  using namespace mjbots;

  moteus::Controller::DefaultArgProcess(argc, argv);

  moteus::Controller::Options options;
  options.id = 1;

  // Only position and velocity are sent by default when sending
  // position mode commands.  If you want to send any other fields you
  // need to specify their resolution.  That can either be done here
  // in the `Controller::Options` structure, or on a per-command basis
  // using the format override argument.
  options.position_format.accel_limit = moteus::kFloat;

  moteus::Controller controller(options);

  // Command a stop to the controller in order to clear any faults.
  controller.SetStop();

  moteus::PositionMode::Command cmd;

  auto print = [](auto maybe_result) {
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

      if (r.mode == moteus::Mode::kFault ||
          r.mode == moteus::Mode::kPositionTimeout) {
        ::printf("\n\nExiting because of fault %d/%d.\n",
                 static_cast<int>(r.mode), r.fault);
        std::exit(1);
      }
    }
  };

  while (true) {
    cmd.position = 0.5;
    cmd.accel_limit = 2.0;

    print(controller.SetPositionWaitComplete(cmd, 0.01));

    cmd.position = 0.0;
    print(controller.SetPositionWaitComplete(cmd, 0.01));
  }

  return 0;
}
