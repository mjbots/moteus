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
/// Demonstrate how diagnostic mode commands, like configuration
/// operations, can be performed.

#include <iostream>

#include "moteus.h"

int main(int argc, char** argv) {
  using namespace mjbots;

  // Let the user configure a default transport.
  moteus::Controller::DefaultArgProcess(argc, argv);

  moteus::Controller controller;

  // When using the diagnostic protocol, it is important to know that
  // applications like tview may leave the controller "spewing" on the
  // diagnostic channel, i.e. sending unsolicited data.  In order stop
  // that, a client needs to issue a "tel stop" command, and then
  // flush all data that is present.  After that point, normal
  // commands can be issued.
  controller.DiagnosticWrite("tel stop\n");
  controller.DiagnosticFlush();


  // DiagnosticCommand will always return the result.  The `conf get`
  // diagnostic command is unique in that it replies with a single
  // line and no final "OK", so we use the `kExpectSingleLine` option.
  const auto old_kp = std::stod(
      controller.DiagnosticCommand("conf get servo.pid_position.kp",
                                   moteus::Controller::kExpectSingleLine));
  const auto new_kp = 4.0;

  std::ostringstream ostr;
  ostr << "conf set servo.pid_position.kp " << new_kp;

  // The `conf set` diagnostic command returns nothing.
  controller.DiagnosticCommand(ostr.str());

  std::cout << "Changed kp from " << old_kp << " to " << new_kp << "\n";

  return 0;
}
