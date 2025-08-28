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

int main(int argc, char **argv)
{
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

  // Command a stop to the controller.
  controller.SetStop();
}
