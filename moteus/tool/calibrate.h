// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#pragma once

#include <string>
#include <vector>

#include "mjlib/base/visitor.h"

namespace moteus {
namespace tool {

struct CalibrationResult {
  bool invert = false;
  int poles = 0;
  std::vector<double> offset;

  // Debugging information.
  double total_phase = 0;
  double total_delta = 0;
  double ratio = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(invert));
    a->Visit(MJ_NVP(poles));
    a->Visit(MJ_NVP(offset));
  }
};

/// Attempt to solve the calibration problem for a moteus servo.  @p
/// lines is the console output of the "d cal" command, including the
/// initial and final line.
///
/// This will throw an mjlib::base::system_error on error.
CalibrationResult Calibrate(const std::vector<std::string>& lines);

}
}
