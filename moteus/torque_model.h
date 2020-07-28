// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include <algorithm>

#include "moteus/math.h"

namespace moteus {

/// Provides facilities for converting to and from torque and phase
/// current for a given motor.
class TorqueModel {
 public:
  TorqueModel(float torque_constant,
              float current_cutoff_A,
              float current_scale,
              float torque_scale)
      : torque_constant_(torque_constant),
        current_cutoff_A_(current_cutoff_A),
        current_scale_(current_scale),
        torque_scale_(torque_scale) {}

  float current_to_torque(float current) const __attribute__((always_inline)) {
    // We always calculate the "rotation" region cutoff so as to have
    // a somewhat constant calculation time.
    const float rotation_extra =
        torque_scale_ *
        log2f_approx(1.0f + std::max(
                         0.0f, (current - current_cutoff_A_)) *
                     current_scale_);
    return ((current < current_cutoff_A_) ?
            // rotation_extra will always be 0 here, but we add it in
            // anyways to force the evaluation of the above code no
            // matter what.  Thus our loop timing will be relatively
            // constant even when we go into the rotation regime.
            (current * torque_constant_ + rotation_extra) :
            // In this case, rotation_extra should be non-zero.
            (current_cutoff_A_ * torque_constant_ +
             rotation_extra));
  }

  float torque_to_current(float torque) const __attribute__((always_inline)) {
    const float a = (torque - current_cutoff_A_ * torque_constant_) /
                    torque_scale_;
    const float rotation_extra = (pow2f_approx(a) - 1.0f) / current_scale_;

    const float cutoff_torque =
        current_cutoff_A_ * torque_constant_;
    return (torque < cutoff_torque) ?
        (torque / torque_constant_) :
        (current_cutoff_A_ + rotation_extra);
  }

  const float torque_constant_;
  const float current_cutoff_A_;
  const float current_scale_;
  const float torque_scale_;
};

}
