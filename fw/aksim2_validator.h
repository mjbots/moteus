// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include <cstdint>

#include "fw/ccm.h"

namespace moteus {

// Gates the "active" state of an AksIM-2 absolute encoder so that it
// only transitions from inactive to active after several consecutive
// valid readings that agree with one another.
//
// At power-on (and after a power-cycle during operation) an absolute
// encoder may emit a frame or two with a not-yet-acquired position --
// frequently zero -- before it has settled.  If such a transient
// reading is consumed, motor_position will latch the boot output
// position on it and then track the true position in as motion,
// shifting the output wrap window by the configured output offset.
//
// Requiring a few consistent readings (no large jump between
// consecutive samples) rejects those transients without depending on
// the encoder's error flag.
class Aksim2Validator {
 public:
  // The AksIM-2 reports a 22-bit absolute value.
  static constexpr uint32_t kCpr = 4194304;

  // The number of consecutive consistent, valid readings required
  // before allowing a transition from inactive to active.
  static constexpr int kStartupCount = 5;

  // The largest change between consecutive readings tolerated while
  // qualifying for the active state.  Anything larger restarts the
  // qualification, anchored on the new reading.  This is far above any
  // real per-sample motion but well below the gross jumps produced by
  // an unsettled encoder.
  static constexpr uint32_t kMaxStartupDelta = kCpr / 16;

  // Given the most recent reading, return whether the source should be
  // considered active.  'was_active' is the prior active state;
  // 'reading_valid' is true when the encoder reports no error.
  bool Update(bool was_active, bool reading_valid, uint32_t value)
      MOTEUS_CCM_ATTRIBUTE {
    if (was_active) {
      // Once active, remain active until an error is reported, at
      // which point we require re-qualification.
      if (!reading_valid) {
        count_ = 0;
        return false;
      }
      return true;
    }

    if (!reading_valid) {
      count_ = 0;
    } else if (count_ == 0) {
      count_ = 1;
    } else {
      const uint32_t delta = CircularDistance(value, last_value_);
      count_ = (delta > kMaxStartupDelta) ? 1 : (count_ + 1);
    }
    last_value_ = value;

    return count_ >= kStartupCount;
  }

 private:
  // The shortest distance between two readings on the CPR ring.
  static uint32_t CircularDistance(uint32_t a, uint32_t b) MOTEUS_CCM_ATTRIBUTE {
    const uint32_t d = (a > b) ? (a - b) : (b - a);
    return (d < kCpr - d) ? d : (kCpr - d);
  }

  int count_ = 0;
  uint32_t last_value_ = 0;
};

}
