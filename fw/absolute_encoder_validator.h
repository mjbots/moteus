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

// Gates the "active" state of an absolute encoder so that
// motor_position only consumes readings the encoder is actually able
// to produce.
//
// At power-on, absolute encoders typically need a set-up window
// (e.g. the AksIM-2 datasheet MBD01 quotes 100 ms typical / 200 ms
// worst case) before the position pipeline has converged on its
// first valid reading.  In practice, several variants begin
// answering the serial protocol earlier than that, returning
// well-formed frames carrying value=0 with their error/warning bits
// not always set -- frames that are indistinguishable at the
// protocol layer from a real shaft-at-zero reading.  Latching
// motor_position's boot output on one of those would lead to an
// incorrect home, off by one revolution once the true reading
// arrives.
//
// The simplest gate that handles this reliably without depending on
// any device-specific status flag is: accept the first valid
// non-zero reading immediately, and accept a sustained stream of
// zeros only after a wait window comfortably above the worst-case
// set-up time has elapsed.
class AbsoluteEncoderValidator {
 public:
  // After the first valid (no-error) reading, accept a sustained
  // stream of zeros as legitimate (i.e. shaft truly at the encoder's
  // raw zero position) once this much time has passed.  Sized
  // comfortably above the AksIM-2 datasheet's 200 ms worst-case
  // set-up time.
  static constexpr uint32_t kZeroTimeoutMs = 250;

  // Update the active state given the most recent reading.
  //   was_active     -- the prior active state
  //   reading_valid  -- true when the encoder reports no error
  //   value          -- parsed position
  //   now_ms         -- monotonic millisecond timestamp (uint32 wrap-safe;
  //                     MillisecondTimer::ms_since_boot is suitable)
  bool Update(bool was_active,
              bool reading_valid,
              uint32_t value,
              uint32_t now_ms) MOTEUS_CCM_ATTRIBUTE {
    if (was_active) {
      // Stay active until the encoder reports an error.  Any error
      // forces requalification from scratch.
      if (!reading_valid) {
        first_valid_seen_ = false;
        return false;
      }
      return true;
    }

    if (!reading_valid) {
      first_valid_seen_ = false;
      return false;
    }

    if (value != 0) {
      // Non-zero implies the encoder's position pipeline has produced
      // a real reading -- it would not synthesise a non-zero default.
      return true;
    }

    // Value is zero, which is ambiguous: it could be the encoder's
    // pre-acquisition default, or a legitimate shaft-at-zero reading.
    // Anchor on the first valid reading and wait kZeroTimeoutMs.
    if (!first_valid_seen_) {
      first_valid_seen_ = true;
      first_valid_ms_ = now_ms;
      return false;
    }
    return (now_ms - first_valid_ms_) >= kZeroTimeoutMs;
  }

 private:
  bool first_valid_seen_ = false;
  uint32_t first_valid_ms_ = 0;
};

}
